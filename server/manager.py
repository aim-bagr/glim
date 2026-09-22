import asyncio
import os
import re
import signal
import time
import shutil
import json
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Set
from server.models import JobStartRequest, JobProgress, DatasetInfo, RunSummary

PROGRESS_REGEX = re.compile(
    r"\[progress\] bag_time: (?P<bag_time>[\d\.]+)s \| scans: (?P<scans>\d+) \| imu: (?P<imu>\d+) \| speed: (?P<speed>[\d\.]+)x \| queue: odom=(?P<odom>\d+) sub=(?P<sub>\d+) glob=(?P<glob>\d+)"
)

def format_bytes(size: int) -> str:
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size < 1024.0:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{size:.1f} PB"

class ProcessManager:
    def __init__(self, data_root: str = "/data", config_base: str = "/opt/glim/config"):
        # Auto-detect data_root if /data doesn't exist (e.g. running on host)
        if not os.path.exists(data_root) and os.path.exists("/home/vishal/data"):
            data_root = "/home/vishal/data"
        if not os.path.exists(config_base) and os.path.exists("/home/vishal/Code/glim/config"):
            config_base = "/home/vishal/Code/glim/config"

        self.data_root = Path(data_root)
        self.results_dir = self.data_root / "glim_results"
        self.config_base = Path(config_base)
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.active_process: Optional[asyncio.subprocess.Process] = None
        self.job: JobProgress = JobProgress(state="idle")
        self.log_history: List[str] = []
        self.max_log_lines = 1000
        self.subscribers: Set[asyncio.Queue] = set()
        self.start_wall_time: float = 0.0

    def subscribe(self) -> asyncio.Queue:
        q = asyncio.Queue()
        self.subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue):
        self.subscribers.discard(q)

    async def broadcast(self, message: dict):
        dead_queues = []
        for q in self.subscribers:
            try:
                q.put_nowait(message)
            except Exception:
                dead_queues.append(q)
        for q in dead_queues:
            self.subscribers.discard(q)

    def list_datasets(self) -> List[DatasetInfo]:
        datasets = []
        if not self.data_root.exists():
            return datasets

        patterns = ["*.mcap", "*.aimbag", "*/*.mcap"]
        found_files = []
        for pattern in patterns:
            found_files.extend(self.data_root.glob(pattern))

        # Filter out files inside glim_results
        for path in sorted(set(found_files)):
            if "glim_results" in str(path):
                continue
            try:
                st = path.stat()
                datasets.append(DatasetInfo(
                    name=path.name,
                    path=str(path),
                    size_bytes=st.st_size,
                    size_human=format_bytes(st.st_size),
                    modified_at=datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                ))
            except Exception:
                continue
        return datasets

    def list_runs(self) -> List[RunSummary]:
        runs = []
        if not self.results_dir.exists():
            return runs

        for entry in sorted(self.results_dir.iterdir(), reverse=True):
            if not entry.is_dir():
                continue
            submaps = [d for d in entry.iterdir() if d.is_dir() and d.name.isdigit()]
            traj_tum = (entry / "trajectory_tum.txt").exists()
            traj_lidar = (entry / "traj_lidar.txt").exists()

            total_size = sum(f.stat().st_size for f in entry.rglob("*") if f.is_file())
            st = entry.stat()
            runs.append(RunSummary(
                name=entry.name,
                path=str(entry),
                modified_at=datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
                submaps_count=len(submaps),
                has_trajectory_tum=traj_tum,
                has_traj_lidar=traj_lidar,
                size_bytes=total_size,
                size_human=format_bytes(total_size)
            ))
        return runs

    def prepare_config_dir(self, req: JobStartRequest, run_name: str) -> str:
        temp_dir = Path(f"/tmp/glim_cfg_{run_name}")
        if temp_dir.exists():
            shutil.rmtree(temp_dir)
        shutil.copytree(self.config_base, temp_dir)

        # Patch sub_mapping_gpu.json & sub_mapping_cpu.json
        for sm_cfg_name in ["config_sub_mapping_gpu.json", "config_sub_mapping_cpu.json"]:
            sm_path = temp_dir / sm_cfg_name
            if sm_path.exists():
                try:
                    with open(sm_path, "r") as f:
                        lines = f.readlines()
                    new_lines = []
                    for line in lines:
                        if '"max_num_keyframes"' in line:
                            line = re.sub(r'("max_num_keyframes"\s*:\s*)[^,\n\r]+', rf'\g<1>{req.max_num_keyframes}', line)
                        elif '"submap_target_num_points"' in line:
                            line = re.sub(r'("submap_target_num_points"\s*:\s*)[^,\n\r]+', rf'\g<1>{req.submap_target_num_points}', line)
                        new_lines.append(line)
                    with open(sm_path, "w") as f:
                        f.writelines(new_lines)
                except Exception as e:
                    print(f"Error patching {sm_path}: {e}")

        # Patch config_global_mapping_pose_graph.json
        pg_path = temp_dir / "config_global_mapping_pose_graph.json"
        if pg_path.exists():
            try:
                with open(pg_path, "r") as f:
                    lines = f.readlines()
                new_lines = []
                for line in lines:
                    if '"min_travel_dist"' in line:
                        line = re.sub(r'("min_travel_dist"\s*:\s*)[^,\n\r]+', rf'\g<1>{req.min_travel_dist}', line)
                    new_lines.append(line)
                with open(pg_path, "w") as f:
                    f.writelines(new_lines)
            except Exception as e:
                print(f"Error patching {pg_path}: {e}")

        return str(temp_dir)

    async def start_job(self, req: JobStartRequest) -> JobProgress:
        if self.active_process and self.active_process.returncode is None:
            raise RuntimeError("A SLAM job is already running.")

        run_name = req.run_name.strip() if req.run_name else f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        output_dir = self.results_dir / run_name
        output_dir.mkdir(parents=True, exist_ok=True)

        config_dir = self.prepare_config_dir(req, run_name)

        cmd = [
            "glim_mcap",
            "--headless",
            "-c", config_dir,
            "-i", req.dataset_path,
            "-o", str(output_dir),
            "-s", str(req.start_offset_sec)
        ]
        if req.max_duration_sec and req.max_duration_sec > 0:
            cmd.extend(["-d", str(req.max_duration_sec)])

        self.start_wall_time = time.time()
        self.log_history = []
        self.job = JobProgress(
            state="running",
            run_name=run_name,
            dataset_path=req.dataset_path,
            output_dir=str(output_dir),
            started_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            elapsed_sec=0.0
        )

        try:
            self.active_process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT
            )
        except Exception as e:
            self.job.state = "failed"
            self.job.error_message = str(e)
            await self.broadcast({"type": "status", "data": self.job.dict()})
            raise

        asyncio.create_task(self._monitor_stream(self.active_process))
        await self.broadcast({"type": "status", "data": self.job.dict()})
        return self.job

    async def _monitor_stream(self, proc: asyncio.subprocess.Process):
        while True:
            line_bytes = await proc.stdout.readline()
            if not line_bytes:
                break
            line = line_bytes.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue

            self.log_history.append(line)
            if len(self.log_history) > self.max_log_lines:
                self.log_history.pop(0)

            # Match progress stats
            match = PROGRESS_REGEX.search(line)
            if match:
                self.job.bag_time = float(match.group("bag_time"))
                self.job.scans = int(match.group("scans"))
                self.job.imu = int(match.group("imu"))
                self.job.speed = float(match.group("speed"))
                self.job.odom_queue = int(match.group("odom"))
                self.job.sub_queue = int(match.group("sub"))
                self.job.glob_queue = int(match.group("glob"))
                self.job.elapsed_sec = round(time.time() - self.start_wall_time, 1)

            await self.broadcast({
                "type": "log",
                "line": line,
                "progress": self.job.dict()
            })

        ret_code = await proc.wait()
        self.job.return_code = ret_code
        self.job.elapsed_sec = round(time.time() - self.start_wall_time, 1)

        if self.job.state == "finalizing":
            self.job.state = "stopped"
        elif ret_code == 0:
            self.job.state = "completed"
        else:
            self.job.state = "failed"
            self.job.error_message = f"Process exited with code {ret_code}"

        await self.broadcast({
            "type": "status",
            "data": self.job.dict()
        })

    async def stop_job(self) -> JobProgress:
        if not self.active_process or self.active_process.returncode is not None:
            return self.job

        self.job.state = "finalizing"
        await self.broadcast({"type": "status", "data": self.job.dict()})
        await self.broadcast({"type": "log", "line": "[service] Stop requested by user. Sending SIGINT for clean finalization..."})

        # Send SIGINT to allow glim_mcap to catch signal and save map cleanly
        try:
            self.active_process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass

        # Wait up to 30 seconds for graceful finalization
        for _ in range(30):
            if self.active_process.returncode is not None:
                break
            await asyncio.sleep(1)

        # Force kill if still hung
        if self.active_process.returncode is None:
            await self.broadcast({"type": "log", "line": "[service] Process did not exit in 30s. Sending SIGKILL."})
            try:
                self.active_process.kill()
            except ProcessLookupError:
                pass

        return self.job
