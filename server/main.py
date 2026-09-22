import os
import re
import struct
import asyncio
from pathlib import Path
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from server.models import JobStartRequest, JobProgress
from server.manager import ProcessManager

app = FastAPI(title="GLIM Headless SLAM Service", version="1.0.0")

# Enable CORS for local dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

manager = ProcessManager()

# Static web directory
web_dir = Path("/opt/glim/web")
if not web_dir.exists() and Path("./web").exists():
    web_dir = Path("./web").resolve()

@app.get("/api/status")
async def get_status():
    return manager.job.dict()

@app.get("/api/datasets")
async def get_datasets():
    return [d.dict() for d in manager.list_datasets()]

@app.get("/api/runs")
async def get_runs():
    return [r.dict() for r in manager.list_runs()]

@app.post("/api/jobs/start")
async def start_job(req: JobStartRequest):
    try:
        job = await manager.start_job(req)
        return job.dict()
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/jobs/stop")
async def stop_job():
    job = await manager.stop_job()
    return job.dict()

@app.get("/api/jobs/logs")
async def get_logs():
    return {"lines": manager.log_history}

@app.get("/api/runs/{run_name}/trajectory")
async def get_run_trajectory(run_name: str, opt: bool = True):
    run_dir = manager.results_dir / run_name
    if not run_dir.exists():
        raise HTTPException(status_code=404, detail="Run directory not found")

    target = "traj_lidar.txt" if opt else "trajectory_tum.txt"
    file_path = run_dir / target
    if not file_path.exists():
        file_path = run_dir / "trajectory_tum.txt"

    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Trajectory file not found")

    return FileResponse(path=str(file_path), filename=f"{run_name}_{file_path.name}", media_type="text/plain")

@app.get("/api/runs/{run_name}/files/{filename}")
async def get_run_file(run_name: str, filename: str):
    file_path = manager.results_dir / run_name / filename
    if not file_path.exists() or not file_path.is_file():
        raise HTTPException(status_code=404, detail="File not found")
    return FileResponse(path=str(file_path), filename=filename)

# In-memory cache for parsed submap metadata
_submap_metadata_cache = {}

@app.get("/api/runs/{run_name}/submaps")
async def get_run_submaps(run_name: str):
    if run_name in _submap_metadata_cache:
        return _submap_metadata_cache[run_name]

    run_dir = manager.results_dir / run_name
    if not run_dir.exists():
        raise HTTPException(status_code=404, detail="Run not found")

    submaps = []
    submap_dirs = sorted([d for d in run_dir.iterdir() if d.is_dir() and d.name.isdigit()], key=lambda d: int(d.name))
    for d in submap_dirs:
        submap_id = int(d.name)
        data_file = d / "data.txt"
        pts_file = d / "points_compact.bin"
        pts_bytes = pts_file.stat().st_size if pts_file.exists() else 0
        if not data_file.exists():
            continue

        text = data_file.read_text(errors="ignore")
        m = re.search(r"T_world_origin:\s*\n([^\n]+\n[^\n]+\n[^\n]+\n[^\n]+)", text)
        if not m:
            continue
        rows = [[float(v) for v in line.split()] for line in m.group(1).strip().split("\n")]
        # Column-major for Three.js
        col_major = [
            rows[0][0], rows[1][0], rows[2][0], rows[3][0],
            rows[0][1], rows[1][1], rows[2][1], rows[3][1],
            rows[0][2], rows[1][2], rows[2][2], rows[3][2],
            rows[0][3], rows[1][3], rows[2][3], rows[3][3]
        ]
        submaps.append({
            "id": submap_id,
            "num_points": pts_bytes // 12,
            "pos": [rows[0][3], rows[1][3], rows[2][3]],
            "matrix": col_major
        })

    _submap_metadata_cache[run_name] = submaps
    return submaps

@app.get("/api/runs/{run_name}/submaps/{submap_id}/points")
async def get_submap_points(run_name: str, submap_id: int):
    pts_file = manager.results_dir / run_name / f"{submap_id:06d}" / "points_compact.bin"
    if not pts_file.exists():
        raise HTTPException(status_code=404, detail="Submap points not found")
    return FileResponse(path=str(pts_file), media_type="application/octet-stream")

@app.get("/api/runs/{run_name}/submaps/{submap_id}/intensities")
async def get_submap_intensities(run_name: str, submap_id: int):
    int_file = manager.results_dir / run_name / f"{submap_id:06d}" / "intensities_compact.bin"
    if not int_file.exists():
        raise HTTPException(status_code=404, detail="Submap intensities not found")
    return FileResponse(path=str(int_file), media_type="application/octet-stream")

@app.get("/api/runs/{run_name}/trajectory_binary")
async def get_trajectory_binary(run_name: str, opt: bool = True):
    import struct
    run_dir = manager.results_dir / run_name
    target = "traj_lidar.txt" if opt else "trajectory_tum.txt"
    file_path = run_dir / target
    if not file_path.exists():
        file_path = run_dir / "trajectory_tum.txt"
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Trajectory not found")

    cache_path = run_dir / f"{file_path.stem}.bin"
    if not cache_path.exists() or cache_path.stat().st_mtime < file_path.stat().st_mtime:
        coords = []
        with open(file_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 4:
                    coords.extend([float(parts[1]), float(parts[2]), float(parts[3])])
        bin_data = struct.pack(f"{len(coords)}f", *coords)
        with open(cache_path, "wb") as f:
            f.write(bin_data)

    return FileResponse(path=str(cache_path), media_type="application/octet-stream")

@app.websocket("/api/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    queue = manager.subscribe()

    # Send current state & initial backlog of logs
    try:
        await websocket.send_json({"type": "status", "data": manager.job.dict()})
        for line in manager.log_history[-100:]:
            await websocket.send_json({"type": "log", "line": line, "progress": manager.job.dict()})

        while True:
            msg = await queue.get()
            await websocket.send_json(msg)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        manager.unsubscribe(queue)

# Serve web frontend
if web_dir.exists():
    app.mount("/", StaticFiles(directory=str(web_dir), html=True), name="web")
