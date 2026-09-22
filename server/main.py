import os
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
