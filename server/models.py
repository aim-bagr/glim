from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any

class DatasetInfo(BaseModel):
    name: str
    path: str
    size_bytes: int
    size_human: str
    modified_at: str

class JobStartRequest(BaseModel):
    dataset_path: str = Field(..., description="Path to input MCAP/aimbag file")
    run_name: Optional[str] = Field(None, description="Custom name for output folder in /data/glim_results")
    max_num_keyframes: int = Field(25, description="Max keyframes per submap")
    submap_target_num_points: int = Field(15000, description="Target downsampled points per submap")
    min_travel_dist: float = Field(20.0, description="Min travel distance between loop candidate submaps (meters)")
    start_offset_sec: float = Field(0.0, description="Start playback offset in seconds")
    max_duration_sec: Optional[float] = Field(None, description="Maximum playback duration in seconds (optional)")

class JobProgress(BaseModel):
    state: str = "idle"  # idle, running, finalizing, completed, stopped, failed
    run_name: Optional[str] = None
    dataset_path: Optional[str] = None
    output_dir: Optional[str] = None
    started_at: Optional[str] = None
    elapsed_sec: float = 0.0
    bag_time: float = 0.0
    scans: int = 0
    imu: int = 0
    speed: float = 0.0
    odom_queue: int = 0
    sub_queue: int = 0
    glob_queue: int = 0
    return_code: Optional[int] = None
    error_message: Optional[str] = None

class RunSummary(BaseModel):
    name: str
    path: str
    modified_at: str
    submaps_count: int
    has_trajectory_tum: bool
    has_traj_lidar: bool
    size_bytes: int
    size_human: str
