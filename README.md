![GLIM](docs/assets/logo2.png "GLIM Logo")

# GLIM (AIM Standalone Fork)

This repository is an optimized standalone fork of [koide3/glim](https://github.com/koide3/glim) developed by the **Autonomous Infrastructure Mapping (AIM)** team.

> [!NOTE]
> **Looking for the core algorithm, theory, or upstream ROS packages?**
> Please refer to the upstream repository at **[koide3/glim](https://github.com/koide3/glim)** and the official **[GLIM Documentation](https://koide3.github.io/glim/)**.

---

## Container Releases (GHCR)

Pre-built CUDA-accelerated Docker containers are published automatically to GitHub Container Registry (**GHCR**):

```bash
# Pull the latest stable container from main
docker pull ghcr.io/aim-bagr/glim:latest

# Or pull an immutable build tagged by git commit SHA (7 chars)
docker pull ghcr.io/aim-bagr/glim:<git-sha7>
```

Every container image carries standard OCI revision metadata (`org.opencontainers.image.revision=<full-sha>`) and evaluation labels (`slam-eval.tool=glim`).

---

## Fork Features & Improvements

This fork transforms GLIM into a production-grade, standalone LiDAR/LiDAR-inertial SLAM engine with zero host pollution:

### 1. Standalone MCAP Runner (`glim_mcap`)
- **Zero ROS Dependencies**: Ingests MCAP datasets directly without requiring ROS, ROS 2, or running roscore/rosbag bridges.
- **High-Throughput Streaming**: Integrates [`thirdparty/aimcap`](https://github.com/aim-bagr/aimcap) with chunk-by-chunk deserialization and column timestamp deskewing.
- **Queue Backpressure Throttling**: Automatically throttles ingestion when odometry or submapping queues back up, preventing memory exhaustion on massive bags.
- **Standardized TUM Output**: Emits trajectory poses directly in standard TUM format (`timestamp x y z qx qy qz qw`) alongside factor graph binaries.

### 2. IMU-Less Continuous-Time SLAM
- **Automatic Fallback (`--no-imu`)**: Automatically falls back to Continuous-Time ICP (CT-ICP) odometry estimation when IMU data is unavailable.
- **Dedicated Configurations**: Tuned `config_no_imu.json` and `config_no_imu_cpu.json` for stable registration without inertial priors.
- **Legacy Point Cloud Support**: Decodes legacy packed 16-byte `int32` point cloud payloads with coordinate scaling via `--scale <float>`.
- **Intra-Scan Timestamp Synthesis**: Synthesizes intra-scan relative point timestamps when per-point offsets are missing.
- **Front-End Odometry Mode (`--odom-only`)**: Quickly verify odometry trajectories while bypassing submapping and global graph optimization.

### 3. Deterministic Preprocessing
- **Reproducible Random Grid Sampling**: Replaces thread-schedule-dependent downsampling with a deterministic voxel-keyed sampling algorithm (`splitmix64` PRNG seeded by frame seed + voxel coordinates). Repeated runs on the same dataset yield 100% bitwise-identical point clouds regardless of CPU thread count or OpenMP scheduling.

### 4. Headless Control Service & 3D Web Dashboard
- **Web UI & REST/WebSocket Service (`server/` & `web/`)**: Complete browser-based mission control for headless remote SLAM execution.
- **Interactive Three.js 3D Viewer**: Fullscreen point cloud visualizer with live submap streaming, trajectory lines, and loop closure edges.
- **Visibility Layers & Hotkeys**:
  - `M`: Toggle point cloud map visibility
  - `O`: Toggle trajectory line
  - `L`: Toggle loop closure factor lines
  - `Space` / `P`: Play / pause automated submap sequencing with variable speed controls (0.5x – 4x)
- **Deep-Linking via URL Parameters**: Auto-load runs and initialize layer visibility via URL params (e.g. `?run=my_run&map=1&traj=1&loops=1`).

### 5. Interactive Offline 3D Viewer (`glim_offline_viewer`)
- **Submap Isolation**: Inspect single submaps or contiguous submap clusters (`DragIntRange2`) with camera auto-centering and metadata display.
- **Context Menus**: Right-click any submap sphere or point to isolate or focus the camera.

---

## Quickstart

The easiest way to run this fork is using the provided [`run_glim.sh`](run_glim.sh) launcher script. It automatically handles GPU detection (`--gpus all`), X11 display forwarding, dataset volume mounts, and container lifecycle.

### 1. Run Standalone SLAM on an MCAP File

```bash
# GPU-accelerated run (default)
./run_glim.sh -i /path/to/dataset.mcap -o ./results

# IMU-less run (Continuous-Time ICP)
./run_glim.sh -i /path/to/dataset.mcap -o ./results --no-imu

# Odometry-only mode with custom topics and coordinate scale
./run_glim.sh -i /path/to/dataset.mcap -o ./results \
  --lidar /ouster/points \
  --scale 0.01 \
  --odom-only

# Enable real-time X11 GUI visualization
./run_glim.sh -i /path/to/dataset.mcap -o ./results --gui
```

### 2. Launch the 3D Web Dashboard

```bash
./run_glim.sh --web 8080
```
Open **`http://localhost:8080`** in your browser to inspect datasets, trigger headless SLAM runs, and visualize live or past runs in 3D.

### 3. Launch the Interactive Offline 3D Viewer

```bash
./run_glim.sh --view ./results
```

---

## Running Directly with Docker / GHCR

You can also run the published container directly without cloning the repository:

```bash
docker run --rm -it --gpus all \
  --user "$(id -u):$(id -g)" \
  -v /path/to/mcap_dir:/data:ro \
  -v /path/to/output_dir:/output \
  ghcr.io/aim-bagr/glim:latest \
  glim_mcap -i /data/recording.mcap -o /output
```

---

## Container & Evaluation Conventions

This repository adheres to the container naming and labeling standard defined in [`docs/STANDALONE_SLAM_RUNBOOK.md`](docs/STANDALONE_SLAM_RUNBOOK.md):

| Mode | Container Name Format | Standard Labels |
| :--- | :--- | :--- |
| **Batch SLAM** | `glim-<dataset>-<yyyymmdd-hhmm>` | `slam-eval.tool=glim`, `slam-eval.dataset=<name>`, `slam-eval.run-id=<ts>` |
| **Offline Viewer** | `glim-view-<dataset>` | `slam-eval.tool=glim`, `slam-eval.dataset=<name>`, `slam-eval.run-id=<ts>` |
| **Web Service** | `glim-web` | `slam-eval.tool=glim`, `slam-eval.dataset=web`, `slam-eval.run-id=<ts>` |

---

## Upstream Attribution & Citation

GLIM was developed by Kenji Koide and researchers at the National Institute of Advanced Industrial Science and Technology (AIST), Japan. If you use GLIM in academic work, please cite:

```bibtex
@article{koide2024glim,
  title={GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author={Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal={Robotics and Autonomous Systems},
  year={2024},
  publisher={Elsevier},
  doi={10.1016/j.robot.2024.104750}
}
```

This package is licensed under the MIT License. See [LICENSE](LICENSE) for details.
