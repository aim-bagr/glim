#!/usr/bin/env bash
set -e

# ==============================================================================
# GLIM Standalone MCAP Runner Script (via Docker + CUDA)
# ==============================================================================

function show_help() {
  cat << 'EOF'
Usage: ./run_glim.sh [script_options] [glim_options]

Script Options:
  --web [port]          Launch headless web control service & dashboard (default port: 8080)
  --view <dir>          Launch interactive 3D viewer on a previously saved results directory
  --gui                 Enable X11 GUI forwarding for real-time 3D visualization
  --cpu                 Use CPU-only estimation and mapping (default: GPU)
  --rebuild             Rebuild the docker image before running
  -h, --help            Show this help message

GLIM Options (passed to glim_mcap):
  -i, --input <file>    Path to input MCAP file (e.g. /home/vishal/data/recording.mcap)
  -o, --output <dir>    Output directory for trajectory and factor graph
  -d, --duration <sec>  Maximum duration in seconds to process (default: full file)
  -s, --start <sec>     Start offset in seconds from beginning of recording
  -r, --rate <float>    Playback rate multiplier (1.0 = real-time, 0 = max speed)
  --lidar <topic>       LiDAR topic (default: /ouster/points)
  --imu <topic>         IMU topic (default: /ouster/imu)
  --no-imu              Run without IMU (uses continuous-time CT-ICP odometry)
  --scale <float>       Coordinate scale factor (e.g. 0.01 for cm to m)
  --cpu                 Use CPU-only odometry and mapping
  --odom-only           Run front-end odometry only (disable sub-mapping and global mapping)
  --headless            Run without visualizer GUI (default if --gui is omitted)

Examples:
  # Fast headless run on local MCAP (GPU default):
  ./run_glim.sh -i /home/vishal/data/joen-6_2026-08-28T21-45-00.171Z-decoded.mcap -o ~/data/glim_results

  # View previously completed mapping results (3D viewer):
  ./run_glim.sh --view ~/data/glim_results
EOF
  exit 0
}

WEB_MODE=0
WEB_PORT=8080
GUI_MODE=0
CPU_MODE=0
REBUILD=0
VIEW_DIR=""
GLIM_ARGS=()
INPUT_FILE=""
OUTPUT_DIR=""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GIT_SHA="$(git -C "$SCRIPT_DIR" rev-parse --short=7 HEAD 2>/dev/null || echo "latest")"
if [[ "$GIT_SHA" != "latest" ]]; then
  git -C "$SCRIPT_DIR" diff --quiet HEAD 2>/dev/null || GIT_SHA="${GIT_SHA}-dirty"
fi
IMAGE_TAG="${IMAGE_TAG:-$GIT_SHA}"
IMAGE_NAME="${IMAGE_NAME:-glim:${IMAGE_TAG}}"

function ensure_image() {
  if [[ "$REBUILD" -eq 1 ]] || ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    if [[ "$REBUILD" -ne 1 ]]; then
      if docker image inspect "ghcr.io/aim-bagr/glim:$IMAGE_TAG" &>/dev/null; then
        IMAGE_NAME="ghcr.io/aim-bagr/glim:$IMAGE_TAG"
        return
      elif docker image inspect "ghcr.io/aim-bagr/glim:latest" &>/dev/null; then
        IMAGE_NAME="ghcr.io/aim-bagr/glim:latest"
        return
      elif docker image inspect "glim:mcap" &>/dev/null; then
        IMAGE_NAME="glim:mcap"
        return
      fi
    fi
    echo "Building Docker image ($IMAGE_NAME)..."
    FULL_SHA="$(git -C "$SCRIPT_DIR" rev-parse HEAD 2>/dev/null || echo "unknown")"
    docker build --build-arg GIT_SHA="$FULL_SHA" -t "$IMAGE_NAME" "$SCRIPT_DIR"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --web)
      WEB_MODE=1
      if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
        WEB_PORT="$2"
        shift 2
      else
        shift
      fi
      ;;
    --view)
      VIEW_DIR="$2"
      GUI_MODE=1
      shift 2
      ;;
    --gui)
      GUI_MODE=1
      shift
      ;;
    --cpu)
      CPU_MODE=1
      GLIM_ARGS+=("--cpu")
      shift
      ;;
    --rebuild)
      REBUILD=1
      shift
      ;;
    -h|--help)
      show_help
      ;;
    -i|--input)
      INPUT_FILE="$2"
      shift 2
      ;;
    -o|--output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      GLIM_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "$WEB_MODE" -eq 1 ]]; then
  DATA_DIR="${DATA_DIR:-/home/vishal/data}"
  mkdir -p "${DATA_DIR}/glim_results"

  ensure_image

  echo "=========================================================="
  echo " Starting GLIM Headless SLAM Web Service"
  echo " Dashboard URL : http://localhost:${WEB_PORT:-8080}"
  echo " Data Root     : ${DATA_DIR}"
  echo " Image         : ${IMAGE_NAME}"
  echo "=========================================================="

  CONTAINER_NAME="glim-web"
  docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

  exec docker run --rm -it \
    --name "$CONTAINER_NAME" \
    --label slam-eval.tool=glim \
    --label slam-eval.dataset=web \
    --label slam-eval.run-id="$(date +%Y%m%d-%H%M)" \
    --gpus all \
    --ipc=host \
    --ulimit memlock=-1 \
    --ulimit stack=67108864 \
    -p "${WEB_PORT:-8080}:8080" \
    -e MALLOC_ARENA_MAX=2 \
    -v "${DATA_DIR}:/data" \
    -v "${SCRIPT_DIR}/config:/opt/glim/config:ro" \
    -v "${SCRIPT_DIR}/server:/opt/glim/server:ro" \
    -v "${SCRIPT_DIR}/web:/opt/glim/web:ro" \
    "$IMAGE_NAME" python3 -m uvicorn server.main:app --host 0.0.0.0 --port 8080
fi

if [[ -n "$VIEW_DIR" ]]; then
  if [[ ! -d "$VIEW_DIR" ]]; then
    echo "Error: Map directory does not exist: $VIEW_DIR"
    exit 1
  fi
  VIEW_ABS="$(realpath "$VIEW_DIR")"

  ensure_image

  TARGET_DISPLAY="$DISPLAY"
  if [[ -z "$TARGET_DISPLAY" ]]; then
    if [[ -S /tmp/.X11-unix/X1 ]]; then
      TARGET_DISPLAY=":1"
    elif [[ -S /tmp/.X11-unix/X0 ]]; then
      TARGET_DISPLAY=":0"
    else
      TARGET_DISPLAY=":0"
    fi
  fi

  AUTH_FILE="$XAUTHORITY"
  if [[ -z "$AUTH_FILE" ]]; then
    if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
      AUTH_FILE="/run/user/$(id -u)/gdm/Xauthority"
    elif [[ -f "$HOME/.Xauthority" ]]; then
      AUTH_FILE="$HOME/.Xauthority"
    fi
  fi

  DATASET="$(basename "$VIEW_ABS" | tr -c 'A-Za-z0-9_.\n-' '-')"
  CONTAINER_NAME="glim-view-${DATASET}"
  docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

  DOCKER_ARGS=(
    --rm
    --name "$CONTAINER_NAME"
    --label slam-eval.tool=glim
    --label slam-eval.dataset="${DATASET}"
    --label slam-eval.run-id="$(date +%Y%m%d-%H%M)"
    --gpus all
    --user "$(id -u):$(id -g)"
    --net=host
    --ipc=host
    -e "DISPLAY=${TARGET_DISPLAY}"
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    -v "$VIEW_ABS:/output"
    -v "$SCRIPT_DIR/config:/opt/glim/config:ro"
  )

  if [[ -n "$AUTH_FILE" && -f "$AUTH_FILE" ]]; then
    DISPLAY="$TARGET_DISPLAY" XAUTHORITY="$AUTH_FILE" xhost +local: >/dev/null 2>&1 || true
    DOCKER_ARGS+=(
      -e "XAUTHORITY=/tmp/.Xauthority"
      -v "$AUTH_FILE:/tmp/.Xauthority:ro"
    )
  fi

  echo "Launching GLIM Offline 3D Viewer..."
  echo "  Map directory : $VIEW_ABS"
  echo "  Display       : $TARGET_DISPLAY"
  echo "  Container     : $CONTAINER_NAME"
  echo "  Image         : $IMAGE_NAME"

  docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" glim_offline_viewer /output
  exit 0
fi

if [[ -z "$INPUT_FILE" ]]; then
  echo "Error: Input MCAP file is required (-i /path/to/file.mcap)"
  exit 1
fi

if [[ ! -f "$INPUT_FILE" ]]; then
  echo "Error: File does not exist: $INPUT_FILE"
  exit 1
fi

# Resolve absolute paths
INPUT_ABS="$(realpath "$INPUT_FILE")"
INPUT_DIR="$(dirname "$INPUT_ABS")"
INPUT_BASENAME="$(basename "$INPUT_ABS")"

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$(pwd)/glim_results"
fi
mkdir -p "$OUTPUT_DIR"
OUTPUT_ABS="$(realpath "$OUTPUT_DIR")"

ensure_image

DATASET="$(basename "${INPUT_FILE%.*}" | tr -c 'A-Za-z0-9_.\n-' '-')"
RUN_TS="$(date +%Y%m%d-%H%M)"
CONTAINER_NAME="glim-${DATASET}-${RUN_TS}"

DOCKER_ARGS=(
  --rm
  --name "$CONTAINER_NAME"
  --label slam-eval.tool=glim
  --label slam-eval.dataset="${DATASET}"
  --label slam-eval.run-id="${RUN_TS}"
  --gpus all
  --user "$(id -u):$(id -g)"
  -e "MALLOC_ARENA_MAX=2"
  -v "$INPUT_DIR:/data:ro"
  -v "$OUTPUT_ABS:/output"
  -v "$SCRIPT_DIR/config:/opt/glim/config:ro"
)

if [[ "$GUI_MODE" -eq 1 ]]; then
  # Auto-detect DISPLAY if not set in current environment (e.g. over SSH)
  if [[ -z "$DISPLAY" ]]; then
    if [[ -S /tmp/.X11-unix/X1 ]]; then
      TARGET_DISPLAY=":1"
    elif [[ -S /tmp/.X11-unix/X0 ]]; then
      TARGET_DISPLAY=":0"
    else
      TARGET_DISPLAY=":0"
    fi
  else
    TARGET_DISPLAY="$DISPLAY"
  fi

  # Auto-detect XAUTHORITY
  if [[ -z "$XAUTHORITY" ]]; then
    if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
      AUTH_FILE="/run/user/$(id -u)/gdm/Xauthority"
    elif [[ -f "$HOME/.Xauthority" ]]; then
      AUTH_FILE="$HOME/.Xauthority"
    fi
  else
    AUTH_FILE="$XAUTHORITY"
  fi

  if [[ -n "$AUTH_FILE" && -f "$AUTH_FILE" ]]; then
    DISPLAY="$TARGET_DISPLAY" XAUTHORITY="$AUTH_FILE" xhost +local: >/dev/null 2>&1 || true
    DOCKER_ARGS+=(
      -e "XAUTHORITY=/tmp/.Xauthority"
      -v "$AUTH_FILE:/tmp/.Xauthority:ro"
    )
  fi

  DOCKER_ARGS+=(
    --net=host
    --ipc=host
    -e "DISPLAY=${TARGET_DISPLAY}"
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
  )
else
  GLIM_ARGS+=("--headless")
fi

echo "Running GLIM MCAP via Docker..."
echo "  Input     : $INPUT_ABS"
echo "  Output    : $OUTPUT_ABS"
echo "  Mode      : $( [[ $CPU_MODE -eq 1 ]] && echo 'CPU' || echo 'GPU (CUDA)' )"
echo "  GUI       : $( [[ $GUI_MODE -eq 1 ]] && echo 'Enabled' || echo 'Headless' )"
echo "  Container : $CONTAINER_NAME"
echo "  Image     : $IMAGE_NAME"

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" \
  glim_mcap -i "/data/$INPUT_BASENAME" -o /output "${GLIM_ARGS[@]}"
