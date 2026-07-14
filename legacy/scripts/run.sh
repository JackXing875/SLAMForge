#!/bin/bash
# ---------------------------------------------------------------------------
# DeepVO launch script.
# ---------------------------------------------------------------------------

POSE_DIR="./data/poses"
POSE_FILE="${POSE_DIR}/trajectory.csv"
CPP_EXECUTABLE="./build/deepvo_app"
PYTHON_VIEWER="./scripts/viewer_3d.py"
PYTHON_EXECUTABLE="./venv/bin/python"

if [ $# -ne 1 ] && [ $# -ne 5 ]; then
    echo "Usage: ./scripts/run.sh <input_video_path> [fx fy cx cy]"
    exit 1
fi

VIDEO_PATH="$1"
EXTRA_ARGS=()
if [ $# -eq 5 ]; then
    EXTRA_ARGS=("$2" "$3" "$4" "$5")
fi

mkdir -p "${POSE_DIR}"

if [ -f "${POSE_FILE}" ]; then
    rm "${POSE_FILE}"
fi
touch "${POSE_FILE}"

echo "  Starting DeepVO Launch Sequence      "

cleanup() {
    echo -e "\n[Launch Script] Shutting down DeepVO system..."
    kill "${PYTHON_PID}" 2>/dev/null
    exit 0
}

trap cleanup SIGINT EXIT

echo "[Launch Script] Starting Python 3D Visualizer in background..."
"${PYTHON_EXECUTABLE}" "${PYTHON_VIEWER}" "${POSE_FILE}" &
PYTHON_PID=$!

sleep 1

echo "[Launch Script] Igniting C++ VO Engine..."
"${CPP_EXECUTABLE}" "${VIDEO_PATH}" "${POSE_FILE}" "${EXTRA_ARGS[@]}"

wait "${PYTHON_PID}"
