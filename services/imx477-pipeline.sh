#!/usr/bin/env bash

LOG_DIR="/var/log/imx477_pipeline"
LOG_FILE="${LOG_DIR}/pipeline.log"

mkdir -p "${LOG_DIR}"

{
  echo "===== $(date) IMX477 pipeline starting ====="

  echo "[INFO] Sourcing TROS Humble environment (/opt/tros/humble/setup.bash)..."
  if ! source /opt/tros/humble/setup.bash; then
    echo "[ERROR] Failed to source /opt/tros/humble/setup.bash"
    exit 1
  fi

  echo "[INFO] Sourcing dual_cam_ws overlay (/root/dual_cam_ws/install/setup.bash)..."
  if ! source /root/dual_cam_ws/install/setup.bash; then
    echo "[ERROR] Failed to source /root/dual_cam_ws/install/setup.bash"
    exit 1
  fi

  unset RMW_IMPLEMENTATION
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

  echo "[DEBUG] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<not set>}"
  echo "[DEBUG] LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-<not set>}"
  echo "[DEBUG] PATH=${PATH}"

  echo "[INFO] Launching 4K camera node (imx477_dual_camera / dual_camera.launch.py)..."
  ros2 launch imx477_dual_camera dual_camera.launch.py &
  CAM_PID=$!
  echo "[INFO] Camera node PID: ${CAM_PID}"

  sleep 5

  echo "[INFO] Launching VPS resizer node (imx477_vps_resizer / vps_resizer_node)..."
  ros2 run imx477_vps_resizer vps_resizer_node &
  VPS_PID=$!
  echo "[INFO] VPS resizer PID: ${VPS_PID}"

  echo "[INFO] Waiting for child processes to exit..."
  wait "${CAM_PID}"
  CAM_STATUS=$?
  wait "${VPS_PID}"
  VPS_STATUS=$?

  echo "[INFO] Camera node exit code: ${CAM_STATUS}"
  echo "[INFO] VPS resizer node exit code: ${VPS_STATUS}"

  if [ "${CAM_STATUS}" -ne 0 ] || [ "${VPS_STATUS}" -ne 0 ]; then
    echo "[ERROR] One of the nodes exited with non-zero status. Failing pipeline."
    exit 1
  fi

  echo "[INFO] IMX477 pipeline finished successfully."

} >> "${LOG_FILE}" 2>&1