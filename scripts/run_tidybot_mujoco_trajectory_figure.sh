#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="/home/chenwh/ros2_ws/src/TetraPGA-preview"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="${OUT_ROOT:-${ROOT_DIR}/reviewer_revision_experiments/09_tidybot_trajectory_visualization}"
OUT_DIR="${OUT_DIR:-${OUT_ROOT}/tidybot_mujoco_mpc_${TIMESTAMP}}"

BACKEND="${BACKEND:-tetrapga}"
DURATION_S="${DURATION_S:-20.0}"
TIMEOUT_PAD_S="${TIMEOUT_PAD_S:-40.0}"
DT="${DT:-0.02}"
HORIZON="${HORIZON:-20}"
CONTROL_RATE_HZ="${CONTROL_RATE_HZ:-50.0}"
SOLVE_BUDGET_MS="${SOLVE_BUDGET_MS:-10.0}"
ENFORCE_SOLVE_BUDGET="${ENFORCE_SOLVE_BUDGET:-true}"
ENABLE_VIEWER="${ENABLE_VIEWER:-false}"
STATE_COLUMN="${STATE_COLUMN:-q}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog_tidybot_mpc_${TIMESTAMP}}"

mkdir -p "${OUT_DIR}" "${ROS_LOG_DIR}"
export ROS_LOG_DIR
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

# ROS setup files may read unset variables.
set +u
source /opt/ros/humble/setup.bash
source /home/chenwh/ros2_ws/install/setup.bash
set -u

timeout_s="$(python3 - <<PY
duration = float("${DURATION_S}")
pad = float("${TIMEOUT_PAD_S}")
print(int(duration + pad))
PY
)"

prefix="${OUT_DIR}/tidybot_${BACKEND}"
summary_csv="${prefix}_summary.csv"
cycles_csv="${prefix}_cycles.csv"
rm -f "${summary_csv}" "${cycles_csv}"

{
  echo "backend: ${BACKEND}"
  echo "duration_s: ${DURATION_S}"
  echo "dt: ${DT}"
  echo "horizon: ${HORIZON}"
  echo "control_rate_hz: ${CONTROL_RATE_HZ}"
  echo "solve_budget_ms: ${SOLVE_BUDGET_MS}"
  echo "enforce_solve_budget: ${ENFORCE_SOLVE_BUDGET}"
  echo "enable_viewer: ${ENABLE_VIEWER}"
  echo "state_column: ${STATE_COLUMN}"
  echo "output_prefix: ${prefix}"
  echo "ros_log_dir: ${ROS_LOG_DIR}"
  echo "start_time: $(date --iso-8601=seconds)"
} >"${OUT_DIR}/metadata.txt"

echo "Output directory: ${OUT_DIR}"
echo "Running TidyBot MuJoCo closed-loop MPC..."
echo "Timeout: ${timeout_s}s"

launch_log="${OUT_DIR}/ros2_launch.log"
if timeout --signal=INT "${timeout_s}s" \
  ros2 launch ga_ocp_ros2 ga_ocp_mujoco_closed_loop_tidybot.launch.py \
    backend:="${BACKEND}" \
    duration_s:="${DURATION_S}" \
    dt:="${DT}" \
    horizon:="${HORIZON}" \
    control_rate_hz:="${CONTROL_RATE_HZ}" \
    solve_budget_ms:="${SOLVE_BUDGET_MS}" \
    enforce_solve_budget:="${ENFORCE_SOLVE_BUDGET}" \
    enable_viewer:="${ENABLE_VIEWER}" \
    output_prefix:="${prefix}" \
    >"${launch_log}" 2>&1; then
  echo "ROS2 launch finished normally."
else
  rc=$?
  echo "ROS2 launch exited with code ${rc}; checking whether CSV files were written."
fi

if [[ ! -f "${cycles_csv}" ]]; then
  echo "Missing cycles CSV: ${cycles_csv}" >&2
  echo "See launch log: ${launch_log}" >&2
  exit 3
fi

figure_dir="${OUT_DIR}/figure_${BACKEND}"
echo "Generating trajectory figure from ${cycles_csv}..."
"${ROOT_DIR}/scripts/visualize_tidybot_trajectory.py" \
  --cycles-csv "${cycles_csv}" \
  --output-dir "${figure_dir}" \
  --state-column "${STATE_COLUMN}"

echo
echo "Done."
echo "Summary CSV: ${summary_csv}"
echo "Cycles CSV: ${cycles_csv}"
echo "Figure directory: ${figure_dir}"
