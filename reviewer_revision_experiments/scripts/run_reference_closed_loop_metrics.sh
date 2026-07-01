#!/usr/bin/env bash
set -u

ROOT_DIR="/home/chenwh/ros2_ws/src/TetraPGA-preview"
EXP_DIR="${ROOT_DIR}/reviewer_revision_experiments/04_closed_loop_mpc_metrics"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

BACKENDS_STR="${BACKENDS_STR:-tetrapga pinocchio casadi}"
ROBOTS_STR="${ROBOTS_STR:-ur leap tidybot}"
DURATION_S="${DURATION_S:-20}"
TIMEOUT_PAD_S="${TIMEOUT_PAD_S:-240}"
ENFORCE_SOLVE_BUDGET="${ENFORCE_SOLVE_BUDGET:-true}"
ENABLE_VIEWER="${ENABLE_VIEWER:-false}"
ACCELERATION_WEIGHT="${ACCELERATION_WEIGHT:-1e-6}"
LOG_ROOT="${LOG_ROOT:-${EXP_DIR}/reference_batch_${TIMESTAMP}}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog_ga_ocp_reference_${TIMESTAMP}}"

mkdir -p "${LOG_ROOT}" "${ROS_LOG_DIR}"
export ROS_LOG_DIR
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

set +u
source /opt/ros/humble/setup.bash
source /home/chenwh/ros2_ws/install/setup.bash
set -u

IFS=' ' read -r -a BACKENDS <<< "${BACKENDS_STR}"
IFS=' ' read -r -a ROBOTS <<< "${ROBOTS_STR}"
FAILED_CASES=()

timeout_seconds() {
  python3 - <<PY
duration = float("${1}")
pad = float("${TIMEOUT_PAD_S}")
print(int(duration + pad))
PY
}

robot_launch() {
  case "$1" in
    ur) echo "ga_ocp_mujoco_closed_loop_ur.launch.py" ;;
    leap) echo "ga_ocp_mujoco_closed_loop_leap.launch.py" ;;
    tidybot) echo "ga_ocp_mujoco_closed_loop_tidybot.launch.py" ;;
    *) return 1 ;;
  esac
}

robot_dt() {
  case "$1" in
    ur) echo "0.008" ;;
    leap|tidybot) echo "0.02" ;;
    *) return 1 ;;
  esac
}

robot_horizon() {
  case "$1" in
    ur) echo "40" ;;
    leap|tidybot) echo "20" ;;
    *) return 1 ;;
  esac
}

robot_control_rate() {
  case "$1" in
    ur) echo "125.0" ;;
    leap|tidybot) echo "50.0" ;;
    *) return 1 ;;
  esac
}

robot_budget_ms() {
  case "$1" in
    ur) echo "4.0" ;;
    leap) echo "8.0" ;;
    tidybot) echo "10.0" ;;
    *) return 1 ;;
  esac
}

run_case() {
  local robot="$1"
  local backend="$2"
  local launch_file
  local dt
  local horizon
  local control_rate
  local budget
  launch_file="$(robot_launch "${robot}")" || return 1
  dt="$(robot_dt "${robot}")" || return 1
  horizon="$(robot_horizon "${robot}")" || return 1
  control_rate="$(robot_control_rate "${robot}")" || return 1
  budget="$(robot_budget_ms "${robot}")" || return 1

  local name="reference_${robot}_${backend}"
  local summary_csv="${LOG_ROOT}/${name}_summary.csv"
  local cycles_csv="${LOG_ROOT}/${name}_cycles.csv"
  local timeout_s
  timeout_s="$(timeout_seconds "${DURATION_S}")"
  rm -f "${summary_csv}" "${cycles_csv}"

  echo
  echo "============================================================"
  echo "Running reference closed-loop case: ${name}"
  echo "Launch: ${launch_file}"
  echo "dt=${dt}, horizon=${horizon}, control_rate=${control_rate}, budget=${budget}, enforce=${ENFORCE_SOLVE_BUDGET}, accel_weight=${ACCELERATION_WEIGHT}"
  echo "Output prefix: ${LOG_ROOT}/${name}"
  echo "Timeout: ${timeout_s}s"
  echo "============================================================"

  if timeout --signal=INT "${timeout_s}s" \
    ros2 launch ga_ocp_ros2 "${launch_file}" \
      backend:="${backend}" \
      duration_s:="${DURATION_S}" \
      dt:="${dt}" \
      horizon:="${horizon}" \
      control_rate_hz:="${control_rate}" \
      acceleration_weight:="${ACCELERATION_WEIGHT}" \
      solve_budget_ms:="${budget}" \
      enforce_solve_budget:="${ENFORCE_SOLVE_BUDGET}" \
      enable_viewer:="${ENABLE_VIEWER}" \
      output_prefix:="${LOG_ROOT}/${name}"; then
    if [[ -f "${summary_csv}" && -f "${cycles_csv}" ]]; then
      echo "[OK] ${name}"
    else
      echo "[FAIL] ${name} (missing summary or cycles CSV)"
      FAILED_CASES+=("${name}")
    fi
  else
    local rc=$?
    if [[ -f "${summary_csv}" && -f "${cycles_csv}" ]]; then
      echo "[OK] ${name} (launch exited with ${rc} after writing CSVs)"
    else
      echo "[FAIL] ${name} (exit=${rc})"
      FAILED_CASES+=("${name}")
    fi
  fi
}

write_summaries() {
  python3 "${ROOT_DIR}/reviewer_revision_experiments/scripts/summarize_reference_closed_loop.py" \
    --batch-root "${LOG_ROOT}" \
    --output-dir "${EXP_DIR}"
}

echo "Experiment root: ${EXP_DIR}"
echo "Log root: ${LOG_ROOT}"
echo "ROS log dir: ${ROS_LOG_DIR}"
echo "Robots: ${ROBOTS_STR}"
echo "Backends: ${BACKENDS_STR}"

for robot in "${ROBOTS[@]}"; do
  for backend in "${BACKENDS[@]}"; do
    run_case "${robot}" "${backend}"
  done
done

write_summaries || FAILED_CASES+=("__summary_generation__")

echo
echo "============================================================"
echo "Reference closed-loop metrics complete"
echo "Results: ${EXP_DIR}"
if [[ ${#FAILED_CASES[@]} -eq 0 ]]; then
  echo "All cases succeeded."
else
  echo "Failed cases:"
  for failed in "${FAILED_CASES[@]}"; do
    echo "  - ${failed}"
  done
  exit 1
fi
echo "============================================================"
