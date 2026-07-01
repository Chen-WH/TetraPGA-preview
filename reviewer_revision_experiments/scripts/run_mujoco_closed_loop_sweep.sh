#!/usr/bin/env bash
set -u

ROOT_DIR="/home/chenwh/ros2_ws/src/TetraPGA-preview"
EXP_DIR="${ROOT_DIR}/reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

BACKENDS_STR="${BACKENDS_STR:-tetrapga}"
DURATION_S="${DURATION_S:-20}"
TIMEOUT_PAD_S="${TIMEOUT_PAD_S:-18}"
MPC_DT="${MPC_DT:-0.02}"
MPC_HORIZON="${MPC_HORIZON:-20}"
CONTROL_RATE_HZ="${CONTROL_RATE_HZ:-50.0}"
UR_BUDGET_MS="${UR_BUDGET_MS:-10.0}"
ENFORCE_SOLVE_BUDGET="${ENFORCE_SOLVE_BUDGET:-false}"
ENABLE_VIEWER="${ENABLE_VIEWER:-false}"
RUN_SMOKE="${RUN_SMOKE:-1}"
RUN_PAPER="${RUN_PAPER:-1}"
RUN_PAYLOAD="${RUN_PAYLOAD:-0}"
LOG_ROOT="${LOG_ROOT:-${EXP_DIR}/paper_scale/batch_${TIMESTAMP}}"
PAPER_LOG_ROOT="${LOG_ROOT}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslog_ga_ocp_mujoco_${TIMESTAMP}}"

mkdir -p "${LOG_ROOT}" "${ROS_LOG_DIR}" "${EXP_DIR}/smoke"
export ROS_LOG_DIR
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

set +u
source /opt/ros/humble/setup.bash
source /home/chenwh/ros2_ws/install/setup.bash
set -u

IFS=' ' read -r -a BACKENDS <<< "${BACKENDS_STR}"
FAILED_CASES=()

timeout_seconds() {
  python3 - <<PY
duration = float("${1}")
pad = float("${TIMEOUT_PAD_S}")
print(int(duration + pad))
PY
}

run_case() {
  local duration="$1"
  local name="$2"
  shift 2
  local summary_csv="${LOG_ROOT}/${name}_summary.csv"
  local cycles_csv="${LOG_ROOT}/${name}_cycles.csv"
  local timeout_s
  timeout_s="$(timeout_seconds "${duration}")"
  rm -f "${summary_csv}" "${cycles_csv}"

  echo
  echo "============================================================"
  echo "Running MuJoCo closed-loop case: ${name}"
  echo "Output prefix: ${LOG_ROOT}/${name}"
  echo "Timeout: ${timeout_s}s"
  echo "Viewer: ${ENABLE_VIEWER}"
  echo "============================================================"

  if timeout --signal=INT "${timeout_s}s" \
    ros2 launch ga_ocp_ros2 ga_ocp_mujoco_closed_loop_ur.launch.py \
      duration_s:="${duration}" \
      dt:="${MPC_DT}" \
      horizon:="${MPC_HORIZON}" \
      control_rate_hz:="${CONTROL_RATE_HZ}" \
      solve_budget_ms:="${UR_BUDGET_MS}" \
      enforce_solve_budget:="${ENFORCE_SOLVE_BUDGET}" \
      enable_viewer:="${ENABLE_VIEWER}" \
      output_prefix:="${LOG_ROOT}/${name}" \
      "$@"; then
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

run_nominal() {
  local backend="$1"
  run_case "${DURATION_S}" "nominal_${backend}" \
    backend:="${backend}" \
    mass_scale:=1.0 \
    link_com_offset_x:=0.0 \
    link_com_offset_y:=0.0 \
    link_com_offset_z:=0.0 \
    plant_payload_mass:=0.0 \
    controller_payload_mass:=0.0 \
    model_payload:=false
}

run_mass_scale_cases() {
  local backend="$1"
  for scale in 0.7 0.8 0.9 1.1 1.2 1.3; do
    local tag
    tag="$(python3 - <<PY
scale = float("${scale}")
print(f"mass_{int(round(scale * 100)):03d}")
PY
)"
    run_case "${DURATION_S}" "${tag}_${backend}" \
      backend:="${backend}" \
      mass_scale:="${scale}" \
      link_com_offset_x:=0.0 \
      link_com_offset_y:=0.0 \
      link_com_offset_z:=0.0 \
      plant_payload_mass:=0.0 \
      controller_payload_mass:=0.0 \
      model_payload:=false
  done
}

run_payload_cases() {
  local backend="$1"
  for payload_mass in 2.0 5.0; do
    local payload_tag
    payload_tag="$(printf '%skg' "${payload_mass%.*}")"
    run_case "${DURATION_S}" "payload_${payload_tag}_modeled_${backend}" \
      backend:="${backend}" \
      mass_scale:=1.0 \
      link_com_offset_x:=0.0 \
      link_com_offset_y:=0.0 \
      link_com_offset_z:=0.0 \
      plant_payload_mass:="${payload_mass}" \
      controller_payload_mass:="${payload_mass}" \
      model_payload:=true

    run_case "${DURATION_S}" "payload_${payload_tag}_ignored_${backend}" \
      backend:="${backend}" \
      mass_scale:=1.0 \
      link_com_offset_x:=0.0 \
      link_com_offset_y:=0.0 \
      link_com_offset_z:=0.0 \
      plant_payload_mass:="${payload_mass}" \
      controller_payload_mass:=0.0 \
      model_payload:=false
  done
}

run_com_cases() {
  local backend="$1"
  for offset in 0.01 0.02 0.05; do
    for axis in x y z; do
      local x="0.0"
      local y="0.0"
      local z="0.0"
      if [[ "${axis}" == "x" ]]; then
        x="${offset}"
      elif [[ "${axis}" == "y" ]]; then
        y="${offset}"
      else
        z="${offset}"
      fi
      local tag
      tag="$(python3 - <<PY
offset = float("${offset}")
print(f"link_com_{int(round(offset * 100)):02d}cm_${axis}")
PY
)"
      run_case "${DURATION_S}" "${tag}_${backend}" \
        backend:="${backend}" \
        mass_scale:=1.0 \
        link_com_offset_x:="${x}" \
        link_com_offset_y:="${y}" \
        link_com_offset_z:="${z}" \
        plant_payload_mass:=0.0 \
        controller_payload_mass:=0.0 \
        model_payload:=false
    done
  done
}

run_external_force_cases() {
  local backend="$1"
  local start_s="2.0"
  local duration_s
  duration_s="$(python3 - <<PY
duration = max(0.0, float("${DURATION_S}") - float("${start_s}"))
print(f"{duration:.9g}")
PY
)"
  for force in 5.0 10.0 20.0; do
    for axis in x y z; do
      local fx="0.0"
      local fy="0.0"
      local fz="0.0"
      if [[ "${axis}" == "x" ]]; then
        fx="${force}"
      elif [[ "${axis}" == "y" ]]; then
        fy="${force}"
      else
        fz="${force}"
      fi
      local tag
      tag="$(printf 'external_%sN_%s' "${force%.*}" "${axis}")"
      run_case "${DURATION_S}" "${tag}_${backend}" \
        backend:="${backend}" \
        mass_scale:=1.0 \
        link_com_offset_x:=0.0 \
        link_com_offset_y:=0.0 \
        link_com_offset_z:=0.0 \
        plant_payload_mass:=0.0 \
        controller_payload_mass:=0.0 \
        model_payload:=false \
        external_force_body_name:=wrist_3_link \
        external_force_start_s:="${start_s}" \
        external_force_duration_s:="${duration_s}" \
        external_force_x:="${fx}" \
        external_force_y:="${fy}" \
        external_force_z:="${fz}"
    done
  done
}

write_combined_summary() {
  local combined_csv="${LOG_ROOT}/combined_summary.csv"
  python3 - "${LOG_ROOT}" "${combined_csv}" <<'PY'
import csv
import pathlib
import sys

log_root = pathlib.Path(sys.argv[1])
combined_csv = pathlib.Path(sys.argv[2])
paths = sorted(p for p in log_root.glob("*_summary.csv") if p != combined_csv)
rows = []
fieldnames = None
for path in paths:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            continue
        if fieldnames is None:
            fieldnames = ["case_name", "summary_file"] + list(reader.fieldnames)
        for row in reader:
            rows.append({"case_name": path.name.removesuffix("_summary.csv"), "summary_file": path.name, **row})
if fieldnames is None:
    raise SystemExit("no summary CSVs found")
with combined_csv.open("w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)
print(combined_csv)
PY
}

write_summary_md() {
  local metrics_csv="${LOG_ROOT}/closed_loop_metrics_summary.csv"
  python3 "${ROOT_DIR}/reviewer_revision_experiments/scripts/summarize_closed_loop_cycles.py" \
    --input-root "${LOG_ROOT}" \
    --output-csv "${metrics_csv}"
  python3 "${ROOT_DIR}/reviewer_revision_experiments/scripts/summarize_mujoco_robustness.py" \
    --batch-root "${LOG_ROOT}" \
    --output-dir "${EXP_DIR}/paper_scale"
  cat > "${EXP_DIR}/summary.md" <<EOF
# MuJoCo Robustness Expanded Sweep

Status: generated by \`scripts/run_mujoco_closed_loop_sweep.sh\`.

Configuration:

- Simulator: MuJoCo through \`ga_ocp_ros2/scripts/joint_command_executor.py\`.
- Controller: GA-OCP closed-loop receding-horizon nominal-model MPC.
- Robot: UR10.
- Backends: ${BACKENDS_STR}.
- Duration per case: ${DURATION_S} s.
- MPC dt: ${MPC_DT} s.
- MPC horizon: ${MPC_HORIZON}.
- Control rate: ${CONTROL_RATE_HZ} Hz.
- Solve budget: ${UR_BUDGET_MS} ms.
- Enforce solve budget: ${ENFORCE_SOLVE_BUDGET}.
- Viewer enabled during this run: ${ENABLE_VIEWER}.
- Payload robustness cases included: ${RUN_PAYLOAD}.
- Link COM offset cases perturb all six UR10 moving links in the MuJoCo plant only.
- External force cases apply a persistent force on \`wrist_3_link\` from 2 s to the end
  of the trial; active-window metrics are reported separately.

Outputs:

- \`${LOG_ROOT#${EXP_DIR}/}/combined_summary.csv\`: per-case tracking/runtime summary.
- \`${LOG_ROOT#${EXP_DIR}/}/closed_loop_metrics_summary.csv\`: tracking, acceleration,
  jerk, torque-rate, and effort-rate metrics from cycle CSVs.
- \`${LOG_ROOT#${EXP_DIR}/}/*_cycles.csv\`: raw closed-loop cycle logs.
EOF
}

echo "Experiment root: ${EXP_DIR}"
echo "Log root: ${LOG_ROOT}"
echo "ROS log dir: ${ROS_LOG_DIR}"
echo "Backends: ${BACKENDS_STR}"

if [[ "${RUN_SMOKE}" == "1" ]]; then
  LOG_ROOT="${EXP_DIR}/smoke"
  for backend in "${BACKENDS[@]}"; do
    run_case "4.0" "visible_smoke_${backend}" \
      backend:="${backend}" \
      mass_scale:=1.0 \
      plant_payload_mass:=0.0 \
      controller_payload_mass:=0.0 \
      model_payload:=false
  done
  write_combined_summary || FAILED_CASES+=("__smoke_combined_summary__")
fi

if [[ "${RUN_PAPER}" == "1" ]]; then
  LOG_ROOT="${PAPER_LOG_ROOT}"
  mkdir -p "${LOG_ROOT}"
  for backend in "${BACKENDS[@]}"; do
    run_nominal "${backend}"
    run_mass_scale_cases "${backend}"
    if [[ "${RUN_PAYLOAD}" == "1" ]]; then
      run_payload_cases "${backend}"
    fi
    run_com_cases "${backend}"
    run_external_force_cases "${backend}"
  done
  write_combined_summary || FAILED_CASES+=("__combined_summary__")
  write_summary_md || FAILED_CASES+=("__metrics_summary__")
fi

echo
echo "============================================================"
echo "MuJoCo closed-loop sweep complete"
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
