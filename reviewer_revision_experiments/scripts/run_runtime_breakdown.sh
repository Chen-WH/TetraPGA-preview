#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/chenwh/ros2_ws/src/TetraPGA-preview"
GA_CORE_DIR="/home/chenwh/ros2_ws/src/GA-OCP/ga_ocp_core"
BUILD_DIR="/home/chenwh/ros2_ws/build/ga_ocp_core"
EXP_DIR="${ROOT_DIR}/reviewer_revision_experiments/07_runtime_breakdown"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

ROBOTS_STR="${ROBOTS_STR:-ur10,leap_hand,unitree_g1}"
BACKENDS_STR="${BACKENDS_STR:-tetrapga,pinocchio,casadi}"
SAMPLES="${SAMPLES:-24}"
SEED="${SEED:-20260627}"
HORIZON="${HORIZON:-50}"
DT="${DT:-0.02}"
MAX_ITERATIONS="${MAX_ITERATIONS:-25}"
POSITION_LIMIT="${POSITION_LIMIT:-0.75}"
BUILD_TARGET="${BUILD_TARGET:-true}"
BUILD_JOBS="${BUILD_JOBS:-1}"
LOG_ROOT="${LOG_ROOT:-${EXP_DIR}/paper_scale/batch_${TIMESTAMP}_offline_runtime}"

mkdir -p "${LOG_ROOT}" "${EXP_DIR}/paper_scale"

set +u
source /opt/ros/humble/setup.bash
source /home/chenwh/ros2_ws/install/setup.bash
set -u

python3 "${ROOT_DIR}/reviewer_revision_experiments/scripts/apply_ga_ocp_offline_runtime_breakdown.py"

if [[ "${BUILD_TARGET}" == "true" ]]; then
  cmake -S "${GA_CORE_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGA_OCP_BUILD_BENCHMARKS=ON
  cmake --build "${BUILD_DIR}" --target Crocoddyl_runtime_breakdown -- -j"${BUILD_JOBS}"
fi

"${BUILD_DIR}/Crocoddyl_runtime_breakdown" \
  --robots="${ROBOTS_STR}" \
  --backends="${BACKENDS_STR}" \
  --samples="${SAMPLES}" \
  --seed="${SEED}" \
  --horizon="${HORIZON}" \
  --dt="${DT}" \
  --max_iterations="${MAX_ITERATIONS}" \
  --position_limit="${POSITION_LIMIT}" \
  --output_dir="${LOG_ROOT}"

cp "${LOG_ROOT}/runtime_breakdown_summary.csv" \
  "${EXP_DIR}/paper_scale/runtime_breakdown_summary.csv"
cp "${LOG_ROOT}/runtime_breakdown_summary.csv" \
  "${EXP_DIR}/paper_scale/runtime_breakdown_combined_summary.csv"
cp "${LOG_ROOT}/runtime_breakdown_stack_summary.csv" \
  "${EXP_DIR}/paper_scale/runtime_breakdown_stack_summary.csv"
cp "${LOG_ROOT}/runtime_breakdown_raw.csv" \
  "${EXP_DIR}/paper_scale/runtime_breakdown_raw.csv"
cp "${LOG_ROOT}/runtime_breakdown_iterations.csv" \
  "${EXP_DIR}/paper_scale/runtime_breakdown_iterations.csv"

cat > "${EXP_DIR}/summary.md" <<EOF
# Offline Runtime Breakdown

Status: completed from random point-to-point FDDP tasks, not closed-loop ROS/MuJoCo MPC.

Outputs:

- \`paper_scale/runtime_breakdown_stack_summary.csv\`: one row per robot/backend, ready for stacked bar plots.
- \`paper_scale/runtime_breakdown_summary.csv\`: detailed per robot/backend summary.
- \`paper_scale/runtime_breakdown_raw.csv\`: one row per random sample solve.
- \`paper_scale/runtime_breakdown_iterations.csv\`: per solver callback/iteration interval timing.
- \`paper_scale/$(basename "${LOG_ROOT}")\`: full batch copy.

Configuration:

- Robots: \`${ROBOTS_STR}\`
- Backends: \`${BACKENDS_STR}\`
- Samples: \`${SAMPLES}\`
- Horizon: \`${HORIZON}\`
- dt: \`${DT}\`
- Max iterations: \`${MAX_ITERATIONS}\`
- Seed: \`${SEED}\`

Instrumentation:

- \`DAM.calc\` and \`DAM.calcDiff\` are timed inside the Crocoddyl solver loop by a benchmark-only differential action wrapper.
- Cost-model residuals are separately timed for state, control, and acceleration regularization.
- Collision cost is intentionally disabled for all three robots, so collision timing columns should remain zero.
- Solver overhead is computed as \`solver total - DAM.calc - DAM.calcDiff\`, covering backward pass, linear algebra, line search, regularization, and other solver-side work.
- The runtime target is compiled as Release and explicitly defines \`NDEBUG\` with \`-O3\`.
EOF

echo "Runtime breakdown complete: ${LOG_ROOT}"
