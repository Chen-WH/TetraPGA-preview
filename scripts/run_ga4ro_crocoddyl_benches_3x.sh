#!/usr/bin/env bash
set -Eeo pipefail

# Source ROS before enabling nounset; ROS setup files may read unset variables.
source /opt/ros/humble/setup.bash
if [[ -f /home/chenwh/ros2_ws/install/setup.bash ]]; then
  source /home/chenwh/ros2_ws/install/setup.bash
fi

set -Eeuo pipefail

source_dir="${SOURCE_DIR:-/home/chenwh/ros2_ws/src/GA4Ro}"
build_dir="${BUILD_DIR:-/tmp/ga4ro_crocoddyl_kmaxad_build}"
out_root="${OUT_ROOT:-/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/08_casadi_dof_benchmark_rerun}"
stamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${OUT_DIR:-${out_root}/ga4ro_crocoddyl_kmaxad_rerun_${stamp}}"
runs="${RUNS:-3}"
build_jobs="${BUILD_JOBS:-1}"
python_executable="${Python_EXECUTABLE:-${PYTHON_EXECUTABLE:-/usr/bin/python3}}"
do_configure="${DO_CONFIGURE:-1}"
do_build="${DO_BUILD:-1}"
benchmarks="${BENCHMARKS:-Crocoddyl_fddp_bench Crocoddyl_sqp_bench}"

if [[ ! "${runs}" =~ ^[0-9]+$ ]] || [[ "${runs}" -lt 1 ]]; then
  echo "RUNS must be a positive integer, got: ${runs}" >&2
  exit 2
fi

if [[ ! "${build_jobs}" =~ ^[0-9]+$ ]] || [[ "${build_jobs}" -lt 1 ]]; then
  echo "BUILD_JOBS must be a positive integer, got: ${build_jobs}" >&2
  exit 2
fi

if [[ ! -d "${source_dir}" ]]; then
  echo "GA4Ro source directory not found: ${source_dir}" >&2
  exit 2
fi

mkdir -p "${out_dir}"

{
  echo "source_dir: ${source_dir}"
  echo "build_dir: ${build_dir}"
  echo "out_dir: ${out_dir}"
  echo "runs_per_benchmark: ${runs}"
  echo "benchmarks: ${benchmarks}"
  echo "taskset_core: ${TASKSET_CORE:-none}"
  echo "cmake_build_type: Release"
  echo "casadi_bench: ON"
  echo "do_configure: ${do_configure}"
  echo "do_build: ${do_build}"
  echo "build_jobs: ${build_jobs}"
  echo "python_executable: ${python_executable}"
  echo "start_time: $(date --iso-8601=seconds)"
  if command -v rg >/dev/null 2>&1; then
    rg -n "constexpr int kMaxLevel_AD" \
      "${source_dir}/benchmark/Crocoddyl_fddp_bench.cpp" \
      "${source_dir}/benchmark/Crocoddyl_sqp_bench.cpp" || true
  fi
  if git -C "${source_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "git_head: $(git -C "${source_dir}" rev-parse HEAD)"
    echo "git_status_short:"
    git -C "${source_dir}" status --short
  fi
  echo "extra_benchmark_args: $*"
} >"${out_dir}/metadata.txt"

echo "Output directory: ${out_dir}"

if [[ "${do_configure}" == "1" ]]; then
  echo "Configuring GA4Ro benchmark build..."
  cmake -S "${source_dir}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGA4RO_ENABLE_CASADI_BENCH=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DPython_EXECUTABLE="${python_executable}" \
    2>&1 | tee "${out_dir}/configure.log"
fi

read -r -a benchmark_array <<<"${benchmarks}"

if [[ "${do_build}" == "1" ]]; then
  for bench in "${benchmark_array[@]}"; do
    echo
    echo "Building ${bench} with -j${build_jobs}..."
    cmake --build "${build_dir}" --target "${bench}" -- -j"${build_jobs}" \
      2>&1 | tee "${out_dir}/build_${bench}.log"
  done
fi

for bench in "${benchmark_array[@]}"; do
  exe="${build_dir}/${bench}"
  if [[ ! -x "${exe}" ]]; then
    echo "Benchmark executable not found or not executable: ${exe}" >&2
    exit 3
  fi

  mkdir -p "${out_dir}/${bench}"

  if [[ -f "${build_dir}/CMakeFiles/${bench}.dir/flags.make" ]]; then
    cp "${build_dir}/CMakeFiles/${bench}.dir/flags.make" \
      "${out_dir}/${bench}/flags.make"
  fi

  "${exe}" --benchmark_list_tests >"${out_dir}/${bench}/benchmark_list_tests.txt"

  for run_idx in $(seq 1 "${runs}"); do
    run_tag="$(printf 'run%02d' "${run_idx}")"
    csv_path="${out_dir}/${bench}/${bench}_${run_tag}.csv"
    log_path="${out_dir}/${bench}/${bench}_${run_tag}.log"

    if [[ -e "${csv_path}" || -e "${log_path}" ]]; then
      echo "Refusing to overwrite existing output for ${bench} ${run_tag}" >&2
      exit 4
    fi

    cmd=("${exe}" "$@" "--benchmark_out=${csv_path}" "--benchmark_out_format=csv")
    if [[ -n "${TASKSET_CORE:-}" ]]; then
      cmd=(taskset -c "${TASKSET_CORE}" "${cmd[@]}")
    fi

    echo
    echo "[$(date --iso-8601=seconds)] START ${bench} ${run_tag}/${runs}"
    printf 'Command:'
    printf ' %q' "${cmd[@]}"
    printf '\n'

    start_s="$(date +%s)"
    {
      echo "start_time: $(date --iso-8601=seconds)"
      printf 'command:'
      printf ' %q' "${cmd[@]}"
      printf '\n'
      "${cmd[@]}"
      echo "end_time: $(date --iso-8601=seconds)"
    } >"${log_path}" 2>&1
    end_s="$(date +%s)"

    line_count="$(wc -l <"${csv_path}")"
    echo "[$(date --iso-8601=seconds)] DONE  ${bench} ${run_tag}/${runs} duration_s=$((end_s - start_s)) csv_lines=${line_count}"
    echo "CSV: ${csv_path}"
    echo "Log: ${log_path}"
  done
done

echo
echo "All runs completed."
echo "Output directory: ${out_dir}"
