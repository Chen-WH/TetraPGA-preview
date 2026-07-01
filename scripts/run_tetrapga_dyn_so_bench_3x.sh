#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

runs="${RUNS:-3}"
bench_exe="${BENCH_EXE:-${repo_root}/build-bench/TetraPGA_dyn_so_bench}"
out_root="${OUT_ROOT:-${repo_root}/reviewer_revision_experiments/08_casadi_dof_benchmark_rerun}"
stamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${OUT_DIR:-${out_root}/tetrapga_dyn_so_bench_3x_${stamp}}"

if [[ ! "${runs}" =~ ^[0-9]+$ ]] || [[ "${runs}" -lt 1 ]]; then
  echo "RUNS must be a positive integer, got: ${runs}" >&2
  exit 2
fi

if [[ ! -x "${bench_exe}" ]]; then
  echo "Benchmark executable not found or not executable: ${bench_exe}" >&2
  echo "Build it first, or set BENCH_EXE=/path/to/TetraPGA_dyn_so_bench." >&2
  exit 2
fi

mkdir -p "${out_dir}"

{
  echo "benchmark: ${bench_exe}"
  echo "repo_root: ${repo_root}"
  echo "out_dir: ${out_dir}"
  echo "runs: ${runs}"
  echo "taskset_core: ${TASKSET_CORE:-none}"
  echo "start_time: $(date --iso-8601=seconds)"
  echo "extra_args: $*"
  if git -C "${repo_root}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "git_head: $(git -C "${repo_root}" rev-parse HEAD)"
    echo "git_status_short:"
    git -C "${repo_root}" status --short
  fi
} >"${out_dir}/metadata.txt"

echo "Output directory: ${out_dir}"
echo "Benchmark: ${bench_exe}"

for run_idx in $(seq 1 "${runs}"); do
  run_tag="$(printf 'run%02d' "${run_idx}")"
  csv_path="${out_dir}/TetraPGA_dyn_so_bench_${run_tag}.csv"
  log_path="${out_dir}/TetraPGA_dyn_so_bench_${run_tag}.log"

  if [[ -e "${csv_path}" || -e "${log_path}" ]]; then
    echo "Refusing to overwrite existing output for ${run_tag}: ${csv_path}" >&2
    exit 3
  fi

  cmd=("${bench_exe}" "$@" "--benchmark_out=${csv_path}" "--benchmark_out_format=csv")
  if [[ -n "${TASKSET_CORE:-}" ]]; then
    cmd=(taskset -c "${TASKSET_CORE}" "${cmd[@]}")
  fi

  echo
  echo "[$(date --iso-8601=seconds)] Starting ${run_tag}/${runs}"
  printf 'Command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  {
    echo "start_time: $(date --iso-8601=seconds)"
    printf 'command:'
    printf ' %q' "${cmd[@]}"
    printf '\n'
    "${cmd[@]}"
    echo "end_time: $(date --iso-8601=seconds)"
  } 2>&1 | tee "${log_path}"

  echo "[$(date --iso-8601=seconds)] Finished ${run_tag}/${runs}"
  echo "CSV: ${csv_path}"
  echo "Log: ${log_path}"
done

echo
echo "All runs completed."
echo "Output directory: ${out_dir}"
