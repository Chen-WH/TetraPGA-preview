# CasADi DoF Benchmark Rerun

Run root:
`reviewer_revision_experiments/08_casadi_dof_benchmark_rerun/release_core2_20260628_234751`

## Build and Run Conditions

- Build type: `Release`
- Release flags in both CMake caches: `-O3 -DNDEBUG`
- Build command policy: each target built separately with `cmake --build ... --target <target> -- -j1`
- Runtime CPU binding: `taskset -c 2 <benchmark>`
- CPU governor was externally configured by the user before this run.

## Completed CSV Outputs

- `TetraPGA_dyn_fo_bench`: 3 runs completed
  - `TetraPGA_dyn_fo_bench/TetraPGA_dyn_fo_bench_run01.csv`
  - `TetraPGA_dyn_fo_bench/TetraPGA_dyn_fo_bench_run02.csv`
  - `TetraPGA_dyn_fo_bench/TetraPGA_dyn_fo_bench_run03.csv`
- `Crocoddyl_fddp_bench`: 3 runs completed
  - `Crocoddyl_fddp_bench/Crocoddyl_fddp_bench_run01.csv`
  - `Crocoddyl_fddp_bench/Crocoddyl_fddp_bench_run02.csv`
  - `Crocoddyl_fddp_bench/Crocoddyl_fddp_bench_run03.csv`
- `Crocoddyl_sqp_bench`: 3 runs completed
  - `Crocoddyl_sqp_bench/Crocoddyl_sqp_bench_run01.csv`
  - `Crocoddyl_sqp_bench/Crocoddyl_sqp_bench_run02.csv`
  - `Crocoddyl_sqp_bench/Crocoddyl_sqp_bench_run03.csv`

## Aggregated Outputs

- `combined_runs_long.csv`: long-format records from all completed run CSVs.
- `combined_summary.csv`: mean/stddev/min/max across completed runs by benchmark, case, and DoF.

## Incomplete Benchmark

`TetraPGA_dyn_so_bench` was started as:

```bash
taskset -c 2 ./TetraPGA_dyn_so_bench --benchmark_out=.../TetraPGA_dyn_so_bench_run01.csv --benchmark_out_format=csv
```

It was interrupted after approximately 58 minutes because it remained on the high-DoF
CasADi second-order RNEA case after completing:

- `serial_chain/CasADi/ComputeRNEASecondOrderDerivatives/6`, DoF = 63

At the last host-side check, the process was still using about one full CPU core and about
2.05 GB RSS, so this was compute-bound rather than a crash. No CSV was produced because the
pivot CSV reporter writes at benchmark completion.

Current source settings for this benchmark are:

- `kBenchmarkIterations = 1024`
- `kCasadiMaxLevel = 8`, corresponding to DoF up to 255

This combination is not practical for a three-run paper benchmark. Recommended follow-up:
split the CasADi second-order benchmark from the TetraPGA/Pinocchio second-order benchmark
and either reduce the CasADi DoF cap or reduce CasADi fixed iterations.
