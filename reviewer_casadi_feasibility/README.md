# CasADi Graph Feasibility Sweep for Reviewer Response

This folder contains the reviewer-response experiment used to distinguish
CasADi graph-construction feasibility from post-construction runtime.

The retained results are from the protected 20 GB run on the no-swap
workstation. Older exploratory runs with higher memory exposure were removed
to avoid confusing the reviewer-facing evidence.

## Contents

- `casadi_graph_probe.cpp`: standalone C++ probe that constructs one CasADi
  dynamics graph for a requested case and DoF.
- `run_casadi_feasibility.py`: Python runner that launches one probe process
  per DoF, applies memory/time limits, records peak RSS, and refines the first
  failure interval to integer DoF.
- `results_oom20_summary.csv`: reviewer-facing summary.
- `results_oom20_notes.md`: reviewer-facing interpretation.
- `results_oom20_memonly_summary.csv`: no-timeout, no-RSS-kill 20GiB
  address-space OOM summary.
- `results_oom20_memonly_notes.md`: interpretation of the no-timeout
  memory-only sweep.
- `results_oom20_first_order/`: raw and summary CSVs for first-order ABA/RNEA
  OOM threshold tests.
- `results_oom20_rnea_so_time/`: raw and summary CSVs for the second-order
  RNEA construction-time check.
- `results_oom20_memonly_first_order/`: raw and summary CSVs for the
  no-timeout first-order memory-only sweep.
- `results_oom20_memonly_rnea_so/`: raw and summary CSVs for the no-timeout
  second-order RNEA check.
- `results_oom20_10min_rnea_so/`: raw and summary CSVs for the RNEA
  second-order 20GiB memory-bottleneck sweep with a 10-minute per-DoF cap.
- `results_oom20_10min_rnea_so_summary.csv`: concise summary for the
  10-minute memory-rule second-order sweep.
- `results_oom20_10min_rnea_so_notes.md`: interpretation of the 10-minute
  memory-rule second-order sweep.

## Build

```bash
cmake -S reviewer_casadi_feasibility -B reviewer_casadi_feasibility/build \
  -DTetraPGA_DIR=/home/chenwh/ros2_ws/install/TetraPGA/lib/cmake/TetraPGA \
  -DCMAKE_PREFIX_PATH=/opt/openrobots:/usr/local
cmake --build reviewer_casadi_feasibility/build -j
```

The `build/` directory is ignored by git.

## Reproduce 20 GB OOM Sweep

The first-order sweep uses a 20 GB address-space cap and a 19.5 GiB RSS
protection line to avoid destabilizing the no-swap desktop session.

```bash
python3 reviewer_casadi_feasibility/run_casadi_feasibility.py \
  --probe reviewer_casadi_feasibility/build/casadi_graph_probe \
  --output-dir reviewer_casadi_feasibility/results_oom20_first_order \
  --cases aba_fo,rnea_fo \
  --max-dof 511 \
  --initial-step 64 \
  --extra-dofs 127,255,511 \
  --memory-limit-gb 20 \
  --rss-kill-gb 19.5 \
  --timeout-s 1800 \
  --poll-s 0.05 \
  --refine-window 8
```

The second-order RNEA check verifies that the 72 DoF case is not memory-limited
under the same cap, but is instead dominated by graph-construction time.

```bash
python3 reviewer_casadi_feasibility/run_casadi_feasibility.py \
  --probe reviewer_casadi_feasibility/build/casadi_graph_probe \
  --output-dir reviewer_casadi_feasibility/results_oom20_rnea_so_time \
  --cases rnea_so \
  --max-dof 72 \
  --initial-step 72 \
  --extra-dofs 72 \
  --memory-limit-gb 20 \
  --rss-kill-gb 19.5 \
  --timeout-s 900 \
  --poll-s 0.1 \
  --refine-window 1
```

## Results

| Case | Interpretation | Last success | First protected-memory failure |
| --- | --- | ---: | ---: |
| `aba_fo` | ABA first-order graph used by forward-dynamics OCP baseline | 407 DoF | 408 DoF |
| `rnea_fo` | RNEA first-order graph used by inverse-dynamics OCP baseline | 413 DoF | 414 DoF |
| `rnea_so` | Second-order RNEA graph for component-level ID-SO baseline | 72 DoF | not reached |

For `rnea_so`, the 72 DoF graph completed under the 20 GB cap with only about
307 MiB peak RSS, but graph construction took about 706 s. This case should be
reported as construction-time limited at that DoF, not memory-limited.

## 20GiB Memory-Only First-Order Sweep

This run disables both wall-clock timeout and the earlier 19.5GiB RSS
protection line, while keeping the 20GiB process address-space cap:

```bash
python3 reviewer_casadi_feasibility/run_casadi_feasibility.py \
  --probe reviewer_casadi_feasibility/build/casadi_graph_probe \
  --output-dir reviewer_casadi_feasibility/results_oom20_memonly_first_order \
  --cases aba_fo,rnea_fo \
  --max-dof 511 \
  --initial-step 64 \
  --extra-dofs 127,255,511 \
  --memory-limit-gb 20 \
  --rss-kill-gb 0 \
  --timeout-s 0 \
  --poll-s 0.1 \
  --refine-window 8
```

| Case | Last success | First memory-limit failure |
| --- | ---: | ---: |
| `aba_fo` | 410 DoF | 411 DoF |
| `rnea_fo` | 417 DoF | 418 DoF |
| `rnea_so` | 72 DoF | not reached |

The failure status is `exit_2` from the probe catching `std::bad_alloc`, not a
runner timeout or RSS-threshold kill. For `rnea_so`, 72 DoF completes with low
memory use but takes about 702 s to construct, so it remains construction-time
limited at that DoF.

## RNEA Second-Order 10-Minute Memory Rule Sweep

For the reviewer memory-bottleneck check that ignores full graph-construction
completion time, each DoF is capped at 600 s. If the probe does not fail within
600 s, it is treated as memory-feasible.

| Case | Last pass | First failure |
| --- | ---: | ---: |
| `rnea_so` | 402 DoF | 403 DoF |

Here, `402 DoF` is a `timeout_ok` pass: it did not fail within 600 s under the
20GiB address-space cap. `403 DoF` failed with `std::bad_alloc` / `exit_2` at
about 455 s.
