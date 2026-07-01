# CasADi 20GiB Memory-Only OOM Sweep

Status: completed.

This run disables wall-clock timeouts and the earlier 19.5GiB RSS protection
line. Each probe is still isolated in a subprocess and receives a 20GiB
`RLIMIT_AS` address-space cap.

Command:

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

Results:

| Case | Last success | First memory-limit failure | Failure status | Last success peak RSS | Last success graph build |
| --- | ---: | ---: | --- | ---: | ---: |
| `aba_fo` | 410 DoF | 411 DoF | `exit_2` / `std::bad_alloc` | 20330.105 MiB | 359.968 s |
| `rnea_fo` | 417 DoF | 418 DoF | `exit_2` / `std::bad_alloc` | 20421.199 MiB | 166.434 s |
| `rnea_so` | 72 DoF | not reached | `ok` | 293.246 MiB | 701.725 s |

Interpretation:

- Removing the 19.5GiB RSS protection line moves the first-order OOM boundary
  slightly upward relative to the protected reviewer-facing run.
- The failure mode is now the probe process returning `exit_2` after catching
  `std::bad_alloc`, rather than the runner killing the process at an RSS
  threshold.
- No timeout was used. The first-order boundary is therefore memory-limited
  under the configured 20GiB address-space cap.
- The `rnea_so` 72 DoF check also used no timeout and no RSS kill. It completed
  with low memory use, so the second-order case remains construction-time
  limited at this DoF rather than memory-limited.
