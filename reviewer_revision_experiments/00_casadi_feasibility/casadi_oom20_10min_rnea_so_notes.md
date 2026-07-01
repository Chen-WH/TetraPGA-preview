# CasADi RNEA Second-Order 20GiB / 10-Minute Memory Rule Sweep

Status: completed.

Rule:

- Per probe: `RLIMIT_AS=20GiB`.
- No RSS-kill threshold.
- Per-DoF wall-clock cap: 600 s.
- If a DoF does not fail within 600 s, it is treated as a feasible memory
  point and recorded as `timeout_ok`.
- If a DoF fails before 600 s, the probe status is used as the memory-failure
  result.

Command:

```bash
python3 reviewer_casadi_feasibility/run_casadi_feasibility.py \
  --probe reviewer_casadi_feasibility/build/casadi_graph_probe \
  --output-dir reviewer_casadi_feasibility/results_oom20_10min_rnea_so \
  --cases rnea_so \
  --max-dof 512 \
  --initial-step 512 \
  --extra-dofs 304,512 \
  --memory-limit-gb 20 \
  --rss-kill-gb 0 \
  --timeout-s 600 \
  --timeout-as-pass \
  --poll-s 5.0 \
  --refine-window 8
```

Result:

| Case | Last pass | First failure | Failure status | Last pass RSS | First failure elapsed |
| --- | ---: | ---: | --- | ---: | ---: |
| `rnea_so` | 402 DoF | 403 DoF | `exit_2` / `std::bad_alloc` | 18338.676 MiB | 455.436 s |

Tested DoFs:

- Passed by 600 s rule: 304, 356, 382, 395, 401, 402.
- Failed before 600 s: 403, 408, 512.

Interpretation:

- Under the user's memory-bottleneck rule, the second-order RNEA CasADi graph
  boundary is 402/403 DoF.
- `timeout_ok` rows are not completed graph-construction timings; they mean
  the process did not fail within 600 s and is therefore treated as memory
  feasible for this specific test.
