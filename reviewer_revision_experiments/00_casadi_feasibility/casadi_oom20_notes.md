# 20 GB Protected CasADi Feasibility Results

All protected measurements were run on the no-swap workstation with:

- `RLIMIT_AS=20GB`
- RSS protection threshold `19.5GiB`
- one CasADi probe process at a time

Summary:

| Case | Last success | First failure | Failure mode | Last-success peak RSS | Last-success graph build |
| --- | ---: | ---: | --- | ---: | ---: |
| `aba_fo` | 407 DoF | 408 DoF | RSS protection | 19927.918 MiB | 345.652 s |
| `rnea_fo` | 413 DoF | 414 DoF | RSS protection | 19843.461 MiB | 157.140 s |
| `rnea_so` | 72 DoF | not reached | time-heavy, not OOM | 306.938 MiB | 705.624 s |

Interpretation for the paper response:

- The high-DoF first-order CasADi graphs used by the OCP CasADi baselines hit
  the protected 20 GB memory budget at about 408-414 DoF on this machine.
- The second-order RNEA CasADi graph does not show memory pressure at 72 DoF;
  instead, graph construction itself takes about 12 minutes. For this case,
  the relevant limitation is construction feasibility/time, not post-graph
  runtime and not memory at 72 DoF.
