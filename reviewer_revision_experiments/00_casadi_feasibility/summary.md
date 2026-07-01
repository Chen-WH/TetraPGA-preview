# CasADi Graph Feasibility

Status: completed from protected reviewer feasibility runs and a new
no-timeout memory-only first-order sweep.

Key results on the protected 20 GB run:

- `aba_fo`: last successful graph construction at 407 DoF; first RSS-limit
  failure at 408 DoF.
- `rnea_fo`: last successful graph construction at 413 DoF; first RSS-limit
  failure at 414 DoF.
- `rnea_so`: 72 DoF completed without memory pressure, but graph construction
  took about 706 s, so the relevant limitation is construction time rather than
  memory at this DoF.

Key results on the 20GiB memory-only first-order run:

- Configuration: `RLIMIT_AS=20GiB`, `rss_kill=0`, `timeout_s=0`.
- `aba_fo`: last successful graph construction at 410 DoF; first
  memory-limit failure at 411 DoF with `std::bad_alloc` / `exit_2`.
- `rnea_fo`: last successful graph construction at 417 DoF; first
  memory-limit failure at 418 DoF with `std::bad_alloc` / `exit_2`.
- `rnea_so`: 72 DoF completed under the same no-timeout, no-RSS-kill memory
  cap; peak RSS was about 293 MiB and graph construction took about 702 s. No
  memory boundary was reached for this second-order check.
- This run deliberately removes the earlier 19.5GiB RSS protection line, so
  its boundary is slightly higher than the protected reviewer-facing run.

Key result on the `rnea_so` 20GiB / 10-minute memory-bottleneck rule run:

- Configuration: `RLIMIT_AS=20GiB`, `rss_kill=0`, `timeout_s=600`,
  `timeout_as_pass=true`.
- Rule: if a DoF does not fail within 600 s, it is treated as memory-feasible
  and recorded as `timeout_ok`.
- `rnea_so`: last pass at 402 DoF; first memory-allocation failure at 403 DoF.
  The 403 DoF probe failed with `std::bad_alloc` / `exit_2` after about
  455 s. The 402 DoF probe did not fail within 600 s and reached about
  18.34 GiB peak RSS.

Use in paper/response: state explicitly that the omitted high-DoF CasADi cases
are graph-construction infeasibility/time cases, not measured runtime points.
Use the protected run for conservative reviewer-facing claims; use the
memory-only run if the response needs a strict "no timeout, memory cap only"
cross-check.
For `rnea_so`, use the 10-minute rule result only when explicitly explaining
the memory-bottleneck boundary under the "do not wait for full construction"
protocol.
