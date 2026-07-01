# Next Steps

Completed in this pass:

- Archived existing CasADi graph feasibility results.
- Ran model/dynamics consistency checks for UR10, LEAP hand, and Stanford
  TidyBot.
- Added and ran a three-model fixed-budget OCP pilot.
- Added and ran a UR10-only obstacle safety-margin sweep pilot.
- Re-ran fixed-budget OCP at paper scale with 20 samples per model.
- Diagnosed Stanford TidyBot fixed-budget non-saturation as an iteration-cap
  artifact and generated `paper_scale_reviewed/` with `max_iterations=100`.
- Tuned and re-ran UR10 obstacle safety-margin sensitivity at paper scale with
  endpoint-clearance rejection sampling.
- Added and ran UR10 robustness expanded sweep at paper scale over
  mass/inertia scaling, end-body COM offset, and external impulse disturbance.
- Added `review_experiment_outputs.py` and generated `review_report.md`.
- Generated `experiment_index.md`.

Recommended next work:

1. Add runtime breakdown instrumentation:
   separate action `calc`, action `calcDiff`, cost/collision residuals, and
   residual solver overhead.

2. Add closed-loop CSV post-processing:
   compute jerk RMS, acceleration variation, torque-rate RMS, tracking RMSE,
   solve-time mean/p95, deadline miss rate, and failure rate from
   `closed_loop_mpc_node` cycle logs.
