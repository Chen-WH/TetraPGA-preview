# UR10 Obstacle Safety-Margin Sensitivity

Status: pilot, tuned sampling, and paper-scale rerun completed.

Paper-scale configuration:

- Model: UR10 only.
- Safety margins: 0.03, 0.05, 0.08, 0.10, 0.15 m.
- Obstacle counts: 2, 4, 8, 16.
- Samples: 20 per setting.
- Horizon: 50.
- Max iterations: 100.
- Target perturbation amplitude: 0.18 rad.
- Collision weight: 1500.
- Obstacle radii: 0.03 to 0.06 m.
- Endpoint clearance rejection threshold: 0.20 m, so sampled environments do
  not start from trivially infeasible initial or target configurations.

Paper-scale outputs:

- `paper_scale/ur10_margin_sweep_paper.csv`: raw per-run rows.
- `paper_scale/ur10_margin_sweep_paper_summary.csv`: grouped means and
  derived rates.
- `paper_scale/ur10_margin_sweep_paper.log`: run log.

Paper-scale observations:

- No benchmark rows failed due to exceptions or sampling failures
  (`failed_rate=0` for all groups).
- At `d_safe=0.03`, collision-free rates are 0.80, 0.80, 0.50, and 0.25 for
  2, 4, 8, and 16 obstacles. Safety-satisfied rates are 0.80, 0.65, 0.30,
  and 0.10.
- At `d_safe=0.15`, collision-free rates are 0.85, 0.85, 0.45, and 0.35, but
  safety-satisfied rates drop to 0.35, 0.15, 0.00, and 0.00. This separates
  physical collision avoidance from satisfying a stricter clearance margin.
- Mean solve time stays around 90 to 110 ms across the sweep. The 16-obstacle
  case is a useful stress case, while 2/4/8 obstacles give the most readable
  margin-sensitivity curves.
- For reporting, prefer `collision_free_rate`, `safety_satisfied_rate`, min
  clearance, solve time, placement error, and jerk over Crocoddyl's raw solver
  convergence boolean.

Sampling implementation note:

- `Crocoddyl_obstacle_margin_sweep.cpp` now exposes obstacle radius and
  endpoint-clearance options and uses rejection sampling to avoid initial and
  target configurations that are already too close to sampled obstacles.

Pilot configuration:

- Model: UR10 only.
- Safety margins: 0.03, 0.05, 0.08, 0.10, 0.15 m.
- Obstacle counts: 4 and 8.
- Samples: 3 per setting.
- Horizon: 40.
- Max iterations: 40.

Pilot outputs:

- `ur10_margin_sweep_pilot.csv`: raw per-run rows.
- `ur10_margin_sweep_pilot_summary.csv`: grouped means and derived rates.

Notes:

- The pilot obstacle sampler is intentionally cluttered and produced many
  collision/safety violations. This is useful for stress testing the sweep
  infrastructure, but the final paper-scale run should tune obstacle placement
  and collision weight so the success-rate curve is informative rather than
  uniformly difficult.
- For paper reporting, prefer `collision_free_rate`, `safety_satisfied_rate`,
  min clearance, solve time, placement error, and jerk over Crocoddyl's raw
  solver convergence boolean.
