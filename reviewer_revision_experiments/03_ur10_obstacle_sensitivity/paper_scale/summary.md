# UR10 Obstacle Sensitivity Paper-Scale Rerun

Status: completed.

Configuration:

- Safety margins: 0.03, 0.05, 0.08, 0.10, 0.15 m.
- Obstacle counts: 2, 4, 8, 16.
- Samples: 20 per setting.
- Horizon: 50.
- Max iterations: 100.
- Target perturbation amplitude: 0.18 rad.
- Collision weight: 1500.
- Obstacle radii: 0.03 to 0.06 m.
- Endpoint clearance rejection threshold: 0.20 m.

Outputs:

- `ur10_margin_sweep_paper.csv`: 400 raw rows.
- `ur10_margin_sweep_paper_summary.csv`: grouped rates and metrics.
- `ur10_margin_sweep_paper.log`: run log.
- `tuning/`: parameter-tuning runs kept for traceability, not for paper
  tables.

Key observations:

- `failed_rate` is 0 for all groups.
- At `d_safe=0.03`, collision-free rates are 0.80, 0.80, 0.50, and 0.25 for
  2, 4, 8, and 16 obstacles.
- At `d_safe=0.15`, safety-satisfied rates are 0.35, 0.15, 0.00, and 0.00 for
  2, 4, 8, and 16 obstacles.
- The 16-obstacle condition is a high-density stress case; 2/4/8 obstacles
  provide the clearest margin-sensitivity trend.
