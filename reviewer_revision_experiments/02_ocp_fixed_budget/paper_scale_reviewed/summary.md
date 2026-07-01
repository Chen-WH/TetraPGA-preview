# Fixed-Budget OCP Reviewed Paper-Scale Rerun

Status: completed and reviewed.

Configuration:

- Models: UR10, LEAP hand left, Stanford TidyBot Gen3.
- Backends: TetraPGA, Pinocchio, CasADi.
- Samples: 20 per model.
- Horizon: 50.
- Max iterations: 100.
- Budgets: 1, 2, 5, 10, 20, 50, 100, 200 ms.
- Seeds are matched to the initial `paper_scale/` run for per-model
  comparability.

Outputs:

- `fixed_budget_paper_reviewed_summary.csv`: merged summary.
- `ur10/ur10_{samples,runs,trace,summary}.csv`.
- `leap_hand/leap_hand_{samples,runs,trace,summary}.csv`.
- `stanford_tidybot/stanford_tidybot_{samples,runs,trace,summary}.csv`.
- `ur10.log`, `leap_hand.log`, `stanford_tidybot.log`.

Key observations:

- UR10 reaches full success at 5 ms for TetraPGA and Pinocchio, and at 20 ms
  for CasADi.
- LEAP hand reaches full success at 10 ms for TetraPGA, 50 ms for Pinocchio,
  and 100 ms for CasADi.
- Stanford TidyBot reaches full success at 100 ms for TetraPGA and Pinocchio,
  and at 200 ms for CasADi. The earlier non-saturated TidyBot curve was due to
  the initial 25-iteration cap.
