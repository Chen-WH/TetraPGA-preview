# Fixed-Budget OCP Paper-Scale Rerun

Status: completed, but superseded for paper reporting by
`../paper_scale_reviewed/`.

Configuration:

- Models: UR10, LEAP hand left, Stanford TidyBot Gen3.
- Backends: TetraPGA, Pinocchio, CasADi.
- Samples: 20 per model.
- Horizon: 50.
- Max iterations: 25.
- Budgets: 1, 2, 5, 10, 20, 50, 100, 200 ms.

Outputs:

- `fixed_budget_paper_summary.csv`: merged summary.
- `ur10/ur10_{samples,runs,trace,summary}.csv`.
- `leap_hand/leap_hand_{samples,runs,trace,summary}.csv`.
- `stanford_tidybot/stanford_tidybot_{samples,runs,trace,summary}.csv`.
- `ur10.log`, `leap_hand.log`, `stanford_tidybot.log`.

Key observations:

- UR10 reaches full success at 5 ms for TetraPGA and Pinocchio, and at 20 ms
  for CasADi.
- LEAP hand reaches full success at 10 ms for TetraPGA, 50 ms for Pinocchio,
  and 100 ms for CasADi.
- Stanford TidyBot is not fully saturated under the 25-iteration cap; use its
  cost/error-vs-budget curves rather than only the success-rate threshold.

Review note:

- This run used `max_iterations=25`, which truncated Stanford TidyBot before
  the larger wall-clock budgets could be evaluated. Use
  `../paper_scale_reviewed/fixed_budget_paper_reviewed_summary.csv` for paper
  tables and plots.
