# Stanford TidyBot Fixed-Budget Diagnosis

Status: completed.

Problem observed:

- The initial paper-scale fixed-budget run used `max_iterations=25`.
- In that run, Stanford TidyBot did not fully saturate by the 200 ms budget:
  TetraPGA reached 85%, Pinocchio reached 75%, and CasADi reached 75%.

Raw failure pattern at the end of the 25-iteration run:

- TetraPGA failed samples: 0, 6, 16.
- Pinocchio failed samples: 6, 7, 15, 16, 17.
- CasADi failed samples: 6, 7, 15, 16, 17.
- Common failures across all three backends: 6 and 16.

Diagnosis:

- The benchmark samples random full-state targets, and success is measured by
  the full 10-DoF configuration error norm `||q_T - q_target|| <= 1e-2`.
- Stanford TidyBot therefore solves a stricter mobile-base-plus-arm state
  regulation problem, not only an end-effector pose problem.
- The failing samples were still reducing error at iteration 25, so the
  largest time budgets were being truncated by `max_iterations`, not by the
  wall-clock budget itself.

Validation rerun:

- Same TidyBot seed and samples as the initial paper-scale run:
  `seed=1912086050`, `samples=20`, `horizon=50`.
- Increased only `max_iterations` from 25 to 100.
- Result: all methods reach 100% final success by the 200 ms budget.

Reviewed result:

- Use `../paper_scale_reviewed/fixed_budget_paper_reviewed_summary.csv` for
  paper tables and plots.
- Keep `../paper_scale/` as the trace of the initial run.

Conclusion:

- The Stanford TidyBot non-saturated curve was an iteration-cap artifact.
- It is not evidence of a TetraPGA model-loading issue, a floating-base support
  issue, or a Crocoddyl backend mismatch.
