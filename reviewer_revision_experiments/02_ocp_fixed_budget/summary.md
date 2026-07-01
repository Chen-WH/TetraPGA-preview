# Three-Model Fixed-Budget OCP

Status: pilot, initial paper-scale rerun, TidyBot diagnosis, and reviewed
paper-scale rerun completed.

Preferred paper-scale configuration:

- Models: UR10, LEAP hand left, Stanford TidyBot Gen3.
- Backends: TetraPGA, Pinocchio, CasADi.
- Samples: 20 per model.
- Horizon: 50.
- Max iterations: 100.
- Budgets: 1, 2, 5, 10, 20, 50, 100, 200 ms.

Preferred paper-scale outputs:

- `paper_scale_reviewed/fixed_budget_paper_reviewed_summary.csv`: merged
  summary across all three models.
- Per-model raw outputs are under `paper_scale_reviewed/ur10/`,
  `paper_scale_reviewed/leap_hand/`, and
  `paper_scale_reviewed/stanford_tidybot/`.
- Logs are `paper_scale_reviewed/ur10.log`, `paper_scale_reviewed/leap_hand.log`,
  and `paper_scale_reviewed/stanford_tidybot.log`.

Reviewed paper-scale observations:

- UR10 reaches full success at 5 ms for TetraPGA and Pinocchio, and at 20 ms
  for CasADi. CasADi reaches 95% at 10 ms.
- LEAP hand reaches 95% success at 5 ms and full success at 10 ms with
  TetraPGA. Pinocchio reaches 85% at 20 ms and full success at 50 ms. CasADi
  reaches 90% at 50 ms and full success at 100 ms.
- Stanford TidyBot is harder at small budgets, but the reviewed run removes
  the hidden iteration cap: TetraPGA reaches 95% at 50 ms and 100% at 100 ms;
  Pinocchio reaches 90% at 50 ms and 100% at 100 ms; CasADi reaches 90% at
  100 ms and 100% at 200 ms.

Diagnosis note:

- The initial `paper_scale/` fixed-budget run used `max_iterations=25`. For
  Stanford TidyBot, this cap truncated several trajectories before the largest
  time budgets were reached, producing a non-saturated curve. Re-running the
  same TidyBot samples with `max_iterations=100` yields full success by the
  200 ms budget, so this was an iteration-cap artifact rather than a model or
  floating-base dynamics failure.
- Keep `paper_scale/` for traceability, but use `paper_scale_reviewed/` for
  paper tables and plots.
- Detailed diagnosis is recorded in `review/tidybot_diagnosis.md`.

Pilot configuration:

- Models: UR10, LEAP hand left, Stanford TidyBot Gen3.
- Backends: TetraPGA, Pinocchio, CasADi.
- Samples: 3 per model.
- Horizon: 20.
- Max iterations: 10.
- Budgets: 1, 5, 10, 20, 50 ms.

Pilot outputs:

- `fixed_budget_pilot_summary.csv`: merged summary across all three models.
- Per-model raw outputs are under `ur10/`, `leap_hand/`, and
  `stanford_tidybot/`.

Notes:

- This is a pipeline validation run, not the final paper-scale statistic.
- TidyBot is now wired into the fixed-budget benchmark and writes the same CSV
  fields as UR10 and LEAP.
- The LEAP/TidyBot success rates remain zero under the pilot success tolerance
  and short iteration budget; use the cost/error curves and a larger run before
  drawing paper conclusions.
