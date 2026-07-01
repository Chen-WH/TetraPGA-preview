# Reviewer Experiment Output Review

This report checks CSV shape, failure flags, and the Stanford TidyBot
fixed-budget diagnosis after the paper-scale rerun.

| Check | Status | Detail |
| --- | --- | --- |
| CasADi feasibility rows | PASS | 3 rows |
| CasADi aba_fo RSS boundary | PASS | first_failure_dof=408 |
| CasADi rnea_fo RSS boundary | PASS | first_failure_dof=414 |
| CasADi mem-only rows | PASS | 3 rows |
| CasADi aba_fo mem-only boundary | PASS | last_success=410, first_failure=411 |
| CasADi rnea_fo mem-only boundary | PASS | last_success=417, first_failure=418 |
| CasADi rnea_so mem-only boundary | PASS | last_success=72, first_failure= |
| CasADi rnea_so 10min memory boundary | PASS | last_success=402, first_failure=403 |
| Three-model dynamics | PASS | 3 pass rows |
| Fixed-budget reviewed shape | PASS | 72 rows |
| Stanford TidyBot reviewed final success | PASS | TetraPGA=1, Pinocchio=1, CasADi=1 |
| TidyBot iteration-cap diagnosis | PASS | original=['25'], reviewed=['100'] |
| UR10 obstacle paper-scale shape | PASS | raw=400, grouped=20 |
| UR10 obstacle failed_rate | PASS | all grouped failed_rate values are 0 |
| UR10 obstacle rate bounds | PASS | all rates are in [0, 1] |
| UR10 robustness paper-scale shape | PASS | raw=240, grouped=12 |
| UR10 robustness solver_failed_rate | PASS | all grouped values are 0 |
| UR10 robustness finite rollout | PASS | all grouped values are 1 |
| UR10 robustness rate bounds | PASS | all rates are in [0, 1] |
| UR10 robustness nominal baseline | PASS | success=1, mean_rmse=0.05137391 |
| MuJoCo reference closed-loop shape | PASS | 9 rows |
| MuJoCo reference case coverage | PASS | reference_leap_casadi, reference_leap_pinocchio, reference_leap_tetrapga, reference_tidybot_casadi, reference_tidybot_pinocchio, reference_tidybot_tetrapga, reference_ur_casadi, reference_ur_pinocchio, reference_ur_tetrapga |
| MuJoCo reference robot coverage | PASS | leap_left, stanford_tidybot, ur |
| MuJoCo reference backend coverage | PASS | casadi, pinocchio, tetrapga |
| MuJoCo reference metric finiteness | PASS | tracking/smoothness columns finite |
| MuJoCo reference command torque hard bounds | PASS | max command torque ratio <= 1 |
| MuJoCo reference failure_rate | PASS | all rows have failure_rate=0 |
| MuJoCo robustness paper-scale shape | PASS | 22 rows |
| MuJoCo robustness perturbation coverage | PASS | external_force_impulse, mass_inertia_scale, nominal, payload_com_offset, payload_mass |
| MuJoCo robustness failure_rate | PASS | all cases have failure_rate=0 |
| MuJoCo robustness finite core metrics | PASS | tracking/runtime/jerk finite |
| Runtime breakdown case coverage | PASS | robots=leap_hand,unitree_g1,ur10; backends=CasADi,Pinocchio,TetraPGA |
| Runtime solver-internal finite timings | PASS | all required solver timing means finite |
| Runtime solver total closure | PASS | solve_total = dam_total + overhead; dam_total = cost_total + non_cost_model |
| Runtime solve-time p95 ordering | PASS | p95 >= mean > 0 |
| Runtime collision timing closure | PASS | collision_total = calc + calcdiff |
| Runtime breakdown success_rate | PASS | all rows have success_rate=1 |

## Stanford TidyBot Diagnosis

- The original paper-scale fixed-budget run used `max_iterations=25`.
- With the same seed and samples, `max_iterations=100` reaches 100% success
  at 200 ms for TetraPGA, Pinocchio, and CasADi.
- The non-saturated TidyBot curve was therefore an iteration-cap artifact,
  not evidence of a model-loading or floating-base dynamics failure.

## Preferred Fixed-Budget Data

Use `02_ocp_fixed_budget/paper_scale_reviewed/fixed_budget_paper_reviewed_summary.csv`
for paper tables and plots. Keep the older `paper_scale/` directory as
traceability for the initial rerun.

Overall status: PASS
