# Reviewer Revision Experiment Index

Root: `/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments`

## 00_casadi_feasibility
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

CSV previews:

`00_casadi_feasibility/casadi_oom20_10min_rnea_so_summary.csv`
| case | meaning | memory_policy | per_dof_time_policy | last_success_dof | first_failure_dof | first_failure_status | last_success_peak_rss_mb | notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| rnea_so | RNEA second-order CasADi graph for component-level ID-SO baseline | RLIMIT_AS=20GiB; no RSS kill | max 600s per DoF; no failure within 600s is treated as pass | 402 | 403 | exit_2 | 18338.676 | Under the 10-minute per-DoF memory-bottleneck rule, 402 DoF did not fail within 600s and is treated as pass; 403 DoF failed with bad_alloc/exit_2 at 455.436s. |

`00_casadi_feasibility/casadi_oom20_memonly_summary.csv`
| case | meaning | memory_policy | last_success_dof | first_failure_dof | first_failure_status | last_success_peak_rss_mb | last_success_graph_build_s | notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aba_fo | ABA first-order CasADi graph used by forward-dynamics OCP baseline | RLIMIT_AS=20GiB; no RSS kill; no timeout | 410 | 411 | exit_2 | 20330.105 | 359.968209537 | 410 DoF succeeded under the 20GiB address-space cap; 411 DoF failed with bad_alloc/exit_2. This run deliberately disables the 19.5GiB RSS protection line and wall-clock timeout. |
| rnea_fo | RNEA first-order CasADi graph used by inverse-dynamics OCP baseline | RLIMIT_AS=20GiB; no RSS kill; no timeout | 417 | 418 | exit_2 | 20421.199 | 166.434337003 | 417 DoF succeeded under the 20GiB address-space cap; 418 DoF failed with bad_alloc/exit_2. This run deliberately disables the 19.5GiB RSS protection line and wall-clock timeout. |
| rnea_so | RNEA second-order CasADi graph for component-level ID-SO baseline | RLIMIT_AS=20GiB; no RSS kill; no timeout | 72 |  |  | 293.246 | 701.724794744 | 72 DoF completed under the 20GiB address-space cap with no timeout and no RSS kill; no OOM boundary was reached in this second-order check, so the observed limitation remains graph-construction time. |

`00_casadi_feasibility/casadi_oom20_summary.csv`
| case | meaning | memory_policy | last_success_dof | first_failure_dof | first_failure_status | last_success_peak_rss_mb | last_success_graph_build_s | notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| aba_fo | ABA first-order CasADi graph used by forward-dynamics OCP baseline | RLIMIT_AS=20GB; RSS kill=19.5GiB | 407 | 408 | rss_limit | 19927.918 | 345.651891835 | 407 DoF succeeded; 408 DoF exceeded the protected RSS threshold. |
| rnea_fo | RNEA first-order CasADi graph used by inverse-dynamics OCP baseline | RLIMIT_AS=20GB; RSS kill=19.5GiB | 413 | 414 | rss_limit | 19843.461 | 157.139863698 | 413 DoF succeeded; 414 DoF exceeded the protected RSS threshold. |
| rnea_so | RNEA second-order CasADi graph for component-level ID-SO baseline | RLIMIT_AS=20GB; RSS kill=19.5GiB | 72 |  |  | 306.938 | 705.624140086 | 72 DoF completed without memory pressure but took about 706 s; this case is construction-time limited at this DoF. |

## 01_model_dynamics_three_models
# Three-Model Dynamics Consistency

Status: completed.

All three target models passed `TetraPGA_models_test`, which compares TetraPGA
inverse dynamics against Pinocchio RNEA and checks ABA recovery from the same
torque sample.

Models:

- UR10: 6 DoF fixed-base manipulator.
- LEAP hand left: 16 DoF articulated hand.
- Stanford TidyBot Gen3: 10 DoF mobile manipulator model, using two
  prismatic base coordinates, one base-yaw coordinate, and a 7 DoF arm.

Use in paper/response: these results support using the three models for the
revision MPC/OCP experiments.

CSV previews:

`01_model_dynamics_three_models/model_dynamics_summary.csv`
| model | urdf | dof | test | status | log |
| --- | --- | --- | --- | --- | --- |
| ur10 | /home/chenwh/ros2_ws/src/robot-assets/ur10/urdf/ur10.urdf | 6 | TetraPGA_models_test | pass | ur10.log |
| leap_hand_left | /home/chenwh/ros2_ws/src/robot-assets/leap_hand/urdf/leap_hand_left.urdf | 16 | TetraPGA_models_test | pass | leap_hand_left.log |
| stanford_tidybot | /home/chenwh/ros2_ws/src/robot-assets/stanford_tidybot/urdf/tidybot_gen3_10dof.urdf | 10 | TetraPGA_models_test | pass | stanford_tidybot.log |

Logs:
- `01_model_dynamics_three_models/leap_hand_left.log`
- `01_model_dynamics_three_models/stanford_tidybot.log`
- `01_model_dynamics_three_models/ur10.log`

## 02_ocp_fixed_budget
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

CSV previews:

`02_ocp_fixed_budget/fixed_budget_pilot_summary.csv`
| scenario | method | budget_ms | num_samples | mean_best_cost | median_best_cost | mean_terminal_q_error | success_rate | mean_iterations | mean_iter_ms | p95_iter_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ur10 | TetraPGA | 1 | 3 | 425.057391828 | 79.648851739 | 0.522217388 | 0 | 1.333333333 | 0.3916855 | 0.4708266 |
| ur10 | TetraPGA | 5 | 3 | 17.516699008 | 4.606607697 | 0.013614886 | 0.666666667 | 4.333333333 | 0.589007692 | 1.6702996 |
| ur10 | TetraPGA | 10 | 3 | 5.63331039 | 4.606607697 | 0.008034758 | 0.666666667 | 5.333333333 | 0.648408375 | 1.55206075 |
| ur10 | TetraPGA | 20 | 3 | 5.63331039 | 4.606607697 | 0.008034758 | 0.666666667 | 5.333333333 | 0.648408375 | 1.55206075 |
| ur10 | TetraPGA | 50 | 3 | 5.63331039 | 4.606607697 | 0.008034758 | 0.666666667 | 5.333333333 | 0.648408375 | 1.55206075 |
| ur10 | Pinocchio | 1 | 3 | 408.684338111 | 79.648851729 | 0.520997837 | 0.333333333 | 2.333333333 | 0.230626286 | 0.3994155 |
| ur10 | Pinocchio | 5 | 3 | 383.670256767 | 4.606607697 | 0.507638742 | 0.666666667 | 3.333333333 | 0.2858123 | 0.4865702 |

`02_ocp_fixed_budget/paper_scale/fixed_budget_paper_summary.csv`
| scenario | method | budget_ms | num_samples | mean_best_cost | median_best_cost | mean_terminal_q_error | success_rate | mean_iterations | mean_iter_ms | p95_iter_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ur10 | TetraPGA | 1 | 20 | 552.803543125 | 401.423430684 | 0.756310081 | 0 | 1 | 0.72430285 | 0.8113659 |
| ur10 | TetraPGA | 2 | 20 | 62.44902662 | 3.279417673 | 0.066195283 | 0.7 | 2.95 | 0.588552644 | 0.8088216 |
| ur10 | TetraPGA | 5 | 20 | 2.545452343 | 2.627758855 | 0.001677919 | 1 | 5.3 | 0.552181858 | 0.7614885 |
| ur10 | TetraPGA | 10 | 20 | 2.545452343 | 2.627758855 | 0.001677919 | 1 | 5.3 | 0.552181858 | 0.7614885 |
| ur10 | TetraPGA | 20 | 20 | 2.545452343 | 2.627758855 | 0.001677919 | 1 | 5.3 | 0.552181858 | 0.7614885 |
| ur10 | TetraPGA | 50 | 20 | 2.545452343 | 2.627758855 | 0.001677919 | 1 | 5.3 | 0.552181858 | 0.7614885 |
| ur10 | TetraPGA | 100 | 20 | 2.545452343 | 2.627758855 | 0.001677919 | 1 | 5.3 | 0.552181858 | 0.7614885 |

Logs:
- `02_ocp_fixed_budget/leap_hand.log`
- `02_ocp_fixed_budget/stanford_tidybot.log`
- `02_ocp_fixed_budget/ur10.log`

## 03_ur10_obstacle_sensitivity
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

CSV previews:

`03_ur10_obstacle_sensitivity/ur10_margin_sweep_pilot_summary.csv`
| d_safe | obstacle_count | samples | solver_success_rate | collision_free_rate | safety_satisfied_rate | mean_solve_ms | mean_final_min_distance | mean_traj_min_distance | mean_safety_violations | mean_collision_violations | mean_placement_error | mean_jerk_rms | mean_torque_rate_rms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.03 | 4 | 3 | 0.0 | 0.0 | 0.0 | 35.95577566666667 | 0.02037537633333333 | -0.03708782633333333 | 25.666666666666668 | 17.0 | 0.25852859366666664 | 79683.035570699 | 3810.0980919906665 |
| 0.03 | 8 | 3 | 0.0 | 0.0 | 0.0 | 36.255932666666666 | 0.007620718333333333 | -0.07277098433333334 | 82.33333333333333 | 52.666666666666664 | 0.3792207816666667 | 40672.675699605 | 3558.16022071 |
| 0.05 | 4 | 3 | 0.0 | 0.0 | 0.0 | 33.84204666666667 | 0.03238631333333333 | -0.031799955 | 36.333333333333336 | 15.333333333333334 | 0.229370713 | 74544.537134892 | 3195.8497771276666 |
| 0.05 | 8 | 3 | 0.0 | 0.0 | 0.0 | 35.43728566666667 | 0.012624592333333334 | -0.07493305466666667 | 98.33333333333333 | 54.0 | 0.3777007143333333 | 41048.774395693334 | 3313.015844243 |
| 0.08 | 4 | 3 | 0.0 | 0.3333333333333333 | 0.0 | 34.743314 | 0.031416952 | -0.027379212333333333 | 53.0 | 15.0 | 0.22154027533333334 | 83506.54714619534 | 3393.111043760333 |
| 0.08 | 8 | 3 | 0.0 | 0.0 | 0.0 | 35.173845 | 0.007445022000000002 | -0.08968662733333332 | 121.0 | 59.333333333333336 | 0.3737888483333333 | 53751.274449434335 | 4986.5538331 |
| 0.1 | 4 | 3 | 0.0 | 0.0 | 0.0 | 35.587992666666665 | 0.030653131666666666 | -0.032633939 | 67.0 | 15.666666666666666 | 0.218044083 | 98269.977715468 | 4551.7147598 |

`03_ur10_obstacle_sensitivity/paper_scale/ur10_margin_sweep_paper_summary.csv`
| d_safe | obstacle_count | num_samples | solver_success_rate | failed_rate | collision_free_rate | safety_satisfied_rate | mean_solve_ms | median_solve_ms | mean_initial_min_distance | mean_final_min_distance | mean_trajectory_min_distance | min_trajectory_min_distance | mean_safety_violation_count | mean_collision_violation_count | mean_placement_error_norm | median_placement_error_norm | mean_path_length | mean_jerk_rms | mean_torque_rate_rms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.03 | 2 | 20 | 0.05 | 0.0 | 0.8 | 0.8 | 95.63214855 | 96.7128245 | 0.3074848274 | 0.0813600585 | 0.0696313795 | -0.085575137 | 6.0 | 4.45 | 0.5601179364 | 0.534037686 | 2.9100270755 | 36278.22626950055 | 8367.9118629093 |
| 0.03 | 4 | 20 | 0.15 | 0.0 | 0.8 | 0.65 | 93.8438937 | 98.6960715 | 0.28696971155 | 0.1025799104 | 0.0662540005 | -0.073791784 | 3.9 | 2.25 | 0.7202693468 | 0.6373526555 | 3.39685465425 | 36941.37804168195 | 10415.14091777495 |
| 0.03 | 8 | 20 | 0.15 | 0.0 | 0.5 | 0.3 | 102.23768485000001 | 104.090384 | 0.2567928917 | 0.058315995649999994 | -0.00586224595 | -0.088412124 | 21.4 | 11.15 | 0.770152562 | 0.6138087345000001 | 3.5960471863000003 | 39591.5136048724 | 10802.419459001449 |
| 0.03 | 16 | 20 | 0.35 | 0.0 | 0.25 | 0.1 | 105.41970054999999 | 111.1945695 | 0.22834877969999998 | 0.01001904255 | -0.03459528525 | -0.098442197 | 45.6 | 23.5 | 1.03573818725 | 0.8942790709999999 | 4.03785616465 | 24930.54933675955 | 12773.56593999725 |
| 0.05 | 2 | 20 | 0.05 | 0.0 | 0.85 | 0.65 | 94.4717761 | 95.60862900000001 | 0.3074848274 | 0.09763599 | 0.08293379105000001 | -0.083235884 | 8.2 | 3.95 | 0.5449324243 | 0.5227974265 | 2.8701874060000003 | 37732.8894694274 | 8465.1576410658 |
| 0.05 | 4 | 20 | 0.25 | 0.0 | 0.8 | 0.5 | 95.8335558 | 96.647389 | 0.28696971155 | 0.10738268740000001 | 0.07252632005000001 | -0.068715459 | 6.45 | 2.0 | 0.6980863085 | 0.6004059020000001 | 3.3710300752999998 | 36105.47122266775 | 10981.3283760266 |
| 0.05 | 8 | 20 | 0.25 | 0.0 | 0.4 | 0.25 | 97.61168649999999 | 102.426804 | 0.2567928917 | 0.0628640704 | -0.007641148150000001 | -0.083019765 | 26.8 | 8.8 | 0.7884667317 | 0.5962491525 | 3.5796691619999996 | 35495.550947183954 | 11859.297846582149 |

Logs:
- `03_ur10_obstacle_sensitivity/ur10_margin_sweep_pilot.log`

## 04_closed_loop_mpc_metrics
# Closed-Loop MPC Reference Metrics

Status: generated by `scripts/run_reference_closed_loop_metrics.sh`.

Configuration:

- Simulator: MuJoCo through `ga_ocp_ros2/scripts/joint_command_executor.py`.
- Controller: GA-OCP closed-loop receding-horizon nominal-model MPC.
- Cases: no plant perturbation, sinusoidal reference tracking.
- Running OCP cost includes state tracking, control effort, soft velocity limits, and acceleration regularization.
- Solver uses box-FDDP with hard control bounds from each robot's effort limits.
- Robots: leap_left, stanford_tidybot, ur.
- Backends: casadi, pinocchio, tetrapga.
- Duration per case: 20 s.
- Viewer enabled during this run: false.

Outputs:

- `reference_batch_20260628_231346/combined_summary.csv`: GA4Ro-like per-case summary.
- `mujoco_closed_loop_metrics_summary.csv`: stable combined summary with jerk/smoothness columns.
- `mujoco_closed_loop_metrics_with_conditions.csv`: same rows plus robot/backend display labels.
- `reference_batch_20260628_231346/*_cycles.csv`: raw closed-loop cycle logs.

Smoothness metrics:

- `accel_rms_norm` is the primary smoothness metric, computed from the planned first-step acceleration `ddq_cmd` when available.
- `plant_accel_rms_norm` is retained as a plant-velocity finite-difference diagnostic.

CSV previews:

`04_closed_loop_mpc_metrics/mujoco_closed_loop_metrics_summary.csv`
| case_name | summary_file | robot | backend | num_cycles | tracking_rmse | tracking_mean | tracking_p95 | torque_ratio_mean | torque_ratio_p95 | torque_ratio_max | command_torque_ratio_mean | command_torque_ratio_p95 | command_torque_ratio_max | solve_time_mean_ms | solve_time_p95_ms | reference_build_mean_ms | reference_build_p95_ms | problem_build_mean_ms | problem_build_p95_ms | warm_start_mean_ms | warm_start_p95_ms | initial_calc_mean_ms | initial_calc_p95_ms | solver_setup_mean_ms | solver_setup_p95_ms | mpc_pipeline_mean_ms | mpc_pipeline_p95_ms | publish_command_mean_ms | publish_command_p95_ms | solver_dam_calc_mean_ms | solver_dam_calc_p95_ms | solver_dam_calcdiff_mean_ms | solver_dam_calcdiff_p95_ms | solver_dynamics_calc_mean_ms | solver_dynamics_calc_p95_ms | solver_dynamics_calcdiff_mean_ms | solver_dynamics_calcdiff_p95_ms | solver_cost_sum_calc_mean_ms | solver_cost_sum_calc_p95_ms | solver_cost_sum_calcdiff_mean_ms | solver_cost_sum_calcdiff_p95_ms | solver_cost_item_total_mean_ms | solver_cost_item_total_p95_ms | solver_collision_cost_total_mean_ms | solver_collision_cost_total_p95_ms | solver_collision_residual_total_mean_ms | solver_collision_residual_total_p95_ms | solver_model_total_mean_ms | solver_model_total_p95_ms | solver_overhead_mean_ms | solver_overhead_p95_ms | initial_model_total_mean_ms | initial_model_total_p95_ms | enable_collision_cost | collision_obstacle_count | collision_weight | collision_safety_distance | acceleration_weight | realtime_ratio_mean | deadline_miss_rate | failure_rate | dt | horizon | solve_budget_ms | control_rate_hz | experiment_duration_s | plant_mass_scale | plant_payload_mass | controller_payload_mass | model_payload | plant_payload_com | controller_payload_com | payload_com_attachment | external_force_body_name | external_force_start_s | external_force_duration_s | external_force | external_torque | cycles_file | dt_estimate_s | velocity_rmse | accel_rms_norm | accel_energy_mean | cmd_accel_rms_norm | cmd_accel_energy_mean | plant_accel_rms_norm | plant_accel_energy_mean | jerk_rms_norm_from_dq | jerk_rms_norm_from_q | torque_rate_rms_norm | effort_rate_rms_norm | robot_display | backend_display |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| reference_leap_casadi | reference_leap_casadi_summary.csv | leap_left | casadi | 1000 | 0.168107994 | 0.165131372 | 0.211748251 | 0.085728417 | 0.119084625 | 0.163756944 | 0.022199916 | 0.023552497 | 0.024770109 | 9.086831413 | 10.823395 | 0.021895055 | 0.051821 | 1.143874124 | 1.959742 | 0.00401653 | 0.007456 | 0.145509886 | 0.273726 | 0.081593227 | 0.139527 | 10.731508172 | 12.99119 | 0.026209216 | 0.040391 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 9.086831413 | 10.823395 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.454341571 | 0 | 0 | 0.02 | 20 | 8 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_leap_casadi_cycles.csv | 0.020000109 | 0.51386696 | 30.7402119 | 944.96063 | 30.7402119 | 944.96063 | 27.9650988 | 782.046749 | 1781.31516 | 1635.45914 | 0.157441828 | 2.41553386 | Leap Hand | CasADi |
| reference_leap_pinocchio | reference_leap_pinocchio_summary.csv | leap_left | pinocchio | 1000 | 0.167710766 | 0.164706972 | 0.209917119 | 0.085711451 | 0.119477414 | 0.160867019 | 0.022280426 | 0.023553564 | 0.024764676 | 8.736700106 | 9.856715 | 0.022507756 | 0.048552 | 1.220032832 | 2.273596 | 0.00460195 | 0.008774 | 0.177874186 | 0.333718 | 0.091359532 | 0.174761 | 10.455973266 | 12.612879 | 0.026933596 | 0.048539 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 8.736700106 | 9.856715 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.436835005 | 0 | 0 | 0.02 | 20 | 8 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_leap_pinocchio_cycles.csv | 0.020000127 | 0.511838351 | 30.6314008 | 938.282713 | 30.6314008 | 938.282713 | 27.7445194 | 769.758357 | 1768.84702 | 1648.65787 | 0.1478944 | 2.43069716 | Leap Hand | Pinocchio |
| reference_leap_tetrapga | reference_leap_tetrapga_summary.csv | leap_left | tetrapga | 1000 | 0.177207974 | 0.175540599 | 0.213631758 | 0.08403317 | 0.112648931 | 0.143103746 | 0.000502845 | 0.001996543 | 0.02198635 | 8.562029052 | 9.333822 | 0.026364609 | 0.069206 | 1.459911213 | 2.777271 | 0.004853358 | 0.009274 | 0.161186319 | 0.317917 | 0.098415522 | 0.178269 | 10.613320371 | 12.638466 | 0.030076653 | 0.05425 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1.058941736 | 1.168586 | 0 | 0 | 0 | 0 | 0 | 0 | 8.562029052 | 9.333822 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.428101453 | 0 | 0 | 0.02 | 20 | 8 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_leap_tetrapga_cycles.csv | 0.019999918 | 0.61919437 | 35.9355762 | 1291.36564 | 35.9355762 | 1291.36564 | 33.9394143 | 1151.88385 | 2186.984 | 2161.40899 | 0.210300713 | 3.18749553 | Leap Hand | GA |
| reference_tidybot_casadi | reference_tidybot_casadi_summary.csv | stanford_tidybot | casadi | 1000 | 0.108727917 | 0.105862901 | 0.137078614 | 0.459836801 | 1.050621893 | 2.546813016 | 0.342140214 | 0.403588226 | 0.466254684 | 10.626527026 | 11.29969 | 0.010571876 | 0.028326 | 0.683925673 | 1.303304 | 0.002602618 | 0.005286 | 0.081606881 | 0.169536 | 0.040536135 | 0.07944 | 11.622277615 | 12.787728 | 0.024586468 | 0.039481 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 10.626527026 | 11.29969 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.531326351 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_tidybot_casadi_cycles.csv | 0.020000116 | 0.0909213546 | 10.4684613 | 109.588683 | 10.4684613 | 109.588683 | 4.93186623 | 24.3233045 | 425.75329 | 262.307085 | 238.816171 | 46119.9676 | TidyBot | CasADi |
| reference_tidybot_pinocchio | reference_tidybot_pinocchio_summary.csv | stanford_tidybot | pinocchio | 1000 | 0.108812629 | 0.105892923 | 0.136583153 | 0.429638918 | 0.787550192 | 2.303253085 | 0.341862248 | 0.406755264 | 0.465960828 | 10.255686567 | 10.651463 | 0.011100175 | 0.027572 | 0.654856217 | 1.291986 | 0.002876764 | 0.005773 | 0.096079525 | 0.19705 | 0.043032389 | 0.08534 | 11.208195414 | 12.251603 | 0.02467047 | 0.040656 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 10.255686567 | 10.651463 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.512784328 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_tidybot_pinocchio_cycles.csv | 0.020000039 | 0.0915953421 | 10.4763453 | 109.753811 | 10.4763453 | 109.753811 | 4.95298047 | 24.5320156 | 426.694934 | 258.958861 | 237.662791 | 40175.728 | TidyBot | Pinocchio |
| reference_tidybot_tetrapga | reference_tidybot_tetrapga_summary.csv | stanford_tidybot | tetrapga | 1000 | 0.115532934 | 0.113556721 | 0.137387823 | 0.489796392 | 0.965690839 | 3.401238447 | 0.036926491 | 0.103458731 | 0.143622911 | 10.2273201 | 10.468184 | 0.014703402 | 0.040143 | 0.595697264 | 1.139047 | 0.002666351 | 0.005477 | 0.078236244 | 0.158881 | 0.043227712 | 0.083396 | 11.140336127 | 11.998259 | 0.024676716 | 0.040337 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1.154447704 | 1.208014 | 0 | 0 | 0 | 0 | 0 | 0 | 10.2273201 | 10.468184 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.511366005 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_tidybot_tetrapga_cycles.csv | 0.019999882 | 0.106456165 | 11.7811568 | 138.795655 | 11.7811568 | 138.795655 | 6.19903111 | 38.4279867 | 522.209514 | 330.396517 | 274.894644 | 48346.467 | TidyBot | GA |
| reference_ur_casadi | reference_ur_casadi_summary.csv | ur | casadi | 2500 | 0.167949347 | 0.163811931 | 0.216809627 | 0.14789696 | 1 | 1 | 0.137881803 | 0.252660966 | 0.378528068 | 4.20355943 | 4.377615 | 0.009505216 | 0.030446 | 0.66350781 | 0.861195 | 0.002598758 | 0.003432 | 0.072647612 | 0.08335 | 0.030313442 | 0.03629 | 5.191018465 | 5.595227 | 0.014961072 | 0.023087 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4.20355943 | 4.377615 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.525444929 | 0 | 0 | 0.008 | 40 | 4 | 125 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_ur_casadi_cycles.csv | 0.00799999 | 0.204561154 | 53.602437 | 2873.22125 | 53.602437 | 2873.22125 | 13.8135272 | 190.813533 | 2595.30081 | 1724.18558 | 1574.865 | 3230.28734 | UR10 | CasADi |

`04_closed_loop_mpc_metrics/mujoco_closed_loop_reference_combined_summary.csv`
| case_name | summary_file | robot | backend | num_cycles | tracking_rmse | tracking_mean | tracking_p95 | torque_ratio_mean | torque_ratio_p95 | torque_ratio_max | command_torque_ratio_mean | command_torque_ratio_p95 | command_torque_ratio_max | solve_time_mean_ms | solve_time_p95_ms | reference_build_mean_ms | reference_build_p95_ms | problem_build_mean_ms | problem_build_p95_ms | warm_start_mean_ms | warm_start_p95_ms | initial_calc_mean_ms | initial_calc_p95_ms | solver_setup_mean_ms | solver_setup_p95_ms | mpc_pipeline_mean_ms | mpc_pipeline_p95_ms | publish_command_mean_ms | publish_command_p95_ms | solver_dam_calc_mean_ms | solver_dam_calc_p95_ms | solver_dam_calcdiff_mean_ms | solver_dam_calcdiff_p95_ms | solver_dynamics_calc_mean_ms | solver_dynamics_calc_p95_ms | solver_dynamics_calcdiff_mean_ms | solver_dynamics_calcdiff_p95_ms | solver_cost_sum_calc_mean_ms | solver_cost_sum_calc_p95_ms | solver_cost_sum_calcdiff_mean_ms | solver_cost_sum_calcdiff_p95_ms | solver_cost_item_total_mean_ms | solver_cost_item_total_p95_ms | solver_collision_cost_total_mean_ms | solver_collision_cost_total_p95_ms | solver_collision_residual_total_mean_ms | solver_collision_residual_total_p95_ms | solver_model_total_mean_ms | solver_model_total_p95_ms | solver_overhead_mean_ms | solver_overhead_p95_ms | initial_model_total_mean_ms | initial_model_total_p95_ms | enable_collision_cost | collision_obstacle_count | collision_weight | collision_safety_distance | acceleration_weight | realtime_ratio_mean | deadline_miss_rate | failure_rate | dt | horizon | solve_budget_ms | control_rate_hz | experiment_duration_s | plant_mass_scale | plant_payload_mass | controller_payload_mass | model_payload | plant_payload_com | controller_payload_com | payload_com_attachment | external_force_body_name | external_force_start_s | external_force_duration_s | external_force | external_torque | cycles_file | dt_estimate_s | velocity_rmse | accel_rms_norm | accel_energy_mean | cmd_accel_rms_norm | cmd_accel_energy_mean | plant_accel_rms_norm | plant_accel_energy_mean | jerk_rms_norm_from_dq | jerk_rms_norm_from_q | torque_rate_rms_norm | effort_rate_rms_norm | robot_display | backend_display |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| reference_leap_casadi | reference_leap_casadi_summary.csv | leap_left | casadi | 1000 | 0.168107994 | 0.165131372 | 0.211748251 | 0.085728417 | 0.119084625 | 0.163756944 | 0.022199916 | 0.023552497 | 0.024770109 | 9.086831413 | 10.823395 | 0.021895055 | 0.051821 | 1.143874124 | 1.959742 | 0.00401653 | 0.007456 | 0.145509886 | 0.273726 | 0.081593227 | 0.139527 | 10.731508172 | 12.99119 | 0.026209216 | 0.040391 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 9.086831413 | 10.823395 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.454341571 | 0 | 0 | 0.02 | 20 | 8 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_leap_casadi_cycles.csv | 0.020000109 | 0.51386696 | 30.7402119 | 944.96063 | 30.7402119 | 944.96063 | 27.9650988 | 782.046749 | 1781.31516 | 1635.45914 | 0.157441828 | 2.41553386 | Leap Hand | CasADi |
| reference_leap_pinocchio | reference_leap_pinocchio_summary.csv | leap_left | pinocchio | 1000 | 0.167710766 | 0.164706972 | 0.209917119 | 0.085711451 | 0.119477414 | 0.160867019 | 0.022280426 | 0.023553564 | 0.024764676 | 8.736700106 | 9.856715 | 0.022507756 | 0.048552 | 1.220032832 | 2.273596 | 0.00460195 | 0.008774 | 0.177874186 | 0.333718 | 0.091359532 | 0.174761 | 10.455973266 | 12.612879 | 0.026933596 | 0.048539 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 8.736700106 | 9.856715 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.436835005 | 0 | 0 | 0.02 | 20 | 8 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_leap_pinocchio_cycles.csv | 0.020000127 | 0.511838351 | 30.6314008 | 938.282713 | 30.6314008 | 938.282713 | 27.7445194 | 769.758357 | 1768.84702 | 1648.65787 | 0.1478944 | 2.43069716 | Leap Hand | Pinocchio |
| reference_leap_tetrapga | reference_leap_tetrapga_summary.csv | leap_left | tetrapga | 1000 | 0.177207974 | 0.175540599 | 0.213631758 | 0.08403317 | 0.112648931 | 0.143103746 | 0.000502845 | 0.001996543 | 0.02198635 | 8.562029052 | 9.333822 | 0.026364609 | 0.069206 | 1.459911213 | 2.777271 | 0.004853358 | 0.009274 | 0.161186319 | 0.317917 | 0.098415522 | 0.178269 | 10.613320371 | 12.638466 | 0.030076653 | 0.05425 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1.058941736 | 1.168586 | 0 | 0 | 0 | 0 | 0 | 0 | 8.562029052 | 9.333822 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.428101453 | 0 | 0 | 0.02 | 20 | 8 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_leap_tetrapga_cycles.csv | 0.019999918 | 0.61919437 | 35.9355762 | 1291.36564 | 35.9355762 | 1291.36564 | 33.9394143 | 1151.88385 | 2186.984 | 2161.40899 | 0.210300713 | 3.18749553 | Leap Hand | GA |
| reference_tidybot_casadi | reference_tidybot_casadi_summary.csv | stanford_tidybot | casadi | 1000 | 0.108727917 | 0.105862901 | 0.137078614 | 0.459836801 | 1.050621893 | 2.546813016 | 0.342140214 | 0.403588226 | 0.466254684 | 10.626527026 | 11.29969 | 0.010571876 | 0.028326 | 0.683925673 | 1.303304 | 0.002602618 | 0.005286 | 0.081606881 | 0.169536 | 0.040536135 | 0.07944 | 11.622277615 | 12.787728 | 0.024586468 | 0.039481 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 10.626527026 | 11.29969 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.531326351 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_tidybot_casadi_cycles.csv | 0.020000116 | 0.0909213546 | 10.4684613 | 109.588683 | 10.4684613 | 109.588683 | 4.93186623 | 24.3233045 | 425.75329 | 262.307085 | 238.816171 | 46119.9676 | TidyBot | CasADi |
| reference_tidybot_pinocchio | reference_tidybot_pinocchio_summary.csv | stanford_tidybot | pinocchio | 1000 | 0.108812629 | 0.105892923 | 0.136583153 | 0.429638918 | 0.787550192 | 2.303253085 | 0.341862248 | 0.406755264 | 0.465960828 | 10.255686567 | 10.651463 | 0.011100175 | 0.027572 | 0.654856217 | 1.291986 | 0.002876764 | 0.005773 | 0.096079525 | 0.19705 | 0.043032389 | 0.08534 | 11.208195414 | 12.251603 | 0.02467047 | 0.040656 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 10.255686567 | 10.651463 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.512784328 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_tidybot_pinocchio_cycles.csv | 0.020000039 | 0.0915953421 | 10.4763453 | 109.753811 | 10.4763453 | 109.753811 | 4.95298047 | 24.5320156 | 426.694934 | 258.958861 | 237.662791 | 40175.728 | TidyBot | Pinocchio |
| reference_tidybot_tetrapga | reference_tidybot_tetrapga_summary.csv | stanford_tidybot | tetrapga | 1000 | 0.115532934 | 0.113556721 | 0.137387823 | 0.489796392 | 0.965690839 | 3.401238447 | 0.036926491 | 0.103458731 | 0.143622911 | 10.2273201 | 10.468184 | 0.014703402 | 0.040143 | 0.595697264 | 1.139047 | 0.002666351 | 0.005477 | 0.078236244 | 0.158881 | 0.043227712 | 0.083396 | 11.140336127 | 11.998259 | 0.024676716 | 0.040337 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 1.154447704 | 1.208014 | 0 | 0 | 0 | 0 | 0 | 0 | 10.2273201 | 10.468184 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.511366005 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_tidybot_tetrapga_cycles.csv | 0.019999882 | 0.106456165 | 11.7811568 | 138.795655 | 11.7811568 | 138.795655 | 6.19903111 | 38.4279867 | 522.209514 | 330.396517 | 274.894644 | 48346.467 | TidyBot | GA |
| reference_ur_casadi | reference_ur_casadi_summary.csv | ur | casadi | 2500 | 0.167949347 | 0.163811931 | 0.216809627 | 0.14789696 | 1 | 1 | 0.137881803 | 0.252660966 | 0.378528068 | 4.20355943 | 4.377615 | 0.009505216 | 0.030446 | 0.66350781 | 0.861195 | 0.002598758 | 0.003432 | 0.072647612 | 0.08335 | 0.030313442 | 0.03629 | 5.191018465 | 5.595227 | 0.014961072 | 0.023087 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4.20355943 | 4.377615 | 0 | 0 | 0 | 4 | 50 | 0.08 | 0.000001 | 0.525444929 | 0 | 0 | 0.008 | 40 | 4 | 125 | 20 | 1 | 0 | 0 | 0 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 | reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_231346/reference_ur_casadi_cycles.csv | 0.00799999 | 0.204561154 | 53.602437 | 2873.22125 | 53.602437 | 2873.22125 | 13.8135272 | 190.813533 | 2595.30081 | 1724.18558 | 1574.865 | 3230.28734 | UR10 | CasADi |

## 05_robustness_expanded_sweep
# Supplemental Numerical Robustness Rollout

Status: completed.

Configuration:

- Model: UR10.
- Controller: receding-horizon nominal-model TetraPGA FDDP.
- Execution mode: offline numerical rollout using the GA-OCP/TetraPGA dynamics
  stack, not MuJoCo.
- Samples: 20.
- Horizon: 50.
- MPC iterations per control step: 25.
- Target perturbation amplitude: 0.25 rad.
- Bounded-tracking success threshold: terminal RMSE <= 0.30 rad.
- Perturbations:
  - mass/inertia scaling: -30%, -20%, -10%, +10%, +20%, +30%.
  - end-body COM offset: 1 cm, 2 cm, 5 cm, random direction.
  - external disturbance: single-step EE wrench impulse at 5 N and 10 N
    levels, scaled by one controller time step.

Outputs:

- `paper_scale/ur10_robustness_sweep.csv`: 240 raw rows.
- `paper_scale/ur10_robustness_sweep_summary.csv`: grouped summary.
- `paper_scale/tuning/`: pilot runs used to choose the closed-loop setup and
  final bounded-tracking threshold.

Key observations:

- There are no solver failures and no non-finite rollouts.
- Nominal tracking reaches 100% bounded-tracking success with mean terminal
  RMSE 0.051 rad.
- Mass/inertia mismatch remains robust for +/-20%, while the +/-30% cases show
  reduced success or increased RMSE.
- External impulse disturbances at 5 N and 10 N remain bounded with 100%
  success; terminal RMSE increases from 0.087 rad to 0.152 rad.
- End-body COM offset is the most sensitive perturbation: success drops from
  0.30 at 1 cm / 2 cm to 0.20 at 5 cm, with a monotonic RMSE increase.

Use this directory only as supplementary numerical evidence. The reviewer-facing
closed-loop simulation evidence is generated separately by the MuJoCo
receding-horizon experiments under `06_mujoco_robustness_expanded_sweep`.

CSV previews:

`05_robustness_expanded_sweep/paper_scale/ur10_robustness_sweep_summary.csv`
| perturbation | level | num_samples | solver_failed_rate | solver_converged_rate | rollout_finite_rate | success_rate | mean_terminal_rmse | median_terminal_rmse | p95_terminal_rmse | mean_trajectory_rmse | mean_planning_terminal_rmse | mean_solve_ms | mean_max_tau_ratio | p95_max_abs_q | p95_max_abs_dq |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| com_offset | 0.01 | 20 | 0 | 1 | 1 | 0.3 | 0.500250133 | 0.464711571 | 0.873794311 | 0.437597686 | 0.000233882 | 1.803460202 | 0.384256332 | 2.311211069 | 7.210961918 |
| com_offset | 0.02 | 20 | 0 | 1 | 1 | 0.3 | 0.55218039 | 0.499721914 | 0.932270495 | 0.513095755 | 0.000233882 | 1.776378566 | 0.384293437 | 2.704975594 | 13.156647047 |
| com_offset | 0.05 | 20 | 0 | 1 | 1 | 0.2 | 0.598734708 | 0.56873402 | 1.050028629 | 0.584941594 | 0.000233882 | 1.776907687 | 0.384393582 | 3.325528711 | 18.788713714 |
| external_force | 5 | 20 | 0 | 1 | 1 | 1 | 0.086740498 | 0.086365115 | 0.128155427 | 0.135463973 | 0.000233882 | 1.8573978 | 0.3842572 | 0.488933679 | 5.584292435 |
| external_force | 10 | 20 | 0 | 1 | 1 | 1 | 0.151818386 | 0.159977343 | 0.244858799 | 0.17284766 | 0.000233882 | 1.857764081 | 0.384259205 | 0.85951947 | 10.994726813 |
| mass_inertia_scale | -0.3 | 20 | 0 | 1 | 1 | 0.85 | 0.256256499 | 0.234329626 | 0.377737577 | 0.197015598 | 0.000233882 | 1.867890683 | 0.359602712 | 0.571687278 | 0.889326731 |
| mass_inertia_scale | -0.2 | 20 | 0 | 1 | 1 | 1 | 0.164180682 | 0.158109078 | 0.232017243 | 0.156498702 | 0.000233882 | 1.923592165 | 0.363803061 | 0.466714713 | 0.72364985 |

## 06_mujoco_robustness_expanded_sweep
# MuJoCo Robustness Expanded Sweep

Status: generated by `scripts/run_mujoco_closed_loop_sweep.sh`.

Configuration:

- Simulator: MuJoCo through `ga_ocp_ros2/scripts/joint_command_executor.py`.
- Controller: GA-OCP closed-loop receding-horizon nominal-model MPC.
- Robot: UR10.
- Backends: tetrapga.
- Duration per case: 20 s.
- MPC dt: 0.02 s.
- MPC horizon: 20.
- Control rate: 50.0 Hz.
- Solve budget: 10.0 ms.
- Enforce solve budget: false.
- Viewer enabled during this run: false.

Outputs:

- `paper_scale/batch_20260627_124744/combined_summary.csv`: per-case tracking/runtime summary.
- `paper_scale/batch_20260627_124744/closed_loop_metrics_summary.csv`: tracking, acceleration,
  jerk, torque-rate, and effort-rate metrics from cycle CSVs.
- `paper_scale/batch_20260627_124744/*_cycles.csv`: raw closed-loop cycle logs.

CSV previews:

`06_mujoco_robustness_expanded_sweep/paper_scale/mujoco_robustness_combined_summary.csv`
| case_name | summary_file | robot | backend | num_cycles | tracking_rmse | tracking_mean | tracking_p95 | torque_ratio_mean | torque_ratio_p95 | torque_ratio_max | solve_time_mean_ms | solve_time_p95_ms | reference_build_mean_ms | reference_build_p95_ms | problem_build_mean_ms | problem_build_p95_ms | warm_start_mean_ms | warm_start_p95_ms | initial_calc_mean_ms | initial_calc_p95_ms | solver_setup_mean_ms | solver_setup_p95_ms | mpc_pipeline_mean_ms | mpc_pipeline_p95_ms | publish_command_mean_ms | publish_command_p95_ms | solver_dam_calc_mean_ms | solver_dam_calc_p95_ms | solver_dam_calcdiff_mean_ms | solver_dam_calcdiff_p95_ms | solver_dynamics_calc_mean_ms | solver_dynamics_calc_p95_ms | solver_dynamics_calcdiff_mean_ms | solver_dynamics_calcdiff_p95_ms | solver_cost_sum_calc_mean_ms | solver_cost_sum_calc_p95_ms | solver_cost_sum_calcdiff_mean_ms | solver_cost_sum_calcdiff_p95_ms | solver_cost_item_total_mean_ms | solver_cost_item_total_p95_ms | solver_collision_cost_total_mean_ms | solver_collision_cost_total_p95_ms | solver_collision_residual_total_mean_ms | solver_collision_residual_total_p95_ms | solver_model_total_mean_ms | solver_model_total_p95_ms | solver_overhead_mean_ms | solver_overhead_p95_ms | initial_model_total_mean_ms | initial_model_total_p95_ms | enable_collision_cost | collision_obstacle_count | collision_weight | collision_safety_distance | realtime_ratio_mean | deadline_miss_rate | failure_rate | dt | horizon | solve_budget_ms | control_rate_hz | experiment_duration_s | plant_mass_scale | plant_payload_mass | controller_payload_mass | model_payload | plant_payload_com | controller_payload_com | payload_com_attachment | external_force_body_name | external_force_start_s | external_force_duration_s | external_force | external_torque |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| com_01cm_x_tetrapga | com_01cm_x_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.165105383 | 0.159034153 | 0.227713389 | 0.143871563 | 0.29591746 | 1 | 3.519758518 | 10.396133 | 0.013765018 | 0.025264 | 0.78536048 | 1.329231 | 0.005201731 | 0.008949 | 0.110931429 | 0.181475 | 0.055188242 | 0.095302 | 4.731388114 | 11.255059 | 0.044708498 | 0.078181 | 1.677090213 | 7.062429 | 0.996920035 | 1.770844 | 1.393292508 | 5.91271 | 0.835738727 | 1.490119 | 0.23517274 | 0.981941 | 0.136470143 | 0.243898 | 0.186271283 | 0.624739 | 0 | 0 | 0 | 0 | 2.674010248 | 8.75569 | 0.84574827 | 1.65703 | 0.104364904 | 0.171159 | 0 | 4 | 50 | 0.08 | 0.175987926 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0.01 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_01cm_y_tetrapga | com_01cm_y_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.16327892 | 0.157280433 | 0.225876792 | 0.143792027 | 0.295175227 | 1 | 3.175016294 | 10.345998 | 0.012240112 | 0.026086 | 0.675303045 | 1.283258 | 0.004158659 | 0.007801 | 0.090609118 | 0.181405 | 0.045160816 | 0.087002 | 4.20103086 | 10.963968 | 0.038379108 | 0.072662 | 1.585391662 | 7.049904 | 0.855435064 | 1.68994 | 1.317616325 | 5.886042 | 0.714809762 | 1.422449 | 0.221882849 | 0.975712 | 0.119415522 | 0.230214 | 0.172534918 | 0.622154 | 0 | 0 | 0 | 0 | 2.440826726 | 8.724039 | 0.734189568 | 1.619312 | 0.085038577 | 0.171456 | 0 | 4 | 50 | 0.08 | 0.158750815 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0 0.01 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_01cm_z_tetrapga | com_01cm_z_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.164416895 | 0.158286602 | 0.227726844 | 0.143623799 | 0.293672441 | 1 | 3.387239017 | 10.413497 | 0.01634804 | 0.031103 | 0.753887518 | 1.307685 | 0.004855451 | 0.008549 | 0.102846119 | 0.178872 | 0.052269145 | 0.089073 | 4.538648005 | 11.180102 | 0.042991916 | 0.078041 | 1.662609079 | 7.086007 | 0.936818322 | 1.73521 | 1.384108557 | 5.909124 | 0.785350745 | 1.45572 | 0.230294135 | 0.977501 | 0.128467208 | 0.237166 | 0.180546913 | 0.622904 | 0 | 0 | 0 | 0 | 2.599427401 | 8.772102 | 0.787811616 | 1.629947 | 0.096675083 | 0.169129 | 0 | 4 | 50 | 0.08 | 0.169361951 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0 0 0.060000000000000005 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_02cm_x_tetrapga | com_02cm_x_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.164171426 | 0.158172681 | 0.227066118 | 0.145339379 | 0.29545916 | 1 | 3.249844513 | 10.376948 | 0.012841716 | 0.027876 | 0.750605299 | 1.414001 | 0.004637279 | 0.008841 | 0.097086119 | 0.189068 | 0.049305052 | 0.092679 | 4.382386796 | 11.015736 | 0.043253825 | 0.084131 | 1.599990925 | 7.075246 | 0.888589183 | 1.696448 | 1.328186544 | 5.90201 | 0.740550373 | 1.421122 | 0.225613211 | 0.977162 | 0.125799667 | 0.243358 | 0.176936335 | 0.623952 | 0 | 0 | 0 | 0 | 2.488580108 | 8.743004 | 0.761264405 | 1.6371 | 0.091054947 | 0.177184 | 0 | 4 | 50 | 0.08 | 0.162492226 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0.02 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_02cm_y_tetrapga | com_02cm_y_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.164408709 | 0.158447059 | 0.227129761 | 0.14559577 | 0.298115184 | 1 | 3.265316343 | 10.387931 | 0.012885079 | 0.026551 | 0.762784622 | 1.34034 | 0.004699178 | 0.008338 | 0.100652095 | 0.184233 | 0.050660475 | 0.092272 | 4.41914435 | 11.173789 | 0.043539403 | 0.078151 | 1.58290929 | 7.063308 | 0.909386939 | 1.729276 | 1.31461956 | 5.877984 | 0.759940777 | 1.451272 | 0.222475561 | 0.974193 | 0.126587895 | 0.238355 | 0.175275245 | 0.623591 | 0 | 0 | 0 | 0 | 2.492296229 | 8.750253 | 0.773020114 | 1.646214 | 0.094377508 | 0.173071 | 0 | 4 | 50 | 0.08 | 0.163265817 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0 0.02 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_02cm_z_tetrapga | com_02cm_z_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.163727223 | 0.15769137 | 0.226562703 | 0.141219711 | 0.289028023 | 1 | 3.407543488 | 10.363656 | 0.016625206 | 0.032881 | 0.761962933 | 1.461885 | 0.004715845 | 0.008913 | 0.100035931 | 0.185375 | 0.050039046 | 0.091239 | 4.567915985 | 11.015582 | 0.042765605 | 0.079001 | 1.678510845 | 7.075291 | 0.937737693 | 1.718772 | 1.392928903 | 5.905456 | 0.777352635 | 1.432161 | 0.236817634 | 0.981311 | 0.136686893 | 0.25415 | 0.191341663 | 0.637645 | 0 | 0 | 0 | 0 | 2.616248538 | 8.751686 | 0.79129495 | 1.628767 | 0.093816823 | 0.175312 | 0 | 4 | 50 | 0.08 | 0.170377174 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0 0 0.070000000000000007 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_05cm_x_tetrapga | com_05cm_x_tetrapga_summary.csv | ur | tetrapga | 1000 | 0.165512409 | 0.159493823 | 0.228821763 | 0.146870534 | 0.30861473 | 1 | 3.573169821 | 10.378297 | 0.015213184 | 0.029453 | 0.858825263 | 1.462386 | 0.005380972 | 0.009047 | 0.116183568 | 0.195452 | 0.056962127 | 0.094352 | 4.876538343 | 11.218301 | 0.046856633 | 0.080356 | 1.697622187 | 7.051439 | 1.016176427 | 1.754793 | 1.408298113 | 5.888096 | 0.847200804 | 1.46263 | 0.240221821 | 0.979192 | 0.143967509 | 0.254418 | 0.193029823 | 0.626095 | 0 | 0 | 0 | 0 | 2.713798614 | 8.734596 | 0.859371207 | 1.650554 | 0.109032277 | 0.183775 | 0 | 4 | 50 | 0.08 | 0.178658491 | 0 | 0 | 0.02 | 20 | 10 | 50 | 20 | 1 | 5 | 5 | 1 | 0.050000000000000003 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |

`06_mujoco_robustness_expanded_sweep/paper_scale/mujoco_robustness_metrics_summary.csv`
| case_name | cycles_file | summary_file | num_cycles | dt_estimate_s | tracking_rmse | tracking_mean | tracking_p95 | velocity_rmse | solve_time_mean_ms | solve_time_p95_ms | torque_ratio_mean | torque_ratio_p95 | failure_rate | accel_rms_norm | jerk_rms_norm_from_dq | jerk_rms_norm_from_q | torque_rate_rms_norm | effort_rate_rms_norm | robot | backend | plant_mass_scale | plant_payload_mass | controller_payload_mass | model_payload | plant_payload_com | controller_payload_com | payload_com_attachment | external_force_body_name | external_force_start_s | external_force_duration_s | external_force | external_torque |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| com_01cm_x_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_01cm_x_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_01cm_x_tetrapga_summary.csv | 1000 | 0.020000405 | 0.165105383 | 0.159034153 | 0.227716218 | 0.158983685 | 3.51975852 | 10.3961894 | 0.143871563 | 0.295959947 | 0 | 3.22168329 | 264.016353 | 166.723644 | 767.45859 | 1446.307 | ur | tetrapga | 1 | 5 | 5 | 1 | 0.01 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_01cm_y_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_01cm_y_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_01cm_y_tetrapga_summary.csv | 1000 | 0.019999895 | 0.16327892 | 0.157280433 | 0.225880038 | 0.159380704 | 3.17501629 | 10.3460619 | 0.143792027 | 0.295198441 | 0 | 3.42894643 | 282.976769 | 163.322004 | 741.652001 | 1383.03671 | ur | tetrapga | 1 | 5 | 5 | 1 | 0 0.01 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_01cm_z_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_01cm_z_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_01cm_z_tetrapga_summary.csv | 1000 | 0.020000095 | 0.164416895 | 0.158286602 | 0.227749673 | 0.159447544 | 3.38723902 | 10.4135927 | 0.143623799 | 0.293679886 | 0 | 3.67132645 | 310.659784 | 165.363176 | 761.00464 | 1450.87298 | ur | tetrapga | 1 | 5 | 5 | 1 | 0 0 0.060000000000000005 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_02cm_x_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_02cm_x_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_02cm_x_tetrapga_summary.csv | 1000 | 0.020000161 | 0.164171426 | 0.158172681 | 0.227069214 | 0.15854375 | 3.24984451 | 10.3769918 | 0.145339379 | 0.295476124 | 0 | 3.19579622 | 261.916272 | 159.931063 | 762.849108 | 1453.01018 | ur | tetrapga | 1 | 5 | 5 | 1 | 0.02 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_02cm_y_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_02cm_y_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_02cm_y_tetrapga_summary.csv | 1000 | 0.020000354 | 0.164408709 | 0.158447059 | 0.227129779 | 0.160462042 | 3.26531634 | 10.3881273 | 0.14559577 | 0.298122082 | 0 | 3.53841347 | 284.806698 | 159.784642 | 756.355994 | 1442.18083 | ur | tetrapga | 1 | 5 | 5 | 1 | 0 0.02 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_02cm_z_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_02cm_z_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_02cm_z_tetrapga_summary.csv | 1000 | 0.019999789 | 0.163727223 | 0.15769137 | 0.22658648 | 0.15810811 | 3.40754349 | 10.3638292 | 0.141219711 | 0.289045779 | 0 | 3.38764227 | 283.999677 | 161.630746 | 744.21261 | 1384.52024 | ur | tetrapga | 1 | 5 | 5 | 1 | 0 0 0.070000000000000007 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |
| com_05cm_x_tetrapga | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_05cm_x_tetrapga_cycles.csv | reviewer_revision_experiments/06_mujoco_robustness_expanded_sweep/paper_scale/batch_20260627_124744/com_05cm_x_tetrapga_summary.csv | 1000 | 0.020000084 | 0.165512409 | 0.159493823 | 0.228830151 | 0.158975146 | 3.57316982 | 10.378384 | 0.146870534 | 0.308652722 | 0 | 3.04235045 | 253.364916 | 159.61737 | 763.841546 | 1449.17853 | ur | tetrapga | 1 | 5 | 5 | 1 | 0.050000000000000003 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0.050000000000000003 | wrist_3_link | -1 | 0 | 0 0 0 | 0 0 0 |

`06_mujoco_robustness_expanded_sweep/paper_scale/mujoco_robustness_paper_summary.csv`
| case_name | perturbation | level | direction | num_cycles | tracking_rmse | tracking_p95 | velocity_rmse | solve_time_mean_ms | solve_time_p95_ms | deadline_miss_rate | failure_rate | torque_ratio_mean | torque_ratio_p95 | accel_rms_norm | jerk_rms_norm_from_dq | torque_rate_rms_norm | plant_mass_scale | plant_payload_mass | controller_payload_mass | model_payload | plant_payload_com | controller_payload_com | external_force | external_force_duration_s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| com_01cm_x_tetrapga | payload_com_offset | 01cm | x | 1000 | 0.165105383 | 0.227713389 | 0.158983685 | 3.519758518 | 10.396133 | 0 | 0 | 0.143871563 | 0.29591746 | 3.22168329 | 264.016353 | 767.45859 | 1 | 5 | 5 | 1 | 0.01 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0 | 0 |
| com_01cm_y_tetrapga | payload_com_offset | 01cm | y | 1000 | 0.16327892 | 0.225876792 | 0.159380704 | 3.175016294 | 10.345998 | 0 | 0 | 0.143792027 | 0.295175227 | 3.42894643 | 282.976769 | 741.652001 | 1 | 5 | 5 | 1 | 0 0.01 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0 | 0 |
| com_01cm_z_tetrapga | payload_com_offset | 01cm | z | 1000 | 0.164416895 | 0.227726844 | 0.159447544 | 3.387239017 | 10.413497 | 0 | 0 | 0.143623799 | 0.293672441 | 3.67132645 | 310.659784 | 761.00464 | 1 | 5 | 5 | 1 | 0 0 0.060000000000000005 | 0 0 0.050000000000000003 | 0 0 0 | 0 |
| com_02cm_x_tetrapga | payload_com_offset | 02cm | x | 1000 | 0.164171426 | 0.227066118 | 0.15854375 | 3.249844513 | 10.376948 | 0 | 0 | 0.145339379 | 0.29545916 | 3.19579622 | 261.916272 | 762.849108 | 1 | 5 | 5 | 1 | 0.02 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0 | 0 |
| com_02cm_y_tetrapga | payload_com_offset | 02cm | y | 1000 | 0.164408709 | 0.227129761 | 0.160462042 | 3.265316343 | 10.387931 | 0 | 0 | 0.14559577 | 0.298115184 | 3.53841347 | 284.806698 | 756.355994 | 1 | 5 | 5 | 1 | 0 0.02 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0 | 0 |
| com_02cm_z_tetrapga | payload_com_offset | 02cm | z | 1000 | 0.163727223 | 0.226562703 | 0.15810811 | 3.407543488 | 10.363656 | 0 | 0 | 0.141219711 | 0.289028023 | 3.38764227 | 283.999677 | 744.21261 | 1 | 5 | 5 | 1 | 0 0 0.070000000000000007 | 0 0 0.050000000000000003 | 0 0 0 | 0 |
| com_05cm_x_tetrapga | payload_com_offset | 05cm | x | 1000 | 0.165512409 | 0.228821763 | 0.158975146 | 3.573169821 | 10.378297 | 0 | 0 | 0.146870534 | 0.30861473 | 3.04235045 | 253.364916 | 763.841546 | 1 | 5 | 5 | 1 | 0.050000000000000003 0 0.050000000000000003 | 0 0 0.050000000000000003 | 0 0 0 | 0 |

## 07_runtime_breakdown
# Offline Runtime Breakdown

Status: completed from random point-to-point FDDP tasks, not closed-loop ROS/MuJoCo MPC.

Outputs:

- `paper_scale/runtime_breakdown_stack_summary.csv`: one row per robot/backend, ready for stacked bar plots.
- `paper_scale/runtime_breakdown_summary.csv`: detailed per robot/backend summary.
- `paper_scale/runtime_breakdown_combined_summary.csv`: compatibility copy of the detailed summary.
- `paper_scale/runtime_breakdown_raw.csv`: one row per random sample solve.
- `paper_scale/runtime_breakdown_iterations.csv`: per solver callback/iteration interval timing.
- `paper_scale/batch_20260627_offline_runtime`: full batch copy.

Configuration:

- Robots: `ur10`, `leap_hand`, `unitree_g1`
- Backends: `TetraPGA`, `Pinocchio`, `CasADi`
- Samples: 24 per robot/backend
- Horizon: 50
- dt: 0.02
- Max iterations: 25
- Seed: 20260627
- Collision cost: disabled for all cases

Instrumentation:

- `DAM.calc` and `DAM.calcDiff` are timed inside the Crocoddyl solver loop by a benchmark-only differential action wrapper.
- Cost-model residuals are separately timed for state, control, and acceleration regularization.
- Solver overhead is computed as `solver total - DAM.calc - DAM.calcDiff`, covering backward pass, linear algebra, line search, regularization, and other solver-side work.
- The runtime target is compiled as Release and explicitly defines `NDEBUG` with `-O3`.

Per-Iteration Stacked-Bar Inputs:

| robot | backend | solve total ms | non-cost model ms | state cost ms | control cost ms | acc cost ms | solver overhead ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| ur10 | TetraPGA | 0.501 | 0.308 | 0.008 | 0.004 | 0.038 | 0.142 |
| ur10 | Pinocchio | 0.525 | 0.312 | 0.018 | 0.005 | 0.037 | 0.153 |
| ur10 | CasADi | 2.054 | 1.842 | 0.018 | 0.005 | 0.037 | 0.153 |
| leap_hand | TetraPGA | 2.236 | 0.973 | 0.022 | 0.007 | 0.230 | 1.004 |
| leap_hand | Pinocchio | 2.295 | 0.989 | 0.041 | 0.007 | 0.223 | 1.036 |
| leap_hand | CasADi | 5.347 | 4.041 | 0.040 | 0.009 | 0.218 | 1.039 |
| unitree_g1 | TetraPGA | 8.956 | 2.626 | 0.073 | 0.016 | 1.187 | 5.054 |
| unitree_g1 | Pinocchio | 9.189 | 2.784 | 0.083 | 0.018 | 1.176 | 5.129 |
| unitree_g1 | CasADi | 14.982 | 8.590 | 0.087 | 0.020 | 1.159 | 5.127 |

All nine robot/backend cases converged on all 24 samples. Collision timing columns are zero as expected.

CSV previews:

`07_runtime_breakdown/paper_scale/runtime_breakdown_combined_summary.csv`
| robot | backend | dof | samples | valid_samples | horizon | dt | max_iterations | success_rate | solver_iterations_mean | solve_total_mean_ms | solve_total_p95_ms | dam_calc_mean_ms | dam_calcdiff_mean_ms | dam_total_mean_ms | cost_state_calc_mean_ms | cost_state_calcdiff_mean_ms | cost_state_total_mean_ms | cost_control_calc_mean_ms | cost_control_calcdiff_mean_ms | cost_control_total_mean_ms | cost_acc_calc_mean_ms | cost_acc_calcdiff_mean_ms | cost_acc_total_mean_ms | cost_collision_calc_mean_ms | cost_collision_calcdiff_mean_ms | cost_collision_total_mean_ms | cost_other_calc_mean_ms | cost_other_calcdiff_mean_ms | cost_other_total_mean_ms | cost_total_mean_ms | non_cost_model_mean_ms | solver_overhead_mean_ms | stack_total_mean_ms | dam_calc_per_iter_mean_ms | dam_calcdiff_per_iter_mean_ms | dam_total_per_iter_mean_ms | cost_state_calc_per_iter_mean_ms | cost_state_calcdiff_per_iter_mean_ms | cost_state_total_per_iter_mean_ms | cost_control_calc_per_iter_mean_ms | cost_control_calcdiff_per_iter_mean_ms | cost_control_total_per_iter_mean_ms | cost_acc_calc_per_iter_mean_ms | cost_acc_calcdiff_per_iter_mean_ms | cost_acc_total_per_iter_mean_ms | cost_collision_calc_per_iter_mean_ms | cost_collision_calcdiff_per_iter_mean_ms | cost_collision_total_per_iter_mean_ms | cost_other_calc_per_iter_mean_ms | cost_other_calcdiff_per_iter_mean_ms | cost_other_total_per_iter_mean_ms | cost_total_per_iter_mean_ms | non_cost_model_per_iter_mean_ms | solver_overhead_per_iter_mean_ms | stack_total_per_iter_mean_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| leap_hand | CasADi | 16 | 24 | 24 | 50 | 0.02 | 25 | 1 | 8.625 | 45.969413833 | 62.8921459 | 3.6004685 | 33.440573458 | 37.041041958 | 0.093371 | 0.251416958 | 0.344787958 | 0.045899875 | 0.033647417 | 0.079547292 | 0.063612625 | 1.809827958 | 1.873440583 | 0 | 0 | 0 | 0 | 0 | 0 | 2.297775833 | 34.743266125 | 8.928371875 | 45.969413833 | 0.406520029 | 3.90188345 | 4.308403478 | 0.010555997 | 0.029358827 | 0.039914824 | 0.005200959 | 0.003928676 | 0.009129634 | 0.00721971 | 0.211275742 | 0.218495452 | 0 | 0 | 0 | 0 | 0 | 0 | 0.26753991 | 4.040863568 | 1.038926323 | 5.347329802 |
| leap_hand | Pinocchio | 16 | 24 | 24 | 50 | 0.02 | 25 | 1 | 8.625 | 19.815109167 | 28.1116647 | 3.747221292 | 7.167491458 | 10.91471275 | 0.097726583 | 0.25492725 | 0.352653833 | 0.03988475 | 0.023269542 | 0.063154292 | 0.056766125 | 1.852377208 | 1.909143333 | 0 | 0 | 0 | 0 | 0 | 0 | 2.324951458 | 8.589761292 | 8.900396417 | 19.815109167 | 0.422939258 | 0.836645614 | 1.259584872 | 0.011046218 | 0.02976225 | 0.040808468 | 0.004513818 | 0.002713379 | 0.007227197 | 0.006438247 | 0.216169896 | 0.222608144 | 0 | 0 | 0 | 0 | 0 | 0 | 0.270643809 | 0.988941063 | 1.035516582 | 2.295101454 |
| leap_hand | TetraPGA | 16 | 24 | 24 | 50 | 0.02 | 25 | 1 | 8.625 | 19.270981083 | 27.00734395 | 3.108078167 | 7.541445333 | 10.6495235 | 0.047522583 | 0.140242042 | 0.187764625 | 0.035759417 | 0.024049583 | 0.059809 | 0.065019833 | 1.905565958 | 1.970585792 | 0 | 0 | 0 | 0 | 0 | 0 | 2.218159417 | 8.431364083 | 8.621457583 | 19.270981083 | 0.350945555 | 0.880589426 | 1.231534981 | 0.005394294 | 0.016413407 | 0.021807701 | 0.004055965 | 0.002813442 | 0.006869407 | 0.007423207 | 0.222473672 | 0.229896879 | 0 | 0 | 0 | 0 | 0 | 0 | 0.258573987 | 0.972960994 | 1.004163934 | 2.235698915 |
| unitree_g1 | CasADi | 29 | 24 | 24 | 50 | 0.02 | 25 | 1 | 9.875 | 147.683678542 | 179.011038 | 7.758818458 | 89.407821375 | 97.166639833 | 0.187632625 | 0.664930542 | 0.852563167 | 0.077432125 | 0.120666333 | 0.198098458 | 0.096042542 | 11.31876475 | 11.414807292 | 0 | 0 | 0 | 0 | 0 | 0 | 12.465468917 | 84.701170917 | 50.517038708 | 147.683678542 | 0.776872814 | 9.078544871 | 9.855417685 | 0.018835404 | 0.067697976 | 0.08653338 | 0.007801898 | 0.012284203 | 0.020086101 | 0.009655674 | 1.149462852 | 1.159118526 | 0 | 0 | 0 | 0 | 0 | 0 | 1.265738006 | 8.589679679 | 5.126865764 | 14.982283449 |
| unitree_g1 | Pinocchio | 29 | 24 | 24 | 50 | 0.02 | 25 | 1 | 9.875 | 90.663099625 | 110.9402352 | 8.001215208 | 32.112809708 | 40.114024917 | 0.182870083 | 0.63766675 | 0.820536833 | 0.070847458 | 0.105268292 | 0.17611575 | 0.079981583 | 11.503873333 | 11.583854917 | 0 | 0 | 0 | 0 | 0 | 0 | 12.5805075 | 27.533517417 | 50.549074708 | 90.663099625 | 0.800639159 | 3.259593751 | 4.06023291 | 0.018350515 | 0.064704806 | 0.083055321 | 0.007137294 | 0.010679615 | 0.017816909 | 0.008007115 | 1.167713758 | 1.175720873 | 0 | 0 | 0 | 0 | 0 | 0 | 1.276593103 | 2.783639807 | 5.128608227 | 9.188841137 |
| unitree_g1 | TetraPGA | 29 | 24 | 24 | 50 | 0.02 | 25 | 1 | 9.875 | 88.30751725 | 107.7444011 | 6.587727125 | 31.934789125 | 38.52251625 | 0.08297575 | 0.639382667 | 0.722358417 | 0.061610042 | 0.097219917 | 0.158829958 | 0.124455167 | 11.559656958 | 11.684112125 | 0 | 0 | 0 | 0 | 0 | 0 | 12.5653005 | 25.95721575 | 49.785001 | 88.30751725 | 0.659180594 | 3.243126318 | 3.902306912 | 0.008348382 | 0.064936973 | 0.073285354 | 0.006194158 | 0.009902928 | 0.016097086 | 0.012573672 | 1.173940509 | 1.186514182 | 0 | 0 | 0 | 0 | 0 | 0 | 1.275896622 | 2.62641029 | 5.053616891 | 8.955923803 |
| ur10 | CasADi | 6 | 24 | 24 | 50 | 0.02 | 25 | 1 | 3.958333333 | 8.1096875 | 9.5211456 | 0.495524375 | 7.011787792 | 7.507312167 | 0.018224708 | 0.051293333 | 0.069518042 | 0.0096715 | 0.011966958 | 0.021638458 | 0.013716208 | 0.131004333 | 0.144720542 | 0 | 0 | 0 | 0 | 0 | 0 | 0.235877042 | 7.271435125 | 0.602375333 | 8.1096875 | 0.125163462 | 1.776581506 | 1.901744967 | 0.004605458 | 0.01299216 | 0.017597617 | 0.002442592 | 0.003021085 | 0.005463676 | 0.003467181 | 0.033180351 | 0.036647533 | 0 | 0 | 0 | 0 | 0 | 0 | 0.059708826 | 1.842036141 | 0.152506649 | 2.054251616 |

`07_runtime_breakdown/paper_scale/runtime_breakdown_stack_summary.csv`
| robot | backend | dof | samples | horizon | dt | max_iterations | success_rate | solve_total_per_iter_mean_ms | non_cost_model_per_iter_mean_ms | cost_state_per_iter_mean_ms | cost_control_per_iter_mean_ms | cost_acc_per_iter_mean_ms | cost_collision_per_iter_mean_ms | cost_other_per_iter_mean_ms | solver_overhead_per_iter_mean_ms | stack_total_per_iter_mean_ms | dam_calc_per_iter_mean_ms | dam_calcdiff_per_iter_mean_ms | dam_total_per_iter_mean_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| leap_hand | CasADi | 16 | 24 | 50 | 0.02 | 25 | 1 | 5.347329802 | 4.040863568 | 0.039914824 | 0.009129634 | 0.218495452 | 0 | 0 | 1.038926323 | 5.347329802 | 0.406520029 | 3.90188345 | 4.308403478 |
| leap_hand | Pinocchio | 16 | 24 | 50 | 0.02 | 25 | 1 | 2.295101454 | 0.988941063 | 0.040808468 | 0.007227197 | 0.222608144 | 0 | 0 | 1.035516582 | 2.295101454 | 0.422939258 | 0.836645614 | 1.259584872 |
| leap_hand | TetraPGA | 16 | 24 | 50 | 0.02 | 25 | 1 | 2.235698915 | 0.972960994 | 0.021807701 | 0.006869407 | 0.229896879 | 0 | 0 | 1.004163934 | 2.235698915 | 0.350945555 | 0.880589426 | 1.231534981 |
| unitree_g1 | CasADi | 29 | 24 | 50 | 0.02 | 25 | 1 | 14.982283449 | 8.589679679 | 0.08653338 | 0.020086101 | 1.159118526 | 0 | 0 | 5.126865764 | 14.982283449 | 0.776872814 | 9.078544871 | 9.855417685 |
| unitree_g1 | Pinocchio | 29 | 24 | 50 | 0.02 | 25 | 1 | 9.188841137 | 2.783639807 | 0.083055321 | 0.017816909 | 1.175720873 | 0 | 0 | 5.128608227 | 9.188841137 | 0.800639159 | 3.259593751 | 4.06023291 |
| unitree_g1 | TetraPGA | 29 | 24 | 50 | 0.02 | 25 | 1 | 8.955923803 | 2.62641029 | 0.073285354 | 0.016097086 | 1.186514182 | 0 | 0 | 5.053616891 | 8.955923803 | 0.659180594 | 3.243126318 | 3.902306912 |
| ur10 | CasADi | 6 | 24 | 50 | 0.02 | 25 | 1 | 2.054251616 | 1.842036141 | 0.017597617 | 0.005463676 | 0.036647533 | 0 | 0 | 0.152506649 | 2.054251616 | 0.125163462 | 1.776581506 | 1.901744967 |

`07_runtime_breakdown/paper_scale/runtime_breakdown_summary.csv`
| robot | backend | dof | samples | valid_samples | horizon | dt | max_iterations | success_rate | solver_iterations_mean | solve_total_mean_ms | solve_total_p95_ms | dam_calc_mean_ms | dam_calcdiff_mean_ms | dam_total_mean_ms | cost_state_calc_mean_ms | cost_state_calcdiff_mean_ms | cost_state_total_mean_ms | cost_control_calc_mean_ms | cost_control_calcdiff_mean_ms | cost_control_total_mean_ms | cost_acc_calc_mean_ms | cost_acc_calcdiff_mean_ms | cost_acc_total_mean_ms | cost_collision_calc_mean_ms | cost_collision_calcdiff_mean_ms | cost_collision_total_mean_ms | cost_other_calc_mean_ms | cost_other_calcdiff_mean_ms | cost_other_total_mean_ms | cost_total_mean_ms | non_cost_model_mean_ms | solver_overhead_mean_ms | stack_total_mean_ms | dam_calc_per_iter_mean_ms | dam_calcdiff_per_iter_mean_ms | dam_total_per_iter_mean_ms | cost_state_calc_per_iter_mean_ms | cost_state_calcdiff_per_iter_mean_ms | cost_state_total_per_iter_mean_ms | cost_control_calc_per_iter_mean_ms | cost_control_calcdiff_per_iter_mean_ms | cost_control_total_per_iter_mean_ms | cost_acc_calc_per_iter_mean_ms | cost_acc_calcdiff_per_iter_mean_ms | cost_acc_total_per_iter_mean_ms | cost_collision_calc_per_iter_mean_ms | cost_collision_calcdiff_per_iter_mean_ms | cost_collision_total_per_iter_mean_ms | cost_other_calc_per_iter_mean_ms | cost_other_calcdiff_per_iter_mean_ms | cost_other_total_per_iter_mean_ms | cost_total_per_iter_mean_ms | non_cost_model_per_iter_mean_ms | solver_overhead_per_iter_mean_ms | stack_total_per_iter_mean_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| leap_hand | CasADi | 16 | 24 | 24 | 50 | 0.02 | 25 | 1 | 8.625 | 45.969413833 | 62.8921459 | 3.6004685 | 33.440573458 | 37.041041958 | 0.093371 | 0.251416958 | 0.344787958 | 0.045899875 | 0.033647417 | 0.079547292 | 0.063612625 | 1.809827958 | 1.873440583 | 0 | 0 | 0 | 0 | 0 | 0 | 2.297775833 | 34.743266125 | 8.928371875 | 45.969413833 | 0.406520029 | 3.90188345 | 4.308403478 | 0.010555997 | 0.029358827 | 0.039914824 | 0.005200959 | 0.003928676 | 0.009129634 | 0.00721971 | 0.211275742 | 0.218495452 | 0 | 0 | 0 | 0 | 0 | 0 | 0.26753991 | 4.040863568 | 1.038926323 | 5.347329802 |
| leap_hand | Pinocchio | 16 | 24 | 24 | 50 | 0.02 | 25 | 1 | 8.625 | 19.815109167 | 28.1116647 | 3.747221292 | 7.167491458 | 10.91471275 | 0.097726583 | 0.25492725 | 0.352653833 | 0.03988475 | 0.023269542 | 0.063154292 | 0.056766125 | 1.852377208 | 1.909143333 | 0 | 0 | 0 | 0 | 0 | 0 | 2.324951458 | 8.589761292 | 8.900396417 | 19.815109167 | 0.422939258 | 0.836645614 | 1.259584872 | 0.011046218 | 0.02976225 | 0.040808468 | 0.004513818 | 0.002713379 | 0.007227197 | 0.006438247 | 0.216169896 | 0.222608144 | 0 | 0 | 0 | 0 | 0 | 0 | 0.270643809 | 0.988941063 | 1.035516582 | 2.295101454 |
| leap_hand | TetraPGA | 16 | 24 | 24 | 50 | 0.02 | 25 | 1 | 8.625 | 19.270981083 | 27.00734395 | 3.108078167 | 7.541445333 | 10.6495235 | 0.047522583 | 0.140242042 | 0.187764625 | 0.035759417 | 0.024049583 | 0.059809 | 0.065019833 | 1.905565958 | 1.970585792 | 0 | 0 | 0 | 0 | 0 | 0 | 2.218159417 | 8.431364083 | 8.621457583 | 19.270981083 | 0.350945555 | 0.880589426 | 1.231534981 | 0.005394294 | 0.016413407 | 0.021807701 | 0.004055965 | 0.002813442 | 0.006869407 | 0.007423207 | 0.222473672 | 0.229896879 | 0 | 0 | 0 | 0 | 0 | 0 | 0.258573987 | 0.972960994 | 1.004163934 | 2.235698915 |
| unitree_g1 | CasADi | 29 | 24 | 24 | 50 | 0.02 | 25 | 1 | 9.875 | 147.683678542 | 179.011038 | 7.758818458 | 89.407821375 | 97.166639833 | 0.187632625 | 0.664930542 | 0.852563167 | 0.077432125 | 0.120666333 | 0.198098458 | 0.096042542 | 11.31876475 | 11.414807292 | 0 | 0 | 0 | 0 | 0 | 0 | 12.465468917 | 84.701170917 | 50.517038708 | 147.683678542 | 0.776872814 | 9.078544871 | 9.855417685 | 0.018835404 | 0.067697976 | 0.08653338 | 0.007801898 | 0.012284203 | 0.020086101 | 0.009655674 | 1.149462852 | 1.159118526 | 0 | 0 | 0 | 0 | 0 | 0 | 1.265738006 | 8.589679679 | 5.126865764 | 14.982283449 |
| unitree_g1 | Pinocchio | 29 | 24 | 24 | 50 | 0.02 | 25 | 1 | 9.875 | 90.663099625 | 110.9402352 | 8.001215208 | 32.112809708 | 40.114024917 | 0.182870083 | 0.63766675 | 0.820536833 | 0.070847458 | 0.105268292 | 0.17611575 | 0.079981583 | 11.503873333 | 11.583854917 | 0 | 0 | 0 | 0 | 0 | 0 | 12.5805075 | 27.533517417 | 50.549074708 | 90.663099625 | 0.800639159 | 3.259593751 | 4.06023291 | 0.018350515 | 0.064704806 | 0.083055321 | 0.007137294 | 0.010679615 | 0.017816909 | 0.008007115 | 1.167713758 | 1.175720873 | 0 | 0 | 0 | 0 | 0 | 0 | 1.276593103 | 2.783639807 | 5.128608227 | 9.188841137 |
| unitree_g1 | TetraPGA | 29 | 24 | 24 | 50 | 0.02 | 25 | 1 | 9.875 | 88.30751725 | 107.7444011 | 6.587727125 | 31.934789125 | 38.52251625 | 0.08297575 | 0.639382667 | 0.722358417 | 0.061610042 | 0.097219917 | 0.158829958 | 0.124455167 | 11.559656958 | 11.684112125 | 0 | 0 | 0 | 0 | 0 | 0 | 12.5653005 | 25.95721575 | 49.785001 | 88.30751725 | 0.659180594 | 3.243126318 | 3.902306912 | 0.008348382 | 0.064936973 | 0.073285354 | 0.006194158 | 0.009902928 | 0.016097086 | 0.012573672 | 1.173940509 | 1.186514182 | 0 | 0 | 0 | 0 | 0 | 0 | 1.275896622 | 2.62641029 | 5.053616891 | 8.955923803 |
| ur10 | CasADi | 6 | 24 | 24 | 50 | 0.02 | 25 | 1 | 3.958333333 | 8.1096875 | 9.5211456 | 0.495524375 | 7.011787792 | 7.507312167 | 0.018224708 | 0.051293333 | 0.069518042 | 0.0096715 | 0.011966958 | 0.021638458 | 0.013716208 | 0.131004333 | 0.144720542 | 0 | 0 | 0 | 0 | 0 | 0 | 0.235877042 | 7.271435125 | 0.602375333 | 8.1096875 | 0.125163462 | 1.776581506 | 1.901744967 | 0.004605458 | 0.01299216 | 0.017597617 | 0.002442592 | 0.003021085 | 0.005463676 | 0.003467181 | 0.033180351 | 0.036647533 | 0 | 0 | 0 | 0 | 0 | 0 | 0.059708826 | 1.842036141 | 0.152506649 | 2.054251616 |

## 08_casadi_dof_benchmark_rerun
_No summary.md yet._

## 09_operator_microbench
# Operator Microbenchmarks

Status: completed.

Configuration:

- Benchmark target: `TetraPGA_operator_bench`
- Raw CSV: `operator_bench_raw.csv`
- Repetitions: 7
- Fixed iterations per benchmark registration: 1,000,000
- Sample batch: 4,096 pre-generated rigid transforms, motors, twists, forces, points, and inertias
- CPU pin: 2
- Random interleaving: True
- Metric: median Google Benchmark CPU time per operator call, in ns

Win count:

- TetraPGA faster: 8 / 14
- Pinocchio faster: 6 / 14

Results:

| category | TetraPGA op | TetraPGA ns | Pinocchio op | Pinocchio ns | speedup | faster |
| --- | --- | ---: | --- | ---: | ---: | --- |
| adjoint_inverse_matrix | `ga_AdM_matrix` | 10.054 | `SE3_toActionMatrixInverse` | 19.886 | 1.98x | TetraPGA |
| adjoint_matrix | `ga_rbm_matrix` | 10.068 | `SE3_toActionMatrix` | 15.023 | 1.49x | TetraPGA |
| commutator_direct | `ga_com` | 3.808 | `Motion_cross` | 9.002 | 2.36x | TetraPGA |
| commutator_matrix | `ga_com_matrix` | 7.991 | `Motion_toActionMatrix` | 10.830 | 1.36x | TetraPGA |
| exp_map | `ga_exp` | 17.036 | `exp6` | 24.763 | 1.45x | TetraPGA |
| force_transform | `ga_rbm` | 13.937 | `SE3_act_Force` | 4.633 | 0.33x | Pinocchio |
| inertia_transform | `rbm_I_AdM` | 50.773 | `SE3_act_Inertia` | 7.700 | 0.15x | Pinocchio |
| log_map | `ga_log` | 9.363 | `log6` | 106.953 | 11.42x | TetraPGA |
| motion_inverse_transform | `ga_AdM` | 13.845 | `SE3_actInv_Motion` | 4.958 | 0.36x | Pinocchio |
| motion_transform_direct | `ga_rbm` | 13.983 | `SE3_act_Motion` | 4.384 | 0.31x | Pinocchio |
| motor_compose | `ga_mul` | 8.522 | `SE3_multiply` | 7.580 | 0.89x | Pinocchio |
| motor_inverse | `ga_rev` | 3.414 | `SE3_inverse` | 3.719 | 1.09x | TetraPGA |
| point_commutator | `pga_com23` | 3.560 | `Motion_point_velocity` | 3.582 | 1.01x | TetraPGA |
| point_transform | `pga_rbm3` | 7.647 | `SE3_act_Vector3` | 3.536 | 0.46x | Pinocchio |

Interpretation notes:

- `pinocchio_over_tetrapga_speedup > 1` means TetraPGA is faster for that operator pair.
- The transform rows compare direct fixed-size operator calls, not whole-model dynamics.
- `force_transform` uses the TetraPGA force-propagation kernel used in the current dynamics implementation and Pinocchio's `SE3.act(Force)`.
- `inertia_transform` compares the TetraPGA matrix expression used in the current dynamics implementation with Pinocchio's optimized `SE3.act(Inertia)` path.

Additional operator pairs worth mentioning in the reviewer response:

- Direct point transform: validates motor sandwiching for collision/contact point updates.
- Direct motion and force transforms: cover frame-change kernels used in RNEA/ABA passes.
- Commutator/cross product: covers velocity-product and Jacobian time-derivative terms.
- Motor/SE3 composition and inverse: covers kinematic chain propagation.
- Exp/log maps: cover retraction, integration, and local error coordinates used by IK/MPC.
- Inertia transform: covers rigid-body inertia rebase in dynamics.

CSV previews:

`09_operator_microbench/operator_bench_summary.csv`
| category | tetrapga_operator | pinocchio_operator | tetrapga_cpu_ns | pinocchio_cpu_ns | pinocchio_over_tetrapga_speedup | faster_backend |
| --- | --- | --- | --- | --- | --- | --- |
| adjoint_inverse_matrix | ga_AdM_matrix | SE3_toActionMatrixInverse | 10.053700 | 19.885700 | 1.977948 | TetraPGA |
| adjoint_matrix | ga_rbm_matrix | SE3_toActionMatrix | 10.067600 | 15.023200 | 1.492233 | TetraPGA |
| commutator_direct | ga_com | Motion_cross | 3.808070 | 9.002090 | 2.363951 | TetraPGA |
| commutator_matrix | ga_com_matrix | Motion_toActionMatrix | 7.991130 | 10.830000 | 1.355253 | TetraPGA |
| exp_map | ga_exp | exp6 | 17.035800 | 24.762600 | 1.453562 | TetraPGA |
| force_transform | ga_rbm | SE3_act_Force | 13.937000 | 4.632680 | 0.332402 | Pinocchio |
| inertia_transform | rbm_I_AdM | SE3_act_Inertia | 50.773500 | 7.699590 | 0.151646 | Pinocchio |
