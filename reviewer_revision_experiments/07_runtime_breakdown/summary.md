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
