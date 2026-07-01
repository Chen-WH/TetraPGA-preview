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
