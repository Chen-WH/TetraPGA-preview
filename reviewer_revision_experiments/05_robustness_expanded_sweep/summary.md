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
