# Reviewer Revision Experiments

This directory stores reviewer-response experiments for the GA-OCP revision.
Each experiment owns one subdirectory with raw logs, CSV outputs, and a short
`summary.md`.

Planned experiment groups:

- `00_casadi_feasibility`: existing CasADi graph-construction feasibility and
  OOM summaries.
- `01_model_dynamics_three_models`: model loading and dynamics consistency for
  UR10, Stanford TidyBot, and LEAP hand.
- `02_ocp_fixed_budget`: fixed-budget OCP/MPC benchmarks for the three models.
- `03_ur10_obstacle_sensitivity`: UR10-only safety-margin sensitivity.
- `04_closed_loop_mpc_metrics`: closed-loop CSV post-processing for tracking,
  latency, jerk, and torque-rate metrics.
- `05_robustness_expanded_sweep`: supplemental offline numerical robustness
  rollout over mass/inertia, end-body COM, and external impulse perturbations.
- `06_mujoco_robustness_expanded_sweep`: true MuJoCo receding-horizon
  nominal-model MPC robustness sweep.
- `09_operator_microbench`: reviewer Major 1 operator-level TetraPGA vs
  Pinocchio microbenchmarks for frame transforms, adjoints, motor operations,
  commutators, exp/log, and inertia transforms.
- `summary`: generated cross-experiment index.

Run the summary collector from the repository root:

```bash
python3 reviewer_revision_experiments/scripts/collect_summaries.py
```
