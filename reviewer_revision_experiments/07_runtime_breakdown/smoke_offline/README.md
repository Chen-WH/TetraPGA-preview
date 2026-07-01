# Offline Runtime Breakdown

This run uses random point-to-point FDDP tasks, not closed-loop ROS/MuJoCo MPC.

- Robots: ur10
- Backends: TetraPGA
- Samples: 1
- Seed: 539362855
- Horizon: 50
- dt: 0.02
- Max iterations: 2
- Position limit: 0.75
- Warmup excluded from CSV: yes

Cost terms are state regularization, acceleration regularization, and effort regularization. Collision terms are intentionally disabled for all robots.
