# LEAP Hand trajectory visualization

Source trajectory: `/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_223000/reference_leap_tetrapga_cycles.csv`

State column: `q`

MuJoCo model: `/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/09_mujoco_trajectory_visualization/leap_left_trajectory_20260630_235410/leap_left_render_scene.xml`

Tracked target(s): geom:if_tip (index tip), geom:mf_tip (middle tip), geom:rf_tip (ring tip), geom:th_tip (thumb tip)

Rows: 1000

Duration: 19.980 s

Final tracking error: 0.174952

Solve-time p95: 9.385 ms

Generated files:

- `leap_left_trajectory.csv`: target positions and MPC metrics.
- `leap_left_mujoco_final.png`: MuJoCo final-state render.
- `leap_left_mujoco_snapshots.png`: four MuJoCo snapshots along the trajectory.
- `leap_left_trajectory_figure.png`: paper/response-letter figure.

Suggested caption:

> LEAP Hand closed-loop receding-horizon MPC trajectory in MuJoCo. The plot shows the rendered robot state, the tracked Cartesian target path, and solver/tracking metrics over a 20 s rollout.

For LEAP hand, the Cartesian path is computed from the four named fingertip geoms and their centroid.
