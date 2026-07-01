# UR10 trajectory visualization

Source trajectory: `/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_223000/reference_ur_tetrapga_cycles.csv`

State column: `q`

MuJoCo model: `/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/09_mujoco_trajectory_visualization/ur_trajectory_20260630_235225/ur10_render_scene.xml`

Tracked target(s): body:attachment (tool frame)

Rows: 2500

Duration: 19.992 s

Final tracking error: 0.213119

Solve-time p95: 4.354 ms

Generated files:

- `ur_trajectory.csv`: target positions and MPC metrics.
- `ur_mujoco_final.png`: MuJoCo final-state render.
- `ur_mujoco_snapshots.png`: four MuJoCo snapshots along the trajectory.
- `ur_trajectory_figure.png`: paper/response-letter figure.

Suggested caption:

> UR10 closed-loop receding-horizon MPC trajectory in MuJoCo. The plot shows the rendered robot state, the tracked Cartesian target path, and solver/tracking metrics over a 20 s rollout.
