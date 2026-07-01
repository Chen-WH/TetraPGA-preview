# Stanford TidyBot trajectory visualization

Source trajectory: `/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/04_closed_loop_mpc_metrics/reference_batch_20260628_223000/reference_tidybot_tetrapga_cycles.csv`

State column: `q`

MuJoCo model: `/home/chenwh/ros2_ws/src/robot-assets/stanford_tidybot/mjcf/scene.xml`

Rows: 1000

Duration: 19.980 s

Final tracking error: 0.131505

Solve-time p95: 10.505 ms

Generated files:

- `tidybot_trajectory.csv`: time, mobile-base pose, end-effector site pose, and MPC metrics.
- `tidybot_mujoco_final.png`: MuJoCo final-state render.
- `tidybot_mujoco_snapshots.png`: four MuJoCo snapshots along the trajectory.
- `tidybot_trajectory_figure.png`: paper/response-letter figure.

Suggested caption:

> Stanford TidyBot closed-loop receding-horizon MPC trajectory in MuJoCo. The mobile base and Gen3 arm are planned jointly over 10 generalized coordinates; the plot shows the mobile-base path, the end-effector path, and solver/tracking metrics over a 20 s rollout.

MuJoCo rendering fallback/error: `ValueError: Image width 960 > framebuffer width 640. Either reduce the image
width or specify a larger offscreen framebuffer in the model XML using the
clause:
<visual>
  <global offwidth="my_width"/>
</visual>`
