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
