#!/usr/bin/env python3
"""Apply MuJoCo closed-loop fixes for the 50 Hz reviewer rerun."""

from __future__ import annotations

from pathlib import Path


ROOT = Path("/home/chenwh/ros2_ws/src/GA-OCP")
EXECUTOR = ROOT / "ga_ocp_ros2/scripts/joint_command_executor.py"
LAUNCH = ROOT / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_ur.launch.py"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def patch_executor() -> None:
    text = EXECUTOR.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "        self._lookup_external_force_body(model)\n"
        "        self._lookup_joint_addresses(model)\n"
        "        data.ctrl[self.ctrl_addrs] = self.default_target\n",
        "        self._lookup_external_force_body(model)\n"
        "        self._lookup_joint_addresses(model)\n"
        "        data.qpos[self.qpos_addrs] = self.default_target\n"
        "        data.qvel[self.qvel_addrs] = 0.0\n"
        "        data.ctrl[self.ctrl_addrs] = self.default_target\n"
        "        mujoco.mj_forward(model, data)\n",
        "MuJoCo initial qpos reset",
    )
    EXECUTOR.write_text(text, encoding="utf-8")


def patch_launch() -> None:
    text = LAUNCH.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "    solve_budget_ms = LaunchConfiguration('solve_budget_ms')\n"
        "    duration_s = LaunchConfiguration('duration_s')\n",
        "    solve_budget_ms = LaunchConfiguration('solve_budget_ms')\n"
        "    duration_s = LaunchConfiguration('duration_s')\n"
        "    dt = LaunchConfiguration('dt')\n"
        "    horizon = LaunchConfiguration('horizon')\n"
        "    control_rate_hz = LaunchConfiguration('control_rate_hz')\n",
        "50Hz launch configs",
    )
    text = replace_once(
        text,
        "                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),\n"
        "                'experiment_duration_s': ParameterValue(duration_s, value_type=float),\n",
        "                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),\n"
        "                'experiment_duration_s': ParameterValue(duration_s, value_type=float),\n"
        "                'dt': ParameterValue(dt, value_type=float),\n"
        "                'horizon': ParameterValue(horizon, value_type=int),\n"
        "                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),\n",
        "50Hz node params",
    )
    text = replace_once(
        text,
        "        DeclareLaunchArgument('solve_budget_ms', default_value='10.0'),\n"
        "        DeclareLaunchArgument('duration_s', default_value='20.0'),\n",
        "        DeclareLaunchArgument('solve_budget_ms', default_value='10.0'),\n"
        "        DeclareLaunchArgument('duration_s', default_value='20.0'),\n"
        "        DeclareLaunchArgument('dt', default_value='0.008'),\n"
        "        DeclareLaunchArgument('horizon', default_value='40'),\n"
        "        DeclareLaunchArgument('control_rate_hz', default_value='125.0'),\n",
        "50Hz launch args",
    )
    LAUNCH.write_text(text, encoding="utf-8")


def main() -> None:
    patch_executor()
    patch_launch()
    print("Applied GA-OCP MuJoCo qpos reset and launch timing overrides.")


if __name__ == "__main__":
    main()
