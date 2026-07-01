#!/usr/bin/env python3
"""Expose GA-OCP closed-loop budget enforcement as a launch argument."""

from __future__ import annotations

from pathlib import Path


LAUNCH = Path(
    "/home/chenwh/ros2_ws/src/GA-OCP/"
    "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_ur.launch.py"
)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def main() -> None:
    text = LAUNCH.read_text(encoding="utf-8")
    text = replace_once(
        text,
        "    solve_budget_ms = LaunchConfiguration('solve_budget_ms')\n"
        "    duration_s = LaunchConfiguration('duration_s')\n",
        "    solve_budget_ms = LaunchConfiguration('solve_budget_ms')\n"
        "    enforce_solve_budget = LaunchConfiguration('enforce_solve_budget')\n"
        "    duration_s = LaunchConfiguration('duration_s')\n",
        "launch configuration",
    )
    text = replace_once(
        text,
        "                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),\n"
        "                'experiment_duration_s': ParameterValue(duration_s, value_type=float),\n",
        "                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),\n"
        "                'enforce_solve_budget': ParameterValue(enforce_solve_budget, value_type=bool),\n"
        "                'experiment_duration_s': ParameterValue(duration_s, value_type=float),\n",
        "node parameter",
    )
    text = replace_once(
        text,
        "        DeclareLaunchArgument('solve_budget_ms', default_value='10.0'),\n"
        "        DeclareLaunchArgument('duration_s', default_value='20.0'),\n",
        "        DeclareLaunchArgument('solve_budget_ms', default_value='10.0'),\n"
        "        DeclareLaunchArgument('enforce_solve_budget', default_value='true'),\n"
        "        DeclareLaunchArgument('duration_s', default_value='20.0'),\n",
        "launch argument",
    )
    LAUNCH.write_text(text, encoding="utf-8")
    print(f"Patched {LAUNCH}")


if __name__ == "__main__":
    main()
