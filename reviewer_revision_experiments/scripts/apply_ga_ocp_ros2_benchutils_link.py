#!/usr/bin/env python3
"""Link benchmark for closed_loop_mpc_node when it includes BenchUtils.hpp."""

from __future__ import annotations

from pathlib import Path


PATH = Path("/home/chenwh/ros2_ws/src/GA-OCP/ga_ocp_ros2/CMakeLists.txt")


def replace_once(text: str, old: str, new: str) -> str:
    if old not in text:
        raise RuntimeError(f"pattern not found: {old[:80]!r}")
    return text.replace(old, new, 1)


def main() -> int:
    text = PATH.read_text()
    if "find_package(benchmark REQUIRED)" not in text:
        text = replace_once(
            text,
            "if(GA_OCP_ROS2_ENABLE_CASADI AND NOT CASADI_FOUND)\n",
            "if(GA_OCP_ROS2_ENABLE_CASADI AND CASADI_FOUND)\n  find_package(benchmark REQUIRED)\nendif()\nif(GA_OCP_ROS2_ENABLE_CASADI AND NOT CASADI_FOUND)\n",
        )
    if "benchmark::benchmark" not in text:
        text = replace_once(
            text,
            "  target_link_libraries(closed_loop_mpc_node ${CASADI_LIBRARIES})\n",
            "  target_link_libraries(closed_loop_mpc_node ${CASADI_LIBRARIES} benchmark::benchmark)\n",
        )
    PATH.write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
