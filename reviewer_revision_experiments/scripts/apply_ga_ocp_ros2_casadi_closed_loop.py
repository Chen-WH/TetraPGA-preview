#!/usr/bin/env python3
"""Enable the CasADi backend for the GA-OCP closed-loop ROS2 node only."""

from __future__ import annotations

from pathlib import Path


PATH = Path("/home/chenwh/ros2_ws/src/GA-OCP/ga_ocp_ros2/CMakeLists.txt")


def replace_once(text: str, old: str, new: str) -> str:
    if old not in text:
        raise RuntimeError(f"pattern not found: {old[:80]!r}")
    return text.replace(old, new, 1)


def main() -> int:
    text = PATH.read_text()
    if "GA_OCP_ROS2_ENABLE_CASADI" not in text:
        text = replace_once(
            text,
            "find_package(visualization_msgs REQUIRED)\n",
            """find_package(visualization_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CASADI QUIET casadi)
option(GA_OCP_ROS2_ENABLE_CASADI "Enable CasADi backend for closed_loop_mpc_node" ON)
if(GA_OCP_ROS2_ENABLE_CASADI AND NOT CASADI_FOUND)
  message(WARNING "CasADi was not found; closed_loop_mpc_node will reject backend:=casadi")
endif()
""",
        )
        text = replace_once(
            text,
            """target_link_libraries(closed_loop_mpc_node
  TetraPGA::TetraPGA
  crocoddyl::crocoddyl
  pinocchio::pinocchio
)
""",
            """target_link_libraries(closed_loop_mpc_node
  TetraPGA::TetraPGA
  crocoddyl::crocoddyl
  pinocchio::pinocchio
)
if(GA_OCP_ROS2_ENABLE_CASADI AND CASADI_FOUND)
  target_link_libraries(closed_loop_mpc_node ${CASADI_LIBRARIES})
  target_compile_definitions(closed_loop_mpc_node PRIVATE GA_OCP_HAS_CASADI_BENCH=1)
  target_include_directories(closed_loop_mpc_node PRIVATE ${CASADI_INCLUDE_DIRS})
  target_link_directories(closed_loop_mpc_node PRIVATE ${CASADI_LIBRARY_DIRS})
endif()
""",
        )
    PATH.write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
