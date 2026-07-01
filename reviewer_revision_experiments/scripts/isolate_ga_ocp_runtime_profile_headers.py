#!/usr/bin/env python3
"""Move GA-OCP runtime profiling into runtime-only Crocoddyl headers."""

from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path("/home/chenwh/ros2_ws/src/GA-OCP")
CORE_INCLUDE = ROOT / "ga_ocp_core/include/ga_ocp"
ACTIONS = CORE_INCLUDE / "CrocoddylActions.hpp"
RESIDUALS = CORE_INCLUDE / "CrocoddylResiduals.hpp"
RUNTIME_ACTIONS = CORE_INCLUDE / "CrocoddylActionsRuntimeProfile.hpp"
RUNTIME_RESIDUALS = CORE_INCLUDE / "CrocoddylResidualsRuntimeProfile.hpp"
NODE = ROOT / "ga_ocp_ros2/src/closed_loop_mpc_node.cpp"


def git_show(path: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(ROOT), "show", f"HEAD:{path}"], text=True
    )


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def main() -> None:
    actions_text = ACTIONS.read_text(encoding="utf-8")
    residuals_text = RESIDUALS.read_text(encoding="utf-8")
    actions_instrumented = "RuntimeProfilerRecordDamCalc" in actions_text
    residuals_instrumented = "RuntimeProfilerRecordCollisionResidualCalc" in residuals_text
    runtime_actions_ready = (
        RUNTIME_ACTIONS.exists()
        and "RuntimeProfilerRecordDamCalc" in RUNTIME_ACTIONS.read_text(encoding="utf-8")
    )
    runtime_residuals_ready = (
        RUNTIME_RESIDUALS.exists()
        and "RuntimeProfilerRecordCollisionResidualCalc"
        in RUNTIME_RESIDUALS.read_text(encoding="utf-8")
    )

    if actions_instrumented != residuals_instrumented:
        raise RuntimeError("Only one original Crocoddyl header is instrumented; refusing to guess.")
    if actions_instrumented:
        RUNTIME_ACTIONS.write_text(actions_text, encoding="utf-8")
        RUNTIME_RESIDUALS.write_text(
            residuals_text.replace(
                "#include \"ga_ocp/CrocoddylActions.hpp\"\n",
                "#include \"ga_ocp/CrocoddylActionsRuntimeProfile.hpp\"\n",
                1,
            ),
            encoding="utf-8",
        )
        ACTIONS.write_text(
            git_show("ga_ocp_core/include/ga_ocp/CrocoddylActions.hpp"),
            encoding="utf-8",
        )
        RESIDUALS.write_text(
            git_show("ga_ocp_core/include/ga_ocp/CrocoddylResiduals.hpp"),
            encoding="utf-8",
        )
    elif not (runtime_actions_ready and runtime_residuals_ready):
        raise RuntimeError(
            "Runtime-profile Crocoddyl headers are missing and the original headers "
            "are not instrumented. Run apply_ga_ocp_solver_internal_breakdown.py first."
        )

    node_text = NODE.read_text(encoding="utf-8")
    node_text = replace_once(
        node_text,
        "#include \"ga_ocp/CrocoddylActions.hpp\"\n"
        "#include \"ga_ocp/CrocoddylResiduals.hpp\"\n"
        "#include \"ga_ocp/RuntimeProfiler.hpp\"\n",
        "#include \"ga_ocp/CrocoddylActionsRuntimeProfile.hpp\"\n"
        "#include \"ga_ocp/CrocoddylResidualsRuntimeProfile.hpp\"\n"
        "#include \"ga_ocp/RuntimeProfiler.hpp\"\n",
        "closed_loop_mpc_node runtime-only includes",
    )
    NODE.write_text(node_text, encoding="utf-8")
    print("Isolated GA-OCP runtime profiling into runtime-only headers.")


if __name__ == "__main__":
    main()
