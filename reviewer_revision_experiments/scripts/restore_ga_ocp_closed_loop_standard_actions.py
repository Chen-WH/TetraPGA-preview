#!/usr/bin/env python3
"""Use standard GA-OCP Crocoddyl actions/residuals in the closed-loop node."""

from __future__ import annotations

from pathlib import Path


PATH = Path("/home/chenwh/ros2_ws/src/GA-OCP/ga_ocp_ros2/src/closed_loop_mpc_node.cpp")


def replace_once(text: str, old: str, new: str) -> str:
    if old not in text:
        raise RuntimeError(f"pattern not found: {old[:80]!r}")
    return text.replace(old, new, 1)


def main() -> int:
    text = PATH.read_text()
    text = replace_once(
        text,
        '''#include "ga_ocp/CrocoddylActionsRuntimeProfile.hpp"
#include "ga_ocp/CrocoddylResidualsRuntimeProfile.hpp"
#include "ga_ocp/RuntimeProfiler.hpp"
#include "TetraPGA/Collision.hpp"
#include "TetraPGA/ModelRepo.hpp"

#ifdef GA_OCP_HAS_CASADI_BENCH
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/autodiff/casadi-algo.hpp>
#endif
''',
        '''#include "ga_ocp/CrocoddylActions.hpp"
#include "ga_ocp/CrocoddylResiduals.hpp"
#include "ga_ocp/RuntimeProfiler.hpp"
#include "TetraPGA/Collision.hpp"
#include "TetraPGA/ModelRepo.hpp"

#ifdef GA_OCP_HAS_CASADI_BENCH
#include "ga_ocp/BenchUtils.hpp"
#endif
''',
    )

    local_start = text.find("\n#ifdef GA_OCP_HAS_CASADI_BENCH\nclass InlineAutoDiffABADerivatives")
    if local_start != -1:
        local_end = text.find("\nstd::string CsvEscape", local_start)
        if local_end == -1:
            raise RuntimeError("failed to find end of local CasADi action block")
        text = text[:local_start] + "\n" + text[local_end:]

    text = text.replace("CsvEscape(", "LocalCsvEscape(")
    text = text.replace("FormatCsvNumber(", "LocalFormatCsvNumber(")

    PATH.write_text(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
