#!/usr/bin/env python3
"""Apply GA-OCP benchmark additions for reviewer revision experiments.

This script is intentionally narrow and idempotent because GA-OCP is outside
the writable sandbox root in this session.
"""

from __future__ import annotations

from pathlib import Path


GA_ROOT = Path("/home/chenwh/ros2_ws/src/GA-OCP")
PATCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "patches"
    / "ga_ocp_revision_benchmarks.patch"
)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Unable to find replacement anchor for {label}")
    return text.replace(old, new, 1)


def extract_new_file_from_patch(path: str) -> str:
    lines = PATCH_FILE.read_text(encoding="utf-8").splitlines()
    marker = f"+++ b/{path}"
    for i, line in enumerate(lines):
        if line == marker:
            start = i + 1
            break
    else:
        raise RuntimeError(f"Unable to find {path} in {PATCH_FILE}")

    while start < len(lines) and not lines[start].startswith("@@ "):
        start += 1
    if start >= len(lines):
        raise RuntimeError(f"Unable to find hunk body for {path}")

    out: list[str] = []
    for line in lines[start + 1 :]:
        if line.startswith("diff --git "):
            break
        if line.startswith("+") and not line.startswith("+++"):
            out.append(line[1:])
        elif line.startswith("\\"):
            continue
        elif line:
            raise RuntimeError(f"Unexpected non-added line while extracting {path}: {line}")
    return "\n".join(out) + "\n"


def update_cmake() -> None:
    path = GA_ROOT / "ga_ocp_core" / "CMakeLists.txt"
    text = path.read_text(encoding="utf-8")
    old = (
        "    ga_ocp_add_benchmark(Crocoddyl_sqp_bench benchmark/Crocoddyl_sqp_bench.cpp)\n"
        "    target_link_libraries(Crocoddyl_sqp_bench mim_solvers::mim_solvers)\n"
    )
    new = (
        "    ga_ocp_add_benchmark(Crocoddyl_sqp_bench benchmark/Crocoddyl_sqp_bench.cpp)\n"
        "    ga_ocp_add_benchmark(Crocoddyl_obstacle_margin_sweep benchmark/Crocoddyl_obstacle_margin_sweep.cpp)\n"
        "    target_link_libraries(Crocoddyl_sqp_bench mim_solvers::mim_solvers)\n"
    )
    path.write_text(replace_once(text, old, new, "CMake benchmark target"), encoding="utf-8")


def update_budget_bench() -> None:
    path = GA_ROOT / "ga_ocp_core" / "benchmark" / "Crocoddyl_fddp_budget_bench.cpp"
    text = path.read_text(encoding="utf-8")
    replacements = [
        (
            "  kLeapHand,\n"
            "  kBinaryTree,\n"
            "  kBinaryTree31Dof,\n",
            "  kLeapHand,\n"
            "  kBinaryTree,\n"
            "  kStanfordTidyBot,\n"
            "  kBinaryTree31Dof,\n",
            "scenario enum",
        ),
        (
            "    case ScenarioKind::kBinaryTree:\n"
            "      return \"binary_tree\";\n"
            "    case ScenarioKind::kBinaryTree31Dof:\n",
            "    case ScenarioKind::kBinaryTree:\n"
            "      return \"binary_tree\";\n"
            "    case ScenarioKind::kStanfordTidyBot:\n"
            "      return \"stanford_tidybot\";\n"
            "    case ScenarioKind::kBinaryTree31Dof:\n",
            "scenario name",
        ),
        (
            "  if (value == \"binary_tree\" || value == \"tree\") {\n"
            "    return ScenarioKind::kBinaryTree;\n"
            "  }\n"
            "  if (value == \"binary_tree_31dof\" || value == \"bt31\" || value == \"tree31\") {\n",
            "  if (value == \"binary_tree\" || value == \"tree\") {\n"
            "    return ScenarioKind::kBinaryTree;\n"
            "  }\n"
            "  if (value == \"stanford_tidybot\" || value == \"tidybot\") {\n"
            "    return ScenarioKind::kStanfordTidyBot;\n"
            "  }\n"
            "  if (value == \"binary_tree_31dof\" || value == \"bt31\" || value == \"tree31\") {\n",
            "scenario parser",
        ),
        (
            "          << \"  --scenario=ur10|leap_hand|binary_tree|binary_tree_31dof\\n\"\n",
            "          << \"  --scenario=ur10|leap_hand|stanford_tidybot|binary_tree|binary_tree_31dof\\n\"\n",
            "scenario help",
        ),
        (
            "pinocchio::Model BuildLeapHandPinModel() {\n"
            "  const std::filesystem::path urdf_path = LeapHandUrdfPath();\n"
            "  pinocchio::Model pin_model;\n"
            "  pinocchio::urdf::buildModel(urdf_path.string(), pin_model);\n"
            "  return pin_model;\n"
            "}\n"
            "\n"
            "ScenarioContext BuildScenarioContext(const CliConfig& config) {\n",
            "pinocchio::Model BuildLeapHandPinModel() {\n"
            "  const std::filesystem::path urdf_path = LeapHandUrdfPath();\n"
            "  pinocchio::Model pin_model;\n"
            "  pinocchio::urdf::buildModel(urdf_path.string(), pin_model);\n"
            "  return pin_model;\n"
            "}\n"
            "\n"
            "std::filesystem::path StanfordTidyBotUrdfPath() {\n"
            "  return PackageRoot() / \"robot-assets\" / \"stanford_tidybot\" / \"urdf\" /\n"
            "         \"tidybot_gen3_10dof.urdf\";\n"
            "}\n"
            "\n"
            "pinocchio::Model BuildStanfordTidyBotPinModel() {\n"
            "  pinocchio::Model pin_model;\n"
            "  pinocchio::urdf::buildModel(StanfordTidyBotUrdfPath().string(), pin_model);\n"
            "  return pin_model;\n"
            "}\n"
            "\n"
            "ScenarioContext BuildScenarioContext(const CliConfig& config) {\n",
            "tidybot builders",
        ),
        (
            "  if (config.scenario == ScenarioKind::kLeapHand) {\n"
            "    const std::string urdf_path = LeapHandUrdfPath().string();\n"
            "    context.ga_model = std::make_shared<Model<double>>(leap_hand(urdf_path));\n"
            "    context.pin_model = BuildLeapHandPinModel();\n"
            "    context.dof = context.ga_model->dof_a;\n"
            "#ifdef GA_OCP_HAS_CASADI_BENCH\n"
            "    context.casadi_autodiff =\n"
            "        std::make_shared<InlineAutoDiffABADerivatives>(context.pin_model, \"tetrapga_budget_leap_hand\");\n"
            "#endif\n"
            "    return context;\n"
            "  }\n"
            "\n"
            "  const int dof = config.scenario == ScenarioKind::kBinaryTree31Dof ? 31 : DofFromLevel(config.level);\n",
            "  if (config.scenario == ScenarioKind::kLeapHand) {\n"
            "    const std::string urdf_path = LeapHandUrdfPath().string();\n"
            "    context.ga_model = std::make_shared<Model<double>>(leap_hand(urdf_path));\n"
            "    context.pin_model = BuildLeapHandPinModel();\n"
            "    context.dof = context.ga_model->dof_a;\n"
            "#ifdef GA_OCP_HAS_CASADI_BENCH\n"
            "    context.casadi_autodiff =\n"
            "        std::make_shared<InlineAutoDiffABADerivatives>(context.pin_model, \"tetrapga_budget_leap_hand\");\n"
            "#endif\n"
            "    return context;\n"
            "  }\n"
            "\n"
            "  if (config.scenario == ScenarioKind::kStanfordTidyBot) {\n"
            "    const std::string urdf_path = StanfordTidyBotUrdfPath().string();\n"
            "    context.ga_model = std::make_shared<Model<double>>(urdf_path);\n"
            "    context.pin_model = BuildStanfordTidyBotPinModel();\n"
            "    context.dof = context.ga_model->dof_a;\n"
            "#ifdef GA_OCP_HAS_CASADI_BENCH\n"
            "    context.casadi_autodiff = std::make_shared<InlineAutoDiffABADerivatives>(\n"
            "        context.pin_model, \"tetrapga_budget_stanford_tidybot\");\n"
            "#endif\n"
            "    return context;\n"
            "  }\n"
            "\n"
            "  const int dof = config.scenario == ScenarioKind::kBinaryTree31Dof ? 31 : DofFromLevel(config.level);\n",
            "tidybot scenario context",
        ),
        (
            "std::filesystem::path PackageRoot() {\n"
            "  return std::filesystem::path(__FILE__).parent_path().parent_path();\n"
            "}\n"
            "\n"
            "std::string DefaultOutputPrefix(const CliConfig& config) {\n",
            "std::filesystem::path PackageRoot() {\n"
            "  return std::filesystem::path(__FILE__).parent_path().parent_path();\n"
            "}\n"
            "\n"
            "std::filesystem::path RobotAssetsRoot() {\n"
            "  return std::filesystem::path(GA_OCP_ROBOT_ASSETS_DIR);\n"
            "}\n"
            "\n"
            "std::string DefaultOutputPrefix(const CliConfig& config) {\n",
            "robot assets root helper",
        ),
        (
            "  const std::filesystem::path urdf_path = PackageRoot() / \"robot-assets\" / \"ur10\" / \"urdf\" / \"ur10.urdf\";\n",
            "  const std::filesystem::path urdf_path = RobotAssetsRoot() / \"ur10\" / \"urdf\" / \"ur10.urdf\";\n",
            "ur10 asset path",
        ),
        (
            "  return PackageRoot() / \"robot-assets\" / \"leap_hand\" / \"urdf\" / \"leap_hand_left.urdf\";\n",
            "  return RobotAssetsRoot() / \"leap_hand\" / \"urdf\" / \"leap_hand_left.urdf\";\n",
            "leap asset path",
        ),
        (
            "  return PackageRoot() / \"robot-assets\" / \"stanford_tidybot\" / \"urdf\" /\n"
            "         \"tidybot_gen3_10dof.urdf\";\n",
            "  return RobotAssetsRoot() / \"stanford_tidybot\" / \"urdf\" /\n"
            "         \"tidybot_gen3_10dof.urdf\";\n",
            "tidybot asset path",
        ),
    ]

    for old, new, label in replacements:
        text = replace_once(text, old, new, label)
    path.write_text(text, encoding="utf-8")


def write_obstacle_sweep() -> None:
    rel_path = "ga_ocp_core/benchmark/Crocoddyl_obstacle_margin_sweep.cpp"
    path = GA_ROOT / rel_path
    if path.exists():
        return
    path.write_text(extract_new_file_from_patch(rel_path), encoding="utf-8")


def fix_benchutils_extra_endif() -> None:
    path = GA_ROOT / "ga_ocp_core" / "include" / "ga_ocp" / "BenchUtils.hpp"
    text = path.read_text(encoding="utf-8")
    if text.endswith("\n#endif\n\n#endif\n"):
        path.write_text(text[:-8], encoding="utf-8")


def main() -> None:
    update_cmake()
    update_budget_bench()
    write_obstacle_sweep()
    fix_benchutils_extra_endif()
    print("Applied GA-OCP revision benchmark additions.")


if __name__ == "__main__":
    main()
