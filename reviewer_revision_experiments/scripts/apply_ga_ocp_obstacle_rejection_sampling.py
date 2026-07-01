#!/usr/bin/env python3
"""Apply controlled obstacle sampling changes to the GA-OCP benchmark.

The GA-OCP repository lives outside this workspace's writable root in the
Codex sandbox, so this script is kept here as a reproducible edit artifact and
run with explicit approval when needed.
"""

from __future__ import annotations

from pathlib import Path


GA_OCP_ROOT = Path("/home/chenwh/ros2_ws/src/GA-OCP")
SOURCE = GA_OCP_ROOT / "ga_ocp_core/benchmark/Crocoddyl_obstacle_margin_sweep.cpp"


def replace_once(text: str, old: str, new: str) -> str:
    if old not in text:
        raise RuntimeError(f"pattern not found:\n{old[:400]}")
    return text.replace(old, new, 1)


def main() -> int:
    text = SOURCE.read_text()
    if "endpoint_clearance" in text and "MakeObstacleEnvironment(model, q0, q_ref" in text:
        print(f"{SOURCE} already contains rejection-sampling obstacle controls")
        return 0

    text = replace_once(
        text,
        "#include <numeric>\n#include <sstream>\n",
        "#include <numeric>\n#include <random>\n#include <sstream>\n",
    )

    text = replace_once(
        text,
        "  double collision_weight = 100.0;\n"
        "  double control_weight = 1e-4;\n"
        "  std::string output_csv;\n",
        "  double collision_weight = 100.0;\n"
        "  double control_weight = 1e-4;\n"
        "  double obstacle_radius_min = 0.04;\n"
        "  double obstacle_radius_max = 0.10;\n"
        "  double endpoint_clearance = 0.06;\n"
        "  int obstacle_max_attempts = 2000;\n"
        "  std::string output_csv;\n",
    )

    text = replace_once(
        text,
        '          << "  --position_amplitude=<double>\\n"\n'
        '          << "  --collision_weight=<double>\\n";\n',
        '          << "  --position_amplitude=<double>\\n"\n'
        '          << "  --collision_weight=<double>\\n"\n'
        '          << "  --obstacle_radius_min=<double>\\n"\n'
        '          << "  --obstacle_radius_max=<double>\\n"\n'
        '          << "  --endpoint_clearance=<double>\\n"\n'
        '          << "  --obstacle_max_attempts=<int>\\n";\n',
    )

    text = replace_once(
        text,
        "    } else if (ConsumeOption(arg, \"collision_weight\", &value)) {\n"
        "      config.collision_weight = std::stod(value);\n"
        "    } else {\n",
        "    } else if (ConsumeOption(arg, \"collision_weight\", &value)) {\n"
        "      config.collision_weight = std::stod(value);\n"
        "    } else if (ConsumeOption(arg, \"obstacle_radius_min\", &value)) {\n"
        "      config.obstacle_radius_min = std::stod(value);\n"
        "    } else if (ConsumeOption(arg, \"obstacle_radius_max\", &value)) {\n"
        "      config.obstacle_radius_max = std::stod(value);\n"
        "    } else if (ConsumeOption(arg, \"endpoint_clearance\", &value)) {\n"
        "      config.endpoint_clearance = std::stod(value);\n"
        "    } else if (ConsumeOption(arg, \"obstacle_max_attempts\", &value)) {\n"
        "      config.obstacle_max_attempts = std::stoi(value);\n"
        "    } else {\n",
    )

    text = replace_once(
        text,
        "  if (config.output_csv.empty()) {\n"
        "    config.output_csv = \"Crocoddyl_obstacle_margin_sweep.csv\";\n"
        "  }\n"
        "  return config;\n",
        "  if (config.output_csv.empty()) {\n"
        "    config.output_csv = \"Crocoddyl_obstacle_margin_sweep.csv\";\n"
        "  }\n"
        "  if (config.obstacle_radius_min <= 0.0 ||\n"
        "      config.obstacle_radius_max < config.obstacle_radius_min) {\n"
        "    throw std::invalid_argument(\"invalid obstacle radius range\");\n"
        "  }\n"
        "  if (config.endpoint_clearance < 0.0) {\n"
        "    throw std::invalid_argument(\"endpoint_clearance must be non-negative\");\n"
        "  }\n"
        "  if (config.obstacle_max_attempts <= 0) {\n"
        "    throw std::invalid_argument(\"obstacle_max_attempts must be positive\");\n"
        "  }\n"
        "  return config;\n",
    )

    old_make_env = """Environment<double> MakeObstacleEnvironment(const int obstacle_count, const std::uint32_t seed) {
  std::vector<SSP<double>> obstacles;
  obstacles.reserve(static_cast<std::size_t>(obstacle_count));

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> xdist(-0.55, 0.35);
  std::uniform_real_distribution<double> ydist(-0.55, 0.55);
  std::uniform_real_distribution<double> zdist(0.35, 1.15);
  std::uniform_real_distribution<double> rdist(0.06, 0.14);

  for (int i = 0; i < obstacle_count; ++i) {
    SSP<double> obs;
    obs.id = i;
    obs.radius = rdist(rng);
    obs.center = Point3D<double>(xdist(rng), ydist(rng), zdist(rng), 1.0);
    obstacles.push_back(obs);
  }
  return Environment<double>(obstacles);
}
"""
    new_make_env = """CollisionSummary SummarizeConfigurationCollision(const Model<double>& model,
                                                 const Environment<double>& env,
                                                 const Eigen::VectorXd& q,
                                                 double d_safe);

Environment<double> MakeObstacleEnvironment(const Model<double>& model,
                                            const Eigen::VectorXd& q0,
                                            const Eigen::VectorXd& q_ref,
                                            const int obstacle_count,
                                            const std::uint32_t seed,
                                            const CliConfig& config) {
  std::vector<SSP<double>> obstacles;
  obstacles.reserve(static_cast<std::size_t>(obstacle_count));

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> xdist(-0.55, 0.35);
  std::uniform_real_distribution<double> ydist(-0.55, 0.55);
  std::uniform_real_distribution<double> zdist(0.35, 1.15);
  std::uniform_real_distribution<double> rdist(config.obstacle_radius_min,
                                               config.obstacle_radius_max);

  for (int i = 0; i < obstacle_count; ++i) {
    bool accepted = false;
    for (int attempt = 0; attempt < config.obstacle_max_attempts; ++attempt) {
      SSP<double> obs;
      obs.id = i;
      obs.radius = rdist(rng);
      obs.center = Point3D<double>(xdist(rng), ydist(rng), zdist(rng), 1.0);

      std::vector<SSP<double>> trial_obstacles = obstacles;
      trial_obstacles.push_back(obs);
      Environment<double> trial_env(trial_obstacles);
      const CollisionSummary initial_collision =
          SummarizeConfigurationCollision(model, trial_env, q0, 0.0);
      const CollisionSummary target_collision =
          SummarizeConfigurationCollision(model, trial_env, q_ref, 0.0);
      if (initial_collision.min_distance >= config.endpoint_clearance &&
          target_collision.min_distance >= config.endpoint_clearance) {
        obstacles.push_back(obs);
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      throw std::runtime_error("failed to sample obstacle environment with endpoint clearance");
    }
  }
  return Environment<double>(obstacles);
}
"""
    text = replace_once(text, old_make_env, new_make_env)

    text = replace_once(
        text,
        "          const Environment<double> env = MakeObstacleEnvironment(obstacle_count, sample_seed);\n",
        "          const Environment<double> env =\n"
        "              MakeObstacleEnvironment(model, q0, q_ref, obstacle_count, sample_seed, config);\n",
    )

    SOURCE.write_text(text)
    print(f"Updated {SOURCE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
