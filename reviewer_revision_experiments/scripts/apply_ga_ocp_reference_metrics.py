#!/usr/bin/env python3
"""Patch the external GA-OCP workspace for reference closed-loop metrics."""

from __future__ import annotations

from pathlib import Path


GA_OCP = Path("/home/chenwh/ros2_ws/src/GA-OCP")
DOC_GA_OCP = Path("/home/chenwh/Documents/GA-OCP")


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text()
    if old not in text:
        raise RuntimeError(f"pattern not found in {path}: {old[:80]!r}")
    path.write_text(text.replace(old, new, 1))


def ensure_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)


def patch_closed_loop_node() -> None:
    path = GA_OCP / "ga_ocp_ros2/src/closed_loop_mpc_node.cpp"
    tidy_block = r'''
  if (robot == "stanford_tidybot" || robot == "tidybot") {
    const std::string tidybot_urdf =
        share_dir + "/robot-assets/stanford_tidybot/urdf/tidybot_gen3_10dof.urdf";
    Model<double> ga_model = model_from_name("stanford_tidybot", tidybot_urdf);
    ga_model.qa0.resize(10);
    ga_model.qa0 <<
        0.0, 0.0, 0.0,
        0.0, 0.26179939, 3.14159265, -2.26892803,
        0.0, 0.95993109, 1.57079633;

    RobotConfig config{
        "stanford_tidybot",
        tidybot_urdf,
        "tidybot_gen3_10dof",
        MakeJointNames({
            "base_x_joint",
            "base_y_joint",
            "base_yaw_joint",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        }),
        ga_model,
        Eigen::Vector3d::Zero(),
        Eigen::Quaterniond::Identity(),
        "",
        Eigen::VectorXd::Constant(10, 0.18),
    };
    config.default_amplitudes <<
        0.18, 0.18, 0.25,
        0.24, 0.18, 0.24, 0.16, 0.20, 0.14, 0.18;
    return config;
  }

'''
    replace_once(
        path,
        '  throw std::invalid_argument("Unsupported robot: " + robot);\n',
        tidy_block + '  throw std::invalid_argument("Unsupported robot: " + robot);\n',
    )

    replace_once(
        path,
        '''#ifdef GA_OCP_HAS_CASADI_BENCH
    if (backend_ == BackendKind::kCasadi) {
      const std::string cache_tag =
          robot_config_.robot == "ur" ? "closed_loop_ur10" : "closed_loop_leap";
      casadi_autodiff_ = std::make_shared<InlineAutoDiffABADerivatives>(pin_model_, cache_tag);
    }
#else
''',
        '''#ifdef GA_OCP_HAS_CASADI_BENCH
    if (backend_ == BackendKind::kCasadi) {
      std::string cache_tag = "closed_loop_" + robot_config_.robot;
      if (robot_config_.robot == "ur") {
        cache_tag = "closed_loop_ur10";
      } else if (robot_config_.robot == "leap_left") {
        cache_tag = "closed_loop_leap";
      } else if (robot_config_.robot == "stanford_tidybot") {
        cache_tag = "closed_loop_tidybot";
      }
      casadi_autodiff_ = std::make_shared<InlineAutoDiffABADerivatives>(pin_model_, cache_tag);
    }
#else
''',
    )


def patch_joint_executor() -> None:
    path = GA_OCP / "ga_ocp_ros2/scripts/joint_command_executor.py"
    old = '''    'leap_left': RobotConfig(
        robot='leap_left',
        scene_relative_path='robot-assets/leap_hand/mjcf/scene_left.xml',
        joint_state_names=[str(i) for i in range(16)],
        mujoco_joint_names=[
            'if_mcp', 'if_rot', 'if_pip', 'if_dip',
            'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
            'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip',
            'th_cmc', 'th_axl', 'th_mcp', 'th_ipl',
        ],
        actuator_names=[
            'if_mcp_act', 'if_rot_act', 'if_pip_act', 'if_dip_act',
            'mf_mcp_act', 'mf_rot_act', 'mf_pip_act', 'mf_dip_act',
            'rf_mcp_act', 'rf_rot_act', 'rf_pip_act', 'rf_dip_act',
            'th_cmc_act', 'th_axl_act', 'th_mcp_act', 'th_ipl_act',
        ],
        default_target=np.zeros(16, dtype=float),
        control_mode='direct',
        kp=np.zeros(16, dtype=float),
        kd=np.zeros(16, dtype=float),
        effort_limit=np.full(16, np.inf, dtype=float),
    ),
}
'''
    new = '''    'leap_left': RobotConfig(
        robot='leap_left',
        scene_relative_path='robot-assets/leap_hand/mjcf/scene_left.xml',
        joint_state_names=[str(i) for i in range(16)],
        mujoco_joint_names=[
            'if_mcp', 'if_rot', 'if_pip', 'if_dip',
            'mf_mcp', 'mf_rot', 'mf_pip', 'mf_dip',
            'rf_mcp', 'rf_rot', 'rf_pip', 'rf_dip',
            'th_cmc', 'th_axl', 'th_mcp', 'th_ipl',
        ],
        actuator_names=[
            'if_mcp_act', 'if_rot_act', 'if_pip_act', 'if_dip_act',
            'mf_mcp_act', 'mf_rot_act', 'mf_pip_act', 'mf_dip_act',
            'rf_mcp_act', 'rf_rot_act', 'rf_pip_act', 'rf_dip_act',
            'th_cmc_act', 'th_axl_act', 'th_mcp_act', 'th_ipl_act',
        ],
        default_target=np.zeros(16, dtype=float),
        control_mode='direct',
        kp=np.zeros(16, dtype=float),
        kd=np.zeros(16, dtype=float),
        effort_limit=np.full(16, np.inf, dtype=float),
    ),
    'stanford_tidybot': RobotConfig(
        robot='stanford_tidybot',
        scene_relative_path='robot-assets/stanford_tidybot/mjcf/scene.xml',
        joint_state_names=[
            'base_x_joint',
            'base_y_joint',
            'base_yaw_joint',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
        ],
        mujoco_joint_names=[
            'joint_x',
            'joint_y',
            'joint_th',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
        ],
        actuator_names=[
            'joint_x',
            'joint_y',
            'joint_th',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
        ],
        default_target=_array([
            0.0, 0.0, 0.0,
            0.0, 0.26179939, 3.14159265, -2.26892803,
            0.0, 0.95993109, 1.57079633,
        ]),
        control_mode='direct',
        kp=np.zeros(10, dtype=float),
        kd=np.zeros(10, dtype=float),
        effort_limit=_array([1000.0, 1000.0, 1000.0, 39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]),
    ),
    'tidybot': RobotConfig(
        robot='stanford_tidybot',
        scene_relative_path='robot-assets/stanford_tidybot/mjcf/scene.xml',
        joint_state_names=[
            'base_x_joint',
            'base_y_joint',
            'base_yaw_joint',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
        ],
        mujoco_joint_names=[
            'joint_x',
            'joint_y',
            'joint_th',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
        ],
        actuator_names=[
            'joint_x',
            'joint_y',
            'joint_th',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
        ],
        default_target=_array([
            0.0, 0.0, 0.0,
            0.0, 0.26179939, 3.14159265, -2.26892803,
            0.0, 0.95993109, 1.57079633,
        ]),
        control_mode='direct',
        kp=np.zeros(10, dtype=float),
        kd=np.zeros(10, dtype=float),
        effort_limit=_array([1000.0, 1000.0, 1000.0, 39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]),
    ),
}
'''
    replace_once(path, old, new)


def patch_leap_launch() -> None:
    path = GA_OCP / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_leap.launch.py"
    ensure_text(
        path,
        '''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ros2_share = get_package_share_directory('ga_ocp_ros2')
    config = f"{ros2_share}/config/closed_loop_mpc_leap.yaml"

    backend = LaunchConfiguration('backend')
    solve_budget_ms = LaunchConfiguration('solve_budget_ms')
    enforce_solve_budget = LaunchConfiguration('enforce_solve_budget')
    duration_s = LaunchConfiguration('duration_s')
    dt = LaunchConfiguration('dt')
    horizon = LaunchConfiguration('horizon')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    output_prefix = LaunchConfiguration('output_prefix')
    enable_viewer = LaunchConfiguration('enable_viewer')

    closed_loop_node = Node(
        package='ga_ocp_ros2',
        executable='closed_loop_mpc_node',
        name='closed_loop_mpc_node',
        output='screen',
        parameters=[
            config,
            {
                'backend': backend,
                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),
                'enforce_solve_budget': ParameterValue(enforce_solve_budget, value_type=bool),
                'experiment_duration_s': ParameterValue(duration_s, value_type=float),
                'dt': ParameterValue(dt, value_type=float),
                'horizon': ParameterValue(horizon, value_type=int),
                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),
                'output_prefix': output_prefix,
            },
        ],
    )

    mujoco_executor_node = Node(
        package='ga_ocp_ros2',
        executable='joint_command_executor.py',
        name='mujoco_joint_executor_node',
        output='screen',
        parameters=[
            {
                'robot': 'leap_left',
                'enable_viewer': ParameterValue(enable_viewer, value_type=bool),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='tetrapga'),
        DeclareLaunchArgument('solve_budget_ms', default_value='8.0'),
        DeclareLaunchArgument('enforce_solve_budget', default_value='true'),
        DeclareLaunchArgument('duration_s', default_value='20.0'),
        DeclareLaunchArgument('dt', default_value='0.02'),
        DeclareLaunchArgument('horizon', default_value='20'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('enable_viewer', default_value='true'),
        DeclareLaunchArgument('output_prefix', default_value=''),
        RegisterEventHandler(
            OnProcessExit(
                target_action=closed_loop_node,
                on_exit=[EmitEvent(event=Shutdown(reason='closed-loop mpc node finished'))],
            )
        ),
        closed_loop_node,
        mujoco_executor_node,
    ])
''',
    )


def add_tidybot_config_and_launch() -> None:
    ensure_text(
        GA_OCP / "ga_ocp_ros2/config/closed_loop_mpc_tidybot.yaml",
        '''closed_loop_mpc_node:
  ros__parameters:
    robot: stanford_tidybot
    backend: tetrapga
    dt: 0.02
    horizon: 20
    max_iterations: 80
    solve_budget_ms: 10.0
    control_rate_hz: 50.0
    experiment_duration_s: 20.0
    stop_tol: 1.0e-7
    use_warm_start: true
    auto_start: true
    shutdown_on_finish: true
    state_running_weight: 8.0
    state_terminal_weight: 120.0
    control_weight: 0.0005
    velocity_limit_weight: 20.0
    velocity_limit_scale: 0.9
    reference_frequency_hz: 0.08
    reference_secondary_ratio: 0.45
    reference_amplitude_scale: 0.95
    reference_ramp_duration_s: 1.5
    reference_amplitudes:
      [0.18, 0.18, 0.25, 0.24, 0.18, 0.24, 0.16, 0.20, 0.14, 0.18]
''',
    )
    ensure_text(
        GA_OCP / "ga_ocp_ros2/launch/ga_ocp_mujoco_closed_loop_tidybot.launch.py",
        '''from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    ros2_share = get_package_share_directory('ga_ocp_ros2')
    config = f"{ros2_share}/config/closed_loop_mpc_tidybot.yaml"

    backend = LaunchConfiguration('backend')
    solve_budget_ms = LaunchConfiguration('solve_budget_ms')
    enforce_solve_budget = LaunchConfiguration('enforce_solve_budget')
    duration_s = LaunchConfiguration('duration_s')
    dt = LaunchConfiguration('dt')
    horizon = LaunchConfiguration('horizon')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    output_prefix = LaunchConfiguration('output_prefix')
    enable_viewer = LaunchConfiguration('enable_viewer')

    closed_loop_node = Node(
        package='ga_ocp_ros2',
        executable='closed_loop_mpc_node',
        name='closed_loop_mpc_node',
        output='screen',
        parameters=[
            config,
            {
                'backend': backend,
                'solve_budget_ms': ParameterValue(solve_budget_ms, value_type=float),
                'enforce_solve_budget': ParameterValue(enforce_solve_budget, value_type=bool),
                'experiment_duration_s': ParameterValue(duration_s, value_type=float),
                'dt': ParameterValue(dt, value_type=float),
                'horizon': ParameterValue(horizon, value_type=int),
                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),
                'output_prefix': output_prefix,
            },
        ],
    )

    mujoco_executor_node = Node(
        package='ga_ocp_ros2',
        executable='joint_command_executor.py',
        name='mujoco_joint_executor_node',
        output='screen',
        parameters=[
            {
                'robot': 'stanford_tidybot',
                'enable_viewer': ParameterValue(enable_viewer, value_type=bool),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='tetrapga'),
        DeclareLaunchArgument('solve_budget_ms', default_value='10.0'),
        DeclareLaunchArgument('enforce_solve_budget', default_value='true'),
        DeclareLaunchArgument('duration_s', default_value='20.0'),
        DeclareLaunchArgument('dt', default_value='0.02'),
        DeclareLaunchArgument('horizon', default_value='20'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('enable_viewer', default_value='true'),
        DeclareLaunchArgument('output_prefix', default_value=''),
        RegisterEventHandler(
            OnProcessExit(
                target_action=closed_loop_node,
                on_exit=[EmitEvent(event=Shutdown(reason='closed-loop mpc node finished'))],
            )
        ),
        closed_loop_node,
        mujoco_executor_node,
    ])
''',
    )


def patch_matlab_script() -> None:
    ensure_text(
        DOC_GA_OCP / "AAA_cpp_mpc_bench.m",
        '''%% Reference tracking figure (bar version with mean-to-p95 line)
% Input: reviewer_revision_experiments/04_closed_loop_mpc_metrics/mujoco_closed_loop_metrics_summary.csv
% Output:
%   fig_reference_tracking.pdf
%   fig_reference_tracking.png

clear; clc; close all;

%% -------------------------------------------------
% Global style
%% -------------------------------------------------
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');

font_axes   = 22;   % tick labels
font_label  = 24;   % axis labels
font_title  = 24;   % subplot titles
font_legend = 22;   % legend
line_width  = 1.8;  % axes / bars / lines

%% -------------------------------------------------
% Read data
%% -------------------------------------------------
csv_file = '/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/04_closed_loop_mpc_metrics/mujoco_closed_loop_metrics_summary.csv';
T = readtable(csv_file);

% Keep only reference tracking rows
T = T(startsWith(string(T.case_name), "reference"), :);

%% -------------------------------------------------
% Canonicalize robot names
%% -------------------------------------------------
robot_canon = strings(height(T),1);
for i = 1:height(T)
    r = lower(string(T.robot(i)));
    if contains(r, "ur")
        robot_canon(i) = "UR10";
    elseif contains(r, "leap")
        robot_canon(i) = "Leap Hand";
    elseif contains(r, "tidy") || contains(r, "stanford")
        robot_canon(i) = "Stanford TidyBot";
    else
        robot_canon(i) = string(T.robot(i)); % fallback
    end
end
T.robot_canon = robot_canon;

robot_order_all = ["UR10","Leap Hand","Stanford TidyBot"];
present_robots = unique(T.robot_canon, 'stable');
robot_order = robot_order_all(ismember(robot_order_all, present_robots));
backend_order   = ["tetrapga","pinocchio","casadi"];
backend_display = {'TetraPGA','Pinocchio','CasADi'};

%% -------------------------------------------------
% Colors
%% -------------------------------------------------
C = [
    "#D95319"   % TetraPGA
    "#0072BD"   % Pinocchio
    "#EDB120"   % CasADi
];

%% -------------------------------------------------
% Extract values
%% -------------------------------------------------
nR = numel(robot_order);
nB = numel(backend_order);

RMSE   = nan(nR,nB);
Tmean  = nan(nR,nB);
Tp95   = nan(nR,nB);
Torque = nan(nR,nB);
Jerk   = nan(nR,nB);

for i = 1:nR
    for j = 1:nB
        idx = strcmpi(string(T.robot_canon), robot_order(i)) & ...
              strcmpi(string(T.backend), backend_order(j));
        row = T(idx,:);
        if ~isempty(row)
            RMSE(i,j)   = row.tracking_rmse(1);
            Tmean(i,j)  = row.solve_time_mean_ms(1);
            Tp95(i,j)   = row.solve_time_p95_ms(1);
            Torque(i,j) = row.torque_ratio_mean(1);
            Jerk(i,j)   = row.jerk_rms_norm_from_dq(1);
        end
    end
end

%% -------------------------------------------------
% Plot
%% -------------------------------------------------
fig = figure('Color','w','Position',[100 100 1300 840]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

xlabels = cellstr(robot_order);

%% -------------------------------------------------
% (a) Tracking RMSE
%% -------------------------------------------------
ax1 = nexttile;
hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
set(ax1, ...
    'FontName','Times New Roman', ...
    'FontSize',font_axes, ...
    'LineWidth',1.1);

b1 = bar(ax1, RMSE, 'grouped', 'LineWidth', line_width);
for j = 1:nB
    b1(j).FaceColor = C(j,:);
end

set(ax1, 'XTick', 1:nR, 'XTickLabel', xlabels);

ylabel(ax1, 'Tracking RMSE', ...
    'FontName','Times New Roman', ...
    'FontSize',font_label);

title(ax1, '(a)', ...
    'FontName','Times New Roman', ...
    'FontWeight','bold', ...
    'FontSize',font_title);

%% -------------------------------------------------
% (b) Solve time mean + line to p95
%% -------------------------------------------------
ax2 = nexttile;
hold(ax2,'on'); box(ax2,'on'); grid(ax2,'on');
set(ax2, ...
    'FontName','Times New Roman', ...
    'FontSize',font_axes, ...
    'LineWidth',1.1);

b2 = bar(ax2, Tmean, 'grouped', 'LineWidth', line_width);
for j = 1:nB
    b2(j).FaceColor = C(j,:);
end

set(ax2, 'XTick', 1:nR, 'XTickLabel', xlabels);

ylabel(ax2, 'Solve time mean (ms)', ...
    'FontName','Times New Roman', ...
    'FontSize',font_label);

title(ax2, '(b)', ...
    'FontName','Times New Roman', ...
    'FontWeight','bold', ...
    'FontSize',font_title);

% Draw line from mean to p95
for j = 1:nB
    xj = b2(j).XEndPoints;
    for i = 1:nR
        if ~isnan(Tmean(i,j)) && ~isnan(Tp95(i,j))
            % vertical line
            line(ax2, [xj(i) xj(i)], [Tmean(i,j) Tp95(i,j)], ...
                'Color', C(j,:), 'LineWidth', line_width);

            % top cap
            line(ax2, [xj(i)-0.04 xj(i)+0.04], [Tp95(i,j) Tp95(i,j)], ...
                'Color', C(j,:), 'LineWidth', line_width);
        end
    end
end

%% -------------------------------------------------
% (c) Mean torque ratio
%% -------------------------------------------------
ax3 = nexttile;
hold(ax3,'on'); box(ax3,'on'); grid(ax3,'on');
set(ax3, ...
    'FontName','Times New Roman', ...
    'FontSize',font_axes, ...
    'LineWidth',1.1);

b3 = bar(ax3, Torque, 'grouped', 'LineWidth', line_width);
for j = 1:nB
    b3(j).FaceColor = C(j,:);
end

set(ax3, 'XTick', 1:nR, 'XTickLabel', xlabels);

ylabel(ax3, 'Mean torque ratio', ...
    'FontName','Times New Roman', ...
    'FontSize',font_label);

title(ax3, '(c)', ...
    'FontName','Times New Roman', ...
    'FontWeight','bold', ...
    'FontSize',font_title);

%% -------------------------------------------------
% (d) Jerk / smoothness
%% -------------------------------------------------
ax4 = nexttile;
hold(ax4,'on'); box(ax4,'on'); grid(ax4,'on');
set(ax4, ...
    'FontName','Times New Roman', ...
    'FontSize',font_axes, ...
    'LineWidth',1.1);

b4 = bar(ax4, Jerk, 'grouped', 'LineWidth', line_width);
for j = 1:nB
    b4(j).FaceColor = C(j,:);
end

set(ax4, 'XTick', 1:nR, 'XTickLabel', xlabels);

ylabel(ax4, 'Jerk RMS norm', ...
    'FontName','Times New Roman', ...
    'FontSize',font_label);

title(ax4, '(d)', ...
    'FontName','Times New Roman', ...
    'FontWeight','bold', ...
    'FontSize',font_title);

%% -------------------------------------------------
% Legend
%% -------------------------------------------------
lgd = legend(ax4, backend_display, ...
    'Orientation','horizontal', ...
    'Location','northoutside', ...
    'Box','off', ...
    'FontName','Times New Roman', ...
    'FontSize',font_legend);
lgd.Layout.Tile = 'north';

%% -------------------------------------------------
% Optional: tighten y-limits
%% -------------------------------------------------
% ylim(ax2, [0, 1.1*max(Tp95(:))]);

%% -------------------------------------------------
% Export
%% -------------------------------------------------
exportgraphics(fig, 'fig_reference_tracking.pdf', 'ContentType','vector');
exportgraphics(fig, 'fig_reference_tracking.png', 'Resolution',300);
''',
    )


def main() -> int:
    patch_closed_loop_node()
    patch_joint_executor()
    patch_leap_launch()
    add_tidybot_config_and_launch()
    patch_matlab_script()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
