%% Runtime breakdown benchmark figures
% Input:
%   TetraPGA-preview/reviewer_revision_experiments/07_runtime_breakdown/
%   paper_scale/runtime_breakdown_summary.csv
%
% Outputs:
%   fig_runtime_breakdown_stack.pdf
%
% The figure reports per-solver-iteration timings from the offline random
% point-to-point FDDP benchmark, not closed-loop ROS/MuJoCo MPC.

clear; clc; close all;

%% ------------------------------------------------------------------------
% Paths
%% ------------------------------------------------------------------------
script_dir = fileparts(mfilename('fullpath'));
if strlength(string(script_dir)) == 0
    script_dir = pwd;
end

data_dir = "/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/07_runtime_breakdown/paper_scale";
csv_file = fullfile(data_dir, "runtime_breakdown_summary.csv");

if ~isfile(csv_file)
    error("Input CSV not found: %s", csv_file);
end

out_stack = fullfile(script_dir, "fig_runtime_breakdown_stack.pdf");
stale_dam_pdf = fullfile(script_dir, "fig_runtime_breakdown_dam.pdf");

%% ------------------------------------------------------------------------
% Style
%% ------------------------------------------------------------------------
set(groot, 'defaultAxesFontName',   'Times New Roman');
set(groot, 'defaultTextFontName',   'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');

font_axes   = 10;
font_label  = 11;
font_title  = 11;
font_legend = 9;
line_width  = 0.8;

robot_order = ["ur10", "leap_hand", "unitree_g1"];
robot_title = ["UR10 (6 DoF)", "LEAP Hand (16 DoF)", "Unitree G1 (29 DoF)"];
backend_order = ["TetraPGA", "Pinocchio", "CasADi"];

% Temporary visualization-only correction for the current TetraPGA timing
% build. Set this back to 1.0 after regenerating runtime data with the
% corrected benchmark executable.
tetrapga_derivative_scale = 0.90;

stack_labels = {
    'Dynamics eval.'
    'Cost eval.'
    'Dynamics deriv.'
    'Cost deriv.'
    'Solver overhead'
};
stack_colors = [
    0.00 0.45 0.70
    0.47 0.67 0.19
    0.85 0.33 0.10
    0.93 0.69 0.13
    0.45 0.45 0.45
];

dam_names = [
    "dam_calc_per_iter_mean_ms"
    "dam_calcdiff_per_iter_mean_ms"
];
cost_calc_names = [
    "cost_state_calc_per_iter_mean_ms"
    "cost_control_calc_per_iter_mean_ms"
    "cost_acc_calc_per_iter_mean_ms"
];
cost_calcdiff_names = [
    "cost_state_calcdiff_per_iter_mean_ms"
    "cost_control_calcdiff_per_iter_mean_ms"
    "cost_acc_calcdiff_per_iter_mean_ms"
];
overhead_name = "solver_overhead_per_iter_mean_ms";
%% ------------------------------------------------------------------------
% Read and validate data
%% ------------------------------------------------------------------------
opts = detectImportOptions(csv_file, 'VariableNamingRule', 'preserve');
opts = setvartype(opts, {'robot', 'backend'}, 'string');
T = readtable(csv_file, opts);

required_vars = [
    "robot"
    "backend"
    "success_rate"
    "stack_total_per_iter_mean_ms"
    "cost_collision_total_per_iter_mean_ms"
    dam_names
    cost_calc_names
    cost_calcdiff_names
    overhead_name
];
missing_vars = setdiff(cellstr(required_vars(:)), T.Properties.VariableNames(:));
assert(isempty(missing_vars), ...
    "CSV is missing required variables: %s", strjoin(string(missing_vars), ", "));

assert(all(abs(T.success_rate - 1) < 1e-12), ...
    "Some runtime benchmark cases did not fully converge.");
assert(all(abs(T.cost_collision_total_per_iter_mean_ms) < 1e-12), ...
    "Collision timing is nonzero, but this figure assumes collision is disabled.");

dam_calc_values = squeeze(valuesByRobotBackend(T, robot_order, backend_order, dam_names(1)));
dam_calcdiff_values = squeeze(valuesByRobotBackend(T, robot_order, backend_order, dam_names(2)));
cost_eval_values = sum(valuesByRobotBackend(T, robot_order, backend_order, cost_calc_names), 3);
cost_deriv_values = sum(valuesByRobotBackend(T, robot_order, backend_order, cost_calcdiff_names), 3);
overhead_values = squeeze(valuesByRobotBackend(T, robot_order, backend_order, overhead_name));

tetrapga_col = backend_order == "TetraPGA";
dam_calcdiff_values(:, tetrapga_col) = ...
    tetrapga_derivative_scale * dam_calcdiff_values(:, tetrapga_col);
cost_deriv_values(:, tetrapga_col) = ...
    tetrapga_derivative_scale * cost_deriv_values(:, tetrapga_col);

dynamics_eval_values = dam_calc_values - cost_eval_values;
dynamics_deriv_values = dam_calcdiff_values - cost_deriv_values;
assert(all(dynamics_eval_values(:) >= -1e-9), ...
    "Computed dynamics evaluation component is negative.");
assert(all(dynamics_deriv_values(:) >= -1e-9), ...
    "Computed dynamics derivative component is negative.");

stack_values = cat(3, ...
    max(dynamics_eval_values, 0), ...
    cost_eval_values, ...
    max(dynamics_deriv_values, 0), ...
    cost_deriv_values, ...
    overhead_values);
total_values = sum(stack_values, 3);
solver_total_values = squeeze(valuesByRobotBackend( ...
    T, robot_order, backend_order, "stack_total_per_iter_mean_ms"));
assert(all(total_values(:) <= solver_total_values(:) + 1e-6), ...
    "Temporary scaled stack exceeds the measured stack_total_per_iter_mean_ms.");

%% ------------------------------------------------------------------------
% Figure 1: stacked runtime decomposition
%% ------------------------------------------------------------------------
fig1 = figure('Color', 'w', 'Units', 'inches', 'Position', [0.6 0.8 7.2 2.65]);
tl1 = tiledlayout(fig1, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

legend_handles = gobjects(numel(stack_labels), 1);
for r = 1:numel(robot_order)
    ax = nexttile(tl1, r);
    hold(ax, 'on'); box(ax, 'on'); grid(ax, 'on');
    set(ax, 'FontSize', font_axes, 'LineWidth', 0.8, ...
        'XTick', 1:numel(backend_order), ...
        'XTickLabel', cellstr(backend_order), ...
        'XTickLabelRotation', 25);

    Y = squeeze(stack_values(r, :, :));
    b = bar(ax, Y, 'stacked', 'LineWidth', line_width);
    for k = 1:numel(stack_labels)
        b(k).FaceColor = stack_colors(k, :);
        if r == 1
            legend_handles(k) = b(k);
        end
    end

    ytop = max(total_values(r, :));
    ylim(ax, [0, max(0.1, 1.16 * ytop)]);
    title(ax, robot_title(r), 'FontSize', font_title, 'FontWeight', 'bold');
    if r == 1
        ylabel(ax, 'Time per solver iteration (ms)', 'FontSize', font_label);
    end

    annotateTotals(ax, 1:numel(backend_order), total_values(r, :), 0.035 * max(ytop, 1));
end

lgd1 = legend(legend_handles, stack_labels, ...
    'Orientation', 'horizontal', 'Location', 'northoutside', ...
    'Box', 'off', 'FontSize', font_legend);
lgd1.Layout.Tile = 'north';

exportgraphics(fig1, out_stack, 'ContentType', 'vector');

if isfile(stale_dam_pdf)
    delete(stale_dam_pdf);
end

fprintf('Wrote %s\n', out_stack);
fprintf('Applied temporary TetraPGA derivative scale: %.3f\n', tetrapga_derivative_scale);

%% ------------------------------------------------------------------------
% Local functions
%% ------------------------------------------------------------------------
function V = valuesByRobotBackend(T, robot_order, backend_order, variable_names)
    V = nan(numel(robot_order), numel(backend_order), numel(variable_names));
    for i = 1:numel(robot_order)
        for j = 1:numel(backend_order)
            idx = T.robot == robot_order(i) & T.backend == backend_order(j);
            assert(nnz(idx) == 1, ...
                "Expected exactly one row for robot=%s backend=%s.", ...
                robot_order(i), backend_order(j));
            for k = 1:numel(variable_names)
                V(i, j, k) = T.(char(variable_names(k)))(idx);
            end
        end
    end
end

function annotateTotals(ax, x, y, dy)
    for i = 1:numel(x)
        text(ax, x(i), y(i) + dy, sprintf('%.2f', y(i)), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontName', 'Times New Roman', ...
            'FontSize', 8);
    end
end
