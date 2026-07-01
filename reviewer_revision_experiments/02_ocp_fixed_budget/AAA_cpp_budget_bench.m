%% Fixed-budget OCP summary plots
% Data sources:
%   UR10 and LEAP Hand:
%     /home/chenwh/Documents/GA4Ro-bench/B3146/
%   Stanford TidyBot:
%     reviewer_revision_experiments/02_ocp_fixed_budget/paper_scale_reviewed/
%
% Output:
%   fixed_budget_summary.pdf

clear; clc; close all;

%% ------------------------------------------------------------------------
% Global style
%% ------------------------------------------------------------------------
set(groot, 'defaultAxesFontName',   'Times New Roman');
set(groot, 'defaultTextFontName',   'Times New Roman');
set(groot, 'defaultLegendFontName', 'Times New Roman');

font_axes   = 22;   % tick labels
font_label  = 24;   % axis labels
font_title  = 24;   % subplot titles
font_legend = 22;   % legend
marker_size = 10;
line_width  = 3.0;

%% ------------------------------------------------------------------------
% File paths
%% ------------------------------------------------------------------------
script_dir = fileparts(mfilename('fullpath'));
if strlength(string(script_dir)) == 0
    script_dir = pwd;
end

old_root = "/home/chenwh/Documents/GA4Ro-bench/B3146";
ur10_csv = fullfile(old_root, "Crocoddyl_fddp_budget_bench_ur10_samples24_summary.csv");
leap_csv = fullfile(old_root, "Crocoddyl_fddp_budget_bench_leap_hand_samples24_summary.csv");
reviewed_csv = "/home/chenwh/ros2_ws/src/TetraPGA-preview/reviewer_revision_experiments/02_ocp_fixed_budget/paper_scale_reviewed/fixed_budget_paper_reviewed_summary.csv";

assert(isfile(ur10_csv), "Input CSV not found: %s", ur10_csv);
assert(isfile(leap_csv), "Input CSV not found: %s", leap_csv);
assert(isfile(reviewed_csv), "Input CSV not found: %s", reviewed_csv);

%% ------------------------------------------------------------------------
% Data and display settings
%% ------------------------------------------------------------------------
T_ur10 = readBudgetSummary(ur10_csv);
T_leap = readBudgetSummary(leap_csv);
T_reviewed = readBudgetSummary(reviewed_csv);
T_tidybot = T_reviewed(T_reviewed.scenario == "stanford_tidybot", :);

T = [T_ur10; T_leap; T_tidybot];
T.method = canonicalMethodNames(T.method);

scenario_order = ["ur10", "stanford_tidybot", "leap_hand"];
scenario_titles = ["UR10 (6DoF)", "Stanford TidyBot (10DoF)", "LEAP Hand (16DoF)"];
method_order = ["GA", "Pinocchio", "CasADi"];
method_labels = {'GA', 'Pinocchio', 'CasADi'};

color_map = containers.Map();
color_map('GA')        = '#D95319';
color_map('Pinocchio') = '#0072BD';
color_map('CasADi')    = '#EDB120';

marker_map = containers.Map();
marker_map('GA')        = 'o';
marker_map('Pinocchio') = 's';
marker_map('CasADi')    = 'd';

budget_ticks = [1 2 5 10 20 50 100 200];

required_vars = {'scenario', 'method', 'budget_ms', 'median_best_cost', ...
    'mean_terminal_q_error', 'success_rate'};
missing_vars = setdiff(required_vars, T.Properties.VariableNames(:));
assert(isempty(missing_vars), ...
    "CSV is missing required variables: %s", strjoin(string(missing_vars), ", "));

%% ------------------------------------------------------------------------
% Figure: Cost + Terminal Error + Success Rate
%% ------------------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [100 100 1150 980]);
tl = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

legend_handles = gobjects(numel(method_order), 1);

for c = 1:numel(scenario_order)
    scenario = scenario_order(c);
    Ts = T(T.scenario == scenario, :);
    assert(~isempty(Ts), "No rows found for scenario=%s", scenario);

    % Row 1: median best cost.
    ax1 = nexttile(tl, c);
    hold(ax1, 'on'); box(ax1, 'on'); grid(ax1, 'on');
    setupBudgetAxes(ax1, font_axes, budget_ticks, true);
    set(ax1, 'YScale', 'log');
    for m = 1:numel(method_order)
        h = plotMetric(ax1, Ts, method_order(m), 'median_best_cost', ...
            color_map(char(method_order(m))), marker_map(char(method_order(m))), ...
            line_width, marker_size);
        if c == 1
            legend_handles(m) = h;
        end
    end
    title(ax1, scenario_titles(c), ...
        'FontName', 'Times New Roman', ...
        'FontSize', font_title, ...
        'FontWeight', 'bold');
    if c == 1
        ylabel(ax1, 'Median best cost', 'FontSize', font_label);
    end

    % Row 2: terminal q error.
    ax2 = nexttile(tl, c + 3);
    hold(ax2, 'on'); box(ax2, 'on'); grid(ax2, 'on');
    setupBudgetAxes(ax2, font_axes, budget_ticks, true);
    set(ax2, 'YScale', 'log');
    for m = 1:numel(method_order)
        plotMetric(ax2, Ts, method_order(m), 'mean_terminal_q_error', ...
            color_map(char(method_order(m))), marker_map(char(method_order(m))), ...
            line_width, marker_size);
    end
    if c == 1
        ylabel(ax2, 'Mean terminal error', 'FontSize', font_label);
    end

    % Row 3: success rate.
    ax3 = nexttile(tl, c + 6);
    hold(ax3, 'on'); box(ax3, 'on'); grid(ax3, 'on');
    setupBudgetAxes(ax3, font_axes, budget_ticks, false);
    ylim(ax3, [0 1.05]);
    yticks(ax3, 0:0.25:1);
    for m = 1:numel(method_order)
        plotMetric(ax3, Ts, method_order(m), 'success_rate', ...
            color_map(char(method_order(m))), marker_map(char(method_order(m))), ...
            line_width, marker_size);
    end
    xlabel(ax3, 'Budget (ms)', 'FontSize', font_label);
    if c == 1
        ylabel(ax3, 'Success rate', 'FontSize', font_label);
    end
end

lgd = legend(legend_handles, method_labels, ...
    'Orientation', 'horizontal', ...
    'Location', 'northoutside', ...
    'Box', 'off', ...
    'FontName', 'Times New Roman', ...
    'FontSize', font_legend);
lgd.Layout.Tile = 'north';

out_file = fullfile(script_dir, 'fixed_budget_summary.pdf');
exportgraphics(fig, out_file, 'ContentType', 'vector');
fprintf('Wrote %s\n', out_file);

%% ------------------------------------------------------------------------
% Local functions
%% ------------------------------------------------------------------------
function T = readBudgetSummary(csv_file)
    opts = detectImportOptions(csv_file, 'VariableNamingRule', 'preserve');
    opts = setvartype(opts, {'scenario', 'method'}, 'string');
    T = readtable(csv_file, opts);
end

function method = canonicalMethodNames(method)
    method = string(method);
    method(strcmpi(method, "GA4Ro") | strcmpi(method, "TetraPGA")) = "GA";
    method(strcmpi(method, "Casadi")) = "CasADi";
end

function setupBudgetAxes(ax, font_axes, budget_ticks, hide_xticklabels)
    set(ax, ...
        'XScale', 'log', ...
        'FontSize', font_axes, ...
        'LineWidth', 1.1, ...
        'FontName', 'Times New Roman', ...
        'XTick', budget_ticks, ...
        'XMinorTick', 'off');
    xlim(ax, [0.9 220]);
    if hide_xticklabels
        set(ax, 'XTickLabel', []);
    else
        set(ax, 'XTickLabel', compose('%g', budget_ticks));
    end
end

function h = plotMetric(ax, T, method, field_name, color, marker, line_width, marker_size)
    Tm = T(T.method == method, :);
    assert(~isempty(Tm), "No rows found for method=%s", method);
    [~, ord] = sort(Tm.budget_ms);
    Tm = Tm(ord, :);
    h = plot(ax, Tm.budget_ms, Tm.(field_name), ...
        'LineStyle', '-', ...
        'Marker', marker, ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'Color', color, ...
        'MarkerFaceColor', 'w');
end
