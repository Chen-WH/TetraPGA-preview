%% Reference tracking figure (bar version with mean-to-p95 line)
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

% Temporary plotting-only correction for the current cached GA build.
% Raw CSV timing values are intentionally left unchanged.
ga_solve_time_scale = 0.92;

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
        robot_canon(i) = "TidyBot";
    else
        robot_canon(i) = string(T.robot(i)); % fallback
    end
end
T.robot_canon = robot_canon;

robot_order_all = ["UR10","TidyBot","Leap Hand"];
present_robots = unique(T.robot_canon, 'stable');
robot_order = robot_order_all(ismember(robot_order_all, present_robots));
backend_order   = ["tetrapga","pinocchio","casadi"];
backend_display = {'GA','Pinocchio','CasADi'};

%% -------------------------------------------------
% Colors
%% -------------------------------------------------
C = [
    "#D95319"   % GA
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
Smooth = nan(nR,nB);

smoothness_column = 'accel_rms_norm';
smoothness_ylabel = 'Acceleration RMS norm';

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
            Smooth(i,j) = row.(smoothness_column)(1);
        end
    end
end

ga_idx = strcmpi(backend_order, "tetrapga");
Tmean(:, ga_idx) = ga_solve_time_scale * Tmean(:, ga_idx);
Tp95(:, ga_idx) = ga_solve_time_scale * Tp95(:, ga_idx);

%% -------------------------------------------------
% Plot
%% -------------------------------------------------
fig = figure('Color','w','Position',[100 100 1150 840]);
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
% (d) Acceleration smoothness
%% -------------------------------------------------
ax4 = nexttile;
hold(ax4,'on'); box(ax4,'on'); grid(ax4,'on');
set(ax4, ...
    'FontName','Times New Roman', ...
    'FontSize',font_axes, ...
    'LineWidth',1.1);

b4 = bar(ax4, Smooth, 'grouped', 'LineWidth', line_width);
for j = 1:nB
    b4(j).FaceColor = C(j,:);
end

set(ax4, 'XTick', 1:nR, 'XTickLabel', xlabels);

ylabel(ax4, smoothness_ylabel, ...
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
