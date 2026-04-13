function metrics = compare_kqlmpc_edmd_logs(base_log_path, comp_log_path, opts)
% compare_kqlmpc_edmd_logs
% Compare baseline KQLMPC run and EDMD-compensated run from saved logger MAT files.
%
% Example:
%   compare_kqlmpc_edmd_logs( ...
%       'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data\baseline.mat', ...
%       'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data\comp.mat');
%
% Inputs:
%   base_log_path : baseline run MAT file path (residual.mode = 0)
%   comp_log_path : compensated run MAT file path (residual.mode = 1 or 2)
%
% Options:
%   opts.agent_id       : agent index in logger data, default 1
%   opts.figure_id      : figure number, default 701
%   opts.base_label     : default 'Baseline'
%   opts.comp_label     : default 'EDMD Compensation'
%   opts.time_range     : [t0 tf], default all
%   opts.save_png       : true/false, default false
%   opts.png_path       : png output path if save_png = true

arguments
    base_log_path (1,:) char
    comp_log_path (1,:) char
    opts.agent_id (1,1) double = 1
    opts.figure_id (1,1) double = 701
    opts.base_label (1,:) char = 'Baseline'
    opts.comp_label (1,:) char = 'EDMD Compensation'
    opts.time_range (1,2) double = [-inf inf]
    opts.save_png (1,1) logical = false
    opts.png_path (1,:) char = ''
end

base_log = load(base_log_path);
comp_log = load(comp_log_path);

base = extract_run_data(base_log.log, opts.agent_id, opts.time_range);
comp = extract_run_data(comp_log.log, opts.agent_id, opts.time_range);

metrics = struct();
metrics.base = compute_metrics(base);
metrics.comp = compute_metrics(comp);
metrics.improvement.pos_rmse_pct = safe_improvement(metrics.base.pos_rmse, metrics.comp.pos_rmse);
metrics.improvement.vel_rmse_pct = safe_improvement(metrics.base.vel_rmse, metrics.comp.vel_rmse);

fig = figure(opts.figure_id); clf(fig);
tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(base.t, base.pos_err_norm, 'LineWidth', 1.8); hold on;
plot(comp.t, comp.pos_err_norm, 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('||p - p_{ref}|| [m]');
title(sprintf('Position Error Norm | RMSE %.4f -> %.4f', ...
    metrics.base.pos_rmse, metrics.comp.pos_rmse));
legend(opts.base_label, opts.comp_label, 'Location', 'best');

nexttile;
plot(base.t, base.vel_err_norm, 'LineWidth', 1.8); hold on;
plot(comp.t, comp.vel_err_norm, 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('||v - v_{ref}|| [m/s]');
title(sprintf('Velocity Error Norm | RMSE %.4f -> %.4f', ...
    metrics.base.vel_rmse, metrics.comp.vel_rmse));
legend(opts.base_label, opts.comp_label, 'Location', 'best');

nexttile;
plot3(base.ref(:,1), base.ref(:,2), base.ref(:,3), 'k--', 'LineWidth', 1.2); hold on;
plot3(base.p(:,1), base.p(:,2), base.p(:,3), 'LineWidth', 1.6);
plot3(comp.p(:,1), comp.p(:,2), comp.p(:,3), 'LineWidth', 1.6);
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('3D Trajectory');
legend('Reference', opts.base_label, opts.comp_label, 'Location', 'best');

nexttile;
plot(base.t, base.u(:,1), 'LineWidth', 1.4); hold on;
plot(comp.t, comp.u(:,1), 'LineWidth', 1.6);
if ~isempty(comp.u_nom)
    plot(comp.t, comp.u_nom(:,1), '--', 'LineWidth', 1.2);
end
grid on;
xlabel('Time [s]');
ylabel('Thrust');
title('Input U1');
legend_entries = {opts.base_label, opts.comp_label};
if ~isempty(comp.u_nom)
    legend_entries{end+1} = 'Comp u_{nom}';
end
legend(legend_entries, 'Location', 'best');

nexttile;
plot(base.t, base.u(:,2:4), 'LineWidth', 1.2); hold on;
plot(comp.t, comp.u(:,2:4), '--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Torque Inputs');
title('Inputs U2-U4');
legend('Base U2', 'Base U3', 'Base U4', 'Comp U2', 'Comp U3', 'Comp U4', 'Location', 'bestoutside');

nexttile;
if ~isempty(comp.du_edmd)
    plot(comp.t, comp.du_edmd, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('\Delta u_{edmd}');
    title(sprintf('EDMD Compensation (mode=%d)', comp.residual_mode));
    legend('\Delta u_1', '\Delta u_2', '\Delta u_3', '\Delta u_4', 'Location', 'best');
else
    text(0.1, 0.5, 'No delta_u_edmd field found in compensated log.', 'FontSize', 12);
    axis off;
end

sgtitle(sprintf('KQLMPC Comparison | Pos Improve %.2f%% | Vel Improve %.2f%%', ...
    metrics.improvement.pos_rmse_pct, metrics.improvement.vel_rmse_pct));

disp('=== KQLMPC EDMD Comparison ===');
fprintf('Baseline position RMSE     : %.6f m\n', metrics.base.pos_rmse);
fprintf('Compensated position RMSE  : %.6f m\n', metrics.comp.pos_rmse);
fprintf('Position RMSE improvement  : %.2f %%\n', metrics.improvement.pos_rmse_pct);
fprintf('Baseline velocity RMSE     : %.6f m/s\n', metrics.base.vel_rmse);
fprintf('Compensated velocity RMSE  : %.6f m/s\n', metrics.comp.vel_rmse);
fprintf('Velocity RMSE improvement  : %.2f %%\n', metrics.improvement.vel_rmse_pct);
fprintf('Baseline max pos error     : %.6f m\n', metrics.base.pos_max);
fprintf('Compensated max pos error  : %.6f m\n', metrics.comp.pos_max);
fprintf('Comp residual mode         : %d\n', comp.residual_mode);

if opts.save_png
    png_path = opts.png_path;
    if isempty(png_path)
        png_path = fullfile(fileparts(comp_log_path), 'kqlmpc_edmd_compare.png');
    end
    exportgraphics(fig, png_path, 'Resolution', 180);
    fprintf('Saved figure: %s\n', png_path);
end
end

function run = extract_run_data(log_struct, agent_id, time_range)
t_all = double(log_struct.Data.t(:));
ids = find(t_all >= time_range(1) & t_all <= time_range(2));
if isempty(ids)
    ids = 1:numel(t_all);
end

agent = log_struct.Data.agent(agent_id);
n = numel(ids);

run = struct();
run.t = t_all(ids);
run.p = nan(n, 3);
run.v = nan(n, 3);
run.ref = nan(n, 3);
run.ref_v = nan(n, 3);
run.u = nan(n, 4);
run.u_nom = nan(n, 4);
run.du_edmd = nan(n, 4);
run.residual_mode = -1;

for i = 1:n
    k = ids(i);

    est = agent.estimator.result{1, k};
    ref = agent.reference.result{1, k};
    ctl = agent.controller.result{1, k};

    if isfield(est, 'state')
        run.p(i, :) = double(est.state.p(:))';
        run.v(i, :) = double(est.state.v(:))';
    end

    if isfield(ref, 'state')
        run.ref(i, :) = double(ref.state.p(:))';
        run.ref_v(i, :) = double(ref.state.v(:))';
    end

    if isfield(ctl, 'input')
        run.u(i, :) = reshape(double(ctl.input(1:4)), 1, []);
    else
        run.u(i, :) = reshape(double(agent.input{1, k}(1:4)), 1, []);
    end

    if isfield(ctl, 'u_nom')
        run.u_nom(i, :) = reshape(double(ctl.u_nom(1:4)), 1, []);
    end
    if isfield(ctl, 'delta_u_edmd')
        run.du_edmd(i, :) = reshape(double(ctl.delta_u_edmd(1:4)), 1, []);
    end
    if isfield(ctl, 'residual_mode')
        run.residual_mode = double(ctl.residual_mode);
    end
end

run.pos_err = run.p - run.ref;
run.vel_err = run.v - run.ref_v;
run.pos_err_norm = vecnorm(run.pos_err, 2, 2);
run.vel_err_norm = vecnorm(run.vel_err, 2, 2);

if all(all(isnan(run.u_nom)))
    run.u_nom = [];
end
if all(all(isnan(run.du_edmd)))
    run.du_edmd = [];
end
end

function metrics = compute_metrics(run)
metrics = struct();
metrics.pos_rmse = sqrt(mean(run.pos_err(:).^2, 'omitnan'));
metrics.vel_rmse = sqrt(mean(run.vel_err(:).^2, 'omitnan'));
metrics.pos_max = max(run.pos_err_norm, [], 'omitnan');
metrics.vel_max = max(run.vel_err_norm, [], 'omitnan');
end

function val = safe_improvement(base_val, comp_val)
if abs(base_val) < 1e-12
    val = 0;
else
    val = (base_val - comp_val) / base_val * 100;
end
end
