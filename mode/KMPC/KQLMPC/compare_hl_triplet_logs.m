function metrics = compare_hl_triplet_logs(nominal_log_path, error_log_path, comp_log_path, opts)
% compare_hl_triplet_logs
% Compare nominal HLC, HLC+error, and HLC+error+EDMD runs from saved logs.

if nargin < 4 || ~isstruct(opts)
    opts = struct();
end

opts = set_default_local(opts, 'agent_id', 1);
opts = set_default_local(opts, 'state_source', 'plant');
opts = set_default_local(opts, 'phase', 'tfl');
opts = set_default_local(opts, 'time_range', [-inf, inf]);
opts = set_default_local(opts, 'nominal_label', 'HLC');
opts = set_default_local(opts, 'error_label', 'HLC + error');
opts = set_default_local(opts, 'comp_label', 'HLC + error + EDMD');
opts = set_default_local(opts, 'save_outputs', true);
opts = set_default_local(opts, 'output_dir', '');
opts = set_default_local(opts, 'output_stem', 'hl_triplet_compare');
opts = set_default_local(opts, 'figure_visible', 'off');
opts = set_default_local(opts, 'close_figures', true);

nominal_data = load(nominal_log_path);
error_data = load(error_log_path);
comp_data = load(comp_log_path);

nominal = extract_run_data(nominal_data.log, opts.agent_id, opts.state_source, opts.phase, opts.time_range);
error_run = extract_run_data(error_data.log, opts.agent_id, opts.state_source, opts.phase, opts.time_range);
comp = extract_run_data(comp_data.log, opts.agent_id, opts.state_source, opts.phase, opts.time_range);

metrics = struct();
metrics.nominal = compute_pos_metrics(nominal);
metrics.error = compute_pos_metrics(error_run);
metrics.comp = compute_pos_metrics(comp);
metrics.improvement_error_to_comp.xyz_pct = safe_improvement_vec(metrics.error.rmse_xyz, metrics.comp.rmse_xyz);
metrics.improvement_error_to_comp.norm_pct = safe_improvement(metrics.error.rmse_norm, metrics.comp.rmse_norm);
metrics.phase = opts.phase;
metrics.state_source = opts.state_source;

font_name = 'Times New Roman';
font_size = 14;
line_width = 1.8;
ref_line_width = 2.2;
ref_color = [0.35, 0.35, 0.35];
nominal_color = [0.10, 0.35, 0.90];
error_color = [0.86, 0.24, 0.12];
comp_color = [0.00, 0.62, 0.32];

fig_pos = figure('Visible', opts.figure_visible, 'Color', 'w', 'Position', [120, 80, 1080, 820]);
tl_pos = tiledlayout(fig_pos, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
h_ref = plot3(nominal.ref(:, 1), nominal.ref(:, 2), nominal.ref(:, 3), '--', ...
    'Color', ref_color, 'LineWidth', ref_line_width); hold on;
h_nom = plot3(nominal.p(:, 1), nominal.p(:, 2), nominal.p(:, 3), '-', ...
    'Color', nominal_color, 'LineWidth', line_width, ...
    'Marker', 'o', 'MarkerIndices', marker_ids(size(nominal.p, 1), 48), ...
    'MarkerSize', 4.5, 'MarkerFaceColor', 'none');
h_err = plot3(error_run.p(:, 1), error_run.p(:, 2), error_run.p(:, 3), '-', ...
    'Color', error_color, 'LineWidth', line_width, ...
    'Marker', 's', 'MarkerIndices', marker_ids(size(error_run.p, 1), 48), ...
    'MarkerSize', 4.5, 'MarkerFaceColor', 'none');
h_comp = plot3(comp.p(:, 1), comp.p(:, 2), comp.p(:, 3), '-', ...
    'Color', comp_color, 'LineWidth', line_width, ...
    'Marker', '^', 'MarkerIndices', marker_ids(size(comp.p, 1), 48), ...
    'MarkerSize', 4.5, 'MarkerFaceColor', 'none');
grid on;
axis equal;
xlabel('x [m]', 'FontName', font_name, 'FontSize', font_size);
ylabel('y [m]', 'FontName', font_name, 'FontSize', font_size);
zlabel('z [m]', 'FontName', font_name, 'FontSize', font_size);
title('p1-p2-p3', 'FontName', font_name, 'FontSize', font_size + 2);
set(gca, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
view([-38, 24]);
legend([h_ref, h_nom, h_err, h_comp], ...
    {'Reference', opts.nominal_label, opts.error_label, opts.comp_label}, ...
    'Location', 'northeastoutside', 'FontName', font_name, 'FontSize', font_size - 1);

nexttile;
plot(nominal.t, nominal.ref(:, 1), '--', 'Color', ref_color, 'LineWidth', ref_line_width); hold on;
plot(nominal.t, nominal.p(:, 1), '-', 'Color', nominal_color, 'LineWidth', line_width);
plot(error_run.t, error_run.p(:, 1), '-', 'Color', error_color, 'LineWidth', line_width);
plot(comp.t, comp.p(:, 1), '-', 'Color', comp_color, 'LineWidth', line_width);
grid on;
xlabel('Time [s]', 'FontName', font_name, 'FontSize', font_size);
ylabel('p1 [m]', 'FontName', font_name, 'FontSize', font_size);
title(sprintf('p1 | %.4f / %.4f / %.4f', ...
    metrics.nominal.rmse_xyz(1), metrics.error.rmse_xyz(1), metrics.comp.rmse_xyz(1)), ...
    'FontName', font_name, 'FontSize', font_size + 1);
set(gca, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
legend('Reference', opts.nominal_label, opts.error_label, opts.comp_label, ...
    'Location', 'best', 'FontName', font_name, 'FontSize', font_size - 1);

nexttile;
plot(nominal.t, nominal.ref(:, 2), '--', 'Color', ref_color, 'LineWidth', ref_line_width); hold on;
plot(nominal.t, nominal.p(:, 2), '-', 'Color', nominal_color, 'LineWidth', line_width);
plot(error_run.t, error_run.p(:, 2), '-', 'Color', error_color, 'LineWidth', line_width);
plot(comp.t, comp.p(:, 2), '-', 'Color', comp_color, 'LineWidth', line_width);
grid on;
xlabel('Time [s]', 'FontName', font_name, 'FontSize', font_size);
ylabel('p2 [m]', 'FontName', font_name, 'FontSize', font_size);
title(sprintf('p2 | %.4f / %.4f / %.4f', ...
    metrics.nominal.rmse_xyz(2), metrics.error.rmse_xyz(2), metrics.comp.rmse_xyz(2)), ...
    'FontName', font_name, 'FontSize', font_size + 1);
set(gca, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
legend('Reference', opts.nominal_label, opts.error_label, opts.comp_label, ...
    'Location', 'best', 'FontName', font_name, 'FontSize', font_size - 1);

nexttile;
plot(nominal.t, nominal.ref(:, 3), '--', 'Color', ref_color, 'LineWidth', ref_line_width); hold on;
plot(nominal.t, nominal.p(:, 3), '-', 'Color', nominal_color, 'LineWidth', line_width);
plot(error_run.t, error_run.p(:, 3), '-', 'Color', error_color, 'LineWidth', line_width);
plot(comp.t, comp.p(:, 3), '-', 'Color', comp_color, 'LineWidth', line_width);
grid on;
xlabel('Time [s]', 'FontName', font_name, 'FontSize', font_size);
ylabel('p3 [m]', 'FontName', font_name, 'FontSize', font_size);
title(sprintf('p3 | %.4f / %.4f / %.4f', ...
    metrics.nominal.rmse_xyz(3), metrics.error.rmse_xyz(3), metrics.comp.rmse_xyz(3)), ...
    'FontName', font_name, 'FontSize', font_size + 1);
set(gca, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
legend('Reference', opts.nominal_label, opts.error_label, opts.comp_label, ...
    'Location', 'best', 'FontName', font_name, 'FontSize', font_size - 1);

sgtitle(tl_pos, sprintf(['HLC Triplet Comparison (%s state, phase=%s) | ', ...
    '||p|| RMSE %.4f / %.4f / %.4f | Improve %.2f%%'], ...
    opts.state_source, opts.phase, ...
    metrics.nominal.rmse_norm, metrics.error.rmse_norm, metrics.comp.rmse_norm, ...
    metrics.improvement_error_to_comp.norm_pct), ...
    'Interpreter', 'none', 'FontName', font_name, 'FontSize', font_size + 3);

fig_u = figure('Visible', opts.figure_visible, 'Color', 'w', 'Position', [140, 100, 1080, 820]);
tl_u = tiledlayout(fig_u, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for ch = 1:4
    nexttile;
    plot(nominal.t, nominal.u(:, ch), '-', 'Color', nominal_color, 'LineWidth', line_width); hold on;
    plot(error_run.t, error_run.u(:, ch), '-', 'Color', error_color, 'LineWidth', line_width);
    plot(comp.t, comp.u(:, ch), '-', 'Color', comp_color, 'LineWidth', line_width);
    grid on;
    xlabel('Time [s]', 'FontName', font_name, 'FontSize', font_size);
    ylabel(sprintf('u%d', ch), 'FontName', font_name, 'FontSize', font_size);
    title(sprintf('Input u%d', ch), 'FontName', font_name, 'FontSize', font_size + 1);
    set(gca, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
    legend(opts.nominal_label, opts.error_label, opts.comp_label, ...
        'Location', 'best', 'FontName', font_name, 'FontSize', font_size - 1);
end

sgtitle(tl_u, sprintf('Input Comparison (%s)', opts.phase), ...
    'Interpreter', 'none', 'FontName', font_name, 'FontSize', font_size + 3);

rmse_table = table( ...
    ["p1"; "p2"; "p3"; "pnorm"], ...
    [metrics.nominal.rmse_xyz(:); metrics.nominal.rmse_norm], ...
    [metrics.error.rmse_xyz(:); metrics.error.rmse_norm], ...
    [metrics.comp.rmse_xyz(:); metrics.comp.rmse_norm], ...
    [metrics.improvement_error_to_comp.xyz_pct(:); metrics.improvement_error_to_comp.norm_pct], ...
    'VariableNames', {'Axis', 'NominalRMSE', 'ErrorRMSE', 'CompRMSE', 'CompImproveVsErrorPct'});
metrics.rmse_table = rmse_table;

disp('=== HLC Triplet Comparison ===');
disp(rmse_table);

if opts.save_outputs
    output_dir = opts.output_dir;
    if isempty(output_dir)
        output_dir = fileparts(comp_log_path);
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    pos_png = fullfile(output_dir, sprintf('%s_position.png', opts.output_stem));
    input_png = fullfile(output_dir, sprintf('%s_input.png', opts.output_stem));
    csv_path = fullfile(output_dir, sprintf('%s_rmse.csv', opts.output_stem));

    drawnow;
    print(fig_pos, pos_png, '-dpng', '-r300');
    print(fig_u, input_png, '-dpng', '-r300');
    writetable(rmse_table, csv_path);

    metrics.position_png = pos_png;
    metrics.input_png = input_png;
    metrics.rmse_csv = csv_path;

    fprintf('Saved position figure: %s\n', pos_png);
    fprintf('Saved input figure   : %s\n', input_png);
    fprintf('Saved RMSE csv       : %s\n', csv_path);
end

if opts.close_figures
    close(fig_pos);
    close(fig_u);
end
end

function run = extract_run_data(log_struct, agent_id, state_source, phase_chars, time_range)
t_all = double(log_struct.Data.t(:));
phase_all = char(log_struct.Data.phase(:).');
last_idx = find(log_struct.Data.phase, 1, 'last');
if isempty(last_idx)
    last_idx = numel(t_all);
end

t_all = t_all(1:last_idx);
phase_all = lower(phase_all(1:last_idx)).';
mask_time = t_all >= time_range(1) & t_all <= time_range(2);

if isempty(phase_chars)
    mask_phase = true(size(mask_time));
else
    mask_phase = ismember(phase_all, lower(phase_chars));
end

ids = find(mask_time & mask_phase);
if isempty(ids)
    error('No samples found for phase=%s and time range [%.3f, %.3f].', ...
        phase_chars, time_range(1), time_range(2));
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
run.u_error = nan(n, 4);
run.du_edmd = nan(n, 4);

for i = 1:n
    k = ids(i);
    st = get_state_entry(agent, state_source, k);
    ref = agent.reference.result{1, k}.state;
    ctl = agent.controller.result{1, k};

    run.p(i, :) = double(st.p(:))';
    run.v(i, :) = double(st.v(:))';
    run.ref(i, :) = extract_ref_p(ref)';
    run.ref_v(i, :) = extract_ref_v(ref)';

    if isfield(ctl, 'input')
        run.u(i, :) = reshape(double(ctl.input(1:4)), 1, []);
    else
        run.u(i, :) = reshape(double(agent.input{1, k}(1:4)), 1, []);
    end

    if isfield(ctl, 'u_nom')
        run.u_nom(i, :) = reshape(double(ctl.u_nom(1:4)), 1, []);
    end
    if isfield(ctl, 'u_error')
        run.u_error(i, :) = reshape(double(ctl.u_error(1:4)), 1, []);
    end
    if isfield(ctl, 'delta_u_edmd')
        run.du_edmd(i, :) = reshape(double(ctl.delta_u_edmd(1:4)), 1, []);
    end
end
end

function st = get_state_entry(agent, state_source, k)
switch lower(char(state_source))
    case 'plant'
        st = agent.plant.result{1, k}.state;
    case 'estimator'
        st = agent.estimator.result{1, k}.state;
    otherwise
        error('Unsupported state_source: %s', state_source);
end
end

function p = extract_ref_p(ref)
if (isstruct(ref) && isfield(ref, 'p')) || isprop(ref, 'p')
    p = double(ref.p(:));
elseif ((isstruct(ref) && isfield(ref, 'xd')) || isprop(ref, 'xd')) && numel(ref.xd) >= 3
    p = double(ref.xd(1:3));
else
    p = zeros(3, 1);
end
end

function v = extract_ref_v(ref)
if (isstruct(ref) && isfield(ref, 'v')) || isprop(ref, 'v')
    v = double(ref.v(:));
elseif ((isstruct(ref) && isfield(ref, 'xd')) || isprop(ref, 'xd')) && numel(ref.xd) >= 7
    v = double(ref.xd(5:7));
else
    v = zeros(3, 1);
end
end

function metrics = compute_pos_metrics(run)
err = run.p - run.ref;
metrics = struct();
metrics.rmse_xyz = sqrt(mean(err.^2, 1, 'omitnan'));
metrics.rmse_norm = sqrt(mean(sum(err.^2, 2), 'omitnan'));
end

function idx = marker_ids(n, stride)
if n <= 0
    idx = 1;
else
    idx = 1:max(1, stride):n;
end
end

function val = safe_improvement(base_val, comp_val)
if abs(base_val) < 1e-12
    val = 0;
else
    val = (base_val - comp_val) / base_val * 100;
end
end

function vec = safe_improvement_vec(base_vec, comp_vec)
vec = zeros(size(base_vec));
for i = 1:numel(base_vec)
    vec(i) = safe_improvement(base_vec(i), comp_vec(i));
end
end

function s = set_default_local(s, key, value)
if ~isfield(s, key) || isempty(s.(key))
    s.(key) = value;
end
end
