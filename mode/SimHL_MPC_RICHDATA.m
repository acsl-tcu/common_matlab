%% Rich-data batch configuration for HL-MPC / EDMD residual collection
%% （KQ版RICHDATAの検証済みフレームを流用、plant=Model_Quat13、controller=HL_MPC_EDMD_MEC）
if exist('app', 'var') == 1
    modeType = 1;
    script_path = mfilename('fullpath');
    if isempty(script_path)
        tmp = matlab.desktop.editor.getActive;
        script_path = tmp.Filename;
    end
else
    script_path = mfilename('fullpath');
    if isempty(script_path)
        tmp = matlab.desktop.editor.getActive;
        script_path = tmp.Filename;
    end
    modeType = 0;
end

dir = fileparts(script_path);
project_root = erase(dir, '\mode');

if ~strcmpi(pwd, project_root)
    cd(project_root);
end

if ~contains(path, dir)
        [~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
        cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
        close all hidden; clc;
        userpath('clear');
end

if ~exist(fullfile('Data', 'Sim_data'), 'dir')
    mkdir(fullfile('Data', 'Sim_data'));
end

dt = get_base_value('dt', 0.025);
te = get_base_value('te', 120);
run_index = get_base_value('run_index', 1);
run_indices = get_base_value('run_indices', []);
preview_reference = get_base_value('preview_reference', 0);
execution_mode = lower(string(get_base_value('execution_mode', 'batch')));
autorun_batch = logical(get_base_value('autorun_batch', ~modeType));
save_prefix = string(get_base_value('save_prefix', "HL_"));
save_separate = logical(get_base_value('save_separate', 0));
silent_batch = logical(get_base_value('silent_batch', ~modeType));
preview_duration = get_base_value('preview_duration', []);
preview_save_figure = logical(get_base_value('preview_save_figure', 0));
preview_figure_prefix = string(get_base_value('preview_figure_prefix', "HL_preview_"));
arming_steps = max(1, round(get_base_value('arming_steps', 1)));
takeoff_duration_in = get_base_value('takeoff_duration', []);
flight_duration_in = get_base_value('flight_duration', []);
landing_duration_in = get_base_value('landing_duration', []);

% ===== 3条件比較のための注入変数 =====
% case_type: 'nominal' | 'degraded' | 'compensated'
%   nominal     : 標称質量慣性 + 激励OFF + 補償なし（基準）
%   degraded    : 変質量慣性  + 激励ON  + 補償なし（劣化）
%   compensated : 変質量慣性  + 激励ON  + EDMD補償ON（回復）
% 個別に上書きしたい場合は use_plant_perturb / use_excitation / use_compensation を直接指定。
case_type = lower(string(get_base_value('case_type', 'degraded')));
switch case_type
    case "nominal"
        def_perturb = 0; def_excite = 0; def_comp = 0;
    case "compensated"
        def_perturb = 1; def_excite = 1; def_comp = 1;
    otherwise  % degraded
        def_perturb = 1; def_excite = 1; def_comp = 0;
end
use_plant_perturb = logical(get_base_value('use_plant_perturb', def_perturb));
use_excitation    = logical(get_base_value('use_excitation',    def_excite));
use_compensation  = logical(get_base_value('use_compensation',  def_comp));
edmd_model_file   = string(get_base_value('edmd_model_file', ""));
hlmpc_mpc_profile = string(get_base_value('hlmpc_mpc_profile', "mismatch"));
hlmpc_ref_override = get_base_value('hlmpc_ref_override', []);

% setup 関数へ渡すための条件束
case_opts = struct( ...
    'case_type', case_type, ...
    'use_plant_perturb', use_plant_perturb, ...
    'use_excitation', use_excitation, ...
    'use_compensation', use_compensation, ...
    'edmd_model_file', edmd_model_file, ...
    'hlmpc_mpc_profile', hlmpc_mpc_profile, ...
    'ref_override', hlmpc_ref_override);

cfg = rich_dataset_config();

if modeType
    schedule = resolve_phase_schedule(dt, te, arming_steps, ...
        takeoff_duration_in, flight_duration_in, landing_duration_in);
    [time, logger, motive, agent, dataset_meta, run_index_wrapped] = ...
        setup_richdata_case(modeType, dt, schedule.total_duration, schedule.sample_count, run_index, preview_reference, case_opts);
    in_prog_func = @(app) post(app);
    post_func = @(app) post(app);
    assignin('base', 'edmd_dataset_meta', dataset_meta);
    assignin('base', 'edmd_dataset_schedule', cfg);
    fprintf('EDMD rich-data run %d/%d : %s\n', run_index_wrapped, cfg.total_runs, dataset_meta.label);
    print_run_summary(dataset_meta, schedule);
    motive.getData(agent);
else
    if execution_mode == "preview"
        schedule = resolve_phase_schedule(dt, te, arming_steps, ...
            takeoff_duration_in, flight_duration_in, landing_duration_in);
        if ~isempty(preview_duration)
            schedule.flight_steps = max(1, round(double(preview_duration) / dt));
            schedule.flight_duration = schedule.flight_steps * dt;
            schedule.total_duration = schedule.takeoff_duration + schedule.flight_duration + schedule.landing_duration;
            schedule.sample_count = schedule.arming_steps + schedule.takeoff_steps + schedule.flight_steps + schedule.landing_steps;
        end
        [time, logger, ~, agent, dataset_meta, run_index_wrapped] = ...
            setup_richdata_case(modeType, dt, schedule.total_duration, schedule.sample_count, run_index, preview_reference, case_opts);
        assignin('base', 'edmd_dataset_meta', dataset_meta);
        assignin('base', 'edmd_dataset_schedule', cfg);
        fprintf('Preview rich-data run %d/%d : %s\n', run_index_wrapped, cfg.total_runs, dataset_meta.label);
        print_run_summary(dataset_meta, schedule);
        run_phase_sequence(agent, time, logger, schedule, silent_batch);
        render_standard_preview(logger, time, dataset_meta, run_index, preview_save_figure, preview_figure_prefix);

        % ===== テストモードでの健全性チェック（図は表示済み）=====
        [diverged, reason] = check_run_health(agent, logger);
        if diverged
            warning('SimHL_MPC_RICHDATA:previewDiverged', ...
                ['[Preview run %d : %s] で異常を検出しました。\n', ...
                 '理由: %s\n', ...
                 '図を確認してください。バッチ実行ではこの条件で中止されます。'], ...
                run_index_wrapped, dataset_meta.label, reason);
        else
            fprintf('[Preview健全性OK] run %d : %s 発散なし。\n', run_index_wrapped, dataset_meta.label);
        end
    else
        if execution_mode == "run"
            run_indices = run_index;
            autorun_batch = false;
        elseif isempty(run_indices)
            if autorun_batch || execution_mode == "batch"
                run_indices = 1:cfg.total_runs;
            else
                run_indices = run_index;
            end
        end

        for j = run_indices
            schedule = resolve_phase_schedule(dt, te, arming_steps, ...
                takeoff_duration_in, flight_duration_in, landing_duration_in);
            [time, logger, ~, agent, dataset_meta, run_index_wrapped] = ...
                setup_richdata_case(modeType, dt, schedule.total_duration, schedule.sample_count, j, preview_reference, case_opts);

            assignin('base', 'edmd_dataset_meta', dataset_meta);
            assignin('base', 'edmd_dataset_schedule', cfg);
            fprintf('EDMD rich-data run %d/%d : %s\n', run_index_wrapped, cfg.total_runs, dataset_meta.label);
            print_run_summary(dataset_meta, schedule);

            run_phase_sequence(agent, time, logger, schedule, silent_batch);

            % ===== 発散・異常検出：問題があれば中止 =====
            [diverged, reason] = check_run_health(agent, logger);
            if diverged
                error('SimHL_MPC_RICHDATA:diverged', ...
                    ['[run %d/%d : %s] で異常検出により中止しました。\n', ...
                     '理由: %s\n', ...
                     '対策: 激励振幅(u1_amp等)を下げる、慣性/質量の摂動幅を小さくする、', ...
                     'またはMPC予測域Hを調整してください。'], ...
                    run_index_wrapped, cfg.total_runs, dataset_meta.label, reason);
            end

            save_name = strcat(char(save_prefix), num2str(j));
            logger.save(save_name, "separate", save_separate);
            fprintf('Saved logger to Data/Sim_data with name prefix: %s\n', save_name);

            if ~autorun_batch
                break;
            end
        end
    end
end

function render_standard_preview(logger, time, dataset_meta, run_index, save_figure, figure_prefix)
logger.plot({1, "p", "er"}, ...
    "fig_num", 5, ...
    "xrange", [time.ts, time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
fig_time = figure(5);
sgtitle(sprintf('Run %d Position-vs-Time: %s', run_index, dataset_meta.label), 'Interpreter', 'none');
drawnow

logger.plot({1, "p1-p2-p3", "er"}, ...
    "fig_num", 4, ...
    "phase", 'tfl', ...
    "color", 0, ...
    "linewidth", 2.5, ...
    "fontsize", 14);
fig = figure(4);
ax = gca;
view(ax, 3);
grid(ax, 'on');
axis(ax, 'equal');
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
zlabel(ax, 'z [m]');
title(ax, '3D Trajectory Preview');
sgtitle(sprintf('Run %d Preview: %s', run_index, dataset_meta.label), 'Interpreter', 'none');
drawnow

if save_figure
    if ~exist('Data/Sim_data', 'dir')
        mkdir('Data/Sim_data');
    end
    file_path_3d = fullfile('Data', 'Sim_data', sprintf('%s%d_3d.png', char(figure_prefix), run_index));
    file_path_t = fullfile('Data', 'Sim_data', sprintf('%s%d_pt.png', char(figure_prefix), run_index));
    saveas(fig, file_path_3d);
    saveas(fig_time, file_path_t);
    fprintf('Saved preview figures: %s , %s\n', file_path_3d, file_path_t);
end
end

function print_run_summary(dataset_meta, schedule)
fprintf('Schedule: takeoff=%.1fs, flight=%.1fs, landing=%.1fs, total=%.1fs\n', ...
    schedule.takeoff_duration, schedule.flight_duration, schedule.landing_duration, schedule.total_duration);
if isfield(dataset_meta, 'period_sec') && isfinite(dataset_meta.period_sec) && dataset_meta.period_sec > 0
    fprintf('Reference period: %.1fs, expected laps in flight: %.2f\n', ...
        dataset_meta.period_sec, schedule.flight_duration / dataset_meta.period_sec);
end
end

function [diverged, reason] = check_run_health(agent, logger)
% plant真値の軌跡から発散・異常を検出する。
% 閾値を超えたら diverged=true と理由文字列を返す。
    diverged = false;
    reason = '';

    % --- 発散判定の閾値 ---
    POS_LIMIT  = 5.0;    % 位置が±5mを超えたら発散とみなす
    Z_FLOOR    = -1.0;   % 高度が-1mを下回ったら墜落
    ANG_LIMIT  = 1.5;    % 姿勢角(roll/pitch)が約86度を超えたら異常
    VEL_LIMIT  = 15.0;   % 速度が15m/sを超えたら発散

    % plant真値の時系列を取り出す
    p = []; v = []; q = [];
    try
        pr = agent.plant.result;  % cell配列を想定
        if iscell(pr)
            K = numel(pr);
            p = nan(3, K); v = nan(3, K); q = nan(3, K);
            for k = 1:K
                if isempty(pr{k}) || ~isfield(pr{k}, 'state') && ~isprop(pr{k}, 'state')
                    continue;
                end
                st = pr{k}.state;
                try; p(:,k) = double(st.p(:)); catch; end
                try; v(:,k) = double(st.v(:)); catch; end
                try
                    qq = st.getq('euler'); q(:,k) = double(qq(:));
                catch; end
            end
        end
    catch
        % plant.result が想定形でない場合は estimator から取得を試みる
        try
            st = agent.estimator.result.state;
            p = double(st.p(:)); v = double(st.v(:));
        catch
        end
    end

    if isempty(p)
        % 状態が取れない場合は判定不能（発散とはしない）
        return;
    end

    % --- NaN / Inf チェック ---
    if any(~isfinite(p(:))) || (~isempty(v) && any(~isfinite(v(:))))
        diverged = true;
        reason = '状態にNaN/Infが発生（数値発散）';
        return;
    end

    % --- 位置発散チェック ---
    max_xy = max(max(abs(p(1:2, :))));
    if max_xy > POS_LIMIT
        diverged = true;
        reason = sprintf('水平位置が±%.1fmを超過（最大 %.2fm）', POS_LIMIT, max_xy);
        return;
    end

    min_z = min(p(3, :));
    if min_z < Z_FLOOR
        diverged = true;
        reason = sprintf('高度が%.1fmを下回った（最低 %.2fm）= 墜落', Z_FLOOR, min_z);
        return;
    end

    % --- 速度発散チェック ---
    if ~isempty(v)
        max_v = max(vecnorm(v, 2, 1));
        if max_v > VEL_LIMIT
            diverged = true;
            reason = sprintf('速度が%.1fm/sを超過（最大 %.2fm/s）', VEL_LIMIT, max_v);
            return;
        end
    end

    % --- 姿勢角発散チェック ---
    if ~isempty(q) && any(isfinite(q(:)))
        max_ang = max(max(abs(q(1:2, :))));
        if isfinite(max_ang) && max_ang > ANG_LIMIT
            diverged = true;
            reason = sprintf('姿勢角(roll/pitch)が%.2frad(約%.0f度)を超過', ANG_LIMIT, ANG_LIMIT*180/pi);
            return;
        end
    end
end


function cfg = rich_dataset_config()
cfg.counts = struct( ...
    "spline", 32, ...
    "p2p", 24, ...
    "saddle", 28, ...
    "figure8", 20, ...
    "heart", 12, ...
    "star", 12);
cfg.total_runs = cfg.counts.spline + cfg.counts.p2p + cfg.counts.saddle + ...
    cfg.counts.figure8 + cfg.counts.heart + cfg.counts.star;
end

function schedule = resolve_phase_schedule(dt, te_default, arming_steps, takeoff_duration_in, flight_duration_in, landing_duration_in)
if isempty(takeoff_duration_in)
    takeoff_duration = 6.0;
else
    takeoff_duration = double(takeoff_duration_in);
end

if isempty(landing_duration_in)
    landing_duration = 4.0;
else
    landing_duration = double(landing_duration_in);
end

if isempty(flight_duration_in)
    flight_duration = max(5.0, double(te_default) - takeoff_duration - landing_duration);
else
    flight_duration = double(flight_duration_in);
end

schedule.arming_steps = max(1, round(arming_steps));
schedule.takeoff_steps = max(1, round(takeoff_duration / dt));
schedule.flight_steps = max(1, round(flight_duration / dt));
schedule.landing_steps = max(1, round(landing_duration / dt));
schedule.takeoff_duration = schedule.takeoff_steps * dt;
schedule.flight_duration = schedule.flight_steps * dt;
schedule.landing_duration = schedule.landing_steps * dt;
schedule.total_duration = schedule.takeoff_duration + schedule.flight_duration + schedule.landing_duration;
schedule.sample_count = schedule.arming_steps + schedule.takeoff_steps + schedule.flight_steps + schedule.landing_steps;
end

function [time, logger, motive, agent, dataset_meta, run_index_wrapped] = setup_richdata_case(modeType, dt, te, sample_count, run_index, preview_reference, case_opts)
if nargin < 7 || isempty(case_opts)
    % 後方互換：未指定なら従来通り（劣化条件相当）
    case_opts = struct('case_type', "degraded", 'use_plant_perturb', 1, ...
        'use_excitation', 1, 'use_compensation', 0, 'edmd_model_file', "", ...
        'hlmpc_mpc_profile', "mismatch", 'ref_override', []);
end
ts = 0;
time = TIME(ts, dt, te);
motive = Connector_Natnet_sim(dt);
logger = LOGGER(1, sample_count, 0, [], []);

initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
agent.plant = MODEL_CLASS(agent, Model_Quat13(dt, initial_state, 1));

% ===== 条件1: 質量＋慣性の摂動（perturb）=====
% HL成功版(SimKQ_LMPC)と全く同じ方式: plant構築直後に plant.param を直接変更する。
% （EulerAngleは Setting.param=parameter.get(...) で plant.param に物理値を持つ。
%   Quat13で効かない場合は、下の plant.param 書き換えが動力学に反映されていない
%   ことを意味するので、その場合は Model_Quat13 側の修正が必要。）
if case_opts.use_plant_perturb
    agent.plant.param(1) = 0.71;   % 質量
    agent.plant.param(6) = 0.20;   % jx
    agent.plant.param(7) = 0.20;   % jy
    agent.plant.param(8) = 0.20;   % jz
    if evalin('base', 'exist(''hlmpc_perturb_mass'', ''var'')')
        agent.plant.param(1) = double(evalin('base', 'hlmpc_perturb_mass'));
    end
    if evalin('base', 'exist(''hlmpc_perturb_inertia'', ''var'')')
        perturb_inertia = double(evalin('base', 'hlmpc_perturb_inertia'));
        if isscalar(perturb_inertia)
            perturb_inertia = repmat(perturb_inertia, 1, 3);
        end
        if numel(perturb_inertia) ~= 3
            error('hlmpc_perturb_inertia must be scalar or 3-element vector.');
        end
        agent.plant.param(6) = perturb_inertia(1);
        agent.plant.param(7) = perturb_inertia(2);
        agent.plant.param(8) = perturb_inertia(3);
    end
    fprintf('[PERTURB-DIAG] use_perturb=1 適用後: plant.param(1)=%.4f param(6)=%.4f\n', ...
        agent.plant.param(1), agent.plant.param(6));
else
    fprintf('[PERTURB-DIAG] use_perturb=0 (nominal): plant.param(1)=%.4f param(6)=%.4f\n', ...
        agent.plant.param(1), agent.plant.param(6));
end

agent.estimator = EKF(agent, Estimator_EKF(agent, dt, ...
    MODEL_CLASS(agent, Model_EulerAngle(dt, initial_state, 1)), ["p", "q"]));

if modeType
    agent.sensor = MOTIVE(agent, Sensor_Motive(1, 0, motive));
else
    agent.sensor = DIRECT_SENSOR(agent, 0.0);
end

cfg = rich_dataset_config();
run_index = max(1, round(run_index));
run_index_wrapped = mod(run_index - 1, cfg.total_runs) + 1;
if isfield(case_opts, 'ref_override') && ~isempty(case_opts.ref_override)
    [agent.reference.time_var, dataset_meta] = build_override_reference(agent, case_opts.ref_override, run_index_wrapped);
else
    [agent.reference.time_var, dataset_meta] = build_dataset_reference(agent, run_index_wrapped, preview_reference);
end

% takeoff / landing 参照（HL-MPC の四相飛行に必要）
agent.reference.takeoff = TAKEOFF_REFERENCE(agent, []);
agent.reference.landing = LANDING_REFERENCE(agent, dt, 0.1);

% ===== 3条件とも HL_MPC_EDMD_MEC を使用（スイッチで区別）=====
% nominal     : 摂動なし, 補償OFF, 激励OFF
% degraded    : 摂動あり, 補償OFF, 激励ON（紊乱）
% compensated : 摂動あり, 補償ON,  激励ON
if isfield(case_opts, 'edmd_model_file') && strlength(case_opts.edmd_model_file) > 0
    mpc_param = Controller_HL_MPC_EDMD_MEC(dt, agent, char(case_opts.edmd_model_file), ...
        'plant', double(case_opts.use_compensation), char(case_opts.hlmpc_mpc_profile));
else
    mpc_param = Controller_HL_MPC_EDMD_MEC(dt, agent, '', ...
        'plant', double(case_opts.use_compensation), char(case_opts.hlmpc_mpc_profile));
end

% 激励（紊乱）の ON/OFF。以前のHLと同じ multisine を流用。
% 真の慣性が小さい(1.8e-3等)ため、激励が強いと発散する。
% 残差を励起しつつ発散しない控えめな振幅に設定。
mpc_param.edmd_disturbance_enable = double(case_opts.use_excitation);
if case_opts.use_excitation
    mpc_param.explore.enable     = 1;
    % H=12短縮が主たる劣化要因。激励は残差励起の補助のため控えめに。
    mpc_param.explore.u1_amp     = [0.04; 0.02];   % 推力
    mpc_param.explore.u2_amp     = [0.012; 0.006]; % roll
    mpc_param.explore.u3_amp     = [0.012; 0.006]; % pitch
    mpc_param.explore.u1_freq    = [0.8; 1.5];
    mpc_param.explore.u2_freq    = [1.0; 1.8];
    mpc_param.explore.u3_freq    = [1.2; 2.0];
    mpc_param.explore.du_cap     = [0.5; 0.05; 0.05; 0.01];
    mpc_param.explore.start_time = 3.0;
    mpc_param.explore.ramp_time  = 2.0;
else
    mpc_param.explore.enable = 0;
end

agent.controller.hlmpc = HL_MPC_EDMD_MEC(agent, mpc_param);

% ===== 摂動の実適用（コントローラ構築の後）=====
% HL_MPC_EDMD_MEC は構築時に obj.param.P = self.parameter.get(...) で
% 標称パラメータを数値コピーして固定する。したがってこの時点で
% agent.parameter を変更しても MPC の標称モデルは標称値のまま、
% plant(Model_Quat13, agent.parameterを実時読み)だけが摂動値を使う。
% これにより「plant失配・MPC標称」という本来の劣化条件が成立する。
agent.cha_allocation = struct("reference", "time_var", ...
    "a", struct("reference", "takeoff"), ...
    "t", struct("reference", "takeoff"), ...
    "l", struct("reference", "landing"));
agent.cha_allocation.controller = "hlmpc";
agent.cha_allocation.f.controller = ["hlmpc"];
end

function run_phase_sequence(agent, time, logger, schedule, silent_output)
for i = 1:length(agent)
    agent(i).cha_allocation = expand_cha_allocation(agent(i));
end

run_phase_steps(agent, time, logger, 'a', schedule.arming_steps, false, silent_output);
run_phase_steps(agent, time, logger, 't', schedule.takeoff_steps, true, silent_output);
run_phase_steps(agent, time, logger, 'f', schedule.flight_steps, true, silent_output);
run_phase_steps(agent, time, logger, 'l', schedule.landing_steps, true, silent_output);
end

function run_phase_steps(agent, time, logger, phase_char, step_count, advance_time, silent_output)
for k = 1:step_count
    do_calculation_step(agent, time, logger, phase_char, silent_output);
    if advance_time
        time.t = time.t + time.dt;
    end
end
end

function do_calculation_step(agent, time, logger, phase_char, silent_output)
for i = 1:length(agent)
    do_allocated_prop(agent, i, "sensor", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "estimator", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "reference", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "controller", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "input_transform", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "plant", time, logger, phase_char, silent_output);
end
logger.logging(time, phase_char, agent, []);
time.k = logger.k;

% ===== ステップ毎の軽量発散チェック（中途発散を即中止）=====
% 重い状態でMPCのQPが解けなくなる前に止めることで、計算が暴走するのを防ぐ。
for i = 1:length(agent)
    try
        st = agent(i).plant.state;
        p = double(st.p(:));
        if any(~isfinite(p)) || max(abs(p(1:2))) > 8.0 || p(3) < -2.0
            error('SimHL_MPC_RICHDATA:stepDiverged', ...
                ['ステップ実行中に発散を検出（phase=%s, t=%.2fs）。\n', ...
                 'p=[%.2f %.2f %.2f]。激励/摂動が強すぎる可能性。'], ...
                phase_char, time.t, p(1), p(2), p(3));
        end
    catch ME
        if strcmp(ME.identifier, 'SimHL_MPC_RICHDATA:stepDiverged')
            rethrow(ME);
        end
        % 状態取得失敗は無視（plant構造が想定外でも通常処理は継続）
    end
end
end

function do_allocated_prop(agent, idx, prop, time, logger, phase_char, silent_output)
entry = agent(idx).cha_allocation.(phase_char).(prop);
if isempty(entry)
    call_fn = @() agent(idx).(prop).do(time, phase_char, logger, [], agent, idx);
    res = execute_prop_call(call_fn, silent_output);
else
    call_fn = @() agent(idx).(prop).(entry(1)).do(time, phase_char, logger, [], agent, idx);
    res = execute_prop_call(call_fn, silent_output);
    for j = 2:length(entry)
        call_fn = @() agent(idx).(prop).(entry(j)).do(time, phase_char, logger, [], agent, idx);
        res = merge_result(res, execute_prop_call(call_fn, silent_output));
    end
end
agent(idx).(prop).result = res;
end

function res = execute_prop_call(call_fn, silent_output)
if silent_output
    tmp_res = [];
    evalc('tmp_res = call_fn();');
    res = tmp_res;
else
    res = call_fn();
end
end

function AL = expand_cha_allocation(agent)
AL = struct();
for cha = ['a', 't', 'f', 'l']
    AL.(cha) = struct();
    for prop = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"]
        AL.(cha).(prop) = [];
    end
end

if isprop(agent, "cha_allocation")
    al = agent.cha_allocation;
    for prop = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"]
        if isfield(al, prop)
            for cha = ['a', 't', 'f', 'l']
                AL.(cha).(prop) = al.(prop);
            end
        end
    end

    for cha = ['a', 't', 'f', 'l']
        if isfield(al, cha)
            for prop = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"]
                if isfield(al.(cha), prop)
                    AL.(cha).(prop) = al.(cha).(prop);
                end
            end
        end
    end
end
end

function [ref_obj, meta] = build_dataset_reference(agent, run_index, preview_reference)
cfg = rich_dataset_config();
b1 = cfg.counts.spline;
b2 = b1 + cfg.counts.p2p;
b3 = b2 + cfg.counts.saddle;
b4 = b3 + cfg.counts.figure8;
b5 = b4 + cfg.counts.heart;

if run_index <= b1
    local_index = run_index;
    [ref_obj, meta] = build_spline_reference(agent, run_index, local_index, preview_reference);
elseif run_index <= b2
    local_index = run_index - b1;
    [ref_obj, meta] = build_p2p_reference(agent, run_index, local_index);
elseif run_index <= b3
    local_index = run_index - b2;
    [ref_obj, meta] = build_saddle_reference(agent, run_index, local_index);
elseif run_index <= b4
    local_index = run_index - b3;
    [ref_obj, meta] = build_figure8_reference(agent, run_index, local_index);
elseif run_index <= b5
    local_index = run_index - b4;
    [ref_obj, meta] = build_heart_reference(agent, run_index, local_index);
else
    local_index = run_index - b5;
    [ref_obj, meta] = build_star_reference(agent, run_index, local_index);
end
end

function [ref_obj, meta] = build_override_reference(agent, ref_override, run_index)
if iscell(ref_override)
    ref_args = ref_override;
    func_name = string(ref_args{1});
    group_name = "override";
    label = char(func_name);
    period_sec = NaN;
elseif isstruct(ref_override)
    if ~isfield(ref_override, 'func')
        error('hlmpc_ref_override struct must contain a func field.');
    end
    func_name = string(ref_override.func);
    if isfield(ref_override, 'args')
        arg_list = ref_override.args;
        if isstruct(arg_list)
            arg_list = struct_to_name_value_cell(arg_list);
        elseif ~iscell(arg_list)
            error('hlmpc_ref_override.args must be a cell array or scalar struct.');
        end
    else
        arg_list = {};
    end
    ref_args = {char(func_name), arg_list};
    if isfield(ref_override, 'use_hl') && logical(ref_override.use_hl)
        ref_args{end + 1} = "HL";
    end
    if isfield(ref_override, 'group')
        group_name = string(ref_override.group);
    else
        group_name = "override";
    end
    if isfield(ref_override, 'label')
        label = char(string(ref_override.label));
    else
        label = char(func_name);
    end
    if isfield(ref_override, 'period_sec')
        period_sec = double(ref_override.period_sec);
    else
        period_sec = NaN;
    end
else
    error('hlmpc_ref_override must be a TIME_VARYING_REFERENCE cell array or struct.');
end

ref_obj = TIME_VARYING_REFERENCE(agent, ref_args);
meta = struct( ...
    "group", group_name, ...
    "run_index", run_index, ...
    "local_index", 1, ...
    "period_sec", period_sec, ...
    "label", label);
end

function arg_list = struct_to_name_value_cell(s)
names = fieldnames(s);
arg_list = cell(1, 2 * numel(names));
for i = 1:numel(names)
    arg_list{2*i - 1} = names{i};
    arg_list{2*i} = s.(names{i});
end
end

function [ref_obj, meta] = build_spline_reference(agent, run_index, local_index, preview_reference)
point_bank = [10, 12, 12, 14];
point_dt_bank = [3.5, 4.0, 4.5, 5.0];
xy_limit_bank = [0.85, 0.95, 1.05, 0.90];
z_center_bank = [0.62, 0.68, 0.74, 0.66];
z_sigma_bank = [0.16, 0.18, 0.15, 0.20];
variant = mod(local_index - 1, numel(point_bank)) + 1;
seed = 1000 + run_index;

ref_obj = TIME_VARYING_REFERENCE(agent, { ...
    "gen_ref_spline_edmd", ...
    {"point", point_bank(variant), ...
     "order", 9, ...
     "point_dt", point_dt_bank(variant), ...
     "ManualSetting", 0, ...
     "check", preview_reference, ...
     "seed", seed, ...
     "xy_limit", xy_limit_bank(variant), ...
     "z_center", z_center_bank(variant), ...
     "z_sigma", z_sigma_bank(variant), ...
     "z_min", 0.35, ...
     "z_max", 1.05}});

meta = struct( ...
    "group", "spline", ...
    "run_index", run_index, ...
    "local_index", local_index, ...
    "period_sec", point_dt_bank(variant) * (point_bank(variant) - 1), ...
    "label", sprintf("Spline 3D seed=%d point_dt=%.1f", seed, point_dt_bank(variant)));
end

function [ref_obj, meta] = build_p2p_reference(agent, run_index, local_index)
templates = p2p_templates();
hold_time_bank = [4.5, 5.0, 5.5, 6.0];
variant = mod(local_index - 1, numel(templates)) + 1;
hold_time = hold_time_bank(mod(local_index - 1, numel(hold_time_bank)) + 1);

ref_obj = MPC_POINT_REFERENCE(agent, {templates{variant}, hold_time});
meta = struct( ...
    "group", "p2p", ...
    "run_index", run_index, ...
    "local_index", local_index, ...
    "period_sec", NaN, ...
    "label", sprintf("P2P 3D template=%d hold=%.1fs", variant, hold_time));
end

function [ref_obj, meta] = build_saddle_reference(agent, run_index, local_index)
% saddle を2グループに分割:
%   前半（local_index 前半）: 平面サドル（z方向size=0、高度一定0.6）
%   後半（local_index 後半）: 3D起伏サドル（z方向size=0.2、上下起伏）
% orig高度は takeoff(0.6m) と揃えて高度突変を防ぐ。

freq_bank  = [8, 10, 12, 14];
xy_bank = [ ...
    1.00, 1.00; ...
    1.00, 1.00; ...
    1.00, 1.00; ...
    1.00, 1.00];
phase_bank = [-pi, -pi/2, 0, pi/2];

variant = mod(local_index - 1, numel(freq_bank)) + 1;

% saddle総数28の前半/後半で平面/3Dを切替（前14=平面、後14=3D起伏）
saddle_total = 28;
half = ceil(saddle_total / 2);
if local_index <= half
    z_size = 0.0;            % 平面サドル
    saddle_kind = 'planar';
else
    z_size = 0.2;            % 3D起伏サドル（上下0.2m）
    saddle_kind = '3D';
end

orig_z = 0.6;   % takeoff高度と一致させ突変を防ぐ
z_size = 0.2;
saddle_kind = '3D';
size_vec = [xy_bank(variant, 1), xy_bank(variant, 2), z_size];
orig_vec = [0, 0, orig_z];

ref_obj = TIME_VARYING_REFERENCE(agent, { ...
    "gen_ref_saddle", ...
    {"freq", freq_bank(variant), ...
     "orig", orig_vec, ...
     "size", size_vec, ...
     "phase", phase_bank(variant)}, ...
    "HL"});

meta = struct( ...
    "group", "saddle", ...
    "run_index", run_index, ...
    "local_index", local_index, ...
    "period_sec", freq_bank(variant), ...
    "label", sprintf("%s saddle T=%ds size=[%.2f %.2f %.2f]", ...
        saddle_kind, freq_bank(variant), size_vec(1), size_vec(2), size_vec(3)));
end

function [ref_obj, meta] = build_figure8_reference(agent, run_index, local_index)
freq_bank = [14, 16, 18, 20];
size_bank = [ ...
    0.85, 0.85, 0.18; ...
    1.00, 0.75, 0.22; ...
    0.75, 1.00, 0.25; ...
    0.90, 0.90, 0.20];
orig_bank = [ ...
    0, 0, 0.65; ...
    0, 0, 0.65; ...
    0, 0, 0.70; ...
    0, 0, 0.60];
phase_bank = [0, pi/4, pi/2, -pi/4];
variant = mod(local_index - 1, numel(freq_bank)) + 1;

ref_obj = TIME_VARYING_REFERENCE(agent, { ...
    "gen_ref_figure8", ...
    {"freq", freq_bank(variant), ...
     "orig", orig_bank(variant, :), ...
     "size", size_bank(variant, :), ...
     "phase", phase_bank(variant)}});

meta = struct( ...
    "group", "figure8", ...
    "run_index", run_index, ...
    "local_index", local_index, ...
    "period_sec", freq_bank(variant), ...
    "label", sprintf("Figure8 3D T=%ds z_amp=%.2f", freq_bank(variant), size_bank(variant, 3)));
end

function [ref_obj, meta] = build_heart_reference(agent, run_index, local_index)
freq_bank = [18, 20, 22, 24];
size_bank = [ ...
    0.80, 0.80, 0.15; ...
    0.85, 0.75, 0.18; ...
    0.75, 0.85, 0.20; ...
    0.90, 0.70, 0.16];
orig_bank = [ ...
    0, 0, 0.68; ...
    0, 0, 0.65; ...
    0, 0, 0.70; ...
    0, 0, 0.64];
phase_bank = [-pi/2, -pi/3, -2*pi/3, -pi/4];
variant = mod(local_index - 1, numel(freq_bank)) + 1;

ref_obj = TIME_VARYING_REFERENCE(agent, { ...
    "gen_ref_heart", ...
    {"freq", freq_bank(variant), ...
     "orig", orig_bank(variant, :), ...
     "size", size_bank(variant, :), ...
     "phase", phase_bank(variant)}});

meta = struct( ...
    "group", "heart", ...
    "run_index", run_index, ...
    "local_index", local_index, ...
    "period_sec", freq_bank(variant), ...
    "label", sprintf("Heart 3D T=%ds z_amp=%.2f", freq_bank(variant), size_bank(variant, 3)));
end

function [ref_obj, meta] = build_star_reference(agent, run_index, local_index)
freq_bank = [18, 20, 22, 24];
size_bank = [ ...
    0.75, 0.75, 0.14; ...
    0.85, 0.70, 0.18; ...
    0.70, 0.85, 0.20; ...
    0.90, 0.75, 0.16];
orig_bank = [ ...
    0, 0, 0.68; ...
    0, 0, 0.65; ...
    0, 0, 0.70; ...
    0, 0, 0.64];
phase_bank = [pi/2, pi/3, 2*pi/3, pi/4];
variant = mod(local_index - 1, numel(freq_bank)) + 1;

ref_obj = TIME_VARYING_REFERENCE(agent, { ...
    "gen_ref_star", ...
    {"freq", freq_bank(variant), ...
     "orig", orig_bank(variant, :), ...
     "size", size_bank(variant, :), ...
     "phase", phase_bank(variant)}});

meta = struct( ...
    "group", "star", ...
    "run_index", run_index, ...
    "local_index", local_index, ...
    "period_sec", freq_bank(variant), ...
    "label", sprintf("Star 3D T=%ds z_amp=%.2f", freq_bank(variant), size_bank(variant, 3)));
end

function templates = p2p_templates()
templates = {
    struct( ...
        "f", [0; 0; 0.60], ...
        "g", [0.90; 0.00; 0.90], ...
        "h", [0.25; -0.85; 0.42], ...
        "j", [-0.75; -0.50; 0.98], ...
        "k", [-0.90; 0.35; 0.55], ...
        "l", [0.15; 0.95; 0.82], ...
        "m", [0.95; 0.45; 0.38], ...
        "z", [0; 0; 0.60]), ...
    struct( ...
        "f", [0; 0; 0.60], ...
        "g", [0.75; -0.70; 0.45], ...
        "h", [-0.15; -1.00; 0.92], ...
        "j", [-0.95; -0.15; 0.50], ...
        "k", [-0.35; 0.85; 1.00], ...
        "l", [0.55; 0.90; 0.40], ...
        "m", [1.00; 0.00; 0.78], ...
        "z", [0; 0; 0.60]), ...
    struct( ...
        "f", [0; 0; 0.60], ...
        "g", [0.20; -0.95; 0.95], ...
        "h", [-0.85; -0.75; 0.40], ...
        "j", [-1.00; 0.10; 0.88], ...
        "k", [-0.20; 0.95; 0.48], ...
        "l", [0.80; 0.80; 1.02], ...
        "m", [0.95; -0.10; 0.42], ...
        "z", [0; 0; 0.60]), ...
    struct( ...
        "f", [0; 0; 0.60], ...
        "g", [1.00; 0.25; 0.50], ...
        "h", [0.55; -0.85; 1.00], ...
        "j", [-0.55; -0.95; 0.36], ...
        "k", [-1.00; -0.20; 0.86], ...
        "l", [-0.55; 0.90; 0.44], ...
        "m", [0.55; 0.95; 0.96], ...
        "z", [0; 0; 0.60])};
end

function value = get_base_value(name, default_value)
value = default_value;
try
    if evalin('base', sprintf('exist(''%s'', ''var'')', name))
        value = evalin('base', name);
    end
catch
    value = default_value;
end
end

function post(app)
fig = figure(100); clf
tiledlayout(2, 2, "TileSpacing", "compact", "Padding", "compact")

nexttile
app.logger.plot({1, "p", "er"}, ...
    "ax", gca, ...
    "xrange", [app.time.ts, app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Position Error")

nexttile
app.logger.plot({1, "input", ""}, ...
    "ax", gca, ...
    "xrange", [app.time.ts, app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Control Input")

nexttile
app.logger.plot({1, "v", "er"}, ...
    "ax", gca, ...
    "xrange", [app.time.ts, app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Velocity Error")

nexttile
app.logger.plot({1, "p1-p2-p3", "er"}, ...
    "ax", gca, ...
    "phase", 'tfl', ...
    "color", 0, ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Phase Plot")
end
