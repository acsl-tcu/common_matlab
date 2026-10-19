%% HLC＋Koopman誤差補償。学習モードではランダム軌道と4入力励振を使用する。
dataset_setting = mec_dataset_settings();
trajectory_seed = 1;
if exist('Setting','var')
    if isfield(Setting,'mec_dataset')
        dataset_setting = Setting.mec_dataset;
    end
    if isfield(Setting,'trajectory_seed')
        trajectory_seed = Setting.trajectory_seed;
    end
end

% 既存のモードスクリプトと同じパス設定。
activeEditor = matlab.desktop.editor.getActive;
if ~isempty(activeEditor)
    modeDir = fileparts(activeEditor.Filename);
    if ~contains(path,modeDir)
        rootDir = modeDir;
        [~,lastDir] = fileparts(modeDir);
        if strcmpi(lastDir,'mode')
            rootDir = fileparts(modeDir);
        end
        cd(rootDir);
        [~,folders] = regexp(genpath('.'),'\.\\\.git.*?;','match','split');
        cellfun(@(p) addpath(p),folders,'UniformOutput',false);
        close all hidden;
        userpath('clear');
    end
end

%% シミュレーション時間とログ
ts = 0;
dt = 0.025;
te = 65;
if dataset_setting.enabled
    te = max(te,dataset_setting.flight_time+25);
end
time = TIME(ts,dt,te);
in_prog_func = @(app) dfunc(app);
post_func = @(app) dfunc(app);
logger = LOGGER(1,size(ts:dt:te,2),0,[],[]);
logger.display_func = @(agent,t) build_display_vector(agent,t);
logger.display_on = true;
if dataset_setting.enabled
    logger.display_on = dataset_setting.display_on;
end
fprintf('ref:[px py pz]  est:[px py pz]  U:[T tx ty tz]\n');

%% 初期状態
initial_state.p = arranged_position([0,0],1,1,0);
initial_state.q = [1;0;0;0];
initial_state.v = [0;0;0];
initial_state.w = [0;0;0];

%% ノミナルパラメータと、実機相当プラントのモデル誤差
agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE"); % ノミナルパラメータ
plant_model = Model_EulerAngle(dt,initial_state,1);

% plant_model.param.param(6) = 0.185; % 実機相当プラントのjx
% plant_model.param.param(7) = 0.185; % 実機相当プラントのjy

agent.plant = MODEL_CLASS(agent,plant_model);

%% 世界座標の外力。設定はmec_dataset_settings.mで一括して行う。
% 4入力励振（推力・roll・pitch・yaw）とは別に、実機相当プラントだけへ加える。
wind_mode = dataset_setting.wind_mode;
wind_amplitude = dataset_setting.wind_amplitude;
wind_axis = dataset_setting.wind_axis;
wind_step_start = dataset_setting.wind_step_start;
wind_step_end = dataset_setting.wind_step_end;
wind_sweep_start = dataset_setting.wind_sweep_start;
wind_sweep_duration = dataset_setting.wind_sweep_duration;
wind_frequency_range = dataset_setting.wind_frequency_range;
wind_ramp_time = dataset_setting.wind_ramp_time;

assert(ismember(wind_axis,[1,2,3]),'風外力の方向は1、2、3のいずれかにしてください。');
assert(isfinite(wind_amplitude) && wind_amplitude>=0, ...
    '風外力の振幅は0以上の有限値にしてください。');
assert(wind_step_start<wind_step_end,'step外力の開始時刻は終了時刻より前にしてください。');
assert(wind_sweep_duration>0 && wind_ramp_time>0 ...
    && wind_ramp_time<=wind_sweep_duration/2, ...
    'sweep外力の継続時間と立ち上げ時間を確認してください。');
assert(numel(wind_frequency_range)==2 && wind_frequency_range(1)>0 ...
    && wind_frequency_range(2)>wind_frequency_range(1), ...
    'sweep周波数は0<下限<上限にしてください。');

switch wind_mode
    case 'none'
        agent.plant.external_force_world = [];
    case 'step'
        agent.plant.external_force_world = @(t,x) ...
            force_along_axis(wind_axis,wind_amplitude*double( ...
            t>=wind_step_start && t<wind_step_end));
    case 'sweep'
        agent.plant.external_force_world = @(t,x) ...
            wind_force_sweep_local(t,wind_amplitude,wind_axis, ...
            wind_sweep_start,wind_sweep_duration,wind_frequency_range,wind_ramp_time);
    otherwise
        error('外力モードが不明です：%s',wind_mode);
end

%% 外力が運動方程式の加速度へ正しく加算されることを確認する。
if ~isempty(agent.plant.external_force_world)
    x_test = agent.plant.state.get();
    % 検証入力にはホバリング相当の推力を使う。
    p_test = agent.plant.param;
    u_test = [p_test(1)*p_test(9);0;0;0];
    t_test = 12.3;
    base_dx = agent.plant.method(x_test,u_test,p_test);
    wind_dx = agent.plant.dynamics_with_external_force(t_test,x_test,u_test);
    force_test = agent.plant.external_force_world(t_test,x_test);
    added_a = wind_dx(7:9) - base_dx(7:9);
    expected_a = force_test(:)/p_test(1);
    assert(norm(added_a(:)-expected_a(:)) ...
        <= 1e-9*max(1,norm(expected_a)), ...
        'Wind force is not reflected correctly in the plant dynamics.');
    fprintf('External force RHS check passed: Fz=%.4f N, daz=%.4f m/s^2\n', ...
        force_test(3),added_a(3));
end

%% センサと推定器。EKFの予測モデルには外力を与えない。
agent.sensor.set_function_class('direct', DIRECT_SENSOR(agent,0.0,struct('output_list',["p","q"])));
agent.estimator.set_function_class('ekf', EKF(agent,Estimator_EKF(agent,dt, MODEL_CLASS(agent,Model_EulerAngle(dt,initial_state,1)),["p","q"])));

%% 参照軌道と制御器
if dataset_setting.enabled
    % 位置から4階微分までを返すので、追加のシンボリック微分は不要。
    agent.reference.set_function_class('time_varying', TIME_VARYING_REFERENCE(agent,{"gen_ref_random", {"T",dataset_setting.flight_time,"start",[0;0;1],"seed",trajectory_seed,"plot",false,"verbose",false}}));
else
    agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_circle",{"freq",10,"center",[0;0;1],"radius",1.0},4}));
    % agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_for_hovering", {"position", [0;0;1]}}));
    % agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_star", {"freq",-10,"center",[0;0;1],"radius",1.0}}));
    % agent.reference.set_function_class("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p", {"p0",[0;0;1],"pf",[1;1;1],"T",10}}));
    % agent.reference.set_function_cltass("time_varying", TIME_VARYING_REFERENCE(agent,{"gen_ref_spline", {"point",17,"order",3,"filename",4,"ManualSetting",0,"point_dt",5,}}));
end

agent.reference.set_function_class('takeoff',TAKEOFF_REFERENCE(agent,'zd',1));
agent.reference.set_function_class('landing',LANDING_REFERENCE(agent,'dt',dt,'vd',0));
agent.cha_allocation.a.reference = "takeoff";
agent.cha_allocation.t.reference = "takeoff";
agent.cha_allocation.f.reference = "time_varying";
agent.cha_allocation.l.reference = "landing";

agent.controller.set_function_class('nominal',HLC(agent,Controller_HL(dt)));
mec_param = Controller_HL(dt);
mec_param.dataset = dataset_setting;
mec_param.trajectory_seed = trajectory_seed;
mec_param.use_compensation = dataset_setting.use_compensation;
% 一括検証などから明示指定された場合は設定ファイルより優先する。
if isfield(Setting,'mec_use_compensation')
    mec_param.use_compensation = Setting.mec_use_compensation;
end
if isfield(Setting,'mec_gain_file')
    mec_param.gain_file = Setting.mec_gain_file;
end
% mec_param.use_compensation = use_kmec; % 指定時はmec_dataset_settingsの値より優先

% 次の行を有効にすると使用ゲインを変更できる。コメントのままならMECKCの既定値。
% mec_param.gain_file = 'new_gain.mat'; % K_full（4行26列）を含むMATファイル
agent.controller.set_function_class('mec',MECKC(agent,mec_param));
agent.cha_allocation.controller = ["nominal","mec"];
% 保存したデータだけで、軌道と入力励振の条件を再現できるようにする。
mec_dataset_metadata = struct('trajectory_seed',trajectory_seed, ...
    'setting',dataset_setting,'excitation_profile',agent.controller.mec.excitation_profile, ...
    'plant_parameters',agent.plant.param,'nominal_parameters',agent.controller.mec.param.P, ...
    'wind_mode',wind_mode,'wind_amplitude',wind_amplitude,'wind_axis',wind_axis, ...
    'dt',dt,'error_definition','nominal_input_one_step_v1');
mec_dataset_metadata.use_compensation = agent.controller.mec.use_compensation;
mec_dataset_metadata.compensation_gain = agent.controller.mec.gain;
mec_dataset_metadata.excitation_enabled = agent.controller.mec.collect_dataset && ...
    agent.controller.mec.excitation_enabled;
if dataset_setting.enabled
    mec_dataset_metadata.reference_time = 0:dt:dataset_setting.flight_time;
    mec_dataset_metadata.reference = agent.reference.time_varying.func( ...
        mec_dataset_metadata.reference_time);
end

%% 結果の描画
function dfunc(app)
app.logger.plot({1,'p','er'},'ax',app.UIAxes, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',1);
app.logger.plot({1,'q','e'}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',2);
app.logger.plot({1,'v','er'}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',3);

app.logger.plot({1,'controller.result.input',''}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',6);
app.logger.plot({1,'controller.result.delta_u',''}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',7);
app.logger.plot({1,'controller.result.delta_u_mec',''}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',16);

app.logger.plot( ...
    {{1,'controller.result.z_p_forward',''}, ...
    {1,'controller.result.z_n_forward','s'}}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',13);
app.logger.plot( ...
    {{1,'controller.result.z_p_back',''}, ...
    {1,'controller.result.z_n_back','s'}}, ...
    'xrange',[app.time.ts,app.time.te],'fig_num',14);
app.logger.plot({1,'p1-p2-p3','er'},'color',0,'fig_num',9);
if isfield(app.logger.Data.agent.controller.result{1},'excitation')
    app.logger.plot({1,'controller.result.excitation',''}, ...
        'xrange',[app.time.ts,app.time.te],'fig_num',15);
end

% 指定した世界座標の外力を描画する。
time_grid = app.time.ts:app.time.dt:app.time.te;
wind_values = zeros(3,numel(time_grid));
% 外力はプラントの関数から取得し、入力差delta_uとは区別する。
% GUI初期化中などで描画できない場合は警告にとどめる。
try
    model = app.agent(1).plant;
    if ~isempty(model.external_force_world)
        sample_state = model.state.get();
        for k = 1:numel(time_grid)
            wind_values(:,k) = model.external_force_world( ...
                time_grid(k),sample_state);
        end
    end
    figure(11); clf;
    plot(time_grid,wind_values','LineWidth',1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('External force [N]');
    legend('F_x','F_y','F_z','Location','best');
    title('Prescribed external force (plant only)');
catch ME
    warning('SimMEC_K:WindPlot', ...
        'External-force plot skipped: %s',ME.message);
end

% 推力のノミナル値・入力差・最終値を分けて描画する。
% 学習時の入力差には、MEC補償と入力励振の両方を含む。
try
    result_cells = app.logger.Data.agent.controller.result;
    saved_t = app.logger.Data.t;
    ns = min(numel(result_cells),numel(saved_t));
    t_plot = [];
    totalT = [];
    mecT = [];
    for k = 1:ns
        r = result_cells{k};
        if isstruct(r) && isfield(r,'input') && isfield(r,'delta_u') ...
                && ~isempty(r.input) && ~isempty(r.delta_u) ...
                && isfinite(saved_t(k))
            t_plot(end+1,1) = saved_t(k); %#ok<AGROW>
            totalT(end+1,1) = r.input(1); %#ok<AGROW>
            mecT(end+1,1) = r.delta_u(1); %#ok<AGROW>
        end
    end
    if ~isempty(t_plot)
        nominalT = totalT - mecT;
        figure(8); clf;
        subplot(3,1,1);
        plot(t_plot,nominalT,'LineWidth',1.2); grid on;
        ylabel('u_{nom,1} [N]'); title('Nominal thrust');
        subplot(3,1,2);
        plot(t_plot,mecT,'LineWidth',1.2); grid on;
        ylabel('\Delta u_1 [N]'); title('Input difference (MEC + excitation)');
        subplot(3,1,3);
        plot(t_plot,totalT,'LineWidth',1.2); grid on;
        xlabel('Time [s]'); ylabel('u_{cmd,1} [N]');
        title('Total commanded thrust');
    end
catch ME
    warning('SimMEC_K:InputPlot', ...
        'Input decomposition plot skipped: %s',ME.message);
end
end

function F = force_along_axis(axis_idx,magnitude)
F = zeros(3,1);
F(axis_idx) = magnitude;
end

function F = wind_force_sweep_local(t,A,axis_idx,t_start,T_sweep,frequency_range,T_ramp)
% 設定された周波数を掃引する、世界座標の外力[N]を作る。
f_start = frequency_range(1);
f_end = frequency_range(2);
F = zeros(3,1);
if t < t_start || t >= t_start+T_sweep
    return;
end
tau = t-t_start;
slope = (f_end-f_start)/T_sweep;
phase = 2*pi*(f_start*tau+0.5*slope*tau^2);
if tau < T_ramp
    env = 0.5*(1-cos(pi*tau/T_ramp));
elseif tau > T_sweep-T_ramp
    env = 0.5*(1-cos(pi*(T_sweep-tau)/T_ramp));
else
    env = 1;
end
F(axis_idx) = A*env*cos(phase);
end

function v = build_display_vector(agent,t)
idx = 1;
if ~isprop(agent(idx).reference.result.state,'xd')
    v = [];
    return;
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result,'state')
    p = agent(idx).estimator.result.state.p;
else
    p = [NaN;NaN;NaN];
end
u = agent(idx).controller.result.input;
v = sprintf(['%c %.3f : R [%7.3f,%7.3f,%7.3f] : ', ...
    'P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]'], ...
    agent(idx).cha,t.t,xd(1:3)',p',u');
end
