close all
N = 1; % the number of agents
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % termina time
time = TIME(ts,dt,te,N);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
logger.set_time_handler(time);
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",0.14);%0.0968);%0.968
agent.parameter.set("cableL",2.0);%0.0968);%0.968
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];
% Note: set the model error after setting "plant"
agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));%dt,initial,id,agent,modelName
agent.parameter.set("loadmass",0.01);%0.0968);%0.968

% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}}); % imitation of Motive camera (motion capture system)
motive.getData(agent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agent.sensor.set_function_class("motive", MOTIVE(agent, motive, dt,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
function y = motive_output(obj,data)
% Motiveの生データから、機体姿勢と吊り荷位置・ケーブル方向を整理する。
% 出力は推定器の入力として [p; euler; pL; pT] の形に整形する。
p = data.rigid(obj.rigid_id(1)).p;
pT = data.rigid(obj.rigid_id(2)).p - p;
pT = pT/norm(pT);
y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
    MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...%"Load_mL_HL"
    ["p", "q","pL","pT"])));%expの流用 質量推定有

agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",10,"center",[0;0;1.5],"radius",[0,0,0]},6})); %円系軌道
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[0;0;3.0], "p1",[10;10;3.0], "t_go",10.0, "t_hold",5.0, "t_back",10.0},6})); %P2P
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",9,"center",[0;0;1.5],"radius",[1,1,0]},6})); % triangle
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ：l移行時のrefは牽引物用なので

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
% agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Hinf_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.l.reference =["landing","sload"];

%%

function post(app)
close all
% シミュレーション終了後の結果表示とアニメーション作成。
% app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
app.logger.plot({{1, "p", "r"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "p1-p2", "er"},"phase","f", "fig_num",300);
% app.logger.plot({1, "state.mL", "e"},"phase","f");
%app.logger.plot({1, "estimator.result.ekf_mL", ""},"phase","tfl", "fig_num",2);
% app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({1, "estimator.result.state.pT", "e"},"phase","f","fig_num",2);
% app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);% 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
app.logger.plot({1, "input",""}, "phase","tf", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({{1, "controller.result.before_notch2",""},{1, "controller.result.after_notch2",""}}, "phase","f", "fig_num",300); 
% app.logger.plot({1, "controller.result.after_notch2",""}, "phase","f", "fig_num",300); 
% app.logger.plot({1, "controller.result.before_notch2:3",""}, "phase","f", "fig_num",500); 
% app.logger.plot({1, "controller.result.after_notch3",""}, "phase","f", "fig_num",200);
% app.logger.plot({{1, "controller.result.after_notch2:3",""},{1,"controller.result.real2:3",""}}, "phase","f", "fig_num",1000); 
% app.logger.plot({1, "controller.result.chirp_roll",""},"phase","f", "fig_num",4000);
% app.logger.plot({1, "controller.result.chirp_pitch",""},"phase","f", "fig_num",5000);
app.logger.plot({1, "controller.result.chirp",""},"phase","f", "fig_num",5000);
% disp_rmse(app.logger, "f")

% app.logger.plot({{1, "reference.result.state.xd1:3", ""},{1,"reference.result.state.xd5:7",""}},"phase","f","fig_num",4);
% app.logger.plot({{1, "reference.result.state.xd9:11", ""},{1,"reference.result.state.xd13:15",""}},"phase","f","fig_num",5);
% app.logger.plot({1,"controller.result.mSeqSignal",""}, "phase","f", "fig_num",300);
% app.logger.plot({1,"controller.result.chirp",""},"phase","f","fig_num",400)
% disp_bode(app.logger, "f")
% disp_bode_angle_clean(app.logger)
% disp_fft(app.logger, "estimator.result.state.pL", "e", "f")
% disp_fft(app.logger, "input2:4", "", "f")
% disp_fft(app.logger, "input1", "", "f")
% disp_fft_time(app.logger, "estimator.result.state.pL", "e", "f", 0.025)
% disp_fft_time(app.logger, "input", "", "f", 0.025)
% disp_bode_fft(app.logger, "f", 0.025)
t_all = app.logger.Data.t(1:app.logger.k);
    phase_all = app.logger.Data.phase(1:app.logger.k);
    idx_f = find(phase_all == 102);   % 102 = 'f' のASCIIコード

    diff_idx = diff(idx_f);
    break_points = find(diff_idx > 1);

    segment_starts = [1; break_points + 1];
    segment_ends = [break_points; length(idx_f)];
    segment_lengths = segment_ends - segment_starts + 1;

    [~, longest_seg] = max(segment_lengths);
    idx_f_clean = idx_f(segment_starts(longest_seg):segment_ends(longest_seg));

    t_start = t_all(idx_f_clean(1));
    t_end = t_all(idx_f_clean(end));

    fprintf('f区間: %.3fs_%.3fs_%dpoint \n', t_start, t_end, length(idx_f_clean));


% disp_bode(app.logger, [2.975, 48.600])
% disp_bode_idx(app.logger, idx_f_clean)
disp_bode_simple(app.logger, "f")



show_suspended_load_animation(app); % アニメーション描画
end

function show_suspended_load_animation(app)
% 単機の吊り下げモデルをアニメーション表示する。
if app.logger.k <= 1
    return
end

mov = DRAW_SUSPENDED_LOAD(app.logger, ...
    "target", 1, ...
    "self", app.agent(1));
mov.animation(app.logger,"target", 1,"self", app.agent(1));%表示だけ用

% mov.animation(app.logger,"target", 1,"self", app.agent(1),"mp4", true, "pause", 0);%mp4保存用

% mov.animation(app.logger, "target", 1,"self", app.agent(1), "gif", "Data/suspended_load.gif", ...
%     "fps", 20,"gif_delay", 0.05,"skip", 2, "pause", 0);%gif保存用
end

function in_prog(app)
% 実行中に推定状態をUIに表示する。
app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
end

function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力、推定質量を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
    if isprop(agent(idx).estimator.result.state, "mL")
        mL = agent(idx).estimator.result.state.mL;
    else
        mL = NaN;
    end
else
    p = [NaN; NaN; NaN];
    mL = NaN;
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f",agent(idx).cha, time.t, xd(1:3)', p', u', mL);
end


function disp_rmse(logger, phase)
% estimator と reference の position を取得（Nx3）
p_est = logger.data(1,"estimator.result.state.pL","e","phase",phase);
p_ref = logger.data(1,"p","r","phase",phase);
% サイズチェック
N = min(size(p_est,1), size(p_ref,1));
p_est = p_est(1:N,:);
p_ref = p_ref(1:N,:);
diff = p_est - p_ref;
RMSE_x = sqrt(mean(diff(:,1).^2));
RMSE_y = sqrt(mean(diff(:,2).^2));
RMSE_z = sqrt(mean(diff(:,3).^2));
fprintf('\n===== Position RMSE (total time) =====\n');
fprintf(' RMSE_x = %.6f [m]\n', RMSE_x);
fprintf(' RMSE_y = %.6f [m]\n', RMSE_y);
fprintf(' RMSE_z = %.6f [m]\n', RMSE_z);
fprintf('=====================================\n\n');
end

% function disp_bode_simple(logger, phase)
% % tfestimateの結果をそのままプロットする、シンプルな関数
% % u(n) と y(n+1) を対応させる
% % disp_bode_simple(app.logger, "f")
% 
% Fs = 1/0.025;
% 
% u = logger.data(1, "controller.result.tmp", "", "phase", phase);
% q = logger.data(1, "estimator.result.state.q", "e", "phase", phase);
% 
% for i = 1:2
%     if i == 1
%         ci = 2; co = 1; nm = 'roll';
%     else
%         ci = 3; co = 2; nm = 'pitch';
%     end
% 
%     N = min(size(u,1), size(q,1));
% 
%     % --- u(n) と y(n+1) を対応させる ---
%     u_n   = u(1:N-1, ci);
%     y_np1 = q(2:N, co);
% 
%     [h, f] = tfestimate(u_n, y_np1, [], [], [], Fs);
%     omega = 2*pi*f;
% 
%     figure('Color','w', 'Position', [100 100 900 500]);
%     t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
%     title(t, sprintf('%s torque \\rightarrow %s angle (u(n) \\rightarrow y(n+1))', nm, nm), 'FontSize', 17);
% 
%     nexttile;
%     semilogx(omega, 20*log10(abs(h)), 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Gain [dB]', 'FontSize', 15);
% 
%     nexttile;
%     semilogx(omega, angle(h)*180/pi, 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Phase [deg]', 'FontSize', 15);
%     xlabel('\omega [rad/s]', 'FontSize', 15);
% end
% end

% function disp_bode_simple(logger, phase)
% % spaの結果をそのままプロットする、シンプルな関数
% % u(n) と y(n+1) を対応させる
% % disp_bode_simple(app.logger, "f")
% 
% Fs = 1/0.025;
% dt = 1/Fs;
% 
% u = logger.data(1, "controller.result.tmp", "", "phase", phase);
% q = logger.data(1, "estimator.result.state.q", "e", "phase", phase);
% 
% for i = 1:2
%     if i == 1
%         ci = 2; co = 1; nm = 'roll';
%     else
%         ci = 3; co = 2; nm = 'pitch';
%     end
% 
%     N = min(size(u,1), size(q,1));
% 
%     % --- u(n) と y(n+1) を対応させる ---
%     u_n   = u(1:N-1, ci);
%     y_np1 = q(2:N, co);
% 
%     data_id = iddata(y_np1, u_n, dt);
%     g = spa(data_id);
% 
%     [mag, ph, w] = bode(g);
%     mag = squeeze(mag);
%     ph = squeeze(ph);
%     omega = squeeze(w);
% 
%     figure('Color','w', 'Position', [100 100 900 500]);
%     t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
%     title(t, sprintf('%s torque \\rightarrow %s angle (u(n) \\rightarrow y(n+1)), spa', nm, nm), 'FontSize', 17);
% 
%     nexttile;
%     semilogx(omega, 20*log10(mag), 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Gain [dB]', 'FontSize', 15);
% 
%     nexttile;
%     semilogx(omega, ph, 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Phase [deg]', 'FontSize', 15);
%     xlabel('\omega [rad/s]', 'FontSize', 15);
% end
% end

% function disp_bode_simple(logger, phase)
% % tfestimateの結果をそのままプロットする、シンプルな関数
% % u(n) と y(n+1) を対応させる
% % disp_bode_simple(app.logger, "f")
% 
% Fs = 1/0.025;
% 
% u = logger.data(1, "controller.result.tmp", "", "phase", phase);
% q = logger.data(1, "estimator.result.state.pL", "e", "phase", phase);
% 
% for i = 1:2
%     if i == 1
%         ci = 2; co = 1; nm = 'roll';
%     else
%         ci = 3; co = 2; nm = 'pitch';
%     end
% 
%     N = min(size(u,1), size(q,1));
% 
%     % --- u(n) と y(n+1) を対応させる ---
%     u_n   = u(1:N-1, ci);
%     y_np1 = q(2:N, co);
% 
%     [h, f] = tfestimate(u_n, y_np1, [], [], [], Fs);
%     omega = 2*pi*f;
% 
%     figure('Color','w', 'Position', [100 100 900 500]);
%     t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
%     title(t, sprintf('%s torque \\rightarrow %s angle (u(n) \\rightarrow y(n+1))', nm, nm), 'FontSize', 17);
% 
%     nexttile;
%     semilogx(omega, 20*log10(abs(h)), 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Gain [dB]', 'FontSize', 15);
% 
%     nexttile;
%     semilogx(omega, angle(h)*180/pi, 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Phase [deg]', 'FontSize', 15);
%     xlabel('\omega [rad/s]', 'FontSize', 15);
% end
% end

function disp_bode_simple(logger, phase)
% spaの結果をそのままプロットする、シンプルな関数
% u(n) と y(n+1) を対応させる
% disp_bode_simple(app.logger, "f")

Fs = 1/0.025;
dt = 1/Fs;

u = logger.data(1, "controller.result.tmp", "", "phase", phase);
q = logger.data(1, "estimator.result.state.pL", "e", "phase", phase);
% q = logger.data(1, "q", "e", "phase", phase);
for i = 1:2
    if i == 1
        ci = 2; co = 1; nm = 'roll';
    else
        ci = 3; co = 2; nm = 'pitch';
    end

    N = min(size(u,1), size(q,1));

    % --- u(n) と y(n+1) を対応させる ---
    u_n   = u(1:N-1, ci);
    y_np1 = q(2:N, co);

    data_id = iddata(y_np1, u_n, dt);
    g = spa(data_id);

    [mag, ph, w] = bode(g);
    mag = squeeze(mag);
    ph = squeeze(ph);
    omega = squeeze(w);

    figure('Color','w', 'Position', [100 100 900 500]);
    t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
    title(t, sprintf('%s  (u(n) \\rightarrow y(n+1)), spa',  nm), 'FontSize', 17);

    nexttile;
    semilogx(omega, 20*log10(mag), 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Gain [dB]', 'FontSize', 15);

    nexttile;
    semilogx(omega, ph, 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Phase [deg]', 'FontSize', 15);
    xlabel('\omega [rad/s]', 'FontSize', 15);
end
end