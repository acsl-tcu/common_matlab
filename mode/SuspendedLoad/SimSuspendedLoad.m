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
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",15,"center",[0;0;1.5],"radius",[1,1,0]},4})); %円系軌道
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[0;0;3.0], "p1",[10;10;3.0], "t_go",10.0, "t_hold",5.0, "t_back",10.0},4})); %P2P
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",9,"center",[0;0;1.5],"radius",[1,1,0]},4})); % triangle
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ：l移行時のrefは牽引物用なので

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
agent.cha_allocation.t.reference =["takeoff","sload"];
agent.cha_allocation.f.reference =["timevarying","sload"];
agent.cha_allocation.l.reference =["landing","sload"];

%%

function post(app)
% シミュレーション終了後の結果表示とアニメーション作成。
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
app.logger.plot({1, "p1-p2", "re"},"phase","f", "fig_num",300);
% app.logger.plot({1, "state.mL", "e"},"phase","f");
%app.logger.plot({1, "estimator.result.ekf_mL", ""},"phase","tfl", "fig_num",2);
% app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f","fig_num",2);
app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);% 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
% app.logger.plot({1, "input", ""}, "phase","f", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({{1, "reference.result.state.xd1:3", ""},{1,"reference.result.state.xd5:7",""}},"phase","f","fig_num",4);
% app.logger.plot({{1, "reference.result.state.xd9:11", ""},{1,"reference.result.state.xd13:15",""}},"phase","f","fig_num",5);
% disp_bode(app.logger, "f")
% %周波数応答の確認
% % phase f のみのデータを logger.data で直接取得
% % U  = app.logger.data(1, "controller.result.tmp", "", "phase", "f");
% Fs = 1/0.025;
% u   = app.logger.data(1, "controller.result.tmp", "", "phase", "f");
% pL  = app.logger.data(1, "estimator.result.state.pL", "e", "phase", "f");
% pLq = app.logger.data(1, "estimator.result.state.q", "e", "phase", "f");
% size(u)
% size(pL)
% size(pLq)   % 4列ならクォータニオン、3列ならオイラー角
% disp(size(pLq))
% figure;
% % --- 位置(z,x,y)の周波数応答 ---
% Fs = 1/0.025;
% 
% for i = 1:4
%     if i == 1
%         ci = 1; co = 3; use_pLq = false; nm = 'z';
%     elseif i == 2
%         ci = 3; co = 1; use_pLq = false; nm = 'x';
%     elseif i == 3
%         ci = 2; co = 2; use_pLq = false; nm = 'y';
%     else
%         ci = 4; co = 3; use_pLq = true;  nm = 'yaw';
%     end
% 
%     if use_pLq
%         [h,f] = tfestimate(u(:,ci), pLq(:,co), [], [], [], Fs);
%     else
%         [h,f] = tfestimate(u(:,ci), pL(:,co), [], [], [], Fs);
%     end
%     omega = 2*pi*f;
% 
%     % 0Hz成分を除いた正の周波数のみで桁(decade)を計算
%     omega_pos = omega(omega > 0);
%     dec_min = floor(log10(omega_pos(1)));
%     dec_max = ceil(log10(omega_pos(end)));
%     xticks_dec = 10.^(dec_min:dec_max);
% 
%     % --- 軸ごとに個別の横長の図を作成 ---
%     figure('Color','w', 'Position', [100 100 900 500]);
%     t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
%     title(t, nm, 'FontSize', 17);
% 
%     % ゲイン線図
%     nexttile;
%     semilogx(omega, 20*log10(abs(h)), 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Gain [dB]', 'FontSize', 15);
%     set(gca, 'FontSize', 13, 'XTick', xticks_dec);
%     xlim([omega_pos(1) omega(end)]);
% 
%     % 位相線図
%     nexttile;
%     semilogx(omega, unwrap(angle(h))*180/pi, 'LineWidth', 1.5);
%     grid on; box on;
%     ylabel('Phase [deg]', 'FontSize', 15);
%     xlabel('\omega [rad/s]', 'FontSize', 15);
%     set(gca, 'FontSize', 13, 'XTick', xticks_dec);
%     xlim([omega_pos(1) omega(end)]);
% end

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




function disp_bode(logger, phase)
% 周波数応答(Bode線図)とコヒーレンスを確認する関数
% disp_bode(app.logger, "f")
% u(n-1) と y(n) を対応させる（一制御周期前の入力を使用）
% 区間長はデータ長に応じて自動調整（n_segments分割）

Fs = 1/0.025;
n_segments = 4;   % 分割数（前回8→4に変更。区間を長くして低周波の分解能を優先）

u   = logger.data(1, "controller.result.tmp", "", "phase", phase);
pL  = logger.data(1, "estimator.result.state.pL", "e", "phase", phase);
pLq = logger.data(1, "estimator.result.state.q", "e", "phase", phase);

for i = 1:4
    if i == 1
        ci = 1; co = 3; use_pLq = false; nm = 'z';
    elseif i == 2
        ci = 3; co = 1; use_pLq = false; nm = 'x';
    elseif i == 3
        ci = 2; co = 2; use_pLq = false; nm = 'y';
    else
        ci = 4; co = 3; use_pLq = true;  nm = 'yaw';
    end

    if use_pLq
        y_out = pLq(:,co);
    else
        y_out = pL(:,co);
    end

    % --- u(n-1) と y(n) を対応させる ---
    N = length(y_out);
    u_n_minus_1 = u(1:N-1, ci);
    y_n         = y_out(2:N);

    % --- 区間長をデータ長から自動計算 ---
    Nd = length(y_n);
    wl = floor(Nd / n_segments * 2);
    wl = 2^floor(log2(wl));
    window = hann(wl);
    noverlap = round(wl/2);
    nfft = wl*2;

    [h,f]     = tfestimate(u_n_minus_1, y_n, window, noverlap, nfft, Fs);
    [cxy, fc] = mscohere(u_n_minus_1, y_n, window, noverlap, nfft, Fs);

    omega   = 2*pi*f;
    omega_c = 2*pi*fc;

    omega_pos = omega(omega > 0);
    dec_min = floor(log10(omega_pos(1)));
    dec_max = ceil(log10(omega_pos(end)));
    xticks_dec = 10.^(dec_min:dec_max);

    % --- Bode線図 ---
    figure('Color','w', 'Position', [100 100 900 500]);
    t = tiledlayout(2,1, 'TileSpacing','compact', 'Padding','compact');
    title(t, sprintf('%s (u(n-1) \\rightarrow y(n)), phase=%s, wl=%d', nm, phase, wl), 'FontSize', 17);

    nexttile;
    semilogx(omega, 20*log10(abs(h)), 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Gain [dB]', 'FontSize', 15);
    set(gca, 'FontSize', 13, 'XTick', xticks_dec);
    xlim([omega_pos(1) omega(end)]);

    nexttile;
    semilogx(omega, unwrap(angle(h))*180/pi, 'LineWidth', 1.5);
    grid on; box on;
    ylabel('Phase [deg]', 'FontSize', 15);
    xlabel('\omega [rad/s]', 'FontSize', 15);
    set(gca, 'FontSize', 13, 'XTick', xticks_dec);
    xlim([omega_pos(1) omega(end)]);

    % --- コヒーレンス ---
    figure('Color','w', 'Position', [1050 100 900 300]);
    semilogx(omega_c, cxy, 'LineWidth', 1.5);
    grid on; box on;
    ylim([0 1]);
    xlabel('\omega [rad/s]', 'FontSize', 15);
    ylabel('Coherence', 'FontSize', 15);
    title(sprintf('%s coherence (wl=%d)', nm, wl), 'FontSize', 15);
    set(gca, 'FontSize', 13, 'XTick', xticks_dec);
    xlim([omega_pos(1) omega(end)]);
end
end