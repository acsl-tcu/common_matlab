% ts = 0; % initial time
% dt = 0.025; % sampling period
% te = 50; % termina time
% time = TIME(ts,dt,te);
% in_prog_func = @(app) in_prog(app);
% post_func = @(app) post(app);
% logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
% logger.display_func = @(agent, time) build_display_vector(agent, time);
% logger.display_on = true;
% fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");
% 
% % drone plant setting
% agent = DRONE;
% agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
% agent.parameter.set("loadmass",0.14);%0.0968);%0.968
% agent.parameter.set("cableL",2.0);%0.0968);%0.968
% initial_state.q  = [0; 0; 0];
% initial_state.w  = [0; 0; 0];
% initial_state.vL = [0; 0; 0];
% initial_state.p = [0; 0; 0];
% initial_state.pT = [0; 0; -1];
% initial_state.wL = [0; 0; 0];
% initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
% initial_state.v = [0; 0; 0];
% % Note: set the model error after setting "plant"
% agent.plant = MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state,1,agent));%dt,initial,id,agent,modelName
% agent.parameter.set("loadmass",0.01);%0.0968);%0.968
% 
% % Sim only: getData works after setting "plant"
% motive = Connector_Natnet_sim(dt, {{1,"p","q"},{1,"pL","pT"}}); % imitation of Motive camera (motion capture system)
% motive.getData(agent);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
% function y = motive_output(obj,data)
% % Motiveの生データから、機体姿勢と吊り荷位置・ケーブル方向を整理する。
% % 出力は推定器の入力として [p; euler; pL; pT] の形に整形する。
% p = data.rigid(obj.rigid_id(1)).p;
% pT = data.rigid(obj.rigid_id(2)).p - p;
% pT = pT/norm(pT);
% y = [p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
% end
% 
% agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent,dt,...
%     MODEL_CLASS(agent,Model_Suspended_Load(dt, initial_state, 1,agent,"Load_mL_HL")),...%"Load_mL_HL"
%     ["p", "q","pL","pT"])));%expの流用 質量推定有
% 
% agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
% L = agent.parameter.cableL;
% % agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",[0;0;1.5],"radius",[1,1,0]},6})); %円系軌道
% % agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[0;0;3.0], "p1",[2;2;3.0], "t_go",3.0, "t_hold",3.0, "t_back",3.0},6})); %P2P
% % agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",9,"center",[0;0;1.5],"radius",[1,1,0]},6})); % triangle
% % agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;3.0], "p1", [0;15.0;3.0], "t_go", 15.0}, 6}));
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;3.0], "p1", [0;0;30], "t_go", 60.0}, 6}));
% agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
% agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent,"zd",3.0,"te",5));
% agent.reference.set_function_class("landing", LANDING_REFERENCE(agent,"dt",dt,"zd",-L,"te",3)); % zd = -Lとするのがミソ：l移行時のrefは牽引物用なので
% 
% % 紐中点を対象とした Actuator CBF コントローラに置き換え
% % agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD_CBF(agent,Controller_HL_Suspended_Load(dt,agent)));
% agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD_MULTISPHERE_CBF(agent, Controller_HL_Suspended_Load(dt, agent)));
% 
% agent.set_cha_allocation_for_all("sensor","motive");
% agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
% agent.cha_allocation.a.reference =["takeoff","sload"]; % aも忘れずにセットする
% agent.cha_allocation.t.reference =["takeoff","sload"];
% agent.cha_allocation.f.reference =["timevarying","sload"];
% agent.cha_allocation.l.reference =["landing","sload"];
% 
% %%
% 
% function post(app)
% % シミュレーション終了後の結果表示とアニメーション作成。
% app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
% % app.logger.plot({1, "state.mL", "e"},"phase","tfl");
% %app.logger.plot({1, "estimator.result.ekf_mL", ""},"phase","tfl", "fig_num",2);
% % app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
% % app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
% app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f","fig_num",2);
% app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);% 速度: v_x, v_y, v_z
% 
% % ★ 表面からの距離（d_surf）の推移をプロット
% % 値が 0 以上であれば、設定したシステムマージン（r_system=1.0m）が保たれていることを示します。
% app.logger.plot({1, "controller.result.d_surf", ""}, "phase", "tf", "fig_num", 10);
% 
% show_suspended_load_animation(app); % アニメーション描画
% end
% 
% function show_suspended_load_animation(app)
% % 単機の吊り下げモデルをアニメーション表示する。
% if app.logger.k <= 1
%     return
% end
% 
% mov = DRAW_SUSPENDED_LOAD(app.logger, ...
%     "target", 1, ...
%     "self", app.agent(1));
% 
% % 障害物（楕円体）の描画を追加
% % ★ 環境ファイルから楕円体パラメータを読み込み、マージンを含めて描画します
% obs_list = ENVIRONMENT_OBSTACLE();
% hold(mov.ax, 'on');
% for i = 1:length(obs_list)
%     [sx, sy, sz] = sphere(30); % 解像度を少し上げる
%     p_obs = obs_list(i).p_obs;
%     R_obs = obs_list(i).R_obs;
%     Q_obs = obs_list(i).Q_obs;
%     d_margin = obs_list(i).d_margin;
% 
%     % 描画用の形状行列 (本来の楕円体 + 安全マージン)
%     Q_draw = Q_obs + d_margin * eye(3);
% 
%     % 単位球の各頂点を楕円体に変形・回転・平行移動
%     pts = [sx(:), sy(:), sz(:)]';
%     pts_trans = R_obs * Q_draw * pts + p_obs;
% 
%     sx_trans = reshape(pts_trans(1,:), size(sx));
%     sy_trans = reshape(pts_trans(2,:), size(sy));
%     sz_trans = reshape(pts_trans(3,:), size(sz));
% 
%     surf(mov.ax, sx_trans, sy_trans, sz_trans, ...
%         'FaceColor', 'red', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
% end
% 
% mov.animation(app.logger,"target", 1,"self", app.agent(1));%表示だけ用
% 
% % mov.animation(app.logger,"target", 1,"self", app.agent(1),"mp4", true, "pause", 0);%mp4保存用
% 
% % mov.animation(app.logger, "target", 1,"self", app.agent(1), "gif", "Data/suspended_load.gif", ...
% %     "fps", 20,"gif_delay", 0.05,"skip", 2, "pause", 0);%gif保存用
% end
% 
% function in_prog(app)
% % 実行中に推定状態をUIに表示する。
% app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
% end
% 
% 
% 
% function v = build_display_vector(agent, time)
% % コンソール表示用の文字列を作る。
% % 参照位置、推定位置、入力、推定質量を並べる。
% idx = 1;
% if ~isprop(agent(idx).reference.result.state, "xd")
%     v = [];
%     return
% end
% xd = agent(idx).reference.result.state.xd;
% if isfield(agent(idx).estimator.result, "state")
%     p = agent(idx).estimator.result.state.p;
%     if isprop(agent(idx).estimator.result.state, "mL")
%         mL = agent(idx).estimator.result.state.mL;
%     else
%         mL = NaN;
%     end
% else
%     p = [NaN; NaN; NaN];
%     mL = NaN;
% end
% u = agent(idx).controller.result.input;
% v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f] : mL %7.3f",agent(idx).cha, time.t, xd(1:3)', p', u', mL);
% end

ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % target, number, fExp, items, agent_items, option
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass", 0.14);
agent.parameter.set("cableL", 2.0);
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT*agent.parameter.cableL;
initial_state.v = [0; 0; 0];

% Note: set the model error after setting "plant"
agent.plant = MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent));
agent.parameter.set("loadmass", 0.01);

% Sim only: getData works after setting "plant"
motive = Connector_Natnet_sim(dt, {{1, "p", "q"}, {1, "pL", "pT"}});
motive.getData(agent);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% drone setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
agent.sensor.set_function_class("motive", MOTIVE(agent, motive, "output_func", @motive_output, "rigid_id", [1,2], "state_list", {["p","q"],"p"}));

function y = motive_output(obj, data)
    p = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT / norm(pT);
    y = [p; Quat2Eul(data.rigid(obj.rigid_id(1)).q); data.rigid(obj.rigid_id(2)).p; pT];
end

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent, dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent, "Load_mL_HL")), ...
    ["p", "q", "pL", "pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));

L = agent.parameter.cableL;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;3.0], "p1", [0;0;30], "t_go", 60.0}, 6}));
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;3.0], "p1", [0;15.0;3.0], "t_go", 15.0}, 6}));
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent, "zd", 3.0, "te", 5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent, "dt", dt, "zd", -L, "te", 3));

% 2層幾何CBFコントローラの割り当て
agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD_MULTISPHERE_CBF(agent, Controller_HL_Suspended_Load(dt, agent)));

agent.set_cha_allocation_for_all("sensor", "motive");
agent.set_cha_allocation_for_all("estimator", ["ekf", "loadstate"]);
agent.cha_allocation.a.reference = ["takeoff", "sload"];
agent.cha_allocation.t.reference = ["takeoff", "sload"];
agent.cha_allocation.f.reference = ["timevarying", "sload"];
agent.cha_allocation.l.reference = ["landing", "sload"];

%%
function post(app)
    % ========================================================
    % 1. 基本プロット
    % ========================================================
    app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}}, "phase", "f", "fig_num", 2);
    app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}}, "fig_num", 3);
    app.logger.plot({1, "controller.result.d_surf", ""}, "phase", "tf", "fig_num", 10);
    app.logger.plot({1, "controller.result.controllertime_total", ""}, "phase", "f", "fig_num", 4);
    app.logger.plot({1, "estimator.result.state.mL", ""}, "phase", "tf", "fig_num", 32);
    % ========================================================
    % 2. 時系列ログデータの安全な抽出
    % ========================================================
    if iscell(app.logger.Data.agent)
        agent_log = app.logger.Data.agent{1};
    else
        agent_log = app.logger.Data.agent(1);
    end
    
    res_log = agent_log.controller.result;
    
    % 実際の有効ステップ数を安全に判定 (最小でも2点以上確保)
    N_total = min(numel(res_log), length(app.logger.Data.t));
    if isprop(app.logger, 'k') && app.logger.k > 1
        N_steps = min(N_total, app.logger.k - 1);
    else
        N_steps = N_total;
    end
    
    t_span = app.logger.Data.t(1:N_steps);
    
    u_mod     = zeros(4, N_steps);
    u_nom     = zeros(4, N_steps);
    u_diff    = zeros(4, N_steps);
    c_time    = zeros(1, N_steps);
    att_deg   = zeros(3, N_steps);
    cable_deg = zeros(1, N_steps);
    cbf_act   = zeros(1, N_steps);
    qp_stat   = zeros(1, N_steps);
    d_surf    = zeros(1, N_steps);
    
    for k = 1:N_steps
        if iscell(res_log)
            rk = res_log{k};
        else
            rk = res_log(k);
        end
        
        if isempty(rk); continue; end
        
        if isfield(rk, 'input');          u_mod(:, k)  = rk.input(:); end
        if isfield(rk, 'u_nom');          u_nom(:, k)  = rk.u_nom(:); end
        if isfield(rk, 'u_diff');         u_diff(:, k) = rk.u_diff(:); end
        if isfield(rk, 'controllertime'); c_time(k)    = rk.controllertime; end
        if isfield(rk, 'drone_att_deg');  att_deg(:, k)= rk.drone_att_deg(:); end
        if isfield(rk, 'cable_tilt_deg'); cable_deg(k) = rk.cable_tilt_deg; end
        if isfield(rk, 'cbf_active');     cbf_act(k)   = rk.cbf_active; end
        if isfield(rk, 'qp_status');      qp_stat(k)   = rk.qp_status; end
        if isfield(rk, 'd_surf');         d_surf(k)    = rk.d_surf; end
    end
    
    % ========================================================
    % 3. 追加詳細グラフのプロット
    % ========================================================
    figure(11); clf; set(gcf, 'Name', '1. Control Inputs (Nominal vs CBF)');
    titles_u = {'Thrust T [N]', '\tau_x [Nm]', '\tau_y [Nm]', '\tau_z [Nm]'};
    for k = 1:4
        subplot(4, 1, k);
        plot(t_span, u_nom(k, :), 'r--', 'LineWidth', 1.2); hold on;
        plot(t_span, u_mod(k, :), 'b-', 'LineWidth', 1.5);
        ylabel(titles_u{k}); grid on;
        if k == 1
            legend('Nominal (Flatness)', 'Modified (Safe CBF)', 'Location', 'best');
            title('1. Control Input Comparison');
            ylim([-22, 22]); % 推力限界 [-20, 20]
            yline([-20, 20], 'k:', 'LineWidth', 1.0);
        else
            ylim([-1.2, 1.2]); % トルク限界 [-1, 1]
            yline([-1.0, 1.0], 'r--', 'LineWidth', 1.0);
        end
    end
    xlabel('Time [s]');
    
    figure(12); clf; set(gcf, 'Name', '2. Modification Amount');
    titles_du = {'\Delta Thrust [N]', '\Delta \tau_x [Nm]', '\Delta \tau_y [Nm]', '\Delta \tau_z [Nm]'};
    for k = 1:4
        subplot(4, 1, k);
        plot(t_span, u_diff(k, :), 'm-', 'LineWidth', 1.5); hold on;
        ylabel(titles_du{k}); grid on;
        if k == 1
            title('2. Modification Amount: u_{CBF} - u_{nom}');
        else
            ylim([-1.2, 1.2]); % トルク修正量を ±1.0 Nm 範囲に固定して視認性を確保
            yline([-1.0, 1.0], 'r--', 'LineWidth', 1.0);
        end
    end
    xlabel('Time [s]');
    
    figure(13); clf; set(gcf, 'Name', '3. Computation Time');
    plot(t_span, c_time, 'k-', 'LineWidth', 1.2); hold on;
    yline(25.0, 'r--', 'Sampling Limit (25 ms)');
    yline(mean(c_time), 'b:', sprintf('Mean: %.2f ms', mean(c_time)));
    ylabel('Time [ms]'); xlabel('Time [s]');
    title('3. Execution Time per Step (HOCBF-QP)'); grid on;
    ylim([0, max(30, max(c_time) * 1.2)]);
    
    figure(14); clf; set(gcf, 'Name', '4. Drone Attitude');
    att_names = {'Roll \phi [deg]', 'Pitch \theta [deg]', 'Yaw \psi [deg]'};
    for k = 1:3
        subplot(3, 1, k);
        plot(t_span, att_deg(k, :), 'b-', 'LineWidth', 1.5); hold on;
        if k <= 2
            yline([35, -35], 'r--', 'Limit (\pm 35 deg)');
            ylim([-45, 45]);
        end
        ylabel(att_names{k}); grid on;
        if k == 1; title('4. Drone Attitude Angles'); end
    end
    xlabel('Time [s]');
    
    figure(15); clf; set(gcf, 'Name', '5. Cable Swing Angle');
    plot(t_span, cable_deg, 'g-', 'LineWidth', 1.5); hold on;
    yline(45.0, 'r--', 'Soft Limit (45 deg)');
    ylabel('Tilt from Vertical [deg]'); xlabel('Time [s]');
    title('5. Suspended Load Cable Swing Angle'); grid on;
    ylim([0, max(50, max(cable_deg) * 1.1)]);
    
    figure(16); clf; set(gcf, 'Name', '6. Status');
    subplot(2, 1, 1);
    area(t_span, cbf_act, 'FaceColor', [1.0, 0.6, 0.2], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    ylabel('CBF Active'); ylim([0, 1.2]); yticks([0, 1]); yticklabels({'OFF', 'ACTIVE'});
    title('6. CBF Activation & QP Feasibility'); grid on;
    subplot(2, 1, 2);
    stairs(t_span, qp_stat, 'b-', 'LineWidth', 1.5);
    ylabel('QP Status'); ylim([-0.2, 1.2]); yticks([0, 1]); yticklabels({'Fallback', 'Optimal'});
    xlabel('Time [s]'); grid on;
    
    figure(17); clf; set(gcf, 'Name', '7. Mass Estimation');
    app.logger.plot({1, "estimator.result.state.mL", ""}, "phase", "tf", "fig_num", 17);
    yline(0.14, 'k--', 'True Mass (0.14 kg)', 'LineWidth', 1.5);
    title('7. Suspended Load Mass Estimation (EKF)'); grid on;
    
    figure(18); clf; set(gcf, 'Name', '8. Surface Clearance Distance');
    plot(t_span, d_surf, 'b-', 'LineWidth', 1.5); hold on;
    yline(0.0, 'r-', 'Collision Boundary (d_{surf} = 0)');
    yline(0.5, 'y--', 'Safety Margin Boundary (0.5 m)');
    ylabel('Clearance d_{surf} [m]'); xlabel('Time [s]');
    title('8. Minimum Euclidean Distance to Obstacle Surface'); grid on;
    
    % ========================================================
    % 4. アニメーション実行
    % ========================================================
    show_suspended_load_animation(app);
end
%%
function show_suspended_load_animation(app)
    if app.logger.k <= 1
        return
    end
    mov = DRAW_SUSPENDED_LOAD(app.logger, ...
        "target", 1, ...
        "self", app.agent(1));
    
    obs_list = ENVIRONMENT_OBSTACLE();
    hold(mov.ax, 'on');
    [sx0, sy0, sz0] = sphere(30);  % 単位球の頂点を一度だけ生成
    for i = 1:length(obs_list)
        p_obs    = obs_list(i).p_obs;
        r_obs    = obs_list(i).r_obs;     % 球体障害物の半径
        d_margin = obs_list(i).d_margin;
        
        % ① 障害物実体コア → 赤色 (ハード制約)
        surf(mov.ax, ...
            r_obs * sx0 + p_obs(1), ...
            r_obs * sy0 + p_obs(2), ...
            r_obs * sz0 + p_obs(3), ...
            'FaceColor', [0.85, 0.1, 0.1], 'FaceAlpha', 0.8, 'EdgeColor', 'none');
        
        % ② コア安全境界 (r_obs + r_sys=0.4m) → 赤い薄いシェル
        r_core = r_obs + 0.4;
        surf(mov.ax, ...
            r_core * sx0 + p_obs(1), ...
            r_core * sy0 + p_obs(2), ...
            r_core * sz0 + p_obs(3), ...
            'FaceColor', [0.85, 0.1, 0.1], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
        
        % ③ マージン外殻 → オレンジ色・半透明 (ソフト制約)
        r_mrg = r_obs + 0.4 + d_margin;
        surf(mov.ax, ...
            r_mrg * sx0 + p_obs(1), ...
            r_mrg * sy0 + p_obs(2), ...
            r_mrg * sz0 + p_obs(3), ...
            'FaceColor', [1.0, 0.55, 0.0], 'FaceAlpha', 0.20, 'EdgeColor', 'none');
    end
    mov.animation(app.logger, "target", 1, "self", app.agent(1));
    % mp4保存する場合は以下のコメントを外す:
    % mov.animation(app.logger, "target", 1, "self", app.agent(1), "mp4", true, "pause", 0);
end

function in_prog(app)
    app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
end

function v = build_display_vector(agent, time)
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

