ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % termina time
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
agent.sensor.set_function_class("motive", MOTIVE(agent, motive,"output_func",@motive_output,"rigid_id",[1,2],"state_list",{["p","q"],"p"}));
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
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",[0;0;1.5],"radius",[1,1,0]},6})); %円系軌道
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_p2p_back_and_forth",{"p0",[0;0;3.0], "p1",[2;2;3.0], "t_go",3.0, "t_hold",3.0, "t_back",3.0},6})); %P2P
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_triangle",{"freq",9,"center",[0;0;1.5],"radius",[1,1,0]},6})); % triangle
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_line", {"p0", [0;0;3.0], "p1", [0;15.0;3.0], "t_go", 15.0}, 6}));
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, ...
%     {"gen_ref_p2p_line", { ...
%     "p0", [0; 0; 3.0], ...        % 開始位置
%     "direction", [0; 1; 0], ...   % 進行方向 (y方向)
%     "velocity", 0.5, ...          % 巡航速度 0.5 m/s
%     "t_acc", 3.0, ...             % 3秒かけて滑らかに加速
%     "t_start", 2.0 ...            % 開始2秒後から移動開始
%     }, 6}));
% =========================================================================
% 【検証軌道 5 種の切り替え設定】(引数形式を args = {関数名, {param...}, order} に修正)
% =========================================================================
% --- Case 1: Y軸正弦往復 (低速加減速) ---
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_case1_line", {"freq", 10.0, "center", [0, 0, 3.0], "radius", [0, 2.0, 0], "phase", 0}, 6}));

% --- Case 2: 垂直・縦方向急加減速 (上下バウンディング・ピッチ急変) ---
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_case2_vertical", {"freq", 8.0, "center", [0, 0, 3.0], "radius", [1.0, 0, 0.8], "phase", 0}, 6}));

% --- Case 3: 緩旋回 (低速一定曲率) ---
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_case3_slow_circle", {"freq", 12.0, "center", [1.5, 0, 3.0], "radius", [1.5, 1.5, 0], "phase", -pi}, 6}));

% --- Case 4: S字・8の字旋回 (曲率反転・ロール揺り返し) ---
% agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_case4_figure8", {"freq", 8.0, "center", [0, 0, 3.0], "radius", [2.0, 1.2, 0], "phase", 0}, 6}));

% --- Case 5: 急旋回＋3次元加速 (サドル型ストレステスト) ---
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_case5_fast_saddle", {"freq", 4.5, "center", [1.5, 0, 3.0], "radius", [1.5, 1.5, 0.5], "phase", -pi}, 6}));
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
% シミュレーション終了後の結果自動解析 (全体 / 初期10秒 / 10秒〜終了 3分割評価)
if app.logger.k > 1
    try
        % 1. agent ログ構造体の取得
        if iscell(app.logger.Data.agent)
            agent_log = app.logger.Data.agent{1};
        else
            agent_log = app.logger.Data.agent(1);
        end
        N_steps = app.logger.k - 1;

        p_drone  = zeros(3, N_steps);
        p_load   = zeros(3, N_steps);
        pL_ref   = zeros(3, N_steps);
        is_f     = false(1, N_steps);
        time_vec = zeros(1, N_steps);

        % 2. 1ステップずつ安全抽出
        for i = 1:N_steps
            % 時刻
            try
                time_vec(i) = app.logger.Data.t(i);
            catch
                time_vec(i) = (i - 1) * 0.025;
            end

            % (A) 機体実位置 p (plant.result{i}.state)
            try
                st_p = agent_log.plant.result{i}.state;
                if isprop(st_p, "p") || isfield(st_p, "p")
                    p_drone(:, i) = st_p.p(1:3);
                else
                    p_drone(:, i) = st_p(1:3);
                end
            catch
                p_drone(:, i) = [NaN; NaN; NaN];
            end

            % (B) 荷物推定位置 pL (estimator.result{i}.state.pL)
            try
                st_e = agent_log.estimator.result{i}.state;
                if isprop(st_e, "pL") || isfield(st_e, "pL")
                    p_load(:, i) = st_e.pL(1:3);
                else
                    p_load(:, i) = st_e(1:3);
                end
            catch
                p_load(:, i) = [NaN; NaN; NaN];
            end

            % (C) 荷物目標位置 pL_ref (reference.result{i}.state.xd)
            try
                st_r = agent_log.reference.result{i}.state;
                if isprop(st_r, "xd") || isfield(st_r, "xd")
                    raw_xd = st_r.xd;
                    pL_ref(:, i) = raw_xd(1:3);
                elseif isprop(st_r, "p") || isfield(st_r, "p")
                    pL_ref(:, i) = st_r.p(1:3);
                else
                    pL_ref(:, i) = st_r(1:3);
                end
            catch
                pL_ref(:, i) = [NaN; NaN; NaN];
            end

            % (D) フェーズ判定 (cha == 'f' または 時刻 5秒以降)
            try
                c = agent_log.cha(i);
                is_f(i) = isequal(c, 'f');
            catch
                is_f(i) = (time_vec(i) >= 5.0);
            end
        end

        % 3. 名目位置の作成 (pT = [0;0;-1] 準拠: +Zが上方, -Zが下方)
        L_val  = app.agent(1).parameter.cableL;
        pQ_ref = pL_ref + [0; 0; L_val];      % 機体名目目標位置
        pL_nom = p_drone - [0; 0; L_val];     % 荷物名目下垂位置

        % 4. 誤差系列・揺動変位・揺動角の計算
        err_drone_series = vecnorm(p_drone - pQ_ref, 2, 1);
        err_load_series  = vecnorm(p_load  - pL_ref, 2, 1);
        err_swing_series = vecnorm(p_load  - pL_nom, 2, 1);

        % 索相対ベクトル r = pL - pQ
        r_rel = p_load - p_drone;
        r_norm = vecnorm(r_rel, 2, 1);
        % 鉛直下向き [-0; -0; -1] との内積: -r_rel(3, :)
        cos_theta = -r_rel(3, :) ./ max(r_norm, 1e-6);
        cos_theta = min(1.0, max(-1.0, cos_theta)); % 数値誤差クリッピング
        theta_swing_deg_series = acos(cos_theta) * (180 / pi);

        % 5. 評価区間の 3 分割マスク作成
        idx_f_all = find(is_f);
        if isempty(idx_f_all)
            fprintf("[WARN] 飛行フェーズ 'f' のデータが存在しません。\n");
            return;
        end

        t_flight_start = time_vec(idx_f_all(1));
        t_split        = t_flight_start + 10.0;
        t_flight_end   = time_vec(idx_f_all(end));

        % 区間①: フライト全体
        idx_f_whole = is_f;
        % 区間②: フライト開始〜10秒 (過渡応答)
        idx_f_0_10  = is_f & (time_vec <= t_split);
        % 区間③: 10秒〜フライト終了 (定常追従)
        idx_f_10_end = is_f & (time_vec > t_split);

        % 6. 各区間の最大値抽出関数 (インライン安全抽出)
        get_max = @(series, mask) max(series(mask), [], 'omitnan');

        % --- 区間①: 全体 ---
        e_drone_all = get_max(err_drone_series, idx_f_whole);
        e_load_all  = get_max(err_load_series,  idx_f_whole);
        r_swing_all = get_max(err_swing_series, idx_f_whole);
        th_deg_all  = get_max(theta_swing_deg_series, idx_f_whole);

        % --- 区間②: 開始〜10秒 ---
        e_drone_0_10 = get_max(err_drone_series, idx_f_0_10);
        e_load_0_10  = get_max(err_load_series,  idx_f_0_10);
        r_swing_0_10 = get_max(err_swing_series, idx_f_0_10);
        th_deg_0_10  = get_max(theta_swing_deg_series, idx_f_0_10);

        % --- 区間③: 10秒〜終了 ---
        if any(idx_f_10_end)
            e_drone_10_end = get_max(err_drone_series, idx_f_10_end);
            e_load_10_end  = get_max(err_load_series,  idx_f_10_end);
            r_swing_10_end = get_max(err_swing_series, idx_f_10_end);
            th_deg_10_end  = get_max(theta_swing_deg_series, idx_f_10_end);
        else
            e_drone_10_end = NaN; e_load_10_end = NaN; r_swing_10_end = NaN; th_deg_10_end = NaN;
        end

        % 7. コンソール整形表示 (3カラム比較)
        fprintf("\n========================================================================================\n");
        fprintf(" 【本試行の運動・不確実性同定結果 (区間別 3 分割比較)】\n");
        fprintf("  フライト時間: %.2f [s] 〜 %.2f [s] (総飛行時間: %.2f [s])\n", ...
            t_flight_start, t_flight_end, t_flight_end - t_flight_start);
        fprintf("----------------------------------------------------------------------------------------\n");
        fprintf("  評価項目                        [フライト全体]    [開始〜10秒 (過渡)]   [10秒〜終了 (定常)]\n");
        fprintf("  1. ドローン追従誤差 (eps_drone):    %7.4f [m]         %7.4f [m]           %7.4f [m]\n", ...
            e_drone_all, e_drone_0_10, e_drone_10_end);
        fprintf("  2. 荷物追従誤差     (eps_load) :    %7.4f [m]         %7.4f [m]           %7.4f [m]\n", ...
            e_load_all,  e_load_0_10,  e_load_10_end);
        fprintf("  3. 索・荷物揺動変位 (r_swing)  :    %7.4f [m]         %7.4f [m]           %7.4f [m]\n", ...
            r_swing_all, r_swing_0_10, r_swing_10_end);
        fprintf("  4. 索最大傾斜角     (theta_deg):    %7.2f [deg]       %7.2f [deg]         %7.2f [deg]\n", ...
            th_deg_all,  th_deg_0_10,  th_deg_10_end);
        fprintf("========================================================================================\n\n");

    catch ME
        fprintf("[WARN] 誤差自動解析中にエラーが発生しました: %s\n", ME.message);
    end
end

% シミュレーション終了後の結果表示とアニメーション作成。
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"ax",app.UIAxes,"phase","tfl");
% app.logger.plot({1, "state.mL", "e"},"phase","tfl");
%app.logger.plot({1, "estimator.result.ekf_mL", ""},"phase","tfl", "fig_num",2);
% app.logger.plot({1, "p", "er"},"phase","tf", "fig_num",1); % 位置: p_x,p_y,p_z
% app.logger.plot({1, "q", "e"}, "phase","tfl", "fig_num",2 ); % 角度: θ_roll, θ_pitch, θ_yaw
app.logger.plot({{1, "p", "er"},{1, "estimator.result.state.pL", "e"}},"phase","f","fig_num",2);
app.logger.plot({{1, "v1:2", "er"},{1,"estimator.result.state.vL1:2",""}},"fig_num",3);% 速度: v_x, v_y, v_z
% app.logger.plot({1, "w", "e"}, "phase","tf", "fig_num",4); % 角速度: ω_roll, ω_ptich, ω_yaw
% app.logger.plot({1, "input", ""}, "phase","f", "fig_num",5); % 制御入力: Thrust, roll, pitch, yaw
% app.logger.plot({{1, "reference.result.state.xd1:3", ""},{1,"reference.result.state.xd5:7",""}},"phase","f","fig_num",4);
% app.logger.plot({{1, "reference.result.state.xd9:11", ""},{1,"reference.result.state.xd13:15",""}},"phase","f","fig_num",5);

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

% -------------------------------------------------------------------------
% Case 1: Y軸正弦往復 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case1_line(param)
arguments
    param.freq   = 10.0
    param.center = [0 0 3.0]  % 高度を 3.0 に統一
    param.radius = [0 2.0 0]
    param.phase  = 0          % sin(0) = 0 より [0, 0, 3.0] からスタート
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ origin(1); scale(2)*sin(w*t + phase) + origin(2); origin(3); 0 ];
end

% -------------------------------------------------------------------------
% Case 2: 垂直・縦方向急加減速 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case2_vertical(param)
arguments
    param.freq   = 8.0
    param.center = [0 0 3.0]  % 高度 3.0
    param.radius = [1.0 0 0.8]
    param.phase  = 0
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
% sin(w*t) にすることで t=0 で [0, 0, 3.0] からスムーズに立ち上がり
ref = @(t) [ scale(1)*sin(w*t + phase) + origin(1); ...
             origin(2); ...
             scale(3)*sin(2*w*t + phase) + origin(3); ...
             0 ];
end

% -------------------------------------------------------------------------
% Case 3: 緩旋回 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case3_slow_circle(param)
arguments
    param.freq   = 12.0
    param.center = [1.5 0 3.0] % 円の中心を X=+1.5 にずらす
    param.radius = [1.5 1.5 0] % 半径 1.5m
    param.phase  = -pi         % cos(-pi)=-1 より x = -1.5 + 1.5 = 0 からスタート
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ scale(1)*cos(w*t + phase) + origin(1); ...
             scale(2)*sin(w*t + phase) + origin(2); ...
             origin(3); ...
             0 ];
end

% -------------------------------------------------------------------------
% Case 4: S字・8の字旋回 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case4_figure8(param)
arguments
    param.freq   = 8.0
    param.center = [0 0 3.0]  % 高度 3.0
    param.radius = [2.0 1.2 0]
    param.phase  = 0          % sin(0)=0 より [0, 0, 3.0] からスタート
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ scale(1)*sin(w*t + phase) + origin(1); ...
             scale(2)*sin(2*w*t + phase) + origin(2); ...
             origin(3); ...
             0 ];
end

% -------------------------------------------------------------------------
% Case 5: 急旋回＋3次元加速 (初期値: [0, 0, 3.0])
% -------------------------------------------------------------------------
function ref = gen_ref_case5_fast_saddle(param)
arguments
    param.freq   = 4.5
    param.center = [1.5 0 3.0] % X中心を +1.5 にオフセット
    param.radius = [1.5 1.5 0.5]
    param.phase  = -pi         % t=0 で [0, 0, 3.0] から開始
end
T = param.freq; origin = param.center; scale = param.radius; phase = param.phase;
w = 2*pi/T; syms t real
ref = @(t) [ scale(1)*cos(w*t + phase) + origin(1); ...
             scale(2)*sin(w*t + phase) + origin(2); ...
             scale(3)*sin(2*w*t) + origin(3); ... % sin(0)=0 で Z も 3.0
             0 ];
end
