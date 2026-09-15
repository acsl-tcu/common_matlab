% =========================================================================
% 通常テーマ1（懸垂負荷系クアッドロータ）
% 案3: 離散時間 MPC-CBF (Li et al. CCDC 2024 + Greeff et al. IROS 2018)
% =========================================================================
ts = 0;             % 初期時刻 [s]
dt = 0.025;         % サンプリング周期 [s] (40 Hz シミュレーションステップ)
te = 50;            % 終了時刻 [s]
time = TIME(ts, dt, te);

in_prog_func = @(app) in_prog(app);
post_func    = @(app) post(app);

logger = LOGGER(1, size(ts:dt:te, 2), 0, [], []); 
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示項目\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

% -------------------------------------------------------------------------
% 1. ドローン本体および懸垂負荷パラメータ設定
% -------------------------------------------------------------------------
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass", 0.14);   % 真の荷物質量 [kg]
agent.parameter.set("cableL",   2.0);    % ケーブル長 L [m]

initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p  = [0; 0; 0];
initial_state.pT = [0; 0; -1];           % ケーブル単位方向ベクトル (初期真下)
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT * agent.parameter.cableL;
initial_state.v  = [0; 0; 0];

% プラントモデル生成
agent.plant = MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent));

% 推定器側の初期推定質量 (モデル化誤差の設定)
agent.parameter.set("loadmass", 0.01);

% モーションキャプチャ模擬
motive = Connector_Natnet_sim(dt, {{1, "p", "q"}, {1, "pL", "pT"}});
motive.getData(agent);

% センサ割当
agent.sensor.set_function_class("motive", MOTIVE(agent, motive, ...
    "output_func", @motive_output, "rigid_id", [1, 2], "state_list", {["p", "q"], "p"}));

function y = motive_output(obj, data)
    % Motive 生データから姿勢および荷物位置・ケーブル方向を整形
    p  = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT / norm(pT);
    y  = [p; Quat2Eul(data.rigid(obj.rigid_id(1)).q); data.rigid(obj.rigid_id(2)).p; pT];
end

% 推定器割当 (EKF質量推定 + 懸垂状態管理)
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent, dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent, "Load_mL_HL")), ...
    ["p", "q", "pL", "pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));

L = agent.parameter.cableL;

% -------------------------------------------------------------------------
% 2. 参照軌道 (Reference) 生成層の設定
% -------------------------------------------------------------------------
% 公称直線軌道 (z方向に 1.5 m/s で高速上昇)
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", { ...
        "p0",        [0, 0, 3.0], ...     % 開始位置 (離陸完了高度と完全一致)
        "velocity",  1.5, ...             % 巡航速度 1.5 m/s
        "direction", [0, 0, 1] ...        % 進行方向 (鉛直上向き)
    }, 6});

% 【最新版】離散時間 MPC-CBF リプランナ設定 (Li et al. 2024 + Greeff 2018)
replan_opts = struct();
replan_opts.sensor_range = 6.0;   % 実機LiDAR模擬 探知距離 (面対面最短 6.0m)
replan_opts.safe_margin  = 0.4;   % 安全離隔マージン [m]
replan_opts.N_horiz      = 10;    % 予測ホライズンステップ数
replan_opts.dt_mpc       = 0.10;  % MPC 最適化周期 (10 Hz)
replan_opts.gamma_cbf    = 0.65;  % 離散CBF減衰パラメータ (0 < gamma <= 1)
replan_opts.r_load       = 0.15;  % 荷物半径 [m]
replan_opts.r_drone      = 0.30;  % 機体半径 [m]

agent.reference.set_function_class("timevarying", ...
    REPLANNING_LINEAR_MPC_CBF(agent, nominal_ref, replan_opts));

agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent, "zd", 3.0, "te", 5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent, "dt", dt, "zd", -L, "te", 3));

% -------------------------------------------------------------------------
% 3. 下位ハイレベルコントローラ (HLC)
% -------------------------------------------------------------------------
agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent, Controller_HL_Suspended_Load(dt, agent)));

% チャンネル割当
agent.set_cha_allocation_for_all("sensor", "motive");
agent.set_cha_allocation_for_all("estimator", ["ekf", "loadstate"]);
agent.cha_allocation.a.reference = ["takeoff", "sload"];
agent.cha_allocation.t.reference = ["takeoff", "sload"];
agent.cha_allocation.f.reference = ["timevarying", "sload"];
agent.cha_allocation.l.reference = ["landing", "sload"];

%% ========================================================================
%  事後解析・可視化コールバック関数群
% ========================================================================
function post(app)
    % シミュレーション終了後の結果プロット
    app.logger.plot({{1, "p", "er"}, {1, "estimator.result.state.pL", "e"}}, "ax", app.UIAxes, "phase", "tfl");
    app.logger.plot({{1, "p", "er"}, {1, "estimator.result.state.pL", "e"}}, "phase", "f", "fig_num", 2);
    app.logger.plot({{1, "v1:2", "er"}, {1, "estimator.result.state.vL1:2", ""}}, "fig_num", 3);
    
    % アニメーション描画
    show_suspended_load_animation(app);
end

function show_suspended_load_animation(app)
    if app.logger.k <= 1
        return
    end
    mov = DRAW_SUSPENDED_LOAD_MPC_CBF(app.logger, "target", 1, "self", app.agent(1));
    mov.animation(app.logger, "target", 1, "self", app.agent(1));
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
    v = sprintf("%c %6.3f : R [%6.3f,%6.3f,%6.3f] : P [%6.3f,%6.3f,%6.3f] : U [%6.3f,%6.3f,%6.3f,%6.3f] : mL %6.3f", ...
        agent(idx).cha, time.t, xd(1:3)', p', u', mL);
end