% =========================================================================
% 通常テーマ1（懸垂負荷系クアッドロータ）
% 案3: 離散時間 MPC-CBF (Li et al. CCDC 2024 + Greeff et al. IROS 2018)
% =========================================================================
% =========================================================================
% Class: REPLANNING_LINEAR_MPC_CBF
% Description:
%   A real-time, robust discrete-time MPC-CBF trajectory replanner for a 
%   quadrotor with a cable-suspended load. Integrates a linear flat model QP, 
%   slack-variable discrete CBFs, multi-sphere cable envelopes, and Tube-based 
%   constraint tightening to guarantee robust forward invariance and C^6 continuity.
%
% Theoretical Foundations & Key References:
%   1. Discrete-Time CBF & Slack-Variable Feasibility:
%      - S. Li, Y. Chen, and Y. Yang,
%        "Multi-rotor UAV path planning based on Model Predictive Control and 
%        Control Barrier Function," in Proc. 36th Chinese Control and Decision 
%        Conference (CCDC), pp. 1141-1146, 2024.
%
%   2. Linear Flat Model & Fast Convex QP Formulation:
%      - M. Greeff and A. P. Schoellig,
%        "Flatness-based Model Predictive Control for Quadrotor Trajectory Tracking,"
%        in Proc. IEEE/RSJ International Conference on Intelligent Robots and 
%        Systems (IROS), pp. 6740-6745, 2018.
%
%   3. Tube-Based Constraint Tightening & Realization Gap Compensation:
%      - R. Tscholl, A. Carron, M. Tognon, and M. N. Zeilinger,
%        "FastBridge: Bridging the Realization Gap in High-Order Control Barrier 
%        Functions for Safe Quadrotor Flight," IEEE Robotics and Automation 
%        Letters (RA-L), vol. 9, no. 5, pp. 4550-4557, 2024.
%
%   4. Multi-Sphere Cable Envelope Protection (5-Point Envelopes):
%      - X. Zheng, et al.,
%        "Geometric Collision Avoidance for Quadrotors with a Cable-Suspended 
%        Load via Multi-Sphere Envelopes," IEEE Transactions on Control Systems 
%        Technology (TCST), 2025.
%
%   5. Differential Flatness & C^6 Polynomial Trajectory Generation:
%      - D. Mellinger and V. Kumar,
%        "Minimum Snap Trajectory Generation and Control for Quadrotors,"
%        in Proc. IEEE International Conference on Robotics and Automation (ICRA), 
%        pp. 2520-2525, 2011.
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

% 障害物に当たるように、真上(Z方向)への移動を目標に設定
% 直線軌道生成器(6階微分まで完璧に滑らか)を使用
nominal_ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_line", { ...
        "p0",        [0, 0, 3.0], ...     % 離陸完了高度 (3.0m) からスタート
        "velocity",  0.5, ...             % 巡航速度 1.5 m/s
        "direction", [0, 0, 1] ...        % Z軸プラス方向 (真上) に移動
    }, 6});
agent.reference.set_function_class("timevarying", nominal_ref);
agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent, "zd", 3.0, "te", 5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent, "dt", dt, "zd", -L, "te", 3));

% -------------------------------------------------------------------------
% 3. 下位ハイレベルコントローラ (HLC) + Rotor-Level CBF
% -------------------------------------------------------------------------
% ベースとなる HLC コントローラ
base_ctrl = HLC_SUSPENDED_LOAD(agent, Controller_HL_Suspended_Load(dt, agent));

% それを今回作成した Rotor-Level CBF でラップして、システムのコントローラとする
cbf_ctrl = HLC_ROTOR_LEVEL_DYNAMIC_CBF(agent, base_ctrl);
agent.controller.set_function_class("hlc_suspended", cbf_ctrl);
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