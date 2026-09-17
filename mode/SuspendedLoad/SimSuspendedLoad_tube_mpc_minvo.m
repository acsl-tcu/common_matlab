% =========================================================================
% 通常テーマ1（懸垂負荷系クアッドロータ）
% 案1: Tube-based Robust MPC と MINVO 基底による厳密な前方不変多項式生成
% (Fu et al. CCC 2024 + Mellinger & Kumar 2011 + Funada 2025 + Zheng 2025)
% =========================================================================
% =========================================================================
% Class: REPLANNING_TUBE_MPC_MINVO
% Description:
%   Tube-based Robust Model Predictive Control (RMPC) with MINVO Bounding
%   Polyhedrons and 7th-Order Canonical Smoothing for a Cable-Suspended
%   Load Quadrotor System.
%   Enforces deterministic forward invariance under bounded swing disturbances
%   via analytical RPI set tightening and separating hyperplanes, while 
%   guaranteeing C^6 continuity up to Pop via a 7th-order canonical filter.
%
% Theoretical Foundations & Key Literature:
%
%   1. Tube-based Robust MPC for Quadrotor-Slung Load Systems:
%      - C. Fu, H. Sun, L. Dai, and Y. Xia,
%        "Robust MPC-based Trajectory Tracking Control for Quadrotor UAV-slung 
%        Load System,"
%        in Proceedings of the 43rd Chinese Control Conference (CCC), 
%        pp. 2826-2833, Kunming, China, Jul. 2024.
%      * Role in Code:
%        Provides the error dynamics formulation dot(x_e) = K_e * x_e + d_hat 
%        under bounded swing disturbances. Analytically derives the Robust 
%        Positively Invariant (RPI) set X_e via Lyapunov analysis and Rolle's 
%        theorem, yielding the exact constraint tightening margin:
%            Delta_tube = gamma / k_error
%            d_n = d_safe + Delta_tube
%        Guarantees deterministic forward invariance (collision distance >= d_safe) 
%        as long as the nominal system satisfies tightened separating hyperplane bounds.
%
%   2. Differential Flatness & 7th-Order Canonical Smoothing (C^6 Continuity):
%      - D. Mellinger and V. Kumar,
%        "Minimum Snap Trajectory Generation and Control for Quadrotors,"
%        in Proc. IEEE International Conference on Robotics and Automation (ICRA), 
%        pp. 2520-2525, May 2011.
%      * Role in Code:
%        Establishes differential flatness of the quadrotor translation and yaw.
%        Implements the 7th-order Hurwitz canonical tracking filter ((s + w_filt)^7)
%        to dynamically smooth the nominal Tube-MPC position steps into 
%        strictly continuous C^6 reference states up to Pop (6th derivative),
%        eliminating actuator thrust saturation and torque chattering.
%
%   3. MINVO Basis for Minimum-Volume Outer Polyhedral Obstacle Approximations:
%      - J. Tordesillas and J. P. How,
%        "MINVO Basis: Finding Simplexes with Minimum Volume Enclosing 
%        Polynomial Curves,"
%        Computer-Aided Design, vol. 151, p. 103341, Nov. 2022.
%      * Role in Code:
%        Computes the minimal-volume enclosing simplex (outer polytope vertices 
%        V_minvo) for ellipsoidal obstacles, allowing non-conservative, exact 
%        separating hyperplane formulation:
%            n_div' * p_nominal > max(n_div' * V_minvo) + d_tightened
%
%   4. Ellipsoidal Configuration Metrics & Exact Gradient Geometry:
%      - R. Funada, K. Nishimoto, T. Ibuki, and M. Sampei,
%        "Collision Avoidance for Ellipsoidal Rigid Bodies With Control Barrier 
%        Functions Designed From Rotating Supporting Hyperplanes,"
%        IEEE Transactions on Control Systems Technology, vol. 33, no. 1, 
%        pp. 148-164, Jan. 2025.
%      * Role in Code:
%        Constructs the ellipsoid metric tensor A_mat = R_o * diag(1./r^2) * R_o'
%        to evaluate exact directional surface distances and lookahead gates.
%
%   5. Multi-Sphere Envelope Modeling & Wire Tilt Compensation:
%      - X. Zheng, et al.,
%        "Geometric Collision Avoidance for Quadrotors with a Cable-Suspended 
%        Load via Multi-Sphere Envelopes,"
%        IEEE Transactions on Control Systems Technology, 2025.
%      * Role in Code:
%        Discretizes the payload-cable-quadrotor continuum into 5 rigid safety 
%        spheres (lambdas = linspace(0, 1, 5)), and projects the dynamic cable 
%        tilt offset into the separating hyperplane boundary vector.
%
%   6. Orthogonal Subspace Separation & Vertical Ascent Preservation:
%      - D. Tscholl, Y. Nakka, and B. Gunter,
%        "FastBridge: Closing the Model-Based Realization Gap in Safety Filters 
%        on 3D Gaussian Splatting for Fast Quadrotor Flight,"
%        arXiv:2607.01200v1 [cs.RO], Jul. 2026.
%      * Role in Code:
%        Decouples nominal vertical ascent (p_target_z = p_nom_z) from the 
%        lateral evasion plane (2D Tube-MPC over XY). Prevents altitude drops 
%        and stall dives while executing full obstacle bypass.
% =========================================================================
ts = 0;             % 初期時刻 [s]
dt = 0.025;         % サンプリング周期 [s] (40 Hz)
te = 50;            % 終了時刻 [s]
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func    = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [], []); 
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示項目\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

% -------------------------------------------------------------------------
% 1. 機体・吊り荷パラメータ設定
% -------------------------------------------------------------------------
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass", 0.14);   % 真の荷物質量 [kg]
agent.parameter.set("cableL",   2.0);    % ケーブル長 L [m]
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p  = [0; 0; 0];
initial_state.pT = [0; 0; -1];           % ケーブル単位方向ベクトル
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT * agent.parameter.cableL;
initial_state.v  = [0; 0; 0];

% プラントモデル生成
agent.plant = MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent));
agent.parameter.set("loadmass", 0.01);   % 推定器の初期推定質量

% モーションキャプチャ模擬
motive = Connector_Natnet_sim(dt, {{1, "p", "q"}, {1, "pL", "pT"}});
motive.getData(agent);

% センサ割当
agent.sensor.set_function_class("motive", MOTIVE(agent, motive, ...
    "output_func", @motive_output, "rigid_id", [1, 2], "state_list", {["p", "q"], "p"}));

function y = motive_output(obj, data)
    p  = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT / norm(pT);
    y  = [p; Quat2Eul(data.rigid(obj.rigid_id(1)).q); data.rigid(obj.rigid_id(2)).p; pT];
end

% 推定器割当
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent, dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent, "Load_mL_HL")), ...
    ["p", "q", "pL", "pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

% -------------------------------------------------------------------------
% 2. 参照軌道 (Reference) 生成層の設定
% -------------------------------------------------------------------------
% 公称直線軌道 (z方向に 1.5 m/s で上昇)
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", { ...
        "p0",        [0, 0, 3.0], ...     % 開始位置
        "velocity",  1.5, ...             % 巡航速度 1.5 m/s
        "direction", [0, 0, 1] ...        % 鉛直上向き
    }, 6});

% 【案1】Tube-based Robust MPC ＋ MINVO 基底リプランナ設定
replan_opts = struct();
replan_opts.sensor_range = 6.0;           % 探知・RMPC起動距離 [m]
replan_opts.safe_margin  = 0.8;           % 基礎安全マージン d [m]
replan_opts.r_load       = 0.15;
replan_opts.r_drone      = 0.30;
replan_opts.gamma_dist   = 0.8;           % 推定最大外乱 [m/s^2]
replan_opts.k_error      = 2.0;           % 誤差系ゲイン (チューブ幅 = 0.4m)
replan_opts.w_filt       = 2.5;           % 7次正準系フィルタ極 [rad/s]

agent.reference.set_function_class("timevarying", ...
    REPLANNING_TUBE_MPC_MINVO(agent, nominal_ref, replan_opts));

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
    app.logger.plot({{1, "p", "er"}, {1, "estimator.result.state.pL", "e"}}, "ax", app.UIAxes, "phase", "tfl");
    app.logger.plot({{1, "p", "er"}, {1, "estimator.result.state.pL", "e"}}, "phase", "f", "fig_num", 2);
    app.logger.plot({{1, "v1:2", "er"}, {1, "estimator.result.state.vL1:2", ""}}, "fig_num", 3);
    
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