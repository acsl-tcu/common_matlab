% =========================================================================
% 通常テーマ1（懸垂負荷系クアッドロータ）
% 案5: 階層型セーフティフィルタ (Softplus-CBF) ＋ MPC (Cohen et al. 2023)
% =========================================================================
% =========================================================================
% Class: REPLANNING_MPC_SMOOTH_CBF
% Description:
%   Hierarchical Safety-Critical Controller for a Quadrotor with a 
%   Cable-Suspended Load (QCSP) System avoiding 3D Ellipsoidal Obstacles.
%   Seamlessly combines Coarse-grid Receding Horizon MPC, C^inf Smooth 
%   Softplus-CBF Safety Filtering, Multi-Sphere Cable Geometric Enveloping, 
%   and 7th-Order Canonical Flatness Smoothing.
%
% Theoretical Foundations & Key Literature:
%
%   1. Smooth Safety Filters via the Implicit Function Theorem (C^inf Invariance):
%      - M. H. Cohen, P. Ong, G. Bahati, and A. D. Ames,
%        "Characterizing Smooth Safety Filters via the Implicit Function Theorem,"
%        arXiv:2309.12614v1 [eess.SY], Sep. 2023.
%      * Role in Code:
%        Replaces non-smooth ReLU / hard-QP barrier constraints with the C^inf
%        smooth universal formula (Eq. 27b):
%            lambda_softplus(a, b) = (1 / beta) * ln(1 + exp(-beta * a / b))
%        Guarantees deterministic forward invariance of the safe set without
%        introducing kinks, chattering, or discontinuities in higher derivatives.
%
%   2. Multi-Sphere Cable Envelopes & Relative-Degree-2 HOCBF for QCSP Systems:
%      - Z. Zheng, S. Tong, Y. Zhang, Y. Zhu, T. Li, and J. Shao,
%        "Robust Safe Control for Nonlinear Quadrotor with a Cable-Suspended 
%        Payload Systems via Control Barrier Function and Disturbance Estimator,"
%        Control Engineering Practice, vol. 165, p. 106564, 2025.
%      * Role in Code:
%        Models the continuous payload-cable-quadrotor system as a discretized
%        chain of rigid protection spheres (lambdas = linspace(0, 1, 5)).
%        Constructs high-order relative-degree-2 barrier metrics:
%            h_j(p) = 0.5 * (||p_sph_j - p_obs||^2 - (r_obs + r_sph_j)^2) >= 0
%        Enables strict envelope collision avoidance for the quadrotor, 
%        cable continuum, and payload simultaneously.
%
%   3. Exact 3D Ellipsoidal Geometry & Directional Metric Tensors:
%      - R. Funada, K. Nishimoto, T. Ibuki, and M. Sampei,
%        "Collision Avoidance for Ellipsoidal Rigid Bodies With Control Barrier 
%        Functions Designed From Rotating Supporting Hyperplanes,"
%        IEEE Transactions on Control Systems Technology, vol. 33, no. 1, 
%        pp. 148-164, Jan. 2025.
%      * Role in Code:
%        Integrates exact 3D ellipsoidal surface metrics via the tensor:
%            A_mat = R_obs * diag(1 ./ radii.^2) * R_obs'
%        Evaluates directional surface distances r_eff_xy and Mahalanobis 
%        clearance margins for arbitrary 3D obstacle configurations.
%
%   4. Differential Flatness & 7th-Order Canonical Smoothing (C^6 Continuity):
%      - D. Mellinger and V. Kumar,
%        "Minimum Snap Trajectory Generation and Control for Quadrotors,"
%        in Proc. IEEE International Conference on Robotics and Automation (ICRA), 
%        pp. 2520-2525, May 2011.
%      * Role in Code:
%        Implements the 7th-order Hurwitz canonical tracking filter ((s + w_filt)^7)
%        to dynamically smooth discrete MPC position steps into continuous C^6 
%        reference trajectories up to Pop (28-dimensional output vector xd),
%        preventing actuator thrust saturation and torque chattering in HLC.
%
%   5. Frenet-Serret Dynamic Null-Space Decomposition (Ascent Preservation):
%      - D. Tscholl, Y. Nakka, and B. Gunter,
%        "FastBridge: Closing the Model-Based Realization Gap in Safety Filters 
%        on 3D Gaussian Splatting for Fast Quadrotor Flight,"
%        arXiv:2607.01200v1 [cs.RO], Jul. 2026.
%      * Role in Code:
%        Decouples nominal feedforward velocity (along t_head) from the lateral 
%        evasion subspace (2D normal plane E_norm = [n1, n2]). Prevents downward 
%        dives, reverse-thrust stalls, and trapping under convex obstacles.
% =========================================================================
ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func    = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [], []); 
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示項目\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]  mL\n\n");

% drone plant setting
agent = DRONE;
agent.parameter = DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass", 0.14);
agent.parameter.set("cableL",   2.0);
initial_state.q  = [0; 0; 0];
initial_state.w  = [0; 0; 0];
initial_state.vL = [0; 0; 0];
initial_state.p  = [0; 0; 0];
initial_state.pT = [0; 0; -1];
initial_state.wL = [0; 0; 0];
initial_state.pL = initial_state.p + initial_state.pT * agent.parameter.cableL;
initial_state.v  = [0; 0; 0];

agent.plant = MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent));
agent.parameter.set("loadmass", 0.01);

motive = Connector_Natnet_sim(dt, {{1, "p", "q"}, {1, "pL", "pT"}});
motive.getData(agent);

agent.sensor.set_function_class("motive", MOTIVE(agent, motive, ...
    "output_func", @motive_output, "rigid_id", [1, 2], "state_list", {["p", "q"], "p"}));

function y = motive_output(obj, data)
    p  = data.rigid(obj.rigid_id(1)).p;
    pT = data.rigid(obj.rigid_id(2)).p - p;
    pT = pT / norm(pT);
    y  = [p; Quat2Eul(data.rigid(obj.rigid_id(1)).q); data.rigid(obj.rigid_id(2)).p; pT];
end

agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF_SuspendedLoad(agent, dt, ...
    MODEL_CLASS(agent, Model_Suspended_Load(dt, initial_state, 1, agent, "Load_mL_HL")), ...
    ["p", "q", "pL", "pT"])));
agent.estimator.set_function_class("loadstate", SUSPENDED_LOAD_STATE_MANAGER(agent));
L = agent.parameter.cableL;

% 公称直線軌道 (z方向に 1.5 m/s で上昇)
nominal_ref = TIME_VARYING_REFERENCE(agent, ...
    {"gen_ref_line", { ...
        "p0",        [0, 0, 3.0], ...
        "velocity",  1.5, ...
        "direction", [0, 0, 1] ...
    }, 6});

% リプランナ設定 (静的球指定を撤廃し、ENVIRONMENT_OBSTACLE_ELLIPSE と完全連携)
replan_opts = struct();
replan_opts.sensor_range  = 6.0;   % 6.0m 手前で動的計算開始
replan_opts.safe_margin   = 0.8;   % 楕円体外殻からの安全マージン
replan_opts.beta_softplus = 15.0;  % Cohen et al. Softplus パラメータ
replan_opts.w_filt        = 2.5;   % Mellinger 7次フィルタ極

agent.reference.set_function_class("timevarying", ...
    REPLANNING_MPC_SMOOTH_CBF(agent, nominal_ref, replan_opts));

agent.reference.set_function_class("sload", SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class("takeoff", TAKEOFF_REFERENCE(agent, "zd", 3.0, "te", 5));
agent.reference.set_function_class("landing", LANDING_REFERENCE(agent, "dt", dt, "zd", -L, "te", 3));

agent.controller.set_function_class("hlc_suspended", HLC_SUSPENDED_LOAD(agent, Controller_HL_Suspended_Load(dt, agent)));

agent.set_cha_allocation_for_all("sensor", "motive");
agent.set_cha_allocation_for_all("estimator", ["ekf", "loadstate"]);
agent.cha_allocation.a.reference = ["takeoff", "sload"];
agent.cha_allocation.t.reference = ["takeoff", "sload"];
agent.cha_allocation.f.reference = ["timevarying", "sload"];
agent.cha_allocation.l.reference = ["landing", "sload"];

%% コールバック
function post(app)
    app.logger.plot({{1, "p", "er"}, {1, "estimator.result.state.pL", "e"}}, "ax", app.UIAxes, "phase", "tfl");
    app.logger.plot({{1, "p", "er"}, {1, "estimator.result.state.pL", "e"}}, "phase", "f", "fig_num", 2);
    app.logger.plot({{1, "v1:2", "er"}, {1, "estimator.result.state.vL1:2", ""}}, "fig_num", 3);
    show_suspended_load_animation(app);
end

function show_suspended_load_animation(app)
    if app.logger.k <= 1, return; end
    % 楕円体障害物と緑色の予測軌道を描画する DRAW_SUSPENDED_LOAD_MPC_CBF を使用
    mov = DRAW_SUSPENDED_LOAD_MPC_CBF(app.logger, "target", 1, "self", app.agent(1));
    mov.animation(app.logger, "target", 1, "self", app.agent(1));
end

function in_prog(app)
    app.TextArea.Text = "estimator : " + app.agent.estimator.result.state.get();
end

function v = build_display_vector(agent, time)
    idx = 1;
    if ~isprop(agent(idx).reference.result.state, "xd"), v = []; return; end
    xd = agent(idx).reference.result.state.xd;
    if isfield(agent(idx).estimator.result, "state")
        p = agent(idx).estimator.result.state.p;
        if isprop(agent(idx).estimator.result.state, "mL"), mL = agent(idx).estimator.result.state.mL; else, mL = NaN; end
    else
        p = [NaN; NaN; NaN]; mL = NaN;
    end
    u = agent(idx).controller.result.input;
    v = sprintf("%c %6.3f : R [%6.3f,%6.3f,%6.3f] : P [%6.3f,%6.3f,%6.3f] : U [%6.3f,%6.3f,%6.3f,%6.3f] : mL %6.3f", ...
        agent(idx).cha, time.t, xd(1:3)', p', u', mL);
end

