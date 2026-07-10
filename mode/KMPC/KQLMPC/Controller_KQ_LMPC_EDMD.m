function Controller = Controller_KQ_LMPC_EDMD(dt, agent)
% =========================================================================
%  KQ-LMPC (Koopman LPV-MPC) パラメータ設定
%  核心: MPC設定 + 重み。补偿系(EDMD残差など)は residual.mode で切替(默认0=OFF)
%  ※ Koopman A/B は KQ_LMPC_EDMD_CONTROLLER 内で解析的に構成 (旧: .matから読込)
% =========================================================================
%% HL param
    Controller = Controller_HL(dt);
    Controller.dt_drone = Controller.dt;
%% common param
    Controller.jx = agent.parameter.jx;
    Controller.jy = agent.parameter.jy;
    Controller.jz = agent.parameter.jz;
    Controller.gravity = agent.parameter.gravity;
    Controller.m = agent.parameter.mass;
    Controller.state_size = 12;
    Controller.input_size = 4;
    Controller.total_size = Controller.state_size + Controller.input_size;
    Controller.dt = 0.025;          % MPCステップ幅
    Controller.H =20;              % predict horizon
    Controller.input.u = [Controller.m * 9.81; 0; 0; 0]; % 総推力，トルク
    torque_th = 1.5; thrust_th = 1.5;
    Controller.input_max = [Controller.m * 9.81 + thrust_th; torque_th; torque_th; torque_th];
    Controller.input_min = [Controller.m * 9.81 - thrust_th;-torque_th;-torque_th;-torque_th];
    Controller.ref_input = Controller.input.u; % 入力の目標値ー初設定
    Controller.input.Bestcost_now = [1e5, 1e3]; % show()/初期化用
    Controller.input.Initsigma = [0.1;1.5e-2;1.5e-2;1.5e-2]; % show()の表示専用
    %% ----- 旧モンテカルロ実装の残り (QP経路では未使用のためコメントアウト) -----
    % Controller.particle_num = 50000;  % mento carlo number of samples
    % Controller.input.Maxinput = 1.5;
    % Controller.input.Constinput = 10;
    % Controller.input.range = [[10;30;30;10], [0.1;0.1;0.1;0.1]]; % max min
    % Controller.input.Constsigma = 5.0*[1;1;1;1];
    % Controller.input.Maxsigma = [1;1e-3;1e-3;1e-3];
    % Controller.input.Minsigma = [0.01;1e-5;1e-5;1e-5];
    % Controller.input.lb = [0; -1; -1; -1];
    % Controller.input.ub = [10; 1;  1;  1];
    % Controller.du_max = [0.5; 0.1; 0.1; 0.05]; % 未使用 (Δu抑制は weight.RP のソフト惩罚で実施)
    % Controller.test.sigma = 1; % 標準偏差を固定
    % Controller.test.input = 0; % 推力以外の入力を0固定: 0:固定なし,1:トルク,2:自由

%% ===== [补偿·可选] EDMD残差补偿の設定 (mode=0 でOFF, デフォルト) =====
    Controller.residual.mode = 0; % 0: off(デフォルト), 1: LQR residual feedback, 2: one-step pseudo inverse, 3: torque EDMD + vertical PI, 5: mode2 + smoothing
    Controller.residual.model_file = 'C:\Users\acsl_students\Documents\GitHub_subfolder\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_kyo.mat';
    Controller.residual.alpha = 1.0;
    Controller.residual.du_max = [1.5; 0.3; 0.3; 0.3];
    Controller.residual.q_scale = 200.0;
    Controller.residual.r_scale = 0.02;
    Controller.residual.pinv_damping = 1e-2;
    Controller.residual.use_reference = 1;
    Controller.residual.use_aligned_reference = 1;
    Controller.residual.mode25_torque_only = 1;
    Controller.residual.z_kp = 1.5;
    Controller.residual.z_kv = 0.8;
    Controller.residual.z_ki = 0.6;
    Controller.residual.z_int_limit = 1.0;
    Controller.residual.torque_scale = [2.5; 2.5; 1.5];
    Controller.residual.torque_beta = 0.2;
    Controller.residual.full_beta = 0.1;
    Controller.residual.lqr_torque_only = 1;
    Controller.residual.lqr_beta = 0.2;

%% ===== 実験用 重み =====
  Controller.weight.P  = 1.2*diag([1200; 1200; 1200]);  % 位置: 2000→1200, 外环松30%+
Controller.weight.Q  = 0.5*diag([300; 500; 500]);     % 姿态: 维持验证值, yaw不动
Controller.weight.V  = 1.2*diag([700; 500; 500]);     % 速度: 1050→840, 同步降
Controller.weight.W  = 0.4*diag([220; 200; 200]);     % 角速度: 66→88, 相对阻尼反而变厚
Controller.weight.R  = 2*diag([1; 0.5; 0.5; 0.5]);    % 输入: 不动, 内环保持灵敏
Controller.weight.RP = 150*diag([1; 0.05; 0.05; 0.05]);
Controller.weight.RP_axis = [1; 50; 50; 20];          % yaw维持验证值, 这轮不碰

    Controller.weight.Pf = Controller.weight.P;
    Controller.weight.Vf = Controller.weight.V;
    Controller.weight.Qf = Controller.weight.Q;
    Controller.weight.Wf = Controller.weight.W;

%% 以下は変更なし
    fprintf("Koopman Quasilinear LPVMPC controller\n")
    Controller.name = "kqlmpc";
    Controller.type = "KQ_LMPC_CONTROLLER";
end