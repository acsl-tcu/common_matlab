function Controller = Controller_NMPC(dt, agent)
% Controller_NMPC  CasADi 非線形MPC（NMPC_simple クラス）用のコントローラ設定
%
%   Controller = Controller_NMPC(dt, agent);
%   ctrl       = NMPC_simple(self, Controller);
%
% Controller_MPC_KMC_kyo と同じ構成（HL param / common param / weight）にしてあり、
% 末尾で NMPC_simple が読む Controller.nmpc にまとめ直している。
% Koopman モデルは使わないので model_file 引数は無し。

    %% HL param（フレームワーク共通の基礎設定を継承）
    Controller = Controller_HL(dt);
    Controller.dt_drone = Controller.dt;

    %% common param
    Controller.m          = agent.parameter.mass;
    Controller.state_size = 12;                 % [p(3); v(3); eul(3); w(3)]
    Controller.input_size = 4;                  % [T; tau_x; tau_y; tau_z]
    Controller.total_size = Controller.state_size + Controller.input_size;

    Controller.dt = 0.02;                       % NMPC ステップ幅 [s]
    Controller.H  = 12;                         % 予測ホライズン（ステップ数）

    Controller.input.u  = [Controller.m * 9.81; 0; 0; 0];   % 総推力，トルク（ホバリング）
    Controller.ref_input = Controller.input.u;               % 入力の目標値

    % --- 入力制約 ---
    % 2 通り用意：
    %   (a) lb/ub          : 絶対値の上下限（HL_MPC の出力クリップと同じ）
    %   (b) input_max/min  : ホバリング ±閾値（KMC 版と同じ考え方）
    % use_hover_band = true にすると (b) を NLP の制約に使う
    torque_th = 1;  thrust_th = 1.5;
    Controller.input_max = [Controller.m * 9.81 + thrust_th;  torque_th;  torque_th;  torque_th];
    Controller.input_min = [Controller.m * 9.81 - thrust_th; -torque_th; -torque_th; -torque_th];
    Controller.input.lb  = [0; -1; -1; -1];
    Controller.input.ub  = [10; 1;  1;  1];
    Controller.input.use_hover_band = false;

    %% weight
    % P : 位置 [x y z]        V : 速度 [vx vy vz]
    % Q : 姿勢角 [roll pitch yaw]   W : 角速度 [p q r]
    % R : 入力（ホバリングからの偏差）  RP : 1 ステップ前の入力との差（0* で無効化）
    % sim 用
    Controller.weight.P  = diag([80; 80; 100]);
    Controller.weight.V  = diag([2; 2; 2]);
    Controller.weight.Q  = diag([15; 15; 5]);
    Controller.weight.W  = diag([0.1; 0.1; 0.1]);
    Controller.weight.R  = diag([2; 30; 30; 15]);
    Controller.weight.RP = 0*diag([40; 20; 20; 20]);

    % 実験用（必要なら上と入れ替え）
    % Controller.weight.P  = 1.3*diag([300; 300; 500]);
    % Controller.weight.V  = diag([100; 100; 100]);
    % Controller.weight.Q  = 1e3*diag([1; 1; 1]);
    % Controller.weight.W  = diag([100; 100; 100]);
    % Controller.weight.R  = diag([100; 150; 150; 100]);
    % Controller.weight.RP = 0*diag([100; 100; 100; 100]);

    % 終端重み（デフォルトはステージ重みの terminal_scale 倍）
    Controller.weight.terminal_scale = 10;
    Controller.weight.Pf = Controller.weight.terminal_scale * Controller.weight.P;
    Controller.weight.Vf = Controller.weight.terminal_scale * Controller.weight.V;
    Controller.weight.Qf = Controller.weight.terminal_scale * Controller.weight.Q;
    Controller.weight.Wf = Controller.weight.terminal_scale * Controller.weight.W;

    %% NMPC 専用設定
    Controller.nmpc.angle_max_deg = 60;         % roll / pitch の上限 [deg]（90° 特異点回避）
    Controller.nmpc.substeps      = 1;          % 1 ステップ内の RK4 分割数（dt が粗いとき 2〜4）
    Controller.nmpc.max_iter      = 100;        % IPOPT 最大反復
    Controller.nmpc.tol           = 1e-5;
    Controller.nmpc.acceptable_tol = 1e-3;
    Controller.nmpc.print_level   = 0;          % 0: 無出力, 5: 反復表示
    Controller.nmpc.expand        = true;       % MX→SX 展開（小規模なら高速）

    %% NMPC_simple が読む形にまとめる（ここは触らない）
    Controller.nmpc.N    = Controller.H;
    Controller.nmpc.dt   = Controller.dt;
    Controller.nmpc.Q    = [diag(Controller.weight.P);  diag(Controller.weight.V);
                            diag(Controller.weight.Q);  diag(Controller.weight.W)]';
    Controller.nmpc.QN   = [diag(Controller.weight.Pf); diag(Controller.weight.Vf);
                            diag(Controller.weight.Qf); diag(Controller.weight.Wf)]';
    Controller.nmpc.R    = diag(Controller.weight.R)';
    Controller.nmpc.RP   = diag(Controller.weight.RP)';
    if Controller.input.use_hover_band
        Controller.nmpc.umin = Controller.input_min;
        Controller.nmpc.umax = Controller.input_max;
    else
        Controller.nmpc.umin = Controller.input.lb;
        Controller.nmpc.umax = Controller.input.ub;
    end
    Controller.nmpc.u_hover = Controller.input.u;

    fprintf("CasADi nonlinear MPC controller\n");
    fprintf("  dt = %.3f s, H = %d, horizon = %.2f s\n", ...
            Controller.dt, Controller.H, Controller.dt*Controller.H);
end