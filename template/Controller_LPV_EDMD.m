function Controller = Controller_LPV_EDMD(dt, model_file, agent)
% Controller_LPV_EDMD
% LPV_EDMD_FUSION_CONTROLLER のパラメータ設定ファイル
%
% 融合観測量: z = [z_edmd(26); z_lpv(45)] = 71次元
% 代価関数:   ||z(k) - z_ref(k)||²_Q  統一形式
% z_ref:      [klift_edmd(x_ref); klift_lpv(x_ref)] で計算
%
% 呼び出し:
%   Controller = Controller_LPV_EDMD(dt, 'xxx.mat', agent);

    %% ── 基本設定 ──────────────────────────────────────────────
    Controller = Controller_HL(dt);
    Controller.dt_drone = Controller.dt;

    Controller.jx      = agent.parameter.jx;
    Controller.jy      = agent.parameter.jy;
    Controller.jz      = agent.parameter.jz;
    Controller.gravity = agent.parameter.gravity;
    Controller.m       = agent.parameter.mass;   % 名義質量

    Controller.state_size = 12;
    Controller.input_size = 4;
    Controller.total_size = Controller.state_size + Controller.input_size;
    Controller.Kmodel     = model_file;

    Controller.dt = 0.025;
    Controller.H  = 12;
    Controller.particle_num = 50000;

    Controller.input.Bestcost_now = [1e5, 1e3];
    Controller.input.Initsigma    = [0.1; 1.5e-2; 1.5e-2; 1.5e-2];
    Controller.input.Maxsigma     = [1; 1e-3; 1e-3; 1e-3];
    Controller.input.Minsigma     = [0.01; 1e-5; 1e-5; 1e-5];
    Controller.input.u            = [Controller.m * 9.81; 0; 0; 0];

    torque_th = 0.5; thrust_th = 1.5;
    Controller.input_max = [Controller.m*9.81+thrust_th;  torque_th;  torque_th;  torque_th];
    Controller.input_min = [Controller.m*9.81-thrust_th; -torque_th; -torque_th; -torque_th];
    Controller.ref_input = Controller.input.u;
    Controller.input.lb  = [0; -1; -1; -1];
    Controller.input.ub  = [10;  1;  1;  1];

    %% ── EDMDモデル読み込み ────────────────────────────────────
    load(model_file, 'est');
    [Controller.koopman.A_edmd, Controller.koopman.B_edmd, ~] = AB_transfer(est.A, est.B, est.C, dt, Controller.dt);
    n_edmd = size(Controller.koopman.A_edmd, 1);   % = 26
    fprintf('[LPV_EDMD] EDMD: n=%d  file: %s\n', n_edmd, model_file);

    %% ── LPVリフト関数（F_edmd）をcontrollerに渡す ────────────
    % select_observableでEDMDのリフト関数を取得
    [F_edmd, ~] = select_observable(model_file);
    % LPV_EDMD_FUSION_CONTROLLERのklift_edmdはこれを呼ぶ
    Controller.F_edmd = F_edmd;

    %% ── 解析LPV A行列（固定）────────────────────────────────
    M_lpv = 3; N_lpv = 2;
    n_lpv = 9*M_lpv + 9*N_lpv;   % = 45
    A_lpv = build_analytic_A(M_lpv, N_lpv);
    fprintf('[LPV_EDMD] LPV解析: n=%d  m_nominal=%.4f\n', n_lpv, Controller.m);

    %% ── B_lpv初期値（hover姿勢での名義値）──────────────────
    % 実行時は毎ステップ build_analytic_B で更新される
    % ここではzero初期化（コンストラクタで最初のステップまでに更新される）
    B_lpv_init = zeros(n_lpv, 4);
    B_lpv_init(3*M_lpv+1 : 3*M_lpv+3, 1) = (1/Controller.m)*[0;0;1];  % hover近似

    %% ── 融合A/B行列 ──────────────────────────────────────────
    % z = [z_edmd(26); z_lpv(45)]
    Controller.koopman.lpvA =  A_lpv;           % 71×71
    Controller.koopman.lpvB = B_lpv_init;             % 71×4

    %% ── 重み設定 ──────────────────────────────────────────────
    % EDMD部・LPV部それぞれの観測量スケールに合わせて設定
    % build_Q_stage()内でスケール係数を掛けるため、ここは物理単位で設定する

    % EDMD部（物理状態 p,q,v,ω に直接対応）
    Controller.weight.P = 1.3*diag([300;300;500]);    % 位置　10,20刻み  20;1;30
    Controller.weight.Q = 1e3*diag([1;1;1]);    % 速度  10,20刻み  30;20;10
    Controller.weight.V = diag([100;100;100]); % 15良い気がする
    Controller.weight.W = diag([100;100;100]);  % 姿勢角，角速度　1,2刻み 
    % Controller.weight.R = diag([100; 150; 150; 100]); % 入力
    % Controller.weight.RP = 0*diag([100; 100; 100; 100]);  % 1ステップ前の入力との差    0*(無効化)

    % LPV部はbuild_Q_stageでweight.P/Q/V/Wを流用しscale_lpv=0.01を掛ける
    % → LPV観測量スケールに自動調整される

    % 入力重み（両モデル共通、1つのQPで使用）
    Controller.weight.R  = diag([50; 20; 20; 20]);   % [F, τx, τy, τz]
    Controller.weight.RP = diag([10;  5;  5;  5]);   % 入力変化率

    Controller.weight.Pf = Controller.weight.P;
    Controller.weight.Qf = Controller.weight.Q;
    Controller.weight.Vf = Controller.weight.V;
    Controller.weight.Wf = Controller.weight.W;

    Controller.test.sigma = 1;
    Controller.test.input = 0;

    fprintf('[LPV_EDMD] 融合A=%dx%d  B=%dx%d\n', ...
        size(Controller.koopman.lpvA), size(Controller.koopman.lpvB));
    fprintf('[LPV_EDMD] コントローラ: LPV_EDMD_FUSION_CONTROLLER\n');

    Controller.name = "lpvedmd_fusion";
    Controller.type = "LPV_EDMD_FUSION_CONTROLLER";
end


%% ── ローカル関数 ──────────────────────────────────────────────

function A = build_analytic_A(M, N)
    % KQ_LMPC_CONTROLLERのget_Koopman_Aと完全一致
    Ap = zeros(9*M);
    Ap(1:3*(M-1),   4:3*M)        = eye(3*(M-1));
    Ap(1:3*(M-1),   3*M+1:6*M-3)  = eye(3*(M-1));
    Ap(3*M+1:6*M-3, 3*M+4:6*M)   = eye(3*(M-1));
    Ap(3*M+1:6*M-3, 6*M+1:9*M-3) = eye(3*(M-1));
    Ap(6*M+1:9*M-3, 6*M+4:9*M)   = eye(3*(M-1));
    Aso3 = zeros(9*N);
    for i = 1:N-1
        Aso3(9*i-8:9*i, 9*i+1:9*i+9) = eye(9);
    end
    A = blkdiag(Ap, Aso3);
end