function Controller = Controller_MPC_LPVEDMD_kyo(dt, model_file, agent)
 %% HL param
    Controller = Controller_HL(dt);
    Controller.dt_drone = Controller.dt;
%% common param
    Controller.m = agent.parameter.mass;
    Controller.state_size = 12;
    Controller.input_size = 4;
    Controller.total_size = Controller.state_size + Controller.input_size;
    Controller.Kmodel = model_file;
    Controller.dt = 0.02;
    Controller.H = 12;
    Controller.particle_num = 50000;
    Controller.input.Maxinput = 1.5;
    Controller.input.Constinput = 10;
    Controller.input.range = [[10;30;30;10], [0.1;0.1;0.1;0.1]];
    Controller.input.Bestcost_now = [1e5, 1e3];
    Controller.input.Constsigma = 5.0*[1;1;1;1];
    Controller.input.Initsigma = [0.1;1.5e-2;1.5e-2;1.5e-2];
    Controller.input.Maxsigma = [1;1e-3;1e-3;1e-3];
    Controller.input.Minsigma = [0.01;1e-5;1e-5;1e-5];
    Controller.input.u = [Controller.m * 9.81;0;0;0];
    torque_th = 1; thrust_th = 1.5;
    Controller.input_max = [Controller.m * 9.81 + thrust_th; torque_th; torque_th; torque_th];
    Controller.input_min = [Controller.m * 9.81 - thrust_th;-torque_th;-torque_th;-torque_th];
    Controller.ref_input = Controller.input.u;
    Controller.input.lb = [0; -1; -1; -1];
    Controller.input.ub = [10; 1;  1;  1];

    %% ====== structured_koopman_model 読み込み ======
    kr = load('structured_koopman_model3.mat');
    if abs(kr.dt - Controller.dt) > 0.005
        fprintf('[警告] 学習dt=%.3f ≠ 制御dt=%.3f\n', kr.dt, Controller.dt);
        scale_dt = Controller.dt / kr.dt;
        Controller.koopman.A = kr.A_k ^ scale_dt;
        Controller.koopman.B = kr.B_k * scale_dt;
        Controller.koopman.C = kr.C_k;
    else
        Controller.koopman.A = kr.A_k;
        Controller.koopman.B = kr.B_k;
        Controller.koopman.C = kr.C_k;
    end
    Controller.koopman.input_scales = kr.input_scales;
    Controller.koopman.centers      = kr.centers;
    Controller.koopman.x_mean_rbf   = kr.x_mean_rbf;
    Controller.koopman.x_std_rbf    = kr.x_std_rbf;
    Controller.koopman.sigma_rbf    = kr.sigma_rbf;
    Controller.koopman.keep_phys    = kr.keep_phys;
    Controller.koopman.keep_edmd    = kr.keep_edmd;
    Controller.koopman.keep_comb    = kr.keep_comb;
    Controller.koopman.n_phys       = kr.n_phys;
    Controller.koopman.n_edmd       = kr.n_edmd;

    %% 重み設定（b_k+力矩スケーリングで補正済み→均衡設定）
 Controller.weight.P = 1*diag([600; 600; 800]);   % x/y位置: 200→600
Controller.weight.Q = diag([800; 800; 400]);   % x/y姿态: 500→800
Controller.weight.V = diag([300; 300; 800]);   % x/y速度稍降，z维持
Controller.weight.W = diag([30;  30;  50 ]);   % 角速度: 100→30（放松姿态惩罚）
Controller.weight.R = diag([80;  20;  20;  30]);  % 力矩R(2:4): 40/40/60→20/20/30
Controller.weight.RP = 0*diag([3; 1; 1; 1]);
Controller.weight.Pf = Controller.weight.P;
Controller.weight.Qf = Controller.weight.Q;
Controller.weight.Vf = Controller.weight.V;
Controller.weight.Wf = Controller.weight.W;
    %% z空間の重み変換
    nz = size(kr.A_k, 1);
    epsilon = 1e-8;
    Q_x  = blkdiag(Controller.weight.P,  Controller.weight.Q, ...
                    Controller.weight.V,  Controller.weight.W);
    Qf_x = blkdiag(Controller.weight.Pf, Controller.weight.Qf, ...
                    Controller.weight.Vf, Controller.weight.Wf);
    C_dec = kr.C_k;
    Controller.weight.stagestate    = C_dec' * Q_x  * C_dec + epsilon * eye(nz);
    Controller.weight.terminalstate = C_dec' * Qf_x * C_dec + epsilon * eye(nz);

    Controller.test.sigma = 1;
    Controller.test.input = 0;
    fprintf("Structured Koopman (LPV+EDMD+RBF) MPC controller\n")
    fprintf("n_z=%d  rho=%.4f\n", nz, max(abs(eig(kr.A_k))));
    Controller.name = "kmcmpc";
    Controller.type = "MPC_CONTROLLER_KMC_kyo";
end