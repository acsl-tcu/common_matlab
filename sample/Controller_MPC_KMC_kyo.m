function Controller = Controller_MPC_KMC_kyo(dt, model_file, agent)
 %% HL param
    Controller = Controller_HL(dt);
    Controller.dt_drone = Controller.dt;
%% common param
    Controller.m = agent.parameter.mass;
    Controller.state_size = 12;
    Controller.input_size = 4;
    Controller.total_size = Controller.state_size + Controller.input_size;
    Controller.Kmodel = model_file;
    Controller.dt = 0.025;          % MPCステップ幅
    Controller.H = 12;              %predict horizon
    Controller.particle_num = 50000;%mento carlo number of samples
    Controller.input.Maxinput = 1.5;
    Controller.input.Constinput = 10;
    Controller.input.range = [[10;30;30;10], [0.1;0.1;0.1;0.1]]; % max min
    Controller.input.Bestcost_now = [1e5, 1e3];
    Controller.input.Constsigma = 5.0*[1;1;1;1];
    Controller.input.Initsigma = [0.1;1.5e-2;1.5e-2;1.5e-2]; % default 0.1
    Controller.input.Maxsigma = [1;1e-3;1e-3;1e-3];
    Controller.input.Minsigma = [0.01;1e-5;1e-5;1e-5];
    Controller.input.u = [Controller.m * 9.81;0;0;0]; % 総推力，トルク
    torque_th = 1; thrust_th = 1.5;
    Controller.input_max = [Controller.m * 9.81 + thrust_th; torque_th; torque_th; torque_th];
    Controller.input_min = [Controller.m * 9.81 - thrust_th;-torque_th;-torque_th;-torque_th];
    Controller.ref_input = Controller.input.u; %入力の目標値ー初設定
    Controller.input.lb = [0; -1; -1; -1];
    Controller.input.ub = [10; 1;  1;  1];
    %% load model from koopman setting in the simxxx & change sampling time
   
    load(model_file, 'est');
    [Controller.koopman.A, Controller.koopman.B, Controller.koopman.C]  = AB_transfer(est.A, est.B, est.C, dt, Controller.dt);
    if isfield(est, 'Ae'); [Controller.koopman.Ae,Controller.koopman.Be,Controller.koopman.Ce] = AB_transfer(est.Ae, est.Be, est.Ce, dt, Controller.dt); end
    %-- 観測量の選択
    [Controller.F, Controller.code] = select_observable(model_file);
    %2025-07-15_Exp_Kyo_code00_randompp2 model 用重み
    % Controller.weight.P = 1*diag([200;200;200]);    % 位置　10,20刻み  20;1;30
    % Controller.weight.Q = 10*diag([10;10;10]);    % 速度  10,20刻み  30;20;10
    % Controller.weight.V = 1*diag([15;15;15]); % 15良い気がする
    % Controller.weight.W = 0.1*diag([5;5;0]);  % 姿勢角，角速度　1,2刻み 
    % Controller.weight.R = 0.01*diag([900; 100; 100; 3500]); % 入力
    % Controller.weight.RP = 0*diag([1; 0; 0; 0]);  % 1ステップ前の入力との差    0*(無効化)
   
    %%　実験　hovering
    Controller.weight.P = 1*diag([300;300;300]);    % 位置　10,20刻み  20;1;30
    Controller.weight.Q = 1*diag([10;10;10]);    % 速度  10,20刻み  30;20;10
    Controller.weight.V = 10*diag([15;15;15]); % 15良い気がする
    Controller.weight.W = 10*diag([10;30;0]);  % 姿勢角，角速度　1,2刻み 
    Controller.weight.R = 1*diag([1000; 100; 100; 3500]); % 入力
    Controller.weight.RP = 0*diag([1; 0; 0; 0]);  % 1ステップ前の入力との差    0*(無効化)
    %%
    % Controller.weight.P = 0.01*diag([5; 5; 5]);    % 位置　10,20刻み  20;1;30
    % Controller.weight.Q = 0.001*diag([30; 20; 10]);    % 速度  10,20刻み  30;20;10
    % Controller.weight.V = diag([15; 10; 10]); % 15良い気がする
    % Controller.weight.W = 10*diag([1; 1; 0]);  % 姿勢角，角速度　1,2刻み 
    % Controller.weight.R = 1*diag([100; 100; 100; 1]); % 入力
    % Controller.weight.RP = 1 * diag([1; 1; 1; 1]);  % 1ステップ前の入力との差
    Controller.weight.Pf = Controller.weight.P;
    Controller.weight.Vf = Controller.weight.V;
    Controller.weight.Qf = Controller.weight.Q;
    Controller.weight.Wf = Controller.weight.W;
    Controller.test.sigma = 1; % 標準偏差を固定
    Controller.test.input = 0; % 推力以外の入力を0固定: 0:固定なし,1:トルク,2:自由
    %% 以下は変更なし
    fprintf("Koopman Monte Carlo MPC controller\n")
    disp(strcat('model:', model_file));
    Controller.name = "kmcmpc";
    Controller.type = "MPC_CONTROLLER_KMC_kyo";
    % Controller.param = Controller_param;
end