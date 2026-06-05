function Controller = Controller_KQ_LMPC(dt, agent)
 %% HL param
    Controller = Controller_HL(dt);
    Controller.dt_drone = Controller.dt;
%% common param
   Controller.jx= agent.parameter.jx ;
    Controller.jy= agent.parameter.jy ;
    Controller.jz= agent.parameter.jz ;
    Controller.gravity= agent.parameter.gravity ;
    Controller.m = agent.parameter.mass;
    Controller.state_size = 12;
    Controller.input_size = 4;
    Controller.total_size = Controller.state_size + Controller.input_size;
    Controller.dt = 0.025;          % MPCステップ幅
    Controller.H = 30;              %predict horizon
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
    torque_th = 0.5; thrust_th = 1.5;
    Controller.input_max = [Controller.m * 9.81 + thrust_th; torque_th; torque_th; torque_th];
    Controller.input_min = [Controller.m * 9.81 - thrust_th;-torque_th;-torque_th;-torque_th];
    Controller.ref_input = Controller.input.u; %入力の目標値ー初設定
    Controller.input.lb = [0; -1; -1; -1];
    Controller.input.ub = [10; 1;  1;  1];
    %% load model from koopman setting in the simxxx & change sampling time
   
    % load(model_file, 'est');
    % [Controller.koopman.A, Controller.koopman.B, Controller.koopman.C]  = AB_transfer(est.A, est.B, est.C, dt, Controller.dt);
    % if isfield(est, 'Ae'); [Controller.koopman.Ae,Controller.koopman.Be,Controller.koopman.Ce] = AB_transfer(est.Ae, est.Be, est.Ce, dt, Controller.dt); end
    % %-- 観測量の選択
    % [Controller.F, Controller.code] = select_observable(model_file);
   
    %% 実験用　重み

     Controller.weight.P = 0.5*diag([1000;1000;1200]);    % 位置　10,20刻み  20;1;30
    Controller.weight.Q = 0.1*diag([300;300;500]);    % 姿勢角15良い気がする
    Controller.weight.V = 0.8*diag([500;500;500]);% 速度  10,20刻み  30;20;10
    Controller.weight.W = 0.1*diag([200;200;200]);  %角速度　1,2刻み 
    Controller.weight.R = 50*diag([1; 0.2; 0.1; 0.1]); % 入力
    Controller.weight.RP =50*diag([1; 0.1; 0.1; 0.1]);  % 1ステップ前の入力との差    0*(無効化)
    %%　実験用　重み
    % Controller.weight.P = 0.7*diag([1000;1000;1000]);    % 位置　10,20刻み  20;1;30
    % Controller.weight.Q = 0.1*diag([300;300;500]);    % 姿勢角15良い気がする
    % Controller.weight.V = 1*diag([500;500;500]);% 速度  10,20刻み  30;20;10
    % Controller.weight.W = 0.1*diag([200;200;200]);  %角速度　1,2刻み 
    % Controller.weight.R = 1*diag([0.1; 0.2; 0.1; 0.1]); % 入力
    % Controller.weight.RP =1*diag([1; 0.1; 0.1; 0.1]);  % 1ステップ前の入力との差    0*(無効化)
    %%　2025-07-30_exp_koseki_code00_randompp　用重み
    % Controller.weight.P = diag([500;500;200]);    % 位置　10,20刻み  20;1;30
    % Controller.weight.Q = 1e4*diag([1;1;1]);    % 速度  10,20刻み  30;20;10
    % Controller.weight.V = diag([50;50;100]); % 15良い気がする
    % Controller.weight.W = diag([1;1;0]);  % 姿勢角，角速度　1,2刻み 
    % Controller.weight.R = diag([1; 1; 1; 1000]); % 入力
    % Controller.weight.RP = 0*diag([100; 1; 1; 1]);  % 1ステップ前の入力との差    0*(無効化)
    %%　実験　hovering
    % Controller.weight.P = 1*diag([200;200;200]);    % 位置　10,20刻み  20;1;30
    % Controller.weight.Q = 10*diag([10;10;10]);    % 速度  10,20刻み  30;20;10
    % Controller.weight.V = 1*diag([15;15;15]); % 15良い気がする
    % Controller.weight.W = 0.1*diag([5;5;0]);  % 姿勢角，角速度　1,2刻み 
    % Controller.weight.R = 0.1*diag([900; 100; 100; 3500]); % 入力
    % Controller.weight.RP = 0*diag([1; 0; 0; 0]);  % 1ステップ前の入力との差    0*(無効化)
    
    Controller.weight.Pf = Controller.weight.P;
    Controller.weight.Vf = Controller.weight.V;
    Controller.weight.Qf = Controller.weight.Q;
    Controller.weight.Wf = Controller.weight.W;
    Controller.test.sigma = 1; % 標準偏差を固定
    Controller.test.input = 0; % 推力以外の入力を0固定: 0:固定なし,1:トルク,2:自由
    %% 以下は変更なし
    fprintf("Koopman Quasilinear LPVMPC controller\n")
    Controller.name = "kqlmpc";
    Controller.type = "KQ_LMPC_CONTROLLER";
    % Controller.param = Controller_param;
end