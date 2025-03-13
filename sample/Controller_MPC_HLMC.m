function Controller = Controller_MPC_HLMC(dt)
%% HLを制御モデルとしたMCMPC %%
    %-- HL
    Controller = Controller_HL(dt);

    %-- MPC
    Controller.dt = 0.1; % MPCステップ幅
    Controller.H = 10;
    Controller.particle_num = 1000;

    % Controller.constParticle_num = 100000;
    Controller.input.sigma = 1*[0.1,1,1,1];
    Controller.input.Maxsigma = 10 * [0.1,1,1,1]; % 10 0.3452
    Controller.input.Minsigma = 0.1 * [0.1,1,1,1];
    Controller.input.range = [[10;20;20;1], 1e-1*[0.1;1;1;0.1]]; % max min
    Controller.input.input_TH = Controller.input.range(:,1); % 初期値の設定
    Controller.input.Bestcost_now = [1e1, 1e-5, 1e-5, 1e-5, 1e-5];

    Controller.total_size = 16;
    Controller.state_size = 12;
    Controller.input_size = 4;

    Controller.input.lb = [0; -1; -1; -1];
    Controller.input.ub = [10; 1;  1;  1];

    %% 赤沼
    Controller.Z = 1e2 * diag([100; 1]);
    Controller.X = diag([1000;1000;1;1]);
    Controller.Y = Controller.X;
    Controller.PHI = diag([10; 1]);

    Controller.Zf = diag([1; 1]);
    Controller.Xf = Controller.X; 
    Controller.Yf = Controller.X;
    Controller.PHIf = Controller.PHI;

    Controller.AP = 1e3;

    Controller.R = diag([1.0; 1*[1.0; 1.0; 1.0]]); 
    Controller.RP = 1e-1 * diag([1.0; 1*[1.0; 1.0; 1.0]]); 
    %
    Controller.input.u = [0;0;0;0];
    Controller.ref_input = [0;0;0;0];

    %%  
    disp('MCMPC using HL model')
    % 
    Controller.name = "mcmpc"; % HLでもMCだから
    Controller.type = "MPC_CONTROLLER_HLMC_akanuma"; % file
    % Controller.param = Controller;
end