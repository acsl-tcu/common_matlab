function Controller = Controller_HL_MPC(dt, agent)
%Koopman-MCMPC parameter setting
    Controller.input.Maxinput = 1.5;
    Controller.input.Constinput = 10;
    Controller.input.range = [[10;30;30;10], [0.1;0.1;0.1;0.1]]; % max min
    Controller.input.Bestcost_now = [1e5, 1e3];
    Controller.input.Constsigma = 5.0*[1;1;1;1];
    Controller.dt = 0.025; % MPCステップ幅
    Controller.H = 30;
    %% common param
    Controller.m = agent.parameter.mass;
    Controller.state_size = 12;
    Controller.input_size = 4;
    Controller.total_size = Controller.state_size + Controller.input_size;
    Ac2 = [0,1;0,0];
    Bc2 = [0;1];
    Ac4 = diag([1,1,1],1);
    Bc4 = [0;0;0;1];
   

    syms sz1 [2 1] real
    syms sF1 [1 2] real
    [Ad1,Bd1,~,~] = ssdata(c2d(ss(Ac2,Bc2,[1,0],[0]),dt));
    Controller.Vf = matlabFunction([-sF1*sz1, -sF1*(Ad1-Bd1*sF1)*sz1, -sF1*(Ad1-Bd1*sF1)^2*sz1, -sF1*(Ad1-Bd1*sF1)^3*sz1],"Vars",{sz1,sF1});

    syms sz2 [4 1] real
    syms sF2 [1 4] real
    syms sz3 [4 1] real
    syms sF3 [1 4] real
    syms sz4 [2 1] real
    syms sF4 [1 2] real
    Controller.Vs = matlabFunction([-sF2*sz2;-sF3*sz3;-sF4*sz4],"Vars",{sz2,sz3,sz4,sF2,sF3,sF4});
    [Ad2int, Bd2int, ~, ~] = ssdata(c2d(ss(Ac2, Bc2, eye(2), zeros(2,1)), Controller.dt));
    [Ad4int, Bd4int, ~, ~] = ssdata(c2d(ss(Ac4, Bc4, eye(4), zeros(4,1)), Controller.dt));

    Controller.mpc.N = Controller.H;

    Controller.mpc.A = {Ad2int, Ad4int, Ad4int, Ad2int};
    Controller.mpc.B = {Bd2int, Bd4int, Bd4int, Bd2int};
    %% sim
    % Controller.mpc.Q = {
    %     diag([100,1]), ...%z
    %     diag([200,50,10,1]), ...%x
    %     diag([200,50,10,1]), ...%y
    %     diag([100,10])   %yaw
    %     };
    % Controller.mpc.R = {0.01, 0.015, 0.015, 0.01};%z,x,y,yaw  
    %%  exp
    Controller.mpc.Q = {
        diag([100,1]), ...%z
        diag([300,60,20,1]), ...%x
        diag([350,60,20,1]), ...%y
        diag([100,10])   %yaw
        };
    Controller.mpc.R = {0.03, 0.03, 0.03, 0.01};%z,x,y,yaw  
    %%
    Controller.mpc.lb = {-10, -10, -10, -10};
    Controller.mpc.ub = { 10,  10,  10,  10};

Controller.mpc.Nvf = 4;
    %  Controller.weight.P = diag([50;50;20]);    % 位置　10,20刻み  20;1;30
    % Controller.weight.Q = 1e4*diag([1;1;1]);    % 速度  10,20刻み  30;20;10
    % Controller.weight.V = diag([50;50;100]); % 15良い気がする
    % Controller.weight.W = diag([1;1;1]);  % 姿勢角，角速度　1,2刻み 
    % Controller.weight.R = diag([1; 1; 1; 1000]); % 入力
    % Controller.weight.RP = 0*diag([100; 1; 1; 1]);  % 1ステップ前の入力との差    0*(無効化)
    % Controller.weight.Pf = Controller.weight.P;
    % Controller.weight.Vf = Controller.weight.V;
    % Controller.weight.Qf = Controller.weight.Q;
    % Controller.weight.Wf = Controller.weight.W;

    Controller.input.Initsigma = [0.5;1e-3;1e-3;1e-3]; % default 0.1
    Controller.input.Maxsigma = [1;1e-3;1e-3;1e-3];
    Controller.input.Minsigma = [0.01;1e-5;1e-5;1e-5];
    %% input
    Controller.input.u = [Controller.m * 9.81;0;0;0]; % 総推力，トルク
    Controller.ref_input = Controller.input.u; %入力の目標値
    Controller.input.lb = [0; -1; -1; -1];
    Controller.input.ub = [10; 1;  1;  1];
    for s = 1:4
    A = Controller.mpc.A{s};
    B = Controller.mpc.B{s};

    if s == 1
        N = Controller.mpc.Nvf;
    else
        N = Controller.mpc.N;
    end

    Q = Controller.mpc.Q{s};
    R = Controller.mpc.R{s};

    nx = size(A,1);
    nu = size(B,2);

    Sx = zeros(nx*N,nx);
    Su = zeros(nx*N,nu*N);

    for i = 1:N
        Sx((i-1)*nx+1:i*nx,:) = A^i;
        for j = 1:i
            Su((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu) = A^(i-j)*B;
        end
    end

    Qb = kron(eye(N), Q);
    Rb = kron(eye(N), R);

  Controller.mpc.Sx{s} = Sx;
Controller.mpc.Su{s} = Su;
Controller.mpc.Qb{s} = Qb;

Hq = 2 * (Su' * Qb * Su + Rb);
Controller.mpc.Hq{s} = (Hq + Hq') / 2;

Controller.mpc.lbq{s} = repmat(Controller.mpc.lb{s}, N, 1);
Controller.mpc.ubq{s} = repmat(Controller.mpc.ub{s}, N, 1);
end

Controller.mpc.opt = optimoptions('quadprog', 'Display', 'off');
    %% 以下は変更なし
    fprintf("HL_MPC controller\n")
    Controller.name = "HL_MPC";
    Controller.type = "CONTROLLER_HL_MPC";
    % Controller.param = Controller_param;


end