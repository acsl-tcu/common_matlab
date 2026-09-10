function Controller = Controller_HL_Hinf_Suspended_Load(dt, agent)  %exmple
% H∞ 状態フィードバック制御器 設計（単機牽引ドローン用）

A2 = diag(1, 1);
B2 = [0; 1];
C2 = eye(2);
A6 = diag([1,1,1,1,1], 1);
B6 = [0;0;0;0;0;1];
C6 = eye(6);

Controller.P = agent.parameter.get();

s = tf('s');

%% プラント定義
P2_z = ss(A2, B2, C2, 0);
P2_z.InputName  = {'u_z'};
P2_z.OutputName = {'pz'; 'vz'};

P6_x = ss(A6, B6, C6, 0);
P6_x.InputName  = {'u_x'};
P6_x.OutputName = {'pLx';'vLx';'px';'vx';'axd';'jx'};

P6_y = ss(A6, B6, C6, 0);
P6_y.InputName  = {'u_y'};
P6_y.OutputName = {'pLy';'vLy';'py';'vy';'ay';'jy'};

P2_yaw = ss(A2, B2, C2, 0);
P2_yaw.InputName  = {'u_yaw'};
P2_yaw.OutputName = {'pyaw'; 'vyaw'};

if class(agent.plant) ~= "DRONE_EXP_MODEL"
    % sim用重み
    % 目標周波数応答（帯域幅で特性を指定）
    PosTarget_z   = tf(1, [1/2.0 1]);
    VelTarget_z   = tf(1, [1/2.0 1]);
    PosTarget_x   = tf(1, [1/0.5 1]);
    VelTarget_x   = tf(1, [1/0.5 1]);
    PosTarget_yaw = tf(1, [1/1.0 1]);
    VelTarget_yaw = tf(1, [1/1.0 1]);

    % 外乱重み
    Wdst_z   = ss(1.0); Wdst_z.u   = 'd_z';   Wdst_z.y   = 'r_z';
    Wdst_x   = ss(1.0); Wdst_x.u   = 'd_x';   Wdst_x.y   = 'r_x';
    Wdst_y   = ss(1.0); Wdst_y.u   = 'd_y';   Wdst_y.y   = 'r_y';
    Wdst_yaw = ss(1.0); Wdst_yaw.u = 'd_yaw'; Wdst_yaw.y = 'r_yaw';

    % 制御入力重み
    Wact_z   = 0.8*tf([1 2],  [1 20]);  Wact_z.u   = 'u_z';   Wact_z.y   = 'e1_z';
    Wact_x   = 0.8*tf([1 0.5],[1 5]);   Wact_x.u   = 'u_x';   Wact_x.y   = 'e1_x';
    Wact_y   = 0.8*tf([1 0.5],[1 5]);   Wact_y.u   = 'u_y';   Wact_y.y   = 'e1_y';
    Wact_yaw = 0.8*tf([1 1],  [1 10]);  Wact_yaw.u = 'u_yaw'; Wact_yaw.y = 'e1_yaw';

    % 位置追従重み（目標応答の逆数）
    Wpos_z   = 1/PosTarget_z;   Wpos_z.u   = 'pz';   Wpos_z.y   = 'e2_z';
    Wpos_x   = 1/PosTarget_x;   Wpos_x.u   = 'px';   Wpos_x.y   = 'e2_x';
    Wpos_y   = 1/PosTarget_x;   Wpos_y.u   = 'py';   Wpos_y.y   = 'e2_y';
    Wpos_yaw = 1/PosTarget_yaw; Wpos_yaw.u = 'pyaw'; Wpos_yaw.y = 'e2_yaw';

    % 速度追従重み
    Wvel_z   = 1/VelTarget_z;   Wvel_z.u   = 'vz';   Wvel_z.y   = 'e3_z';
    Wvel_x   = 1/VelTarget_x;   Wvel_x.u   = 'vx';   Wvel_x.y   = 'e3_x';
    Wvel_y   = 1/VelTarget_x;   Wvel_y.u   = 'vy';   Wvel_y.y   = 'e3_y';
    Wvel_yaw = 1/VelTarget_yaw; Wvel_yaw.u = 'vyaw'; Wvel_yaw.y = 'e3_yaw';

    % 測定ノイズ重み
    Wnoise_z   = ss(0.01); Wnoise_z.u   = 'n_z';   Wnoise_z.y   = 'Wn_z';
    Wnoise_x   = ss(0.01); Wnoise_x.u   = 'n_x';   Wnoise_x.y   = 'Wn_x';
    Wnoise_y   = ss(0.01); Wnoise_y.u   = 'n_y';   Wnoise_y.y   = 'Wn_y';
    Wnoise_yaw = ss(0.01); Wnoise_yaw.u = 'n_yaw'; Wnoise_yaw.y = 'Wn_yaw';

else
    % exp用重み（保守的）
    PosTarget_z   = tf(1, [1/1.0 1]);
    VelTarget_z   = tf(1, [1/1.0 1]);
    PosTarget_x   = tf(1, [1/0.3 1]);
    VelTarget_x   = tf(1, [1/0.3 1]);
    PosTarget_yaw = tf(1, [1/0.5 1]);
    VelTarget_yaw = tf(1, [1/0.5 1]);

    Wdst_z   = ss(1.0); Wdst_z.u   = 'd_z';   Wdst_z.y   = 'r_z';
    Wdst_x   = ss(1.0); Wdst_x.u   = 'd_x';   Wdst_x.y   = 'r_x';
    Wdst_y   = ss(1.0); Wdst_y.u   = 'd_y';   Wdst_y.y   = 'r_y';
    Wdst_yaw = ss(1.0); Wdst_yaw.u = 'd_yaw'; Wdst_yaw.y = 'r_yaw';

    Wact_z   = 0.8*tf([1 1],  [1 10]);  Wact_z.u   = 'u_z';   Wact_z.y   = 'e1_z';
    Wact_x   = 0.8*tf([1 0.3],[1 3]);   Wact_x.u   = 'u_x';   Wact_x.y   = 'e1_x';
    Wact_y   = 0.8*tf([1 0.3],[1 3]);   Wact_y.u   = 'u_y';   Wact_y.y   = 'e1_y';
    Wact_yaw = 0.8*tf([1 0.5],[1 5]);   Wact_yaw.u = 'u_yaw'; Wact_yaw.y = 'e1_yaw';

    Wpos_z   = 1/PosTarget_z;   Wpos_z.u   = 'pz';   Wpos_z.y   = 'e2_z';
    Wpos_x   = 1/PosTarget_x;   Wpos_x.u   = 'px';   Wpos_x.y   = 'e2_x';
    Wpos_y   = 1/PosTarget_x;   Wpos_y.u   = 'py';   Wpos_y.y   = 'e2_y';
    Wpos_yaw = 1/PosTarget_yaw; Wpos_yaw.u = 'pyaw'; Wpos_yaw.y = 'e2_yaw';

    Wvel_z   = 1/VelTarget_z;   Wvel_z.u   = 'vz';   Wvel_z.y   = 'e3_z';
    Wvel_x   = 1/VelTarget_x;   Wvel_x.u   = 'vx';   Wvel_x.y   = 'e3_x';
    Wvel_y   = 1/VelTarget_x;   Wvel_y.u   = 'vy';   Wvel_y.y   = 'e3_y';
    Wvel_yaw = 1/VelTarget_yaw; Wvel_yaw.u = 'vyaw'; Wvel_yaw.y = 'e3_yaw';

    Wnoise_z   = ss(0.01); Wnoise_z.u   = 'n_z';   Wnoise_z.y   = 'Wn_z';
    Wnoise_x   = ss(0.01); Wnoise_x.u   = 'n_x';   Wnoise_x.y   = 'Wn_x';
    Wnoise_y   = ss(0.01); Wnoise_y.u   = 'n_y';   Wnoise_y.y   = 'Wn_y';
    Wnoise_yaw = ss(0.01); Wnoise_yaw.u = 'n_yaw'; Wnoise_yaw.y = 'Wn_yaw';
end

%% 測定ブロック
meas_pz   = sumblk('y1_z   = pz   + Wn_z');
meas_vz   = sumblk('y2_z   = vz   + Wn_z');
meas_px   = sumblk('y1_x   = px   + Wn_x');
meas_vx   = sumblk('y2_x   = vx   + Wn_x');
meas_px_d = sumblk('y3_x   = px_d + Wn_x');
meas_vx_d = sumblk('y4_x   = vx_d + Wn_x');
meas_ax_d = sumblk('y5_x   = ax_d + Wn_x');
meas_jx_d = sumblk('y6_x   = jx_d + Wn_x');
meas_py   = sumblk('y1_y   = py   + Wn_y');
meas_vy   = sumblk('y2_y   = vy   + Wn_y');
meas_py_d = sumblk('y3_y   = py_d + Wn_y');
meas_vy_d = sumblk('y4_y   = vy_d + Wn_y');
meas_ay_d = sumblk('y5_y   = ay_d + Wn_y');
meas_jy_d = sumblk('y6_y   = jy_d + Wn_y');
meas_pyaw = sumblk('y1_yaw = pyaw + Wn_yaw');
meas_vyaw = sumblk('y2_yaw = vyaw + Wn_yaw');

%% ブロック線図接続
% z軸
IC_z = connect(P2_z, Wdst_z, Wact_z, Wpos_z, Wvel_z, Wnoise_z, ...
    meas_pz, meas_vz, ...
    {'d_z';'n_z';'u_z'}, ...
    {'e1_z';'e2_z';'e3_z';'y1_z';'y2_z'});

% x軸
IC_x = connect(P6_x, Wdst_x, Wact_x, Wpos_x, Wvel_x, Wnoise_x, ...
    meas_px, meas_vx, meas_px_d, meas_vx_d, meas_ax_d, meas_jx_d, ...
    {'d_x';'n_x';'u_x'}, ...
    {'e1_x';'e2_x';'e3_x'; ...
     'y1_x';'y2_x';'y3_x';'y4_x';'y5_x';'y6_x'});

% y軸
IC_y = connect(P6_y, Wdst_y, Wact_y, Wpos_y, Wvel_y, Wnoise_y, ...
    meas_py, meas_vy, meas_py_d, meas_vy_d, meas_ay_d, meas_jy_d, ...
    {'d_y';'n_y';'u_y'}, ...
    {'e1_y';'e2_y';'e3_y'; ...
     'y1_y';'y2_y';'y3_y';'y4_y';'y5_y';'y6_y'});

% yaw軸
IC_yaw = connect(P2_yaw, Wdst_yaw, Wact_yaw, Wpos_yaw, Wvel_yaw, Wnoise_yaw, ...
    meas_pyaw, meas_vyaw, ...
    {'d_yaw';'n_yaw';'u_yaw'}, ...
    {'e1_yaw';'e2_yaw';'e3_yaw';'y1_yaw';'y2_yaw'});

% サイズ確認
[ny_z,  nu_z]  = size(IC_z);
[ny_x,  nu_x]  = size(IC_x);
[ny_yaw,nu_yaw] = size(IC_yaw);
fprintf('IC_z:   出力=%d, 入力=%d\n', ny_z,   nu_z);
fprintf('IC_x:   出力=%d, 入力=%d\n', ny_x,   nu_x);
fprintf('IC_yaw: 出力=%d, 入力=%d\n', ny_yaw, nu_yaw);

%% hinfsyn
nmeas_2 = 2;
nmeas_6 = 6;
ncon    = 1;

[K1, ~, gamma1] = hinfsyn(IC_z,   nmeas_2, ncon);
fprintf('z   gamma = %.4f\n', gamma1);

[K2, ~, gamma2] = hinfsyn(IC_x,   nmeas_6, ncon);
fprintf('x   gamma = %.4f\n', gamma2);

[K3, ~, gamma3] = hinfsyn(IC_y,   nmeas_6, ncon);
fprintf('y   gamma = %.4f\n', gamma3);

[K4, ~, gamma4] = hinfsyn(IC_yaw, nmeas_2, ncon);
fprintf('yaw gamma = %.4f\n', gamma4);

%% 閉ループ極から静的ゲイン抽出
[AK1,BK1,CK1,DK1] = ssdata(ss(K1));
[AK2,BK2,CK2,DK2] = ssdata(ss(K2));
[AK3,BK3,CK3,DK3] = ssdata(ss(K3));
[AK4,BK4,CK4,DK4] = ssdata(ss(K4));

A_cl_z   = [A2+B2*DK1*C2, B2*CK1; BK1*C2, AK1];
A_cl_x   = [A6+B6*DK2*C6, B6*CK2; BK2*C6, AK2];
A_cl_yaw = [A2+B2*DK4*C2, B2*CK4; BK4*C2, AK4];

poles_z   = sort(eig(A_cl_z),   'ComparisonMethod', 'real');
poles_x   = sort(eig(A_cl_x),   'ComparisonMethod', 'real');
poles_yaw = sort(eig(A_cl_yaw), 'ComparisonMethod', 'real');

fprintf('z  閉ループ極: '); disp(poles_z.');
fprintf('x  閉ループ極: '); disp(poles_x.');
fprintf('yaw閉ループ極: '); disp(poles_yaw.');

%% placeで静的ゲイン設計
try
    Controller.F1 = place(A2, B2, poles_z(1:2));
catch
    error('z: place失敗 閉ループ極を確認してください');
end

try
    Controller.F2 = place(A6, B6, poles_x(1:6));
    Controller.F3 = Controller.F2;
catch
    error('x/y: place失敗 閉ループ極を確認してください');
end

try
    Controller.F4 = place(A2, B2, poles_yaw(1:2));
catch
    error('yaw: place失敗 閉ループ極を確認してください');
end

%% 確認
fprintf('z  固有値: ');   disp(eig(A2-B2*Controller.F1).');
fprintf('x  固有値: ');   disp(eig(A6-B6*Controller.F2).');
fprintf('yaw固有値: ');   disp(eig(A2-B2*Controller.F4).');

disp('F1 ='); disp(Controller.F1);
disp('F2 ='); disp(Controller.F2);
disp('F3 ='); disp(Controller.F3);
disp('F4 ='); disp(Controller.F4);

Controller.dt = dt;
eig(A6 - B6*Controller.F2)
end