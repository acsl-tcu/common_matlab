% generate_SuspendedLoad_RotorLevel_Dynamics.m
% =========================================================================
% SuspendedLoad_RotorLevel_Dynamics.m の自動生成スクリプト
%
% 概要:
%   with_load_model_mL_euler_for_HL.m と同じ吊り荷付きドローン状態方程式を、
%   各ロータ推力 [f1,f2,f3,f4] を入力として Symbolic Math Toolbox で導出し、
%   matlabFunction でコードを自動生成する。
%
%   クリッピング (max/min) はシンボリック処理と相性が悪いため、
%   本スクリプトは「クリッピングなし」の生コードを _raw.m として出力し、
%   後処理でクリッピング・ヘッダ修正を行う。
%
% 生成ファイル:
%   SuspendedLoad_RotorLevel_Dynamics.m  <- 本スクリプト実行後に作成
%
% 使用方法:
%   MATLAB で本スクリプトを実行 (Symbolic Math Toolbox 必須)
%
% 状態変数 (in1, 25次元): [p(3); euler(3); v(3); w(3); pL(3); vL(3); pT(3); wL(3); mL(1)]
% 入力変数 (in2,  4次元): [f1; f2; f3; f4]  各ロータ推力 [N]
% パラメータ (in3, 21次元): [mass,Lx,Ly,lx,ly,jx,jy,jz,gravity,km1..4,k1..4,rotor_r,Length,loadmass,cableL]
%
% 参考:
%   with_load_model_mL_euler_for_HL.m     (元の [T;tau] 入力の状態方程式)
%   THRUST2FORCE_TORQUE.m                  (アロケーション行列の定義)
%   generateController_SuspendedLoad.m     (生成スクリプトのパターン)
% =========================================================================

mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

disp("==============================================================");
disp(" generate_SuspendedLoad_RotorLevel_Dynamics");
disp(" 吊り荷付きドローン ロータレベル入力 状態方程式 生成");
disp("==============================================================");

%% 1. シンボリック変数の定義
% --- 状態変数 (25次元) ---
syms p1 p2 p3 real           % 機体位置
syms phi theta psi real      % 機体姿勢 (roll, pitch, yaw)
syms dp1 dp2 dp3 real        % 機体速度
syms o1 o2 o3 real           % 機体角速度 (ボディ座標)
syms pl1 pl2 pl3 real        % 荷物位置
syms vl1 vl2 vl3 real        % 荷物速度
syms pT1 pT2 pT3 real        % 紐方向単位ベクトル
syms ol1 ol2 ol3 real        % 紐角速度
syms mL real                 % 荷物質量 (推定値)

% --- 物理パラメータ (21次元, DRONE_PARAM_SUSPENDED_LOAD.m 順) ---
syms m real                  % 機体質量         in3: 1
syms Lx Ly real              % アーム全長        in3: 2,3
syms lx ly real              % ロータオフセット  in3: 4,5
syms jx jy jz real           % 慣性モーメント    in3: 6,7,8
syms gravity real            % 重力加速度        in3: 9
syms km1 km2 km3 km4 real    % 反力トルク係数    in3: 10..13
syms k1p k2p k3p k4p real    % 推力係数 (未使用) in3: 14..17
syms rotor_r_s Length_s real % 形状パラメータ    in3: 18,19
syms loadmass real           % 荷物質量初期値    in3: 20
syms cableL real             % 紐の長さ          in3: 21

% --- 入力変数 (4次元): 各ロータ推力 ---
syms f1 f2 f3 f4 real

disp("Variables defined.");

%% 2. アロケーション行列 B
% [T; tau_x; tau_y; tau_z] = B * [f1; f2; f3; f4]
% (THRUST2FORCE_TORQUE.m と同一)
B_mat = [1,    1,         1,       1;
        -ly,  -ly,     (Ly-ly), (Ly-ly);
         lx,  -(Lx-lx), lx,    -(Lx-lx);
         km1, -km2,    -km3,    km4];

u_body = B_mat * [f1; f2; f3; f4];
T  = u_body(1);   % 合力推力 [N]
tx = u_body(2);   % roll  トルク [N*m]
ty = u_body(3);   % pitch トルク [N*m]
tz = u_body(4);   % yaw   トルク [N*m]

disp("Allocation matrix B defined.");

%% 3. 回転行列 R (Z-Y-X オイラー角, ボディ->ワールド)
Rx_mat = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
Ry_mat = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
Rz_mat = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
R = Rz_mat * Ry_mat * Rx_mat;
e3 = [0; 0; 1];

disp("Rotation matrix defined.");

%% 4. 状態方程式の導出
% --- 幾何ベクトル ---
pT = [pT1; pT2; pT3];
ol = [ol1; ol2; ol3];
ob = [o1; o2; o3];

% --- 紐の運動 ---
dpT_vec   = cross(ol, pT);            % 紐方向の時間微分
FpT       = (pT.' * (T * (R * e3)));  % 推力の紐方向射影
wL_sq     = dpT_vec.' * dpT_vec;      % 紐の角速度の二乗
m_total   = m + mL;

% --- 荷物加速度 (ワールド座標) ---
a_load = -gravity * e3 + ((FpT - m * cableL * wL_sq) / m_total) * pT;

% --- 紐角速度の時間微分 ---
a_ol = cross(pT, -(T * (R * e3))) / (m * cableL);

% --- オイラー角レート ---
W_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
d_eul = W_inv * ob;

% --- 機体角速度ダイナミクス ---
J_mat = diag([jx, jy, jz]);
d_ob  = J_mat \ ([tx; ty; tz] - cross(ob, J_mat * ob));

% --- 機体速度ダイナミクス ---
% F_thrust - 重力 - 紐張力(荷物重量+荷物加速度) / 機体質量
d_dp = (T * (R * e3) - m * gravity * e3 - mL * (a_load + gravity * e3)) / m;

%% 5. 状態微分ベクトルの組み立て
% in1 の並び: [p(3); euler(3); dp(3); ob(3); pl(3); vl(3); pT(3); ol(3); mL(1)]
%  = [p1;p2;p3; phi;theta;psi; dp1;dp2;dp3; o1;o2;o3;
%     pl1;pl2;pl3; vl1;vl2;vl3; pT1;pT2;pT3; ol1;ol2;ol3; mL]

dx_full = [dp1;dp2;dp3;         ...  % d(p)      = v (= dp1..dp3 は速度そのもの)
           d_eul;               ...  % d(euler)
           d_dp;                ...  % d(dp)     = 機体加速度
           d_ob;                ...  % d(ob)     = 角加速度
           vl1;vl2;vl3;         ...  % d(pl)     = vl (荷物速度)
           a_load;              ...  % d(vl)     = 荷物加速度
           dpT_vec;             ...  % d(pT)     = cross(ol,pT)
           a_ol;                ...  % d(ol)     = 紐角速度の時間微分
           sym(0)];                  % d(mL)     = 0 (定数ダイナミクス)

disp("State equations assembled.");
disp("Simplifying... (数分かかります)");
dx_simplified = simplify(dx_full, 'Steps', 15);
disp("Simplification complete.");

%% 6. 入力・パラメータベクトルの定義 (matlabFunction 用)
in2_sym = [f1; f2; f3; f4];
in1_sym = [p1;p2;p3; phi;theta;psi; dp1;dp2;dp3; o1;o2;o3;
           pl1;pl2;pl3; vl1;vl2;vl3; pT1;pT2;pT3; ol1;ol2;ol3; mL];
in3_sym = [m; Lx; Ly; lx; ly; jx; jy; jz; gravity;
           km1; km2; km3; km4; k1p; k2p; k3p; k4p;
           rotor_r_s; Length_s; loadmass; cableL];

%% 7. matlabFunction でコード生成 (raw 版: クリッピングなし)
out_filename = 'SuspendedLoad_RotorLevel_Dynamics_raw.m';
disp("Exporting to " + out_filename + " ...");
matlabFunction(dx_simplified, ...
    'file',    out_filename, ...
    'vars',    {in1_sym, in2_sym, in3_sym}, ...
    'outputs', {'dx'}, ...
    'Optimize', true);
disp(out_filename + " generated!");

%% 8. ヘッダ・クリッピング処理の追加 (post-processing)
disp("Adding header and clipping to final file...");

raw_lines = readlines(out_filename);
header = [
    "function dx = SuspendedLoad_RotorLevel_Dynamics(in1, in2, in3)"
    "%SuspendedLoad_RotorLevel_Dynamics  吊り荷付きドローン ロータレベル状態方程式"
    "%    DX = SuspendedLoad_RotorLevel_Dynamics(IN1, IN2, IN3)"
    "%"
    "% with_load_model_mL_euler_for_HL.m と同じ構造だが、入力が各ロータ推力となる。"
    "% 内部でアロケーション行列 B により [T;tau] を計算し、"
    "% 飽和クリッピング後の実効 [T_eff;tau_eff] を状態方程式に適用する。"
    "%"
    "% Input order:"
    "%   in1 (state)     = [""p1"",""p2"",""p3"",""roll"",""pitch"",""yaw"","
    "%                       ""dp1"",""dp2"",""dp3"",""o1"",""o2"",""o3"","
    "%                       ""pl1"",""pl2"",""pl3"",""vl1"",""vl2"",""vl3"","
    "%                       ""pT1"",""pT2"",""pT3"",""ol1"",""ol2"",""ol3"",""mL""];"
    "%   in2 (input)     = [""f1"", ""f2"", ""f3"", ""f4""];  % 各ロータ推力 [N]"
    "%   in3 (parameter) = [""mass"", ""Lx"", ""Ly"", ""lx"", ""ly"", ""jx"", ""jy"", ""jz"", ""gravity"", ""km1"", ""km2"", ""km3"", ""km4"", ""k1"", ""k2"", ""k3"", ""k4"", ""rotor_r"", ""Length"", ""loadmass"", ""cableL""];"
    "%"
    "% 2026/09/11  自動生成 by generate_SuspendedLoad_RotorLevel_Dynamics.m"
    ""
    "% --- ロータ推力クリッピング (非負 / 最大 5.0 N) ---"
    "f1 = max(0, min(5.0, in2(1,:)));"
    "f2 = max(0, min(5.0, in2(2,:)));"
    "f3 = max(0, min(5.0, in2(3,:)));"
    "f4 = max(0, min(5.0, in2(4,:)));"
    ""
];

% raw ファイルの関数定義行を除去してボディ部分だけ抽出
body_start = 1;
for i = 1:length(raw_lines)
    if startsWith(strtrim(raw_lines(i)), "function ")
        body_start = i + 1;
        break;
    end
end
body_lines = raw_lines(body_start:end);

final_lines = [header; body_lines];
writelines(final_lines, 'SuspendedLoad_RotorLevel_Dynamics.m');

% raw ファイルを削除
delete(out_filename);

disp(" ");
disp("==============================================================");
disp(" SuspendedLoad_RotorLevel_Dynamics.m generated successfully!");
disp("==============================================================");
disp(" ");
disp("注意: 生成された関数内の変数名 (f1,f2,f3,f4) が");
disp("クリッピング後の値を参照しているか確認してください。");
disp("matlabFunction の出力によっては変数名が自動変更される場合があります。");
