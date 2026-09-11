% generate_SuspendedLoad_RotorLevel_Dynamics.m
% =========================================================================
% SuspendedLoad_RotorLevel_Dynamics.m の自動生成スクリプト
%
% 概要:
%   with_load_model_mL_euler_for_HL.m と同じ吊り荷付きドローン状態方程式を、
%   Symbolic Math Toolbox で導出し、matlabFunction でコードを自動生成する。
%
%   コントローラからの入力は in2 = [T; tau_x; tau_y; tau_z] であるため、
%   生成される関数の内部で以下の中間処理を行うように後処理を付加する：
%     1. [T; tau] をロータ推力空間 [f1; f2; f3; f4] へ変換
%     2. ロータ推力を 0 ~ 5.0 N で飽和クリッピング
%     3. クリッピング後のロータ推力から実効 [T_eff; tau_eff] を再計算
%     4. シンボリック導出された状態方程式に実効入力を適用
%
%   これにより、EKF や コントローラ との入力次元の互換性を保ちながら、
%   プラント内部での正確なロータ飽和シミュレーションを実現し、
%   今後の CBF 設計などのベースとなるシンボリックモデルを提供します。
% =========================================================================

mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

disp("==============================================================");
disp(" generate_SuspendedLoad_RotorLevel_Dynamics");
disp(" 吊り荷付きドローン状態方程式 (ロータ飽和考慮版) 生成スクリプト");
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

% --- 実効入力変数 (4次元): [T_eff; tau_x_eff; tau_y_eff; tau_z_eff] ---
syms u1 u2 u3 u4 real
T  = u1;
tx = u2;
ty = u3;
tz = u4;

disp("Variables defined.");

%% 2. 回転行列 R (Z-Y-X オイラー角, ボディ->ワールド)
Rx_mat = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
Ry_mat = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
Rz_mat = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
R = Rz_mat * Ry_mat * Rx_mat;
e3 = [0; 0; 1];

disp("Rotation matrix defined.");

%% 3. 状態方程式の導出
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
d_dp = (T * (R * e3) - m * gravity * e3 - mL * (a_load + gravity * e3)) / m;

%% 4. 状態微分ベクトルの組み立て
dx_full = [dp1;dp2;dp3;         ...  % d(p)
           d_eul;               ...  % d(euler)
           d_dp;                ...  % d(dp)
           d_ob;                ...  % d(ob)
           vl1;vl2;vl3;         ...  % d(pl)
           a_load;              ...  % d(vl)
           dpT_vec;             ...  % d(pT)
           a_ol;                ...  % d(ol)
           sym(0)];                  % d(mL) = 0

disp("State equations assembled.");
disp("Simplifying... (数分かかります)");
dx_simplified = simplify(dx_full, 'Steps', 15);
disp("Simplification complete.");

%% 5. 入力・パラメータベクトルの定義 (matlabFunction 用)
u_eff_sym = [u1; u2; u3; u4];
in1_sym = [p1;p2;p3; phi;theta;psi; dp1;dp2;dp3; o1;o2;o3;
           pl1;pl2;pl3; vl1;vl2;vl3; pT1;pT2;pT3; ol1;ol2;ol3; mL];
in3_sym = [m, Lx, Ly, lx, ly, jx, jy, jz, gravity, ...
           km1, km2, km3, km4, k1p, k2p, k3p, k4p, ...
           rotor_r_s, Length_s, loadmass, cableL];

%% 6. matlabFunction でコード生成 (raw 版: u_eff 入力)
out_filename = 'SuspendedLoad_RotorLevel_Dynamics_raw.m';
disp("Exporting to " + out_filename + " ...");
matlabFunction(dx_simplified, ...
    'file',    out_filename, ...
    'vars',    {in1_sym, u_eff_sym, in3_sym}, ...
    'outputs', {'dx'}, ...
    'Optimize', true);
disp(out_filename + " generated!");

%% 7. ヘッダ・クリッピング処理の追加 (post-processing)
disp("Adding wrapper for rotor saturation...");

raw_lines = readlines(out_filename);

header = [
    "function dx = SuspendedLoad_RotorLevel_Dynamics(in1, in2, in3)"
    "%SuspendedLoad_RotorLevel_Dynamics  吊り荷付きドローンのロータレベル状態方程式"
    "%    DX = SuspendedLoad_RotorLevel_Dynamics(IN1, IN2, IN3)"
    "%"
    "% この関数は generate_SuspendedLoad_RotorLevel_Dynamics.m によって自動生成されました。"
    "% コントローラからの入力 [T; tau] を受け取り、内部でロータ推力に変換・クリッピングし、"
    "% 実効 [T_eff; tau_eff] を用いて状態方程式を計算します。"
    "%"
    "% Input order:"
    "%   in1 (state)     = 25 dimensional state vector"
    "%   in2 (input)     = [T; tau_x; tau_y; tau_z]"
    "%   in3 (parameter) = [mass, Lx, Ly, lx, ly, jx, jy, jz, gravity, km1, km2, km3, km4, ...] (21 dims)"
    ""
    "% -----------------------------------------------------------------------"
    "% [T; tau] -> ロータ推力変換 -> クリップ -> 実効 [T_eff; tau_eff]"
    "% -----------------------------------------------------------------------"
    "T_cmd    = in2(1,:);"
    "tau_x    = in2(2,:);"
    "tau_y    = in2(3,:);"
    "tau_z    = in2(4,:);"
    ""
    "Lx_s  = in3(:,2);"
    "Ly_s  = in3(:,3);"
    "lx_s  = in3(:,4);"
    "ly_s  = in3(:,5);"
    "km1_s = in3(:,10);"
    "km2_s = in3(:,11);"
    "km3_s = in3(:,12);"
    "km4_s = in3(:,13);"
    ""
    "B_mat = [1,    1,         1,       1;"
    "        -ly_s, -ly_s,    (Ly_s-ly_s), (Ly_s-ly_s);"
    "         lx_s, -(Lx_s-lx_s), lx_s, -(Lx_s-lx_s);"
    "         km1_s, -km2_s,  -km3_s,  km4_s];"
    ""
    "IIT = inv(B_mat);"
    "f_raw = IIT * [T_cmd; tau_x; tau_y; tau_z];"
    ""
    "% 各ロータの最大推力 [N] (非負・最大5.0)"
    "f_clipped = max(0, min(5.0, f_raw));"
    ""
    "% 実効入力を再計算"
    "u_eff = B_mat * f_clipped;"
    "u1 = u_eff(1,:);"
    "u2 = u_eff(2,:);"
    "u3 = u_eff(3,:);"
    "u4 = u_eff(4,:);"
    ""
    "% -----------------------------------------------------------------------"
    "% シンボリック導出された状態方程式 (u1, u2, u3, u4 を使用)"
    "% -----------------------------------------------------------------------"
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

% body_lines の中で u_eff_sym1.. みたいな変数が使われていたら u1.. に直す
% matlabFunction は in2_sym の要素を in21, in22 と名付けることがあるため。
% 今回は vars で {in1_sym, u_eff_sym, in3_sym} を渡している。
% u_eff_sym をそのまま u1, u2, u3, u4 として使わせるための置換
body_text = join(body_lines, newline);
body_text = regexprep(body_text, 'u_eff_sym1', 'u1');
body_text = regexprep(body_text, 'u_eff_sym2', 'u2');
body_text = regexprep(body_text, 'u_eff_sym3', 'u3');
body_text = regexprep(body_text, 'u_eff_sym4', 'u4');

% 更に、matlabFunction は in2(1,:), in2(2,:) という書き方をする場合がある
body_text = regexprep(body_text, 'in2\(1,\s*:\)', 'u1');
body_text = regexprep(body_text, 'in2\(2,\s*:\)', 'u2');
body_text = regexprep(body_text, 'in2\(3,\s*:\)', 'u3');
body_text = regexprep(body_text, 'in2\(4,\s*:\)', 'u4');

final_text = join(header, newline) + newline + body_text;
writelines(final_text, 'SuspendedLoad_RotorLevel_Dynamics.m');

% raw ファイルを削除
delete(out_filename);

disp(" ");
disp("==============================================================");
disp(" SuspendedLoad_RotorLevel_Dynamics.m generated successfully!");
disp("==============================================================");
