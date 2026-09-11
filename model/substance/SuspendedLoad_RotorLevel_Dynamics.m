function dx = SuspendedLoad_RotorLevel_Dynamics(in1, in2, in3)
%SuspendedLoad_RotorLevel_Dynamics  吊り荷付きドローンのロータレベル状態方程式
%    DX = SuspendedLoad_RotorLevel_Dynamics(IN1, IN2, IN3)
%
% with_load_model_mL_euler_for_HL.m と同じ構造だが、
% ロータ推力への変換と飽和クリッピングをプラント内部で行う。
% これにより EKF は [T;tau] のまま使え、プラントは飽和を正しく表現できる。
%
% Input order:
%   in1 (state)     = ["", "", "", "roll", "pitch", "yaw", "dp1", "dp2", "dp3",
%                       "o1", "o2", "o3", "", "", "", "dpl1", "dpl2", "dpl3",
%                       "pT1", "pT2", "pT3", "ol1", "ol2", "ol3", "mL"];
%   in2 (input)     = ["u1", "u2", "u3", "u4"];  % [T; tau_x; tau_y; tau_z] (元と同じ)
%   in3 (parameter) = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4", "rotor_r", "Length", "loadmass", "cableL"];
%
% Parameter index (DRONE_PARAM_SUSPENDED_LOAD.m の properties 順と一致):
%   1:mass  2:Lx  3:Ly  4:lx  5:ly  6:jx  7:jy  8:jz  9:gravity
%   10:km1  11:km2  12:km3  13:km4  14:k1 ... 21:cableL
%
% 2026/09/11  更新: in2 を [T;tau] 形式に変更 (EKF 互換)
%             内部でロータ変換・クリッピングを行い実効 [T_eff;tau_eff] を適用

% -----------------------------------------------------------------------
% [T; tau_x; tau_y; tau_z] -> ロータ推力変換 -> クリップ -> 実効 [T; tau]
% -----------------------------------------------------------------------
T_cmd    = in2(1,:);
tau_x    = in2(2,:);
tau_y    = in2(3,:);
tau_z    = in2(4,:);

% アロケーション行列パラメータ
Lx  = in3(:,2);
Ly  = in3(:,3);
lx  = in3(:,4);
ly  = in3(:,5);
km1 = in3(:,10);
km2 = in3(:,11);
km3 = in3(:,12);
km4 = in3(:,13);

% アロケーション行列 B: [T;tau] = B * [f1;f2;f3;f4]
% B = [1,    1,       1,       1;
%      -ly,  -ly,     Ly-ly,   Ly-ly;
%       lx,  -(Lx-lx), lx,    -(Lx-lx);
%       km1, -km2,    -km3,    km4]
%
% inv(B) を構成する (数値的に計算するが、係数がスカラーなのでベクトル化)
%   inv(B) * [T; tau_x; tau_y; tau_z] -> [f1; f2; f3; f4]

% 各ロータの最大推力 [N]
f_max = 5.0;

% inv(B) を使ってロータ推力を計算 (Lx=Ly, lx=ly=Lx/2 の等腕X型を前提)
% 一般的な数値 inv(B) を計算してスカラー係数として展開する
% B の構造から解析的に逆行列を計算:
% (等腕配置: Lx=Ly=L, lx=ly=L/2 の場合)
% B = [1,    1,    1,    1;
%      -L/2, -L/2, L/2,  L/2;
%       L/2, -L/2, L/2, -L/2;
%       km,  -km,  -km,  km]   (km1=km2=km3=km4=km)
% inv(B) = 1/4 * [1, -1/ly_eff, 1/lx_eff, 1/km_eff;
%                  1, -1/ly_eff,-1/lx_eff,-1/km_eff;
%                  1,  1/ly_eff, 1/lx_eff,-1/km_eff;
%                  1,  1/ly_eff,-1/lx_eff, 1/km_eff]
% ただし ly_eff = Ly-2*ly (等腕), lx_eff = Lx-2*lx

% 一般配置での inv(B) を数値で計算 (行列として定義してベクトル化)
% スカラーパラメータとして取り出し
Lx_s = Lx(1); Ly_s = Ly(1); lx_s = lx(1); ly_s = ly(1);
km1_s = km1(1); km2_s = km2(1); km3_s = km3(1); km4_s = km4(1);

B_mat = [1,    1,         1,       1;
        -ly_s, -ly_s,    (Ly_s-ly_s), (Ly_s-ly_s);
         lx_s, -(Lx_s-lx_s), lx_s, -(Lx_s-lx_s);
         km1_s, -km2_s,  -km3_s,  km4_s];
IIT = inv(B_mat);  % [f1;f2;f3;f4] = IIT * [T;tau_x;tau_y;tau_z]

% 各ロータ推力を計算
u_cmd = [T_cmd; tau_x; tau_y; tau_z];
f_raw = IIT * u_cmd;  % クリッピング前

% ロータ推力クリッピング (非負・最大推力)
f_clipped = max(0, min(f_max, f_raw));

% 実効 [T;tau] を計算 (クリッピング後のロータ推力から戻す)
u_eff = B_mat * f_clipped;
u1 = u_eff(1,:);  % 実効合力 T_eff
u2 = u_eff(2,:);  % 実効 tau_x_eff
u3 = u_eff(3,:);  % 実効 tau_y_eff
u4 = u_eff(4,:);  % 実効 tau_z_eff

% -----------------------------------------------------------------------
% 以降は with_load_model_mL_euler_for_HL.m と同一の状態方程式
% -----------------------------------------------------------------------
cableL = in3(:,21);
dp1 = in1(7,:);
dp2 = in1(8,:);
dp3 = in1(9,:);
dpl1 = in1(16,:);
dpl2 = in1(17,:);
dpl3 = in1(18,:);
gravity = in3(:,9);
jx = in3(:,6);
jy = in3(:,7);
jz = in3(:,8);
m = in3(:,1);
mL = in1(25,:);
o1 = in1(10,:);
o2 = in1(11,:);
o3 = in1(12,:);
ol1 = in1(22,:);
ol2 = in1(23,:);
ol3 = in1(24,:);
pT1 = in1(19,:);
pT2 = in1(20,:);
pT3 = in1(21,:);
pitch = in1(5,:);
roll = in1(4,:);
yaw = in1(6,:);
t2 = cos(pitch);
t3 = cos(roll);
t4 = sin(pitch);
t5 = sin(roll);
t6 = m+mL;
t7 = ol1.*pT2;
t8 = ol2.*pT1;
t9 = ol1.*pT3;
t10 = ol3.*pT1;
t11 = ol2.*pT3;
t12 = ol3.*pT2;
t13 = 1.0./cableL;
t14 = -gravity;
t15 = 1.0./jx;
t16 = 1.0./jy;
t17 = 1.0./jz;
t18 = 1.0./m;
t23 = pitch./2.0;
t24 = roll./2.0;
t25 = yaw./2.0;
t19 = 1.0./t2;
t20 = -t8;
t21 = -t10;
t22 = -t12;
t26 = cos(t23);
t27 = cos(t24);
t28 = cos(t25);
t29 = sin(t23);
t30 = sin(t24);
t31 = sin(t25);
t32 = 1.0./t6;
t33 = t7+t20;
t34 = t9+t21;
t35 = t11+t22;
t39 = t26.*t27.*t28;
t40 = t26.*t27.*t31;
t41 = t26.*t28.*t30;
t42 = t27.*t28.*t29;
t43 = t26.*t30.*t31;
t44 = t27.*t29.*t31;
t45 = t28.*t29.*t30;
t46 = t29.*t30.*t31;
t36 = t33.^2;
t37 = t34.^2;
t38 = t35.^2;
t47 = -t44;
t48 = -t45;
t52 = t39+t46;
t53 = t42+t43;
t49 = t36+t37+t38;
t54 = t40+t48;
t55 = t41+t47;
t56 = t52.^2;
t57 = t53.^2;
t62 = t52.*t53.*2.0;
t50 = cableL.*m.*t49;
t58 = t54.^2;
t59 = t55.^2;
t60 = -t57;
t63 = t52.*t55.*2.0;
t64 = t53.*t54.*2.0;
t66 = t54.*t55.*2.0;
t61 = -t59;
t65 = -t64;
t67 = t62+t66;
t68 = t63+t65;
t69 = pT1.*t67.*u1;
t70 = pT2.*t67.*u1;
t71 = pT3.*t67.*u1;
t77 = t56+t58+t60+t61;
t72 = pT1.*t68.*u1;
t73 = pT2.*t68.*u1;
t74 = pT3.*t68.*u1;
t78 = pT1.*t77.*u1;
t79 = pT2.*t77.*u1;
t80 = pT3.*t77.*u1;
t81 = t70+t72;
t82 = t74+t79;
t85 = -pT1.*t32.*(t50-t69+t73-t80);
t86 = -pT2.*t32.*(t50-t69+t73-t80);
t87 = -pT3.*t32.*(t50-t69+t73-t80);
mt1 = [dp1;dp2;dp3;t19.*(o1.*t2+o3.*t3.*t4+o2.*t4.*t5);o2.*t3-o3.*t5;t19.*(o3.*t3+o2.*t5);t85-cableL.*(ol2.*t33+ol3.*t34-pT2.*t13.*t18.*t81-pT3.*t13.*t18.*(t71-t78));t86-cableL.*(-ol1.*t33+ol3.*t35+pT1.*t13.*t18.*t81+pT3.*t13.*t18.*t82);t14+t87+cableL.*(ol1.*t34+ol2.*t35+pT2.*t13.*t18.*t82-pT1.*t13.*t18.*(t71-t78));t15.*u2+t15.*(jy.*o2.*o3-jz.*o2.*o3);t16.*u3-t16.*(jx.*o1.*o3-jz.*o1.*o3);t17.*u4+t17.*(jx.*o1.*o2-jy.*o1.*o2);dpl1;dpl2;dpl3;t85;t86;t14+t87;t35;-t9+t10;t33;-t13.*t18.*t82];
mt2 = [-t13.*t18.*(t71-t78);t13.*t18.*t81;0.0];
dx = [mt1;mt2];
end
