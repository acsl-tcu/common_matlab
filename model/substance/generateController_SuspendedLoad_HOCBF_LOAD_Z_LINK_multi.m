% このスクリプトが置いてあるフォルダの絶対パスを取得して移動
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

%% Define the nonlinear physical model of a quadrotor
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly  jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
syms mL l Length cableL rotor_r real % LengthとcableLは同じ意味,lは機体の長さ正方形を仮定
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 ol1 ol2 ol3 real
syms pT1 pT2 pT3 real
syms obj % dummy
%% Controller design
clc
syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t) % 牽引物体の位置＋機体のyaw角
syms t real
xd = [xd1(t),xd2(t),xd3(t),xd4(t)]; % reference を後で決める時はこっち
%% 
p	= [  p1;  p2;  p3];             % Position　：xb : 進行方向，zb ：ホバリング時に上向き
dp	= [ dp1; dp2; dp3];             % Velocity
ddp	= [ddp1;ddp2;ddp3];             % Accelaletion
q	= [  q0;  q1;  q2;  q3];        % Quaternion
ob	= [  o1;  o2;  o3];             % Angular velocity
pl  = [ pl1; pl2; pl3];             % Load position
dpl = [dpl1;dpl2;dpl3];             % Load velocity
ol  = [ ol1; ol2; ol3];             % Load angular velocity
pT  = [ pT1; pT2; pT3];             % String position
x=[q;ob;pl;dpl;pT;ol];
% physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL];
% f = FL(x,physicalParam);
% g = GL(x,physicalParam);
% physicalParam = [m, jx, jy, jz, gravity,mL,cableL];

syms dstx dsty real
physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL, dstx, dsty];

f = FLxyDst(x,physicalParam);
g = GLxyDst(x,physicalParam);
physicalParam = [m, jx, jy, jz, gravity,mL,cableL,dstx, dsty];
%% 1st layer
clc
% % Define virtual output: h1
h1 = pl3 - xd(3);
dh1 = LieD(h1,f,x)+diff(h1,t);
alpha1 = LieD(dh1,f,x)+diff(dh1,t);
beta1 = simplify(LieD(dh1,g,x));
H = [1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1); zeros(3,1),eye(3)];
%%
% syms e1 e2 real
% He = [1/(beta1(1)+e1),-beta1(2)/(beta1(1)+e1),-beta1(3)/(beta1(1)+e1),-beta1(4)/(beta1(1)+e1); zeros(3,1),eye(3)];
% H = pinv(beta1);
% nH = null(H');


% syms v1(t) dv1(t) ddv1(t) dddv1(t) ddddv1(t)
% dv1(t) = diff(v1(t),t);
% ddv1(t) = diff(v1(t),t,2);
% dddv1(t) = diff(v1(t),t,3);
% ddddv1(t) = diff(v1(t),t,4);
% Qz = diag([1.0,1.0]);               % Weight of state
% Rz = 0.1;                           % Weight of input
% Nz = 0;
% Fz = lqr([0,1;0,0], [0;1], Qz, Rz, Nz);
 %v1 = -Fz * [h1;dh1]; % v1を事前に決めておく時はこっち
%% 2nd layer
syms v2 v3 v4
FG = simplify(f+g*H*[-alpha1+v1(t);u2;u3;u4]);	% v1を後で設計する時はこっち
g1 = simplify(MyCoeff(FG,[u2;u3;u4]));
f1 = subs(FG,[u2,u3,u4],[0,0,0]);
%simplify(FG-(f1+g1*[v2;v3;v4]))   % For check
simplify(FG-(f1+g1*[u2;u3;u4]))   % For check

% FG = simplify(f+g*(H*(-alpha1+v1(t))+nH*[v2;v3;v4]));	% v1を後で設計する時はこっち
% g1 = simplify(MyCoeff(FG,[v2;v3;v4]));
% f1 = subs(FG,[v2,v3,v4],[0,0,0]);

%%
% % Define virtual output: h2, h3, h4
h2 = pl1 - xd(1);
h3 = pl2 - xd(2);
[~,~,yaw] = Quat2Eul(q);
h4 = yaw - xd(4);

%% **************************************** % %
clc
dh4 = LieD(h4,f1,x)+diff(h4,t);
dh2 = LieD(h2,f1,x)+diff(h2,t);
dh3 = LieD(h3,f1,x)+diff(h3,t);
d2h2 = LieD(dh2,f1,x)+diff(dh2,t);
d2h3 = LieD(dh3,f1,x)+diff(dh3,t);
d3h2 = LieD(d2h2,f1,x)+diff(d2h2,t);
d3h3 = LieD(d2h3,f1,x)+diff(d2h3,t);
d4h2 = LieD(d3h2,f1,x)+diff(d3h2,t);
d4h3 = LieD(d3h3,f1,x)+diff(d3h3,t);
d5h2 = LieD(d4h2,f1,x)+diff(d4h2,t);
d5h3 = LieD(d4h3,f1,x)+diff(d4h3,t);
%% % For check
	[LieD(h2,g1,x),LieD(h3,g1,x),LieD(h4,g1,x)]
	simplify([LieD(dh2,g1,x),LieD(dh3,g1,x)])
	simplify([LieD(d2h2,g1,x),LieD(d2h3,g1,x)])
  	simplify([LieD(d3h2,g1,x),LieD(d3h3,g1,x)])
 	simplify([LieD(d4h2,g1,x),LieD(d4h3,g1,x)])
 	% simplify([LieD(d5h2,g1,x),LieD(d5h3,g1,x)])
%% trial
% simplify してからmatlabFunctionは意味がない（というか遅くなった）
% a2_1=simplify(LieD(d5h2,f1,x)+diff(d5h2,t));
% matlabFunction(subs(a2_1,[xdReff vInput1f], [XDf V1vf]),"file",'test1.m','Vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'})
% matlabFunction(subs(LieD(d5h2,f1,x)+diff(d5h2,t),[xdReff vInput1f], [XDf V1vf]),"file",'test2.m','Vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'})
%%
% 2+6+6+2
% % Derive 2nd layer controller
% alpha2 = [LieD(d5h2,f1,x)+diff(d5h2,t); LieD(d5h3,f1,x)+diff(d5h3,t)];
% beta2 = [LieD(d5h2,g1,x); LieD(d5h3,g1,x)];
% H2 = pinv(beta2);
% nb2 = null(beta2);
alpha2 = [LieD(d5h2,f1,x)+diff(d5h2,t); LieD(d5h3,f1,x)+diff(d5h3,t); LieD(dh4,f1,x)+diff(dh4,t)];
beta2 = [LieD(d5h2,g1,x); LieD(d5h3,g1,x); LieD(dh4,g1,x)];
%%
% FG2 = simplify(f1+g1*(H2*(-alpha2+[v2(t);v3(t)])+nb2*[v4]));	% v1を後で設計する時はこっち
% g2 = simplify(MyCoeff(FG2,[v4]));
% f2 = subs(FG,v4,0);
% syms v2(t) v3(t) v4(t)
    % salpha = simplify(alpha2);
    % sbeta2 = simplify(beta2);
    % U2 = sbeta2\(-salpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
    % U2 = inv(beta2)\(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
%     U2e = (adjoint(beta2)/(det(beta2)+e2))*(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
%end
%% Initialize xd as an unspecified function of t
% % If regenerate Uf, Us or Xd functions, evaluate this section.
    xd = [xd1(t),xd2(t),xd3(t),xd4(t)];
    dxd = diff(xd,t);
    d2xd = diff(dxd,t);
    d3xd = diff(d2xd,t);
    d4xd = diff(d3xd,t);
    d5xd = diff(d4xd,t);
    d6xd = diff(d5xd,t);
%% Set variables for output functions
%fix
    syms Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 d2Xd1 d2Xd2 d2Xd3 d2Xd4 d3Xd1 d3Xd2 d3Xd3 d3Xd4 d4Xd1 d4Xd2 d4Xd3 d4Xd4 d5Xd1 d5Xd2 d5Xd3 d5Xd4 d6Xd1 d6Xd2 d6Xd3 d6Xd4 real
    syms V1 V2 V3 V4 dV1 d2V1 d3V1 d4V1 d5V1 real
    XD = {Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 d2Xd1 d2Xd2 d2Xd3 d2Xd4 d3Xd1 d3Xd2 d3Xd3 d3Xd4 d4Xd1 d4Xd2 d4Xd3 d4Xd4 d5Xd1 d5Xd2 d5Xd3 d5Xd4 d6Xd1 d6Xd2 d6Xd3 d6Xd4};
    V1v = {V1 dV1 d2V1 d3V1 d4V1 d5V1};
    xdRef = [xd dxd d2xd d3xd d4xd d5xd d6xd];
    vInput1 = [v1(t) diff(v1(t),t) diff(v1(t),t,2) diff(v1(t),t,3) diff(v1(t),t,4) diff(v1(t),t,5)];
    XDf = flip(XD);
    V1vf = flip(V1v);
    xdReff = flip(xdRef);
    vInput1f = flip(vInput1);
 %% Make functions of virtual output
% % If either model, virtual output or parameters is changed, then evaluate this section.
    disp("Start: make functions of virtual states.");
    % matlabFunction(subs([h1;dh1], [xdReff], [XDf]),'file','Z1_SuspendedLoad.m','vars',{obj x cell2sym(XD) physicalParam},'outputs',{'cZ1'});
    % matlabFunction(subs([h2;dh2;d2h2;d3h2;d4h2;d5h2], [xdReff vInput1f], [XDf V1vf]),'file','Z2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ2'});
    % matlabFunction(subs([h3;dh3;d2h3;d3h3;d4h3;d5h3], [xdReff vInput1f], [XDf V1vf]),'file','Z3_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ3'});
    % matlabFunction(subs([h4;dh4], [xdReff vInput1f], [XDf V1vf]),'file','Z4_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ4'});
    %xydst用
    matlabFunction(subs([h1;dh1], [xdReff], [XDf]),'file','Z1_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) physicalParam},'outputs',{'cZ1'});
    matlabFunction(subs([h2;dh2;d2h2;d3h2;d4h2;d5h2], [xdReff vInput1f], [XDf V1vf]),'file','Z2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ2'});
    matlabFunction(subs([h3;dh3;d2h3;d3h3;d4h3;d5h3], [xdReff vInput1f], [XDf V1vf]),'file','Z3_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ3'});
    matlabFunction(subs([h4;dh4], [xdReff vInput1f], [XDf V1vf]),'file','Z4_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ4'});
%% Make functions of virtual inputs
clc
    disp("Start: make functions of virtual inputs.");
    clear dt
    syms f11 f12 f21 f22 f23 f24 f25 f26 f31 f32 f33 f34 f35 f36 f41 f42 dt k real
    F1 = [f11 f12];
    F2 = [f21 f22 f23 f24 f25 f26];
    F3 = [f31 f32 f33 f34 f35 f36];
    F4 = [f41 f42];
    A1=[0,1;0,0]-[0;1]*F1; % closed loop : continuous
    % matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdRef], [XD]),'file','Vf_SuspendedLoad.m','vars',{x cell2sym(XD) physicalParam F1},'outputs',{'V1'});
    % matlabFunction(subs([-F2*[h2;dh2;d2h2;d3h2;d4h2;d5h2],-F3*[h3;dh3;d2h3;d3h3;d4h3;d5h3],-F4*[h4;dh4]], [xdRef vInput1], [XD V1v]),'file','Vs_SuspendedLoad.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});

    A1 = expm([0,1;0,0]*dt)-int(expm([0,1;0,0]*(dt-k))*[0;1],k,[0,dt])*F1; % closed loop discrete
    % matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdRef], [XD]),'file','Vfd_SuspendedLoad.m','vars',{dt x cell2sym(XD) physicalParam F1},'outputs',{'Vd1'});
    A2 = expm(diag([1,1,1,1,1],1)*dt)-int(expm(diag([1,1,1,1,1],1)*(dt-k))*[0;0;0;0;0;1],k,[0,dt])*F2; % closed loop discrete
    A3 = expm(diag([1,1,1,1,1],1)*dt)-int(expm(diag([1,1,1,1,1],1)*(dt-k))*[0;0;0;0;0;1],k,[0,dt])*F3; % closed loop discrete
    A4 = expm([0 1;0 0]*dt)-int(expm([0 1;0 0]*(dt-k))*[0;1],k,[0,dt])*F4; % closed loop discrete
    
    %xydst用
    A1=[0,1;0,0]-[0;1]*F1; % closed loop : continuous
    matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdReff], [XDf]),'file','Vf_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) F1},'outputs',{'V1'});
    A1 = expm([0,1;0,0]*dt)-int(expm([0,1;0,0]*(dt-k))*[0;1],k,[0,dt])*F1; % closed loop discrete
    matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdReff], [XDf]),'file','Vfd_SuspendedLoadxyDst.m','vars',{obj dt x cell2sym(XD) F1},'outputs',{'Vd1'});
    matlabFunction(subs([-F2*[h2;dh2;d2h2;d3h2;d4h2;d5h2],-F3*[h3;dh3;d2h3;d3h3;d4h3;d5h3],-F4*[h4;dh4]], [xdReff vInput1f], [XDf V1vf]),'file','Vs_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});

    % % For check
%     Vf(0,x0,Xd(0))
%     Vs(0,x0,Xd(0),Vf(0,x0,Xd(0)))
%% =========================================================================
%% 徹底デバッグ：2nd layer 操作入力 [u2; u3] に対する HOCBF 微分チェーン検証
%% =========================================================================
clc;
disp("=================================================================");
disp(" 2nd layer システム (f_xy, g1_xy) による Lie 微分チェーンの徹底検証");
disp("=================================================================");

% 1. 全ダイナミクス FG_xy の定義
FG_xy = simplify(f + g * [u1; u2; u3; u4]);

% 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy （u1 は保持）
f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);

% 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列 (19x2 行列)
g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列 (19x1 ベクトル)

% 3. パラメータ設定と機体基準位置 xQ の定義
L_cable = physicalParam(7);
xQ = pl - L_cable * pT; % 機体位置ベクトル (xL と pT から代数計算)

syms xo yo zo ro rl real
syms gamma1 gamma2 gamma3 gamma4 real

% 4. 複数球の構成 (lam = 0.0: 機体, 0.5: ロープ中間, 1.0: 荷物)
lambda_list = [0.0, 0.5, 1.0];
num_spheres = length(lambda_list);

A_cbf_list = sym(zeros(num_spheres, 2));
b_cbf_list = sym(zeros(num_spheres, 1));

disp(" ");
disp("--- [階層別 Lie 微分検証開始] ---");

for k = 1:num_spheres
    lam = lambda_list(k);
    p_sphere = xQ + lam * L_cable * pT; % 球の中心位置
    
    % 安全関数 h
    h_k = (p_sphere(1) - xo)^2 + (p_sphere(2) - yo)^2 + (p_sphere(3) - zo)^2 - (ro + rl)^2;
    
    % ---------------------------------------------------------------------
    % Lie 微分の順次計算と各階層での g1_xy ([u2; u3]) 出現判定
    % ---------------------------------------------------------------------
    % 1階微分
    cbf1_k = h_k;
    L_g_1 = LieD(cbf1_k, g1_xy, x);
    cbf2_k = LieD(cbf1_k, f_xy, x) + diff(cbf1_k, t) + gamma1 * cbf1_k;
    
    % 2階微分
    L_g_2 = LieD(cbf2_k, g1_xy, x);
    cbf3_k = LieD(cbf2_k, f_xy, x) + diff(cbf2_k, t) + gamma2 * cbf2_k;
    
    % 3階微分
    L_g_3 = LieD(cbf3_k, g1_xy, x);
    cbf4_k = LieD(cbf3_k, f_xy, x) + diff(cbf3_k, t) + gamma3 * cbf3_k;
    
    % 4階微分（最上階）
    L_f_4  = LieD(cbf4_k, f_xy, x) + diff(cbf4_k, t);
    L_g_4  = LieD(cbf4_k, g1_xy, x);  % [1 x 2] 行列 (u2, u3 の係数)
    L_gy_4 = LieD(cbf4_k, g1_yaw, x); % スカラー (u4 の係数)
    
    % ---------------------------------------------------------------------
    % デバッグ出力 (コンソール表示)
    % ---------------------------------------------------------------------
    fprintf('\n【Protection Sphere %d (lambda = %.1f)】\n', k, lam);
    fprintf('  1階 LieD(cbf1, g1_xy) = [%s]\n', char(simplify(L_g_1)));
    fprintf('  2階 LieD(cbf2, g1_xy) = [%s]\n', char(simplify(L_g_2)));
    fprintf('  3階 LieD(cbf3, g1_xy) = [%s]\n', char(simplify(L_g_3)));
    
    is_nonzero_4 = ~isequal(simplify(L_g_4), sym([0, 0]));
    if is_nonzero_4
        fprintf('  4階 LieD(cbf4, g1_xy) = [サイズ: %dx%d] -> ✅✅✅ NON-ZERO! [u2; u3] の係数を獲得！\n', size(L_g_4,1), size(L_g_4,2));
    else
        fprintf('  4階 LieD(cbf4, g1_xy) = [0, 0] -> ❌ 4階でも非出現。モデルの連結状態を確認してください。\n');
    end
    
    % QP 制約式 A_qp * [u2; u3] <= b_qp の構築 (符号反転)
    A_cbf_list(k, :) = -L_g_4;
    b_cbf_list(k, 1) =  L_f_4 + L_gy_4 * u4 + gamma4 * cbf4_k;
end

% 5. 実数値シミュレーション用の変数代入置換
syms U1_val V4 real
A_cbf_subs = subs(A_cbf_list, [xdReff, u1, u4], [XDf, U1_val, V4]);
b_cbf_subs = subs(b_cbf_list, [xdReff, u1, u4], [XDf, U1_val, V4]);

% 6. 関数ファイル書き出し
gamma_params = [gamma1; gamma2; gamma3; gamma4];
obs_params   = [xo; yo; zo; ro];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:); 

disp(" ");
disp("Exporting: CBF_Constraints_multi_spheres_2ndLayer.m を書き出しています...");
matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_multi_spheres_2ndLayer.m', ...
               'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
               'outputs', {'A_qp', 'b_qp'});

disp("=================================================================");
disp(" 徹底デバッグ完了：結果ログを確認してください");
disp("=================================================================");
%% =========================================================================
%% 【確定検証デバッグ付き】純粋な荷物位置 pL を対象とした HOCBF (相対次数4) 導出
%% =========================================================================
clc;
disp("=================================================================");
disp(" Start: 純粋な荷物位置 pL に対する 2nd layer HOCBF の検証と導出");
disp("=================================================================");

% 1. 全ダイナミクス FG_xy の定義と 2nd layer への分解
FG_xy = simplify(f + g * [u1; u2; u3; u4]);

% 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy（推力 u1 はシンボリック保持）
f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);

% 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列

% 3. 障害物パラメータとクラスK関数のゲイン設定
syms xo yo zo ro real
syms rl real
syms gamma1 gamma2 gamma3 gamma4 real

% 🌟 【ターゲット】純粋な荷物位置 pl = [pl1; pl2; pl3] を用いた安全関数
h_payload_xy = (pl(1) - xo)^2 + (pl(2) - yo)^2 + (pl(3) - zo)^2 - (ro + rl)^2;

% -------------------------------------------------------------------------
% 🔍【生成時デバッグ・Lie 微分チェーンの段階的検証】
% -------------------------------------------------------------------------
disp(" ");
disp("--- [Lie 微分チェーンの動作検証を開始します] ---");

% 1階微分
cbf1_pL = h_payload_xy;
Lg_1 = LieD(cbf1_pL, g1_xy, x);
cbf2_pL = LieD(cbf1_pL, f_xy, x) + diff(cbf1_pL, t) + gamma1 * cbf1_pL;

% 2階微分
Lg_2 = LieD(cbf2_pL, g1_xy, x);
cbf3_pL = LieD(cbf2_pL, f_xy, x) + diff(cbf2_pL, t) + gamma2 * cbf2_pL;

% 3階微分
Lg_3 = LieD(cbf3_pL, g1_xy, x);
cbf4_pL = LieD(cbf3_pL, f_xy, x) + diff(cbf3_pL, t) + gamma3 * cbf3_pL;

% 4階微分 (最上階)
L_f_pL  = LieD(cbf4_pL, f_xy, x) + diff(cbf4_pL, t);
L_g_pL  = LieD(cbf4_pL, g1_xy, x);  % [1 x 2] 行列 (u2, u3 の係数)
L_gy_pL = LieD(cbf4_pL, g1_yaw, x); % スカラー (u4 の係数)

% -------------------------------------------------------------------------
% 📊 【コンソール出力とアサート判定】
% -------------------------------------------------------------------------
fprintf('  1階 LieD(cbf1, g1_xy) = [%s]\n', char(simplify(Lg_1)));
fprintf('  2階 LieD(cbf2, g1_xy) = [%s]\n', char(simplify(Lg_2)));
fprintf('  3階 LieD(cbf3, g1_xy) = [%s]\n', char(simplify(Lg_3)));

is_nonzero_4 = ~isequal(simplify(L_g_pL), sym([0, 0]));

if is_nonzero_4
    disp(" ");
    disp(" ✅ 【検証成功】荷物位置 pL に対する 4階微分で [u2; u3] の係数を獲得しました！");
    fprintf('    L_g_pL のサイズ: [%d x %d]\n', size(L_g_pL,1), size(L_g_pL,2));
else
    disp(" ");
    error(" ❌ 【検証失敗】4階微分でも L_g_pL が [0, 0] です。ダイナミクス定義を確認してください。");
end

% -------------------------------------------------------------------------
% 4. QP 用形式 A_qp * [u2; u3] <= b_qp への整理 (符号反転)
% -------------------------------------------------------------------------
A_payload_sym = -L_g_pL;
b_payload_sym =  L_f_pL + L_gy_pL * u4 + gamma4 * cbf4_pL;

% 5. 実数値シミュレーション用の変数置換
syms U1_val V4 real
A_payload_subs = subs(A_payload_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
b_payload_subs = subs(b_payload_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);

% 6. M ファイルとして書き出し
gamma_params = [gamma1; gamma2; gamma3; gamma4];
obs_params   = [xo; yo; zo; ro];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:); 

disp(" ");
disp("Exporting: CBF_Constraints_payload_xy.m を書き出しています...");
matlabFunction(A_payload_subs, b_payload_subs, 'file', 'CBF_Constraints_payload_xy.m', ...
    'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
    'outputs', {'A_qp', 'b_qp'});

disp("=================================================================");
disp(" 確定検証完了：純粋な荷物位置 pL に対する CBF Mファイルが生成されました！");
disp("=================================================================");
%% 【複数球対応・確定検証版】実入力 u1 を保持した HOCBF 導出 (xy方向: 相対次数4)
%% =========================================================================
disp("-----------------------------------------------------------------");
disp("Start: 複数球 HOCBF (xy方向) の導出と相対次数検証を開始します...");

% 1. 全ダイナミクス FG_xy の定義
FG_xy = simplify(f + g * [u1; u2; u3; u4]);

% 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy （u1 は保持）
f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);

% 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列

% 3. 障害物パラメータとクラスK関数のゲイン設定
syms xo yo zo ro real
syms rl real
syms gamma1 gamma2 gamma3 gamma4 real
L_cable = physicalParam(7);

% 4. 複数球の構成 (機体: lam=0.0, 中間: lam=0.5, 荷物: lam=1.0)
lambda_list = [0.0, 0.5, 1.0]; 
num_spheres = length(lambda_list);

A_cbf_list = sym(zeros(num_spheres, 2));
b_cbf_list = sym(zeros(num_spheres, 1));

for k = 1:num_spheres
    lam = lambda_list(k);
    
    % 位置ベクトルの代数表現 (機体: lam=0, 荷物: lam=1)
    p_sphere = pl - (1 - lam) * L_cable * pT;
    
    % 安全関数 h の定義
    h_k = (p_sphere(1) - xo)^2 + (p_sphere(2) - yo)^2 + (p_sphere(3) - zo)^2 - (ro + rl)^2;
    
    % ---------------------------------------------------------------------
    % 🌟 【相対次数4の厳密検証セクション】
    % ---------------------------------------------------------------------
    % 各階層における入力応答性 (LieD(cbf_i, g1_xy)) をチェック
    Lg_h1 = LieD(h_k, g1_xy, x);
    cbf2_k = LieD(h_k, f_xy, x) + diff(h_k, t) + gamma1 * h_k;
    
    Lg_h2 = LieD(cbf2_k, g1_xy, x);
    cbf3_k = LieD(cbf2_k, f_xy, x) + diff(cbf2_k, t) + gamma2 * cbf2_k;
    
    Lg_h3 = LieD(cbf3_k, g1_xy, x);
    cbf4_k = LieD(cbf3_k, f_xy, x) + diff(cbf3_k, t) + gamma3 * cbf3_k;
    
    L_f_k  = LieD(cbf4_k, f_xy, x) + diff(cbf4_k, t);
    L_g_k  = LieD(cbf4_k, g1_xy, x);  % [1 x 2] 入力 u2, u3 の係数
    L_gy_k = LieD(cbf4_k, g1_yaw, x); % u4 (yaw) の係数
    
    % 🔍【自動判定コード】4階微分で確実に入力が出現しているか検証
    is_Lg4_nonzero = ~isequal(L_g_k, sym([0, 0]));
    
    if is_Lg4_nonzero
        fprintf('  [Sphere %d (lam=%.1f)] ✅ 相対次数 4 が確認されました！(L_g*cbf4 != 0)\n', k, lam);
    else
        error('  [Sphere %d (lam=%.1f)] ❌ 警告: 4階微分に入力が出現しませんでした。数式モデルを確認してください。', k, lam);
    end
    
    % A_qp * [u2; u3] <= b_qp の形式 (符号反転)
    A_cbf_list(k, :) = -L_g_k;
    b_cbf_list(k, 1) =  L_f_k + L_gy_k * u4 + gamma4 * cbf4_k;
end

% 5. 実数値シミュレーション用の変数置換
syms U1_val V4 real
A_cbf_subs = subs(A_cbf_list, [xdReff, u1, u4], [XDf, U1_val, V4]);
b_cbf_subs = subs(b_cbf_list, [xdReff, u1, u4], [XDf, U1_val, V4]);

% 6. Mファイルとして書き出し
gamma_params = [gamma1; gamma2; gamma3; gamma4];
obs_params   = [xo; yo; zo; ro];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:); 

disp("Exporting: CBF_Constraints_multi_spheres_xy.m を書き出しています...");
matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_multi_spheres_xy.m', ...
               'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
               'outputs', {'A_qp', 'b_qp'});
disp("Done: xy方向 複数球 CBF 関数のエクスポートが完了しました！");


%% =========================================================================
%% 【複数球対応・確定検証版】高次制御バリア関数 z方向 (HOCBF: 相対次数2)
%% =========================================================================
disp("-----------------------------------------------------------------");
disp("Start: 複数球 HOCBF (z方向) の導出と相対次数検証を開始します...");

FG_z = simplify(f + g * [u1; u2; u3; u4]);
g_z  = simplify(MyCoeff(FG_z, [u1; u2; u3; u4]));
f_z  = subs(FG_z, [u1, u2, u3, u4], [0, 0, 0, 0]);

% u1（推力）に掛かる1列目を抽出
g1_z = g_z(:, 1); 

syms gamma1_z gamma2_z real
A_cbf_list_z = sym(zeros(num_spheres, 1));
b_cbf_list_z = sym(zeros(num_spheres, 1));

for k = 1:num_spheres
    lam = lambda_list(k);
    
    % 各球の位置ベクトル
    p_sphere = pl - (1 - lam) * L_cable * pT;
    
    % z方向/3次元距離の安全関数
    h_k_z = (p_sphere(1) - xo)^2 + (p_sphere(2) - yo)^2 + (p_sphere(3) - zo)^2 - (ro + rl)^2;
    
    % ---------------------------------------------------------------------
    % 🌟 【相対次数2の厳密検証セクション】
    % ---------------------------------------------------------------------
    Lg_hz1 = LieD(h_k_z, g1_z, x);
    cbf2_k_z = LieD(h_k_z, f_z, x) + diff(h_k_z, t) + gamma1_z * h_k_z;
    
    L_f_cbf2_z = LieD(cbf2_k_z, f_z, x) + diff(cbf2_k_z, t);
    L_g_cbf2_z = LieD(cbf2_k_z, g1_z, x); % u1 の係数 (1x1 スカラー)
    
    % 🔍【自動判定コード】2階微分で確実に入力 u1 が出現しているか検証
    is_Lgz2_nonzero = ~isequal(L_g_cbf2_z, sym(0));
    
    if is_Lgz2_nonzero
        fprintf('  [Sphere %d (lam=%.1f)] ✅ 相対次数 2 が確認されました！(L_g*cbf2_z != 0)\n', k, lam);
    else
        error('  [Sphere %d (lam=%.1f)] ❌ 警告: z方向の2階微分に入力 u1 が出現しませんでした。', k, lam);
    end
    
    % A_qp * u1 <= b_qp の形に変形 (符号反転)
    A_cbf_list_z(k, 1) = -L_g_cbf2_z; 
    b_cbf_list_z(k, 1) =  L_f_cbf2_z + gamma2_z * cbf2_k_z;
end

% 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
A_cbf_subs_z = subs(A_cbf_list_z, [xdReff, vInput1f], [XDf, V1vf]);
b_cbf_subs_z = subs(b_cbf_list_z, [xdReff, vInput1f], [XDf, V1vf]);

% Mファイルとしてエクスポート
gamma_params_z = [gamma1_z; gamma2_z];
disp("Exporting: CBF_Constraints_multi_spheres_z.m を書き出しています...");
matlabFunction(A_cbf_subs_z, b_cbf_subs_z, 'file', 'CBF_Constraints_multi_spheres_z.m', ...
    'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params_z, sys_params, physicalParam}, ...
    'outputs', {'A_qp', 'b_qp'});
disp("Done: z方向 複数球 CBF 関数のエクスポートが完了しました！");
disp("-----------------------------------------------------------------");


%% Local functions
function tmp = DeleteCommentLine(fname)
% % DeleteCommentLine : Remove comments and blank lines in functions
% % % Load file
	fp = fopen(strcat(fname,'.m'),'r');
	str = textscan(fp,'%s','delimiter','\n');
	str = str{1};
	fclose(fp);
	tmp = length(str);
% % % Check comment line and blank line
	str = str(~strncmp(str,'%',1));
	str = str(~strncmp(str,'',1));
	tmp2 = length(str);
% % % Update file
	if (tmp~=tmp2) && (tmp2 ~= 0)
		fp = fopen(strcat(fname,'.m'),'w');
		cellfun(@(a) fprintf(fp,'%s\n',a),str,'UniformOutput',false);
		fclose(fp);
	end
end
function pd = pdiff(flist, vars)
% % flistをvarsで微分したもののシンボリック配列を返す？
    flist = flist(:);
    vars = vars(:);
%    pd = arrayfun(@(x) diff(flist(1), x), vars)';
    pdiff_tmp = @(flist, vars) arrayfun(@(func) arrayfun(@(x) diff(func, x), vars)',flist,'UniformOutput',false);
    pd = cell2sym(pdiff_tmp(flist,vars));
end
function dh = LieD(h,f,x)
    x = x(:);
    dh = pdiff(h,x)*f;
end
function mat = MyCoeff(M,vars)
    vars = vars(:);
    mat = coder.nullcopy(sym(zeros(length(M),length(vars))));
    for i = 1:length(M)
        for j = 1:length(vars)
            mat(i,j) = subs(M(i)-subs(M(i),vars(j),0),vars(j),1);
        end
    end
end
function [eul,th,psi] = Quat2Eul(q,varargin)
% % phi...roll, th...pitch, psi...yaw
    switch nargin
        case 1
            q0 = q(1);
            q1 = q(2);
            q2 = q(3);
            q3 = q(4);
        case 4
            q0 = q;
            q1 = varargin{1};
            q2 = varargin{2};
            q3 = varargin{3};
    end
	phi = atan2((2*(q0*q1 + q2*q3)),(q0^2 - q1^2 - q2^2 + q3^2));
	th  = asin(2*(q0*q2 - q1*q3));
	psi = atan2((2*(q0*q3 + q1*q2)),(q0^2 + q1^2 - q2^2 - q3^2));
	switch  nargout
        case 1
		eul = [phi, th, psi];
        case 3
		eul = phi;
	end
end
function R=Rodrigues(u,th)
    u = u(:);
    u = u/norm(u);
    R=u*u'+cos(th)*(eye(3)-u*u')+sin(th)*Skew(u);
end
function Om = Skew(o)
% % Skew : Skew symmetric matrix
    o  = o(:);
    o1 = o(1);
    o2 = o(2);
    o3 = o(3);
    Om = [  0,-o3, o2;
		   o3,  0,-o1;
		  -o2, o1,  0];
end
function R = RodriguesQuaternion(q)
% % RodriguesQuaternion : R is the rotation matrix
    q  = q(:);
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
	E = [-q1, q0,-q3, q2;
		 -q2, q3, q0,-q1;
		 -q3,-q2, q1, q0];
	L = [-q1, q0, q3,-q2;
		 -q2,-q3, q0, q1;
		 -q3, q2,-q1, q0];
	R = E*L';
end
