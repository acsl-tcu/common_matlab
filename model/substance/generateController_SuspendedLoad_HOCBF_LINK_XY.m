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


% %% =========================================================================
% %% 【決定版】外部確定入力 u1 をそのままドリフト項に保持する HOCBF 自動導出
% %% =========================================================================
% disp("Start: 実入力 u1 を含むダイナミクス f_xy による HOCBF の導出を開始します。");
% 
% % 1. 全ダイナミクス FG_xy の定義
% FG_xy = simplify(f + g * [u1; u2; u3; u4]);
% 
% % 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy
% %    🌟 u1（推力）は 0 にせず、シンボリック変数 u1 のまま f_xy に残ります！
% f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);
% 
% % 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
% g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
% g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
% g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列
% 
% % 3. 障害物安全関数の定義 (ロープ中心 p_mid)
% syms xo yo zo ro real
% syms rl real
% L_cable = physicalParam(7);
% p_mid = pl - 0.5 * L_cable * pT;
% 
% h_cbf_xy = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% cbf1 = h_cbf_xy;
% 
% % 4. クラスK関数のゲイン
% syms gamma1 gamma2 gamma3 gamma4 real
% 
% % 5. f_xy を用いた Lie 微分の計算（相対次数 4）
% % ※ f_xy の中に u1 が入っているため、Lie 微分の中に u1 が自然な形で組み込まれます
% cbf2 = LieD(cbf1, f_xy, x) + diff(cbf1, t) + gamma1 * cbf1; % h2
% cbf3 = LieD(cbf2, f_xy, x) + diff(cbf2, t) + gamma2 * cbf2; % h3
% cbf4 = LieD(cbf3, f_xy, x) + diff(cbf3, t) + gamma3 * cbf3; % h4
% 
% % 最上階での展開（u2, u3 が現れる階層）
% L_f_cbf4  = LieD(cbf4, f_xy, x) + diff(cbf4, t);
% L_g_cbf4  = LieD(cbf4, g1_xy, x);  % [1 x 2] 行列
% L_gy_cbf4 = LieD(cbf4, g1_yaw, x); % スカラー
% 
% % A_qp * [u2; u3] <= b_qp の形に整理（符号反転）
% A_cbf_sym = -L_g_cbf4;
% b_cbf_sym =  L_f_cbf4 + L_gy_cbf4 * u4 + gamma4 * cbf4;
% 
% % 6. 実数値シミュレーション用の変数置換
% %    u1 を外部入力記号 U1_val に、u4 を V4 に、目標軌道微分を XDf に置換
% syms U1_val V4 real
% A_cbf_subs = subs(A_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% 
% % 7. Mファイルとしてエクスポート
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:); 
% 
% disp("Exporting: CBF_Constraints_xyotamesi.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_HOCBF_xylink.m', ...
%     'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
%% =========================================================================
%% 【決定版】外部確定入力 u1 をそのままドリフト項に保持する HOCBF 自動導出
%% =========================================================================
disp("Start: 実入力 u1 を含むダイナミクス f_xy による HOCBF の導出を開始します。");

% 1. 全ダイナミクス FG_xy の定義
FG_xy = simplify(f + g * [u1; u2; u3; u4]);

% 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy
%    🌟 u1（推力）は 0 にせず、シンボリック変数 u1 のまま f_xy に残ります！
f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);

% 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列

% 3. 障害物安全関数の定義 (ロープ中心 p_mid)
syms xo yo zo ro real
syms rl real
L_cable = physicalParam(7);
p_mid = pl - 0.5 * L_cable * pT;

h_cbf_xy = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
cbf1 = h_cbf_xy;

% 4. クラスK関数のゲイン
syms gamma1 gamma2 gamma3 gamma4 real

% 5. f_xy を用いた Lie 微分の計算（相対次数 4）
% ※ f_xy の中に u1 が入っているため、Lie 微分の中に u1 が自然な形で組み込まれます
cbf2 = LieD(cbf1, f_xy, x) + diff(cbf1, t) + gamma1 * cbf1; % h2
cbf3 = LieD(cbf2, f_xy, x) + diff(cbf2, t) + gamma2 * cbf2; % h3
cbf4 = LieD(cbf3, f_xy, x) + diff(cbf3, t) + gamma3 * cbf3; % h4

% 最上階での展開（u2, u3 が現れる階層）
L_f_cbf4  = LieD(cbf4, f_xy, x) + diff(cbf4, t);
L_g_cbf4  = LieD(cbf4, g1_xy, x);  % [1 x 2] 行列
L_gy_cbf4 = LieD(cbf4, g1_yaw, x); % スカラー

% A_qp * [u2; u3] <= b_qp の形に整理（符号反転）
A_cbf_sym = -L_g_cbf4;
b_cbf_sym =  L_f_cbf4 + L_gy_cbf4 * u4 + gamma4 * cbf4;

% 6. 実数値シミュレーション用の変数置換
%    u1 を外部入力記号 U1_val に、u4 を V4 に、目標軌道微分を XDf に置換
syms U1_val V4 real
A_cbf_subs = subs(A_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
b_cbf_subs = subs(b_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);

% 🌟 各階層の関数（h1, h2, h3, h4）を代入用に置換処理
h1_subs = subs(cbf1, [xdReff, u1, u4], [XDf, U1_val, V4]);
h2_subs = subs(cbf2, [xdReff, u1, u4], [XDf, U1_val, V4]);
h3_subs = subs(cbf3, [xdReff, u1, u4], [XDf, U1_val, V4]);
h4_subs = subs(cbf4, [xdReff, u1, u4], [XDf, U1_val, V4]);

% 7. Mファイルとしてエクスポート
gamma_params = [gamma1; gamma2; gamma3; gamma4];
obs_params   = [xo; yo; zo; ro];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:); 

disp("Exporting: CBF_Constraints_HOCBF_xylink.m を書き出しています...");
matlabFunction(A_cbf_subs, b_cbf_subs, h1_subs, h2_subs, h3_subs, h4_subs, ...
    'file', 'CBF_Constraints_HOCBF_xylink.m', ...
    'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
    'outputs', {'A_qp', 'b_qp', 'h1', 'h2', 'h3', 'h4'});
disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【決定版】相対次数4のシステムに対する 論文(Zheng et al., 2025)手法の直接拡張
% %% =========================================================================
% disp("Start: 相対次数4システム用・論文型多重合成CBF(H)の導出を開始します。");
% 
% % 1. 全ダイナミクス FG_xy の定義
% FG_xy = simplify(f + g * [u1; u2; u3; u4]);
% 
% % 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy
% f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);
% 
% % 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
% g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
% g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
% g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列
% 
% % 3. 基礎距離関数 h(x) の定義 (ロープ中心 p_mid)
% syms xo yo zo ro real
% syms rl real
% L_cable = physicalParam(7);
% p_mid = pl - 0.5 * L_cable * pT;
% 
% % h_0(x) (0階: 相対次数4)
% h_cbf_xy = 0.5 * ((p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2);
% 
% % 4. クラスK関数のゲイン (単一パラメータ)
% syms gamma1 gamma2 gamma3 gamma4 real 
% 
% % =========================================================================
% % 🌟 【論文手法の直接拡張】 段階的な H 関数合成による階数低減 (4階 -> 1階)
% % =========================================================================
% 
% % --- Step 1: 1階微分と第1合成 (相対次数 4 -> 3) ---
% L_f_h0 = LieD(h_cbf_xy, f_xy, x) + diff(h_cbf_xy, t);
% H1     = (atan(L_f_h0) + pi/2) * h_cbf_xy;
% 
% % --- Step 2: 2階微分と第2合成 (相対次数 3 -> 2) ---
% L_f_H1 = LieD(H1, f_xy, x) + diff(H1, t);
% H2     = (atan(L_f_H1) + pi/2) * H1;
% 
% % --- Step 3: 3階微分と第3合成 (相対次数 2 -> 1) ---
% L_f_H2 = LieD(H2, f_xy, x) + diff(H2, t);
% H3     = (atan(L_f_H2) + pi/2) * H2;
% 
% % --- Step 4: 最終 Lie 微分（ここで制御入力 u2, u3 が抽出されます！） ---
% L_f_H3  = LieD(H3, f_xy, x) + diff(H3, t);
% L_g_H3  = LieD(H3, g1_xy, x);  % [1 x 2] 行列
% L_gy_H3 = LieD(H3, g1_yaw, x); % スカラー
% 
% % D. A_qp * [u2; u3] <= b_qp の形に整理
% A_cbf_sym = -L_g_H3;
% b_cbf_sym =  L_f_H3 + L_gy_H3 * u4 + gamma1 * H3;
% 
% % 5. 実数値シミュレーション用の変数置換
% syms U1_val V4 real
% A_cbf_subs = subs(A_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% 
% % 6. Mファイルとしてエクスポート
% gamma_params = [gamma1; gamma2; gamma3; gamma4]; % 呼び出し側の引数互換性用
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:); 
% 
% disp("Exporting: CBF_Constraints_xylink.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_xylink_one.m', ...
%     'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: 相対次数4直接対応・論文型CBF関数の書き出しが完了しました！");
% %% =========================================================================
% %% 【改修版】紐角度
% %% =========================================================================
% disp("Start: マルチセグメント球体 HOCBF(xy方向) の導出を開始します。");
% syms u1_val real
% syms max_angle real
% % 1. 物理モデル・運動方程式の分解
% FG_xy = simplify(f + g * [u1_val; u2; u3; u4]);
% g_xy  = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
% f_xy  = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);
% g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
% g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列
% 
% % 2. シンボリック変数の定義
% syms gamma1 gamma2 gamma3 gamma4 real          % CBF ゲイン
% 
% pTz = pT(3); 
% min_pTz = cos(max_angle); % 下限値 (例: cos(30 deg) ≈ 0.866)
% 
% % h_cable = -pTz - min_pTz >= 0
% cbf1 = -pTz - min_pTz;
% 
% % 4. 単一球体に対する 2階 HOCBF の導出（相対次数2）
% % cbf2 = dot_cbf1 + gamma1 * cbf1
% cbf2 = LieD(cbf1, f_xy, x) + diff(cbf1, t) +  gamma1 * cbf1;
% cbf3 = LieD(cbf2, f_xy, x) + diff(cbf2, t) +  gamma2 * cbf2;
% cbf4 = LieD(cbf3, f_xy, x) + diff(cbf3, t) +  gamma3 * cbf3;
% 
% L_f_cbf4 = LieD(cbf4, f_xy, x) + diff(cbf4, t);
% L_g_cbf4 = LieD(cbf4, g1_xy, x); % [1 x 1] のシンボリックスカラー
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf4;
% b_cbf_sym = L_f_cbf4 + gamma4 * cbf4;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params  = [gamma1; gamma2; gamma3; gamma4];
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_Cable.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_Cable.m', ...
%     'vars', {obj, x, XD_sym, cell2sym(V1v),u1_val, max_angle, gamma_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: ケーブル CBF関数の生成が完了しました！");
%% =========================================================================
%% u1の微分の導出
%% =========================================================================

% %% =========================================================================
% %% Make functions of actual inputs taking t, x, xd, v1 and v2 as arguments
% % % If either model, virtual output or parameters is changed, then evaluate this section. It'll take few minutes.
% % % Usage: u = Uf(...) + Us(...)
%     % matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','Uf_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     % matlabFunction(subs(H(:,2:4), [xdRef vInput1], [XD V1v]),'file','H234_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'H234'});
% 
%     %以下二つはとても重い
%     % matlabFunction(subs(inv(beta2), [xdRef vInput1], [XD V1v]),'file','inv_beta2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'inv_beta2'});
% 
%     % matlabFunction(subs((-alpha2+[v2(t);v3(t);v4(t)]), [flip(xdRef) flip(vInput1) v2(t) v3(t) v4(t)], [flip(XD) flip(V1v) V2 V3 V4]),'file','vs_alpha2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'vs_alpha2'});
%     % matlabFunction(subs(alpha2(1), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha21_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha21'});
%     % matlabFunction(subs(alpha2(2), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha22_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha22'});
%     % matlabFunction(subs(alpha2(3), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha23_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha23'});
% 
%     % a2_v2 = subs((-alpha2+[v2(t);v3(t);v4(t)]), [flip(xdRef) flip(vInput1) v2(t) v3(t) v4(t)], [flip(XD) flip(V1v) V2 V3 V4]);
%     % matlabFunction(a2_v2,'file','vs_alpha2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'vs_alpha2'});
% 
%     % xyDst
%     matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdReff vInput1f], [XDf V1vf]),'file','Uf_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     matlabFunction(subs(H(:,2:4), [xdReff vInput1f], [XDf V1vf]),'file','H234_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'H234'});
%     matlabFunction(subs(beta2, [xdReff vInput1f], [XDf V1vf]),'file','Beta2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'beta2'});
%     %以下はとても重い
%     v2_a2 = subs([V2;V3;V4] - alpha2, [xdReff vInput1f], [XDf V1vf]);
%     matlabFunction(v2_a2,'file','V2_alpha2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'});
% 
% %理想のfunctionだけどUsが重すぎるので分割している．
%     % matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XD V1v]),'file','Uf_SuspededLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     % matlabFunction(subs(H(:,2:4)*U2, [xdRef vInput1 v2(t) v3(t) v4(t)], [XD V1v [V2 V3 V4]]),'file','Us_SuspededLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'U2'});
% 
% % % For check
% %     Uf(0,x0,Xd(0),Vf(0,x0,Xd(0)))
% %     Us(0,x0,Xd(0),Vf(0,x0,Xd(0)),Vs(0,x0,Xd(0),Vf(0,x0,Xd(0))))
% % %%
% % matlabFunction(subs(alpha1, [xdRef], [XD]),'file','alpha1.m','vars',{obj cell2sym(XD) physicalParam},'outputs',{'al1'});
% % matlabFunction(subs(alpha2, [xdRef vInput1], [XD V1v]),'file','alpha2.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'al2'});
% % %%
% % matlabFunction(subs(beta2, [xdRef vInput1], [XD V1v]),'file','beta2.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'be2'});
% % %%
% % matlabFunction(subs(He, [xdRef vInput1], [XD V1v]),'file','He.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) e1 physicalParam},'outputs',{'mat'});
% % %%
% % matlabFunction(beta1,'file','beta1.m','vars',{obj t x physicalParam},'outputs',{'beta1'});


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
