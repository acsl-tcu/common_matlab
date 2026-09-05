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
% %% 【FastBridge 準拠】幾何チェーンルールによる相対次数4 ECBF 厳密導出
% %%  - p_mid の運動方程式連鎖から直接 \Omega と M の結合を抽出
% %% =========================================================================
% disp("==============================================================");
% disp(" Start: FastBridge 型 相対次数4 ECBF (幾何連鎖トルク感度抽出) 導出");
% disp(" Target: A_cbf * u <= b_cbf");
% disp("==============================================================");
% 
% % 1. シンボリック変数の定義
% syms u1_var u2_var u3_var u4_var real
% u_decision = [u1_var; u2_var; u3_var; u4_var];
% tau_decision = [u2_var; u3_var; u4_var]; % 3軸トルク
% 
% syms u1_nom real                       % トルク結合用推力 (ホバリング/公称推力)
% syms du1 d2u1 real                     % 推力1階・2階差分
% syms xo yo zo ro rl real               % 障害物位置・半径
% syms lambda real                       % Hurwitz ゲイン
% 
% % 物理パラメータの展開
% mQ_val = physicalParam(1);
% JQ_mat = diag([physicalParam(2), physicalParam(3), physicalParam(4)]);
% g_acc  = physicalParam(5);
% mL_val = physicalParam(6);
% L_val  = physicalParam(7);
% e3     = [0; 0; 1];
% 
% % 2. 状態の幾何表現
% R_mat = RodriguesQuaternion(q); % 機体回転行列 (3x3)
% p_obs = [xo; yo; zo];
% r_safe = ro + rl;
% 
% % 牽引紐中点位置 p_mid
% p_mid = pl(:) - 0.5 * L_val * pT(:);
% r_rel = p_mid - p_obs; % 障害物から中点への相対ベクトル
% 
% % 基本バリア関数 h0
% cbf_h0 = r_rel.' * r_rel - r_safe^2;
% 
% % -------------------------------------------------------------------------
% % 3. 中点 p_mid の運動学・力学連鎖の厳密構築
% % -------------------------------------------------------------------------
% disp(" -> 中点 p_mid の加速度・ジャーク・スナップ連鎖を展開中...");
% 
% dpT = Skew(ol) * pT(:);
% dp_mid = dpl(:) - 0.5 * L_val * dpT;
% 
% % 力学的結合係数
% Gamma_mid = (pT(:)*pT(:).') / (mQ_val + mL_val) - (eye(3) - pT(:)*pT(:).') / (2 * mQ_val);
% drift_acc = - (mQ_val * L_val * (dpT.' * dpT) / (mQ_val + mL_val)) * pT(:) - g_acc * e3 ...
%             - 0.5 * L_val * (Skew(ol)*dpT);
% 
% % 【加速度】最適化決定変数 u1_var を線形に含む
% d2p_mid_u = drift_acc + Gamma_mid * (R_mat * e3) * u1_var;
% % 【現在加速度】(高階微分内の自乗和汚染を防ぐため、公称推力 u1_nom で静的に評価)
% d2p_mid_eval = drift_acc + Gamma_mid * (R_mat * e3) * u1_nom;
% 
% % 【ジャーク】推力微分 du1 がドリフトとして入る
% d_Re3 = - R_mat * Skew(e3) * ob(:);
% d3p_mid = LieD(drift_acc, f, x) + Gamma_mid * (R_mat * e3) * du1 + Gamma_mid * d_Re3 * u1_nom;
% 
% % 【スナップ】トルク tau = [u2; u3; u4] が線形アフィンで現れる
% Gamma_tau = Gamma_mid * (- R_mat * Skew(e3)) * (JQ_mat \ eye(3)) * u1_nom;
% ob_dot_drift = - (JQ_mat \ (Skew(ob) * (JQ_mat * ob(:))));
% d_Re3_drift  = - R_mat * Skew(Skew(ob)*e3) * ob(:) - R_mat * Skew(e3) * ob_dot_drift;
% d4p_mid_drift = LieD(d3p_mid, f, x) + Gamma_mid * (R_mat * e3) * d2u1 + Gamma_mid * d_Re3_drift * u1_nom;
% 
% % 4階微分スナップ式 (トルクのみがアフィン決定変数)
% d4p_mid = d4p_mid_drift + Gamma_tau * tau_decision;
% 
% % -------------------------------------------------------------------------
% % 4. バリア関数の高階時間微分の厳密統合
% % -------------------------------------------------------------------------
% disp(" -> バリア関数 h の高階微分の統合...");
% 
% % 相対位置ベクトル (障害物 -> 中点)
% r_rel = p_mid - p_obs;
% 
% % 1階微分 (速度)
% dh0_dt  = 2 * (r_rel.' * dp_mid);
% 
% % 2階微分 (加速度: u1_var が出現)
% d2h0_dt = 2 * (dp_mid.' * dp_mid) + 2 * (r_rel.' * d2p_mid_u);
% 
% % 3階微分 (ジャーク: du1 を含むドリフト)
% d3h0_dt = 6 * (dp_mid.' * d2p_mid_eval) + 2 * (r_rel.' * d3p_mid);
% 
% % 4階微分 (スナップ: tau_decision が出現, 加速度自乗和は d2p_mid_eval で固定)
% d4h0_dt = 6 * (d2p_mid_eval.' * d2p_mid_eval) + 8 * (dp_mid.' * d3p_mid) + 2 * (r_rel.' * d4p_mid);
% 
% % Hurwitz 4次 ECBF 多項式
% cbf_psi4 = simplify( ...
%     d4h0_dt + ...
%     4 * lambda * d3h0_dt + ...
%     6 * lambda^2 * d2h0_dt + ...
%     4 * lambda^3 * dh0_dt + ...
%     lambda^4 * cbf_h0 ...
% );
% 
% % -------------------------------------------------------------------------
% % 5. 入力アフィン分離 (A_cbf * u <= b_cbf)
% % -------------------------------------------------------------------------
% disp(" -> 入力感度行列 (A_cbf, b_cbf) の厳密抽出...");
% 
% % 決定変数 u_decision = [u1_var; u2_var; u3_var; u4_var]
% J_u = jacobian(cbf_psi4, u_decision);
% J_u_clean   = simplify(subs(J_u, u_decision, [0; 0; 0; 0]));
% drift_clean = simplify(subs(cbf_psi4, u_decision, [0; 0; 0; 0]));
% 
% % 🌟 厳密な定義: psi_4 >= 0 <===> - J_u * u <= drift
% A_cbf_sym = - J_u_clean;
% b_cbf_sym = drift_clean;
% 
% % 監視用バリア値
% h0_eval = cbf_h0;
% h1_eval = dh0_dt;
% h2_eval = simplify(subs(d2h0_dt, u1_var, u1_nom));
% h3_eval = simplify(d3h0_dt);
% % -------------------------------------------------------------------------
% % 6. シンボル置換とクリーンアップ
% % -------------------------------------------------------------------------
% syms u1 u2 u3 u4 real
% raw_u = [u1; u2; u3; u4];
% 
% A_clean = subs(A_cbf_sym, raw_u, [u1_nom; 0; 0; 0]);
% b_clean = subs(b_cbf_sym, raw_u, [u1_nom; 0; 0; 0]);
% h0_clean = subs(h0_eval,  raw_u, [u1_nom; 0; 0; 0]);
% h1_clean = subs(h1_eval,  raw_u, [u1_nom; 0; 0; 0]);
% h2_clean = subs(h2_eval,  raw_u, [u1_nom; 0; 0; 0]);
% h3_clean = subs(h3_eval,  raw_u, [u1_nom; 0; 0; 0]);
% 
% A_cbf_subs = subs(A_clean, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h0_subs    = subs(h0_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h1_subs    = subs(h1_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h2_subs    = subs(h2_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h3_subs    = subs(h3_clean, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % -------------------------------------------------------------------------
% % 7. Mファイル関数としてエクスポート
% % -------------------------------------------------------------------------
% obs_params   = [xo; yo; zo; ro];
% cascade_vars = [du1; d2u1; u1_nom];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:);
% 
% target_vars = { ...
%     obj, ...
%     x, ...
%     XD_sym, ...
%     cascade_vars, ...   % [du1; d2u1; u1_nom]
%     obs_params, ...     % [xo; yo; zo; ro]
%     lambda, ...         % Hurwitz ゲイン
%     sys_params, ...     % rl
%     physicalParam ...
% };
% 
% all_expressions = [A_cbf_subs(:); b_cbf_subs; h0_subs; h1_subs; h2_subs; h3_subs];
% all_funvars = symvar(all_expressions);
% 
% declared_list = [];
% for k = 1:length(target_vars)
%     v_k = target_vars{k};
%     if isa(v_k, 'sym')
%         declared_list = [declared_list; v_k(:)]; %#ok<AGROW>
%     end
% end
% declared_vars = symvar(declared_list);
% residual_vars = setdiff(all_funvars, declared_vars);
% 
% if ~isempty(residual_vars)
%     warning('未定義シンボルを 0 に置換します: %s', char(residual_vars));
%     A_cbf_subs = subs(A_cbf_subs, residual_vars, zeros(size(residual_vars)));
%     b_cbf_subs = subs(b_cbf_subs, residual_vars, zeros(size(residual_vars)));
%     h0_subs    = subs(h0_subs, residual_vars, zeros(size(residual_vars)));
%     h1_subs    = subs(h1_subs, residual_vars, zeros(size(residual_vars)));
%     h2_subs    = subs(h2_subs, residual_vars, zeros(size(residual_vars)));
%     h3_subs    = subs(h3_subs, residual_vars, zeros(size(residual_vars)));
% end
% 
% export_fname = 'FastBridge_ECBF_SlungLoad_MidPoint';
% disp(['Exporting: ', export_fname, '.m を生成中...']);
% 
% matlabFunction( ...
%     A_cbf_subs, ...
%     b_cbf_subs, ...
%     h0_subs, ...
%     h1_subs, ...
%     h2_subs, ...
%     h3_subs, ...
%     'file', strcat(export_fname, '.m'), ...
%     'vars', target_vars, ...
%     'outputs', { ...
%         'A_cbf', ...
%         'b_cbf', ...
%         'h0', ...
%         'h1', ...
%         'h2', ...
%         'h3' ...
%     } ...
% );
% 
% DeleteCommentLine(export_fname);
% disp("==============================================================");
% disp(" Done: 幾何連鎖によるトルク感度確定版 ECBF の生成完了！");
% disp("==============================================================");
%% =========================================================================
%% 【FastBridge 準拠】幾何チェーンルールによる相対次数4 ECBF 厳密導出
%%  - p_mid の運動方程式連鎖から直接 \Omega と M の結合を抽出
%%  - [k0, k1, k2, k3] 独立ゲイン版
%% =========================================================================
disp("==============================================================");
disp(" Start: FastBridge 型 相対次数4 ECBF (独立ゲイン・幾何連鎖) 導出");
disp(" Target: A_cbf * u <= b_cbf");
disp("==============================================================");

% 1. シンボリック変数の定義
syms u1_var u2_var u3_var u4_var real
u_decision = [u1_var; u2_var; u3_var; u4_var];
tau_decision = [u2_var; u3_var; u4_var]; % 3軸トルク

syms u1_nom real                       % トルク結合用推力 (ホバリング/公称推力)
syms du1 d2u1 real                     % 推力1階・2階差分
syms xo yo zo ro rl real               % 障害物位置・半径

% 🌟 4つの独立ゲイン [k0:位置, k1:速度, k2:加速度, k3:ジャーク]
syms k0 k1 k2 k3 real
cbf_gains = [k0; k1; k2; k3];

% 物理パラメータの展開
mQ_val = physicalParam(1);
JQ_mat = diag([physicalParam(2), physicalParam(3), physicalParam(4)]);
g_acc  = physicalParam(5);
mL_val = physicalParam(6);
L_val  = physicalParam(7);
e3     = [0; 0; 1];

% 2. 状態の幾何表現
R_mat = RodriguesQuaternion(q); % 機体回転行列 (3x3)
p_obs = [xo; yo; zo];
r_safe = ro + rl;

% 牽引紐中点位置 p_mid
p_mid = pl(:) - 0.5 * L_val * pT(:);
r_rel = p_mid - p_obs; % 障害物から中点への相対ベクトル

% 基本バリア関数 h0
cbf_h0 = r_rel.' * r_rel - r_safe^2;

% -------------------------------------------------------------------------
% 3. 中点 p_mid の運動学・力学連鎖の厳密構築
% -------------------------------------------------------------------------
disp(" -> 中点 p_mid の加速度・ジャーク・スナップ連鎖を展開中...");

dpT = Skew(ol) * pT(:);
dp_mid = dpl(:) - 0.5 * L_val * dpT;

% 力学的結合係数
Gamma_mid = (pT(:)*pT(:).') / (mQ_val + mL_val) - (eye(3) - pT(:)*pT(:).') / (2 * mQ_val);
drift_acc = - (mQ_val * L_val * (dpT.' * dpT) / (mQ_val + mL_val)) * pT(:) - g_acc * e3 ...
            - 0.5 * L_val * (Skew(ol)*dpT);

% 【加速度】最適化決定変数 u1_var を線形に含む
d2p_mid_u = drift_acc + Gamma_mid * (R_mat * e3) * u1_var;

% 【現在加速度】(高階微分内の自乗和汚染を防ぐため、公称推力 u1_nom で静的に評価)
d2p_mid_eval = drift_acc + Gamma_mid * (R_mat * e3) * u1_nom;

% 【ジャーク】推力微分 du1 がドリフトとして入る
d_Re3 = - R_mat * Skew(e3) * ob(:);
d3p_mid = LieD(drift_acc, f, x) + Gamma_mid * (R_mat * e3) * du1 + Gamma_mid * d_Re3 * u1_nom;

% 【スナップ】トルク tau = [u2; u3; u4] が線形アフィンで現れる
Gamma_tau = Gamma_mid * (- R_mat * Skew(e3)) * (JQ_mat \ eye(3)) * u1_nom;
ob_dot_drift = - (JQ_mat \ (Skew(ob) * (JQ_mat * ob(:))));
d_Re3_drift  = - R_mat * Skew(Skew(ob)*e3) * ob(:) - R_mat * Skew(e3) * ob_dot_drift;
d4p_mid_drift = LieD(d3p_mid, f, x) + Gamma_mid * (R_mat * e3) * d2u1 + Gamma_mid * d_Re3_drift * u1_nom;

% 4階微分スナップ式
d4p_mid = d4p_mid_drift + Gamma_tau * tau_decision;

% -------------------------------------------------------------------------
% 4. バリア関数の高階時間微分の厳密統合 (独立ゲイン)
% -------------------------------------------------------------------------
disp(" -> バリア関数 h の高階微分の統合...");

% 相対位置ベクトル (障害物 -> 中点)
r_rel = p_mid - p_obs;

% 1階微分 (速度)
dh0_dt  = 2 * (r_rel.' * dp_mid);

% 2階微分 (加速度: u1_var が出現)
d2h0_dt = 2 * (dp_mid.' * dp_mid) + 2 * (r_rel.' * d2p_mid_u);

% 3階微分 (ジャーク: du1 を含むドリフト)
d3h0_dt = 6 * (dp_mid.' * d2p_mid_eval) + 2 * (r_rel.' * d3p_mid);

% 4階微分 (スナップ: tau_decision が出現, 加速度自乗和は d2p_mid_eval で固定)
d4h0_dt = 6 * (d2p_mid_eval.' * d2p_mid_eval) + 8 * (dp_mid.' * d3p_mid) + 2 * (r_rel.' * d4p_mid);

% 🌟 独立ゲイン版 4次 ECBF 多項式
cbf_psi4 = simplify( ...
    d4h0_dt + ...
    k3 * d3h0_dt + ...
    k2 * d2h0_dt + ...
    k1 * dh0_dt + ...
    k0 * cbf_h0 ...
);

% -------------------------------------------------------------------------
% 5. 入力アフィン分離 (A_cbf * u <= b_cbf)
% -------------------------------------------------------------------------
disp(" -> 入力感度行列 (A_cbf, b_cbf) の厳密抽出...");

J_u = jacobian(cbf_psi4, u_decision);
J_u_clean   = simplify(subs(J_u, u_decision, [0; 0; 0; 0]));
drift_clean = simplify(subs(cbf_psi4, u_decision, [0; 0; 0; 0]));

% 🌟 整合定義: psi_4 >= 0 <===> A_cbf * u <= b_cbf
A_cbf_sym = J_u_clean;
b_cbf_sym = - drift_clean;

% 監視用バリア値
h0_eval = cbf_h0;
h1_eval = dh0_dt;
h2_eval = simplify(subs(d2h0_dt, u1_var, u1_nom));
h3_eval = simplify(d3h0_dt);

% -------------------------------------------------------------------------
% 6. シンボル置換とクリーンアップ
% -------------------------------------------------------------------------
syms u1 u2 u3 u4 real
raw_u = [u1; u2; u3; u4];

A_clean = subs(A_cbf_sym, raw_u, [u1_nom; 0; 0; 0]);
b_clean = subs(b_cbf_sym, raw_u, [u1_nom; 0; 0; 0]);
h0_clean = subs(h0_eval,  raw_u, [u1_nom; 0; 0; 0]);
h1_clean = subs(h1_eval,  raw_u, [u1_nom; 0; 0; 0]);
h2_clean = subs(h2_eval,  raw_u, [u1_nom; 0; 0; 0]);
h3_clean = subs(h3_eval,  raw_u, [u1_nom; 0; 0; 0]);

A_cbf_subs = subs(A_clean, [xdReff, vInput1f], [XDf, V1vf]);
b_cbf_subs = subs(b_clean, [xdReff, vInput1f], [XDf, V1vf]);
h0_subs    = subs(h0_clean, [xdReff, vInput1f], [XDf, V1vf]);
h1_subs    = subs(h1_clean, [xdReff, vInput1f], [XDf, V1vf]);
h2_subs    = subs(h2_clean, [xdReff, vInput1f], [XDf, V1vf]);
h3_subs    = subs(h3_clean, [xdReff, vInput1f], [XDf, V1vf]);

% -------------------------------------------------------------------------
% 7. Mファイル関数としてエクスポート
% -------------------------------------------------------------------------
obs_params   = [xo; yo; zo; ro];
cascade_vars = [du1; d2u1; u1_nom];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:);

target_vars = { ...
    obj, ...
    x, ...
    XD_sym, ...
    cascade_vars, ...   % [du1; d2u1; u1_nom]
    obs_params, ...     % [xo; yo; zo; ro]
    cbf_gains, ...      % 🌟 [k0; k1; k2; k3] (独立ゲインベクトル)
    sys_params, ...     % rl
    physicalParam ...
};

all_expressions = [A_cbf_subs(:); b_cbf_subs; h0_subs; h1_subs; h2_subs; h3_subs];
all_funvars = symvar(all_expressions);

declared_list = [];
for k = 1:length(target_vars)
    v_k = target_vars{k};
    if isa(v_k, 'sym')
        declared_list = [declared_list; v_k(:)]; %#ok<AGROW>
    end
end
declared_vars = symvar(declared_list);
residual_vars = setdiff(all_funvars, declared_vars);

if ~isempty(residual_vars)
    warning('未定義シンボルを 0 に置換します: %s', char(residual_vars));
    A_cbf_subs = subs(A_cbf_subs, residual_vars, zeros(size(residual_vars)));
    b_cbf_subs = subs(b_cbf_subs, residual_vars, zeros(size(residual_vars)));
    h0_subs    = subs(h0_subs, residual_vars, zeros(size(residual_vars)));
    h1_subs    = subs(h1_subs, residual_vars, zeros(size(residual_vars)));
    h2_subs    = subs(h2_subs, residual_vars, zeros(size(residual_vars)));
    h3_subs    = subs(h3_subs, residual_vars, zeros(size(residual_vars)));
end

export_fname = 'FastBridge_ECBF_SlungLoad_MidPoint';
disp(['Exporting: ', export_fname, '.m を生成中...']);

matlabFunction( ...
    A_cbf_subs, ...
    b_cbf_subs, ...
    h0_subs, ...
    h1_subs, ...
    h2_subs, ...
    h3_subs, ...
    'file', strcat(export_fname, '.m'), ...
    'vars', target_vars, ...
    'outputs', { ...
        'A_cbf', ...
        'b_cbf', ...
        'h0', ...
        'h1', ...
        'h2', ...
        'h3' ...
    } ...
);

DeleteCommentLine(export_fname);
disp("==============================================================");
disp(" Done: 独立ゲイン対応 FastBridge ECBF 関数の生成完了！");
disp("==============================================================");
%% =========================================================================
%% 【厳密検証テスト】FastBridge ECBF 感度・符号・数値誤差解析スクリプト
%%  1. 各入力 [u1:推力, u2:Roll, u3:Pitch, u4:Yaw] の感度存在テスト (ゼロ検知)
%%  2. 障害物食い込み時・接近時の不等式制約の符号代数テスト (A*u <= b の整合性)
%%  3. 仮想的な回避方向への入力に対する応答テスト
%% =========================================================================
disp(" ");
disp("==============================================================");
disp(" [Verification Test] 生成関数の厳密検証テストを開始します");
disp("==============================================================");

% -------------------------------------------------------------------------
% 1. シンボリック感度勾配 J_u の非ゼロ判定
% -------------------------------------------------------------------------
disp("--- 1. シンボリック感度 J_u の構造解析 (各入力への結合確認) ---");
has_u1 = ~isequal(simplify(J_u_clean(1)), sym(0));
has_u2 = ~isequal(simplify(J_u_clean(2)), sym(0));
has_u3 = ~isequal(simplify(J_u_clean(3)), sym(0));
has_u4 = ~isequal(simplify(J_u_clean(4)), sym(0));

fprintf('  d(psi)/du1 (推力 f_T 感度)     : %s\n', mat2str(has_u1));
fprintf('  d(psi)/du2 (Rollトルク Mx 感度): %s\n', mat2str(has_u2));
fprintf('  d(psi)/du3 (Pitchトルク My 感度): %s\n', mat2str(has_u3));
fprintf('  d(psi)/du4 (Yawトルク Mz 感度) : %s\n', mat2str(has_u4));

if ~(has_u1 && has_u2 && has_u3)
    error('【重大エラー】推力またはRoll/Pitchトルクの感度が0です！導出ロジックを確認してください。');
else
    disp('  [PASS] 推力および水平トルク(Roll/Pitch)への感度結合を確認しました。');
end

% -------------------------------------------------------------------------
% 2. テスト環境の構築と数値テスト
% -------------------------------------------------------------------------
disp(" ");
disp("--- 2. 数値テスト環境による代数符号＆感度の評価 ---");

% テストパラメータの設定
if exist('physicalParam_num', 'var')
    P_val = physicalParam_num;
elseif exist('param', 'var') && isfield(param, 'physical')
    P_val = param.physical;
else
    % [m, jx, jy, jz, gravity, mL, cableL, dstx, dsty, dstz]
    P_val = [1.5, 0.039, 0.051, 0.102, 9.81, 0.5, 0.8, 0.0, 0.0, 0.0];
end

m_total = P_val(1) + P_val(6); % mQ + mL
f_hover = m_total * P_val(5);   % ホバリング推力
u_nom_test = [f_hover; 0.0; 0.0; 0.0];
cascade_test = [0.0; 0.0; f_hover]; % [du1; d2u1; u1_nom]
lambda_test = 1.2;
rl_test = 0.5;

% ダミーの目標値軌道ベクトル (28次元以上)
XD_test = zeros(60, 1);

% テスト状態: 荷物が原点、紐が鉛直下向き (機体は真上)、静止状態
x_safe = zeros(19, 1);
x_safe(1:4)   = [1; 0; 0; 0];       % q (水平姿勢)
x_safe(5:7)   = [0; 0; 0];          % w (角速度ゼロ)
x_safe(8:10)  = [0; 0; 1.0];        % pL (荷物位置)
x_safe(11:13) = [0; 0; 0];          % vL (速度ゼロ)
x_safe(14:16) = [0; 0; -1.0];       % pT (紐単位ベクトル: 真下)
x_safe(17:19) = [0; 0; 0];          % wL (紐角速度ゼロ)

% 中点位置 p_mid の手計算確認: pL - 0.5*L*pT = [0; 0; 1.0] - 0.5*0.8*[0; 0; -1] = [0; 0; 1.4]
p_mid_expected = x_safe(8:10) - 0.5 * P_val(7) * x_safe(14:16);

% 障害物設定: Y軸上の前方 [0; 2.0; 1.4]、半径 0.5m
p_obs_test = [0.0; 2.0; p_mid_expected(3)];
r_obs_test = 0.5;
obs_param_test = [p_obs_test; r_obs_test];

% -------------------------------------------------------------------------
% [Case A] 安全圏（離れている状態）でのテスト
% -------------------------------------------------------------------------
[A_safe, b_safe, h0_s, h1_s, h2_s, h3_s] = FastBridge_ECBF_SlungLoad_MidPoint(...
    obj, x_safe, XD_test, cascade_test, obs_param_test, lambda_test, rl_test, P_val);

A_safe = double(real(A_safe));
b_safe = double(real(b_safe));

% 制約違反量: viol = A*u - b (正なら違反、負なら安全)
viol_safe = A_safe * u_nom_test - b_safe;

fprintf('  [Case A: 安全圏 (距離 2.0m)]\n');
fprintf('    h0 (距離2乗マージン): %+8.4f (正であるべき)\n', double(h0_s));
fprintf('    A_cbf (感度 1x4)   : [%+9.3e, %+9.3e, %+9.3e, %+9.3e]\n', A_safe);
fprintf('    b_cbf (ドリフト)   : %+9.3e\n', b_safe);
fprintf('    A*u_nom - b        : %+9.3e (負であるべき: 安全)\n', viol_safe);

% -------------------------------------------------------------------------
% [Case B] 衝突・食い込み状態でのテスト (障害物を中点の至近距離へ配置)
% -------------------------------------------------------------------------
% 障害物を中点とほぼ同位置 [0; 0.1; 1.4] に配置 (完全に食い込み)
p_obs_crash = [0.0; 0.1; p_mid_expected(3)];
obs_param_crash = [p_obs_crash; r_obs_test];

[A_crash, b_crash, h0_c, h1_c, h2_c, h3_c] = FastBridge_ECBF_SlungLoad_MidPoint(...
    obj, x_safe, XD_test, cascade_test, obs_param_crash, lambda_test, rl_test, P_val);

A_crash = double(real(A_crash));
b_crash = double(real(b_crash));
viol_crash = A_crash * u_nom_test - b_crash;

fprintf('  [Case B: 衝突・食い込み状態 (距離 0.1m)]\n');
fprintf('    h0 (距離2乗マージン): %+8.4f (負であるべき)\n', double(h0_c));
fprintf('    A_cbf (感度 1x4)   : [%+9.3e, %+9.3e, %+9.3e, %+9.3e]\n', A_crash);
fprintf('    b_cbf (ドリフト)   : %+9.3e\n', b_crash);
fprintf('    A*u_nom - b        : %+9.3e (正であるべき: 制約違反)\n', viol_crash);

% -------------------------------------------------------------------------
% 3. 符号整合性の判定
% -------------------------------------------------------------------------
disp(" ");
disp("--- 3. 符号整合性の自動判定結果 ---");
pass_sign = true;

if h0_s <= 0 || viol_safe > 0
    warning('【符号異常】安全圏なのに制約違反と判定されている、または h0 <= 0 です。');
    pass_sign = false;
end

if h0_c >= 0 || viol_crash <= 0
    warning('【符号異常】衝突しているのに A*u - b <= 0 (安全) と誤判定されています！符号が逆転しています。');
    pass_sign = false;
end

if pass_sign
    disp('  [PASS] 符号の整合性を確認: 衝突時に確実に A*u - b > 0 (違反) となり、QP が介入します。');
else
    disp('  [FAIL] 符号の定義に不整合があります。導出部の正負を見直してください。');
end
disp("==============================================================");
% % 4. 微分展開
% % -------------------------------------------------------------------------
% % --- [A] 機体姿勢 (相対次数 2) ---
% % 1階微分 (角速度レベル)
% L_f_h_att0 = LieD(h_att_0, f, x);
% psi_att_1  = simplify(L_f_h_att0 + gamma_att1 * h_att_0);
% 
% % 2階微分 (角加速度・トルクレベル)
% L_f_psi_att1 = LieD(psi_att_1, f, x);
% L_g_psi_att1 = LieD(psi_att_1, g_all, x); % 1x4 (u2, u3 が支配的)
% 
% Total_att = simplify(L_f_psi_att1 + L_g_psi_att1 * u_vec + gamma_att2 * psi_att_1);
% 
% % --- [B] 紐傾斜角 (相対次数 2) ---
% % 1階微分 (紐角速度レベル)
% L_f_h_cb0 = LieD(h_cable_0, f, x);
% psi_cb_1  = simplify(L_f_h_cb0 + gamma_cb1 * h_cable_0);
% 
% % 2階微分 (紐加速度・推力結合レベル)
% L_f_psi_cb1 = LieD(psi_cb_1, f, x);
% L_g_psi_cb1 = LieD(psi_cb_1, g_all, x); % 1x4 (u1 が出現)
% 
% Total_cable = simplify(L_f_psi_cb1 + L_g_psi_cb1 * u_vec + gamma_cb2 * psi_cb_1);
% 
% % 5. 動作点 u0 周りでの線形アフィン化
% % -------------------------------------------------------------------------
% % 姿勢角制約: A_att * u <= b_att
% grad_att = jacobian(Total_att, u_vec);
% J_att_u0 = simplify(subs(grad_att, u_vec, u0_vec));
% F_att_u0 = simplify(subs(Total_att, u_vec, u0_vec));
% A_att_sym = - J_att_u0;
% b_att_sym = simplify(F_att_u0 - J_att_u0 * u0_vec);
% 
% % 紐傾斜角制約: A_cable * u <= b_cable
% grad_cb = jacobian(Total_cable, u_vec);
% J_cb_u0 = simplify(subs(grad_cb, u_vec, u0_vec));
% F_cb_u0 = simplify(subs(Total_cable, u_vec, u0_vec));
% A_cb_sym = - J_cb_u0;
% b_cb_sym = simplify(F_cb_u0 - J_cb_u0 * u0_vec);
% 
% % 6. 目標軌道・中間変数の置換
% A_att_subs = subs(A_att_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_att_subs = subs(b_att_sym, [xdReff, vInput1f], [XDf, V1vf]);
% h_att_subs = subs(h_att_0, [xdReff, vInput1f], [XDf, V1vf]);
% 
% A_cb_subs  = subs(A_cb_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cb_subs  = subs(b_cb_sym, [xdReff, vInput1f], [XDf, V1vf]);
% h_cb_subs  = subs(h_cable_0, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 7. Mファイル関数としてエクスポート
% angle_limits = [theta_att_max; theta_cable_max];
% gamma_angles = [gamma_att1; gamma_att2; gamma_cb1; gamma_cb2];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:);
% 
% export_fname = 'CBF_Constraints_Attitude_Cable_Taylor';
% disp(['Exporting: ', export_fname, '.m を書き出しています...']);
% 
% matlabFunction( ...
%     A_att_subs, ...
%     b_att_subs, ...
%     h_att_subs, ...
%     A_cb_subs, ...
%     b_cb_subs, ...
%     h_cb_subs, ...
%     'file', strcat(export_fname, '.m'), ...
%     'vars', { ...
%         obj, ...
%         x, ...
%         XD_sym, ...
%         u0_vec, ...
%         angle_limits, ...
%         gamma_angles, ...
%         sys_params, ...
%         physicalParam ...
%     }, ...
%     'outputs', { ...
%         'A_att', ...
%         'b_att', ...
%         'h_att', ...
%         'A_cable', ...
%         'b_cable', ...
%         'h_cable' ...
%     } ...
% );
% 
% DeleteCommentLine(export_fname);
% disp(" Done: 姿勢角 & 紐傾斜角 HOCBF 関数の生成が完了しました！");
% disp("==============================================================");
% %% =========================================================================
% %% 【検証用】姿勢角 & 紐傾斜角 HOCBF の構造・符号・感度解析
% %% =========================================================================
% disp("==============================================================");
% disp(" [Verification] 姿勢角 & 紐傾斜角 HOCBF の厳密性チェック");
% disp("==============================================================");
% 
% % 1. 各入力に対する感度構造のチェック
% disp("--- 1. 感度勾配の構造解析 ---");
% fprintf('  [姿勢角] dF/du1(推力): %s, dF/du2(Mx): %s, dF/du3(My): %s, dF/du4(Mz): %s\n', ...
%     mat2str(~isequal(simplify(grad_att(1)), sym(0))), ...
%     mat2str(~isequal(simplify(grad_att(2)), sym(0))), ...
%     mat2str(~isequal(simplify(grad_att(3)), sym(0))), ...
%     mat2str(~isequal(simplify(grad_att(4)), sym(0))));
% 
% fprintf('  [紐角度] dF/du1(推力): %s, dF/du2(Mx): %s, dF/du3(My): %s, dF/du4(Mz): %s\n', ...
%     mat2str(~isequal(simplify(grad_cb(1)), sym(0))), ...
%     mat2str(~isequal(simplify(grad_cb(2)), sym(0))), ...
%     mat2str(~isequal(simplify(grad_cb(3)), sym(0))), ...
%     mat2str(~isequal(simplify(grad_cb(4)), sym(0))));
% 
% % 2. 符号代数チェック
% disp(" ");
% disp("--- 2. A_qp, b_qp の符号整合性チェック ---");
% pass_att = isequal(simplify(expand((b_att_sym - A_att_sym * u_vec) - (F_att_u0 + J_att_u0 * (u_vec - u0_vec)))), sym(0));
% pass_cb  = isequal(simplify(expand((b_cb_sym - A_cb_sym * u_vec) - (F_cb_u0 + J_cb_u0 * (u_vec - u0_vec)))), sym(0));
% 
% if pass_att && pass_cb
%     disp("  [PASS] 姿勢角・紐角度ともに符号の一致を確認しました (A*u <= b <==> F_lin >= 0)");
% else
%     disp("  [FAIL] 代数式に不一致があります！");
% end
% 
% % 3. 数値テスト実行
% disp(" ");
% disp("--- 3. 数値環境での感度値テスト ---");
% x_test = zeros(19, 1);
% x_test(1:3)   = [0.0; 0.0; 1.5];   % 荷物位置
% x_test(7:9)   = [0.0; 0.0; -1.0];  % 紐真下
% x_test(10:12) = [0.0; 0.0; 0.0];   % 姿勢水平 (q = [1;0;0;0])
% xd_test = zeros(60, 1);
% 
% if exist('param', 'var') && isfield(param, 'physical')
%     P_test = param.physical;
% else
%     P_test = [1.5, 0.039, 0.051, 0.102, 9.81, 0.5, 0.8, 0.0, 0.0, 0.0];
% end
% 
% f_hov = (P_test(1) + P_test(6)) * P_test(5);
% u0_test = [f_hov; 0.0; 0.0; 0.0];
% 
% ang_limits_test = [deg2rad(30); deg2rad(20)]; % 機体30度, 紐20度
% gamma_ang_test  = [5.0; 10.0; 5.0; 10.0];
% 
% [A_at_v, b_at_v, h_at_v, A_cb_v, b_cb_v, h_cb_v] = CBF_Constraints_Attitude_Cable_Taylor(...
%     obj, x_test, xd_test, u0_test, ang_limits_test, gamma_ang_test, 0.05, P_test);
% 
% fprintf('  [姿勢角] h_att0: %+6.3f | A_att (1x4): [%+7.2f, %+7.2f, %+7.2f, %+7.2f] | b_att: %+7.2f\n', ...
%     h_at_v, A_at_v(1), A_at_v(2), A_at_v(3), A_at_v(4), b_at_v);
% fprintf('  [紐角度] h_cb0 : %+6.3f | A_cb  (1x4): [%+7.2f, %+7.2f, %+7.2f, %+7.2f] | b_cb : %+7.2f\n', ...
%     h_cb_v, A_cb_v(1), A_cb_v(2), A_cb_v(3), A_cb_v(4), b_cb_v);
% disp("==============================================================");
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
