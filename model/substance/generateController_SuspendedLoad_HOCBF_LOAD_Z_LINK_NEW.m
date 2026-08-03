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
% %% 【追加】高次制御バリア関数z方向 (HOCBF) の自動導出と関数エクスポート
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% FG_z = simplify(f+g*[u1;u2;u3;u4]);
% g_z = simplify(MyCoeff(FG_z,[u1;u2;u3;u4]));
% f_z = subs(FG_z,[u1,u2,u3,u4],[0,0,0,0]);
% simplify(FG_z-(f_z+g_z*[u1;u2;u3;u4]))
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms rl real
% % 荷物の位置 pl = [pl1; pl2; pl3] と障害物の距離の2乗から安全を定義
% h_cbf_z = (pl(1) - xo)^2 + (pl(2) - yo)^2 + (pl(3) - zo)^2 - (ro + rl)^2;
% 
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 real
% g1_z = g_z(:, 1);% ★ $u_1$（推力）に掛かる列を抽出
% 
% % 3. 2階の高次CBFの導出（相対次数2）
% % cbf1 (h)   >= 0 
% % cbf2 (dot_h + gamma1 * h) >= 0
% cbf2 = LieD(h_cbf_z, f_z, x) + diff(h_cbf_z, t) + gamma1 * h_cbf_z; %h2
% 
% 
% % 4. 最適化問題（QP）へのマッピング
% % cbf2_dot + gamma2 * cbf2 >= 0  を導出
% % ここで L_g_cbf2 * u1 + L_f_cbf2 + gamma2 * cbf2 >= 0
% L_f_cbf2  = LieD(cbf2, f_z, x) + diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g1_z, x);  % [1 x 1] のシンボリックスカラー（u1 の係数）
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf2; 
% b_cbf_sym = L_f_cbf2 + gamma2 * cbf2;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params = [gamma1; gamma2];
% obs_params = [xo; yo; zo; ro];
% sys_params = rl;
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_z.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_z.m', ...
%     'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【追加】高次制御バリア関数z方向 (HOCBF) の自動導出と関数エクスポート
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% FG_z = simplify(f+g*[u1;u2;u3;u4]);
% g_z = simplify(MyCoeff(FG_z,[u1;u2;u3;u4]));
% f_z = subs(FG_z,[u1,u2,u3,u4],[0,0,0,0]);
% simplify(FG_z-(f_z+g_z*[u1;u2;u3;u4]))
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms rl real
% 
% % physicalParam からケーブル長 L を参照
% L_cable = physicalParam(7);
% 
% % 🌟 【核心部】 安全関数の基準位置を「ロープ（リンク）の中心点 p_mid」に設定
% p_mid = pl - 0.5 * L_cable * pT;
% 
% % 荷物の位置 pl = [pl1; pl2; pl3] と障害物の距離の2乗から安全を定義
% h_cbf_z = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% cbf1 = h_cbf_z;
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 real
% g1_z = g_z(:, 1);% ★ $u_1$（推力）に掛かる列を抽出
% 
% % 3. 2階の高次CBFの導出（相対次数2）
% % cbf1 (h)   >= 0 
% % cbf2 (dot_h + gamma1 * h) >= 0
% cbf2 = LieD(cbf1, f_z, x) + diff(cbf1, t) + gamma1 * cbf1; %h2
% 
% 
% % 4. 最適化問題（QP）へのマッピング
% % cbf2_dot + gamma2 * cbf2 >= 0  を導出
% % ここで L_g_cbf2 * u1 + L_f_cbf2 + gamma2 * cbf2 >= 0
% L_f_cbf2  = LieD(cbf2, f_z, x) + diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g1_z, x);  % [1 x 1] のシンボリックスカラー（u1 の係数）
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf2; 
% b_cbf_sym = L_f_cbf2 + gamma2 * cbf2;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params = [gamma1; gamma2];
% obs_params = [xo; yo; zo; ro];
% sys_params = rl;
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_zlink.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_zlink.m', ...
%     'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【追加】高次制御バリア関数z方向 (HOCBF) の自動導出と関数エクスポート z方向のみの制約での回避
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% FG_z = simplify(f+g*[u1;u2;u3;u4]);
% g_z = simplify(MyCoeff(FG_z,[u1;u2;u3;u4]));
% f_z = subs(FG_z,[u1,u2,u3,u4],[0,0,0,0]);
% simplify(FG_z-(f_z+g_z*[u1;u2;u3;u4]))
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms rl real
% 
% % physicalParam からケーブル長 L を参照
% L_cable = physicalParam(7);
% 
% % 🌟 【核心部】 安全関数の基準位置を「ロープ（リンク）の中心点 p_mid」に設定
% p_mid = pl - 0.5 * L_cable * pT;
% 
% R_safe = ro + rl; % 表面までの安全距離（合計半径）
% 
% % 🌟 【核心部】 xy 水平距離の評価
% dist_xy_sq = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2;
% 
% % 後から xy を動かして離れた時に「フライングで高度を落として球の斜めに激突する」のを防ぐため、
% % xy の有効領域を少し広め (k_xy = 1.5〜2.0) に見なした重み付け多項式にする
% k_xy = 0.2; % xy離脱速度と高度復帰のバランス係数
% 
% % HOCBF 安全関数 (中心距離ベースの拡張形)
% % (pz - zo)^2 + k_xy * d_xy^2 >= R_safe^2
% % これにより、現在地が zo より上なら「上へ」、下なら「下へ」自律退避します
% h_cbf_z = (p_mid(3) - zo)^2 + k_xy * dist_xy_sq - R_safe^2;
% cbf1 = h_cbf_z;
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 real
% g1_z = g_z(:, 1);% ★ $u_1$（推力）に掛かる列を抽出
% 
% % 3. 2階の高次CBFの導出（相対次数2）
% % cbf1 (h)   >= 0 
% % cbf2 (dot_h + gamma1 * h) >= 0
% cbf2 = LieD(cbf1, f_z, x) + diff(cbf1, t) + gamma1 * cbf1; %h2
% 
% 
% % 4. 最適化問題（QP）へのマッピング
% % cbf2_dot + gamma2 * cbf2 >= 0  を導出
% % ここで L_g_cbf2 * u1 + L_f_cbf2 + gamma2 * cbf2 >= 0
% L_f_cbf2  = LieD(cbf2, f_z, x) + diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g1_z, x);  % [1 x 1] のシンボリックスカラー（u1 の係数）
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf2; 
% b_cbf_sym = L_f_cbf2 + gamma2 * cbf2;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params = [gamma1; gamma2];
% obs_params = [xo; yo; zo; ro];
% sys_params = rl;
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_zlink2.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_zlink2.m', ...
%     'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【追加】高次制御バリア関数z方向 (HOCBF) の自動導出と関数エクスポート　楕円 回避は仕様とするが、もっと、制約に工夫が必要
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% FG_z = simplify(f+g*[u1;u2;u3;u4]);
% g_z = simplify(MyCoeff(FG_z,[u1;u2;u3;u4]));
% f_z = subs(FG_z,[u1,u2,u3,u4],[0,0,0,0]);
% simplify(FG_z-(f_z+g_z*[u1;u2;u3;u4]))
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms a_sys b_sys real       % 機体側の楕円長軸・短軸半径 (a, b=c)
% 
% % physicalParam からケーブル長 L を参照
% L_cable = physicalParam(7);
% 
% % 🌟 【核心部】 安全関数の基準位置を「ロープ（リンク）の中心点 p_mid」に設定
% p_mid = pl - 0.5 * L_cable * pT;
% 
% p_obs = [xo; yo; zo];            % 障害物中心 (3x1)
% 
% % 🌟 3. 機体（またはシステム）の姿勢クオータニオン q_sys = [q0, q1, q2, q3]
% %  x の中のクオータニオン変数（例: q0=x(1), q1=x(2)... など）をそのまま使用
% %  ※ お使いのモデルのクオータニオン変数に合わせて設定してください
% q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
% 
% % クオータニオンから機体の回転行列 R_sys(q) を作成
% R_sys = [1 - 2*(q2^2 + q3^2),  2*(q1*q2 - q0*q3),  2*(q1*q3 + q0*q2);
%          2*(q1*q2 + q0*q3),  1 - 2*(q1^2 + q3^2),  2*(q2*q3 - q0*q1);
%          2*(q1*q3 - q0*q2),  2*(q2*q3 + q0*q1),  1 - 2*(q1^2 + q2^2)];
% 
% % 4. 障害物半径 ro を足し込んだ拡張対角行列 D
% a_eff = a_sys + ro;
% b_eff = b_sys + ro;
% 
% D = [1/(a_eff^2),          0,          0;
%                0, 1/(b_eff^2),          0;
%                0,          0, 1/(b_eff^2)];
% 
% % 5. ワールド座標系における機体楕円形状行列 M
% M = R_sys * D * R_sys.';
% 
% % 6. 🌟 楕円体機体用 安全関数 h_cbf_z の定義
% % (p_mid - p_obs)^T * M * (p_mid - p_obs) >= 1
% diff_p = p_mid - p_obs;
% h_cbf_z = diff_p.' * M * diff_p - 1.0;
% cbf1 = h_cbf_z;
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 real
% g1_z = g_z(:, 1);% ★ $u_1$（推力）に掛かる列を抽出
% 
% % 3. 2階の高次CBFの導出（相対次数2）
% % cbf1 (h)   >= 0 
% % cbf2 (dot_h + gamma1 * h) >= 0
% cbf2 = LieD(cbf1, f_z, x) + diff(cbf1, t) + gamma1 * cbf1; %h2
% 
% 
% % 4. 最適化問題（QP）へのマッピング
% % cbf2_dot + gamma2 * cbf2 >= 0  を導出
% % ここで L_g_cbf2 * u1 + L_f_cbf2 + gamma2 * cbf2 >= 0
% L_f_cbf2  = LieD(cbf2, f_z, x) + diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g1_z, x);  % [1 x 1] のシンボリックスカラー（u1 の係数）
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf2; 
% b_cbf_sym = L_f_cbf2 + gamma2 * cbf2;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params = [gamma1; gamma2];
% obs_params = [xo; yo; zo; ro];
% sys_params   = [a_sys; b_sys];
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_zlink3.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_zlink3.m', ...
%     'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
%% =========================================================================
%% 【改修版】マルチセグメント球体モデルによる HOCBF (z方向) 自動導出
%% =========================================================================
disp("Start: マルチセグメント球体 HOCBF(z方向) の導出を開始します。");

% 1. 物理モデル・運動方程式の分解
FG_z = simplify(f + g * [u1; u2; u3; u4]);
g_z  = simplify(MyCoeff(FG_z, [u1; u2; u3; u4]));
f_z  = subs(FG_z, [u1, u2, u3, u4], [0, 0, 0, 0]);
g1_z = g_z(:, 1); % u1 (推力) に関する入力行列

% 2. シンボリック変数の定義
syms xo yo zo ro real            % 障害物 (中心 [xo, yo, zo], 半径 ro)
syms gamma1 gamma2 real          % CBF ゲイン
syms lambda_j rl_j real          % システム側球体パラメータ (比率 lambda_j, 半径 rl_j)

% physicalParam からケーブル長 L を参照
L_cable = physicalParam(7);

% 3. 数式定義（第 j 球体の中心位置 x_c^j と CBF h^{ij}）
% pl: 荷物位置 (3x1), pT: ケーブル姿勢単位ベクトル (3x1)
p_obs = [xo; yo; zo];
x_c_j = pl - lambda_j * L_cable * pT; % x_c^j = x_L - lambda^j * L * p

% 🌟 【ご指定の定式化】 障害物回避制約
% h_cbf_z = (x_c(1) - xo)^2 + (x_c(2) - yo)^2 + (x_c(3) - zo)^2 - (ro + rl)^2
cbf1_single = (x_c_j(1) - xo)^2 + (x_c_j(2) - yo)^2 + (x_c_j(3) - zo)^2 - (ro + rl_j)^2;

% 4. 単一球体に対する 2階 HOCBF の導出（相対次数2）
% cbf2 = dot_cbf1 + gamma1 * cbf1
dot_cbf1 = LieD(cbf1_single, f_z, x) + diff(cbf1_single, t);
cbf2_single = dot_cbf1 + gamma1 * cbf1_single;

% L_f_cbf2 + L_g_cbf2 * u1 + gamma2 * cbf2 >= 0
L_f_cbf2_single = LieD(cbf2_single, f_z, x) + diff(cbf2_single, t);
L_g_cbf2_single = LieD(cbf2_single, g1_z, x); % [1 x 1] のシンボリックスカラー

% A_qp * u1 <= b_qp の形に変形（符号反転）
A_cbf_sym = -L_g_cbf2_single;
b_cbf_sym = L_f_cbf2_single + gamma2 * cbf2_single;

% 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);

% 6. Mファイルとして関数エクスポート
gamma_params  = [gamma1; gamma2];
obs_params    = [xo; yo; zo; ro];
sphere_params = [lambda_j; rl_j]; % [球体位置比率; システム側球体半径]

XD_sym = cell2sym(XD);
XD_sym = XD_sym(:); % 縦ベクトル化

disp("Exporting: CBF_Constraints_zlink4.m を書き出しています...");
matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_zlink4.m', ...
    'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, sphere_params, gamma_params, physicalParam}, ...
    'outputs', {'A_qp', 'b_qp'});
disp("Done: マルチセグメント球体 CBF関数の生成が完了しました！");
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
