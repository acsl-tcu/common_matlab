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
% %% HO-CBF (高次制御バリア関数) の自動導出セクション 【最上流レイヤー同調・完全復元版・修正版】
% %% =========================================================================
% clc
% disp('========================================================================');
% disp(' 最上流仮想入力 [v1; u2; u3] に完全同期したHO-CBF関数の生成を開始します');
% disp('========================================================================');
% 
% % 🌟 1. 未知の外乱(dstx, dsty)を0とした「公称全システムダイナミクス」の抽出
% f_nominal = subs(f, [dstx, dsty], [0, 0]); % FLxyDstから作成した状態空間方程式のドリフト項 0,0としているため外乱無し
% g_nominal = g; % 入力ゲイン行列
% 
% % 🌟 2. 最上流の操作変数 [v1_cmd; u2_cmd; u3_cmd] の定義　z,x,yの仮想入力の定義
% syms v1_cmd u2_cmd u3_cmd real
% u_top_layer = [v1_cmd; u2_cmd; u3_cmd];
% 
% % 各階層のCBFチェーンゲインの定義
% syms k_cbf1 k_cbf2 k_cbf3 k_cbf4 k_cbf5 real % 荷物用(相対次数5)
% syms d1 d2 real                             % ドローン用(相対次数2)
% syms ox oy oz ro real                       % 障害物情報
% syms r_load r_drone real                    % 安全マージン
% 
% % 高度線形化行列 H を適用した、最上流入力同期型の公称閉ループダイナミクスを全プロセスで使用
% % (第4入力はyaw用のダミーとして0をパッキング)
% FG_top_drone = simplify(f_nominal + g_nominal * H * [v1_cmd; u2_cmd; u3_cmd; 0]);
% 
% %% -----------------------------------------------------------------
% %% 1. ドローン本体のCBF (相対次数 2)
% %% -----------------------------------------------------------------
% disp('▶️ [Drone Layer] ドローン本体の距離関数 h_drone を定義中...');
% p_drone = [p1; p2; p3]; % ドローンの位置
% v_drone = [dp1; dp2; dp3]; % ドローンの速度
% h_drone_sym = 0.5 * ((p_drone(1) - ox)^2 + (p_drone(2) - oy)^2 + (p_drone(3) - oz)^2 - (ro + r_drone)^2); % 制御バリア関数ℎ=1/2 (‖𝑟_𝑞^𝑖+𝑟_𝑜^𝑗 ‖^2−(𝑟_𝑜^𝑖+𝑟_𝑞^𝑗 )^2 )
% 
% % 1段目の全微分（ダイナミクスには一貫して FG_top_drone を使用）
% dot_h_drone = LieD(h_drone_sym, FG_top_drone, x);
% phi1_drone = dot_h_drone + d1 * h_drone_sym;
% 
% % 2段目の全微分
% dot_phi1_drone_total = LieD(phi1_drone, FG_top_drone, x);
% 
% % 【確実な代数分離】jacobianによるAの抽出と、残差によるbの抽出
% A_drone_top_sym = jacobian(dot_phi1_drone_total, u_top_layer);
% b_pure_drone    = simplify(dot_phi1_drone_total - A_drone_top_sym * u_top_layer);
% b_drone_top_sym = b_pure_drone + d2 * phi1_drone;
% 
% %% -----------------------------------------------------------------
% %% 2. 荷物（ペイロード）のCBF (相対次数 5)
% %% -----------------------------------------------------------------
% disp('▶️ [Load Layer] 荷物の距離関数 h0 から相対次数5のバックステッピングを展開中...');
% h0 = 0.5 * ((pl(1) - ox)^2 + (pl(2) - oy)^2 + (pl(3) - oz)^2 - (ro + r_load)^2);
% 
% % 各階層の時間発展において、部分線形化が考慮された FG_top_drone を一貫して使用
% h1 = simplify(LieD(h0, FG_top_drone, x) + k_cbf1 * h0);
% h2 = simplify(LieD(h1, FG_top_drone, x) + k_cbf2 * h1);
% h3 = simplify(LieD(h2, FG_top_drone, x) + k_cbf3 * h2);
% h4 = simplify(LieD(h3, FG_top_drone, x) + k_cbf4 * h3);
% 
% disp('▶️ [Load Layer 5] 荷物側の最上流入力結合マトリクスを展開中...');
% dot_h4_total = LieD(h4, FG_top_drone, x);
% 
% % 【確実な代数分離】jacobianによるAの抽出と、残差によるbの抽出
% A_load_top_sym = jacobian(dot_h4_total, u_top_layer);
% b_pure_load    = simplify(dot_h4_total - A_load_top_sym * u_top_layer);
% b_load_top_sym = b_pure_load + k_cbf5 * h4;
% 
% %% -----------------------------------------------------------------
% %% 3. 変数の置換と完全クリーンアップ（xおよび参照XDへの幾何マッピング）
% %% -----------------------------------------------------------------
% disp('▶️ 幾何学的閉ループ関係による状態変数 x への置換を実行中...');
% p_equiv  = pl + cableL * pT;
% wL_cross_pT = [ol2*pT3 - ol3*pT2; ol3*pT1 - ol1*pT2; ol1*pT2 - ol2*pT1];
% dp_equiv = dpl + cab
% leL * wL_cross_pT;
% 
% % 全シンボリック式の置換調停
% A_drone_top_sym = subs(A_drone_top_sym, [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% b_drone_top_sym = subs(b_drone_top_sym, [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% A_load_top_sym  = subs(A_load_top_sym,  [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% b_load_top_sym  = subs(b_load_top_sym,  [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% 
% A_load_top_sym  = subs(A_load_top_sym,  xdReff, XDf);
% b_load_top_sym  = subs(b_load_top_sym,  xdReff, XDf);
% A_drone_top_sym = subs(A_drone_top_sym, xdReff, XDf);
% b_drone_top_sym = subs(b_drone_top_sym, xdReff, XDf);
% 
% %% 【確定解決版】4. Mファイル関数としてのエクスポート
% disp('💾 新・最上流同期型CBF関数の書き出し中（変数クリーンアップ実施）...');
% 
% % 🌟超重要：数式の奥深くに残った入力変数のゴミを、0を代入することで完全に消滅させる
% % すでにjacobianでAとbに分離した後のため、0を代入しても数理的な値は一切変わりません
% A_load_top_clean  = subs(A_load_top_sym,  [v1_cmd, u2_cmd, u3_cmd], [0, 0, 0]);
% b_load_top_clean  = subs(b_load_top_sym,  [v1_cmd, u2_cmd, u3_cmd], [0, 0, 0]);
% A_drone_top_clean = subs(A_drone_top_sym, [v1_cmd, u2_cmd, u3_cmd], [0, 0, 0]);
% b_drone_top_clean = subs(b_drone_top_sym, [v1_cmd, u2_cmd, u3_cmd], [0, 0, 0]);
% 
% % 🌟パラメータをすべて「確実に1次元の縦ベクトル」として結合する
% cbfParam_top_load  = [ox; oy; oz; ro; r_load;  k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; physicalParam(:)];
% cbfParam_top_drone = [ox; oy; oz; ro; r_drone; d1; d2; physicalParam(:)];
% 
% XD_vars = cell2sym(XD);
% 
% % 🌟引数にはクリーンアップした数式（_clean）を渡す
% matlabFunction(A_load_top_clean, b_load_top_clean, 'file', 'CBF_Constraints_Load_TopLayer.m', ...
%     'vars', {obj, x, XD_vars, cbfParam_top_load, t}, 'outputs', {'A_load_top', 'b_load_top'}, 'Optimize', true);
% 
% matlabFunction(A_drone_top_clean, b_drone_top_clean, 'file', 'CBF_Constraints_Drone.m', ...
%     'vars', {obj, x, XD_vars, cbfParam_top_drone, t}, 'outputs', {'A_drone_top', 'b_drone_top'}, 'Optimize', true);
% 
% disp('========================================================================');
% disp('🎉 [エラー解消] ドローン用＆荷物用の複数障害物対応HLC関数が完全に出力されました！');
% disp('========================================================================');

% %% =========================================================================
% %% HO-CBF (高度階層型線形化・完全デカップリング保護版) 自動導出セクション
% %% =========================================================================
% clc
% disp('========================================================================');
% disp(' 最上流仮想入力 [v1_cmd; v2_cmd; v3_cmd] に対する相対次数5のHO-CBFを生成します');
% disp('========================================================================');
% 
% % 1. 公称システムの抽出（未知外乱dstx, dstyを0として公称閉ループダイナミクスを構築）ここは問題ない
% f_nominal = subs(f, [dstx, dsty], [0, 0]); 
% g_nominal = g; 
% 
% % クラス(HLC)側の v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)] と完全同期した変更する入力　問題なし
% syms v1_cmd v2_cmd v3_cmd real
% u_top_layer = [v1_cmd; v2_cmd; v3_cmd];
% 
% % HO-CBFチェーンゲイン 問題なし
% syms k_cbf1 k_cbf2 k_cbf3 k_cbf4 k_cbf5 real
% % 障害物情報と保護半径のシンボル　問題なし
% syms ox oy oz ro real                       
% syms r_sphere real 
% syms lambda_val real                        % 荷物(0)〜ドローン(1)の分割位置
% 
% % 🚨【バグ修正解決】：HLCコントローラが実際に計算している2層の結合ダイナミクスを完璧に再現
% % 1層目の高度仮想入力に v1_cmd を、2層目の水平・yaw仮想入力に [v2_cmd; v3_cmd; 0] をマッピング
% % これにより、水平移動のシンボルが 5次の全微分チェーン（Lie微分）に100%漏れなく結合します！
% us_cbf_link = beta2 \ ([v2_cmd; v3_cmd; 0] - alpha2); 
% uf_cbf_link = H(1,1) * (-alpha1 + v1_cmd); % 未定義変数エラーを解消する正しいシンボリック表現
% 
% % 全系実入力ベクトル [uf; us] を構成（yaw用の第4入力は0で固定）
% % 上流のダイナミクス合成時に残ったダミー変数 [u2, u3, u4] を 0 に落として数式を完全にクリーンアップ
% u_full_dynamic = [uf_cbf_link; us_cbf_link];
% u_full_dynamic = subs(u_full_dynamic, [u2, u3, u4], [0, 0, 0]);
% 
% % 公称プラントモデルに完全結合閉ループダイナミクスをドッキング
% FG_top_synced = simplify(f_nominal + g_nominal * u_full_dynamic);
% 
% % 5. 配置割合 lambda_val に応じた保護球の汎用位置ベクトルの定義
% p_sphere = pl - lambda_val * cableL * pT;
% 
% % 6. バリア関数（ゼロ超レベルセット）の基礎定義
% h0_sphere = 0.5 * ((p_sphere(1) - ox)^2 + (p_sphere(2) - oy)^2 + (p_sphere(3) - oz)^2 - (ro + r_sphere)^2);
% 
% %% -----------------------------------------------------------------
% %% 相対次数5のHO-CBFバックステッピング展開
% %% -----------------------------------------------------------------
% disp('▶️ [HO-CBF] 5階層の全微分（Lie微分チェーン）を計算中...');
% h1_s = simplify(LieD(h0_sphere, FG_top_synced, x) + k_cbf1 * h0_sphere);
% h2_s = simplify(LieD(h1_s, FG_top_synced, x) + k_cbf2 * h1_s);
% h3_s = simplify(LieD(h2_s, FG_top_synced, x) + k_cbf3 * h2_s);
% h4_s = simplify(LieD(h3_s, FG_top_synced, x) + k_cbf4 * h3_s); 
% % 5階層目（ここで最上流入力 [v1_cmd, v2_cmd, v3_cmd] が代数的に結合します）
% dot_h4_sphere = LieD(h4_s, FG_top_synced, x);
% 
% %% -----------------------------------------------------------------
% %% 幾何学的マッピング変数（等価式）の明示的定義
% %% -----------------------------------------------------------------
% p_equiv  = pl + cableL * pT;                                     
% wL_cross_pT = [ol2.*pT3 - ol3.*pT2; ol3.*pT1 - ol1.*pT3; ol1.*pT2 - ol2.*pT1]; 
% dp_equiv = dpl + cableL * wL_cross_pT;                           
% 
% %% -----------------------------------------------------------------
% %% 操作入力マトリクス A_s と残差ベクトル b_s の代数分離と完全置換
% %% -----------------------------------------------------------------
% disp('▶️ [HO-CBF] 操作入力マトリクス A_s と残差ベクトル b_s の代数分離を実行中...');
% A_sphere_sym = jacobian(dot_h4_sphere, u_top_layer);
% b_pure_sphere = simplify(dot_h4_sphere - A_sphere_sym * u_top_layer);
% b_sphere_sym = b_pure_sphere + k_cbf5 * h4_s;
% 
% disp('▶️ [HO-CBF] 幾何学マッピングおよび不要成分の完全クリーンアップ中...');
% A_sphere_clean = subs(A_sphere_sym,   [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% b_sphere_clean = subs(b_sphere_sym,   [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% 
% % 数式に残ったダミー入力変数の消滅処理と参照軌道の置換
% A_sphere_clean = subs(A_sphere_clean, [v1_cmd, v2_cmd, v3_cmd, xdReff], [0, 0, 0, XDf]);
% b_sphere_clean = subs(b_sphere_clean, [v1_cmd, v2_cmd, v3_cmd, xdReff], [0, 0, 0, XDf]);
% 
% disp('💾 HLCのP配列（9要素）に完全同期したCBF関数ファイルをエクスポート中...');
% syms mass_hlc jx_hlc jy_hlc jz_hlc gravity_hlc loadmass_hlc cableL_hlc dummy1 dummy2 real
% P_hlc_style = [mass_hlc; jx_hlc; jy_hlc; jz_hlc; gravity_hlc; loadmass_hlc; cableL_hlc; dummy1; dummy2];
% 
% A_sphere_hlc = subs(A_sphere_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                     [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% b_sphere_hlc = subs(b_sphere_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                     [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% 
% cbfParam_compiled = [ox; oy; oz; ro; r_sphere; lambda_val; k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; P_hlc_style(:)];
% XD_vars = cell2sym(XD);
% 
% % Mファイル関数を上書きエクスポート
% matlabFunction(A_sphere_hlc, b_sphere_hlc, 'file', 'CBF_Constraints_Synced_Order5.m', ...
%     'vars', {obj, x, XD_vars, cbfParam_compiled, t}, 'outputs', {'A_s', 'b_s'}, 'Optimize', true);
% 
% disp('========================================================================');
% disp('🎉 [水平回避・完全開通] 3軸すべてに数式が詰まった高次CBF関数が正常に出力されました！');
% disp('========================================================================');

% %% =========================================================================
% %% HO-CBF (水平2軸・相対次数5・実入力ベース軽量版) 自動導出セクション
% %% =========================================================================
% clc
% disp('========================================================================');
% disp(' 🌟 メモリ爆発を完全に回避した、荷物の水平に対する相対次数5のHO-CBFを生成します');
% disp('========================================================================');
% 
% % 1. 公称システムの抽出（未知外乱dstx, dstyを0として公称閉ループダイナミクスを構築）
% f_nominal = subs(f, [dstx, dsty], [0, 0]); 
% g_nominal = g; 
% 
% % 🚨【メモリ防衛】：逆行列は絶対に計算せず、実入力 [u2; u3; u4] の文字のまま結合します！
% % yaw用の第4入力は0固定
% syms u2_cmd u3_cmd real
% u_top_layer = [u2_cmd; u3_cmd];
% 
% % 1层目の高度方向は名目入力のままで結合（uf = H(:,1)*(-alpha1 + v1(t))）
% uf_cbf_link = H(1,1) * (-alpha1 + v1(t)); 
% 
% % 全系実入力ベクトルを安全に結合（ダミー変数u2, u3, u4から一度完全に切り離して再パッキング）
% u_full_dynamic = [uf_cbf_link; u2_cmd; u3_cmd; 0];
% u_full_dynamic = subs(u_full_dynamic, [u2, u3, u4], [0, 0, 0]);
% 
% % 公称プラントモデルにドッキング（文字数が肥大化しない超軽量な式になります）
% FG_top_synced = simplify(f_nominal + g_nominal * u_full_dynamic);
% 
% % HO-CBFチェーンゲインと保護半径定義
% syms k_cbf1 k_cbf2 k_cbf3 k_cbf4 k_cbf5 real
% syms ox oy oz ro r_sphere lambda_val real                        
% 
% p_sphere = pl - lambda_val * cableL * pT;
% % 水平方向だけの2次元バリア関数（Z軸はHLCに任せるため拘束から解放）
% h0_sphere = 0.5 * ((p_sphere(1) - ox)^2 + (p_sphere(2) - oy)^2 - (ro + r_sphere)^2);
% 
% %% -----------------------------------------------------------------
% %% 相対次数5のHO-CBFバックステッピング展開（たったこれだけの式なので爆速処理）
% %% -----------------------------------------------------------------
% disp('▶️ [HO-CBF] 水平5階層の全微分（Lie微分チェーン）を計算中...');
% h1_s = simplify(LieD(h0_sphere, FG_top_synced, x) + k_cbf1 * h0_sphere);
% h2_s = simplify(LieD(h1_s, FG_top_synced, x) + k_cbf2 * h1_s);
% h3_s = simplify(LieD(h2_s, FG_top_synced, x) + k_cbf3 * h2_s);
% h4_s = simplify(LieD(h3_s, FG_top_synced, x) + k_cbf4 * h3_s); 
% % 5階層目（ここで実入力 [u2_cmd, u3_cmd] が代数的に美しく結合します）
% dot_h4_sphere = LieD(h4_s, FG_top_synced, x);
% 
% %% -----------------------------------------------------------------
% %% 操作入力マトリクス A_s と残差ベクトル b_s の代数分離と完全置換
% %% -----------------------------------------------------------------
% disp('▶️ [HO-CBF] 操作入力の代数分離を実行中...');
% % 🚨 逆行列を介していないため、極めて短く綺麗なヤコビアンが抽出されます
% A_sphere_sym = jacobian(dot_h4_sphere, u_top_layer);
% b_pure_sphere = simplify(dot_h4_sphere - A_sphere_sym * u_top_layer);
% b_sphere_sym = b_pure_sphere + k_cbf5 * h4_s;
% 
% disp('▶️ [HO-CBF] 幾何学マッピングおよび不要成分の完全クリーンアップ中...');
% p_equiv  = pl + cableL * pT;                                     
% wL_cross_pT = [ol2.*pT3 - ol3.*pT2; ol3.*pT1 - ol1.*pT3; ol1.*pT2 - ol2.*pT1]; 
% dp_equiv = dpl + cableL * wL_cross_pT;                           
% 
% A_sphere_clean = subs(A_sphere_sym,   [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% b_sphere_clean = subs(b_sphere_sym,   [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% 
% % 参照軌道(XD)への置換とダミー文字の完全消去
% A_sphere_clean = subs(A_sphere_clean, [v1(t), u2_cmd, u3_cmd, xdReff], [v1, 0, 0, XDf]);
% b_sphere_clean = subs(b_sphere_clean, [v1(t), u2_cmd, u3_cmd, xdReff], [v1, 0, 0, XDf]);
% 
% disp('💾 HLCのP配列（9要素）に完全同期中...');
% syms mass_hlc jx_hlc jy_hlc jz_hlc gravity_hlc loadmass_hlc cableL_hlc dummy1 dummy2 real
% P_hlc_style = [mass_hlc; jx_hlc; jy_hlc; jz_hlc; gravity_hlc; loadmass_hlc; cableL_hlc; dummy1; dummy2];
% 
% % 確実な最終一括置換
% A_sphere_hlc = subs(A_sphere_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                     [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% b_sphere_hlc = subs(b_sphere_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                     [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% 
% cbfParam_compiled = [ox; oy; oz; ro; r_sphere; lambda_val; k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; P_hlc_style(:)];
% XD_vars = cell2sym(XD);
% 
% % Mファイル関数を上書きエクスポート
% matlabFunction(A_sphere_hlc, b_sphere_hlc, 'file', 'CBF_Constraints_Synced_Order5.m', ...
%     'vars', {obj, x, XD_vars, cbfParam_compiled, t}, 'outputs', {'A_s', 'b_s'}, 'Optimize', true);
% 
% disp('========================================================================');
% disp('🎉 [軽量化・完全開通] メモリ不足を完全に克服した5次CBF関数が出力されました！');
% disp('========================================================================');

% %% =========================================================================
% %% HO-CBF (水平2軸・相対次数5・完全メモリ防衛＆代数隔離版)
% %% =========================================================================
% clc
% disp('========================================================================');
% disp(' 🌟 メモリ爆発を100%回避する、最上流仮想入力に対する5次CBFを生成します');
% disp('========================================================================');
% 
% % 🌟 1. 公称ダイナミクスを極限まで軽量化
% % 未知外乱(dstx, dsty)を0とした公称システムを抽出
% f_nominal = subs(f, [dstx, dsty], [0, 0]); 
% g_nominal = g; 
% 
% % 🌟 2. 仮想入力を「ただの独立した文字の集まり」として定義
% % 微分計算中に複雑な分数式（逆行列やalpha1, 2）が混ざるのを完全に遮断します
% syms v1_cbf v2_cmd v3_cmd v4_cbf real
% u_virtual_layer = [v2_cmd; v3_cmd]; % CBFの調整対象（水平の仮想入力）
% 
% % 全系の入力ベクトルを、ただの「シンプルな文字の並び」としてパッキング
% % 1層目の高度(v1)や4軸目のyaw(v4)は、ここでは独立した文字変数として扱います
% u_nominal_simple = [v1_cbf; v2_cmd; v3_cmd; v4_cbf];
% 
% % 閉ループダイナミクスをドッキング
% % 逆行列などを一切含まないため、数式は非常に軽量なままです
% FG_top_synced = simplify(f_nominal + g_nominal * u_nominal_simple);
% 
% % 🌟 3. バリア関数の定義
% % 障害物情報と保護半径のシンボル
% syms ox oy oz ro r_sphere lambda_val real                        
% syms k_cbf1 k_cbf2 k_cbf3 k_cbf4 k_cbf5 real % HO-CBFチェーンゲイン
% 
% p_sphere = pl - lambda_val * cableL * pT;
% % 水平方向の安全性を評価する2次元バリア関数
% h0_sphere = 0.5 * ((p_sphere(1) - ox)^2 + (p_sphere(2) - oy)^2 - (ro + r_sphere)^2);

% %% =========================================================================
% %% HO-CBF (水平2軸・相対次数5・完全メモリ防衛＆代数隔離版) 【エラー完全解決・最終開通版】
% %% =========================================================================
% clc
% disp('========================================================================');
% disp(' 🌟 メモリ爆発を100%回避する、最上流仮想入力に対する5次CBFを生成します');
% disp('========================================================================');
% 
% % 1. 公称ダイナミクスを極限まで軽量化
% f_nominal = subs(f, [dstx, dsty], [0, 0]); 
% g_nominal = g; 
% 
% % 2. 仮想入力を独立した文字として定義
% syms v1_cbf v2_cmd v3_cmd v4_cbf real
% u_virtual_layer = [v2_cmd; v3_cmd]; % CBFの調整対象（水平の仮想入力）
% u_nominal_simple = [v1_cbf; v2_cmd; v3_cmd; v4_cbf];
% 
% % 閉ループダイナミクスをドッキング
% FG_top_synced = simplify(f_nominal + g_nominal * u_nominal_simple);
% 
% % 3. 荷物用バリア関数の定義
% syms ox oy oz ro r_sphere lambda_val real                        
% syms k_cbf1 k_cbf2 k_cbf3 k_cbf4 k_cbf5 real % 荷物用HO-CBFチェーンゲイン
% p_sphere = pl - lambda_val * cableL * pT;
% h0_sphere = 0.5 * ((p_sphere(1) - ox)^2 + (p_sphere(2) - oy)^2 - (ro + r_sphere)^2);
% 
% %% -----------------------------------------------------------------
% %% 荷物側：相対次数5のHO-CBFバックステッピング展開
% %% -----------------------------------------------------------------
% disp('▶️ [Load-CBF] 独立文字のまま5階層の全微分（Lie微分チェーン）を計算中...');
% h1_s = simplify(LieD(h0_sphere, FG_top_synced, x) + k_cbf1 * h0_sphere);
% h2_s = simplify(LieD(h1_s, FG_top_synced, x) + k_cbf2 * h1_s);
% h3_s = simplify(LieD(h2_s, FG_top_synced, x) + k_cbf3 * h2_s);
% h4_s = simplify(LieD(h3_s, FG_top_synced, x) + k_cbf4 * h3_s); 
% dot_h4_sphere = LieD(h4_s, FG_top_synced, x);
% 
% %% -----------------------------------------------------------------
% %% 操作入力マトリクス A_s と残差ベクトル b_s の代数分離
% %% -----------------------------------------------------------------
% disp('▶️ [Load-CBF] 仮想入力 [v2_cmd, v3_cmd] に対する代数分離を実行中...');
% A_sphere_raw = jacobian(dot_h4_sphere, u_virtual_layer);
% b_pure_sphere = simplify(dot_h4_sphere - A_sphere_raw * u_virtual_layer);
% b_sphere_sym = b_pure_sphere + k_cbf5 * h4_s;
% 
% disp('▶️ [Load-CBF] 幾何学マッピング置換を実行中...');
% p_equiv  = pl + cableL * pT;                                     
% wL_cross_pT = [ol2.*pT3 - ol3.*pT2; ol3.*pT1 - ol1.*pT3; ol1.*pT2 - ol2.*pT1]; 
% dp_equiv = dpl + cableL * wL_cross_pT;                           
% A_sphere_clean = subs(A_sphere_raw,   [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% b_sphere_clean = subs(b_sphere_sym,   [p1; p2; p3; dp1; dp2; dp3], [p_equiv; dp_equiv]);
% 
% % 参照軌道(XD)の置換
% A_sphere_clean = subs(A_sphere_clean, xdReff, XDf);
% b_sphere_clean = subs(b_sphere_clean, xdReff, XDf);
% 
% disp('💾 HLCのP配列に完全同期した軽量荷物用CBF関数ファイルをエクスポート中...');
% syms mass_hlc jx_hlc jy_hlc jz_hlc gravity_hlc loadmass_hlc cableL_hlc dummy1 dummy2 real
% P_hlc_style = [mass_hlc; jx_hlc; jy_hlc; jz_hlc; gravity_hlc; loadmass_hlc; cableL_hlc; dummy1; dummy2];
% A_sphere_hlc = subs(A_sphere_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                     [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% b_sphere_hlc = subs(b_sphere_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                     [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% 
% % 🌟【最重要・真の解決】：診断レポートの通り数式の底に残存していた v2_cmd, v3_cmd をここで一撃で完全消滅させます
% A_sphere_hlc = subs(A_sphere_hlc, [v2_cmd, v3_cmd], [0, 0]);
% b_sphere_hlc = subs(b_sphere_hlc, [v2_cmd, v3_cmd], [0, 0]);
% 
% % A_s, b_s の中から「v2_cmd」と「v3_cmd」は跡形もなく消えたため、引数（Vars）に含めません。
% cbfParam_load_compiled = [ox; oy; oz; ro; r_sphere; lambda_val; k_cbf1; k_cbf2; k_cbf3; k_cbf4; k_cbf5; v1_cbf; v4_cbf; P_hlc_style(:)];
% XD_vars = cell2sym(XD);
% 
% % 5次関数のエクスポート
% matlabFunction(A_sphere_hlc, b_sphere_hlc, 'file', 'CBF_Constraints_Synced_Order5.m', ...
%     'vars', {obj, x, XD_vars, cbfParam_load_compiled, t}, 'outputs', {'A_s', 'b_s'}, 'Optimize', true);
% disp('🎉 [5次・荷物用] が正常に出力されました！続けてドローン用（2次）を生成します。');
% 
% 
% %% =========================================================================
% %% HO-CBF (ドローン本体・相対次数2・完全メモリ防衛＆代数隔離版)
% %% =========================================================================
% disp('------------------------------------------------------------------------');
% syms k_drone1 k_drone2 real
% syms r_drone real % ドローン用の安全マージン半径
% 
% % 1. ドローン本体用の2次元（水平面内）バリア関数の定義
% h0_drone = 0.5 * ((p_equiv(1) - ox)^2 + (p_equiv(2) - oy)^2 - (ro + r_drone)^2);
% 
% %% -----------------------------------------------------------------
% %% ドローン側：相対次数2のHO-CBFバックステッピング展開
% %% -----------------------------------------------------------------
% disp('▶️ [Drone-CBF] 独立文字のまま2階層の全微分（Lie微分チェーン）を計算中...');
% h1_drone = simplify(LieD(h0_drone, FG_top_synced, x) + k_drone1 * h0_drone);
% dot_h1_drone = LieD(h1_drone, FG_top_synced, x);
% 
% disp('▶️ [Drone-CBF] 仮想入力 [v2_cmd, v3_cmd] に対する代数分離を実行中...');
% A_drone_raw = jacobian(dot_h1_drone, u_virtual_layer);
% b_pure_drone = simplify(dot_h1_drone - A_drone_raw * u_virtual_layer);
% b_drone_sym = b_pure_drone + k_drone2 * h1_drone;
% 
% A_drone_clean = subs(A_drone_raw, xdReff, XDf);
% b_drone_clean = subs(b_drone_sym, xdReff, XDf);
% 
% disp('💾 HLCのP配列に完全同期したドローン用CBF関数ファイルをエクスポート中...');
% A_drone_hlc = subs(A_drone_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                   [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% b_drone_hlc = subs(b_drone_clean, [m, jx, jy, jz, gravity, mL, cableL], ...
%                                   [mass_hlc, jx_hlc, jy_hlc, jz_hlc, gravity_hlc, loadmass_hlc, cableL_hlc]);
% 
% % 🌟【最重要・真の解決】：ドローン用も、matlabFunctionの直前で完全に残骸を消滅させます
% A_drone_hlc = subs(A_drone_hlc, [v2_cmd, v3_cmd], [0, 0]);
% b_drone_hlc = subs(b_drone_hlc, [v2_cmd, v3_cmd], [0, 0]);
% 
% % ドローン用からも消滅した「v2_cmd」「v3_cmd」を削除したパラメータを指定
% cbfParam_drone_compiled = [ox; oy; oz; ro; r_drone; k_drone1; k_drone2; v1_cbf; v4_cbf; P_hlc_style(:)];
% 
% % 2次関数のエクスポート
% matlabFunction(A_drone_hlc, b_drone_hlc, 'file', 'CBF_Constraints_Drone.m', ...
%     'vars', {obj, x, XD_vars, cbfParam_drone_compiled, t}, 'outputs', {'A_drone', 'b_drone'}, 'Optimize', true);
% 
% disp('========================================================================');
% disp('🎉 [完全開通] 5次(荷物) ＆ 2次(ドローン) の両方のMファイルがエラー無しで出力されました！');
% disp('========================================================================');

%% =========================================================================
%% 【追加】高次制御バリア関数 (HOCBF) の自動導出と関数エクスポート
%% =========================================================================
disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");

% 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
syms xo yo zo ro real
syms rl real
% 荷物の位置 pl = [pl1; pl2; pl3] と障害物の距離の2乗から安全を定義
h_cbf = (pl(1) - xo)^2 + (pl(2) - yo)^2 + (pl(3) - zo)^2 - (ro + rl)^2;

% 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
syms gamma1 gamma2 gamma3 gamma4 gamma5 gamma6 real

% 3. 6階の高次CBFをリー微分(LieD)を用いて順次計算
% f1, g1 は 2nd layer で求めた [u2; u3; u4] に対するシステム方程式
% 今回は u4 (yaw) の項は既知（理想値）として扱うため、g1の3列目を取り出して処理します
g1_xy = g1(:, 1:2); % u2, u3 に掛かる列だけを抽出
g1_yaw = g1(:, 3);  % u4 に掛かる列

% 階層的なCBFの導出
% ※ diff(..., t) は目標軌道 xd(t) などの時間微分をカバーするために維持します
cbf1 = LieD(h_cbf, f1, x) + diff(h_cbf, t) + gamma1 * h_cbf; %h2
cbf2 = LieD(cbf1,  f1, x) + diff(cbf1,  t) + gamma2 * cbf1; %h3
cbf3 = LieD(cbf2,  f1, x) + diff(cbf2,  t) + gamma3 * cbf2; %h4
cbf4 = LieD(cbf3,  f1, x) + diff(cbf3,  t) + gamma4 * cbf3; %h5
cbf5 = LieD(cbf4,  f1, x) + diff(cbf4,  t) + gamma5 * cbf4; %h6

% 最上階（6階）の計算：ここに u2, u3 が現れる
% cbf6_dot = L_f1(cbf5) + L_g1_xy(cbf5)*[u2; u3] + L_g1_yaw(cbf5)*u4 + diff(cbf5, t)
L_f_cbf5  = LieD(cbf5, f1, x) + diff(cbf5, t);
L_g_cbf5  = LieD(cbf5, g1_xy, x);  % [1 x 2] のベクトル（u2, u3 の係数 β）
L_gy_cbf5 = LieD(cbf5, g1_yaw, x); % (u4 の係数)

% 最終的な安全条件式: cbf6_dot + gamma6 * cbf5 >= 0
% つまり、 L_g_cbf5 * [u2; u3] + L_f_cbf5 + L_gy_cbf5 * u4 + gamma6 * cbf5 >= 0
% これを QP用の形式 「 A_qp * [u2; u3] <= b_qp 」に整理します。
% 不等号を反転させるため、符号をマイナスにします。

A_cbf_sym = -L_g_cbf5; % [1 x 2] のシンボリック行ベクトル
b_cbf_sym = L_f_cbf5 + L_gy_cbf5 * u4 + gamma6 * cbf5; % シンボリックスカラー

% 4. 実際の数値シミュレーション側で代入しやすいよう、変数を置き換え (xdRef -> XDf, vInput1f -> V1vf)
% 2nd layerの入力 u4（yaw用）には、コントローラが後で計算する実入力の4番目の要素を指定できるように V4 を代入
syms V4 real
A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f, u4], [XDf, V1vf, V4]);
b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f, u4], [XDf, V1vf, V4]);

% 5. 高速計算用に関数ファイル (Mファイル) としてエクスポート
% クラスK関数のゲインも外部から与えられるように引数に含めます
gamma_params = [gamma1; gamma2; gamma3; gamma4; gamma5; gamma6];
obs_params = [xo; yo; zo; ro];
sys_params = rl;

XD_sym = cell2sym(XD);
XD_sym = XD_sym(:); % 強制的に縦ベクトル化

disp("Exporting: CBF_Constraints_xy.m を書き出しています...");
matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_xy.m', ...
               'vars', {obj, x, XD_sym, cell2sym(V1v), V4, obs_params, gamma_params, sys_params, physicalParam}, ...
               'outputs', {'A_qp', 'b_qp'});

disp("Done: CBF関数の生成が完了しました！");
%% =========================================================================
%% Make functions of actual inputs taking t, x, xd, v1 and v2 as arguments
% % If either model, virtual output or parameters is changed, then evaluate this section. It'll take few minutes.
% % Usage: u = Uf(...) + Us(...)
    % matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','Uf_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
    % matlabFunction(subs(H(:,2:4), [xdRef vInput1], [XD V1v]),'file','H234_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'H234'});
    
    %以下二つはとても重い
    % matlabFunction(subs(inv(beta2), [xdRef vInput1], [XD V1v]),'file','inv_beta2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'inv_beta2'});
    
    % matlabFunction(subs((-alpha2+[v2(t);v3(t);v4(t)]), [flip(xdRef) flip(vInput1) v2(t) v3(t) v4(t)], [flip(XD) flip(V1v) V2 V3 V4]),'file','vs_alpha2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'vs_alpha2'});
    % matlabFunction(subs(alpha2(1), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha21_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha21'});
    % matlabFunction(subs(alpha2(2), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha22_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha22'});
    % matlabFunction(subs(alpha2(3), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha23_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha23'});

    % a2_v2 = subs((-alpha2+[v2(t);v3(t);v4(t)]), [flip(xdRef) flip(vInput1) v2(t) v3(t) v4(t)], [flip(XD) flip(V1v) V2 V3 V4]);
    % matlabFunction(a2_v2,'file','vs_alpha2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'vs_alpha2'});
    
    % xyDst
    matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdReff vInput1f], [XDf V1vf]),'file','Uf_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
    matlabFunction(subs(H(:,2:4), [xdReff vInput1f], [XDf V1vf]),'file','H234_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'H234'});
    matlabFunction(subs(beta2, [xdReff vInput1f], [XDf V1vf]),'file','Beta2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'beta2'});
    %以下はとても重い
    v2_a2 = subs([V2;V3;V4] - alpha2, [xdReff vInput1f], [XDf V1vf]);
    matlabFunction(v2_a2,'file','V2_alpha2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'});

%理想のfunctionだけどUsが重すぎるので分割している．
    % matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XD V1v]),'file','Uf_SuspededLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
    % matlabFunction(subs(H(:,2:4)*U2, [xdRef vInput1 v2(t) v3(t) v4(t)], [XD V1v [V2 V3 V4]]),'file','Us_SuspededLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'U2'});

% % For check
%     Uf(0,x0,Xd(0),Vf(0,x0,Xd(0)))
%     Us(0,x0,Xd(0),Vf(0,x0,Xd(0)),Vs(0,x0,Xd(0),Vf(0,x0,Xd(0))))
% %%
% matlabFunction(subs(alpha1, [xdRef], [XD]),'file','alpha1.m','vars',{obj cell2sym(XD) physicalParam},'outputs',{'al1'});
% matlabFunction(subs(alpha2, [xdRef vInput1], [XD V1v]),'file','alpha2.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'al2'});
% %%
% matlabFunction(subs(beta2, [xdRef vInput1], [XD V1v]),'file','beta2.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'be2'});
% %%
% matlabFunction(subs(He, [xdRef vInput1], [XD V1v]),'file','He.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) e1 physicalParam},'outputs',{'mat'});
% %%
% matlabFunction(beta1,'file','beta1.m','vars',{obj t x physicalParam},'outputs',{'beta1'});


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
