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
syms ax ay dax day real               % 揺れ角 alpha=[ax; ay], dalpha=[dax; day]
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
% x=[q;ob;pl;dpl;pT;ol];
% 【修正箇所】 17次元状態量に必要なベクトルを事前構築
xQ     = p;                         % 機体位置 [p1; p2; p3]
vQ     = dp;                        % 機体速度 [dp1; dp2; dp3]
alpha  = [ax; ay];                  % 揺れ角
dalpha = [dax; day];                % 揺れ角速度
x = [xQ; alpha; vQ; dalpha; q; ob]; % 17次元ベクトル
% 単位方向ベクトル p(alpha)
p_alpha = [cos(ax)*sin(ay);
    sin(ax);
    cos(ax)*cos(ay)];
% physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL];
% f = FL(x,physicalParam);
% g = GL(x,physicalParam);
% physicalParam = [m, jx, jy, jz, gravity,mL,cableL];

syms dstx dsty real
physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL, dstx, dsty];

f = FLxyDst_alpha(x,physicalParam);
g = GLxyDst_alpha(x,physicalParam);
physicalParam = [m, jx, jy, jz, gravity,mL,cableL,dstx, dsty];

% %% =========================================================================
% %% 【改修】論文 (Zheng et al. 2025) に基づく複数保護球 HOCBF の自動導出と関数エクスポート
% %% =========================================================================
% disp("Start: 論文モデルに基づく複数保護球 HOCBF の導出と関数化を開始します。");
% 
% % 1. 17次元状態量 x = [xQ; alpha; vQ; dalpha; q; ob] の定義確認
% % (前段の定義を引き継ぎます)
% 
% % 2. 障害物パラメータのシンボリック定義
% % N個の障害物に対応できるよう配列化、ここでは単一障害物 (xo, yo, zo, ro) に対して
% % m個の保護球をループ生成する構造とします。
% syms xo yo zo ro real
% x_o = [xo; yo; zo]; % 障害物中心
% 
% % 3. 保護球の設定パラメータ (シンボリック配列)
% % 例として m 個の保護球: lambda_j in [0, 1], r_q_j (各球の半径)
% % 実機/シミュレーション側から lambda_list, rq_list として渡せるように設定
% syms m_spheres integer real
% % シンボリック計算用には一般項 j を導出するか、主要な保護球(例: 機体 j=1, 荷物 j=2, 中間 j=3)を自動展開します
% syms lambda_j rq_j real 
% 
% % --- 汎用的な 1 つの保護球 j に対する h^ij(x) の定式化 ---
% % x_q^j = xQ + lambda_j * cableL * p_alpha
% xq_j = xQ + lambda_j * cableL * p_alpha;
% 
% % 距離の2乗に基づく安全関数 h^{ij}
% h_cbf = 0.5 * ( (xq_j - x_o)' * (xq_j - x_o) - (ro + rq_j)^2 );
% 
% % 4. Lie 微分と HOCBF (相対次数 2) の計算
% % cbf1 = h
% % cbf2 = L_f(h) + gamma1 * h
% syms gamma1 gamma2 real
% 
% % L_f(h) = dh/dx * Fl
% Lf_h = LieD(h_cbf, Fl, x);
% 
% cbf2 = Lf_h + gamma1 * h_cbf;
% 
% % L_f(cbf2) と L_g(cbf2)
% Lf_cbf2 = LieD(cbf2, Fl, x);
% Lg_cbf2 = LieD(cbf2, Gl, x); % [1 x 4] ベクトル (入力 U=[u1;u2;u3;u4] に対する係数)
% 
% % 5. QP制約式 A_qp * U <= b_qp の構築
% % L_g(cbf2) * U + L_f(cbf2) + gamma2 * cbf2 >= 0
% %  ==>  -L_g(cbf2) * U <= L_f(cbf2) + gamma2 * cbf2
% A_cbf_single = -Lg_cbf2;
% b_cbf_single = Lf_cbf2 + gamma2 * cbf2;
% 
% disp("Simplifying HOCBF expressions...");
% A_cbf_single = simplify(A_cbf_single);
% b_cbf_single = simplify(b_cbf_single);
% 
% % 6. Mファイルとして関数エクスポート
% % 入力引数:
% % - x: 17次元状態量
% % - obs_param: [xo, yo, zo, ro] (障害物位置・半径)
% % - sphere_param: [lambda_j, rq_j] (保護球の位置割合・半径)
% % - gamma_params: [gamma1, gamma2]
% % - physicalParam: 物理パラメータ
% 
% obs_param = [xo; yo; zo; ro];
% sphere_param = [lambda_j; rq_j];
% gamma_params = [gamma1; gamma2];
% 
% disp("Exporting: CBF_Constraints_MultiSphere.m を書き出しています...");
% matlabFunction(A_cbf_single, b_cbf_single, 'file', 'CBF_Constraints_MultiSphere', ...
%     'vars', {x, obs_param, sphere_param, gamma_params, physicalParam}, ...
%     'outputs', {'A_qp_j', 'b_qp_j'});
% 
% % disp("Done: 単一保護球・単一障害物に対する CBF 評価関数の生成が完了しました！");
% disp("Start: 論文モデルに基づく複数保護球 HOCBF の導出と関数化を開始します。");
% 
% % 1. 障害物パラメータのシンボリック定義
% syms xo yo zo ro real
% x_o = [xo; yo; zo]; % 障害物中心
% 
% % 2. 保護球の設定パラメータ
% syms lambda_j rq_j real 
% 
% % 保護球 j の中心位置: xq_j = xQ + lambda_j * cableL * p_alpha
% xq_j = xQ + lambda_j * cableL * p_alpha;
% 
% % 距離の2乗に基づく安全関数 h^{ij}
% % h_cbf = 0.5 * ( (xq_j(1) - x_o(1))^2 + (xq_j(2) - x_o(2))^2 + (xq_j(3) - x_o(3))^2 - (ro + rq_j)^2 );
% % h_cbf = 0.5 * ( (xq_j(3) - x_o(3))^2 - (ro + rq_j)^2 );
% 
% % 3. 元の LieD / pdiff 関数を用いた HOCBF (相対次数 2) の計算
% syms gamma1 gamma2 real
% 
% % % L_f(h) = LieD(h_cbf, f, x)
% % Lf_h = LieD(h_cbf, f, x);
% % 
% % cbf2 = Lf_h + gamma1 * h_cbf;
% % 
% % % L_f(cbf2) と L_g(cbf2)
% % Lf_cbf2 = LieD(cbf2, f, x);
% % Lg_cbf2 = LieD(cbf2, g, x); % [1 x 4] ベクトル
% 
% %% --- 【改修提案】全系を一括保護する 楕円型 HOCBF (相対次数 2) ---
% % 紐の中央点 (x_mid) をバリアの中心にする
% x_mid = xQ + 0.5 * cableL * p_alpha; 
% 
% % 楕円の各軸の長径パラメータ (a, b: 水平許容範囲, c: 垂直許容範囲)
% % 紐の長さ L や荷物サイズに合わせて調整
% a_x = ro + 0.4;  % X方向のゆとり
% a_y = ro + 0.4;  % Y方向のゆとり
% a_z = ro + 0.5 * cableL + 0.3; % Z方向は紐全体を包み込むように縦長にする
% 
% % 楕円型安全関数 h(x) >= 0 (相対次数 2)
% h_cbf = 0.5 * ( ((x_mid(1) - x_o(1))/a_x)^2 + ...
%     ((x_mid(2) - x_o(2))/a_y)^2 + ...
%     ((x_mid(3) - x_o(3))/a_z)^2 - 1.0 );
% 
% % Lie微分 (相対次数 2 なので Lf_cbf2, Lg_cbf2 まで計算)
% Lf_h = LieD(h_cbf, f, x);
% cbf2 = Lf_h + gamma1 * h_cbf;
% 
% Lf_cbf2 = LieD(cbf2, f, x);
% Lg_cbf2 = LieD(cbf2, g, x);
% 
% % 4. QP制約式 A_qp * U <= b_qp の構築
% A_cbf_single = -Lg_cbf2;
% b_cbf_single = Lf_cbf2 + gamma2 * cbf2;
% 
% disp("Simplifying HOCBF expressions...");
% A_cbf_single = simplify(A_cbf_single);
% b_cbf_single = simplify(b_cbf_single);
% 
% % 5. Mファイルとして関数エクスポート
% obs_param    = [xo; yo; zo; ro];
% sphere_param = [lambda_j; rq_j];
% gamma_params = [gamma1; gamma2];
% 
% disp("Exporting: CBF_Constraints_MultiSphere.m を書き出しています...");
% matlabFunction(A_cbf_single, b_cbf_single, 'file', 'CBF_Constraints_MultiSphere', ...
%     'vars', {x, obs_param, sphere_param, gamma_params, physicalParam}, ...
%     'outputs', {'A_qp_j', 'b_qp_j'});
% 
% disp("Done: 単一保護球・単一障害物に対する CBF 評価関数の生成が完了しました！");
% % disp("Done: 単一保護球・単一障害物に対する CBF 評価関数の生成が完了しました！");
% disp("Start: 論文モデルに基づく複数保護球 HOCBF の導出と関数化を開始します。");
% 
% % 1. 障害物パラメータのシンボリック定義
% syms xo yo zo ro real
% x_o = [xo; yo; zo]; % 障害物中心
% 
% % 2. 保護球の設定パラメータ
% syms lambda_j rq_j real 
% 
% % 保護球 j の中心位置: xq_j = xQ + lambda_j * cableL * p_alpha
% xq_j = xQ + lambda_j * cableL * p_alpha;
% 
% % 距離の2乗に基づく安全関数 h^{ij}
% % h_cbf = 0.5 * ( (xq_j(1) - x_o(1))^2 + (xq_j(2) - x_o(2))^2 + (xq_j(3) - x_o(3))^2 - (ro + rq_j)^2 );
% % h_cbf = 0.5 * ( (xq_j(3) - x_o(3))^2 - (ro + rq_j)^2 );
% 
% % 3. 元の LieD / pdiff 関数を用いた HOCBF (相対次数 2) の計算
% syms gamma1 gamma2 real
% 
% % % L_f(h) = LieD(h_cbf, f, x)
% % Lf_h = LieD(h_cbf, f, x);
% % 
% % cbf2 = Lf_h + gamma1 * h_cbf;
% % 
% % % L_f(cbf2) と L_g(cbf2)
% % Lf_cbf2 = LieD(cbf2, f, x);
% % Lg_cbf2 = LieD(cbf2, g, x); % [1 x 4] ベクトル
% 
% %% --- 【改修提案】全系を一括保護する 楕円型 HOCBF (相対次数 2) ---
% % 紐の中央点 (x_mid) をバリアの中心にする
% x_mid = xQ + 0.5 * cableL * p_alpha; 
% 
% % 楕円の各軸の長径パラメータ (a, b: 水平許容範囲, c: 垂直許容範囲)
% % 紐の長さ L や荷物サイズに合わせて調整
% a_x = ro + 0.4;  % X方向のゆとり
% a_y = ro + 0.4;  % Y方向のゆとり
% a_z = ro + 0.5 * cableL + 0.3; % Z方向は紐全体を包み込むように縦長にする
% 
% % 楕円型安全関数 h(x) >= 0 (相対次数 2)
% h_cbf = 0.5 * ( ((x_mid(1) - x_o(1))/a_x)^2 + ...
%     ((x_mid(2) - x_o(2))/a_y)^2 + ...
%     ((x_mid(3) - x_o(3))/a_z)^2 - 1.0 );
% 
% % Lie微分 (相対次数 2 なので Lf_cbf2, Lg_cbf2 まで計算)
% Lf_h = LieD(h_cbf, f, x);
% cbf2 = Lf_h + gamma1 * h_cbf;
% 
% Lf_cbf2 = LieD(cbf2, f, x);
% Lg_cbf2 = LieD(cbf2, g, x);
% 
% % 4. QP制約式 A_qp * U <= b_qp の構築
% A_cbf_single = -Lg_cbf2;
% b_cbf_single = Lf_cbf2 + gamma2 * cbf2;
% 
% disp("Simplifying HOCBF expressions...");
% A_cbf_single = simplify(A_cbf_single);
% b_cbf_single = simplify(b_cbf_single);
% 
% % 5. Mファイルとして関数エクスポート
% obs_param    = [xo; yo; zo; ro];
% sphere_param = [lambda_j; rq_j];
% gamma_params = [gamma1; gamma2];
% 
% disp("Exporting: CBF_Constraints_MultiSphere.m を書き出しています...");
% matlabFunction(A_cbf_single, b_cbf_single, 'file', 'CBF_Constraints_MultiSphere', ...
%     'vars', {x, obs_param, gamma_params, physicalParam}, ...
%     'outputs', {'A_qp_j', 'b_qp_j'});
% 
% disp("Done: 単一保護球・単一障害物に対する CBF 評価関数の生成が完了しました！");
%% =========================================================================
%% 【論文 Zheng et al. (2025) 準拠】複数保護球 arctan-HOCBF 自動導出
%% =========================================================================
disp("Start: 論文モデル (Zheng et al. 2025) に基づく arctan 補助 HOCBF の導出を開始します。");

% 1. 障害物パラメータ (位置 [xo; yo; zo], 半径 ro)
syms xo yo zo ro real
x_o = [xo; yo; zo];

% 2. 保護球パラメータ (割合 lambda_j in [0,1], 半径 rq_j)
syms lambda_j rq_j real 

% 保護球 j の中心位置: xq_j = xQ + lambda_j * cableL * p_alpha
xq_j = xQ + lambda_j * cableL * p_alpha;

% 論文 (18) 式: 障害物中心からの相対距離に基づく安全関数 h^{ij}
% h^{ij} = 0.5 * ( || xq_j - x_o ||^2 - (ro + rq_j)^2 )
h_cbf = 0.5 * ( (xq_j - x_o)' * (xq_j - x_o) - (ro + rq_j)^2 );

% 3. 論文 (22) 式: 相対次数 2 に対する arctan 補助バリア関数 H(x) の構築
% L_f(h) の計算
Lf_h = LieD(h_cbf, f, x);

% H(x) = [arctan(L_f(h)) + pi/2] * h
H_cbf = (atan(Lf_h) + pi/2) * h_cbf;

% 4. Lie 微分 L_f(H) と L_g(H) の計算 (論文 (24)-(26) 式)
syms gamma1 real % extended class K ゲイン (単一パラメータ)

Lf_H = LieD(H_cbf, f, x);
Lg_H = LieD(H_cbf, g, x); % [1 x 4] ベクトル

disp(simplify(Lg_H))
disp(simplify(Lg_H))

% 5. QP制約式 A_qp * U <= b_qp の構築
% L_f(H) + L_g(H)*U + gamma1 * H >= 0
%  ==>  -L_g(H)*U <= L_f(H) + gamma1 * H
A_cbf_single = -Lg_H;
b_cbf_single = Lf_H + gamma1 * H_cbf;

disp("Simplifying HOCBF expressions...");
A_cbf_single = simplify(A_cbf_single);
b_cbf_single = simplify(b_cbf_single);

% 6. Mファイル書き出し
obs_param    = [xo; yo; zo; ro];
sphere_param = [lambda_j; rq_j];
gamma_params = gamma1; % arctan 導入によりゲインは1つに統合

disp("Exporting: CBF_Constraints_MultiSphere.m を書き出しています...");
matlabFunction(A_cbf_single, b_cbf_single, 'file', 'CBF_Constraints_MultiSphere', ...
    'vars', {x, obs_param, sphere_param, gamma_params, physicalParam}, ...
    'outputs', {'A_qp_j', 'b_qp_j'});

disp("Done: 論文準拠の arctan 補助 HOCBF 関数の生成が完了しました！");
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
