classdef FUNCTIONAL_MECKC < handle
% クアッドコプター用階層型線形化を使った入力算出
% シミュレーションに使ったMECプログラム
properties
    self
    result
    param
    parameter_name = ["mass", "Lx", "Ly", "lx", "ly", "jx", "jy", "jz", "gravity", "km1", "km2", "km3", "km4", "k1", "k2", "k3", "k4"];
    Vf
    Vs
    agent
    motive
    delta_u
    pre_input = [0;0;0;0];
    x_pre = [0;0;0;0;0;0;0;0;0;0;0;0];
end

methods

    function obj = FUNCTIONAL_MECKC(self, param)
        


        % obj.data_gen_mode = true;
        % obj.data_gen_mode = false;
        % true:Δuを生成・保存, false:MECの検証

        obj.self = self;
        obj.param = param;
        obj.param.P = self.parameter.get(obj.parameter_name);
        obj.result.input = zeros(self.estimator.model.dim(2),1);

        initial_state.p = self.plant.state.p;
        initial_state.q = self.plant.state.q;
        initial_state.v = self.plant.state.v;
        initial_state.w = self.plant.state.w;

        obj.Vf = obj.param.Vf; % 階層１の入力を生成する関数ハンドル
        obj.Vs = obj.param.Vs; % 階層２の入力を生成する関数ハンドル
        
    end

    function result = do(obj,varargin)
        model = obj.self.estimator.result;
        ref = obj.self.reference.result;
        disp(ref.state.p);
        x = [model.state.p(1);
             model.state.p(2);
             model.state.p(3);
             model.state.q(1);
             model.state.q(2);
             model.state.q(3);
             model.state.v(1);
             model.state.v(2);
             model.state.v(3);
             model.state.w(1);
             model.state.w(2);
             model.state.w(3);];
        % x = [R2q(Rb0' * model.state.getq("rotmat")); Rb0' * model.state.p; Rb0' * model.state.v; model.state.w]; % [q, p, v, w]に並べ替え
        %%MECK
        if isfield(varargin{3}.Data.agent, "controller") && isfield(varargin{3}.Data.agent, "estimator") &&(numel(varargin{3}.Data.agent.estimator.result) >= 2)% ループの最初はLoggingされていなくて，参照できないのを回避
                obj.pre_input = varargin{3}.Data.agent.controller.result{end}.input; % LOGGERの中から前時刻の入力を取得
                obj.x_pre = varargin{3}.Data.agent.estimator.result{:,end-1}.state.get; % LOGGERの中から前時刻の状態を取得
        end
        dt = varargin{1}.dt;
        dx = roll_pitch_yaw_thrust_torque_physical_parameter_model(obj.x_pre, obj.pre_input, obj.param.P);
        x_n_now = obj.x_pre + dx*dt;%x_nominal[k+1]
        delta_x = x-x_n_now;
        z_p=quaternions_all(x); %観測量z※プラントの状態を入れてる
        z_n=quaternions_all(x_n_now);%ノミナルの状態
        % y_p=obj.param.C*z_p;
        % y_n=obj.param.C*z_n;

        A = obj.param.est.A;%クープマンモデルのA,B,C
        B = obj.param.est.B;
        C = obj.param.est.C;
        
        %%-----スライディングモード制御-----%%
        % e=obj.x_pre-C*z_p;
        % e = C*(z_n-z_p);
        % Z=z_n-z_p;
        % fai=1;
        % S_1 = [0,0,1,0,0,0,0,0,0,0,0,0];
        % S_2 = [0,0,0,1,0,0,0,0,0,0,0,0];
        % S_3 = [0,0,0,0,0.1,0,0,0,0,0,0,0];
        % S_4 = [0,0,0,0,0,1,0,0,0,0,0,0];
        % S_all = [S_1;S_2;S_3;S_4];
        % sig = S_all*e;
        % sat = zeros(4,1);
        % for i = 1:length(sig)
        % if abs(sig(i)) <= fai
        %     sat(i,1) = sig(i)/fai;
        % else
        %     sat(i,1) = sign(sig(i));
        % end
        % end
        % SCB = S_all * C*B;
        % rank(SCB);
        % cond(SCB);
        % pinv_SCB = pinv(SCB,1e-3);
        % % u_equal = -pinv_SCB*(S_all*x_n_now-S_all*C*A*z_p);
        % u_equal = -pinv_SCB*S_all*C*A*(z_n-z_p);
        % u_controll = -pinv_SCB*diag([1 1 1 1])*sat;
        % obj.result.delta_u = u_equal+u_controll;%Δu計算
        %%-----スライディングモード終わり-----%%
        
        %%%%%-----lqr法-----%%%%%
        
        % Q = diag([100, 100, 100, 1,1,1,ones(1,20)]);
        % R = 1 * eye(4);
        % %---不可制御を含んだdlqr---%
        % [K_full,~,~] = dlqr(A,B,Q,R);
        % K_direct=K_full ;
        %---不可制御を含んだdlqr終わり---%

        
       %---可制御部分をデカップリング---%
        % A = [1 0 0 0;0 -1 0 1;0 0 -1 0;2 0 -1 -1];
        % B = [-1;1;0;-1];
        % C = [1 0 1 0];
        % Uc = ctrb(A, B);
        % k = rank(Uc);
        % [Abar, Bbar,Cbar,Tc,P] = ctrbf(A, B,C);
        % k = sum(P);
        % Ac = Abar(end-(k-1):end, end-(k-1):end);%Abarの右下部分(可制御部分)
        % Bc = Bbar(end-(k-1):end, :);%Bbarの下部分
        % Qc = diag([ones(1,size(Ac,1))]);
        % Rc = 1*eye(4);
        % Kc = dlqr(Ac, Bc, Qc, Rc);

        % 可制御部分だけのゲイン → 全空間（n次元）に拡張
        % K_all = [zeros(size(Kc,1), size(A,1)-k),Kc];   % m x n
        % K_all = [Kc,zeros(size(Kc,1), size(A,1)-k)];   % m x n
        % K_full = K_all / Tc;   % 正しいマッピング: u = -K_full * x
        %---デカップリング終わり---%



       %%%%%-----ここから可制御抜き出し9/16(火)-----%%%%%
        n = size(A, 1);
tol = 1e-14; % 許容誤差
Mc = ctrb(A, B);
k=rank(Mc);
Mo = obsv(A,C);
ImMc_orth = orth(Mc);
% ImMc_orth = [1,0;0,1;0,0;1,0];
KerMo_orth = null(Mo,'rational');
T_inv = [];


% Xa: 可制御かつ不可観測
% ImMc の基底から KerMo の空間に属するものを抽出
if ~isempty(ImMc_orth) && ~isempty(KerMo_orth)
    for i = 1:size(ImMc_orth, 2)
        v = ImMc_orth(:, i);
        % v が KerMo の列空間に含まれるか判定
        if norm(KerMo_orth * (KerMo_orth' * v) - v) < tol
            % 既に T_inv に含まれていないか確認
            if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
                T_inv = [T_inv, v];
            end
        end
    end
end
Xa = T_inv; % ここまでが Xa

% Xb: 可制御かつ可観測
% ImMc の基底のうち、Xa と線形独立なものを抽出
for i = 1:size(ImMc_orth, 2)
    v = ImMc_orth(:, i);
    % T_inv に含まれていないか確認
    if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
        T_inv = [T_inv, v];
    end
end
Xb = T_inv(:, size(Xa,2)+1:size(T_inv,2));

% Xc: 不可制御かつ可観測
% KerMo の基底のうち、Xa, Xb と線形独立なものを抽出
for i = 1:size(KerMo_orth, 2)
    v = KerMo_orth(:, i);
    % T_inv に含まれていないか確認
    if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
        T_inv = [T_inv, v];
    end
end
Xc = T_inv(:, size(Xa,2)+size(Xb,2)+1:size(T_inv,2));

% Xd: 不可制御かつ不可観測
% T_inv に残りの次元を埋める基底を追加
if size(T_inv, 2) < n
    % T_invの列空間の直交補空間をnullで計算
    Xd = null(T_inv');
    T_inv = [T_inv, Xd];
end
% 最終的なXb, Xc, Xdの抽出
Xd = T_inv(:, size(Xa,2)+size(Xb,2)+size(Xc,2)+1:end);

% 3. 変換行列 T の作成と分解の実行
%--------------------------------------------------------------------------
% T_inv の列数がnになっているか確認
if size(T_inv, 2) ~= n
    error('変換行列の列数が状態空間の次元と一致しません。');
end

% 正則性を最終確認
if abs(det(T_inv)) < tol
    error('変換行列が特異行列です。');
end


T = inv(T_inv);
F = T_inv\A*T_inv;
G = T_inv\B;
H = C*T_inv;
%可制御部分抜き出し
Ac = F(1:k, 1:k);
Bc = G(1:k, :);

Qc = diag([ones(1,size(Ac,1))]);
Rc = 1*eye(4);
Kc = dlqr(Ac, Bc, Qc, Rc);
K_all = [Kc,zeros(size(B,2),(size(A,1)-k))];
K_full = K_all/T_inv;


        e = z_n-z_p;
        obj.result.delta_u = -K_full*e;
        %%%%%-----lqr法終わり-----%%%%%

        % obj.result.delta_u = 0;%unだけ確認したいとき
        
        obj.result.input=varargin{5}.controller.nominal.result.u_nominal+obj.result.delta_u;%un+Δu
        result=obj.result;
    end

    function show(obj)
        obj.result
    end

end

end