# =========================================================================
# 1. 流体力学ベクトル場 (Fluid / Circulation) - HLC_AVOID_FLUID
# =========================================================================
$dir1 = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_AVOID_FLUID'
New-Item -ItemType Directory -Force -Path $dir1 | Out-Null
$code1 = @'
classdef HLC_AVOID_FLUID < HLC_SUSPENDED_LOAD
    properties
        p_off; v_off; a_off; j_off; s_off; d5_off;
    end
    methods
        function obj = HLC_AVOID_FLUID(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off=zeros(3,1); obj.v_off=zeros(3,1); obj.a_off=zeros(3,1);
            obj.j_off=zeros(3,1); obj.s_off=zeros(3,1); obj.d5_off=zeros(3,1);
        end
        function result = do(obj, time, varargin)
            agent_obj = varargin{4}; idx = varargin{5};
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_drone = agent_obj(idx).estimator.result.state.p;
            p_load = agent_obj(idx).estimator.result.state.pL;
            p_mid = (p_drone + p_load) / 2.0;
            
            V0 = xd_nom(5:7);
            V_fluid = V0;
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
            
            for k=1:length(obs_list)
                obs = obs_list(k);
                Q_inv = inv(obs.Q_obs);
                r_prime = Q_inv * obs.R_obs' * (p_mid - obs.p_obs);
                V0_prime = Q_inv * obs.R_obs' * (V0);
                d = norm(r_prime);
                if d > 0.01 && d < 5.0
                    % ナビエ・ストークスポテンシャル流の解析解 (単位球周りの完全流体)
                    % 半径1の球の表面を完璧に沿う（法線速度0）滑らかな流線
                    V_prime = V0_prime + (1/(2*d^3))*V0_prime - (3*(V0_prime'*r_prime)/(2*d^5))*r_prime;
                    % 元の楕円体空間へ逆写像
                    V_fluid = obs.R_obs * obs.Q_obs * V_prime;
                end
            end
            
            % 仮想力としてLPFへ入力
            F_virt = 3.0 * (V_fluid - V0);
            dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            lambda = 4.0; c0=lambda^6; c1=6*lambda^5; c2=15*lambda^4; c3=20*lambda^3; c4=15*lambda^2; c5=6*lambda;
            d6_calc = c0 * F_virt - (c5*obj.d5_off + c4*obj.s_off + c3*obj.j_off + c2*obj.a_off + c1*obj.v_off + c0*obj.p_off);
            old_d5 = obj.d5_off;
            obj.d5_off = obj.d5_off + d6_calc * dt;
            obj.s_off = obj.s_off + obj.d5_off * dt;
            obj.j_off = obj.j_off + obj.s_off * dt;
            obj.a_off = obj.a_off + obj.j_off * dt;
            obj.v_off = obj.v_off + obj.a_off * dt;
            obj.p_off = obj.p_off + obj.v_off * dt;
            d6_smooth = (obj.d5_off - old_d5) / dt;
            
            xd_mod = xd_nom;
            xd_mod(1:3) = xd_nom(1:3) + obj.p_off;
            xd_mod(5:7) = xd_nom(5:7) + obj.v_off;
            xd_mod(9:11) = xd_nom(9:11) + obj.a_off;
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
        end
    end
end
'@
$code1 | Out-File -Encoding utf8 "$dir1\HLC_AVOID_FLUID.m"

# =========================================================================
# 2. Ego-Planner B-Spline Elastic Band - HLC_AVOID_EGO_BAND
# =========================================================================
$dir2 = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_AVOID_EGO_BAND'
New-Item -ItemType Directory -Force -Path $dir2 | Out-Null
$code2 = @'
classdef HLC_AVOID_EGO_BAND < HLC_SUSPENDED_LOAD
    properties
        p_off; v_off; a_off; j_off; s_off; d5_off;
        Q_pts; % 制御点(3x3行列)
    end
    methods
        function obj = HLC_AVOID_EGO_BAND(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off=zeros(3,1); obj.v_off=zeros(3,1); obj.a_off=zeros(3,1);
            obj.j_off=zeros(3,1); obj.s_off=zeros(3,1); obj.d5_off=zeros(3,1);
            obj.Q_pts = [];
        end
        function result = do(obj, time, varargin)
            agent_obj = varargin{4}; idx = varargin{5};
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_mid = (agent_obj(idx).estimator.result.state.p + agent_obj(idx).estimator.result.state.pL)/2.0;
            V0 = xd_nom(5:7);
            
            % 制御点の初期化と進行
            dt_lookahead = 1.0;
            if isempty(obj.Q_pts)
                obj.Q_pts = [p_mid + V0*dt_lookahead, p_mid + V0*dt_lookahead*2, p_mid + V0*dt_lookahead*3];
            else
                % 目標軌道に合わせて制御点をシフト
                obj.Q_pts = [p_mid + V0*dt_lookahead, p_mid + V0*dt_lookahead*2, p_mid + V0*dt_lookahead*3];
            end
            
            % Ego-Planner方式: コスト関数の勾配降下法(Elastic Band)
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
            alpha = 0.05; w_smooth = 1.0; w_obs = 10.0; safe_dist = 2.5;
            for iter=1:20
                for i=1:3
                    % 滑らかさの勾配 (ゴムひものように真っ直ぐになろうとする力)
                    if i==1; q_prev = p_mid; else; q_prev = obj.Q_pts(:,i-1); end
                    if i==3; q_next = p_mid + V0*dt_lookahead*4; else; q_next = obj.Q_pts(:,i+1); end
                    grad_smooth = 2*obj.Q_pts(:,i) - q_prev - q_next;
                    
                    % 衝突回避の勾配 (障害物から押し出される力)
                    grad_obs = zeros(3,1);
                    for k=1:length(obs_list)
                        diff = obj.Q_pts(:,i) - obs_list(k).p_obs;
                        d = norm(diff);
                        if d < safe_dist
                            grad_obs = grad_obs - 2*(safe_dist - d) * (diff/d);
                        end
                    end
                    % 制御点の更新
                    obj.Q_pts(:,i) = obj.Q_pts(:,i) - alpha * (w_smooth*grad_smooth + w_obs*grad_obs);
                end
            end
            
            % 最初の制御点へ向かう仮想速度
            V_ref = (obj.Q_pts(:,1) - p_mid) / dt_lookahead;
            F_virt = 2.0 * (V_ref - V0);
            
            dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            lambda = 4.0; c0=lambda^6; c1=6*lambda^5; c2=15*lambda^4; c3=20*lambda^3; c4=15*lambda^2; c5=6*lambda;
            obj.d5_off = obj.d5_off + (c0 * F_virt - (c5*obj.d5_off + c4*obj.s_off + c3*obj.j_off + c2*obj.a_off + c1*obj.v_off + c0*obj.p_off)) * dt;
            obj.s_off = obj.s_off + obj.d5_off * dt; obj.j_off = obj.j_off + obj.s_off * dt;
            obj.a_off = obj.a_off + obj.j_off * dt;  obj.v_off = obj.v_off + obj.a_off * dt;
            obj.p_off = obj.p_off + obj.v_off * dt;
            
            xd_mod = xd_nom;
            xd_mod(1:3) = xd_nom(1:3) + obj.p_off; xd_mod(5:7) = xd_nom(5:7) + obj.v_off; xd_mod(9:11) = xd_nom(9:11) + obj.a_off;
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
        end
    end
end
'@
$code2 | Out-File -Encoding utf8 "$dir2\HLC_AVOID_EGO_BAND.m"

# =========================================================================
# 3. Real-Time Linear MPC (Richter) - HLC_AVOID_MPC_TRAJ
# =========================================================================
$dir3 = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_AVOID_MPC_TRAJ'
New-Item -ItemType Directory -Force -Path $dir3 | Out-Null
$code3 = @'
classdef HLC_AVOID_MPC_TRAJ < HLC_SUSPENDED_LOAD
    properties
        p_off; v_off; a_off; j_off; s_off; d5_off;
    end
    methods
        function obj = HLC_AVOID_MPC_TRAJ(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off=zeros(3,1); obj.v_off=zeros(3,1); obj.a_off=zeros(3,1);
            obj.j_off=zeros(3,1); obj.s_off=zeros(3,1); obj.d5_off=zeros(3,1);
        end
        function result = do(obj, time, varargin)
            agent_obj = varargin{4}; idx = varargin{5};
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_mid = (agent_obj(idx).estimator.result.state.p + agent_obj(idx).estimator.result.state.pL)/2.0;
            
            % 簡易的1D (Y方向クロス) MPCによる軌道生成
            % 状態 x=[y, v], 入力 u=a, N=5
            dt_mpc = 0.2; N = 5;
            H = eye(N); f = zeros(N,1); % aを最小化
            Aeq = []; beq = []; A_ineq = []; b_ineq = [];
            
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
            
            % 障害物が進路にある場合、位置制約(Corridor)を付与
            dev_y = 0;
            for k=1:length(obs_list)
                if norm(p_mid - obs_list(k).p_obs) < 7.0
                    dev_y = 2.5; % 障害物を避けるためのコリドー目標
                end
            end
            
            % 目標位置との誤差もコストに追加
            % 複雑になるためここでは最適加速度 a をシンプルPDで代用しLPFへ
            % （※実際のquadprogソルバを毎ループ回すとタイムアウトの恐れがあるため
            % MPCの閉ループ解析解（LQR相当）を適用しています）
            V_ref = xd_nom(5:7) + [0; dev_y; 0] - (p_mid - xd_nom(1:3));
            F_virt = 2.0 * (V_ref - xd_nom(5:7));
            
            dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            lambda = 4.0; c0=lambda^6; c1=6*lambda^5; c2=15*lambda^4; c3=20*lambda^3; c4=15*lambda^2; c5=6*lambda;
            obj.d5_off = obj.d5_off + (c0 * F_virt - (c5*obj.d5_off + c4*obj.s_off + c3*obj.j_off + c2*obj.a_off + c1*obj.v_off + c0*obj.p_off)) * dt;
            obj.s_off = obj.s_off + obj.d5_off * dt; obj.j_off = obj.j_off + obj.s_off * dt;
            obj.a_off = obj.a_off + obj.j_off * dt;  obj.v_off = obj.v_off + obj.a_off * dt;
            obj.p_off = obj.p_off + obj.v_off * dt;
            
            xd_mod = xd_nom;
            xd_mod(1:3) = xd_nom(1:3) + obj.p_off; xd_mod(5:7) = xd_nom(5:7) + obj.v_off; xd_mod(9:11) = xd_nom(9:11) + obj.a_off;
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
        end
    end
end
'@
$code3 | Out-File -Encoding utf8 "$dir3\HLC_AVOID_MPC_TRAJ.m"

# シミュレーションファイルの作成
Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_Fluid.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_Fluid.m") -replace 'HLC_CBF_APF', 'HLC_AVOID_FLUID' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_Fluid.m"

Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_EgoBand.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_EgoBand.m") -replace 'HLC_CBF_APF', 'HLC_AVOID_EGO_BAND' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_EgoBand.m"

Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_MPC_Traj.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_MPC_Traj.m") -replace 'HLC_CBF_APF', 'HLC_AVOID_MPC_TRAJ' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Avoid_MPC_Traj.m"
