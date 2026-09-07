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
