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
