classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
    properties
        p_off % 仮想バネによる位置オフセット
        v_off % 仮想バネによる速度オフセット
    end
    
    methods
        function obj = HLC_CBF_APF(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off = [0;0;0];
            obj.v_off = [0;0;0];
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_curr = agent_obj(idx).estimator.result.state.p;
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            F_rep = [0;0;0];
            k_rep = 5.0;  
            d_inf = 6.0;  
            
            d_surf_min = 99.9;
            r_safe = 1.0; 
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                diff_local = obs.R_obs' * (p_curr - obs.p_obs);
                Q_inv = inv(obs.Q_obs);
                
                d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
                dist_raw = d_ellip - obs.d_margin;
                if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                
                dist_eff = dist_raw - r_safe;
                if dist_eff < 0.05; dist_eff = 0.05; end
                
                if dist_eff < d_inf
                    grad_local = (Q_inv^2) * diff_local;
                    dir = obs.R_obs * grad_local;
                    dir = dir / norm(dir);
                    
                    if abs(dir(3)) > 0.95
                        [~, min_axis_idx] = min(diag(obs.Q_obs));
                        breaker_local = zeros(3,1);
                        breaker_local(min_axis_idx) = 0.1; % 僅かな力で短い軸へ誘導
                        dir = dir + obs.R_obs * breaker_local;
                        dir = dir / norm(dir);
                    end
                    
                    force = k_rep * (1/dist_eff - 1/d_inf) * (1/dist_eff^2);
                    if force > 10.0; force = 10.0; end % 斥力上限
                    F_rep = F_rep + force * dir;
                end
            end
            
            % ★ アドミタンス・フィルタ（仮想バネ・ダンパ系）による滑らかな軌道生成 ★
            % M * a_off + D * v_off + K * p_off = F_rep
            dt = 0.025;
            if isprop(time, 'dt'); dt = time.dt; end
            
            % D=3.0(強めのダンパで振動防止), K=1.0(バネ)
            a_off = F_rep - 3.0 * obj.v_off - 1.0 * obj.p_off;
            
            obj.v_off = obj.v_off + a_off * dt;
            obj.p_off = obj.p_off + obj.v_off * dt;
            
            xd_mod = xd_nom;
            % 完全に数学的に連続(C2級)なオフセットを加算
            xd_mod(1:3) = xd_nom(1:3) + obj.p_off;
            xd_mod(5:7) = xd_nom(5:7) + obj.v_off;
            xd_mod(9:11) = xd_nom(9:11) + a_off;
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end
