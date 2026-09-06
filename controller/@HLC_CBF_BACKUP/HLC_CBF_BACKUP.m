classdef HLC_CBF_BACKUP < HLC_SUSPENDED_LOAD
    properties
        p_off
        v_off
    end
    
    methods
        function obj = HLC_CBF_BACKUP(self, param)
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
            v_curr = agent_obj(idx).estimator.result.state.v;
            pL_curr = agent_obj(idx).estimator.result.state.pL;
            
            dec_max = 2.0; 
            t_brake = norm(v_curr) / dec_max;
            if t_brake < 0.5; t_brake = 0.5; end
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            danger = false;
            d_surf_min = 99.9;
            
            % ========================================================
            % 【形状近似の切り替え】
            % 1: 複数球近似 (Multi-Sphere)
            % 2: 楕円体近似 (Ellipsoid)
            APPROX_TYPE = 2; 
            % ========================================================
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                
                if APPROX_TYPE == 1
                    protect_curr = [p_curr, (p_curr+pL_curr)/2.0, pL_curr];
                    protect_stop = [p_curr + v_curr * t_brake, (p_curr+pL_curr)/2.0 + v_curr * t_brake, pL_curr + v_curr * t_brake];
                    r_safe = 0.6; 
                    Q_inv = inv(obs.Q_obs);
                    
                    for p_i = 1:3
                        diff_local = obs.R_obs' * (protect_curr(:, p_i) - obs.p_obs);
                        dist_raw = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0 - obs.d_margin;
                        if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                        
                        diff_stop = obs.R_obs' * (protect_stop(:, p_i) - obs.p_obs);
                        dist_stop = sqrt(diff_stop' * (Q_inv^2) * diff_stop) - 1.0 - obs.d_margin;
                        if dist_stop < r_safe; danger = true; end
                    end
                    
                elseif APPROX_TYPE == 2
                    p_center_curr = (p_curr + pL_curr) / 2.0;
                    p_center_stop = p_center_curr + v_curr * t_brake;
                    L_str = norm(p_curr - pL_curr);
                    r_xy = 0.6; r_z = (L_str / 2.0) + 0.3;
                    
                    Q_eff = obs.Q_obs + diag([r_xy, r_xy, r_z]);
                    Q_inv_eff = inv(Q_eff);
                    
                    diff_local = obs.R_obs' * (p_center_curr - obs.p_obs);
                    dist_raw = sqrt(diff_local' * (Q_inv_eff^2) * diff_local) - 1.0 - obs.d_margin;
                    if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                    
                    diff_stop = obs.R_obs' * (p_center_stop - obs.p_obs);
                    dist_stop = sqrt(diff_stop' * (Q_inv_eff^2) * diff_stop) - 1.0 - obs.d_margin;
                    if dist_stop < 0.0; danger = true; end % r_safeはQ_effに内包済み
                end
            end
            
            dt = 0.025; if isprop(time, 'dt'); dt = time.dt; end
            if danger
                v_des_off = -xd_nom(5:7);
                a_off = 3.0 * (v_des_off - obj.v_off);
                if norm(a_off) > dec_max; a_off = dec_max * (a_off / norm(a_off)); end
            else
                a_off = -3.0 * obj.v_off - 1.0 * obj.p_off;
            end
            
            obj.v_off = obj.v_off + a_off * dt;
            obj.p_off = obj.p_off + obj.v_off * dt;
            
            xd_mod = xd_nom;
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

