classdef HLC_CBF_BACKUP < HLC_SUSPENDED_LOAD
    methods
        function obj = HLC_CBF_BACKUP(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_curr = agent_obj(idx).estimator.result.state.p;
            v_curr = agent_obj(idx).estimator.result.state.v;
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            
            dec_max = 2.0; % マージンを広く取ったので緩やかな減速(2.0m/s^2)で安全に止まれる
            t_brake = norm(v_curr) / dec_max;
            if t_brake < 0.5; t_brake = 0.5; end
            p_stop = p_curr + v_curr * t_brake;
            
            danger = false;
            d_surf_min = 99.9;
            r_safe = 2.0;
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                dist_raw = norm(p_curr - obs.p_obs) - obs.d_margin;
                if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                
                dist_stop = norm(p_stop - obs.p_obs) - obs.d_margin;
                if dist_stop < r_safe % 停止位置が実質的な安全マージン内に入ったら危険
                    danger = true;
                end
            end
            
            xd_mod = xd_nom;
            if danger
                v_dir = v_curr / (norm(v_curr) + 1e-6);
                xd_mod(5:7) = [0;0;0];
                xd_mod(9:11) = -dec_max * v_dir;
                xd_mod(13:28) = 0;
            end
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end
