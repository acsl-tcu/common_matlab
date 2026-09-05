classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
    methods
        function obj = HLC_CBF_APF(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            p_curr = agent_obj(idx).estimator.result.state.p;
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            F_rep = [0;0;0];
            k_rep = 5.0; d_inf = 8.0;
            
            d_surf_min = 99.9; % ロガー用
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                diff = p_curr - obs.p_obs;
                dist = norm(diff) - obs.d_margin;
                
                if dist < d_surf_min; d_surf_min = dist; end
                
                if dist > 0.1 && dist < d_inf
                    dir = diff / norm(diff);
                    force = k_rep * (1/dist - 1/d_inf) * (1/dist^2);
                    if force > 10.0; force = 10.0; end
                    
                    if abs(dir(3)) > 0.8
                        dir(1) = dir(1) + 0.5; dir(2) = dir(2) + 0.2; dir = dir / norm(dir);
                    end
                    F_rep = F_rep + force * dir;
                end
            end
            
            xd_mod = xd_nom;
            xd_mod(5:7) = xd_nom(5:7) + F_rep;
            xd_mod(9:11) = xd_nom(9:11) + F_rep;
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            % d_surf を追加
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end
