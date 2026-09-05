classdef HLC_CBF_ATT_PRIORITY < HLC_SUSPENDED_LOAD
    methods
        function obj = HLC_CBF_ATT_PRIORITY(self, param)
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
            
            H = blkdiag(eye(3), 1e5); 
            f = zeros(4,1);
            A_ineq = []; b_ineq = [];
            
            A_max = 5.0; 
            lb = [-A_max; -A_max; -A_max; 0];
            ub = [ A_max;  A_max;  A_max; inf];
            
            % alphaを小さく(0.5)すると、より手前から強く反応し始めます
            alpha = 0.5;
            d_surf_min = 99.9;
            r_safe = 2.5; % 案3はQPによる厳密な張り付きが起きるのでマージンをより厚く(2.5m)
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                diff = p_curr - obs.p_obs;
                dist_raw = norm(diff) - obs.d_margin;
                if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                
                h_val = dist_raw - r_safe; 
                
                if h_val < 10.0
                    dir = diff / norm(diff);
                    h_dot = dir' * v_curr;
                    A_ineq = [A_ineq; -dir', -1];
                    b_ineq = [b_ineq; h_dot + alpha * h_val];
                end
            end
            
            opts = optimoptions('quadprog', 'Display', 'off');
            if isempty(A_ineq)
                u_mod = [0;0;0];
            else
                [U_opt, ~, eflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
                if eflag == 1; u_mod = U_opt(1:3); else; u_mod = [0;0;0]; end
            end
            
            xd_mod = xd_nom;
            % 速度だけでなく加速度指示にも大きく反映させて物理的にすぐブレーキをかけさせる
            xd_mod(5:7) = xd_nom(5:7) + u_mod; 
            xd_mod(9:11) = xd_nom(9:11) + u_mod * 3.0; 
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end
