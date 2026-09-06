classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
    properties
        p_off
        v_off
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
            pL_curr = agent_obj(idx).estimator.result.state.pL;
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            F_rep = [0;0;0];
            k_rep = 5.0;  
            d_inf = 4.0;  
            d_surf_min = 99.9;
            
            % ========================================================
            % 【形状近似の切り替え】
            % 1: 複数球近似 (Multi-Sphere) ... 機体と荷物に独立した球を配置
            % 2: 楕円体近似 (Ellipsoid) ... 紐の中心を原点とする縦長の楕円体で全体を包む
            APPROX_TYPE = 2; 
            % ========================================================
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                
                if APPROX_TYPE == 1
                    % --- 複数球近似 (Multi-Sphere) ---
                    protect_pts = [p_curr, (p_curr+pL_curr)/2.0, pL_curr];
                    r_safe = 0.6; % 各球の半径
                    Q_inv = inv(obs.Q_obs);
                    
                    for p_i = 1:3
                        pt = protect_pts(:, p_i);
                        diff_local = obs.R_obs' * (pt - obs.p_obs);
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
                                breaker_local = zeros(3,1); breaker_local(min_axis_idx) = 0.1; 
                                dir = dir + obs.R_obs * breaker_local; dir = dir / norm(dir);
                            end
                            force = k_rep * (1/dist_eff - 1/d_inf) * (1/dist_eff^2);
                            if force > 10.0; force = 10.0; end
                            F_rep = F_rep + force * dir;
                        end
                    end
                    
                elseif APPROX_TYPE == 2
                    % --- 楕円体近似 (Ellipsoid vs Ellipsoid のミンコフスキー和近似) ---
                    p_center = (p_curr + pL_curr) / 2.0; % 紐の中心
                    L_str = norm(p_curr - pL_curr);
                    r_xy = 0.6; % ドローンの横幅半径
                    r_z = (L_str / 2.0) + 0.3; % 縦の半径 (紐の長さ/2 + 荷物の余裕)
                    
                    % 障害物とドローンの楕円体を合成(ミンコフスキー和の対角近似)
                    Q_eff = obs.Q_obs + diag([r_xy, r_xy, r_z]);
                    Q_inv_eff = inv(Q_eff);
                    
                    diff_local = obs.R_obs' * (p_center - obs.p_obs);
                    d_ellip = sqrt(diff_local' * (Q_inv_eff^2) * diff_local) - 1.0;
                    dist_raw = d_ellip - obs.d_margin;
                    if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                    
                    dist_eff = dist_raw; % r_safeはQ_effに内包済み
                    if dist_eff < 0.05; dist_eff = 0.05; end
                    
                    if dist_eff < d_inf
                        grad_local = (Q_inv_eff^2) * diff_local;
                        dir = obs.R_obs * grad_local;
                        dir = dir / norm(dir);
                        if abs(dir(3)) > 0.95
                            [~, min_axis_idx] = min(diag(obs.Q_obs));
                            breaker_local = zeros(3,1); breaker_local(min_axis_idx) = 0.1; 
                            dir = dir + obs.R_obs * breaker_local; dir = dir / norm(dir);
                        end
                        force = k_rep * (1/dist_eff - 1/d_inf) * (1/dist_eff^2);
                        if force > 10.0; force = 10.0; end
                        F_rep = F_rep + force * dir;
                    end
                end
            end
            
            % アドミタンスフィルタによる滑らかなオフセット生成
            dt = 0.025; if isprop(time, 'dt'); dt = time.dt; end
            a_off = F_rep - 3.0 * obj.v_off - 1.0 * obj.p_off;
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

