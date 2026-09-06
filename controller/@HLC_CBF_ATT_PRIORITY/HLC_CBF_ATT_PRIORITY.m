% classdef HLC_CBF_ATT_PRIORITY < HLC_SUSPENDED_LOAD
%     properties
%         p_off
%         v_off
%         a_off
%         j_off
%         s_off
%         d5_off
%     end
% 
%     methods
%         function obj = HLC_CBF_ATT_PRIORITY(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             obj.p_off  = [0;0;0];
%             obj.v_off  = [0;0;0];
%             obj.a_off  = [0;0;0];
%             obj.j_off  = [0;0;0];
%             obj.s_off  = [0;0;0];
%             obj.d5_off = [0;0;0];
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
%             p_curr = agent_obj(idx).estimator.result.state.p;
%             v_curr = agent_obj(idx).estimator.result.state.v;
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
% 
%             H = blkdiag(eye(3), 1e5); 
%             f = zeros(4,1);
%             A_ineq = []; b_ineq = [];
% 
%             A_max = 5.0; 
%             lb = [-A_max; -A_max; -A_max; 0];
%             ub = [ A_max;  A_max;  A_max; inf];
% 
%             % alphaを小さく(0.5)すると、より手前から強く反応し始めます
%             alpha = 0.5;
%             d_surf_min = 99.9;
%             r_safe = 2.5; % 案3はQPによる厳密な張り付きが起きるのでマージンをより厚く(2.5m)
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
%                 diff = p_curr - obs.p_obs;
%                 dist_raw = norm(diff) - obs.d_margin;
%                 if dist_raw < d_surf_min; d_surf_min = dist_raw; end
% 
%                 h_val = dist_raw - r_safe; 
% 
%                 if h_val < 10.0
%                     dir = diff / norm(diff);
%                     h_dot = dir' * v_curr;
%                     A_ineq = [A_ineq; -dir', -1];
%                     b_ineq = [b_ineq; h_dot + alpha * h_val];
%                 end
%             end
% 
%             opts = optimoptions('quadprog', 'Display', 'off');
%             if isempty(A_ineq)
%                 u_mod = [0;0;0];
%             else
%                 [U_opt, ~, eflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
%                 if eflag == 1; u_mod = U_opt(1:3); else; u_mod = [0;0;0]; end
%             end
% 
%             % ========================================================
%             % 【理論的完全解】CBF-QP出力の C^6 連続化と位置復元
%             % QPの出力を目標速度(v_cmd)と解釈し、平滑化用加速度(a_cmd)を生成
%             % ========================================================
%             if norm(u_mod) > 1e-4
%                 % 障害物回避時: QPの速度指示(u_mod)に追従するよう加速度を発生
%                 a_cmd = 3.0 * (u_mod - obj.v_off);
%             else
%                 % 安全時: 位置と速度のオフセットを0へ滑らかに復帰させる（バネダンパ）
%                 a_cmd = -3.0 * obj.v_off - 1.0 * obj.p_off;
%             end
% 
%             % 4次クリティカルダンピングフィルタによる C^6 連続性の確保
%             dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
%             lambda = 4.0;
%             c0 = lambda^4; c1 = 4 * lambda^3; c2 = 6 * lambda^2; c3 = 4 * lambda;
% 
%             d6_calc = c0 * a_cmd - (c3 * obj.d5_off + c2 * obj.s_off + c1 * obj.j_off + c0 * obj.a_off);
% 
%             old_d5_off = obj.d5_off;
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             d6_smooth = (obj.d5_off - old_d5_off) / dt;
% 
%             % ========================================================
%             % 参照軌道への合成 (28次元すべてに連続なオフセットを加算)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;   % Position
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;   % Velocity
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;   % Acceleration
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;   % Jerk
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;   % Snap
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;  % Pop
%             xd_mod(25:27) = xd_nom(25:27) + d6_smooth;   % Crackle
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% end

classdef HLC_CBF_ATT_PRIORITY < HLC_SUSPENDED_LOAD
    % =========================================================================
    % 未来予測型 2階高次制御バリア関数 (Predictive-HOCBF-QP)
    % 6次アドミタンスフィルタの伝達遅延 (tau_pred) を先読み補正し、
    % 荷物・機体の動的侵入を数学的に完全ゼロ化
    % =========================================================================
    properties
        p_off   % 0th (Position)
        v_off   % 1st (Velocity)
        a_off   % 2nd (Acceleration)
        j_off   % 3rd (Jerk)
        s_off   % 4th (Snap)
        d5_off  % 5th (Crackle)
    end
    
    methods
        function obj = HLC_CBF_ATT_PRIORITY(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off  = zeros(3, 1);
            obj.v_off  = zeros(3, 1);
            obj.a_off  = zeros(3, 1);
            obj.j_off  = zeros(3, 1);
            obj.s_off  = zeros(3, 1);
            obj.d5_off = zeros(3, 1);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            
            p_curr  = agent_obj(idx).estimator.result.state.p;
            v_curr  = agent_obj(idx).estimator.result.state.v;
            pL_curr = agent_obj(idx).estimator.result.state.pL;
            
            if isfield(agent_obj(idx).estimator.result.state, 'vL')
                vL_curr = agent_obj(idx).estimator.result.state.vL;
            else
                vL_curr = v_curr;
            end
            
            % ========================================================
            % 【未来予測ホライズンの設定】
            % 6次クリティカルダンピングフィルタ (lambda = 3.5) の等価時定数遅延
            % ========================================================
            lambda = 3.5;
            tau_pred = 0.55; % 0.55秒先の到達可能位置を先読み評価
            
            % 未来予測位置の算出 (現在位置 + 速度*tau + 0.5*加速度*tau^2)
            p_pred  = p_curr  + v_curr  * tau_pred + 0.5 * obj.a_off * (tau_pred^2);
            pL_pred = pL_curr + vL_curr * tau_pred + 0.5 * obj.a_off * (tau_pred^2);
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            d_surf_min = 99.9;
            d_drone_min = 99.9;
            d_load_min = 99.9;
            
            A_ineq = [];
            b_ineq = [];
            a_des_circ = zeros(3, 1);
            
            omega_cbf = 2.5;
            K_v = 2.0 * omega_cbf;
            K_p = omega_cbf^2;
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                Q_inv = inv(obs.Q_obs);
                
                % 診断用（現在位置での実クリアランス計測）
                diff_d = obs.R_obs' * (p_curr - obs.p_obs);
                d_d_raw = sqrt(diff_d' * (Q_inv^2) * diff_d) - 1.0 - obs.d_margin;
                if d_d_raw < d_drone_min; d_drone_min = d_d_raw; end
                
                diff_L = obs.R_obs' * (pL_curr - obs.p_obs);
                d_L_raw = sqrt(diff_L' * (Q_inv^2) * diff_L) - 1.0 - obs.d_margin;
                if d_L_raw < d_load_min; d_load_min = d_L_raw; end
                
                d_surf_min = min([d_surf_min, d_d_raw, d_L_raw]);
                
                % 【未来予測点でのCBF評価】
                pred_pts  = [p_pred, pL_pred];
                pred_vels = [v_curr, vL_curr];
                r_safes   = [0.65, 0.45];
                
                for p_i = 1:2
                    pt = pred_pts(:, p_i);
                    vt = pred_vels(:, p_i);
                    r_safe = r_safes(p_i);
                    
                    diff_local = obs.R_obs' * (pt - obs.p_obs);
                    d_pred_raw = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0 - obs.d_margin;
                    h_val = d_pred_raw - r_safe;
                    
                    % 検出ホライズン
                    if h_val < 4.5
                        grad_local = (Q_inv^2) * diff_local;
                        dir_w = obs.R_obs * grad_local;
                        norm_d = norm(dir_w);
                        if norm_d > 1e-4; n_vec = dir_w / norm_d; else; n_vec = [1; 0; 0]; end
                        
                        % 3次元接線流線
                        tangent_dir = cross(n_vec, cross(xd_nom(5:7), n_vec));
                        if norm(tangent_dir) < 0.1 || abs(n_vec(3)) > 0.85
                            tangent_dir = cross(n_vec, [0; 1; 0]);
                            if norm(tangent_dir) < 0.1; tangent_dir = cross(n_vec, [1; 0; 0]); end
                        end
                        tangent_dir = tangent_dir / norm(tangent_dir);
                        
                        v_circ_target = 2.2 * tangent_dir;
                        a_des_circ = a_des_circ + 4.0 * (v_circ_target - vt);
                        
                        % Hessian 曲率補正
                        M_mat = obs.R_obs * (Q_inv^2) * obs.R_obs';
                        H_ellip = (eye(3) - n_vec * n_vec') * M_mat / max(0.1, norm_d);
                        h_curv = vt' * H_ellip * vt;
                        
                        h_dot = n_vec' * vt;
                        
                        % 【未来予測型ハードCBF制約】
                        A_ineq = [A_ineq; -n_vec'];
                        b_ineq = [b_ineq; K_v * h_dot + K_p * h_val + h_curv];
                    end
                end
            end
            
            % ========================================================
            % ハード QP 求解 (スラックなし)
            % ========================================================
            H = eye(3);
            f = -a_des_circ;
            A_max = 6.0;
            lb = -A_max * ones(3, 1);
            ub =  A_max * ones(3, 1);
            
            opts = optimoptions('quadprog', 'Display', 'off');
            if isempty(A_ineq)
                a_qp = zeros(3, 1);
            else
                [U_opt, ~, eflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], opts);
                if eflag == 1
                    a_qp = U_opt;
                else
                    a_qp = a_des_circ;
                    if norm(a_qp) > A_max; a_qp = A_max * (a_qp / norm(a_qp)); end
                end
            end
            
            % ========================================================
            % 6次完全一貫アドミタンス補償器
            % ========================================================
            norm_aqp = norm(a_qp);
            if norm_aqp > 1e-4
                a_qp_smooth = A_max * tanh(norm_aqp / A_max) * (a_qp / norm_aqp);
            else
                a_qp_smooth = zeros(3, 1);
            end
            
            F_cmd = a_qp_smooth;
            dt = 0.025;
            if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            
            c0 = lambda^6;
            c1 = 6.0  * lambda^5;
            c2 = 15.0 * lambda^4;
            c3 = 20.0 * lambda^3;
            c4 = 15.0 * lambda^2;
            c5 = 6.0  * lambda;
            
            k_spring = lambda^2;
            d6_calc = c0 * (F_cmd / k_spring) - ...
                      (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
                       c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
            
            obj.d5_off = obj.d5_off + d6_calc    * dt;
            obj.s_off  = obj.s_off  + obj.d5_off * dt;
            obj.j_off  = obj.j_off  + obj.s_off  * dt;
            obj.a_off  = obj.a_off  + obj.j_off  * dt;
            obj.v_off  = obj.v_off  + obj.a_off  * dt;
            obj.p_off  = obj.p_off  + obj.v_off  * dt;
            
            % 診断ログ出力
            t_now = 0;
            if isprop(time, 't'); t_now = time.t; end
            if d_surf_min < 1.2
                fprintf('[PRED t=%.2f] d_surf: %+.3f (Drone:%+.2f, Load:%+.2f) | |a_qp|: %.2f | a_off: [%.2f, %.2f, %.2f] | p_off: [%.2f, %.2f, %.2f]\n', ...
                    t_now, d_surf_min, d_drone_min, d_load_min, norm_aqp, ...
                    obj.a_off(1), obj.a_off(2), obj.a_off(3), ...
                    obj.p_off(1), obj.p_off(2), obj.p_off(3));
            end
            
            % 修正参照軌道の合成
            xd_mod = xd_nom;
            xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
            xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
            xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
            xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
            xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
            xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
            xd_mod(25:27) = xd_nom(25:27) + d6_calc;
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end