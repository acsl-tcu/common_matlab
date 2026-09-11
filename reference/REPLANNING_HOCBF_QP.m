classdef REPLANNING_HOCBF_QP < handle
    % REPLANNING_HOCBF_QP
    % 案4: 高次CBF (HOCBF) を用いた多項式軌道係数のリアルタイムQPフィルタ
    % 相対次数 m=2 (加速度レベル) の HOCBF を公称軌道周りで線形化し、
    % 13次多項式 (Minimum Snap) の係数 C を決定する quadprog の不等式制約として組み込む。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0
        
        obs_center   = [0; 0; 6.0]
        obs_radius   = 0.3
        safe_margin  = 0.4
        trigger_dist = 3.5
        
        L_cable = 2.0
        poly_coeffs 
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        result
    end
    
    methods
        function obj = REPLANNING_HOCBF_QP(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center(:);   end
            if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;      end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;     end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist;    end
            
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
            else
                pL_cur = base_res.state.p;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[HOCBF QP REPLAN] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 1.2;
                    obj.t_duration = max(7.0, min(14.0, total_dist / spd));
                    
                    p0_ref = xd_nominal(1:3);
                    v0_ref = xd_nominal(5:7);
                    
                    fprintf("[HOCBF QP] 線形化高次CBFフィルタ制約 QP の生成・実行...\n");
                    obj.plan_hocbf_qp_trajectory(xd_nominal, dir_nom, spd, total_dist, p0_ref, v0_ref);
                    
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            if obj.replan_active
                tau = time.t - obj.t_start;
                if tau <= obj.t_duration
                    xd = obj.evaluate_smooth_trajectory(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[HOCBF QP REPLAN] 障害物クリア・C^6シームレス合流 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        xd_end = obj.evaluate_smooth_trajectory(obj.t_duration, xd_nominal);
                        obj.p_merge_end = xd_end(1:3);
                        obj.v_merge_vec = xd_end(5:7);
                        obj.replan_active = false;
                        obj.replan_done   = true;
                    end
                    dt_after = time.t - obj.t_merge_end;
                    xd = zeros(28, 1);
                    xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                    xd(5:7) = obj.v_merge_vec;
                    xd(9:28) = 0;
                end
            elseif obj.replan_done
                dt_after = time.t - obj.t_merge_end;
                xd = zeros(28, 1);
                xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                xd(5:7) = obj.v_merge_vec;
                xd(9:28) = 0;
            else
                xd = xd_nominal;
            end
            
            if length(xd) < 28
                xd = [xd; zeros(28 - length(xd), 1)];
            end
            
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_hocbf_qp_trajectory(obj, xd0, dir_nom, spd, total_dist, p0_ref, v0_ref)
            T = obj.t_duration;
            p_end = p0_ref + dir_nom * total_dist;
            
            order = 13;
            n_coeffs = order + 1; % 14
            num_vars = 3 * n_coeffs; % [Cx; Cy; Cz] (42次元)
            
            % --- 1. 目的関数 (Minimum Snap & Crackle) ---
            H_1d = zeros(n_coeffs, n_coeffs);
            w4 = 1.0; 
            w5 = 0.1; 
            for i = 4:order
                for j = 4:order
                    val4 = (factorial(i)/factorial(i-4)) * (factorial(j)/factorial(j-4)) / (i + j - 7);
                    H_1d(i+1, j+1) = H_1d(i+1, j+1) + w4 * val4;
                    if i >= 5 && j >= 5
                        val5 = (factorial(i)/factorial(i-5)) * (factorial(j)/factorial(j-5)) / (i + j - 9);
                        H_1d(i+1, j+1) = H_1d(i+1, j+1) + w5 * val5;
                    end
                end
            end
            H = blkdiag(H_1d, H_1d, H_1d) + 1e-6 * eye(num_vars);
            f = zeros(num_vars, 1);
            
            % --- 2. 等式制約 (始端 0〜6階微分、終端 0〜5階微分) ---
            n_eq_1d = 13;
            Aeq_1d = zeros(n_eq_1d, n_coeffs);
            beq = zeros(3 * n_eq_1d, 1);
            
            for k = 0:6
                row = k + 1;
                for n = k:order
                    Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
            end
            for k = 0:5
                row = 8 + k;
                for n = k:order
                    Aeq_1d(row, n + 1) = prod(n - k + 1 : n) * (1.0)^(n - k);
                end
            end
            Aeq = blkdiag(Aeq_1d, Aeq_1d, Aeq_1d);
            
            xd0_real = xd0;
            xd0_real(1:3) = p0_ref;
            xd0_real(5:7) = v0_ref;
            
            xd1 = zeros(28, 1);
            xd1(1:3) = p_end;
            xd1(5:7) = dir_nom * spd;
            
            for axis = 1:3
                beq_axis = zeros(n_eq_1d, 1);
                for k = 0:6
                    beq_axis(k+1) = (T^k) * xd0_real(4*k + axis);
                end
                for k = 0:5
                    beq_axis(8+k) = (T^k) * xd1(4*k + axis);
                end
                beq( (axis-1)*n_eq_1d + 1 : axis*n_eq_1d ) = beq_axis;
            end
            
            % --- 3. 高次 CBF (HOCBF) 線形不等式制約 ---
            R_safe = obj.obs_radius + obj.safe_margin + 0.2;
            alpha1 = 2.0;
            alpha2 = 2.0;
            
            % HOCBFサンプリング点の生成
            N_sample = 20;
            Aineq = zeros(N_sample * 2, num_vars); % 荷物とドローン
            bineq = zeros(N_sample * 2, 1);
            
            % 公称軌道 (回避のためのガイド) を設定
            % (真っ直ぐ突っ込むと線形化時に勾配が消えるため、少し膨らませる)
            vec_to_obs = obj.obs_center - p0_ref;
            proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
            normal_vec = vec_to_obs - proj_on_line;
            if norm(normal_vec) < 1e-3
                if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                else, normal_vec = cross(dir_nom, [1; 0; 0]); end
            end
            dir_normal = normal_vec / norm(normal_vec);
            
            % 係数行ベクトルの生成関数 (Tスケーリング対応)
            T0_fn = @(u) arrayfun(@(n) u^n, 0:order);
            T1_fn = @(u) arrayfun(@(n) (n>=1)*n*u^max(0,n-1)/T, 0:order);
            T2_fn = @(u) arrayfun(@(n) (n>=2)*n*(n-1)*u^max(0,n-2)/T^2, 0:order);
            
            for m = 1:N_sample
                u_m = m / (N_sample + 1);
                
                % 公称軌道の評価点 (サインカーブ的な迂回軌道を仮定)
                dev = sin(pi * u_m) * (R_safe + 0.5);
                r_k = p0_ref + dir_nom * (total_dist * u_m) + dir_normal * dev;
                v_k = dir_nom * spd + dir_normal * (pi * cos(pi * u_m) * (R_safe + 0.5) * spd / total_dist);
                a_k = dir_normal * (-pi^2 * sin(pi * u_m) * (R_safe + 0.5) * (spd / total_dist)^2);
                
                % u_m での基底
                t0 = T0_fn(u_m);
                t1 = T1_fn(u_m);
                t2 = T2_fn(u_m);
                T0_mat = blkdiag(t0, t0, t0);
                T1_mat = blkdiag(t1, t1, t1);
                T2_mat = blkdiag(t2, t2, t2);
                
                % 荷物の HOCBF
                h2_val = 2*norm(v_k)^2 + 2*(r_k - obj.obs_center)'*a_k + ...
                         2*(alpha1 + alpha2)*(r_k - obj.obs_center)'*v_k + ...
                         alpha1*alpha2*(norm(r_k - obj.obs_center)^2 - R_safe^2);
                grad_r = 2*a_k' + 2*(alpha1 + alpha2)*v_k' + 2*alpha1*alpha2*(r_k - obj.obs_center)';
                grad_v = 4*v_k' + 2*(alpha1 + alpha2)*(r_k - obj.obs_center)';
                grad_a = 2*(r_k - obj.obs_center)';
                
                row_idx = (m-1)*2 + 1;
                Aineq(row_idx, :) = - (grad_r * T0_mat + grad_v * T1_mat + grad_a * T2_mat);
                bineq(row_idx)    = - (grad_r * r_k + grad_v * v_k + grad_a * a_k - h2_val);
                
                % ドローンの HOCBF (上空 L_cable)
                rD_k = r_k + [0; 0; obj.L_cable];
                vD_k = v_k;
                aD_k = a_k;
                
                h2D_val = 2*norm(vD_k)^2 + 2*(rD_k - obj.obs_center)'*aD_k + ...
                          2*(alpha1 + alpha2)*(rD_k - obj.obs_center)'*vD_k + ...
                          alpha1*alpha2*(norm(rD_k - obj.obs_center)^2 - R_safe^2);
                grad_rD = 2*aD_k' + 2*(alpha1 + alpha2)*vD_k' + 2*alpha1*alpha2*(rD_k - obj.obs_center)';
                grad_vD = 4*vD_k' + 2*(alpha1 + alpha2)*(rD_k - obj.obs_center)';
                grad_aD = 2*(rD_k - obj.obs_center)';
                
                row_idx2 = (m-1)*2 + 2;
                % ドローンの基底評価は r(u) + [0;0;L] なので、Cに対する微分係数は同じ
                % 定数項 [0;0;L] は b 側に移項される
                Aineq(row_idx2, :) = - (grad_rD * T0_mat + grad_vD * T1_mat + grad_aD * T2_mat);
                bineq(row_idx2)    = - (grad_rD * r_k + grad_vD * v_k + grad_aD * a_k - h2D_val) + grad_rD * [0; 0; obj.L_cable];
            end
            
            % --- 4. quadprog による係数の決定 ---
            options = optimoptions('quadprog', 'Display', 'off');
            [C_opt, fval, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, [], [], [], options);
            
            if exitflag ~= 1
                fprintf("    [WARNING] QP 最適解が保証されません (exitflag=%d)。制約を緩和します。\n", exitflag);
                bineq = bineq + 10.0;
                [C_opt, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, [], [], [], options);
            end
            
            fprintf("    [DEBUG] 高次CBF (HOCBF) QP 最適化完了 (ExitFlag: %d)\n", exitflag);
            
            obj.poly_coeffs = [C_opt(1:n_coeffs), C_opt(n_coeffs+1 : 2*n_coeffs), C_opt(2*n_coeffs+1 : end)];
        end
        
        function xd = evaluate_smooth_trajectory(obj, tau, xd_nom)
            xd = xd_nom;
            C = obj.poly_coeffs;
            order = 13;
            T = obj.t_duration;
            u = max(0, min(1.0, tau / T));
            
            for k = 0:6
                val_k = zeros(3, 1);
                for n = k:order
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(n + 1, :)' * factor * (u^(n - k));
                end
                xd(4*k + (1:3)) = val_k / (T^k);
            end
        end
    end
end
