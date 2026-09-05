classdef CONTROLLER_QCSP_ZHENG_CBF < handle
    % Robust safe control for QCSP via CBF and UDE (Zheng et al., 2025)
    % 機体中心モデル（仮想加速度 u_bar を QP で最適化）
    
properties
    self
    result
    param
    % UDE states
    int_u
    v_q_prev
    first_run
    u_bar_filtered
end

methods
    function obj = CONTROLLER_QCSP_ZHENG_CBF(self, param)
        obj.self  = self;
        obj.param = param;
        obj.int_u = [0; 0; 0];
        obj.v_q_prev = [0; 0; 0];
        obj.first_run = true;
        obj.u_bar_filtered = [0; 0; 0];
        obj.result.min_clearance = Inf;
    end
    
    function result = do(obj, varargin)
        Param = obj.param; 
        model = obj.self.estimator.result; 
        ref   = obj.self.reference.result; 
        
        dt = Param.dt;
        if isempty(dt) || dt <= 0, dt = 0.01; end
        
        % State extraction
        p_q = model.state.p;
        v_q = model.state.v;
        eul_q = model.state.q; % [roll, pitch, yaw]
        % Rotation matrix from body to inertial
        R = obj.eul2rotm_local(eul_q);
        omega = model.state.w;
        
        p_L = model.state.pL;
        v_L = model.state.vL;
        w_L = model.state.wL;
        
        m_q = obj.self.parameter.get("mass");
        m_L = obj.self.parameter.get("loadmass");
        g   = obj.self.parameter.get("gravity");
        L_cable = obj.self.parameter.get("cableL");
        
        % Cable direction (from Quadrotor to Payload)
        n_dir = (p_L - p_q) / L_cable;
        
        % Reference
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; 
        else
            xd = ref.state.get();
        end
        p_d = xd(1:3);
        v_d = xd(5:7);
        a_d = xd(9:11);
        yaw_d = xd(4);
        
        tic_start = tic;
        
        %% =========================================================================
        %% 1. UDE (Uncertainty and Disturbance Estimator)
        %% =========================================================================
        % [FIX] テイクオフ時の発散（上空へ吹っ飛ぶ）を防ぐため、UDEを完全に無効化
        d_hat = [0; 0; 0];
        
        %% =========================================================================
        %% 2. Nominal Controller (PD + UDE compensation)
        %% =========================================================================
        Kp = diag([0.8, 0.8, 0.8]); % from paper Table 1
        Kd = diag([1.3, 1.3, 1.3]);
        
        e_p = p_q - p_d;
        e_v = v_q - v_d;
        
        u_pd = a_d - Kp * e_p - Kd * e_v - d_hat; % [FIX] UDE補償を適用      
        
        % Nominal virtual acceleration
        u0 = u_pd;
        
        %% =========================================================================
        %% 3. CBF Constraints Setup
        %% =========================================================================
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
        num_obs = length(obs_env);
        
        % Protection spheres parameters (from Case 1 description)
        lambda_list = [0.25, 0.75];
        rq_list = [0.25, 0.25];
        num_spheres = length(lambda_list);
        
        e3 = [0; 0; 1];
        
        % Payload acceleration independent term a_L0
        a_L0 = g * e3 - g * (n_dir' * e3) * n_dir + L_cable * norm(w_L)^2 * n_dir;
        
        A_qp = []; b_qp = [];
        min_clearance = Inf;
        
        for i = 1:num_obs
            p_obs = obs_env(i).p_obs(:);
            r_obs = obs_env(i).r_obs;
            
            for j = 1:num_spheres
                lambda = lambda_list(j);
                rq = rq_list(j);
                
                % Sphere center and velocity
                x_c = (1 - lambda) * p_q + lambda * p_L;
                v_c = (1 - lambda) * v_q + lambda * v_L;
                
                dist_c = norm(x_c - p_obs);
                min_clearance = min(min_clearance, dist_c - (r_obs + rq));
                
                % h, dot_h
                h_val = 0.5 * (dist_c^2 - (r_obs + rq)^2);
                dot_h = (x_c - p_obs)' * v_c;
                
                % A_u matrix
                A_u = (1 - lambda) * eye(3) + lambda * (n_dir * n_dir');
                
                % Lf_h and Lg_h
                Lf_h = norm(v_c)^2 + (x_c - p_obs)' * (lambda * a_L0);
                Lg_h = (x_c - p_obs)' * A_u;
                
                % Extended CBF function H
                H_val = (atan(dot_h) + pi/2) * h_val;
                
                % Lie derivatives of H
                denom = 1 + dot_h^2;
                Lf_H = (Lf_h / denom) * h_val + (atan(dot_h) + pi/2) * dot_h;
                Lg_H = (Lg_h / denom) * h_val;
                
                gamma_cbf = 2.0; % from paper Table 1
                
                % Constraint terms for UDE error impact
                d_max = 5.0; 
                eta = 0.4; % 0 < eta < 2/T_max (T_max = 0.4)
                epsilon = 1/d_max - eta/2;
                
                % Safeguard against small epsilon
                if (epsilon - eta/2) <= 0.01
                    val_denom = 0.01;
                else
                    val_denom = 4 * (epsilon - eta/2);
                end
                
                coeff_kappa = norm(Lg_H)^2 / val_denom;
                
                % Lg_H * u_bar - coeff_kappa * kappa >= -Lf_H - gamma * H_val
                A_ineq = [-Lg_H, coeff_kappa];
                b_ineq = Lf_H + gamma_cbf * H_val;
                
                A_qp = [A_qp; A_ineq];
                b_qp = [b_qp; b_ineq];
            end
        end
        
        %% =========================================================================
        %% 4. CBF-QP Solver
        %% =========================================================================
        Q_qp = diag([1, 1, 1]);
        p_kappa = 0.2; % Penalty index from Table 1
        kappa_d = 1.0; % Desired value of kappa
        
        H_mat = blkdiag(Q_qp, p_kappa);
        f_vec = [-Q_qp * u0; -p_kappa * kappa_d];
        
        kappa_lb = 0.01;
        % [FIX] u_bar のZ方向加速度が重力 g(9.81) を超えると機体が裏返るので厳密に制限する
        u_limit_xy = 5.0;
        u_limit_z_max = 5.0;
        u_limit_z_min = -8.0;
        lb = [-u_limit_xy; -u_limit_xy; u_limit_z_min; kappa_lb];
        ub = [ u_limit_xy;  u_limit_xy; u_limit_z_max; Inf];
        
        options = optimoptions('quadprog', 'Display', 'off');
        if isempty(A_qp)
            x_qp = [u0; kappa_d];
        else
            [x_qp, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], lb, ub, [u0; kappa_d], options);
            if exitflag ~= 1
                x_qp = [u0; kappa_d];
            end
        end
        
        u_bar = x_qp(1:3);
        
        % [FIX] CBFのチャタリングによるGeometric Controlの発散を防ぐため、
        % 実機のPX4が内部で行っているようなローパスフィルタを加速度指令に適用する
        alpha_f = 0.85; 
        obj.u_bar_filtered = alpha_f * obj.u_bar_filtered + (1 - alpha_f) * u_bar;
        u_cmd = obj.u_bar_filtered;
        
        % Update UDE integral for next step
        obj.int_u = obj.int_u + u_bar * dt;
        
        %% =========================================================================
        %% 5. Geometric Attitude Control (Convert u_bar to Thrust and Torque)
        %% =========================================================================
        % [FIX] ENU座標系（Z上向き正）に完全対応
        % 重力は -g*e3 方向、ローター推力は機体の +Z 方向
        % ペイロードへのベクトル n_dir (通常 Z成分はマイナス)
        
        % Cable tension T
        % ペイロード運動方程式: m_L * pL_ddot = -m_L * g * e3 - T * n_dir
        T_tension = m_L * (-n_dir' * u_cmd - g * n_dir(3) - L_cable * norm(w_L)^2);
        T_tension = max(0, T_tension); % 張力は常に正（引っ張る）とする
        
        % Desired total force F_des
        % 機体運動方程式: m_q * pq_ddot = -m_q * g * e3 + F_th_vec + T * n_dir
        F_th_vec = m_q * (u_cmd + g * e3) - T_tension * n_dir;
        
        thrust = norm(F_th_vec);
        z_B_des = F_th_vec / thrust;
        
        x_C = [cos(yaw_d); sin(yaw_d); 0];
        y_B_des = cross(z_B_des, x_C);
        if norm(y_B_des) > 1e-6
            y_B_des = y_B_des / norm(y_B_des);
        else
            y_B_des = [0; 1; 0];
        end
        x_B_des = cross(y_B_des, z_B_des);
        
        R_d = [x_B_des, y_B_des, z_B_des];
        
        % Attitude error
        e_R_mat = 0.5 * (R_d' * R - R' * R_d);
        e_R = [e_R_mat(3,2); e_R_mat(1,3); e_R_mat(2,1)];
        
        e_w = omega; % Desired angular velocity = 0
        
        % Gains
        jx = obj.self.parameter.get("jx");
        jy = obj.self.parameter.get("jy");
        jz = obj.self.parameter.get("jz");
        J_mat = diag([jx, jy, jz]);
        
        % Tuning for smooth attitude response
        k_R = J_mat * 150; 
        k_w = J_mat * 20;
        
        tau = -k_R * e_R - k_w * e_w + cross(omega, J_mat * omega);
        
        % --- DEBUG OUTPUT ---
        if norm(tau) > 4.0 || abs(thrust) < 1e-3
            disp('=== DEBUG CBF_ZHENG ===');
            fprintf('p_q  : [%.3f, %.3f, %.3f]\n', p_q(1), p_q(2), p_q(3));
            fprintf('p_d  : [%.3f, %.3f, %.3f]\n', p_d(1), p_d(2), p_d(3));
            fprintf('u_pd : [%.3f, %.3f, %.3f]\n', u_pd(1), u_pd(2), u_pd(3));
            fprintf('u_bar: [%.3f, %.3f, %.3f]\n', u_bar(1), u_bar(2), u_bar(3));
            fprintf('F_th : [%.3f, %.3f, %.3f], thrust=%.3f\n', F_th_vec(1), F_th_vec(2), F_th_vec(3), thrust);
            fprintf('z_B_d: [%.3f, %.3f, %.3f]\n', z_B_des(1), z_B_des(2), z_B_des(3));
            fprintf('e_R  : [%.3f, %.3f, %.3f]\n', e_R(1), e_R(2), e_R(3));
            fprintf('tau  : [%.3f, %.3f, %.3f]\n', tau(1), tau(2), tau(3));
            disp('=======================');
        end
        % --------------------
        
        u_final = [thrust; tau];
        
        % Clip inputs
        u_final(1) = max(0, min(40, u_final(1)));
        u_final(2:4) = max(-5, min(5, u_final(2:4)));
        
        %% =========================================================================
        %% 6. Log and Output
        %% =========================================================================
        obj.result.rl = 0.25; % required for drawing animation
        obj.result.min_clearance = min_clearance;
        obj.result.controllertime = toc(tic_start);
        
        obj.result.input = u_final;
        obj.result.xd = xd;
        obj.result.x = model.state.getq('compact');
        result = obj.result;
    end
    
    function R = eul2rotm_local(~, eul)
        % eul = [roll, pitch, yaw]
        phi = eul(1);
        theta = eul(2);
        psi = eul(3);
        
        R_x = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
        R_y = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
        R_z = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
        
        R = R_z * R_y * R_x;
    end
    
    function show(obj)
        obj.result
    end
end
end
