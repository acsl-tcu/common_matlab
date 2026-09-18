classdef HLC_ROTOR_LEVEL_DYNAMIC_CBF < handle
    properties
        base_controller
        self
        
        T_prev
        T_curr
        
        m, mL, cableL, jx, jy, jz, g
        L_arm = 0.13;
        c_tau = 0.015;
        
        f_min = 0.0;
        f_max = 5.0; 
        
        k0 = 16.0;
        k1 = 32.0;
        k2 = 24.0;
        k3 = 8.0;
        
        is_initialized = false;
        result
    end

    methods
        function obj = HLC_ROTOR_LEVEL_DYNAMIC_CBF(self, base_controller)
            obj.self = self;
            obj.base_controller = base_controller;
            
            obj.m = self.parameter.get("mass");
            obj.mL = self.parameter.get("loadmass");
            obj.cableL = self.parameter.get("cableL");
            obj.jx = self.parameter.get("jx");
            obj.jy = self.parameter.get("jy");
            obj.jz = self.parameter.get("jz");
            obj.g = 9.81;
            
            obj.T_curr = (obj.m + obj.mL) * obj.g;
            obj.T_prev = obj.T_curr;
            
            obj.result.state = STATE_CLASS(struct('state_list', ["p", "q", "v"], 'num_list', [3, 3, 3]));
        end

        function result = do(obj, varargin)
            time = varargin{1};
            dt   = time.dt;
            if isempty(dt) || dt <= 0, dt = 0.001; end

            base_res = obj.base_controller.do(varargin{:});
            u_nom = base_res.input; % [T_nom; tau_x; tau_y; tau_z]
            
            p_L = obj.self.estimator.result.state.pL;
            v_L = obj.self.estimator.result.state.vL;
            p_T = obj.self.estimator.result.state.pT;
            q_Q = obj.self.estimator.result.state.q;
            w_Q = obj.self.estimator.result.state.w;
            
            ol = [0;0;0]; 
            if isprop(obj.self.estimator.result.state, "wL")
                ol = obj.self.estimator.result.state.wL;
            end

            p_q = p_L - obj.cableL * p_T;
            v_q = v_L - obj.cableL * cross(ol, p_T);

            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
            catch
            end
            
            if length(q_Q) == 3
                q_Q_quat = Eul2Quat(q_Q);
            else
                q_Q_quat = q_Q;
            end
            
            z_state = [p_q; v_q; p_L; v_L; q_Q_quat; w_Q];
            dT_val = (obj.T_curr - obj.T_prev) / dt;
            T_state = [obj.T_curr; dT_val];
            params  = [obj.m; obj.mL; obj.cableL; obj.jx; obj.jy; obj.jz; obj.g];
            cbf_gains = [obj.k0; obj.k1; obj.k2; obj.k3];
            
            A_cbf_all = [];
            b_cbf_all = [];
            
            c_T = 1.0 / (0.5 * dt^2);
            
            for i = 1:length(obs_list)
                obs = obs_list(i);
                p_obs = obs.p_obs;
                r_obs = obs.r_obs_margin;
                obs_params = [p_obs; r_obs];
                
                try
                    [A_cbf, b_cbf] = SuspendedLoad_RealDynExt_CBF(z_state, T_state, params, obs_params, cbf_gains);
                    
                    % Map T_ddot to T_safe
                    A_cbf_T = A_cbf(1) * c_T;
                    A_cbf_tau = A_cbf(2:4);
                    b_cbf_mapped = b_cbf + A_cbf(1) * c_T * (obj.T_curr + dT_val * dt);
                    
                    A_cbf_all = [A_cbf_all; [A_cbf_T, A_cbf_tau, -1.0]];
                    b_cbf_all = [b_cbf_all; b_cbf_mapped];
                catch
                end
            end
            
            % Objective: Track u_nom = [T_nom; tau_nom]
            u_ref = u_nom;
            W_u = diag([1.0, 100.0, 100.0, 100.0]); % Penalize tau deviation heavily, T deviation slightly
            H = blkdiag(W_u, 1.0);
            f_qp = [-W_u * u_ref; 10000.0];
            
            M_alloc = obj.get_allocation_matrix();
            M_inv = inv(M_alloc);
            
            % Physical bounds: 0 <= M_inv * [T; tau] <= 5.0
            A_ineq_rot = [M_inv, zeros(4,1); -M_inv, zeros(4,1)];
            b_ineq_rot = [obj.f_max * ones(4,1); -obj.f_min * ones(4,1)];
            
            if ~isempty(A_cbf_all)
                A_ineq = [A_cbf_all; A_ineq_rot];
                b_ineq = [b_cbf_all; b_ineq_rot];
            else
                A_ineq = A_ineq_rot;
                b_ineq = b_ineq_rot;
            end
            
            % lb/ub for [T, tau_x, tau_y, tau_z, delta]
            lb = [0.1; -inf; -inf; -inf; 0.0];
            ub = [obj.f_max * 4.0; inf; inf; inf; inf];
            
            options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 2000);
            [U_opt, ~, exitflag] = quadprog(H, f_qp, A_ineq, b_ineq, [], [], lb, ub, [u_nom; 0], options);
            
            if exitflag ~= 1
                disp(['[CBF] QP Failed! exitflag = ', num2str(exitflag)]);
                U_opt = [u_nom; 0];
            end
            
            % Update states
            obj.T_prev = obj.T_curr;
            obj.T_curr = U_opt(1);
            
            obj.result.input = U_opt(1:4);
            result = obj.result;
        end

        function M = get_allocation_matrix(obj)
            L = obj.L_arm;
            c = obj.c_tau;
            M = [1, 1, 1, 1;
                 -L/sqrt(2),  L/sqrt(2),  L/sqrt(2), -L/sqrt(2);
                  L/sqrt(2), -L/sqrt(2),  L/sqrt(2), -L/sqrt(2);
                  c,  c, -c, -c];
        end
    end
end
