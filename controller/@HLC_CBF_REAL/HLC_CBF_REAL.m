classdef HLC_CBF_REAL < HLC_SUSPENDED_LOAD
    % HLC_CBF_REAL: 実入力(推力・トルク)に対するカスケード型 CLF-CBF
    % 論文手法に倣い、L_g h=0 の問題を回避するため、
    % 位置制約を「姿勢(Attitude)に対するトルク制約」にマッピングしてQPを解く。
    
    properties
        APPROX_TYPE = 2;
        r_safe = 0.6;
        d_margin_base = 0.5;
        obs_list_cache;
    end
    
    methods
        function obj = HLC_CBF_REAL(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
        end
        
        function result = do(obj, varargin)
            % 1. ノミナル実入力の計算
            result_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
            u_nom = result_nom.input; % [T, Mx, My, Mz]
            
            % 【安全装置0】ノミナル入力の異常値クランプ
            % 誤差が大きすぎてベースコントローラが発狂した際、QPのコスト関数が桁あふれを起こすのを防ぐ
            % 物理限界 (推力: 0~20N, トルク: -1~1Nm) を念頭に、QPのターゲット自身を現実的な値にカットする
            % ※重要: 自由落下(推力0)になると機体は水平・姿勢制御力を完全に失う（特異点）。最低5.0Nは維持させる
            u_nom(1) = max(5.0, min(20.0, u_nom(1)));       % 推力の物理限界＋特異点回避
            u_nom(2:4) = max(-1.0, min(1.0, u_nom(2:4))); % トルクの物理限界
            
            model = obj.self.estimator.result;
            p_curr = model.state.p;
            v_curr = model.state.v;
            pL_curr = model.state.pL;
            q_curr = model.state.q;
            w_curr = model.state.w; % 角速度 [wx, wy, wz]
            
            % ドローンの回転行列と現在の推力軸(Z軸)
            roll = q_curr(1); pitch = q_curr(2); yaw = q_curr(3);
            R_x = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
            R_y = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            R_curr = R_z * R_y * R_x;
            z_B = R_curr(:,3); % 機体の上方向ベクトル
            
            if isempty(obj.obs_list_cache)
                obj.obs_list_cache = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            end
            
            d_surf_min = inf;
            F_rep = [0; 0; 0];
            
            % --- 障害物からの斥力計算 ---
            for i = 1:length(obj.obs_list_cache)
                obs = obj.obs_list_cache(i);
                p_center = (p_curr + pL_curr) / 2.0;
                L_str = norm(p_curr - pL_curr);
                r_xy = obj.r_safe; r_z = (L_str / 2.0) + 0.3;
                Q_eff = obs.Q_obs + diag([r_xy, r_xy, r_z]);
                Q_inv_eff = inv(Q_eff);
                
                diff_local = obs.R_obs' * (p_center - obs.p_obs);
                d_ellip = sqrt(diff_local' * (Q_inv_eff^2) * diff_local) - 1.0;
                dist_raw = d_ellip - obs.d_margin;
                
                if dist_raw < d_surf_min; d_surf_min = dist_raw; end
                
                % 接近時、強い斥力(加速度指令)を発生させる
                if dist_raw < 6.0 && dist_raw > 0.01
                % 楕円体の法線ベクトルを計算
                n_vec = obs.R_obs * (Q_inv_eff^2) * diff_local / norm((Q_inv_eff^2) * diff_local);
                
                % 接近速度
                v_app = dot(v_curr, n_vec);
                
                % 接近している場合のみ、速度に比例した減衰（ダンパ）を優しくかける
                if v_app < 0
                    F_rep = F_rep + 2.0 * abs(v_app) * (1/dist_raw) * n_vec;
                end
                
                % 距離に応じた斥力（ポテンシャル）を手前から非常に優しくかける
                F_rep = F_rep + 3.0 * (1/dist_raw - 1/4.0) * n_vec;
            end    
            end
            
            % 【安全装置1】斥力ベクトルの物理的キャップ (無限大発散による要求推力飽和を防ぐ)
            m = obj.self.parameter.get("mass");
            g = obj.self.parameter.get("gravity");
            J = diag(obj.self.parameter.get(["jx", "jy", "jz"]));
            
            max_accel = 6.0; % 最大加速度要求を 12 m/s^2 に制限
            if norm(F_rep) > max_accel
                F_rep = (F_rep / norm(F_rep)) * max_accel;
            end
            
            % ノミナルの力ベクトル (ノミナル推力 T と機体Z軸で表現)
            F_nom = u_nom(1) * z_B; 
            % 重力補償込みの要求回避ベクトル
            F_des = F_nom + m * F_rep; 
            
            % 要求される安全な推力 (Z軸成分)
            T_safe_req = norm(F_des);
            % 要求される安全な姿勢 (Z軸の向き)
            if T_safe_req < 1e-3
                z_des = [0; 0; 1]; % 要求推力がほぼゼロの時は水平維持
            else
                z_des = F_des / T_safe_req;
            end
            
            % 【安全装置4】横転防止（Maximum Tilt Limit）
            % 【最重要】懸架荷物が揺れて姿勢を引き戻せなくなるのを防ぐため、
            % 傾きを「最大15度」相当に超厳しく制限する！
            % 30度等の急角度は懸架荷物システムでは回復不能になる
            max_tilt_cos = cos(deg2rad(15));
            if z_des(3) < max_tilt_cos
                z_xy = z_des(1:2);
                z_xy = z_xy / (norm(z_xy) + 1e-6) * sin(deg2rad(15));
                z_des = [z_xy; max_tilt_cos];
                z_des = z_des / norm(z_des);
            end
            
            % 現在の姿勢と要求姿勢の誤差ベクトル (外積で回転軸を抽出)
            e_R = cross(z_B, z_des); 
            
            % --- QPによる実入力 u=[T, Mx, My, Mz] の最適化 ---
            H = diag([1, 10, 10, 10, 1e6]); % u と slack
            f_qp = [-10*T_safe_req; -100*u_nom(2); -100*u_nom(3); -100*u_nom(4); 0];
            
            A_ineq = []; b_ineq = [];
            
            if norm(e_R) > 0.01
                % CLF-CBF 姿勢制約: トルク M が角加速度(J^-1 M)を生み、
                % それが姿勢誤差 e_R を減少させる方向(内積が正)に働くこと。
                % e_R^T * \dot{\omega} >= k * ||e_R||^2  =>  -e_R^T J^-1 M - slack <= -k ||e_R||^2
                
                J_inv = inv(J);
                % e_R^T * J_inv は 1x3 のベクトル
                eR_Jinv = e_R' * J_inv;
                
                % 制約: -eR_Jinv * [Mx; My; Mz] - slack <= -Kp * ||e_R||^2 + Kd * e_R^T * w_curr
                % トルク限界内で戦えるようにゲインを引き上げ(積極的に姿勢を変える)
                Kp_att = 25.0; 
                Kd_att = 5.0; 
                b_val = -Kp_att * norm(e_R)^2 + Kd_att * dot(e_R, w_curr);
                
                % A_ineq の列は [T, Mx, My, Mz, slack]
                A_ineq = [0, -eR_Jinv(1), -eR_Jinv(2), -eR_Jinv(3), -1];
                b_ineq = b_val;
            end
            
            % 【安全装置2】推力の最低保証 [T_min, 20] (空中でのモーター停止による落下を防ぐ)
            T_min = m * g * 0.6; % 自重の60%を最低推力とする
            A_lb = [-1,  0,  0,  0,  0;  % -T <= -T_min
                     1,  0,  0,  0,  0;  %  T <= 20
                     0,  1,  0,  0,  0;  %  Mx <= 1
                     0, -1,  0,  0,  0;  % -Mx <= 1
                     0,  0,  1,  0,  0;  %  My <= 1
                     0,  0, -1,  0,  0;  % -My <= 1
                     0,  0,  0,  1,  0;  %  Mz <= 1
                     0,  0,  0, -1,  0]; % -Mz <= 1
                     
            b_lb = [-T_min; 20; 1; 1; 1; 1; 1; 1];
            
            A_all = [A_ineq; A_lb];
            b_all = [b_ineq; b_lb];
            
            options = optimoptions('quadprog', 'Display', 'off');
            [z_opt, ~, exitflag] = quadprog(H, f_qp, A_all, b_all, [], [], [], [], [u_nom; 0], options);
            
            if exitflag < 0 || isempty(z_opt)
                % 【安全装置3】解なし時やパニック時の安全ホバリング・フォールバック
                % 目標軌道との誤差が大きすぎて u_nom が発狂している可能性があるため、
                % u_nom には戻さず、自重支持＋角速度ダンパ（回転ブレーキ）で姿勢を安定化させる
                u_safe = [m * g; -2.0 * J(1,1) * w_curr(1); -2.0 * J(2,2) * w_curr(2); -0.5 * J(3,3) * w_curr(3)];
                exitflag = -99;
            else
                u_safe = z_opt(1:4);
            end
            
            result = result_nom;
            result.u = u_safe;
            result.input = u_safe;
            result.d_surf = d_surf_min;
            result.exitflag = exitflag;
            
            % --- デバッグ表示 (0.25秒 = 10ステップごとに表示) ---
            persistent dbg_cnt_real;
            if isempty(dbg_cnt_real); dbg_cnt_real = 0; end
            dbg_cnt_real = dbg_cnt_real + 1;
            if mod(dbg_cnt_real, 10) == 0
                fprintf('[REAL Debug] Dist:%5.2f | eR_norm:%5.3f | T_req:%5.1f | QP_Exit:%2d | M_y:%6.3f\n', ...
                    d_surf_min, norm(e_R), T_safe_req, exitflag, u_safe(3));
            end
        end
    end
end
