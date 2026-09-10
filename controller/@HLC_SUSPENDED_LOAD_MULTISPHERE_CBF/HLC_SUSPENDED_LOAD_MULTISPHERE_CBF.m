classdef HLC_SUSPENDED_LOAD_MULTISPHERE_CBF < HLC_SUSPENDED_LOAD
    % HLC_SUSPENDED_LOAD_MULTISPHERE_CBF
    % 動的拡張CBF（4次）+ 複数球体近似による障害物回避コントローラ
    %
    % 設計方針:
    %   - 楕円体や分離超平面は一切使わない
    %   - ドローン本体(c=1.0), 紐中点(c=0.5), 荷物(c=0.0) の3点を球で近似
    %   - 障害物も球体として扱う（ENVIRONMENT_OBSTACLE の r_obs を使用）
    %   - CBF条件: h = ||p_sys - p_obs|| - R_safe >= 0  (距離ベースCBF)
    %   - 表面距離: d_surf = ||p_sys - p_obs|| - R_safe_core (正値 = 安全)
    %   - CBF有効化トリガー: d_surf <= 6.0 m

    properties
        T_val
        dT_val
        initialized
    end

    methods
        function obj = HLC_SUSPENDED_LOAD_MULTISPHERE_CBF(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            if ~isfield(obj.result, 'min_clearance')
                obj.result.min_clearance = Inf;
            end
            obj.T_val       = 0;
            obj.dT_val      = 0;
            obj.initialized = false;
        end

        function result = do(obj, varargin)
            t_start = tic;

            % =====================================================================
            % 1. ベースHLC実行 → Nominal入力 (クリッピング前の生の仮想入力) を取得
            % =====================================================================
            res_nom = do@HLC_SUSPENDED_LOAD(obj, varargin{:});
            u_nom   = res_nom.tmp;   % [T_nom; tau_nom] (クリッピングなし)

            T_nom   = u_nom(1);
            tau_nom = u_nom(2:4);

            % =====================================================================
            % 2. パラメータ・状態取得
            % =====================================================================
            model  = obj.self.estimator.result;
            dt     = obj.param.dt;

            P_vec  = obj.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]);
            m      = P_vec(1);
            jx     = P_vec(2);
            jy     = P_vec(3);
            jz     = P_vec(4);
            g_acc  = P_vec(5);
            mL     = P_vec(6);
            cableL = P_vec(7);

            T_hover = (m + mL) * g_acc;
            J_mat   = diag([jx; jy; jz]);

            % 初期化
            if ~obj.initialized || isnan(obj.T_val)
                obj.T_val       = T_nom;
                obj.dT_val      = 0;
                obj.initialized = true;
            end

            % 状態量
            p_load = model.state.p;
            v_load = model.state.v;
            pT     = model.state.pT;
            wL     = model.state.wL;
            R_curr = model.state.getq("rotmat");
            q_quat = R2q(R_curr);
            w_vec  = model.state.w;

            % CBF用状態ベクトル [pL(3); vL(3); pT(3); wL(3); q(4); w(3)] = 19次元
            z_state = [p_load(:); v_load(:); pT(:); wL(:); q_quat(:); w_vec(:)];
            T_state = [obj.T_val; obj.dT_val];

            % =====================================================================
            % 3. 動的拡張のNominal入力 [ddT_nom; tau_nom]
            % =====================================================================
            Kp_T    = 200.0;
            Kd_T    = 30.0;
            ddT_nom = -Kp_T * (obj.T_val - T_nom) - Kd_T * obj.dT_val;
            mu_nom  = [ddT_nom; tau_nom(:)];

            % =====================================================================
            % 4. 純粋球体間距離によるCBF制約構築
            % =====================================================================
            A_ineq = zeros(0, 5);
            b_ineq = zeros(0, 1);

            % 球体障害物リスト（r_obs フィールドを持つ）
            obs_list = ENVIRONMENT_OBSTACLE();

            tau_circ             = zeros(3, 1);
            min_h_this_step      = Inf;
            min_d_surf_this_step = Inf;

            % CBF発動トリガー: 表面距離がこの値以下になったらCBF有効化
            d_detect = 6.0;  % [m]

            % システム球の半径と評価点 (c比率)
            r_sys_base = 0.4;                    % 各点の球半径 [m]
            c_ratios   = [0.0, 0.5, 1.0];        % 0:荷物, 0.5:紐中点, 1.0:ドローン

            drone_params = [m; mL; cableL; jx; jy; jz; g_acc];
            cbf_gains    = [16.0; 32.0; 24.0; 12.0];

            for i = 1:length(obs_list)
                % --- 障害物パラメータ（球体定義） ---
                p_obs    = obs_list(i).p_obs;
                r_obs    = obs_list(i).r_obs;      % 球体半径
                d_margin = obs_list(i).d_margin;

                % 安全半径
                R_safe_core = r_sys_base + r_obs;           % ハード境界
                R_safe_mrg  = r_sys_base + r_obs + d_margin; % マージン境界

                for c_idx = 1:length(c_ratios)
                    c_val = c_ratios(c_idx);

                    % システム球の中心位置
                    p_sys_eval = p_load + c_val * cableL * pT;

                    % 中心間距離
                    dist_to_obs = norm(p_sys_eval - p_obs);

                    % ★ 表面距離（球体同士の正しい評価）
                    %   dist_surface > 0: 安全余裕あり
                    %   dist_surface <= 0: 球体同士が接触・侵入
                    dist_surface = dist_to_obs - R_safe_core;

                    % 全球・全障害物中の最小表面距離を追跡
                    if dist_surface < min_d_surf_this_step
                        min_d_surf_this_step = dist_surface;
                    end

                    % ★ 表面距離 <= d_detect (6m) のときのみCBF発動
                    if dist_surface <= d_detect

                        % --- 接線サーキュレーション（ドローン本体のみ） ---
                        if c_val == 1.0
                            n_vec = (p_sys_eval - p_obs) / max(1e-6, dist_to_obs);
                            if norm(v_load) > 0.1
                                v_dir = v_load / norm(v_load);
                            else
                                v_dir = [0; 0; 1.0];
                            end
                            if v_dir' * n_vec < 0.1
                                cross_vn = cross(v_dir, n_vec);
                                if norm(cross_vn) > 1e-3
                                    tangent_dir = cross(n_vec, cross_vn) / norm(cross_vn);
                                else
                                    tangent_dir = [-n_vec(2); n_vec(1); 0.5];
                                    tangent_dir = tangent_dir / max(1e-6, norm(tangent_dir));
                                end
                                b_z = R_curr * [0; 0; 1];
                                tau_circ = tau_circ + 0.15 * cross(b_z, tangent_dir);
                            end
                        end

                        % ① コア境界（ハード制約: スラック係数 0）
                        [A_core, b_core, ~] = MultiSphere_SuspendedLoad_CBF( ...
                            z_state, T_state, drone_params, p_obs, R_safe_core, c_val, cbf_gains);
                        if ~any(isnan(A_core(:))) && ~any(isnan(b_core(:)))
                            A_ineq = [A_ineq; [reshape(A_core, 1, 4), 0.0]];
                            b_ineq = [b_ineq; b_core];
                        end

                        % ② マージン境界（ソフト制約: スラック係数 -1）
                        [A_mrg, b_mrg, h_mrg] = MultiSphere_SuspendedLoad_CBF( ...
                            z_state, T_state, drone_params, p_obs, R_safe_mrg, c_val, cbf_gains);
                        if h_mrg < min_h_this_step
                            min_h_this_step = h_mrg;
                        end
                        if ~any(isnan(A_mrg(:))) && ~any(isnan(b_mrg(:)))
                            A_ineq = [A_ineq; [reshape(A_mrg, 1, 4), -1.0]];
                            b_ineq = [b_ineq; b_mrg];
                        end

                    end % if dist_surface <= d_detect
                end % for c_idx
            end % for i (obstacles)

            % =====================================================================
            % 5. 機体姿勢制約（傾き35度以内）
            % =====================================================================
            cos_gamma_max = cos(35 * pi / 180);
            cbf_gains_att = [25; 10];
            [A_att, b_att, ~] = Attitude_Limit_CBF( ...
                [q_quat(:); w_vec(:)], [jx; jy; jz], cos_gamma_max, cbf_gains_att);
            if ~any(isnan(A_att(:))) && ~any(isnan(b_att(:)))
                A_att_row = reshape(A_att, 1, numel(A_att));
                if length(A_att_row) >= 4
                    A_ineq = [A_ineq; [A_att_row(1:4), 0.0]];
                else
                    A_ineq = [A_ineq; [0.0, A_att_row, zeros(1, 3 - length(A_att_row)), 0.0]];
                end
                b_ineq = [b_ineq; b_att(1)];
            end

            % =====================================================================
            % 6. 推力・トルク物理限界（既存EllipsoidCBFと同一設定）
            % =====================================================================
            dt2_half    = 0.5 * (dt^2);
            T_pred_base = obj.T_val + obj.dT_val * dt;

            T_limit_min = 6.0;   % 落下防止下限 [N]
            T_limit_max = 20.0;  % 上限 [N]

            ddT_lb = (T_limit_min - T_pred_base) / dt2_half;
            ddT_ub = (T_limit_max - T_pred_base) / dt2_half;
            ddT_lb = max(-1000.0, min(1000.0, ddT_lb));
            ddT_ub = max(-1000.0, min(1000.0, ddT_ub));
            if ddT_lb > ddT_ub; tmp = ddT_lb; ddT_lb = ddT_ub; ddT_ub = tmp; end

            lb = [ddT_lb; -1.0; -1.0; -1.0; 0.0];
            ub = [ddT_ub;  1.0;  1.0;  1.0; inf];

            % =====================================================================
            % 7. QP重み（既存EllipsoidCBFと同一）
            % =====================================================================
            w_mu    = 5.0;
            w_slack = 1e6;
            w_ddT   = 5.0;
            w_tau   = 5.0;

            W_u  = diag([w_ddT, w_tau, w_tau, w_tau]);
            H_qp = blkdiag(W_u, w_slack);

            mu_target = mu_nom + [0; tau_circ];
            f_qp      = [-w_mu * mu_target; 0];

            % =====================================================================
            % 8. QP求解
            % =====================================================================
            options  = optimoptions('quadprog', 'Display', 'off');
            exitflag = -1;

            if ~isempty(A_ineq)
                [mu_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, [], [], lb, ub, [], options);
            else
                exitflag = 1;
                mu_opt = [min(max(mu_target(1), ddT_lb), ddT_ub); ...
                          min(max(mu_target(2:4), -1.0), 1.0); ...
                          0.0];
            end

            if exitflag == 1
                mu_safe = mu_opt(1:4);
            else
                % フォールバック: ホバリング復帰 + 角速度ダンピング
                ddT_fallback = -100.0 * (obj.T_val - T_hover) - 20.0 * obj.dT_val;
                ddT_fallback = min(max(ddT_fallback, ddT_lb), ddT_ub);
                tau_fallback = -2.5 * J_mat * w_vec;
                tau_fallback = min(max(tau_fallback, -1.0), 1.0);
                mu_safe = [ddT_fallback; tau_fallback];
            end

            % =====================================================================
            % 9. 状態更新（既存EllipsoidCBFと同一の安全クリップ）
            % =====================================================================
            obj.dT_val = obj.dT_val + mu_safe(1) * dt;
            obj.T_val  = obj.T_val  + obj.dT_val * dt;
            obj.T_val  = max(T_limit_min, min(T_limit_max, obj.T_val));  % ハードクリップ

            tau_safe = max(-1.0, min(1.0, mu_safe(2:4)));
            u_final  = [obj.T_val; tau_safe];

            % =====================================================================
            % 10. ログ格納
            % =====================================================================
            obj.result.input          = u_final;
            obj.result.u_nom          = u_nom;
            obj.result.u_diff         = u_final - u_nom;
            obj.result.controllertime = toc(t_start) * 1000.0;
            obj.result.drone_att_deg  = Quat2Eul(q_quat) * (180/pi);

            pT_unit = pT / max(1e-6, norm(pT));
            obj.result.cable_tilt_deg = acos(max(-1.0, min(1.0, abs(pT_unit(3))))) * (180.0 / pi);

            obj.result.cbf_active = double(min_h_this_step < 4.0);
            obj.result.qp_status  = double(exitflag == 1);
            obj.result.h_val      = min_h_this_step;
            obj.result.d_surf     = min_d_surf_this_step;

            result = obj.result;
        end
    end
end
