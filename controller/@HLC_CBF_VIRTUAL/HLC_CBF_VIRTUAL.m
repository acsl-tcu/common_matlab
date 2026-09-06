classdef HLC_CBF_VIRTUAL < HLC_SUSPENDED_LOAD
    % HLC_CBF_VIRTUAL: 動的拡張フィルタと状態制約HOCBFを組み合わせた仮想入力CBF
    % 1. v1(Z軸) の微分問題を解決するため、動的拡張コマンドフィルタを通す。
    % 2. v2, v3 のワインドアップ(6階積分の暴走)を防ぐため、
    %    位置だけでなく現在の速度・加速度を組み込んだ低次元化HOCBFを構成する。
    
    properties
        APPROX_TYPE = 2;
        r_safe = 0.6;
        d_margin_base = 0.5;
        obs_list_cache;
        
        % コマンドフィルタ用の状態 (v1_safe を積分して微分値を得る)
        v1_filt = 0;
        v1_dot_filt = 0;
        v1_ddot_filt = 0;
    end
    
    methods
        function obj = HLC_CBF_VIRTUAL(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
        end
        
        function result = do(obj, varargin)
            Param = obj.param;
            dt = Param.dt;
            model = obj.self.estimator.result;
            ref = obj.self.reference.result;
            
            if isprop(ref.state, 'xd'); xd = ref.state.xd; else; xd = ref.state.get(); end
            
            pL = model.state.pL;
            if isprop(model.state, "pT")
                pT = model.state.pT;
            else
                delta = pL - model.state.p;
                if norm(delta) > 1e-9; pT = delta / norm(delta); else; pT = [0; 0; -1]; end
            end
            P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
            
            x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
            
            yaw = wrapToPi(model.state.q(3));
            yawd = xd(4);
            yawUnit = [cos(yaw); sin(yaw); 0];
            yawdUnit = [cos(yawd); sin(yawd); 0];
            deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit);
            xd(4) = -deltaYaw(3) + yaw;
            xd = [xd; zeros(28 - size(xd, 1), 1)];
            
            F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4;
            % ノミナル仮想入力の計算
            vf_nom = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
            
            % 【安全装置0】自由落下(T=0)による特異点崩壊を防止
            % ドローンは推力が0になると水平・姿勢制御力を完全に失う。
            % ベースコントローラが自由落下(-9.81)を要求しても、最低限の推力を残すためにクランプする
            vf_nom(1) = max(-5.0, vf_nom(1));
            
            vs_nom = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nom, P, F2, F3, F4); 
            % 【安全装置4】水平安定化ループの保護と推力限界
            m_drone = obj.self.parameter.get("mass");
            m_total = m_drone + 0.15;
            g = obj.self.parameter.get("gravity");
            
            % 特異点防止：最低でも0.5G確保
            vf_nom(1) = max(-0.5 * g, vf_nom(1));
            
            % 【最重要】水平加速度(vs)をゼロにすると姿勢安定化ループが切断されて「倒れて」しまう。
            % そのため、vsは「最大15度」で制限しつつ、方向と大きさは絶対に維持する。
            max_vs = g * tan(deg2rad(15.0));
            if norm(vs_nom) > max_vs
                vs_nom = (vs_nom / norm(vs_nom)) * max_vs;
            end
            
            % 残りの推力容量を計算し、Z方向(高度)の要求を物理限界内にカットする
            max_F_z = sqrt(19.5^2 - (m_total * norm(vs_nom))^2);
            F_z_req = m_total * (g + vf_nom(1));
            if F_z_req > max_F_z
                vf_nom(1) = (max_F_z / m_total) - g;
            end
            
            p_curr = model.state.p;
            v_curr = model.state.v;
            pL_curr = model.state.pL;
            vL_curr = model.state.vL;
            
            if isempty(obj.obs_list_cache); obj.obs_list_cache = ENVIRONMENT_OBSTACLE_ELLIPSOID(); end
            
            d_surf_min = inf;
            F_rep = [0; 0; 0];
            
            % --- 障害物からの斥力 (現在の位置と速度から計算) ---
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
                
                if dist_raw < 6.0 && dist_raw > 0.01
                    n_vec = obs.R_obs * (Q_inv_eff^2) * diff_local / norm((Q_inv_eff^2) * diff_local);
                    v_app = dot(v_curr, n_vec); % 接近速度
                    % 位置バリアだけでなく、接近速度のバリア(HOCBF近似)を入れてワインドアップを防ぐ
                    h_dot_penalty = 0;
                    if v_app < 0; h_dot_penalty = 2.0 * abs(v_app); end
                    F_rep = F_rep + (1.5 * (1/dist_raw - 1/6.0) + h_dot_penalty) * (1/dist_raw) * n_vec;
                end
            end
            
            % --- QPによる仮想入力 vs (X,Y) の最適化 (ワインドアップ防止) ---
            % H = diag([1, 1, 10, 1000]);
            H = diag([1, 1, 1, 10000]);
            f_qp = [-vs_nom(1); -vs_nom(2); -vs_nom(3); 0];
            
            A_ineq = []; b_ineq = [];
            if norm(F_rep(1:2)) > 0.1
                % ワインドアップを防ぐため、仮想入力は過剰な値にならず、
                % 現在の速度(v_curr)に応じて適切にブレーキをかけるように制約
                A_ineq = [-F_rep(1), -F_rep(2), 0, -1]; 
                % 逃げ指令値(b_ineq)を大きくしすぎない
                b_ineq = -norm(F_rep(1:2)) * 5.0; 
            end
            
            % --- アフィンマッピングによる実入力限界の仮想制約化 ---
            % us = beta2 \ (vs - alpha2) という階層型線形化の線形関係を逆利用し、
            % トルク限界 [-1, 1] を仮想入力 vs の実行可能多面体(ポリトープ)に変換する。
            beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_nom, P);
            vs_zero = [0; 0; 0];
            alpha2_bias = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_nom, vs_zero, P);
            us_bias = beta2 \ alpha2_bias; % vs=0 の時に必要なノミナルトルク
            inv_beta2 = inv(beta2);
            
            % inv_beta2 * vs <= 1 - us_bias
            % -inv_beta2 * vs <= 1 + us_bias
            % ただし、z = [vs2, vs3, vs4, slack]' なのでゼロ列を追加
            A_torque = [inv_beta2, zeros(3,1); -inv_beta2, zeros(3,1)];
            b_torque = [ones(3,1) - us_bias; ones(3,1) + us_bias];
            
            options = optimoptions('quadprog', 'Display', 'off');
            [z_opt, ~, exitflag] = quadprog(H, f_qp, [A_ineq; A_torque], [b_ineq; b_torque], [], [], [], [], [vs_nom'; 0], options);
            
            if exitflag < 0 || isempty(z_opt)
                vs_safe = [0; 0; 0];
            else
                vs_safe = z_opt(1:3);
            end
            
            % --- Z軸仮想入力 (v1) の CBF ---
            v1_nom = vf_nom(1);
            v1_safe_target = v1_nom;
            if F_rep(3) > 0.5
                % 斥力によってv1_safe_targetを上げる
                v1_safe_target = v1_nom + F_rep(3) * 2.0;
            end
            
            % 特異点防止の再確認
            v1_safe_target = max(-5.0, v1_safe_target);
            
            % 【安全装置5】CBF出力に対する安全な推力カット
            % v1_safe_target の特異点防止（最低0.5G確保）
            v1_safe_target = max(-0.5 * g, v1_safe_target);
            
            % vs_safe の最大15度制限
            if norm(vs_safe) > max_vs
                vs_safe = (vs_safe / norm(vs_safe)) * max_vs;
            end
            
            % Z方向の推力を残り容量にカット
            max_F_z_safe = sqrt(19.5^2 - (m_total * norm(vs_safe))^2);
            F_z_safe = m_total * (g + v1_safe_target);
            if F_z_safe > max_F_z_safe
                v1_safe_target = (max_F_z_safe / m_total) - g;
            end
            
            % コマンドフィルタ(動的拡張)による滑らかな微分値の生成
            % 固有角周波数 wn と減衰比 zeta で 2次遅れ系を構成
            wn = 20.0; zeta = 1.0;
            obj.v1_ddot_filt = -2*zeta*wn*obj.v1_dot_filt - wn^2*(obj.v1_filt - v1_safe_target);
            obj.v1_dot_filt = obj.v1_dot_filt + obj.v1_ddot_filt * dt;
            obj.v1_filt = obj.v1_filt + obj.v1_dot_filt * dt;
            
            % フィルタリングされた v1 とその微分を vf_nom と同じ次元の配列として作成
            vf_safe = zeros(size(vf_nom));
            vf_safe(1) = obj.v1_filt;
            vf_safe(2) = obj.v1_dot_filt;
            vf_safe(3) = obj.v1_ddot_filt;
            
            % --- 実入力への変換 ---
            uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
            beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
            vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe, P);
            us = beta2 \ vs_alpha2;
            
            tmp = [uf(1); us];
            
            % 【安全装置2＆3】推力の最低保証 [T_min, 20] と、QP解なし時のフォールバック
            m = obj.self.parameter.get("mass");
            g = obj.self.parameter.get("gravity");
            T_min = m * g * 0.6;
            
            if exitflag < 0 || exitflag == -99
                % 解なし時は純粋なホバリング（推力=自重、トルク=0）にフォールバック
                obj.result.input = [m * g; 0; 0; 0];
                exitflag = -99;
            else
                % 正常時は推力とトルクを物理限界でハードクランプ
                obj.result.input = [max(T_min, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))];
            end
            obj.result.xd = xd;
            obj.result.x = x;
            obj.result.d_surf = d_surf_min;
            obj.result.exitflag = exitflag;
            obj.result.u = obj.result.input;
            
            result = obj.result;
            
            % --- デバッグ表示 (0.25秒 = 10ステップごとに表示) ---
            persistent dbg_cnt_virt;
            if isempty(dbg_cnt_virt); dbg_cnt_virt = 0; end
            dbg_cnt_virt = dbg_cnt_virt + 1;
            if mod(dbg_cnt_virt, 10) == 0
                fprintf('[VIRT Debug] Dist:%5.2f | v1_tgt:%5.2f | vs_norm:%5.2f | QP_Exit:%2d | T_out:%5.1f\n', ...
                    d_surf_min, v1_safe_target, norm(vs_safe), exitflag, obj.result.u(1));
            end
        end
    end
end
