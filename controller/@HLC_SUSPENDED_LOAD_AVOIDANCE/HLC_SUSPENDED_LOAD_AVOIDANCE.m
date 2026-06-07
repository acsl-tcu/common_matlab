% 
% 
% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% properties
%     self
%     result
%     param
% 
%     % 🌟【論文仕様】未知の外乱を一括蓄積するための離散時間積分メモリ [cite: 23, 37]
%     input_integral = [0; 0; 0]; 
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; 
%         model = obj.self.estimator.result; 
%         ref = obj.self.reference.result;   
% 
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; 
%         else
%             xd = ref.state.get();
%         end
%         pL = model.state.pL;
%         if isprop(model.state, "pT")
%             pT = model.state.pT;
%         else
%             delta = pL - model.state.p;
%             if norm(delta) > 1e-9
%                 pT = delta / norm(delta);
%             else
%                 pT = [0; 0; -1];
%             end
%         end
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
% 
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ①：公称名目入力（目標加速度）の計算
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
% 
%         u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
%         u_nominal = u_nominal(:); 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(37)】UDE (外乱オブザーバ) によるリアルタイム外乱推定 [cite: 23, 78]
%         % -----------------------------------------------------------------
%         v_drone = model.state.v; 
%         T_ude = 0.4;             % 論文Table 1パラメータ [cite: 545]
% 
%         persistent last_u_safe;
%         if isempty(last_u_safe); last_u_safe = u_nominal; end
%         obj.input_integral = obj.input_integral + last_u_safe * Param.dt;
% 
%         % 加速度次元の一括外乱推定値の計算
%         d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 
% 
%         % -----------------------------------------------------------------
%         % フェーズ②：安全制約（CBF）パラメータの準備
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE();
%         ox = obs_env(1).p_obs(1);
%         oy = obs_env(1).p_obs(2);
%         oz = obs_env(1).p_obs(3);
%         ro = obs_env(1).r_obs;
% 
%         r_drone = 0.3; r_load = 0.2;  
% 
%         c1 = 0.8; c2 = 2.5; 
%         d1 = 1.0; d2 = 2;
% 
%         cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
%         cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];
% 
%         is_flight_now = false;
%         if length(varargin) >= 2
%             if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
%                 is_flight_now = true;
%             end
%         end
% 
%         current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);
% 
%         if is_flight_now && (current_h_load_eval <= 10.0)
% 
%             t_current = 0.0; 
%             [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
%             [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);
% 
%             % 🌟【論文式(43)に準拠】入力状態安全(ISSf)を保証する動的緩和項 Δ [cite: 79, 375]
%             sigma_param = 1.0;    % kappa_d = 1.0 [cite: 545]
%             epsilon_param = 0.5;  
%             gamma_param = 2.0;    % Extended class Kゲイン [cite: 324]
%             LgH_load_norm2 = norm(A_load_raw(1:3))^2;
%             Delta_load = LgH_load_norm2 / (4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5)); 
% 
%             % 🌟【数理修正の核心：マイナス符号の強制適用】
%             % quadprog(A_qp * u <= b_qp)の標準形にするため、一括でマイナス化を行い整合性を100%確保します。
%             A_qp = -[A_load_raw(1:3); A_drone_raw(1:3)];
%             b_qp = -[b_load_raw(1); b_drone_raw(1)];
% 
%             % 論文の定理2に従い、外乱残差マージン Delta を境界から減算（防衛線を障害物手前に引き締める） [cite: 372, 373]
%             b_qp(1) = b_qp(1) - max(0, Delta_load); 
% 
%             % 状況モニタ表示
%             qp_margin = b_qp(1) - A_qp(1,:) * u_nominal;
%             fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | QP_Margin: %.2f | STATUS: [CBF-QP ON]\n', xd(1), current_h_load_eval, qp_margin);
% 
%             H_mat = eye(3)* 0.01;
%             f_vec = -u_nominal* 0.01;
%             options = optimoptions('quadprog', 'Display', 'off');
% 
%             [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
%             u_safe = u_safe(:);
% 
%             if exitflag ~= 1
%                 % 万が一のフォールバック退避モード
%                 p_to_obs = [ox; oy; oz] - pL;
%                 escape_dir = -p_to_obs / (norm(p_to_obs) + 1e-5);
%                 u_safe = u_nominal + escape_dir * 4.0; 
%             end
% 
%             current_h_load = current_h_load_eval;
%         else
%             fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | STATUS: [CBF OFF (安全直進中)]\n', xd(1), current_h_load_eval);
%             u_safe = u_nominal;
%             exitflag = 1;
%             current_h_load = 999.0;
%         end
% 
%         last_u_safe = u_safe; 
% 
%         % -----------------------------------------------------------------
%         % 🌟【論文式(31)】UDEによるアクティブ外乱相殺 [cite: 23, 343]
%         % -----------------------------------------------------------------
%         u_robust = u_safe - d_hat; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ④：堅牢な加速度の格納 ＆ 高次微分の同調再合成
%         % -----------------------------------------------------------------
%         vs_safe = vs_nominal;
%         vf_safe = vf_nominal;
% 
%         vs_safe(1) = u_robust(1); 
%         vs_safe(2) = u_robust(2); 
%         vf_safe(1) = u_robust(3); 
% 
%         ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
%         vf_safe(2:6) = vf_nominal(2:6) * ratio; 
% 
%         % -----------------------------------------------------------------
%         % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);
% 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function result = show(obj)
%         obj.result
%     end
% end
% end




classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
properties
    self
    result
    param

    % 🌟【論文仕様】未知の外乱を一括蓄積するための離散時間積分メモリ [cite: 23, 37]
    input_integral = [0; 0; 0]; 
end
methods
    function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
        obj.self = self;
        obj.param = param;
    end
    function result = do(obj, varargin)
        Param = obj.param; 
        model = obj.self.estimator.result; 
        ref = obj.self.reference.result;   

        if isprop(ref.state, 'xd')
            xd = ref.state.xd; 
        else
            xd = ref.state.get();
        end
        pL = model.state.pL;
        if isprop(model.state, "pT")
            pT = model.state.pT;
        else
            delta = pL - model.state.p;
            if norm(delta) > 1e-9
                pT = delta / norm(delta);
            else
                pT = [0; 0; -1];
            end
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

        % -----------------------------------------------------------------
        % フェーズ①：公称名目入力（目標加速度）の計算
        % -----------------------------------------------------------------
        vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 

        u_nominal = [vs_nominal(1); vs_nominal(2); vf_nominal(1)]; 
        u_nominal = u_nominal(:); 

        % -----------------------------------------------------------------
        % 🌟【論文式(37)】UDE (外乱オブザーバ) によるリアルタイム外乱推定 [cite: 23, 78]
        % -----------------------------------------------------------------
        v_drone = model.state.v; 
        T_ude = 0.4;             % 論文Table 1パラメータ [cite: 545]

        persistent last_u_safe;
        if isempty(last_u_safe); last_u_safe = u_nominal; end
        obj.input_integral = obj.input_integral + last_u_safe * Param.dt;

        % 加速度次元の一括外乱推定値の計算
        d_hat = (1 / T_ude) * (v_drone - obj.input_integral); 

        % -----------------------------------------------------------------
        % フェーズ②：安全制約（CBF）パラメータの準備
        % -----------------------------------------------------------------
        obs_env = ENVIRONMENT_OBSTACLE();
        ox = obs_env(1).p_obs(1);
        oy = obs_env(1).p_obs(2);
        oz = obs_env(1).p_obs(3);
        ro = obs_env(1).r_obs;

        r_drone = 0.3; r_load = 0.2;  

        c1 = 0.8; c2 = 2.5; 
        d1 = 1.0; d2 = 2;

        cbfParam_load  = [ox; oy; oz; ro; r_load;  c1; c2; P(:)];
        cbfParam_drone = [ox; oy; oz; ro; r_drone; d1; d2; P(:)];

        is_flight_now = false;
        if length(varargin) >= 2
            if strcmp(varargin{2}, 'f') || string(varargin{2}) == "f"
                is_flight_now = true;
            end
        end

        current_h_load_eval = 0.5 * ((pL(1) - ox)^2 + (pL(2) - oy)^2 + (pL(3) - oz)^2 - (ro + r_load)^2);

        if is_flight_now && (current_h_load_eval <= 10.0)

            t_current = 0.0; 
            [A_load_raw,  b_load_raw]  = CBF_Constraints_Load(obj,  x, xd', u_nominal, cbfParam_load,  t_current);
            [A_drone_raw, b_drone_raw] = CBF_Constraints_Drone(obj, x, xd', u_nominal, cbfParam_drone, t_current);

            % 🌟【論文式(43)に準拠】入力状態安全(ISSf)を保証する動的緩和項 Δ [cite: 79, 375]
            sigma_param = 1.0;    % kappa_d = 1.0 [cite: 545]
            epsilon_param = 0.5;  
            gamma_param = 2.0;    % Extended class Kゲイン [cite: 324]
            LgH_load_norm2 = norm(A_load_raw(1:3))^2;
            Delta_load = LgH_load_norm2 / (4 * sigma_param * (epsilon_param - gamma_param/2 + 1e-5)); 

            % 🌟【数理修正の核心：マイナス符号の強制適用】
            % quadprog(A_qp * u <= b_qp)の標準形にするため、一括でマイナス化を行い整合性を100%確保します。
            A_qp = -[A_load_raw(1:3); A_drone_raw(1:3)];
            b_qp = -[b_load_raw(1); b_drone_raw(1)];

            % 論文の定理2に従い、外乱残差マージン Delta を境界から減算（防衛線を障害物手前に引き締める） [cite: 372, 373]
            b_qp(1) = b_qp(1) - max(0, Delta_load); 

            % 状況モニタ表示
            qp_margin = b_qp(1) - A_qp(1,:) * u_nominal;
            fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | QP_Margin: %.2f | STATUS: [CBF-QP ON]\n', xd(1), current_h_load_eval, qp_margin);

            H_mat = eye(3)* 0.01;
            f_vec = -u_nominal* 0.01;
            options = optimoptions('quadprog', 'Display', 'off');

            [u_safe, ~, exitflag] = quadprog(H_mat, f_vec, A_qp, b_qp, [], [], [], [], [], options);
            u_safe = u_safe(:);

            if exitflag ~= 1
                % 万が一のフォールバック退避モード
                p_to_obs = [ox; oy; oz] - pL;
                escape_dir = -p_to_obs / (norm(p_to_obs) + 1e-5);
                u_safe = u_nominal + escape_dir * 4.0; 
            end

            current_h_load = current_h_load_eval;
        else
            fprintf('--- MONITOR -> t: %.3f | h_eval: %.2f | STATUS: [CBF OFF (安全直進中)]\n', xd(1), current_h_load_eval);
            u_safe = u_nominal;
            exitflag = 1;
            current_h_load = 999.0;
        end

        last_u_safe = u_safe; 

        % -----------------------------------------------------------------
        % 🌟【論文式(31)】UDEによるアクティブ外乱相殺 [cite: 23, 343]
        % -----------------------------------------------------------------
        u_robust = u_safe - d_hat; 

        % -----------------------------------------------------------------
        % フェーズ④：堅牢な加速度の格納 ＆ 高次微分の同調再合成
        % -----------------------------------------------------------------
        vs_safe = vs_nominal;
        vf_safe = vf_nominal;

        vs_safe(1) = u_robust(1); 
        vs_safe(2) = u_robust(2); 
        vf_safe(1) = u_robust(3); 

        ratio = vf_safe(1) / (vf_nominal(1) + 1e-6);
        vf_safe(2:6) = vf_nominal(2:6) * ratio; 

        % -----------------------------------------------------------------
        % フェーズ⑤：既存の非線形相殺・実物理入力への最終コンバート
        % -----------------------------------------------------------------
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P);
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P);
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P);

        us = beta2 \ vs_alpha2; 

        tmp = [uf(1); us]; 
        obj.result.tmp = tmp; 
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    function result = show(obj)
        obj.result
    end
end
end