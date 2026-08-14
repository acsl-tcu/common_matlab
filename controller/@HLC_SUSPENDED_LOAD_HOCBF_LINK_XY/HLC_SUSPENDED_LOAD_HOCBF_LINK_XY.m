classdef HLC_SUSPENDED_LOAD_HOCBF_LINK_XY < handle
    % クアッドコプター用階層型線形化（実入力 u1 保持型 HOCBF 単一球体検証版）
    % 条件：障害物＝球体(円)、ロープ中点 p_mid 基準（単一球体）
properties
    self
    result
    param
end

methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LINK_XY(self, param)
        obj.self = self;
        obj.param = param;
        obj.result.min_clearance = Inf;
    end

    function result = do(obj, varargin)
        Param = obj.param; 
        model = obj.self.estimator.result; 
        ref   = obj.self.reference.result; 

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

        yaw      = wrapToPi(model.state.q(3)); 
        yawd     = xd(4); 
        yawUnit  = [cos(yaw); sin(yaw); 0]; 
        yawdUnit = [cos(yawd); sin(yawd); 0]; 
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
        xd(4)    = -deltaYaw(3) + yaw; 
        xd = [xd; zeros(28 - size(xd, 1), 1)];

        tic_start = tic;

        % 仮想・ノミナル入力の算定
        F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4; 
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        us = beta2 \ vs_alpha2; 

        tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
        obj.result.tmp = tmp; 

        %% =========================================================================
        %% 【ステップ 1】 ロープ中点 p_mid (単一球体) クリアランス計算 ＆ フィールド初期化
        %% =========================================================================
        min_surf_dist = Inf;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY(); 
        num_obs = length(obs_env);

        pL = model.state.pL;  
        L_cable = P(7); 
        p_mid = pL - 0.5 * L_cable * pT; % ロープ（リンク）の中点

        % システム側単一球体の半径 (rl)
        rl_sys = 1.2;
        obj.result.rl = rl_sys;

        % アニメーション描画用フィールドのセット
        obj.result.p_mid = p_mid;

        log_p_obs     = cell(1, max(1, num_obs));
        log_r_obs_margin     = cell(1, max(1, num_obs));
        log_r_obs     = cell(1, max(1, num_obs));
        log_d_margin     = cell(1, max(1, num_obs));
        log_r_minimal = cell(1, max(1, num_obs));
        log_r_minimal_no_margin = cell(1, max(1, num_obs));

        for i = 1:num_obs
            xo = obs_env(i).p_obs(1); yo = obs_env(i).p_obs(2); zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs_margin;
            ro_no_margin = obs_env(i).r_obs;
            p_obs = [xo; yo; zo];

            % 単一球体 (p_mid) からマージンを含めた障害物表面までの距離
            d_surf = norm(p_mid - p_obs) - (ro + rl_sys);
            d_surf_no_margin = norm(p_mid - p_obs) - (ro_no_margin + rl_sys);

            if d_surf < min_surf_dist
                min_surf_dist = d_surf;
            end

            log_p_obs{i}     = p_obs;
            log_r_obs_margin{i}     = ro;
            log_r_obs{i}     = obs_env(i).r_obs;
            log_d_margin{i}     = obs_env(i).d_margin;
            log_r_minimal{i} = d_surf;
            log_r_minimal_no_margin{i} = d_surf_no_margin;
        end

        obj.result.min_clearance = min_surf_dist;
        obj.result.p_obs         = log_p_obs;
        obj.result.r_obs         = log_r_obs_margin;
        obj.result.r_minimal     = log_r_minimal;
        obj.result.r_minimal_no_margin     = log_r_minimal_no_margin;

        %% =========================================================================
        %% 【ステップ 2】 実入力 u1 保持型 CBF_Constraints_xyotamesi による xy 方向 QP 安全補正
        %% =========================================================================
        u23_nominal = [tmp(2); tmp(3)]; % [u2_nom; u3_nom]
        u1_val      = tmp(1);           % U1_val (実入力推力 u1 の確定値)
        u4_val      = tmp(4);           % V4 (Yaw軸制御入力 u4)
        
        gamma_params_xy = [2; 8; 16; 32];

        A_xy_qp_list = [];
        b_xy_qp_list = [];
        
        % 🌟 階層ログ配列の事前割り当て（事前初期化）
        log_h1 = zeros(num_obs, 1);
        log_h2 = zeros(num_obs, 1);
        log_h3 = zeros(num_obs, 1);
        log_h4 = zeros(num_obs, 1);

        % 全障害物に対する CBF 制約の積載 (単一球体モデル)
        for i = 1:num_obs
            obs_params = [obs_env(i).p_obs; obs_env(i).r_obs_margin]; % [xo; yo; zo; ro]
            sys_params = rl_sys;                                 % rl
            
            % 🌟 CBF_Constraints_HOCBF_xylink.m から A, b と各階層 h1~h4 を取得
            [A_xy_single, b_xy_single, h1_val, h2_val, h3_val, h4_val] = CBF_Constraints_HOCBF_xylink(...
                obj, x, xd, u1_val, u4_val, obs_params, gamma_params_xy, sys_params, P);
            
            A_xy_qp_list = [A_xy_qp_list; A_xy_single]; %#ok<AGROW>
            b_xy_qp_list = [b_xy_qp_list; b_xy_single]; %#ok<AGROW>
            
            % 各階層の数値を保存
            log_h1(i) = h1_val;
            log_h2(i) = h2_val;
            log_h3(i) = h3_val;
            log_h4(i) = h4_val;
        end

        % 🌟 2次元 QP (quadprog) の実行
        H_qp  = diag([1.0, 1.0]);
        f_qp  = -H_qp * u23_nominal;
        lb_qp = [-1.0; -1.0];
        ub_qp = [ 1.0;  1.0];
        options = optimoptions('quadprog', 'Display', 'off', 'ConstraintTolerance', 1e-4);
        [u23_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_xy_qp_list, b_xy_qp_list, [], [], lb_qp, ub_qp, [], options);

        % 安全ガード付き更新処理
        if exitflag == 1 && ~isempty(u23_safe)
            tmp(2) = u23_safe(1); % u2 (Roll)
            tmp(3) = u23_safe(2); % u3 (Pitch)
        else
            % 解なし時等のフォールバック (ノミナル値をクリッピング)
            tmp(2) = max(-1.0, min(1.0, u23_nominal(1)));
            tmp(3) = max(-1.0, min(1.0, u23_nominal(2)));
        end

        % 🌟 スラック（制約余裕度）の計算
        slack_nom  = A_xy_qp_list * u23_nominal - b_xy_qp_list; % ノミナル入力での違反量 (>0 で違反)
        if exitflag == 1 && ~isempty(u23_safe)
            slack_safe = b_xy_qp_list - A_xy_qp_list * u23_safe; % 安全入力適用後の余裕 (>=0 で安全)
        else
            slack_safe = -slack_nom;
        end

        % 🌟 デバッグ用コンソール出力 (最悪条件の障害物と各階層の数値を表示)
        num_violated = sum(slack_nom > 0);
        if num_violated > 0
            [max_viol, idx] = max(slack_nom);
            fprintf('[HOCBF 違反検知] 違反数:%d/%d | 最悪障害物 Index:%d | 最大違反量:%.3f\n', ...
                num_violated, length(b_xy_qp_list), idx, max_viol);
            fprintf('  ├─ h1 (距離余裕度 h)       : %.4f (正なら安全領域内)\n', log_h1(idx));
            fprintf('  ├─ h2 (速度・1次応答 h2)    : %.4f\n', log_h2(idx));
            fprintf('  ├─ h3 (加速度・2次応答 h3)  : %.4f\n', log_h3(idx));
            fprintf('  └─ h4 (加加速度・3次応答 h4) : %.4f (これが負に近づくとQP介入)\n', log_h4(idx));
            fprintf('  入力補正: u23_nom:[%.2f, %.2f] -> u23_out:[%.2f, %.2f] (exitflag:%d)\n', ...
                u23_nominal(1), u23_nominal(2), tmp(2), tmp(3), exitflag);
        end

        % 🌟 結果の格納（元のフィールド名もすべて維持）
        obj.result.A_xy_qp_list   = A_xy_qp_list;
        obj.result.b_xy_qp_list   = b_xy_qp_list;
        obj.result.slack_check    = slack_nom; % slack_check も維持
        obj.result.num_violated   = num_violated;
        obj.result.tmp_fix        = tmp; 
        obj.result.p_mid          = p_mid;
        obj.result.min_clearance  = min_surf_dist;
        obj.result.p_obs          = log_p_obs;
        obj.result.r_obs_margin          = log_r_obs_margin;
        obj.result.r_obs          = log_r_obs;
        obj.result.d_margin          = log_d_margin;
        obj.result.r_minimal      = log_r_minimal;
        obj.result.r_minimal_no_margin      = log_r_minimal_no_margin;
        obj.result.controllertime = toc(tic_start);

        % 🌟 階層ログデータの保存
        obj.result.log_h1     = log_h1;
        obj.result.log_h2     = log_h2;
        obj.result.log_h3     = log_h3;
        obj.result.log_h4     = log_h4;
        obj.result.slack_nom  = slack_nom;
        obj.result.slack_safe = slack_safe;

        % 最終制御入力 (物理範囲 [-1, 1] 内に収めて出力)
        obj.result.input = [max(0.0, min(20.0, tmp(1))); ... % u1 (推力)
                            max(-1.0, min(1.0,  tmp(2))); ... % u2 (Roll)
                            max(-1.0, min(1.0,  tmp(3))); ... % u3 (Pitch)
                            max(-1.0, min(1.0,  tmp(4)))];   % u4 (Yaw)
        obj.result.xd = xd;
        obj.result.x  = x;
        result        = obj.result;
    end
    function show(obj)
        obj.result
    end
end
end