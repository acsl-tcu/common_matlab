classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% クアッドコプター用階層型線形化を使った入力算出（HOCBF安全フィルター付き）
properties
    self
    result
    param
end
methods
    function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
        obj.self = self;
        obj.param = param;
    end
    function result = do(obj, varargin)
        Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
        model = obj.self.estimator.result; % 推定した状態
        ref = obj.self.reference.result; % 目標値
        % 目標値を取得
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; % 20次元の目標値に対応する用
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
        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
        
        % yaw角の定義域の問題を回避
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
        
        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];
        
        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
        
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
        us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換（理想値）
        
        tmp = [uf(1); us]; % 理想の実入力 [u1; u2; u3; u4]
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

        %% =========================================================================
        %% 【追加】高次制御バリア関数 (HOCBF) による安全フィルター (QP)
        %% =========================================================================
        % 1. 障害物定義関数から環境情報を動的に取得
        obs_env = ENVIRONMENT_OBSTACLE(); 
        
        % 今回は最初のアクティブな障害物 (obs(1): 円柱を包囲した真球) をターゲットにします
        % ※ 複数障害物がある場合は、ここでループを回すか、最も近いものを選択します
        target_obs = obs_env(1); 
        
        % 最小包囲球の中心 [xo; yo; zo] と 半径 ro を抽出
        xo = target_obs.p_obs(1);
        yo = target_obs.p_obs(2);
        zo = target_obs.p_obs(3);
        ro = target_obs.r_obs;
        obs_params = [xo; yo; zo; ro];
        
        % 2. 荷物の物理半径 rl
        % ※ 必要に応じて parameter から取得するか、固定値を設定してください
        rl_val = 0.5; 
        obj.result.rl = rl_val;
        obj.result.p_obs = obs_params(1:3); % 障害物の中心座標 [xo; yo; zo]
        obj.result.r_obs = obs_params(4);    % 幾何バッファ込みの真球半径 ro
        
        % 3. HOCBFのクラスK関数ゲイン [gamma1; ...; gamma6]
        % ※ 挙動が不安定な場合は数値を小さく(例: 0.5)、効きが甘い場合は大きく(例: 5.0)調整してください
        % gamma_params = [2.0; 2.0; 2.0; 2.0; 2.0; 2.0];
        gamma_params = [5.0; 5.0; 5.0; 5.0; 5.0; 5.0];
        
        % 4. 2nd layerの理想入力から、固定する u4 (yawトルク) を抽出
        V4_val = tmp(4); 
        
        % 5. オフライン生成した関数から QP用の制約行列 A_qp, b_qp を取得
        % ※ xd, vf の受け渡し形式は既存の Uf 等の命名規則に合わせて cell2sym を模擬した形にしています
        [A_qp, b_qp] = CBF_Constraints_xy(obj, x, xd, vf, V4_val, obs_params, gamma_params, rl_val, P);
        
        % 6. QP (二次計画法) の実行
        % 理想の roll, pitch 入力 [u2_nominal; u3_nominal]
        u_nominal = tmp(2:3); 
        
        % 目的関数: (u - u_nominal)' * H * (u - u_nominal) を最小化するため、Hは単位行列
        H_qp = eye(2);
        f_qp = -u_nominal; % MATLABの quadprog 形式 1/2*u'*H*u + f'*u
        
        % トルクの物理限界制約 (必要に応じて設定、ここでは既存のmin/max制限 [-1, 1] に合わせています)
        lb = [-1; -1];
        ub = [ 1;  1];
        
        % quadprog オプション（出力を非表示にして高速化）
        options = optimoptions('quadprog', 'Display', 'off');
        
        % 最適化問題を解いて安全な roll, pitch 入力を決定
        [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], lb, ub, [], options);
        
        % 万が一 QP が解けなかった（可行解なしなど）場合のセーフティ
        if exitflag < 1
            u_safe = u_nominal; % 最悪の場合は元の入力をそのまま使用
        end
        
        % 7. QPによって修正された安全な入力を再格納
        tmp(2:3) = u_safe;
        %% =========================================================================

        % 安全のため入力値に制限を付ける．
        obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (推力)
                            max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
                            max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
                            max(-1, min(1, tmp(4)))];    % u4 (yaw)
        
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;
    end
    function show(obj)
        obj.result
    end
end
end