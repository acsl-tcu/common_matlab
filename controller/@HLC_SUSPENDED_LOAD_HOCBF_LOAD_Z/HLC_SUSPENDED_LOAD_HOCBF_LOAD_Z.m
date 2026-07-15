classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z < handle
% クアッドコプター用階層型線形化を使った入力算出（HOCBF安全フィルター付き）
properties
    self
    result
    param
end
methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z(self, param)
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

        % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
        % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
        % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
        
        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];

        tic_start = tic;
        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン
        time_log = cell(1, 6);
        
        tic;
        time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
        
        time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
        
        time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
        
        time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
        
        time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
        
        time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS'); % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
        us = beta2 \ vs_alpha2; 
        
        total_time = toc;
        
        % 🌟 エラーの出た length(obj.result...) を廃止し、単純に「フリーズ検出」として表示
        % 50ms（0.05秒）以上かかったステップをすべてコマンドウィンドウに強制出力します
        if total_time * 1000 > 50 
            fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
            fprintf('1. vf 開始時     : %s\n', time_log{1});
            fprintf('2. vs 開始時     : %s\n', time_log{2});
            fprintf('3. uf 開始時     : %s\n', time_log{3});
            fprintf('4. beta 開始時   : %s\n', time_log{4});
            fprintf('5. alpha 開始時  : %s\n', time_log{5});
            fprintf('6. 左除算 開始時 : %s\n', time_log{6});
            fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
            fprintf('====================================================\n');
        end
        
        tmp = [uf(1); us]; % 実入力へ変換
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
        %% =========================================================================
        %% 【追加】高次制御バリア関数 (HOCBF) による安全フィルター (QP)
        %% =========================================================================
        % 1. 障害物定義関数から環境情報を動的に取得
        tic_cbf_setup = tic;
        obs_env = ENVIRONMENT_OBSTACLE(); 
        num_obs = length(obs_env);
        
        % 2. 荷物の物理半径 rl
        rl_val = 0.5; 
        obj.result.rl = rl_val;
        
        % LOGGER保存用セル配列の初期化 (可変個数のためセル配列でストック)
        log_p_obs = cell(1, num_obs);
        log_r_obs = cell(1, num_obs);
        log_r_minimal = cell(1, num_obs);
        
        % QP用制約の累積用初期化
        A_qp_total = [];
        b_qp_total = [];
        
        % HOCBFのクラスK関数ゲイン [gamma1; ...; gamma6]
        gamma_params = [5.0; 5.0; 5.0; 5.0; 5.0; 5.0];
        V4_val = tmp(4); % yawトルク固定値
        obj.result.t_cbf_setup = toc(tic_cbf_setup);
        
        tic_loop = tic;
        % 全障害物についてループを回し、制約条件をすべて縦に積み上げる
        for i = 1:num_obs
            % 現在のターゲット障害物のパラメータ抽出
            xo = obs_env(i).p_obs(1);
            yo = obs_env(i).p_obs(2);
            zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs;
            obs_params = [xo; yo; zo; ro];
            
            % 🌟 各障害物の時系列情報をログバッファ用配列に回収
            log_p_obs{i} = obs_env(i).p_obs;     % 中心座標 [x; y; z]
            log_r_obs{i} = obs_env(i).r_obs;     % マージン（d_margin）込みの半径
            
            % 🌟 構造体が持っている d_margin を使って、シンプルに一発逆算！
            % 分岐がなくなったので、今後どんな新しい形状が増えてもコードは一切変わりません。
            if isfield(obs_env(i), 'd_margin')
                log_r_minimal{i} = ro - obs_env(i).d_margin;
            else
                log_r_minimal{i} = ro; % フォールバック用
            end
            
            % 単一障害物に対する QP 制約行列 A_qp (1×2), b_qp (1×1) を生成
            [A_qp_single, b_qp_single] = CBF_Constraints_xy(obj, x, xd, vf, V4_val, obs_params, gamma_params, rl_val, P);
            
            % 行列を縦に結合
            A_qp_total = [A_qp_total; A_qp_single];
            b_qp_total = [b_qp_total; b_qp_single];
        end
        obj.result.t_loop = toc(tic_loop); % ループ全体（制約生成）にかかった時間
        
        % 🌟 動的に格納したセル配列を丸ごと logger 保存プロパティへセット
        obj.result.p_obs     = log_p_obs;
        obj.result.r_obs     = log_r_obs;
        obj.result.r_minimal = log_r_minimal;
        
        % 3. QP (二次計画法) の実行
        tic_qp = tic;
        u_nominal = tmp(2:3); % 理想の roll, pitch 入力
        
        H_qp = eye(2);
        f_qp = -u_nominal;
        
        lb = [-1; -1];
        ub = [ 1;  1];
        
        options = optimoptions('quadprog', 'Display', 'off');
        
        % 拡張された累積制約 A_qp_total, b_qp_total を使って最適化
        [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
        
        if exitflag < 1
            u_safe = u_nominal; % 可行解がない場合の緊急セーフティ
        end
        
        tmp(2:3) = u_safe;
        obj.result.t_qp = toc(tic_qp);
        obj.result.controllertime=toc(tic_start);
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