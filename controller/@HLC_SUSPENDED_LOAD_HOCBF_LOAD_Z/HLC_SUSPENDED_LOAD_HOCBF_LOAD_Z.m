% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z < handle
% % クアッドコプター用階層型線形化を使った入力算出（HOCBF z方向安全フィルター付き）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; % 20次元の目標値に対応する用
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
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
% 
%         % yaw角の定義域の問題を回避
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
% 
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS'); % 第二層の実入力（roll,pitch,yawのトルク）への変換
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
%         % フリーズ検出
%         if total_time * 1000 > 50 
%             fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
%             fprintf('1. vf 開始時     : %s\n', time_log{1});
%             fprintf('2. vs 開始時     : %s\n', time_log{2});
%             fprintf('3. uf 開始時     : %s\n', time_log{3});
%             fprintf('4. beta 開始時   : %s\n', time_log{4});
%             fprintf('5. alpha 開始時  : %s\n', time_log{5});
%             fprintf('6. 左除算 開始時 : %s\n', time_log{6});
%             fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
%             fprintf('====================================================\n');
%         end
% 
%         tmp = [uf(1); us]; % 実入力へ変換 [u1; u2; u3; u4]
%         obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
%         %% =========================================================================
%         %% 【変更】高次制御バリア関数z方向 (HOCBF: 相対次数2) による安全フィルター (QP)
%         %% =========================================================================
%         % 1. 障害物定義関数から環境情報を動的に取得
%         tic_cbf_setup = tic;
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obs = length(obs_env);
% 
%         % 2. 荷物の物理半径 rl
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
% 
%         % LOGGER保存用セル配列の初期化
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         % QP用制約の累積用初期化
%         A_qp_total = [];
%         b_qp_total = [];
% 
%         % HOCBF z方向のクラスK関数ゲイン [gamma1; gamma2] (相対次数2に対応)
%         gamma_params = [5.0; 5.0]; 
%         obj.result.t_cbf_setup = toc(tic_cbf_setup);
% 
%         tic_loop = tic;
%         % 全障害物についてループを回し、制約条件をすべて縦に積み上げる
%         for i = 1:num_obs
%             % 現在のターゲット障害物のパラメータ抽出
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             % 障害物の情報をログに記録
%             log_p_obs{i} = obs_env(i).p_obs;     
%             log_r_obs{i} = obs_env(i).r_obs;     
% 
%             if isfield(obs_env(i), 'd_margin')
%                 log_r_minimal{i} = ro - obs_env(i).d_margin;
%             else
%                 log_r_minimal{i} = ro; 
%             end
% 
%             % 🌟 z方向CBF関数 (CBF_Constraints_z.m) を呼び出し
%             % 引数構成: (obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params, sys_params, physicalParam)
%             [A_qp_single, b_qp_single] = CBF_Constraints_z(obj, x, xd, vf, obs_params, gamma_params, rl_val, P);
% 
%             % 行列を縦に結合 (u1 に対する A_qp * u1 <= b_qp 制約)
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
%         obj.result.t_loop = toc(tic_loop); 
% 
%         % セル配列をログにセット
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 3. QP (二次計画法) の実行 (u1 の最適補正)
%         tic_qp = tic;
%         u1_nominal = tmp(1); % 理想の推力（第一層入力）
% 
%         H_qp = 1.0;          % スカラー最適化 (u1のみ)
%         f_qp = -u1_nominal;
% 
%         % 推力 u1 に対する上下限値
%         lb = 0.0;
%         ub = 20.0;
% 
%         options = optimoptions('quadprog', 'Display', 'off');
% 
%         % QPによる u1 の安全化計算
%         [u1_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
% 
%         if exitflag < 1
%             u1_safe = u1_nominal; % 可行解がない場合の緊急フォールバック
%         end
% 
%         % 安全化された u1 を入力ベクトルに再代入
%         tmp(1) = u1_safe;
% 
%         obj.result.t_qp = toc(tic_qp);
%         obj.result.controllertime = toc(tic_start);
%         %% =========================================================================
%         % 安全のため入力値に制限を付ける．
%         obj.result.input = [max(0, min(20, tmp(1))); ... % 安全化された u1 (推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (roll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (pitch)
%                             max(-1, min(1, tmp(4)))];    % u4 (yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z < handle
% % 手法1: 直接定義手法（上限・下限の場合分けによる解析的QP解）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, varargin)
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; % 20次元の目標値に対応する用
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
%         x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
% 
%         % yaw角の定義域の問題を回避
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
% 
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS'); % 第二層の実入力（roll,pitch,yawのトルク）への変換
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
%         % フリーズ検出
%         if total_time * 1000 > 50 
%             fprintf('\n🚨======== 制御フリーズ検出 (処理時間: %.2f ms) ========🚨\n', total_time * 1000);
%             fprintf('1. vf 開始時     : %s\n', time_log{1});
%             fprintf('2. vs 開始時     : %s\n', time_log{2});
%             fprintf('3. uf 開始時     : %s\n', time_log{3});
%             fprintf('4. beta 開始時   : %s\n', time_log{4});
%             fprintf('5. alpha 開始時  : %s\n', time_log{5});
%             fprintf('6. 左除算 開始時 : %s\n', time_log{6});
%             fprintf('7. 全体終了時    : %s\n', datetime('now', 'Format', 'HH:mm:ss.SSSSSS'));
%             fprintf('====================================================\n');
%         end
% 
%         tmp = [uf(1); us]; % 実入力へ変換 [u1; u2; u3; u4]
%         obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
%         %% =========================================================================
%         %% 【手法1】高次制御バリア関数z方向 (HOCBF) による解析的安全フィルター
%         %% =========================================================================
%         % 1. 障害物定義関数から環境情報を動的に取得
%         tic_cbf_setup = tic;
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obs = length(obs_env);
% 
%         % 2. 荷物の物理半径 rl
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
% 
%         % LOGGER保存用セル配列の初期化
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         % 🌟 解析的フィルターのための初期境界（物理的な上下限値を設定）
%         lb = 0.0;
%         ub = 20.0;
%         u1_lower_bound = lb;  % L_total の初期値
%         u1_upper_bound = ub;  % U_total の初期値
% 
%         % HOCBF z方向のクラスK関数ゲイン [gamma1; gamma2]
%         gamma_params = [5.0; 5.0]; 
%         obj.result.t_cbf_setup = toc(tic_cbf_setup);
% 
%         tic_loop = tic;
%         % 全障害物についてループを回し、数式(min/max)に沿って安全区間を絞り込む
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             log_p_obs{i} = obs_env(i).p_obs;     
%             log_r_obs{i} = obs_env(i).r_obs;     
% 
%             if isfield(obs_env(i), 'd_margin')
%                 log_r_minimal{i} = ro - obs_env(i).d_margin;
%             else
%                 log_r_minimal{i} = ro; 
%             end
% 
%             % z方向CBF関数から不等式係数 A_qp * u1 <= b_qp を取得
%             [A_qp_single, b_qp_single] = CBF_Constraints_z(obj, x, xd, vf, obs_params, gamma_params, rl_val, P);
% 
%             % 不等式を A_i * u1 >= B_i の形に変換 (符号反転)
%             A_i = -A_qp_single;
%             B_i = -b_qp_single;
% 
%             % 🌟 【数式のプログラム化】係数の符号に応じて上限・下限を更新
%             if A_i > 1e-9
%                 % 下限制約: u1 >= B_i / A_i (L_safe)
%                 u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%             elseif A_i < -1e-9
%                 % 上限制約: u1 <= B_i / A_i (U_safe)
%                 u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%             end
%         end
%         obj.result.t_loop = toc(tic_loop); 
% 
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 3. 最小ノルム解析解に基づくクリッピング処理
%         tic_qp = tic;
%         u1_nominal = tmp(1); % 階層型線形化から得られた理想のノミナル推力
% 
%         % 安全区間の実行可能性（可解性）をチェック
%         if u1_lower_bound <= u1_upper_bound
%             % 🌟 ノミナル入力を安全な許容区間 [下限, 上限] に丸める (解析解の適用)
%             u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         else
%             % 競合により安全区間が消失した場合の緊急フォールバック
%             u1_safe = u1_nominal; 
%         end
% 
%         % 安全化された u1 を実入力ベクトルに格納
%         tmp(1) = u1_safe;
% 
%         obj.result.t_qp = toc(tic_qp); 
%         obj.result.controllertime = toc(tic_start);
%         %% =========================================================================
%         % 安全のため入力値に制限を付ける．
%         obj.result.input = [max(0, min(20, tmp(1))); ... % 安全化された u1 (推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (roll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (pitch)
%                             max(-1, min(1, tmp(4)))];    % u4 (yaw)
% 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function show(obj)
%         obj.result
%     end
% end
% end

classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z < handle
% 手法2: 中村らの研究ベース（補正量 Delta_u1 の直接算出＋物理限界の厳密結合）
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
        
        % yaw角の定義域の問題を回避
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
        
        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];
        tic_start = tic;
        
        % 階層型線形化による入力計算
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
        time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS'); % 第二層の実入力（roll,pitch,yawのトルク）への変換
        us = beta2 \ vs_alpha2; 
        total_time = toc;
        
        % フリーズ検出
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
        
        tmp = [uf(1); us]; % 実入力へ変換 [u1; u2; u3; u4]
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

        %% =========================================================================
        %% 【手法2】高次制御バリア関数z方向 (HOCBF) による解析的安全フィルター（補正量算出）
        %% =========================================================================
        tic_cbf_setup = tic;
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
        num_obs = length(obs_env);
        
        rl_val = 0.5; 
        obj.result.rl = rl_val;
        
        log_p_obs = cell(1, num_obs);
        log_r_obs = cell(1, num_obs);
        log_r_minimal = cell(1, num_obs);
        
        gamma_params = [5.0; 5.0]; 
        obj.result.t_cbf_setup = toc(tic_cbf_setup);
        
        tic_loop = tic;
        u1_nominal = tmp(1); % 階層型線形化から得られた理想のノミナル推力

        % 🌟【数式結合】補正量 Delta_u1 の許容区間を機体の物理限界 (0 <= u1 <= 20) で初期化
        %  -u1_nominal <= Delta_u1 <= 20 - u1_nominal
        delta_u_L = 0.0 - u1_nominal;      % 下限の初期値 (物理下限)
        delta_u_U = 20.0 - u1_nominal;     % 上限の初期値 (物理上限)

        eps_val = 1e-9; % ゼロ割り防止用の微小値

        % 全障害物についてループを回し、下限 delta_u_L と 上限 delta_u_U を絞り込む
        for i = 1:num_obs
            xo = obs_env(i).p_obs(1);
            yo = obs_env(i).p_obs(2);
            zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs;
            obs_params = [xo; yo; zo; ro];

            log_p_obs{i} = obs_env(i).p_obs;     
            log_r_obs{i} = obs_env(i).r_obs;     

            if isfield(obs_env(i), 'd_margin')
                log_r_minimal{i} = ro - obs_env(i).d_margin;
            else
                log_r_minimal{i} = ro; 
            end

            % z方向CBF関数から A_qp * u1 <= b_qp を取得
            [A_qp_single, b_qp_single] = CBF_Constraints_z(obj, x, xd, vf, obs_params, gamma_params, rl_val, P);

            % 不等式を A_i * u1 >= B_i の形に変換 (符号反転)
            A_i = -A_qp_single;
            B_i = -b_qp_single;

            % ノミナル値に基づく I(x) と J(x) の評価
            I_val = A_i * u1_nominal;
            J_val = B_i;

            % 🌟【数式通りの条件分岐】A_i の符号に応じた Delta_u の境界更新
            if A_i > eps_val
                % 1. 下限制約 A_i > 0 (下にある障害物): Delta_u >= (J - I) / A_i
                % 必要な補正量の「最大値 (max)」で下限を押し上げる
                delta_req = (J_val - I_val) / A_i;
                delta_u_L = max(delta_u_L, delta_req);

            elseif A_i < -eps_val
                % 2. 上限制約 A_i < 0 (上にある障害物): Delta_u <= (J - I) / A_i
                % 必要な補正量の「最小値 (min)」で上限を押し下げる
                delta_req = (J_val - I_val) / A_i;
                delta_u_U = min(delta_u_U, delta_req);
            end
        end
        obj.result.t_loop = toc(tic_loop); 

        obj.result.p_obs     = log_p_obs;
        obj.result.r_obs     = log_r_obs;
        obj.result.r_minimal = log_r_minimal;

        % 🌟【数式通りの最適解算出】
        % Delta_u1* = max(Delta_u_L, min(Delta_u_U, 0))
        tic_qp = tic;

        if delta_u_L <= delta_u_U
            % 【正常時】安全な補正領域が存在する場合
            % 0 (補正なし) を基本とし、必要に応じて [delta_u_L, delta_u_U] の区間にクランプ
            delta_u1_star = max(delta_u_L, min(delta_u_U, 0.0));
            obj.result.is_infeasible = false;
        else
            % 【解なし (Infeasible) 時】物理限界または複数障害物の制約衝突が発生した場合
            warning('HOCBF Infeasible: 物理限界または障害物制約が衝突しました');
            % 可能な限り物理限界 [0, 20] の範囲内で安全側（上限寄り）に止める非常用安全措置
            delta_u1_star = max(0.0 - u1_nominal, min(20.0 - u1_nominal, 0.0));
            obj.result.is_infeasible = true;
        end

        % 最終的な安全推力の決定: u1_safe = u1_nominal + Delta_u1*
        u1_safe = u1_nominal + delta_u1_star;
        tmp(1) = u1_safe;

        obj.result.t_qp = toc(tic_qp); 
        obj.result.controllertime = toc(tic_start);

        %% =========================================================================
        % 実入力ベクトルへの格納（ロール・ピッチ・ヨーは[-1, 1]に安全クランプ）
        obj.result.input = [tmp(1); ...                   % 安全化された u1 (推力)
                            max(-1, min(1, tmp(2))); ...  % u2 (roll)
                            max(-1, min(1, tmp(3))); ...  % u3 (pitch)
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