% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % 手法1: 直接定義手法（上限・下限の場合分けによる解析的QP解）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
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
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
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
%         pL = model.state.pL;  % 荷物の位置 [3 x 1]
%         % または ケーブル長 L_cable と単位ベクトル pT から求める場合：
%         L_cable = P(7); % physicalParam/parameter から取得したケーブル長
%         p_mid = pL - 0.5 * L_cable * pT;
%         % 🌟 【追加】 最短距離比較用の変数を無限大 (Inf) で初期化
%         min_surf_dist = Inf;
% 
%         tic_loop = tic;
%         % 全障害物についてループを回し、数式(min/max)に沿って安全区間を絞り込む
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             % 1. 中心間距離の計算
%             dist_center = norm(p_mid - p_obs);
% 
%             % 2. 表面間距離の計算 (中心間距離 - 障害物半径 - 物理保護球半径 rl)
%             % ※ d_surf > 0 なら離れている、<= 0 なら衝突・境界到達
%             d_surf = dist_center - (ro + rl_val);
% 
%             % 最短距離を更新
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
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
%             [A_qp_single, b_qp_single] = CBF_Constraints_zlink(obj, x, xd, vf, obs_params, gamma_params, rl_val, P);
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
%         % 🌟 LOGGER に記録されるよう controller.result に格納
%         obj.result.min_clearance = min_surf_dist;
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

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % クアッドコプター用階層型線形化を使った入力算出（HOCBF安全フィルター付き）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
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
%         % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
%         % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
%         % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
% 
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
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
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS'); % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
%         % 🌟 エラーの出た length(obj.result...) を廃止し、単純に「フリーズ検出」として表示
%         % 50ms（0.05秒）以上かかったステップをすべてコマンドウィンドウに強制出力します
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
%         tmp = [uf(1); us]; % 実入力へ変換
%         obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
%         %% =========================================================================
%         %% 【追加】高次制御バリア関数 (HOCBF) による安全フィルター (QP)
%         %% =========================================================================
%         % 1. 障害物定義関数から環境情報を動的に取得
%         tic_cbf_setup = tic;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         % 2. 荷物の物理半径 rl
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
% 
%         pL = model.state.pL;  % 荷物の位置 [3 x 1]
%         % または ケーブル長 L_cable と単位ベクトル pT から求める場合：
%         L_cable = P(7); % physicalParam/parameter から取得したケーブル長
%         p_mid = pL - 0.5 * L_cable * pT;
%         % 🌟 【追加】 最短距離比較用の変数を無限大 (Inf) で初期化
%         min_surf_dist = Inf;
% 
% 
%         % LOGGER保存用セル配列の初期化 (可変個数のためセル配列でストック)
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         % QP用制約の累積用初期化
%         A_qp_total = [];
%         b_qp_total = [];
% 
%         % HOCBFのクラスK関数ゲイン [gamma1; ...; gamma6]
%         gamma_params = [5.0; 5.0; 5.0; 5.0];
%         V4_val = tmp(4); % yawトルク固定値
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
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             % 1. 中心間距離の計算
%             dist_center = norm(p_mid - p_obs);
% 
%             % 2. 表面間距離の計算 (中心間距離 - 障害物半径 - 物理保護球半径 rl)
%             % ※ d_surf > 0 なら離れている、<= 0 なら衝突・境界到達
%             d_surf = dist_center - (ro + rl_val);
% 
%             % 最短距離を更新
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             % 🌟 各障害物の時系列情報をログバッファ用配列に回収
%             log_p_obs{i} = obs_env(i).p_obs;     % 中心座標 [x; y; z]
%             log_r_obs{i} = obs_env(i).r_obs;     % マージン（d_margin）込みの半径
% 
%             % 🌟 構造体が持っている d_margin を使って、シンプルに一発逆算！
%             % 分岐がなくなったので、今後どんな新しい形状が増えてもコードは一切変わりません。
%             if isfield(obs_env(i), 'd_margin')
%                 log_r_minimal{i} = ro - obs_env(i).d_margin;
%             else
%                 log_r_minimal{i} = ro; % フォールバック用
%             end
% 
%             % 単一障害物に対する QP 制約行列 A_qp (1×2), b_qp (1×1) を生成
%             [A_qp_single, b_qp_single] = CBF_Constraints_xylink(obj, x, xd, vf, V4_val, obs_params, gamma_params, rl_val, P);
% 
%             % 行列を縦に結合
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
%         obj.result.t_loop = toc(tic_loop); % ループ全体（制約生成）にかかった時間
% 
%         % 🌟 LOGGER に記録されるよう controller.result に格納
%         obj.result.min_clearance = min_surf_dist;
%         % 🌟 動的に格納したセル配列を丸ごと logger 保存プロパティへセット
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 3. QP (二次計画法) の実行
%         tic_qp = tic;
%         u_nominal = tmp(2:3); % 理想の roll, pitch 入力
% 
%         H_qp = eye(2);
%         f_qp = -u_nominal;
% 
%         lb = [-1; -1];
%         ub = [ 1;  1];
% 
%         options = optimoptions('quadprog', 'Display', 'off');
% 
%         % 拡張された累積制約 A_qp_total, b_qp_total を使って最適化
%         [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
% 
%         if exitflag < 1
%             u_safe = u_nominal; % 可行解がない場合の緊急セーフティ
%         end
% 
%         tmp(2:3) = u_safe;
%         obj.result.t_qp = toc(tic_qp);
%         obj.result.controllertime=toc(tic_start);
%         %% =========================================================================
% 
%         % 安全のため入力値に制限を付ける．
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
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

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF付き）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
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
% 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
% 
%         % 仮想入力のゲイン
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
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
%         tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 z 方向 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         tic_cbf_setup = tic;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT;
%         min_surf_dist = Inf;
% 
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         % z方向 CBF 解析解用の上下限境界
%         u1_lower_bound = 0.0;
%         u1_upper_bound = 20.0;
%         gamma_params_z = [5.0; 5.0]; 
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             % z方向 CBF 制約（A_z * u1 <= b_z）を取得
%             [A_z_single, b_z_single] = CBF_Constraints_zlink(obj, x, xd, vf, obs_params, gamma_params_z, rl_val, P);
% 
%             % A_i * u1 >= B_i に変換 (符号反転)
%             A_i = -A_z_single;
%             B_i = -b_z_single;
% 
%             if A_i > 1e-9
%                 u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%             elseif A_i < -1e-9
%                 u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%             end
%         end
% 
%         % z方向の確定安全推力 u1_safe (解析的クランプ)
%         u1_nominal = tmp(1);
%         if u1_lower_bound <= u1_upper_bound
%             u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         else
%             u1_safe = u1_nominal; % 緊急フォールバック
%         end
% 
%         % 🌟 確定した z 方向安全推力をセット
%         tmp(1) = u1_safe;
% 
%         %% =========================================================================
%         %% 【ステップ 2】 確定した u1_safe を U1_val として流し込む xy 方向 HOCBF (QP)
%         %% =========================================================================
%         A_qp_total = [];
%         b_qp_total = [];
% 
%         gamma_params_xy = [5.0; 5.0; 5.0; 5.0];
%         V4_val = tmp(4); % yawトルク固定値
%         U1_val = u1_safe; % 🌟 外部から持ってきた確定推力 u1_safe
% 
%         tic_loop = tic;
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             dist_center = norm(p_mid - p_obs);
%             d_surf = dist_center - (ro + rl_val);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
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
%             % 🌟 U1_val (u1_safe) を直接渡して xy 方向の A_qp (1x2), b_qp (1x1) を計算
%             [A_qp_single, b_qp_single] = CBF_Constraints_xyotamesi(obj, x, xd, U1_val, V4_val, obs_params, gamma_params_xy, rl_val, P);
% 
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
%         obj.result.t_loop = toc(tic_loop); 
% 
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 3. QP (二次計画法) による [u2; u3] (roll, pitch) の最適化
%         tic_qp = tic;
%         u_nominal = tmp(2:3); 
% 
%         H_qp = eye(2);
%         f_qp = -u_nominal;
% 
%         lb = [-1; -1];
%         ub = [ 1;  1];
% 
%         options = optimoptions('quadprog', 'Display', 'off');
% 
%         [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
% 
%         if exitflag < 1
%             u_safe = u_nominal; % 可行解がない場合のセーフティ
%         end
% 
%         tmp(2:3) = u_safe;
%         obj.result.t_qp = toc(tic_qp);
%         obj.result.controllertime = toc(tic_start);
% 
%         %% =========================================================================
%         % 最終的な安全入力の出力
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
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

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF付き）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
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
% 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         tic_start = tic;
% 
%         % 仮想入力のゲイン
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
%         time_log = cell(1, 6);
% 
%         tic;
%         time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
% 
%         time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
% 
%         time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
%         us = beta2 \ vs_alpha2; 
% 
%         total_time = toc;
% 
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
%         tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
%         obj.result.tmp = tmp; 
% 
%         %% =========================================================================
%         %% 【ステップ 1】 z 方向 HOCBF による解析的安全推力 u1_safe の算出
%         %% =========================================================================
%         tic_cbf_setup = tic;
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT;
%         min_surf_dist = Inf;
% 
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         % z方向 CBF 解析解用の上下限境界
%         u1_lower_bound = 0.0;
%         u1_upper_bound = 20.0;
%         gamma_params_z = [5.0; 5.0]; 
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             % z方向 CBF 制約（A_z * u1 <= b_z）を取得
%             [A_z_single, b_z_single] = CBF_Constraints_zlink(obj, x, xd, vf, obs_params, gamma_params_z, rl_val, P);
% 
%             % A_i * u1 >= B_i に変換 (符号反転)
%             A_i = -A_z_single;
%             B_i = -b_z_single;
% 
%             if A_i > 1e-9
%                 u1_lower_bound = max(u1_lower_bound, B_i / A_i);
%             elseif A_i < -1e-9
%                 u1_upper_bound = min(u1_upper_bound, B_i / A_i);
%             end
%         end
% 
%         % z方向の確定安全推力 u1_safe (解析的クランプ)
%         u1_nominal = tmp(1);
%         if u1_lower_bound <= u1_upper_bound
%             u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
%         else
%             u1_safe = u1_nominal; % 緊急フォールバック
%         end
% 
%         % 🌟 確定した z 方向安全推力をセット
%         tmp(1) = u1_safe;
% 
%         %% =========================================================================
%         %% 【ステップ 2】 確定した u1_safe を U1_val として流し込む xy 方向 HOCBF (QP)
%         %% =========================================================================
%         A_qp_total = [];
%         b_qp_total = [];
% 
%         gamma_params_xy = [5.0; 5.0; 5.0; 5.0];
%         gamma_params_cable = [10.0; 10.0]; % ケーブル角度用ゲイン
%         V4_val = tmp(4); 
%         U1_val = u1_safe; 
% 
%         % -------------------------------------------------------------------------
%         % A. 障害物回避制約の累積
%         % -------------------------------------------------------------------------
%         tic_loop = tic;
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             p_obs = [xo; yo; zo];
%             obs_params = [xo; yo; zo; ro];
% 
%             dist_center = norm(p_mid - p_obs);
%             d_surf = dist_center - (ro + rl_val);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
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
%             [A_qp_single, b_qp_single] = CBF_Constraints_xyotamesi(obj, x, xd, U1_val, V4_val, obs_params, gamma_params_xy, rl_val, P);
% 
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
% 
%         % -------------------------------------------------------------------------
%         % B. 🌟 【追加】 ケーブル傾き角 (<= 10 deg) 制約の統合
%         % -------------------------------------------------------------------------
%         [A_cable, b_cable] = CBF_Constraints_cable_angle(obj, x, xd, U1_val, V4_val, gamma_params_cable, P);
% 
%         % QP 制約行列に縦結合 (障害物回避 ＋ ケーブル角度制限)
%         A_qp_total = [A_qp_total; A_cable];
%         b_qp_total = [b_qp_total; b_cable];
% 
%         obj.result.t_loop = toc(tic_loop); 
% 
%         % -------------------------------------------------------------------------
%         % C. QP (二次計画法) の実行
%         % -------------------------------------------------------------------------
%         tic_qp = tic;
%         u_nominal = tmp(2:3); 
% 
%         H_qp = 0.5*eye(2);
%         f_qp = -u_nominal;
% 
%         lb = [-1; -1];
%         ub = [ 1;  1];
% 
%         options = optimoptions('quadprog', 'Display', 'off');
% 
%         % 障害物回避とケーブル傾き制限の両方を満たす [u2; u3] を算出
%         [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
% 
%         if exitflag < 1
%             u_safe = u_nominal; % 解がない場合のバックアップ
%         end
% 
%         tmp(2:3) = u_safe;
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.t_qp = toc(tic_qp);
%         obj.result.controllertime = toc(tic_start);
% 
%         %% =========================================================================
%         % 最終的な安全入力の出力
%         obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
%                             max(-1, min(1, tmp(2))); ...  % u2 (安全化されたroll)
%                             max(-1, min(1, tmp(3))); ...  % u3 (安全化されたpitch)
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

classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
properties
    self
    result
    param
end
methods
    function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
        obj.self = self;
        obj.param = param;
        % 🌟 初期化時のログ参照エラー防止（安全ガード）
        obj.result.min_clearance = Inf;
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
        tic_start = tic;
        
        % 仮想入力のゲイン
        F1 = Param.F1; 
        F2 = Param.F2; 
        F3 = Param.F3; 
        F4 = Param.F4; 
        time_log = cell(1, 6);
        
        tic;
        time_log{1} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
        
        time_log{2} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
        
        time_log{3} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
        
        time_log{4} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
        
        time_log{5} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
        
        time_log{6} = datetime('now', 'Format', 'HH:mm:ss.SSSSSS');
        us = beta2 \ vs_alpha2; 
        
        total_time = toc;
        
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
        
        tmp = [uf(1); us]; % ノミナル入力 [u1_nom; u2_nom; u3_nom; u4_nom]
        obj.result.tmp = tmp; 
        
        %% =========================================================================
        %% 【ステップ 1】 z 方向 HOCBF による解析的安全推力 u1_safe の算出
        %% =========================================================================
        tic_cbf_setup = tic;
        
        % 🌟 ループ前デフォルト値の初期化
        min_surf_dist = Inf;
        obj.result.min_clearance = Inf; 
        
        obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
        num_obs = length(obs_env);
        
        rl_val = 0.5; 
        obj.result.rl = rl_val;
        pL = model.state.pL;  
        L_cable = P(7); 
        p_mid = pL - 0.5 * L_cable * pT;
        
        log_p_obs = cell(1, max(1, num_obs));
        log_r_obs = cell(1, max(1, num_obs));
        log_r_minimal = cell(1, max(1, num_obs));
        
        % z方向 CBF 解析解用の上下限境界
        u1_lower_bound = 0.0;
        u1_upper_bound = 20.0;
        gamma_params_z = [1.0; 3.0]; 
        
        for i = 1:num_obs
            xo = obs_env(i).p_obs(1);
            yo = obs_env(i).p_obs(2);
            zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs;
            obs_params = [xo; yo; zo; ro];
            
            % z方向 CBF 制約（A_z * u1 <= b_z）を取得
            [A_z_single, b_z_single] = CBF_Constraints_zlink(obj, x, xd, vf, obs_params, gamma_params_z, rl_val, P);
            
            % A_i * u1 >= B_i に変換 (符号反転)
            A_i = -A_z_single;
            B_i = -b_z_single;
            if A_i > 1e-9
                u1_lower_bound = max(u1_lower_bound, B_i / A_i);
            elseif A_i < -1e-9
                u1_upper_bound = min(u1_upper_bound, B_i / A_i);
            end
        end
        
        % z方向の確定安全推力 u1_safe (解析的クランプ)
        u1_nominal = tmp(1);
        if u1_lower_bound <= u1_upper_bound
            u1_safe = max(u1_lower_bound, min(u1_upper_bound, u1_nominal));
        else
            u1_safe = u1_nominal; % 緊急フォールバック
        end
        
        % 🌟 確定した z 方向安全推力をセット
        tmp(1) = u1_safe;
        
        %% =========================================================================
        %% 【ステップ 2】 確定した u1_safe を U1_val として流し込む xy 方向 HOCBF (QP)
        %% =========================================================================
        A_qp_total = [];
        b_qp_total = [];
        
        gamma_params_xy = [1.0; 2.0; 5.0; 5.0];
        gamma_params_cable = [10.0; 10.0]; % ケーブル角度用ゲイン
        V4_val = tmp(4); 
        U1_val = u1_safe; 
        
        % -------------------------------------------------------------------------
        % A. 障害物回避制約の累積
        % -------------------------------------------------------------------------
        tic_loop = tic;
        for i = 1:num_obs
            xo = obs_env(i).p_obs(1);
            yo = obs_env(i).p_obs(2);
            zo = obs_env(i).p_obs(3);
            ro = obs_env(i).r_obs;
            p_obs = [xo; yo; zo];
            obs_params = [xo; yo; zo; ro];
            
            dist_center = norm(p_mid - p_obs);
            d_surf = dist_center - (ro + rl_val);
            if d_surf < min_surf_dist
                min_surf_dist = d_surf;
            end
            
            log_p_obs{i} = obs_env(i).p_obs;     
            log_r_obs{i} = obs_env(i).r_obs;     
            
            if isfield(obs_env(i), 'd_margin')
                log_r_minimal{i} = ro - obs_env(i).d_margin;
            else
                log_r_minimal{i} = ro; 
            end
            
            [A_qp_single, b_qp_single] = CBF_Constraints_xyotamesi(obj, x, xd, U1_val, V4_val, obs_params, gamma_params_xy, rl_val, P);
            
            A_qp_total = [A_qp_total; A_qp_single];
            b_qp_total = [b_qp_total; b_qp_single];
        end
        
        % -------------------------------------------------------------------------
        % B. ケーブル傾き角 (<= 10 deg) 制約の統合
        % -------------------------------------------------------------------------
        if exist('CBF_Constraints_cable_angle', 'file') == 2
            [A_cable, b_cable] = CBF_Constraints_cable_angle(obj, x, xd, U1_val, V4_val, gamma_params_cable, P);
            A_qp_total = [A_qp_total; A_cable];
            b_qp_total = [b_qp_total; b_cable];
        end
        
        obj.result.t_loop = toc(tic_loop); 
        
        % -------------------------------------------------------------------------
        % C. QP (二次計画法) の実行
        % -------------------------------------------------------------------------
        tic_qp = tic;
        u_nominal = tmp(2:3); 
        
        H_qp = eye(2); % min 0.5 * ||u - u_nom||^2
        f_qp = -u_nominal;
        
        lb = [-1; -1];
        ub = [ 1;  1];
        
        options = optimoptions('quadprog', 'Display', 'off');
        
        % 障害物回避とケーブル傾き制限の両方を満たす [u2; u3] を算出
        [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
        
        % 🌟 【解なし（Infeasible）時のフォールバック処理】
        if exitflag < 1
            % 1. トルク（roll, pitch）を 0 にして機体を水平に戻す（その場にとどまる姿勢）
            u_safe = [0.0; 0.0]; 
            
            % 2. 推力 u1 も重力釣り合い（ホバリング推力）に強制的に設定
            m_total = P(1) + P(6); % mass (m_Q) + loadmass (m_L)
            g_acc   = P(5);        % gravity (g)
            tmp(1)  = m_total * g_acc; 
        end
        
        tmp(2:3) = u_safe;
        
        % 🌟 結果を確実に格納
        obj.result.min_clearance = min_surf_dist;
        obj.result.p_obs     = log_p_obs;
        obj.result.r_obs     = log_r_obs;
        obj.result.r_minimal = log_r_minimal;
        obj.result.exitflag  = exitflag;
        obj.result.t_qp      = toc(tic_qp);
        obj.result.controllertime = toc(tic_start);
        
        %% =========================================================================
        % 最終的な安全入力の出力
        obj.result.input = [max(0, min(20, tmp(1))); ... % u1 (解析的CBFで確定された推力)
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