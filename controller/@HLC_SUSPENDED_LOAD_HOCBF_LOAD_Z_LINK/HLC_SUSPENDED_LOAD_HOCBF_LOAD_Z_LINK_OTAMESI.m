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

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % クアッドコプター用階層型線形化（z解析的CBF ＋ xy実入力結合型HOCBF ＋ ケーブル角度制限付き）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
%         obj.self = self;
%         obj.param = param;
%         % 🌟 初期化時のログ参照エラー防止（安全ガード）
%         obj.result.min_clearance = Inf;
%     end
% 
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
% 
%         % 🌟 ループ前デフォルト値の初期化
%         min_surf_dist = Inf;
%         obj.result.min_clearance = Inf; 
% 
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
%         pL = model.state.pL;  
%         L_cable = P(7); 
%         p_mid = pL - 0.5 * L_cable * pT;
% 
%         log_p_obs = cell(1, max(1, num_obs));
%         log_r_obs = cell(1, max(1, num_obs));
%         log_r_minimal = cell(1, max(1, num_obs));
% 
%         % z方向 CBF 解析解用の上下限境界
%         u1_lower_bound = 0.0;
%         u1_upper_bound = 20.0;
%         gamma_params_z = [1.0; 3.0]; 
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
%         gamma_params_xy = [1.0; 2.0; 5.0; 5.0];
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
%         % B. ケーブル傾き角 (<= 10 deg) 制約の統合
%         % -------------------------------------------------------------------------
%         if exist('CBF_Constraints_cable_angle', 'file') == 2
%             [A_cable, b_cable] = CBF_Constraints_cable_angle(obj, x, xd, U1_val, V4_val, gamma_params_cable, P);
%             A_qp_total = [A_qp_total; A_cable];
%             b_qp_total = [b_qp_total; b_cable];
%         end
% 
%         obj.result.t_loop = toc(tic_loop); 
% 
%         % -------------------------------------------------------------------------
%         % C. QP (二次計画法) の実行
%         % -------------------------------------------------------------------------
%         tic_qp = tic;
%         u_nominal = tmp(2:3); 
% 
%         H_qp = eye(2); % min 0.5 * ||u - u_nom||^2
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
%         % 🌟 【解なし（Infeasible）時のフォールバック処理】
%         if exitflag < 1
%             % 1. トルク（roll, pitch）を 0 にして機体を水平に戻す（その場にとどまる姿勢）
%             u_safe = [0.0; 0.0]; 
% 
%             % 2. 推力 u1 も重力釣り合い（ホバリング推力）に強制的に設定
%             m_total = P(1) + P(6); % mass (m_Q) + loadmass (m_L)
%             g_acc   = P(5);        % gravity (g)
%             tmp(1)  = m_total * g_acc; 
%         end
% 
%         tmp(2:3) = u_safe;
% 
%         % 🌟 結果を確実に格納
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
%         obj.result.exitflag  = exitflag;
%         obj.result.t_qp      = toc(tic_qp);
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
% 
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % クアッドコプター用階層型線形化 ＋ 3軸全推力CBF安全補正（ガウス・ニュートン書き戻し版）
% properties
%     self
%     result
%     param
%     adaptive_gain = [1.0; 1.0; 1.0; 1.0]; % [df; dMx; dMy; dMz] 動的感度ゲイン
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param;                  % ゲイン設定等
%         model = obj.self.estimator.result; % 推定状態
%         ref   = obj.self.reference.result; % 目標軌道
% 
%         % --- 目標値の取得 ---
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd;
%         else
%             xd = ref.state.get();
%         end
% 
%         % --- 紐ベクトル pT の取得/計算 ---
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
% 
%         % --- 物理パラメータの集約 ---
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
% 
%         % 1. 既存制御器用の19次元フル状態ベクトル [q; w; pL; vL; pT; wL]
%         x_full = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
%         % 2. 🌟 CBF専用の12次元状態ベクトル [pL; vL; pT; wL] (姿勢 q, w を分離)
%         x_cbf = [pL; model.state.vL; pT; model.state.wL];
% 
%         % --- Yaw角の連続性補正 ---
%         yaw = wrapToPi(model.state.q(3));
%         yawd = xd(4);
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         tic_start = tic;
% 
%         % =================================================================
%         % 1. 既存の階層型線形化による公称実入力 [f_nom; M_nom] の算出
%         % =================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4;
% 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x_full, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x_full, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x_full, xd', vf, P); 
% 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x_full, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x_full, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; % 公称実入力 [f_nom; M_nom] in R^4
%         obj.result.tmp = tmp; 
%         obj.result.controllertime = toc(tic_start);
% 
%         % =================================================================
%         % 2. Flight フェーズ判定および CBF-QP / 状態依存動的ゲイン書き戻し
%         % =================================================================
%         current_phase = "FLIGHT";
%         if isprop(obj.self, 'phase')
%             current_phase = string(obj.self.phase);
%         end
% 
%         if strcmp(current_phase, "FLIGHT")
%             % --- A. 公称実入力 tmp=[f_nom; M_nom] から予測仮想力 F_nom を計算 ---
%             q = model.state.getq('compact');
%             R_curr = quat2rotm(q'); % 現在の機体回転行列
%             w = model.state.w;       % 現在の機体角速度
%             J_Q = diag(P(2:4));      % 慣性モーメント [jx, jy, jz]
%             dt_pred = Param.dt;      % 予測時間幅
% 
%             % トルク M_nom から Rodrigues 公式による姿勢予測 R_pred
%             Omega_dot_nom = J_Q \ (tmp(2:4) - cross(w, J_Q * w));
%             delta_theta = w * dt_pred + 0.5 * Omega_dot_nom * (dt_pred^2);
%             angle = norm(delta_theta);
%             if angle > 1e-8
%                 axis_vec = delta_theta / angle;
%                 hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%                 R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%             else
%                 R_rel = eye(3);
%             end
%             R_pred_nom = R_curr * R_rel;
% 
%             % 世界座標系での公称仮想全推力 F_nom
%             F_nom = tmp(1) * R_pred_nom * [0; 0; 1];
% 
%             % --- B. 🌟 12次元状態量 x_cbf と障害物環境を用いた CBF-QP (F* の導出) ---
%             F_star = obj.solve_CBF_QP_3D(x_cbf, F_nom, P);
% 
%             % --- C. 安全仮想力 F* から実入力 tmp_star=[f*; M*] への逆変換 ---
%             [tmp_star, opt_err] = obj.reconstruct_real_input(F_star, tmp, R_curr, w, J_Q, dt_pred);
% 
%             % --- D. リアルタイムログ・表示 ---
%             fprintf('[FLIGHT CBF] F_nom: [%.2f, %.2f, %.2f] | F*: [%.2f, %.2f, %.2f] | ResErr: %.3e | DynGains(J): [%.2f, %.2f, %.2f, %.2f]\n', ...
%                 F_nom(1), F_nom(2), F_nom(3), F_star(1), F_star(2), F_star(3), opt_err, ...
%                 obj.adaptive_gain(1), obj.adaptive_gain(2), obj.adaptive_gain(3), obj.adaptive_gain(4));
% 
%             final_tmp = tmp_star;
%         else
%             final_tmp = tmp;
%         end
% 
%         % =================================================================
%         % 3. 最終実入力のガード（入力飽和制限）と格納
%         % =================================================================
%         obj.result.input = [
%             max(0, min(20, final_tmp(1))); % 推力 f
%             max(-1, min(1, final_tmp(2))); % トルク Mx
%             max(-1, min(1, final_tmp(3))); % トルク My
%             max(-1, min(1, final_tmp(4)))  % トルク Mz
%         ];
% 
%         obj.result.xd = xd;
%         obj.result.x = x_full;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% methods (Access = private)
%     % -----------------------------------------------------------------
%     % 🌟 障害物環境情報を動的取得し、全推力 F* を求める CBF-QP ソルバー
%     % -----------------------------------------------------------------
%     function F_star = solve_CBF_QP_3D(obj, x_cbf, F_nom, P)
%         % 1. 障害物定義環境の動的読み込み
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         rl_val = 0.5; % 保護球半径
%         obj.result.rl = rl_val;
% 
%         L_cable = P(7);
%         pL = x_cbf(1:3);
%         pT = x_cbf(7:9);
%         p_mid = pL - 0.5 * L_cable * pT; % ケーブル中心点
% 
%         A_qp_total = [];
%         b_qp_total = [];
%         gamma_params = [5.0; 10.0]; % 相対次数2用のクラスKゲイン
% 
%         min_surf_dist = Inf;
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         % 2. 各障害物に対する CBF 制約行列 (A_qp_i * F <= b_qp_i) の積み上げ
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             % クリアランス・距離計算
%             dist_center = norm(p_mid - [xo; yo; zo]);
%             d_surf = dist_center - (ro + rl_val);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             log_p_obs{i} = obs_env(i).p_obs;
%             log_r_obs{i} = obs_env(i).r_obs;
%             if isfield(obs_env(i), 'd_margin')
%                 log_r_minimal{i} = ro - obs_env(i).d_margin;
%             else
%                 log_r_minimal{i} = ro;
%             end
% 
%             % 🌟 事前に自動導出した CBF_force 関数に 12次元状態量 x_cbf を代入
%             [A_qp_single, b_qp_single] = CBF_forece(obj, x_cbf, obs_params, gamma_params, rl_val, P);
% 
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
% 
%         % ログ情報の保存
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 3. 3自由度全推力 F* に対する QP 最適化 ( min 1/2 || F_star - F_nom ||^2 )
%         H_qp = eye(3);
%         f_qp = -F_nom;
% 
%         options = optimoptions('quadprog', 'Display', 'off');
%         [F_star_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], [], [], [], options);
% 
%         if exitflag >= 1
%             F_star = F_star_opt;
%         else
%             F_star = F_nom; % 可行解なし（Feasible解なし）時のフォールバック
%         end
%     end
% 
%     % -----------------------------------------------------------------
%     % 仮想力 F* に一致する実入力 [f*; M*] への復元（ガウス・ニュートン反復）
%     % -----------------------------------------------------------------
%     function [tmp_star, min_err] = reconstruct_real_input(obj, F_star, tmp_nom, R_curr, w, J_Q, dt_pred)
%         u_curr = tmp_nom; 
%         e_F = [0; 0; 0];
% 
%         hat_e3 = [ 0, -1,  0;
%                    1,  0,  0;
%                    0,  0,  0];
% 
%         % ループ外での初期定義
%         dF_df = R_curr * [0; 0; 1];
%         dF_dM = -0.5 * (dt_pred^2) * tmp_nom(1) * R_curr * hat_e3 * (J_Q \ eye(3));
% 
%         for iter = 1:5
%             f = u_curr(1);
%             M = u_curr(2:4);
% 
%             Omega_dot = J_Q \ (M - cross(w, J_Q * w));
% 
%             delta_theta = w * dt_pred + 0.5 * Omega_dot * (dt_pred^2);
%             angle = norm(delta_theta);
%             if angle > 1e-8
%                 axis_vec = delta_theta / angle;
%                 hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%                 R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%             else
%                 R_rel = eye(3);
%             end
%             R_pred = R_curr * R_rel;
% 
%             F_calc = f * R_pred * [0; 0; 1];
%             e_F = F_star - F_calc; 
% 
%             dF_df = R_pred * [0; 0; 1];
%             dF_dM = -0.5 * (dt_pred^2) * f * R_curr * hat_e3 * (J_Q \ eye(3));
% 
%             if norm(e_F) < 1e-5
%                 break;
%             end
% 
%             J_F = [dF_df, dF_dM];
% 
%             reg = 0.005;
%             delta_u = (J_F' * J_F + reg * eye(4)) \ (J_F' * e_F + reg * (tmp_nom - u_curr));
% 
%             u_curr = u_curr + delta_u;
%         end
% 
%         tmp_star = u_curr;
%         min_err = norm(e_F);
% 
%         obj.adaptive_gain = [norm(dF_df); norm(dF_dM(:,1)); norm(dF_dM(:,2)); norm(dF_dM(:,3))];
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
% % クアッドコプター用階層型線形化 ＋ 3軸全推力CBF安全補正（原因特定デバッグログ追加版）
% properties
%     self
%     result
%     param
%     adaptive_gain = [1.0; 1.0; 1.0; 1.0]; % [df; dMx; dMy; dMz] 動的感度ゲイン
%     debug_log % 詳細デバッグ情報保存用構造体
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param;                  % ゲイン設定等
%         model = obj.self.estimator.result; % 推定状態
%         ref   = obj.self.reference.result; % 目標軌道
% 
%         % --- 目標値の取得 ---
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd;
%         else
%             xd = ref.state.get();
%         end
% 
%         % --- 紐ベクトル pT の取得/計算 ---
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
% 
%         % --- 物理パラメータの集約 ---
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
% 
%         % 1. 既存制御器用の19次元フル状態ベクトル [q; w; pL; vL; pT; wL]
%         x_full = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
%         % 2. CBF専用の12次元状態ベクトル [pL; vL; pT; wL] (姿勢 q, w を分離)
%         x_cbf = [pL; model.state.vL; pT; model.state.wL];
% 
%         % --- Yaw角の連続性補正 ---
%         yaw = wrapToPi(model.state.q(3));
%         yawd = xd(4);
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         tic_start = tic;
% 
%         % =================================================================
%         % 1. 既存の階層型線形化による公称実入力 [f_nom; M_nom] の算出
%         % =================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4;
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x_full, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x_full, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x_full, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x_full, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x_full, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; % 公称実入力 [f_nom; M_nom] in R^4
%         obj.result.tmp = tmp; 
%         obj.result.controllertime = toc(tic_start);
% 
%         % =================================================================
%         % 2. Flight フェーズ判定および CBF-QP / 状態依存動的ゲイン書き戻し
%         % =================================================================
%         current_phase = "FLIGHT";
%         if isprop(obj.self, 'phase')
%             current_phase = string(obj.self.phase);
%         end
% 
%         if strcmp(current_phase, "FLIGHT")
%             % --- A. 公称実入力 tmp=[f_nom; M_nom] から予測仮想力 F_nom を計算 ---
%             q = model.state.getq('compact');
%             R_curr = quat2rotm(q'); % 現在の機体回転行列
%             w = model.state.w;       % 現在の機体角速度
%             J_Q = diag(P(2:4));      % 慣性モーメント [jx, jy, jz]
%             dt_pred = Param.dt;      % 予測時間幅
% 
%             % トルク M_nom から Rodrigues 公式による姿勢予測 R_pred
%             Omega_dot_nom = J_Q \ (tmp(2:4) - cross(w, J_Q * w));
%             delta_theta = w * dt_pred + 0.5 * Omega_dot_nom * (dt_pred^2);
%             angle = norm(delta_theta);
%             if angle > 1e-8
%                 axis_vec = delta_theta / angle;
%                 hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%                 R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%             else
%                 R_rel = eye(3);
%             end
%             R_pred_nom = R_curr * R_rel;
% 
%             % 世界座標系での公称仮想全推力 F_nom
%             F_nom = tmp(1) * R_pred_nom * [0; 0; 1];
% 
%             % --- B. 12次元状態量 x_cbf と障害物環境を用いた CBF-QP (F* の導出) ---
%             F_star = obj.solve_CBF_QP_3D(x_cbf, F_nom, P);
% 
%             % --- C. 安全仮想力 F* から実入力 tmp_star=[f*; M*] への逆変換 ---
%             [tmp_star, opt_err, iter_cnt] = obj.reconstruct_real_input(F_star, tmp, R_curr, w, J_Q, dt_pred);
% 
%             % --- D. 🌟 原因特定用リアルタイム・詳細デバッグプリント ---
%             dF_norm = norm(F_star - F_nom);
%             fprintf('\n========== [CBF DEBUG STEP] ==========\n');
%             fprintf('F_nom    : [%.2f, %.2f, %.2f] N\n', F_nom(1), F_nom(2), F_nom(3));
%             fprintf('F_star   : [%.2f, %.2f, %.2f] N  (補正差分 ||dF|| = %.2f N)\n', F_star(1), F_star(2), F_star(3), dF_norm);
%             fprintf('逆変換   : 残差Err=%.3e | 反復数=%d | DynGains=[%.2f, %.2f, %.2f, %.2f]\n', ...
%                 opt_err, iter_cnt, obj.adaptive_gain(1), obj.adaptive_gain(2), obj.adaptive_gain(3), obj.adaptive_gain(4));
% 
%             final_tmp = tmp_star;
%         else
%             final_tmp = tmp;
%         end
% 
%         % =================================================================
%         % 3. 最終実入力のガード（入力飽和制限）と不一致検出
%         % =================================================================
%         raw_f = final_tmp(1);
%         raw_M = final_tmp(2:4);
% 
%         sat_f = max(0, min(20, raw_f));
%         sat_Mx = max(-1, min(1, raw_M(1)));
%         sat_My = max(-1, min(1, raw_M(2)));
%         sat_Mz = max(-1, min(1, raw_M(3)));
% 
%         % 🌟 ガード制限でバッサリカットされた場合の警告ログ
%         if abs(raw_f - sat_f) > 1e-3 || norm(raw_M - [sat_Mx; sat_My; sat_Mz]) > 1e-3
%             fprintf('🚨 【入力飽和発生】CBF要求の入力が物理限界を超えました！\n');
%             fprintf('   要求入力: f=%.2f, M=[%.2f, %.2f, %.2f]\n', raw_f, raw_M(1), raw_M(2), raw_M(3));
%             fprintf('   飽和出力: f=%.2f, M=[%.2f, %.2f, %.2f]\n', sat_f, sat_Mx, sat_My, sat_Mz);
%         end
% 
%         obj.result.input = [sat_f; sat_Mx; sat_My; sat_Mz];
%         obj.result.xd = xd;
%         obj.result.x = x_full;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% methods (Access = private)
%     % -----------------------------------------------------------------
%     % 障害物環境情報を動的取得し、全推力 F* を求める CBF-QP ソルバー（デバッグ対応）
%     % -----------------------------------------------------------------
%     function F_star = solve_CBF_QP_3D(obj, x_cbf, F_nom, P)
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
%         rl_val = 0.5; % 保護球半径
%         obj.result.rl = rl_val;
% 
%         L_cable = P(7);
%         pL = x_cbf(1:3);
%         pT = x_cbf(7:9);
%         p_mid = pL - 0.5 * L_cable * pT; % ケーブル中心点
% 
%         A_qp_total = [];
%         b_qp_total = [];
%         gamma_params = [1.2; 8.0]; % 相対次数2用のクラスKゲイン
% 
%         min_surf_dist = Inf;
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             dist_center = norm(p_mid - [xo; yo; zo]);
%             d_surf = dist_center - (ro + rl_val);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             log_p_obs{i} = obs_env(i).p_obs;
%             log_r_obs{i} = obs_env(i).r_obs;
%             if isfield(obs_env(i), 'd_margin')
%                 log_r_minimal{i} = ro - obs_env(i).d_margin;
%             else
%                 log_r_minimal{i} = ro;
%             end
% 
%             % CBF_CBF_force 関数から A_qp, b_qp を取得
%             [A_qp_single, b_qp_single] = CBF_force(obj, x_cbf, obs_params, gamma_params, rl_val, P);
% 
%             % 🌟 各障害物についての詳細デバッグログ
%             lhs_nom = A_qp_single * F_nom;
%             fprintf('障害物[%d] (距離=%.3fm): A_qp*F_nom = %.3f | b_qp = %.3f ', i, d_surf, lhs_nom, b_qp_single);
%             if lhs_nom > b_qp_single
%                 fprintf(' 💥 [CBF発動! 制約違反]\n');
%             else
%                 fprintf(' 🟢 [安全圏内]\n');
%             end
% 
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
% 
%         % ログ情報の保存
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 3自由度全推力 F* に対する QP 最適化
%         H_qp = eye(3);
%         f_qp = -F_nom;
%         options = optimoptions('quadprog', 'Display', 'off');
%         [F_star_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], [], [], [], options);
% 
%         if exitflag >= 1
%             F_star = F_star_opt;
%         else
%             fprintf('⚠️ 【QP求解失敗 (Feasible解なし)】公称値 F_nom へフォールバックします！\n');
%             F_star = F_nom; 
%         end
%     end
% 
%     % -----------------------------------------------------------------
%     % 仮想力 F* に一致する実入力 [f*; M*] への復元（ガウス・ニュートン反復）
%     % -----------------------------------------------------------------
%     % function [tmp_star, min_err, iter] = reconstruct_real_input(obj, F_star, tmp_nom, R_curr, w, J_Q, dt_pred)
%     %     u_curr = tmp_nom; 
%     %     e_F = [0; 0; 0];
%     % 
%     %     hat_e3 = [ 0, -1,  0;
%     %                1,  0,  0;
%     %                0,  0,  0];
%     % 
%     %     % ループ外での初期定義
%     %     dF_df = R_curr * [0; 0; 1];
%     %     dF_dM = -0.5 * (dt_pred^2) * tmp_nom(1) * R_curr * hat_e3 * (J_Q \ eye(3));
%     % 
%     %     for iter = 1:5
%     %         f = u_curr(1);
%     %         M = u_curr(2:4);
%     % 
%     %         Omega_dot = J_Q \ (M - cross(w, J_Q * w));
%     %         delta_theta = w * dt_pred + 0.5 * Omega_dot * (dt_pred^2);
%     %         angle = norm(delta_theta);
%     %         if angle > 1e-8
%     %             axis_vec = delta_theta / angle;
%     %             hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%     %             R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%     %         else
%     %             R_rel = eye(3);
%     %         end
%     %         R_pred = R_curr * R_rel;
%     % 
%     %         F_calc = f * R_pred * [0; 0; 1];
%     %         e_F = F_star - F_calc; 
%     % 
%     %         dF_df = R_pred * [0; 0; 1];
%     %         dF_dM = -0.5 * (dt_pred^2) * f * R_curr * hat_e3 * (J_Q \ eye(3));
%     % 
%     %         if norm(e_F) < 1e-5
%     %             break;
%     %         end
%     % 
%     %         J_F = [dF_df, dF_dM];
%     %         reg = 0.005;
%     %         delta_u = (J_F' * J_F + reg * eye(4)) \ (J_F' * e_F + reg * (tmp_nom - u_curr));
%     %         u_curr = u_curr + delta_u;
%     %     end
%     % 
%     %     tmp_star = u_curr;
%     %     min_err = norm(e_F);
%     %     obj.adaptive_gain = [norm(dF_df); norm(dF_dM(:,1)); norm(dF_dM(:,2)); norm(dF_dM(:,3))];
%     % end
%     function [tmp_star, min_err, iter] = reconstruct_real_input(obj, F_star, tmp_nom, R_curr, w, J_Q, dt_pred)
%         u_curr = tmp_nom; 
%         e_F = [0; 0; 0];
% 
%         hat_e3 = [ 0, -1,  0;
%             1,  0,  0;
%             0,  0,  0];
% 
%         for iter = 1:5
%             f = u_curr(1);
%             M = u_curr(2:4);
% 
%             Omega_dot = J_Q \ (M - cross(w, J_Q * w));
%             delta_theta = w * dt_pred + 0.5 * Omega_dot * (dt_pred^2);
%             angle = norm(delta_theta);
%             if angle > 1e-8
%                 axis_vec = delta_theta / angle;
%                 hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%                 R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%             else
%                 R_rel = eye(3);
%             end
%             R_pred = R_curr * R_rel;
% 
%             F_calc = f * R_pred * [0; 0; 1];
%             e_F = F_star - F_calc; 
% 
%             dF_df = R_pred * [0; 0; 1];
%             dF_dM = -0.5 * (dt_pred^2) * f * R_curr * hat_e3 * (J_Q \ eye(3));
% 
%             if norm(e_F) < 1e-5
%                 break;
%             end
% 
%             J_F = [dF_df, dF_dM];
% 
%             % 🌟【改善ポイント】トルク変化（M - M_nom）に対する重み付きダンピング
%             % 推力 f の追従を優先しつつ、トルク M の急変に強めの正則化をかけて飽和を防ぐ
%             W_reg = diag([0.001, 0.2, 0.2, 0.2]); % 推力重み小、トルク重み大
% 
%             delta_u = (J_F' * J_F + W_reg) \ (J_F' * e_F + W_reg * (tmp_nom - u_curr));
%             u_curr = u_curr + delta_u;
%         end
% 
%         tmp_star = u_curr;
%         min_err = norm(e_F);
%         obj.adaptive_gain = [norm(dF_df); norm(dF_dM(:,1)); norm(dF_dM(:,2)); norm(dF_dM(:,3))];
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
%     % クアッドコプター用階層型線形化 ＋ 3軸全推力CBF安全補正（予知時間＋スルーレート
%     % 制限 早期回避版）回避
% properties
%     self
%     result
%     param
%     adaptive_gain = [1.0; 1.0; 1.0; 1.0]; % [df; dMx; dMy; dMz] 動的感度ゲイン
%     debug_log                              % 詳細デバッグ情報保存用構造体
%     F_star_prev = [0; 0; 9.81];            % スルーレート制限用 (前ステップの CBF 決定力)
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param;                  % ゲイン設定等
%         model = obj.self.estimator.result; % 推定状態
%         ref   = obj.self.reference.result; % 目標軌道
% 
%         % --- 目標値の取得 ---
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd;
%         else
%             xd = ref.state.get();
%         end
% 
%         % --- 紐ベクトル pT の取得/計算 ---
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
% 
%         % --- 物理パラメータの集約 ---
%         P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];
% 
%         % 1. 既存制御器用の19次元フル状態ベクトル [q; w; pL; vL; pT; wL]
%         x_full = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];
% 
%         % 2. CBF専用の12次元状態ベクトル [pL; vL; pT; wL]
%         x_cbf = [pL; model.state.vL; pT; model.state.wL];
% 
%         % --- Yaw角の連続性補正 ---
%         yaw = wrapToPi(model.state.q(3));
%         yawd = xd(4);
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd(4) = -deltaYaw(3) + yaw; 
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         tic_start = tic;
% 
%         % =================================================================
%         % 1. 既存の階層型線形化による公称実入力 [f_nom; M_nom] の算出
%         % =================================================================
%         F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4;
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x_full, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x_full, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x_full, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x_full, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x_full, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
%         tmp = [uf(1); us]; % 公称実入力 [f_nom; M_nom] in R^4
%         obj.result.tmp = tmp; 
%         obj.result.controllertime = toc(tic_start);
% 
%         % =================================================================
%         % 2. Flight フェーズ判定および CBF-QP / 状態依存動的ゲイン書き戻し
%         % =================================================================
%         current_phase = "FLIGHT";
%         if isprop(obj.self, 'phase')
%             current_phase = string(obj.self.phase);
%         end
% 
%         if strcmp(current_phase, "FLIGHT")
%             % --- A. 公称実入力 tmp=[f_nom; M_nom] から予測仮想力 F_nom を計算 ---
%             q = model.state.getq('compact');
%             R_curr = quat2rotm(q'); % 現在の機体回転行列
%             w = model.state.w;       % 現在の機体角速度
%             J_Q = diag(P(2:4));      % 慣性モーメント [jx, jy, jz]
%             dt_pred = Param.dt;      % 予測時間幅
% 
%             % トルク M_nom から Rodrigues 公式による姿勢予測 R_pred
%             Omega_dot_nom = J_Q \ (tmp(2:4) - cross(w, J_Q * w));
%             delta_theta = w * dt_pred + 0.5 * Omega_dot_nom * (dt_pred^2);
%             angle = norm(delta_theta);
%             if angle > 1e-8
%                 axis_vec = delta_theta / angle;
%                 hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%                 R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%             else
%                 R_rel = eye(3);
%             end
%             R_pred_nom = R_curr * R_rel;
% 
%             % 世界座標系での公称仮想全推力 F_nom
%             F_nom = tmp(1) * R_pred_nom * [0; 0; 1];
% 
%             % --- B. 🌟 予知時間を導入した CBF-QP (F* の導出) ---
%             F_star = obj.solve_CBF_QP_3D(x_cbf, F_nom, P);
% 
%             % --- C. 安全仮想力 F* から実入力 tmp_star=[f*; M*] への逆変換 ---
%             [tmp_star, opt_err, iter_cnt] = obj.reconstruct_real_input(F_star, tmp, R_curr, w, J_Q, dt_pred);
% 
%             % --- D. デバッグプリント ---
%             dF_norm = norm(F_star - F_nom);
%             fprintf('\n========== [CBF DEBUG STEP] ==========\n');
%             fprintf('F_nom    : [%.2f, %.2f, %.2f] N\n', F_nom(1), F_nom(2), F_nom(3));
%             fprintf('F_star   : [%.2f, %.2f, %.2f] N  (補正差分 ||dF|| = %.2f N)\n', F_star(1), F_star(2), F_star(3), dF_norm);
%             fprintf('逆変換   : 残差Err=%.3e | 反復数=%d | DynGains=[%.2f, %.2f, %.2f, %.2f]\n', ...
%                 opt_err, iter_cnt, obj.adaptive_gain(1), obj.adaptive_gain(2), obj.adaptive_gain(3), obj.adaptive_gain(4));
% 
%             final_tmp = tmp_star;
%         else
%             final_tmp = tmp;
%         end
% 
%         % =================================================================
%         % 3. 最終実入力のガード（入力飽和制限）と不一致検出
%         % =================================================================
%         raw_f = final_tmp(1);
%         raw_M = final_tmp(2:4);
%         sat_f = max(0, min(20, raw_f));
%         sat_Mx = max(-1, min(1, raw_M(1)));
%         sat_My = max(-1, min(1, raw_M(2)));
%         sat_Mz = max(-1, min(1, raw_M(3)));
% 
%         if abs(raw_f - sat_f) > 1e-3 || norm(raw_M - [sat_Mx; sat_My; sat_Mz]) > 1e-3
%             fprintf('🚨 【入力飽和発生】CBF要求の入力が物理限界を超えました！\n');
%             fprintf('   要求入力: f=%.2f, M=[%.2f, %.2f, %.2f]\n', raw_f, raw_M(1), raw_M(2), raw_M(3));
%             fprintf('   飽和出力: f=%.2f, M=[%.2f, %.2f, %.2f]\n', sat_f, sat_Mx, sat_My, sat_Mz);
%         end
% 
%         obj.result.input = [sat_f; sat_Mx; sat_My; sat_Mz];
%         obj.result.xd = xd;
%         obj.result.x = x_full;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% methods (Access = private)
%     % -----------------------------------------------------------------
%     % 🌟 予知時間 (1.2s) ＋ スルーレート制限を組み込んだ CBF-QP ソルバー
%     % -----------------------------------------------------------------
%     function F_star = solve_CBF_QP_3D(obj, x_cbf, F_nom, P)
%         obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
%         num_obs = length(obs_env);
% 
%         % 🌟 1. 物理保護半径 rl は本来の寸法 (0.5m) を維持！
%         rl_val = 0.5; 
%         obj.result.rl = rl_val;
% 
%         L_cable = P(7);
%         pL = x_cbf(1:3);
%         vL = x_cbf(4:6);
%         pT = x_cbf(7:9);
% 
%         % 🌟 2. 進行方向の予知時間 tau (1.2秒先の位置を監視して手前からゆっくり回避を開始)
%         tau_pred = 1.2; 
%         pL_pred  = pL + tau_pred * vL; % 1.2秒後の予測荷物位置
% 
%         % 予測状態量ベクトルの構築
%         x_cbf_pred = x_cbf;
%         x_cbf_pred(1:3) = pL_pred; % 位置成分だけ予知位置に置換
% 
%         p_mid_curr = pL - 0.5 * L_cable * pT;
% 
%         A_qp_total = [];
%         b_qp_total = [];
% 
%         % クラスKゲイン (手前からマイルドに効かせる設定)
%         gamma_params = [0.5; 50]; 
% 
%         min_surf_dist = Inf;
%         log_p_obs = cell(1, num_obs);
%         log_r_obs = cell(1, num_obs);
%         log_r_minimal = cell(1, num_obs);
% 
%         for i = 1:num_obs
%             xo = obs_env(i).p_obs(1);
%             yo = obs_env(i).p_obs(2);
%             zo = obs_env(i).p_obs(3);
%             ro = obs_env(i).r_obs;
%             obs_params = [xo; yo; zo; ro];
% 
%             % 現在位置での距離（クリアランス表示用）
%             dist_center = norm(p_mid_curr - [xo; yo; zo]);
%             d_surf = dist_center - (ro + rl_val);
%             if d_surf < min_surf_dist
%                 min_surf_dist = d_surf;
%             end
% 
%             log_p_obs{i} = obs_env(i).p_obs;
%             log_r_obs{i} = obs_env(i).r_obs;
%             if isfield(obs_env(i), 'd_margin')
%                 log_r_minimal{i} = ro - obs_env(i).d_margin;
%             else
%                 log_r_minimal{i} = ro;
%             end
% 
%             % 🌟 予知位置 x_cbf_pred を代入して制約を生成
%             [A_qp_single, b_qp_single] = CBF_force(obj, x_cbf_pred, obs_params, gamma_params, rl_val, P);
% 
%             lhs_nom = A_qp_single * F_nom;
%             fprintf('障害物[%d] (現在距離=%.3fm): A_qp*F_nom = %.3f | b_qp = %.3f ', i, d_surf, lhs_nom, b_qp_single);
%             if lhs_nom > b_qp_single
%                 fprintf(' 💥 [CBF発動! 予知回避展開]\n');
%             else
%                 fprintf(' 🟢 [安全圏内]\n');
%             end
% 
%             A_qp_total = [A_qp_total; A_qp_single];
%             b_qp_total = [b_qp_total; b_qp_single];
%         end
% 
%         obj.result.min_clearance = min_surf_dist;
%         obj.result.p_obs     = log_p_obs;
%         obj.result.r_obs     = log_r_obs;
%         obj.result.r_minimal = log_r_minimal;
% 
%         % 🌟 3. 1ステップ(0.025s)あたりの最大力変化幅 (スルーレート制限)
%         % 激しい力のジャンプを物理的に防止してトルク飽和を防ぐ
%         max_dF = [1.5; 1.5; 2.0]; % N/step
%         lb = max([-8.0; -8.0;  2.0], obj.F_star_prev - max_dF);
%         ub = min([ 8.0;  8.0; 18.0], obj.F_star_prev + max_dF);
% 
%         H_qp = eye(3);
%         f_qp = -F_nom;
%         options = optimoptions('quadprog', 'Display', 'off');
% 
%         [F_star_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);
% 
%         if exitflag >= 1
%             F_star = F_star_opt;
%         else
%             F_star = max(lb, min(ub, F_nom));
%         end
% 
%         obj.F_star_prev = F_star; % 前ステップの力を保存
%     end
% 
%     % -----------------------------------------------------------------
%     % 仮想力 F* に一致する実入力 [f*; M*] への復元（トルク最適化版）
%     % -----------------------------------------------------------------
%     function [tmp_star, min_err, iter] = reconstruct_real_input(obj, F_star, tmp_nom, R_curr, w, J_Q, dt_pred)
%         u_curr = tmp_nom; 
%         e_F = [0; 0; 0];
%         hat_e3 = [ 0, -1,  0;
%                    1,  0,  0;
%                    0,  0,  0];
% 
%         for iter = 1:5
%             f = u_curr(1);
%             M = u_curr(2:4);
% 
%             Omega_dot = J_Q \ (M - cross(w, J_Q * w));
%             delta_theta = w * dt_pred + 0.5 * Omega_dot * (dt_pred^2);
%             angle = norm(delta_theta);
%             if angle > 1e-8
%                 axis_vec = delta_theta / angle;
%                 hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
%                 R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
%             else
%                 R_rel = eye(3);
%             end
%             R_pred = R_curr * R_rel;
% 
%             F_calc = f * R_pred * [0; 0; 1];
%             e_F = F_star - F_calc; 
% 
%             dF_df = R_pred * [0; 0; 1];
%             dF_dM = -0.5 * (dt_pred^2) * f * R_curr * hat_e3 * (J_Q \ eye(3));
% 
%             if norm(e_F) < 1e-5
%                 break;
%             end
% 
%             J_F = [dF_df, dF_dM];
% 
%             % 🌟 トルク変化の重みを 0.2 -> 0.05 にマイルド化（手前からの滑らかな姿勢制御を許可）
%             W_reg = diag([0.001, 0.05, 0.05, 0.05]); 
% 
%             delta_u = (J_F' * J_F + W_reg) \ (J_F' * e_F + W_reg * (tmp_nom - u_curr));
%             u_curr = u_curr + delta_u;
%         end
% 
%         tmp_star = u_curr;
%         min_err = norm(e_F);
%         obj.adaptive_gain = [norm(dF_df); norm(dF_dM(:,1)); norm(dF_dM(:,2)); norm(dF_dM(:,3))];
%     end
% end
% end

classdef HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK < handle
    % クアッドコプター用階層型線形化 ＋ 3軸全推力Predictive CBF安全補正
    % (理論符号修正済 ＋ 流体誘導ベクトル ＋ 目標引き寄せ力自動調整 ＋ ケーブル傾き制約)
    properties
        self
        result
        param
        adaptive_gain = [1.0; 1.0; 1.0; 1.0]; % [df; dMx; dMy; dMz] 動的感度ゲイン
        debug_log                              % 詳細デバッグ情報保存用構造体
        F_star_prev = [0; 0; 9.81];            % スルーレート制限用 (前ステップの CBF 決定力)
    end

    methods
        function obj = HLC_SUSPENDED_LOAD_HOCBF_LOAD_Z_LINK(self, param)
            obj.self = self;
            obj.param = param;
        end

        function result = do(obj, varargin)
            Param = obj.param;                  % ゲイン設定等
            model = obj.self.estimator.result; % 推定状態
            ref   = obj.self.reference.result; % 目標軌道

            % --- 目標値の取得 ---
            if isprop(ref.state, 'xd')
                xd = ref.state.xd;
            else
                xd = ref.state.get();
            end

            % --- 紐ベクトル pT の取得/計算 ---
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

            % --- 物理パラメータの集約 ---
            P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];

            % 1. 既存制御器用の19次元フル状態ベクトル [q; w; pL; vL; pT; wL]
            x_full = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL];

            % 2. CBF専用の12次元状態ベクトル [pL; vL; pT; wL]
            x_cbf = [pL; model.state.vL; pT; model.state.wL];

            % --- Yaw角の連続性補正 ---
            yaw = wrapToPi(model.state.q(3));
            yawd = xd(4);
            yawUnit = [cos(yaw); sin(yaw); 0]; 
            yawdUnit = [cos(yawd); sin(yawd); 0]; 
            deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
            xd(4) = -deltaYaw(3) + yaw; 
            xd = [xd; zeros(28 - size(xd, 1), 1)];

            tic_start = tic;

            % =================================================================
            % 1. 既存の階層型線形化による公称実入力 [f_nom; M_nom] の算出
            % =================================================================
            F1 = Param.F1; F2 = Param.F2; F3 = Param.F3; F4 = Param.F4;
            vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x_full, xd', F1); 
            vs = obj.Vs_SuspendedLoadxyDst(x_full, xd', vf, P, F2, F3, F4); 
            uf = obj.Uf_SuspendedLoadxyDst(x_full, xd', vf, P); 
            beta2 = obj.Beta2_SuspendedLoadxyDst(x_full, xd', vf, P); 
            vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x_full, xd', vf, vs', P); 
            us = beta2 \ vs_alpha2; 
            tmp = [uf(1); us]; % 公称実入力 [f_nom; M_nom] in R^4
            obj.result.tmp = tmp; 
            obj.result.controllertime = toc(tic_start);

            % =================================================================
            % 2. Flight フェーズ判定および Predict-CBF-QP / 実入力復元
            % =================================================================
            current_phase = "FLIGHT";
            if isprop(obj.self, 'phase')
                current_phase = string(obj.self.phase);
            end

            if strcmp(current_phase, "FLIGHT")
                % --- A. 姿勢予測 R_pred による予測仮想力 F_nom を計算 ---
                q = model.state.getq('compact');
                R_curr = quat2rotm(q'); % 現在の機体回転行列
                w = model.state.w;       % 現在の機体角速度
                J_Q = diag(P(2:4));      % 慣性モーメント [jx, jy, jz]
                dt_pred = Param.dt;      % 予測時間幅

                % トルク M_nom から Rodrigues 公式による姿勢予測 R_pred
                Omega_dot_nom = J_Q \ (tmp(2:4) - cross(w, J_Q * w));
                delta_theta = w * dt_pred + 0.5 * Omega_dot_nom * (dt_pred^2);
                angle = norm(delta_theta);
                if angle > 1e-8
                    axis_vec = delta_theta / angle;
                    hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
                    R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
                else
                    R_rel = eye(3);
                end
                R_pred_nom = R_curr * R_rel;

                % 姿勢遅れ補償済みの予測公称仮想全推力 F_nom
                F_nom = tmp(1) * R_pred_nom * [0; 0; 1];

                % 🌟【引き寄せ暴走防止】回避による遅れに伴う強烈な直線強要力をマイルド化
                L_cable = P(7);
                p_mid_curr = model.state.pL - 0.5 * L_cable * model.state.pT;
                obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
                min_d = Inf;
                for io = 1:length(obs_env)
                    d_tmp = norm(p_mid_curr - obs_env(io).p_obs(:)) - (obs_env(io).r_obs + 0.5);
                    if d_tmp < min_d, min_d = d_tmp; end
                end

                % 障害物接近時（2.0m以内）、水平方向の目標引き寄せ力を自動スケールカット
                if min_d < 2.0
                    scale_factor = max(0.2, min_d / 2.0); 
                    F_nom(1:2) = F_nom(1:2) * scale_factor;
                end

                % --- B. 🌟 障害物回避 ＋ ケーブル傾き制約を合体した CBF-QP (F* の導出) ---
                F_star = obj.solve_CBF_QP_3D(x_cbf, F_nom, P);

                % --- C. 安全仮想力 F* から実入力 tmp_star=[f*; M*] への逆変換 ---
                [tmp_star, opt_err, iter_cnt] = obj.reconstruct_real_input(F_star, tmp, R_curr, w, J_Q, dt_pred);

                % --- D. デバッグプリント ---
                dF_norm = norm(F_star - F_nom);
                fprintf('\n========== [CBF DEBUG STEP] ==========\n');
                fprintf('F_nom    : [%.2f, %.2f, %.2f] N\n', F_nom(1), F_nom(2), F_nom(3));
                fprintf('F_star   : [%.2f, %.2f, %.2f] N  (補正差分 ||dF|| = %.2f N)\n', F_star(1), F_star(2), F_star(3), dF_norm);
                fprintf('逆変換   : 残差Err=%.3e | 反復数=%d | DynGains=[%.2f, %.2f, %.2f, %.2f]\n', ...
                    opt_err, iter_cnt, obj.adaptive_gain(1), obj.adaptive_gain(2), obj.adaptive_gain(3), obj.adaptive_gain(4));

                final_tmp = tmp_star;
            else
                final_tmp = tmp;
            end

            % =================================================================
            % 3. 最終実入力のガード（入力飽和制限）と不一致検出
            % =================================================================
            raw_f = final_tmp(1);
            raw_M = final_tmp(2:4);
            sat_f = max(0, min(20, raw_f));
            sat_Mx = max(-1, min(1, raw_M(1)));
            sat_My = max(-1, min(1, raw_M(2)));
            sat_Mz = max(-1, min(1, raw_M(3)));

            if abs(raw_f - sat_f) > 1e-3 || norm(raw_M - [sat_Mx; sat_My; sat_Mz]) > 1e-3
                fprintf('🚨 【入力飽和発生】CBF要求の入力が物理限界を超えました！\n');
                fprintf('   要求入力: f=%.2f, M=[%.2f, %.2f, %.2f]\n', raw_f, raw_M(1), raw_M(2), raw_M(3));
                fprintf('   飽和出力: f=%.2f, M=[%.2f, %.2f, %.2f]\n', sat_f, sat_Mx, sat_My, sat_Mz);
            end

            obj.result.input = [sat_f; sat_Mx; sat_My; sat_Mz];
            obj.result.xd = xd;
            obj.result.x = x_full;
            result = obj.result;
        end

        function show(obj)
            obj.result
        end
    end

    methods (Access = private)
        % -----------------------------------------------------------------
        % 🌟 理論符号完全修正 ＋ 接線流体誘導ベクトル型 CBF-QP ソルバー
        % -----------------------------------------------------------------
        function F_star = solve_CBF_QP_3D(obj, x_cbf, F_nom, P)
            obs_env = ENVIRONMENT_OBSTACLE_HOCBF_Z(); 
            num_obs = length(obs_env);

            % 本来の物理保護半径 (0.5m)
            rl_val = 0.5; 
            obj.result.rl = rl_val;

            m_total = P(1) + P(6); % 総質量 (m_Q + m_L)
            g_acc   = P(5);        % 重力加速度 (9.81)
            L_cable = P(7);

            pL = x_cbf(1:3);
            vL = x_cbf(4:6);
            pT = x_cbf(7:9);

            p_mid = pL - 0.5 * L_cable * pT;
            v_mid = vL; 

            A_qp_total = [];
            b_qp_total = [];
            
            % HOCBF ゲイン (滑らかに手前から効かせる設定)
            gamma1 = 1.0;   
            gamma2 = 7.0;   

            min_surf_dist = Inf;
            log_p_obs = cell(1, num_obs);
            log_r_obs = cell(1, num_obs);
            log_r_minimal = cell(1, num_obs);

            % 🌟 流れる回避のための接線誘導ベクトル
            F_flow_bias = [0; 0; 0];

            for i = 1:num_obs
                p_obs = obs_env(i).p_obs(:);
                ro    = obs_env(i).r_obs;
                R_safe = ro + rl_val;

                % 1. 相対位置ベクトル (障害物 -> 機体/荷物)
                r_vec = p_mid - p_obs;
                dist = norm(r_vec);
                d_surf = dist - R_safe;

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

                % 2. 基礎安全関数 h と その時間微分 dot_h
                h_val     = dist^2 - R_safe^2;
                dot_h_val = 2 * (r_vec' * v_mid);

                % 🌟 3. Lie微分の正当な数学展開 (符号全修正)
                % A_qp * F <= b_qp
                A_qp_single = -(2 / m_total) * r_vec'; 
                b_qp_single = 2 * norm(v_mid)^2 - 2 * r_vec(3) * g_acc ...
                              + gamma1 * dot_h_val + gamma2 * h_val;

                lhs_nom = A_qp_single * F_nom;
                fprintf('障害物[%d] (距離=%.3fm): A_qp*F_nom = %.3f | b_qp = %.3f ', i, d_surf, lhs_nom, b_qp_single);
                if lhs_nom > b_qp_single
                    fprintf(' 💥 [CBF介入: 回避行動を展開]\n');
                else
                    fprintf(' 🟢 [安全域]\n');
                end

                % 🌟 4. 【流れるような美しい回避を生むベクトルフィールド誘導】
                if d_surf < 2.2
                    v_dir = v_mid / (norm(v_mid) + 1e-5);
                    r_dir = r_vec / (norm(r_vec) + 1e-5);
                    
                    % 避けるべき接線方向 (側方・上方向)
                    tangent_vec = cross(cross(r_dir, v_dir), r_dir);
                    if norm(tangent_vec) < 1e-3
                        tangent_vec = [0; 1; 0.5]; % 正面突撃時のエスケープ
                    else
                        tangent_vec = tangent_vec / norm(tangent_vec);
                    end

                    weight = (2.2 - d_surf) / 2.2;
                    F_flow_bias = F_flow_bias + weight * (2.5 * tangent_vec + [0; 0; 0.8]); 
                end

                A_qp_total = [A_qp_total; A_qp_single];
                b_qp_total = [b_qp_total; b_qp_single];
            end

            % ケーブル傾き角制約 (<= 18度)
            theta_max_deg = 15.0;
            gamma_cable   = [2.0; 5.0];
            [A_cb, b_cb] = CBF_cable_angle(obj, x_cbf, theta_max_deg, gamma_cable, P);
            A_qp_total = [A_qp_total; A_cb];
            b_qp_total = [b_qp_total; b_cb];

            obj.result.min_clearance = min_surf_dist;
            obj.result.p_obs     = log_p_obs;
            obj.result.r_obs     = log_r_obs;
            obj.result.r_minimal = log_r_minimal;

            % 🌟 スルーレート制限 (推力落ちによる墜落を予防し、下限3.0Nを維持)
            max_dF = [2.0; 2.0; 3.0]; % [N/step]
            lb = max([-10.0; -10.0;  3.0], obj.F_star_prev - max_dF); 
            ub = min([ 10.0;  10.0; 22.0], obj.F_star_prev + max_dF);

            % 🌟 コスト関数に接線バイアスを投入し「滑らかな流体回避」を選択させる
            H_qp = eye(3);
            f_qp = -(F_nom + F_flow_bias);

            options = optimoptions('quadprog', 'Display', 'off');

            [F_star_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp_total, b_qp_total, [], [], lb, ub, [], options);

            if exitflag >= 1
                F_star = F_star_opt;
            else
                F_star = max(lb, min(ub, F_nom));
            end

            obj.F_star_prev = F_star; 
        end

        % -----------------------------------------------------------------
        % 仮想力 F* に一致する実入力 [f*; M*] への復元（R_pred 完全準拠版）
        % -----------------------------------------------------------------
        function [tmp_star, min_err, iter] = reconstruct_real_input(obj, F_star, tmp_nom, R_curr, w, J_Q, dt_pred)
            u_curr = tmp_nom; 
            e_F = [0; 0; 0];
            hat_e3 = [ 0, -1,  0;
                       1,  0,  0;
                       0,  0,  0];

            for iter = 1:5
                f = u_curr(1);
                M = u_curr(2:4);

                % Rodrigues 公式による姿勢予測
                Omega_dot = J_Q \ (M - cross(w, J_Q * w));
                delta_theta = w * dt_pred + 0.5 * Omega_dot * (dt_pred^2);
                angle = norm(delta_theta);
                if angle > 1e-8
                    axis_vec = delta_theta / angle;
                    hat_axis = [0, -axis_vec(3), axis_vec(2); axis_vec(3), 0, -axis_vec(1); -axis_vec(2), axis_vec(1), 0];
                    R_rel = eye(3) + sin(angle) * hat_axis + (1 - cos(angle)) * (hat_axis^2);
                else
                    R_rel = eye(3);
                end
                R_pred = R_curr * R_rel;

                F_calc = f * R_pred * [0; 0; 1];
                e_F = F_star - F_calc; 

                dF_df = R_pred * [0; 0; 1];
                dF_dM = -0.5 * (dt_pred^2) * f * R_curr * hat_e3 * (J_Q \ eye(3));

                if norm(e_F) < 1e-5
                    break;
                end

                J_F = [dF_df, dF_dM];
                
                % トルク急変を適切にダンピングし、飽和を防ぐ正則化重み
                W_reg = diag([1e-4, 0.08, 0.08, 0.08]); 
                delta_u = (J_F' * J_F + W_reg) \ (J_F' * e_F + W_reg * (tmp_nom - u_curr));
                u_curr = u_curr + delta_u;
            end

            tmp_star = u_curr;
            min_err = norm(e_F);
            obj.adaptive_gain = [norm(dF_df); norm(dF_dM(:,1)); norm(dF_dM(:,2)); norm(dF_dM(:,3))];
        end
    end
end