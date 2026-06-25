% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, t, varargin)
%         % 🚨【新規追記】：流れ込んできた t が TIME オブジェクトだった場合の数値抽出処理
%         if isobject(t) || isstruct(t)
%             if isprop(t, 't') || isfield(t, 't')
%                 t = t.t; % TIME.t から double の秒数を取り出す
%             elseif isprop(t, 'time') || isfield(t, 'time')
%                 t = t.time; % もしプロパティ名が time だった場合
%             else
%                 try
%                     t = double(t); % 強制型変換を試みる
%                 catch
%                     t = 0; % どうしてもダメなら0でフォールバック
%                 end
%             end
%         end
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
%         % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
%         % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
%         % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
%         % -----------------------------------------------------------------
%         % 名目（Nominal）仮想操作量の計算 (既存部分)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
%         v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; % [v1_cmd; u2_cmd; u3_cmd]
%         % -----------------------------------------------------------------
%         % 🔥【新規追記】完全同期型 HO-CBF-QP フィルター
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obstacles = length(obs_env);  
% 
%         % 保護球の配置定義 [lambda値 (0:荷物 〜 1:機体), その球の半径]
%         protection_config = [
%             0.00,               0.30;   % 1: 牽引物本体
%             0.25,               0.25;   % 2: ワイヤー下部
%             0.50,               0.25;   % 3: ワイヤー中間
%             0.75,               0.25;   % 4: ワイヤー上部
%             1.00,               0.40;   % 5: ドローン本体
%         ];
%         num_spheres = size(protection_config, 1);
% 
%         % 5階層のHO-CBFゲイン (滑らかさを担保するため、最初は[1.5〜3.0]程度で同調するのが安全です)
%         % K_gains = [2.0, 2.0, 2.0, 2.0, 2.0]; 
%         % K_gains = [0.2, 0.2, 0.2, 0.2, 0.2];
%         % K_gains = [0.05, 0.05, 0.05, 0.05, 0.05];
%         K_gains = [0.005, 0.005, 0.005, 0.005, 0.005];
% 
%         % -----------------------------------------------------------------
%         % 🔥【デバッグ確認】入力変数と状態変数の生データチェック
%         % -----------------------------------------------------------------
%         fprintf('\n====== HLC DEBUG START (t = %.3f) ======\n', t);
%         fprintf('v_nominal (名目入力): [%.4f, %.4f, %.4f]\n', v_nominal(1), v_nominal(2), v_nominal(3));
%         fprintf('pL (荷物位置): [%.4f, %.4f, %.4f]\n', pL(1), pL(2), pL(3));
%         fprintf('pT (ワイヤー方向): [%.4f, %.4f, %.4f]\n', pT(1), pT(2), pT(3));
%         A_qp = [];
%         b_qp = [];
% 
%         for i = 1:num_obstacles
%             ox_val = obs_env(i).p_obs(1);
%             oy_val = obs_env(i).p_obs(2);
%             oz_val = obs_env(i).p_obs(3);
%             ro_val = obs_env(i).r_obs; 
% 
%             for j = 1:num_spheres
%                 lambda_j = protection_config(j, 1);
%                 r_sph_j  = protection_config(j, 2);
% 
%                 cbf_base = [ox_val; oy_val; oz_val; ro_val; r_sph_j; lambda_j; ...
%                             K_gains(1); K_gains(2); K_gains(3); K_gains(4); K_gains(5)];
%                 cbfParam = [cbf_base; P(:)]; 
% 
%                 % 関数の実行時間を計測
%                 tic;
%                 [A_s, b_s] = CBF_Constraints_Synced_Order5([], x, xd', cbfParam, t);
%                 % =========================================================
%                 % 🚨【幾何学ミッチマッチ確認用デバッグコードの修正】
%                 % =========================================================
%                 % 1. 生成スクリプトと100%同期した正しい球の位置を再現（+ から - へ変更）
%                 p_sphere_now = pL - lambda_j * P(7) * pT; % pl - lambda * cableL * pT
% 
%                 % 2. ドローン側が認識している「障害物との距離の2乗」を計算
%                 dist_sq = (p_sphere_now(1) - ox_val)^2 + (p_sphere_now(2) - oy_val)^2 + (p_sphere_now(3) - oz_val)^2;
%                 dist_actual = sqrt(dist_sq); % 物理的な中心間距離 (m)
% 
%                 % 3. 衝突限界半径の合計
%                 limit_radius = ro_val + r_sph_j;
% 
%                 % 4. 本当の幾何学的マージン (これが0以下なら物理的にめり込んでいる)
%                 geometric_margin = dist_actual - limit_radius;
% 
%                 % 5. ログに詳細をすべて焼き出す
%                 fprintf('  [[ 幾何チェック - 障害物%d / 球%d (lambda=%.2f) ]]\n', i, j, lambda_j);
%                 fprintf('    ・計算上の球の位置 : [%.3f, %.3f, %.3f]\n', p_sphere_now(1), p_sphere_now(2), p_sphere_now(3));
%                 fprintf('    ・計算上の障害物   : [%.3f, %.3f, %.3f]\n', ox_val, oy_val, oz_val);
%                 fprintf('    ・中心間距離 : %.3f m  (衝突限界: %.3f m)\n', dist_actual, limit_radius);
%                 fprintf('    ・本当の残り距離 : %.3f m\n', geometric_margin);
%                 % =========================================================
%                 eval_time = toc;
% 
%                 % 🌟【超重要】吐き出された数値のチェック
%                 if any(isnan(A_s)) || any(isinf(A_s)) || any(isnan(b_s)) || any(isinf(b_s))
%                     fprintf('  ⚠️ [警告] 障害物 %d, 球 %d で NaN または Inf が発生！\n', i, j);
%                     fprintf('     A_s: [%s], b_s: %.4f\n', num2str(A_s, ' %.2e'), b_s);
%                 elseif norm(A_s) < 1e-12
%                     fprintf('  ⚠️ [警告] 障害物 %d, 球 %d のゲイン A_s がほぼゼロ (%.2e) です。入力が効いていません。\n', i, j, norm(A_s));
%                 else
%                     fprintf('  ℹ️ 障害物 %d, 球 %d: 計算時間 = %.4f秒, A_sノルム = %.2e, b_s = %.2f\n', i, j, eval_time, norm(A_s), b_s);
%                 end
% 
%                 A_qp = [A_qp; -A_s];
%                 b_qp = [b_qp;  b_s];
%             end
%         end
% 
%         fprintf('--- quadprog 直前チェック ---\n');
%         fprintf('総制約数 (A_qpの行数): %d\n', size(A_qp, 1));
% 
%         % 🚨【最重要追記】：これまで定義されていなかった二次形式用の重み行列をここで定義
%         % Q_matrix = diag([1.0, 1.0, 1.0]); % [v1, u2, u3] の変動に対するペナルティ重み
%         Q_matrix = diag([1.0, 0.001, 0.001]);
%         H_quad   = Q_matrix;              % quadprog用のHessian
%         f_quad   = -Q_matrix * v_nominal; % quadprog用の線形ベクトル
% 
%         % 🌟 quadprogの実行前パルス
%         disp('>> quadprog を起動します...');
% 
%         % 実機・実験用の超高速オプション設定
%         % 実機・実験用の超高速オプション設定
%         qp_options = optimoptions('quadprog', 'Display', 'final', 'Algorithm', 'active-set');
% 
%         tic;
%         try
%             % 🚨【修正箇所】：第9引数の [] を v_nominal に書き換え、初期点として与える
%             [v_safe, ~, exitflag] = quadprog(H_quad, f_quad, A_qp, b_qp, [], [], [], [], v_nominal, qp_options);
%             qp_time = toc;
%             fprintf('>> quadprog 終了: 計算時間 = %.4f秒, exitflag = %d\n', qp_time, exitflag);
%             fprintf('v_safe (安全入力): [%.4f, %.4f, %.4f]\n', v_safe(1), v_safe(2), v_safe(3));
%             fprintf('入力変化量: [%.4f, %.4f, %.4f]\n', v_safe(1)-v_nominal(1), v_safe(2)-v_nominal(2), v_safe(3)-v_nominal(3));
%         catch ME
%             fprintf('❌ quadprog 内部でエラー発生: %s\n', ME.message);
%             v_safe = v_nominal; 
%         end
%         fprintf('====== HLC DEBUG END ======\n\n');
% 
%         % -----------------------------------------------------------------
%         % 安全化された仮想操作量を、各下流レイヤーの変数へ分配 (既存部分)
%         % -----------------------------------------------------------------
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_safe(1); % 完全連続性が保証された安全な v1_cmd
%         vs_safe(1) = v_safe(2); % 完全連続性が保証された安全な u2_cmd
%         vs_safe(2) = v_safe(3); % 完全連続性が保証された安全な u3_cmd
% 
%         % -----------------------------------------------------------------
%         % 下流レイヤーへの非線形相殺・実物理入力への変換 (既存部分)
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, t, varargin)
%         % 🚨【新規追記】：流れ込んできた t が TIME オブジェクトだった場合の数値抽出処理
%         if isobject(t) || isstruct(t)
%             if isprop(t, 't') || isfield(t, 't')
%                 t = t.t; % TIME.t から double の秒数を取り出す
%             elseif isprop(t, 'time') || isfield(t, 'time')
%                 t = t.time; % もしプロパティ名が time だった場合
%             else
%                 try
%                     t = double(t); % 強制型変換を試みる
%                 catch
%                     t = 0; % どうしてもダメなら0でフォールバック
%                 end
%             end
%         end
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
%         % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
%         % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
%         % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
%         yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
%         yawd = xd(4); % 目標yaw角
%         yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
%         yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
%         xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
%         %目標値の格納
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
%         % -----------------------------------------------------------------
%         % 名目（Nominal）仮想操作量の計算 (既存部分)
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x, xd', vf_nominal, P, F2, F3, F4); 
%         v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; % [v1_cmd; u2_cmd; u3_cmd]
%         % -----------------------------------------------------------------
%         % 🔥【新規追記】完全同期型 HO-CBF-QP フィルター
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obstacles = length(obs_env);  
% 
%         % 保護球の配置定義 [lambda値 (0:荷物 〜 1:機体), その球の半径]
%         protection_config = [
%             0.00,               0.30;   % 1: 牽引物本体
%             0.25,               0.25;   % 2: ワイヤー下部
%             0.50,               0.25;   % 3: ワイヤー中間
%             0.75,               0.25;   % 4: ワイヤー上部
%             1.00,               0.40;   % 5: ドローン本体
%         ];
%         num_spheres = size(protection_config, 1);
% 
%         % 5階層のHO-CBFゲイン (滑らかさを担保するため、最初は小さく絞ります)
%         K_gains = [0.05, 0.05, 0.05, 0.05, 0.05];
% 
%         % -----------------------------------------------------------------
%         % 🔥【デバッグ確認】入力変数と状態変数の生データチェック
%         % -----------------------------------------------------------------
%         fprintf('\n====== HLC DEBUG START (t = %.3f) ======\n', t);
%         fprintf('v_nominal (名目入力): [%.4f, %.4f, %.4f]\n', v_nominal(1), v_nominal(2), v_nominal(3));
%         fprintf('pL (荷物位置): [%.4f, %.4f, %.4f]\n', pL(1), pL(2), pL(3));
%         fprintf('pT (ワイヤー方向): [%.4f, %.4f, %.4f]\n', pT(1), pT(2), pT(3));
%         A_qp = [];
%         b_qp = [];
% 
%         for i = 1:num_obstacles
%             ox_val = obs_env(i).p_obs(1);
%             oy_val = obs_env(i).p_obs(2);
%             oz_val = obs_env(i).p_obs(3);
%             oz_val = 0.0; % 直撃コース設定用
%             ro_val = obs_env(i).r_obs; 
% 
%             for j = 1:num_spheres
%                 lambda_j = protection_config(j, 1);
%                 r_sph_j  = protection_config(j, 2);
% 
%                 cbf_base = [ox_val; oy_val; oz_val; ro_val; r_sph_j; lambda_j; ...
%                             K_gains(1); K_gains(2); K_gains(3); K_gains(4); K_gains(5)];
% 
%                 % ---------------------------------------------------------
%                 % 🔥【パラメータ疎通確認自動テストデバッグ】
%                 % ---------------------------------------------------------
%                 % ① 通常時のパラメータでの計算
%                 cbfParam_normal = [cbf_base; P(:)]; 
%                 tic;
%                 [A_s_normal, b_s_normal] = CBF_Constraints_Synced_Order5([], x, xd', cbfParam_normal, t);
%                 eval_time = toc;
% 
%                 % ② ワイヤー長をわざと10倍（偽装）した偽パラメータの作成
%                 P_fake = P;
%                 P_fake(7) = P(7) * 10.0; 
%                 cbfParam_fake = [cbf_base; P_fake(:)];
% 
%                 % ③ 偽パラメータによる再計算
%                 [~, b_s_fake] = CBF_Constraints_Synced_Order5([], x, xd', cbfParam_fake, t);
% 
%                 % ④ 判定結果のログ焼き出し
%                 fprintf('  [[ パラメータ検証 - 障害物%d / 球%d ]]\n', i, j);
%                 fprintf('    ・通常時結果   : A_norm = %.2e, b_s = %.4f\n', norm(A_s_normal), b_s_normal);
%                 fprintf('    ・ワイヤー10倍 : A_norm = %.2e, b_s = %.4f\n', norm(A_s_normal), b_s_fake);
% 
%                 if abs(b_s_normal - b_s_fake) < 1e-6
%                     fprintf('    ❌ [重大な警告] パラメータを変えても出力が1ミリも変わりません！\n');
%                     fprintf('                    数式生成(generate)時のsubs置換が空振りして数式が麻痺しています。\n');
%                 else
%                     fprintf('    ⭕ [大成功] パラメータ変更によって数式の出力が激変しました！\n');
%                     fprintf('                物理パラメータは数式内部へ100%%正しく届いています。\n');
%                 end
% 
%                 % ⑤ 本来の幾何学的残り距離のデバッグ表示
%                 p_sphere_now = pL - lambda_j * P(7) * pT; 
%                 dist_sq = (p_sphere_now(1) - ox_val)^2 + (p_sphere_now(2) - oy_val)^2 + (p_sphere_now(3) - oz_val)^2;
%                 dist_actual = sqrt(dist_sq); 
%                 limit_radius = ro_val + r_sph_j;
%                 geometric_margin = dist_actual - limit_radius;
% 
%                 fprintf('    ・計算上の球位置: [%.3f, %.3f, %.3f]\n', p_sphere_now(1), p_sphere_now(2), p_sphere_now(3));
%                 fprintf('    ・本当の残り距離: %.3f m (中心間距離: %.3f m)\n', geometric_margin, dist_actual);
%                 % ---------------------------------------------------------
% 
%                 % 以降の処理用へ通常データを代入
%                 A_s = A_s_normal;
%                 b_s = b_s_normal;
% 
%                 % 🌟【超重要】吐き出された数値のチェック
%                 if any(isnan(A_s)) || any(isinf(A_s)) || any(isnan(b_s)) || any(isinf(b_s))
%                     fprintf('  ⚠️ [警告] 障害物 %d, 球 %d で NaN または Inf が発生！\n', i, j);
%                 elseif norm(A_s) < 1e-12
%                     fprintf('  ⚠️ [警告] 障害物 %d, 球 %d のゲイン A_s がほぼゼロです。\n', i, j);
%                 else
%                     fprintf('  ℹ️ 障害物 %d, 球 %d: 計算時間 = %.4f秒, A_sノルム = %.2e, b_s = %.2f\n', i, j, eval_time, norm(A_s), b_s);
%                 end
% 
%                 A_qp = [A_qp; -A_s];
%                 b_qp = [b_qp;  b_s];
%             end
%         end
% 
%         fprintf('--- quadprog 直前チェック ---\n');
%         fprintf('総制約数 (A_qpの行数): %d\n', size(A_qp, 1));
% 
%         % 二次形式用の重み行列の定義
%         Q_matrix = diag([1.0, 1.0, 1.0]); 
%         H_quad   = Q_matrix;              
%         f_quad   = -Q_matrix * v_nominal; 
% 
%         disp('>> quadprog を起動します...');
%         qp_options = optimoptions('quadprog', 'Display', 'final', 'Algorithm', 'active-set');
% 
%         tic;
%         try
%             [v_safe, ~, exitflag] = quadprog(H_quad, f_quad, A_qp, b_qp, [], [], [], [], v_nominal, qp_options);
%             qp_time = toc;
%             fprintf('>> quadprog 終了: 計算時間 = %.4f秒, exitflag = %d\n', qp_time, exitflag);
%             fprintf('v_safe (安全入力): [%.4f, %.4f, %.4f]\n', v_safe(1), v_safe(2), v_safe(3));
%             fprintf('入力変化量: [%.4f, %.4f, %.4f]\n', v_safe(1)-v_nominal(1), v_safe(2)-v_nominal(2), v_safe(3)-v_nominal(3));
%         catch ME
%             fprintf('❌ quadprog 内部でエラー発生: %s\n', ME.message);
%             v_safe = v_nominal; 
%         end
%         fprintf('====== HLC DEBUG END ======\n\n');
% 
%         % -----------------------------------------------------------------
%         % 安全化された仮想操作量を、各下流レイヤーの変数へ分配 (既存部分)
%         % -----------------------------------------------------------------
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_safe(1); 
%         vs_safe(1) = v_safe(2); 
%         vs_safe(2) = v_safe(3); 
% 
%         % -----------------------------------------------------------------
%         % 下流レイヤーへの非線形相殺・実物理入力への変換 (既存部分)
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf_safe, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf_safe, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf_safe, vs_safe', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出（水平限定5次CBF・メモリ防衛版）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, t, varargin)
%         % 流れ込んできた t が TIME オブジェクトだった場合の数値抽出処理
%         if isobject(t) || isstruct(t)
%             if isprop(t, 't') || isfield(t, 't')
%                 t = t.t; % TIME.t から double の秒数を取り出す
%             elseif isprop(t, 'time') || isfield(t, 'time')
%                 t = t.time; 
%             else
%                 try t = double(t); catch; t = 0; end
%             end
%         end
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
% 
%         % 目標値を取得（元の完全な長さの配列を get() から無傷で取得）
%         if isprop(ref.state, 'xd')
%             xd_pure = ref.state.xd; 
%         else
%             xd_pure = ref.state.get();
%         end
% 
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
% 
%         % 🚨【メモリ保護の核心】：CBF関数内で「x」という名前の空間が破壊されるのを防ぐため、
%         % HLC本来の計算用変数を「x_clean」として完全に隔離・保護します。
%         x_clean = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; 
% 
%         yaw = wrapToPi(model.state.q(3)); 
%         yawd = xd_pure(4); 
%         yawUnit = [cos(yaw); sin(yaw); 0]; 
%         yawdUnit = [cos(yawd); sin(yawd); 0]; 
%         deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); 
%         xd_pure(4) = -deltaYaw(3) + yaw; 
% 
%         % 導出数式側のVars定義に合わせるためのダミー拡張処理（元データは壊さない）
%         xd_cbf_extended = xd_pure;
%         if size(xd_cbf_extended, 1) < 28
%             xd_cbf_extended = [xd_cbf_extended; zeros(28 - size(xd_cbf_extended, 1), 1)];
%         end
% 
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
% 
%         % -----------------------------------------------------------------
%         % 名目（Nominal）仮想操作量の計算（絶対に汚染されていないピュアな変数のみを使用）
%         % -----------------------------------------------------------------
%         vf_nominal = obj.Vfd_SuspendedLoadxyDst(Param.dt, x_clean, xd_pure', F1); 
%         vs_nominal = obj.Vs_SuspendedLoadxyDst(x_clean, xd_pure', vf_nominal, P, F2, F3, F4); 
%         v_nominal = [vf_nominal(1); vs_nominal(1); vs_nominal(2)]; 
% 
%         % -----------------------------------------------------------------
%         % 🔥 水平限定型 HO-CBF-QP フィルター
%         % -----------------------------------------------------------------
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obstacles = length(obs_env);  
% 
%         protection_config = [
%             0.00,               0.30;   % 1: 牽引物本体
%             0.25,               0.25;   % 2: ワイヤー下部
%             0.50,               0.25;   % 3: ワイヤー中間
%             0.75,               0.25;   % 4: ワイヤー上部
%             1.00,               0.40;   % 5: ドローン本体
%         ];
%         num_spheres = size(protection_config, 1);
% 
%         % 水平5次のスケール肥大化をいなし、手前で綺麗に避ける適正標準ゲイン
%         K_gains = [0.05, 0.05, 0.05, 0.05, 0.05];
% 
%         fprintf('\n====== HLC 水平5次DEBUG START (t = %.3f) ======\n', t);
%         fprintf('v_nominal(名目) : [%.4f, %.4f, %.4f]\n', v_nominal(1), v_nominal(2), v_nominal(3));
%         fprintf('pL(荷物現在位置): [%.4f, %.4f, %.4f]\n', pL(1), pL(2), pL(3));
% 
%         A_qp = [];
%         b_qp = [];
%         for i = 1:num_obstacles
%             ox_val = obs_env(i).p_obs(1);
%             oy_val = obs_env(i).p_obs(2);
%             oz_val = obs_env(i).p_obs(3);
%             ro_val = obs_env(i).r_obs; 
% 
%             for j = 1:num_spheres
%                 lambda_j = protection_config(j, 1);
%                 r_sph_j  = protection_config(j, 2);
% 
%                 cbf_base = [ox_val; oy_val; oz_val; ro_val; r_sph_j; lambda_j; ...
%                             K_gains(1); K_gains(2); K_gains(3); K_gains(4); K_gains(5)];
%                 cbfParam = [cbf_base; P(:)]; 
% 
%                 % シンボリック関数を実行（x_clean, xd_cbf_extendedを使用）
%                 [A_s, b_s] = CBF_Constraints_Synced_Order5([], x_clean, xd_cbf_extended', cbfParam, t);
% 
%                 p_sphere_now = pL - lambda_j * P(7) * pT; 
%                 dist_actual = sqrt((p_sphere_now(1) - ox_val)^2 + (p_sphere_now(2) - oy_val)^2); % 水平距離
%                 limit_radius = ro_val + r_sph_j;
%                 geometric_margin = dist_actual - limit_radius;
% 
%                 fprintf('  [[ 幾何チェック - 障害物%d / 球%d (lambda=%.2f) ]]\n', i, j, lambda_j);
%                 fprintf('    ・水平残り物理距離: %.3f m (中心間距離: %.3f m)\n', geometric_margin, dist_actual);
%                 fprintf('    ℹ️ CBF出力: A_sノルム = %.2e, b_s = %.2f\n', norm(A_s), b_s);
% 
%                % 数値計算（double型）なので、1ミリ秒もかからず、メモリも一切消費しません！
%                 A_s_u23 = [A_s(1), A_s(2)]; % 実入力u2, u3に対する係数 (1行2列)
%                 beta2_u23 = beta2(1:2, 1:2); % 水平方向の相殺マトリクス (2x2)
%                 alpha2_u23 = [alpha2(1); alpha2(2)]; % 水平のドリフト項 (2x1)
% 
%                 % 最上流の仮想入力 [v2_cmd; v3_cmd] に対する正しい A と b に変換
%                 A_s_vs = A_s_u23 / beta2_u23; 
%                 b_s_vs = b_s + A_s_vs * alpha2_u23;
% 
%                 % QPソルバー用の配列にパッキング
%                 A_qp = [A_qp;  A_s_vs]; 
%                 b_qp = [b_qp;  b_s_vs];
%             end
%         end
% 
%         fprintf('--- quadprog 直前チェック ---\n');
%         fprintf('総制約数 (A_qpの行数): %d\n', size(A_qp, 1));
% 
%         % 🚨【重要】：書き換え対象を「水平の2要素（X, Y）」に限定した2x2重みマトリクス
%         Q_matrix = diag([1.0, 1.0]); 
%         H_quad   = Q_matrix;              
% 
%         % ターゲットを名目仮想入力の「2番目(X), 3番目(Y)」に綺麗に絞り込み
%         v_hor_nominal = [v_nominal(2); v_nominal(3)]; 
%         f_quad   = -Q_matrix * v_hor_nominal; 
% 
%         disp('>> quadprog 水平最適化を起動します...');
%         qp_options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
% 
%         tic;
%         try
%             [v_hor_safe, ~, exitflag] = quadprog(H_quad, f_quad, A_qp, b_qp, [], [], [], [], v_hor_nominal, qp_options);
%             qp_time = toc;
%             fprintf('>> quadprog 終了: 計算時間 = %.4f秒, exitflag = %d\n', qp_time, exitflag);
% 
%             % QPソルバーが限界に達した（解なし）場合は名目水平入力をそのまま通して保険をかける
%             if exitflag <= 0, v_hor_safe = v_hor_nominal; end
%         catch ME
%             fprintf('❌ quadprog 内部エラー: %s\n', ME.message);
%             v_hor_safe = v_hor_nominal; 
%         end
% 
%         % 🚨【分配】：高度 v1 は元の完璧に飛んでいた名目入力を100%そのまま通す！
%         % 水平（X, Y）だけをQPで安全化された数値にすり替える
%         vf_safe = vf_nominal;
%         vs_safe = vs_nominal;
% 
%         vf_safe(1) = v_nominal(1);    % 高度は100%無傷（HLCの超安定フライトに完全委任）
%         vs_safe(1) = v_hor_safe(1);   % X方向仮想入力を安全値に更新
%         vs_safe(2) = v_hor_safe(2);   % Y方向仮想入力を安全値に更新
% 
%         fprintf('v_safe (安全仮想入力): [%.4f, %.4f, %.4f]\n', vf_safe(1), vs_safe(1), vs_safe(2));
%         fprintf('水平変化量 : [%.4f, %.4f]\n', vs_safe(1)-v_nominal(2), vs_safe(2)-v_nominal(3));
%         fprintf('====== HLC DEBUG END ======\n\n');
% 
%         % -----------------------------------------------------------------
%         % 下流レイヤーへの非線形相殺・実物理入力への変換 
%         % (🚨 ここでも絶対に上書き・書き換えのない純粋な x_clean と xd_pure を使用！)
%         % -----------------------------------------------------------------
%         uf = obj.Uf_SuspendedLoadxyDst(x_clean, xd_pure', vf_safe, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x_clean, xd_pure', vf_safe, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x_clean, xd_pure', vf_safe, vs_safe', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd_pure; 
%         obj.result.x = x_clean; 
%         result = obj.result;
%     end
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出（最下流実入力4軸一括・高次CBF完全同調保護・徹底デバッグ版）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
%     end
%     function result = do(obj, t, varargin)
%         % 流れ込んできた t が TIME オブジェクトだった場合の数値抽出処理
%         if isobject(t) || isstruct(t)
%             if isprop(t, 't') || isfield(t, 't')
%                 t = t.t; % TIME.t から double の秒数を取り出す
%             elseif isprop(t, 'time') || isfield(t, 'time')
%                 t = t.time; 
%             else
%                 try t = double(t); catch; t = 0; end
%             end
%         end
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
% 
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; 
%         else
%             xd = ref.state.get();
%         end
% 
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
%         % ────────────────────────────────────────────────────────────────-
%         % 🚨【重要：完全隔離ブロック】：下流の Vs_SuspendedLoadxyDst が内部で要求する
%         % 21番目の要素（5階微分）等への境界外アクセスを防ぐため、大元の xd を28次元に拡張。
%         % ────────────────────────────────────────────────────────────────-
%         xd = [xd; zeros(28 - size(xd, 1), 1)];
% 
%         % 仮想入力のゲイン
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
% 
%         % -----------------------------------------------------------------
%         % ① 【公称（Nominal）実入力の計算】 
%         % 荷物を1ミリも揺らさないためのピラミッド数式を最後まで完璧に完結させる
%         % -----------------------------------------------------------------
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         % ノーマル版と1文字の狂いもなく完全一致して出力される理想の実入力（4次元）
%         tmp_nominal = [uf(1); us]; 
% 
%         % -----------------------------------------------------------------
%         % ② 🔥 【実入力4軸一括 HO-CBF-QP 安全フィルター】の割り込み（平時完全バイパス保護版）
%         % -----------------------------------------------------------------
%         % 🚨 CBF関数に渡すため「だけ」に、元の xd を汚染しない隔離拡張ターゲットを作成
%         xd_for_cbf = xd;
% 
%         obs_env = ENVIRONMENT_OBSTACLE(); 
%         num_obstacles = length(obs_env);  
% 
%         % 牽引物、ワイヤー、ドローンをすべて包み込む5連球マルチプロテクション配置
%         protection_config = [
%             0.00,               0.30;   % 1: 牽引物ペイロード本体 (lambda=0)
%             0.25,               0.25;   % 2: ワイヤー下部
%             0.50,               0.25;   % 3: ワイヤー中間
%             0.75,               0.25;   % 4: ワイヤー上部
%             1.00,               0.40;   % 5: ドローン機体本体 (lambda=1)
%         ];
%         num_spheres = size(protection_config, 1);
% 
%         % チェーンゲイン
%         K_gains = [0.05, 0.05, 0.05, 0.05, 0.05]; % 荷物用(5次)
%         K_drone = [0.1, 0.1];                    % ドローン用(2次)
% 
%         % 🌟【安全フィルターの起動圏内マージン (m)】
%         % 障害物のフチからこの距離（例えば 2.0メートル）以内に入った球の制約だけをQPにスタックします。
%         % 平時は制約自体が完全に「空」になるため、ノーマル版と100%完全に同じ挙動を死守します。
%         cbf_activation_distance = 2.0; 
% 
%         fprintf('\n====== 🔍 HLC 最下流4軸一括安全QP 詳細DEBUG (t = %.3f) ======\n', t);
%         fprintf('    ➡️ tmp_nominal(大元理想) : [%s]\n', num2str(tmp_nominal', '%.3f '));
% 
%         A_qp = [];
%         b_qp = [];
%         for i = 1:num_obstacles
%             ox_val = obs_env(i).p_obs(1);
%             oy_val = obs_env(i).p_obs(2);
%             oz_val = obs_env(i).p_obs(3);
%             ro_val = obs_env(i).r_obs; 
% 
%             for j = 1:num_spheres
%                 lambda_j = protection_config(j, 1);
%                 r_sph_j  = protection_config(j, 2);
% 
%                 % 現在の球の残り水平幾何学マージンを計算
%                 p_sphere_now = pL - lambda_j * P(7) * pT; 
%                 dist_actual = sqrt((p_sphere_now(1) - ox_val)^2 + (p_sphere_now(2) - oy_val)^2); 
%                 geometric_margin = dist_actual - (ro_val + r_sph_j);
% 
%                 % 🌟 起動圏内チェック：本当に危ない時だけ、重い数式を呼び出してQPにスタックする
%                 if geometric_margin <= cbf_activation_distance
%                     fprintf('    ⚠️ 【緊急警告：安全フィルター起動圏内】[障害物%d / 球%d (lambda=%.2f)] 残りマージン: %.3f m\n', i, j, lambda_j, geometric_margin);
% 
%                     % ---- (A) 荷物用の5次HO-CBFの呼び出し ----
%                     cbfParam_load = [ox_val; oy_val; oz_val; ro_val; r_sph_j; lambda_j; ...
%                                      K_gains(1); K_gains(2); K_gains(3); K_gains(4); K_gains(5); P(:)]; 
%                     [A_s, b_s] = CBF_Constraints_Synced_Order5([], x, xd_for_cbf', cbfParam_load, t);
% 
%                     % ---- (B) ドローン用の2次HO-CBFの呼び出し ----
%                     cbfParam_drone = [ox_val; oy_val; oz_val; ro_val; r_sph_j; ... 
%                                       K_drone(1); K_drone(2); P(:)];
%                     [A_drone, b_drone] = CBF_Constraints_Drone([], x, xd_for_cbf', cbfParam_drone, t);
% 
%                     fprintf('      ・荷物5次CBF制約: A_s = [%s] | b_s = %.3f\n', num2str(A_s, '%.2e '), b_s);
%                     fprintf('      ・機体2次CBF制約: A_dr = [%s] | b_dr = %.3f\n', num2str(A_drone, '%.2e '), b_drone);
% 
%                     A_qp = [A_qp; A_s;     A_drone]; 
%                     b_qp = [b_qp; b_s;     b_drone];
%                 else
%                     % 平時は数式を呼ばず、ログだけを流してスルー（高速化＆墜落の完全防止）
%                     fprintf('    [平時巡航：安全スルー] [障害物%d / 球%d (lambda=%.2f)] マージン: %.3f m\n', i, j, lambda_j, geometric_margin);
%                 end
%             end
%         end
% 
%         % -----------------------------------------------------------------
%         % ③ 【4軸一括最適化パズル】 ナマの tmp_nominal からの最小距離を解く
%         % -----------------------------------------------------------------
%         H_quad = diag([1.0, 1.0, 1.0, 1.0]); 
%         f_quad = -H_quad * tmp_nominal; 
% 
%         qp_options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');
% 
%         try
%             % 制約行列が空（平時）の場合、quadprogは一切の修正をせず瞬時に tmp_nominal をそのまま返します
%             [tmp_safe, ~, exitflag] = quadprog(H_quad, f_quad, A_qp, b_qp, [], [], [], [], tmp_nominal, qp_options);
%             fprintf('    ▶️ quadprog 終了フラグ exitflag = %d (総アクティブ制約数: %d)\n', exitflag, size(A_qp,1));
%             if exitflag <= 0
%                 tmp_safe = tmp_nominal; 
%             end
%         catch ME
%             fprintf('    ❌ quadprog 内部システムエラー: %s\n', ME.message);
%             tmp_safe = tmp_nominal; 
%         end
% 
%         fprintf('    ➡️ tmp_safe   (最終決定安全値) : [%s]\n', num2str(tmp_safe', '%.3f '));
%         fprintf('====== 🔍 HLC 詳細DEBUG END ======\n\n');
% 
%         % -----------------------------------------------------------------
%         % ④ 【システムへの流し込み】
%         % -----------------------------------------------------------------
%         tmp = tmp_safe; 
% 
%         obj.result.tmp = tmp; 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
%     % クラス外部からの意図しない show 干渉によるバグを防ぐためメソッドを保護
%     function show(obj)
%         obj.result
%     end
% end
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出
% properties
%     self
%     result
%     param
% end
% 
% methods
% 
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
% 
%     end
% 
%     function result = do(obj, varargin)
%         Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
%         model = obj.self.estimator.result; % 推定した状態
%         ref = obj.self.reference.result; % 目標値
% 
%         % 大元の目標値（xd）を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; % 20次元の目標値に対応する用
%         else
%             xd = ref.state.get();
%         end
% 
%         % 荷物の現在位置の抽出（デバッグ表示用）
%         pL_now = [model.state.pL(1); model.state.pL(2); model.state.pL(3)];
% 
%         % =========================================================================
%         % 🎯 【方案①】障害物回避のための目標軌道（xd）の動的滑らか変形マトリクス
%         % =========================================================================
%         try
%             % 1. 障害物情報の定義（ENVIRONMENT_OBSTACLE と同期）
%             x_obs = 0.5;   % 障害物のX座標
%             y_obs = 7.5;   % 障害物のY座標
% 
%             % 2. 回避プロファイル（鐘型関数）のパラメータ設計
%             % 障害物が X=0.5、元の軌道が X=0 なので、左側（Xマイナス方向）へすり抜ける
%             % 安全距離が 0.85m なので、余裕を見て X = -0.6m まで膨らませる（振幅 -0.6）
%             A_avoid = -0.6; 
%             sigma   = 1.5;  % 回避行動を起こすY方向の範囲（広さ）。大きいほど手前から滑らかに避ける
% 
%             % 3. 現在の目標Y座標（xd(2)）をベースに回避量を計算
%             y_ref = xd(2); 
%             dy = y_ref - y_obs;
% 
%             % ガウス関数（鐘型プロファイル）による位置・速度・加速度の補正項の数学的導出
%             % 位置補正: Δx = A * exp( -dy^2 / (2 * sigma^2) )
%             gauss_val = exp(-dy^2 / (2 * sigma^2));
%             delta_x   = A_avoid * gauss_val;
% 
%             % 速度補正: d(Δx)/dt = d(Δx)/dy * dy/dt  (※ dy/dt は目標前進速度 xd(6))
%             v_y = xd(6); 
%             ddelta_x_dy = -A_avoid * (dy / sigma^2) * gauss_val;
%             delta_vx    = ddelta_x_dy * v_y;
% 
%             % 加速度補正: d2(Δx)/dt^2 (※簡単化のため目標等速飛行として前進加速度は0と仮定)
%             d2delta_x_dy2 = -A_avoid * ( (1 / sigma^2) - (dy^2 / sigma^4) ) * gauss_val;
%             delta_ax      = d2delta_x_dy2 * (v_y^2);
% 
%             % 4. 28次元の xd ベクトル内の該当するインデックス（位置・速度・加速度のX成分）を直接書き換え
%             % ※お使いのモデルの型（定数インデックスマップ）に同期させています
%             xd(1)  = xd(1)  + delta_x;   % 目標位置 X
%             xd(5)  = xd(5)  + delta_vx;  % 目標速度 dX
%             xd(9)  = xd(9)  + delta_ax;  % 目標加速度 ddX
% 
%             % 5. 状況を Command Window にマイルドに報告
%             if abs(delta_x) > 0.01
%                 fprintf('✨ [軌道変形中] 目標Xを修正中: %.3f m (実際の荷物位置: [%.2f, %.2f])\n', xd(1), pL_now(1), pL_now(2));
%             end
% 
%         catch ME
%             warning(['目標軌道の動的変形処理でエラーが発生しました: ', ME.message]);
%         end
%         % =========================================================================
% 
%         % 状態の再並べ替えとyaw角定義域処理（既存の正常ロジック）
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
% 
%         % 階層型線形化によるコントローラ計算（変形された安全な xd を元に、トルク・推力を逆算）
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); 
%         us = beta2 \ vs_alpha2; 
% 
%         tmp = [uf(1); us]; 
%         obj.result.tmp = tmp; 
% 
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; 
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% end


% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % クアッドコプター用階層型線形化を使った入力算出（HOCBF安全フィルター付き）
% properties
%     self
%     result
%     param
% end
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
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
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
% 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
%         us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換（理想値）
% 
%         tmp = [uf(1); us]; % 理想の実入力 [u1; u2; u3; u4]
%         obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
% 
%         %% =========================================================================
%         %% 【追加】高次制御バリア関数 (HOCBF) による安全フィルター (QP)
%         %% =========================================================================
%         % 1. 障害物パラメータの設定 [xo; yo; zo; ro]
%         % ※ シミュレーション環境に合わせて適切に書き換えてください
%         xo = 0.1; yo = 7.5; zo = 1.0; ro = 1.0; 
%         obs_params = [xo; yo; zo; ro];
% 
%         % 2. 荷物の物理半径 rl
%         % ※ 必要に応じて parameter から取得するか、固定値を設定してください
%         rl_val = 0.5; 
% 
%         % 3. HOCBFのクラスK関数ゲイン [gamma1; ...; gamma6]
%         % ※ 挙動が不安定な場合は数値を小さく(例: 0.5)、効きが甘い場合は大きく(例: 5.0)調整してください
%         % gamma_params = [2.0; 2.0; 2.0; 2.0; 2.0; 2.0];
%         gamma_params = [5.0; 5.0; 5.0; 5.0; 5.0; 5.0];
% 
%         % 4. 2nd layerの理想入力から、固定する u4 (yawトルク) を抽出
%         V4_val = tmp(4); 
% 
%         % 5. オフライン生成した関数から QP用の制約行列 A_qp, b_qp を取得
%         % ※ xd, vf の受け渡し形式は既存の Uf 等の命名規則に合わせて cell2sym を模擬した形にしています
%         [A_qp, b_qp] = CBF_Constraints_xy(obj, x, xd, vf, V4_val, obs_params, gamma_params, rl_val, P);
% 
%         % 6. QP (二次計画法) の実行
%         % 理想の roll, pitch 入力 [u2_nominal; u3_nominal]
%         u_nominal = tmp(2:3); 
% 
%         % 目的関数: (u - u_nominal)' * H * (u - u_nominal) を最小化するため、Hは単位行列
%         H_qp = eye(2);
%         f_qp = -u_nominal; % MATLABの quadprog 形式 1/2*u'*H*u + f'*u
% 
%         % トルクの物理限界制約 (必要に応じて設定、ここでは既存のmin/max制限 [-1, 1] に合わせています)
%         lb = [-1; -1];
%         ub = [ 1;  1];
% 
%         % quadprog オプション（出力を非表示にして高速化）
%         options = optimoptions('quadprog', 'Display', 'off');
% 
%         % 最適化問題を解いて安全な roll, pitch 入力を決定
%         [u_safe, ~, exitflag] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], lb, ub, [], options);
% 
%         % 万が一 QP が解けなかった（可行解なしなど）場合のセーフティ
%         if exitflag < 1
%             u_safe = u_nominal; % 最悪の場合は元の入力をそのまま使用
%         end
% 
%         % 7. QPによって修正された安全な入力を再格納
%         tmp(2:3) = u_safe;
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