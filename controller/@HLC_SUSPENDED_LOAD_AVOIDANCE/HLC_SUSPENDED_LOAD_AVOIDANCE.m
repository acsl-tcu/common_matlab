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
%         % 目標値を取得
%         if isprop(ref.state, 'xd')
%             xd = ref.state.xd; % 20次元の目標値に対応する用
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
% 
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
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; % 足りない分は0で埋める．
% 
%         % =================================================================
%         % 【新規追記】相対次数2のCBFによる目標速度・位置のサチュレーション（複数障害物一括管理版）
%         % =================================================================
%         % 1. 外部関数から障害物リストを取得
%         obs_list = ENVIRONMENT_OBSTACLE(); 
%         num_obs = length(obs_list);
% 
%         % CBFのゲイン (値が大きいほど障害物の手前で滑らかに減速を開始する)
%         % 前回の4次ECBFで失敗したような発振を防ぐため、1.0〜2.0あたりのマイルドな値から調整してください
%         alpha_cbf = 1.5; 
% 
%         % 2. 現在の荷物の位置 pL と 速度 vL (modelから取得)
%         px = pL(1); py = pL(2); pz = pL(3);
%         vx = model.state.vL(1); vy = model.state.vL(2); vz = model.state.vL(3);
% 
%         % 元々の目標値（xd）から計算される、目標移動速度（dxd1, dxd2）
%         ref_vx = xd(5); 
%         ref_vy = xd(6);
% 
%         % すべての障害物に対して順次安全チェックを実行
%         for idx = 1:num_obs
%             % 障害物のパラメータを抽出
%             ox = obs_list(idx).p_obs(1);
%             oy = obs_list(idx).p_obs(2);
%             oz = obs_list(idx).p_obs(3); % 高度成分（今回はX-Y平面での回避を想定）
%             R_safe = obs_list(idx).R_safe;
% 
%             % 3. 障害物との距離 h の計算 (位置だけで計算可能・ノイズフリー)
%             % ※高度(Z)も考慮する場合は3次元距離に、平面のみなら(pz - oz)^2 の項を消してください
%             h = (px - ox)^2 + (py - oy)^2 + (pz - oz)^2 - R_safe^2;
% 
%             % 障害物に向かうベクトル (機体から見た単位方向)
%             dir_x = px - ox;
%             dir_y = py - oy;
%             dir_z = pz - oz;
%             dist_to_obs = sqrt(dir_x^2 + dir_y^2 + dir_z^2);
% 
%             if dist_to_obs > 1e-6
%                 dir_x = dir_x / dist_to_obs;
%                 dir_y = dir_y / dist_to_obs;
%                 dir_z = dir_z / dist_to_obs;
% 
%                 % 4. 障害物方向への接近速度の許容上限値 V_max をCBF式から逆算
%                 % CBF条件: h_dot + alpha_cbf * h >= 0 
%                 % => 2 * dist_to_obs * V_approach + alpha_cbf * h >= 0
%                 max_approach_speed = (alpha_cbf * h) / (2 * dist_to_obs);
% 
%                 % 現在の目標速度の障害物方向への投影
%                 ref_v_approach = ref_vx * dir_x + ref_vy * dir_y;
% 
%                 % 5. もし目標速度がCBFの制限を破って障害物に突っ込もうとしている場合
%                 if ref_v_approach < -max_approach_speed
%                     % 障害物方向の目標速度成分を、安全な上限値（-max_approach_speed）にクランプ
%                     v_ortho_x = ref_vx - ref_v_approach * dir_x;
%                     v_ortho_y = ref_vy - ref_v_approach * dir_y;
% 
%                     % 安全に修正された目標速度（X-Y平面）
%                     ref_vx = v_ortho_x + (-max_approach_speed) * dir_x;
%                     ref_vy = v_ortho_y + (-max_approach_speed) * dir_y;
% 
%                     % 修正された速度を xd 構造体へ上書き格納
%                     xd(5) = ref_vx; % dXd1 を修正
%                     xd(6) = ref_vy; % dXd2 を修正
% 
%                     % 速度制限に合わせて、目標位置（Xd1, Xd2）も現在位置から破綻しないように修正
%                     xd(1) = px + xd(5) * Param.dt; % Xd1 を修正
%                     xd(2) = py + xd(6) * Param.dt; % Xd2 を修正
% 
%                     % 高階微分（加速度ターゲット等）の目標値も、急激な躍度変化を防ぐために安全側に落とす
%                     xd(9)  = 0; % d2Xd1
%                     xd(10) = 0; % d2Xd2
%                     xd(13) = 0; % d3Xd1
%                     xd(14) = 0; % d3Xd2
%                 end
%             end
%         end
%         % =================================================================
%         % 階層型線形化による入力計算
%         % 仮想入力のゲイン
%         F1 = Param.F1; % z方向サブシステムのゲイン
%         F2 = Param.F2; % x方向サブシステムのゲイン
%         F3 = Param.F3; % y方向サブシステムのゲイン
%         F4 = Param.F4; % yaw方向サブシステムのゲイン
% 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
%         vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
%         % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
%         %tic
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
%         vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
%         us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
%         %obj.result.aa=toc;
%         tmp = [uf(1); us]; % 実入力へ変換
%         obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
% 
%         % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
%         %disp("time: "+ num2str(t,2)+" z position of drone:
%         %"+num2str(model.state.p(3),3)+" estimated load mass:
%         %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
%         obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; %+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1; %入力にノイズを付与可能
%         % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
%         obj.result.xd = xd;
%         obj.result.x = x;
%         result = obj.result;
% 
%     end
% 
%     function show(obj)
%         obj.result
%     end
% end
% 
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % High-Level Controller for Suspended Load with Stable 4th-Order Strict ECBF-QP Filter
% 
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
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
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
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
%         %% =================================================================
%         % 階層型線形化による名目入力（Nominal Input）の計算
%         %% =================================================================
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
% 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nom = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         %% =================================================================
%         % 🛡️ 【安定化版】相対次数4の厳密ECBF-QPフィルターの適用
%         %% =================================================================
%         vs = vs_nom; 
% 
%         current_phase = "f"; 
%         try current_phase = obj.self.cha; catch, end
% 
%         if current_phase == "f"
%             obs_list = ENVIRONMENT_OBSTACLE(); 
%             num_obs = length(obs_list);
% 
%             % 4次極配置ゲイン (滑らかに回避を開始するためマイルドに設定)
%             w_c = 1.0; 
%             K1 = 4 * w_c;
%             K2 = 6 * w_c^2;
%             K3 = 4 * w_c^3;
%             K4 = w_c^4;
% 
%             % 仮想入力 vs は [vs_x; vs_y; vs_yaw] の構成。
%             % 障害物回避は X-Y 平面 (2次元成分) にフォーカスする
%             p = pL(1:2);                    
%             v = model.state.vL(1:2);        
% 
%             try
%                 a = model.state.aL(1:2);    
%                 j_dot = model.state.jL(1:2);
%             catch
%                 a = zeros(2,1);
%                 j_dot = zeros(2,1);
%             end
% 
%             A_qp = [];
%             b_qp = [];
% 
%             vs_nom_col = vs_nom(:); 
%             n_inputs = length(vs_nom_col);
% 
%             for idx = 1:num_obs
%                 ox = obs_list(idx).p_obs(1);
%                 oy = obs_list(idx).p_obs(2);
%                 p_obs = [ox; oy];
%                 R_safe = obs_list(idx).R_safe;
% 
%                 % X-Y平面での相対位置
%                 dp = p - p_obs;
% 
%                 % 2次元平面上での各階微分計算
%                 h = dp' * dp - R_safe^2;
%                 h_dot = 2 * dp' * v;
%                 h_2dot = 2 * (v' * v + dp' * a);
%                 h_3dot = 2 * (3 * v' * a + dp' * j_dot);
% 
%                 B_cbf = 2 * (3 * (a' * a) + 4 * (v' * j_dot)); 
% 
%                 % 仮想入力 vs の1番目がX、2番目がYを駆動するため、結合係数をマップ
%                 A_row = zeros(1, n_inputs);
%                 A_row(1) = -2 * dp(1); 
%                 A_row(2) = -2 * dp(2); 
% 
%                 % 余裕を持たせるためのマージン項を追加
%                 b_row = B_cbf + K1*h_3dot + K2*h_2dot + K3*h_dot + K4*h;
% 
%                 A_qp = [A_qp; A_row];
%                 b_qp = [b_qp; b_row];
%             end
% 
%             if ~isempty(A_qp)
%                 options = optimoptions('quadprog', 'Display', 'none');
% 
%                 % 急激な変化を防ぐため、H_qpの重みを調整
%                 H_qp = eye(n_inputs);
%                 f_qp = -vs_nom_col;
% 
%                 % 仮想入力が爆発して特異点に入らないよう、補正量に上限(バウンド)を設ける
%                 lb = vs_nom_col - [5.0; 5.0; 1.0];
%                 ub = vs_nom_col + [5.0; 5.0; 1.0];
% 
%                 [vs_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], lb, ub, [], options);
% 
%                 if exitflag == 1
%                     vs = reshape(vs_opt, size(vs_nom)); 
%                 else
%                     % QPが過渡的に解けない、または限界を超えそうな場合は急激に動かさずマイルドに追従
%                     vs = vs_nom * 0.5; 
%                 end
%             end
%         end
%         %% =================================================================
% 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         % 特異行列になるのを防ぐための頑健化処理 (万が一beta2の行列式が0に近づいたら正則化)
%         if rcond(beta2) < 1e-6
%             beta2 = beta2 + 1e-4 * eye(size(beta2));
%         end
% 
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
% end

% classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% % High-Level Controller for Suspended Load with High-Performance ECBF-QP Filter
% 
% properties
%     self
%     result
%     param
% end
% 
% methods
%     function obj = HLC_SUSPENDED_LOAD_AVOIDANCE(self, param)
%         obj.self = self;
%         obj.param = param;
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
%         xd = [xd; zeros(28 - size(xd, 1), 1)]; 
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
%         %% =================================================================
%         % 階層型線形化による名目入力（Nominal Input）の計算
%         %% =================================================================
%         F1 = Param.F1; 
%         F2 = Param.F2; 
%         F3 = Param.F3; 
%         F4 = Param.F4; 
% 
%         vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); 
%         vs_nom = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); 
% 
%         %% =================================================================
%         % 🛡️ 【高性能・スラック付き版】相対次数4の厳密ECBF-QPフィルター
%         %% =================================================================
%         vs = vs_nom; 
% 
%         current_phase = "f"; 
%         try current_phase = obj.self.cha; catch, end
% 
%         if current_phase == "f"
%             obs_list = ENVIRONMENT_OBSTACLE(); 
%             num_obs = length(obs_list);
% 
%             % 4次極配置ゲイン (回避をキビキビさせるため 1.0 -> 1.45 に引き上げ)
%             w_c = 1.45; 
%             K1 = 4 * w_c;
%             K2 = 6 * w_c^2;
%             K3 = 4 * w_c^3;
%             K4 = w_c^4;
% 
%             % 吊り荷のX-Y位置・速度・加速度を取得
%             p = pL(1:2);                    
%             v = model.state.vL(1:2);        
% 
%             try
%                 a = model.state.aL(1:2);    
%                 j_dot = model.state.jL(1:2);
%             catch
%                 a = zeros(2,1);
%                 j_dot = zeros(2,1);
%             end
% 
%             A_qp_raw = [];
%             b_qp_raw = [];
% 
%             vs_nom_col = vs_nom(:); 
%             n_inputs = length(vs_nom_col);
% 
%             for idx = 1:num_obs
%                 ox = obs_list(idx).p_obs(1);
%                 oy = obs_list(idx).p_obs(2);
%                 p_obs = [ox; oy];
%                 R_safe = obs_list(idx).R_safe;
% 
%                 dp = p - p_obs;
% 
%                 % 高階CBFの計算
%                 h = dp' * dp - R_safe^2;
%                 h_dot = 2 * dp' * v;
%                 h_2dot = 2 * (v' * v + dp' * a);
%                 h_3dot = 2 * (3 * v' * a + dp' * j_dot);
% 
%                 B_cbf = 2 * (3 * (a' * a) + 4 * (v' * j_dot)); 
% 
%                 % 仮想入力への結合
%                 A_row = zeros(1, n_inputs);
%                 A_row(1) = -2 * dp(1); 
%                 A_row(2) = -2 * dp(2); 
% 
%                 b_row = B_cbf + K1*h_3dot + K2*h_2dot + K3*h_dot + K4*h;
% 
%                 A_qp_raw = [A_qp_raw; A_row];
%                 b_qp_raw = [b_qp_raw; b_row];
%             end
% 
%             if ~isempty(A_qp_raw)
%                 options = optimoptions('quadprog', 'Display', 'none');
% 
%                 % 【スラック変数の導入】 
%                 % 最適化変数を [vsの補正量(n_inputs次元); スラック変数(num_obs次元)] に拡張します。
%                 % これにより「姿勢崩壊限界」と「回避性能」を高度に両立します。
%                 n_vars = n_inputs + num_obs;
% 
%                 % コスト関数 H_qp の設定
%                 % vsの補正には eye(n_inputs) を、スラック変数には大きなペナルティ(1e5)をかける
%                 H_qp = zeros(n_vars, n_vars);
%                 H_qp(1:n_inputs, 1:n_inputs) = eye(n_inputs);
%                 H_qp(n_inputs+1:end, n_inputs+1:end) = 1e5 * eye(num_obs); 
% 
%                 f_qp = [-vs_nom_col; zeros(num_obs, 1)];
% 
%                 % 制約マトリクスの拡張 (A_row * vs <= b_row + slack) -> (A_row * vs - slack <= b_row)
%                 A_qp = [A_qp_raw, -eye(num_obs)];
%                 b_qp = b_qp_raw;
% 
%                 % 入力の上下限を前回より大幅に緩和 (しっかり動けるようにする)
%                 % vsに対する制限
%                 lb_vs = vs_nom_col - [40.0; 40.0; 5.0];
%                 ub_vs = vs_nom_col + [40.0; 40.0; 5.0];
%                 % スラック変数は 0 以上 (制約を緩める方向のみ)
%                 lb_slack = zeros(num_obs, 1);
%                 ub_slack = inf(num_obs, 1);
% 
%                 lb = [lb_vs; lb_slack];
%                 ub = [ub_vs; ub_slack];
% 
%                 [sol_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], lb, ub, [], options);
% 
%                 if exitflag == 1
%                     % 最適化結果から vs の成分だけを抽出
%                     vs_opt = sol_opt(1:n_inputs);
%                     vs = reshape(vs_opt, size(vs_nom)); 
%                 else
%                     % 万が一解けなかった場合はマイルドな名目入力
%                     vs = vs_nom * 0.8; 
%                 end
%             end
%         end
%         %% =================================================================
% 
%         uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); 
%         beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); 
% 
%         if rcond(beta2) < 1e-6
%             beta2 = beta2 + 1e-4 * eye(size(beta2));
%         end
% 
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
% end

classdef HLC_SUSPENDED_LOAD_AVOIDANCE < handle
% クアッドコプター用階層型線形化を使った入力算出
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
        t = varargin{1};
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
        xd = [xd; zeros(28 - size(xd, 1), 1)]; % 足りない分は0で埋める．

        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン

        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
        % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
        %tic
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
        us_nominal = beta2 \ vs_alpha2; % 既存の優秀な名目実入力（roll,pitch,yawのトルク）

        %% =========================================================================
        %% HO-CBF (高次制御バリア関数) 安全フィルターの統合セクション
        %% =========================================================================
        
        % 1. 障害物の現在の環境情報を一元管理関数から取得
        obs_env = ENVIRONMENT_OBSTACLE(); 

        % 🚨【超重要デバッグ】環境関数が本当にオブジェクトを返しているかチェック
        fprintf('\n--- [DEBUG] obs_env 状況確認 ---\n');
        fprintf('取得した障害物の個数: %d\n', length(obs_env));
        if ~isempty(obs_env)
            fprintf('障害物(1)の中心座標: [%.2f, %.2f, %.2f]\n', obs_env(1).p_obs(1), obs_env(1).p_obs(2), obs_env(1).p_obs(3));
            fprintf('障害物(1)の安全半径 R_safe: %.2f\n', obs_env(1).R_safe);
        end
        
        % QP用制約行列の初期化 (A * u >= b の形)
        A_qp = [];
        b_qp = [];
        
        % CBF用の調整ハイパーパラメータの定義
        r_load_val  = 0.2;  % 荷物の保護球半径
        r_drone_val = 0.4;  % ドローン本体の保護球半径
        r_string_val = 0.15; % 紐の保護球半径 (球を増やす場合に使用)
        
        % 収束ゲインの設定
        alpha_g = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]; % 荷物・紐用(6次)ゲイン \alpha_1 ~ \alpha_6
        beta_g  = [2.0, 2.0, 2.0, 2.0];           % ドローン用(4次)ゲイン \beta_1 ~ \beta_4
        
        % 2. アクティブなすべての障害物に対してループを回して制約を累積
        for i = 1:length(obs_env)
            ox_val = obs_env(i).p_obs(1);
            oy_val = obs_env(i).p_obs(2);
            oz_val = obs_env(i).p_obs(3);
            ro_val = obs_env(i).R_safe; % すでに包囲半径+安全マージンが計算済みの値
            
            %% (A) 荷物の6次CBF制約の計算と蓄積
            cbfParam_load = [ox_val; oy_val; oz_val; ro_val; r_load_val; alpha_g'; P(:)];
            % 【修正】第4引数を vs' から us_nominal' に変更！
            [A_load, b_load] = CBF_Constraints_Load(obj, x, xd', us_nominal', cbfParam_load, t);
            A_qp = [A_qp; A_load];
            b_qp = [b_qp; b_load];
            
            %% (B) 紐の上の球の追加（3つ、4つに拡張する場合の処理）
            % 将来的に紐を追加する場合は、ここに上の「lambda_listループ」で計算した
            % CBF_Constraints_String を同様に A_qp, b_qp に [A_qp; A_str] で累積します。
            
            %% (C) ドローン本体の4次CBF制約の計算と蓄積
            cbfParam_drone = [ox_val; oy_val; oz_val; ro_val; r_drone_val; beta_g'; P(:)];
            % 【修正】第4引数を vs' から us_nominal' に変更！
            [A_drone, b_drone] = CBF_Constraints_Drone(obj, x, xd', us_nominal', cbfParam_drone, t);
            A_qp = [A_qp; A_drone];
            b_qp = [b_qp; b_drone];
        end
        
        % 3. QP（二次計画法）による安全トルクの最適化計算
        % 目的関数: min_u (1/2)*|| u - us_nominal ||^2  -->  H = I(単位行列), f_vec = -us_nominal
        H_mat = eye(3);
        f_vec = -us_nominal;
        
        % Yaw軸のシワ寄せ暴走を完全に防止する上下限制限 (上下限ハードリミット)
        % Roll/Pitchは機体の限界トルク、Yawは暴れないように狭めに設定
        lb = [-1.5; -1.5; -0.3]; 
        ub = [ 1.5;  1.5;  0.3];
        
        % quadprogの実行オプション（リアルタイム制御のため出力を非表示にして高速化）
        options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'active-set');

        % 🚨【毎ステップ強制出力①】安全に時間オブジェクトを処理して制約行列を表示
        try
            t_str = char(string(t)); % TIMEオブジェクトを強制的に文字列へ変換
        catch
            t_str = 'Unknown';
        end
        
        fprintf('\n====== HO-CBF STEP DEBUG (t = %s) ======\n', t_str);
        fprintf('A_qp 行列の内訳 (行数: %d):\n', size(A_qp, 1));
        disp(double(A_qp)); % 念のためdoubleキャストして表示
        fprintf('b_qp ベクトルの内訳:\n');
        disp(double(b_qp));
        
        try
            % すべての球の安全を同時に満たす Roll, Pitch, Yaw トルクを計算
            [us_safe, ~, exitflag] = quadprog(H_mat, f_vec, -A_qp, -b_qp, [], [], lb, ub, [], options);
            % 🚨【毎ステップ強制出力②】QPソルバーの計算結果チェック
            fprintf('QP Exit Flag: %d\n', exitflag);
            fprintf('名目トルク us_nominal: [%.4f, %.4f, %.4f]\n', us_nominal(1), us_nominal(2), us_nominal(3));
            if exitflag == 1
                us = us_safe; % 安全なトルクが見つかればそれを実入力として採用
                fprintf('安全トルク us_safe   : [%.4f, %.4f, %.4f]\n', us(1), us(2), us(3));
                fprintf('トルク修正量 (差分)  : [%.4f, %.4f, %.4f]\n', us(1)-us_nominal(1), us(2)-us_nominal(2), us(3)-us_nominal(3));
            else
                % 万が一、最適解が極限状態で厳密に見つからなかった場合のセーフティ
                warning('QP安全フィルターが極限状態で厳密解を逃しました。名目値を出力します。');
                us = us_nominal;
            end
            fprintf('===========================================\n');

        
        catch ME
            fprintf('🚨 QPソルバー実行エラー: %s\n', ME.message);
            us = us_nominal; % ソルバーエラー時のフォールバック
        end
        
        %% =========================================================================

        %obj.result.aa=toc;
        tmp = [uf(1); us];
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納

        % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
        %disp("time: "+ num2str(t,2)+" z position of drone:
        %"+num2str(model.state.p(3),3)+" estimated load mass:
        %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
        obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))]; %+[normrnd(0,0.01,1);normrnd(0,0.001,[3,1])]*1; %入力にノイズを付与可能
        % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;

    end

    function show(obj)
        obj.result
    end
end

end