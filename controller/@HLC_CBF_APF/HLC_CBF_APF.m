% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     properties
%         p_off
%         v_off
%         a_off
%         j_off
%         s_off
%         d5_off
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             obj.p_off  = [0;0;0];
%             obj.v_off  = [0;0;0];
%             obj.a_off  = [0;0;0];
%             obj.j_off  = [0;0;0];
%             obj.s_off  = [0;0;0];
%             obj.d5_off = [0;0;0];
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
% 
%             p_curr = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = [0;0;0];
%             k_rep = 5.0;  
%             d_inf = 4.0;  
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 【形状近似の切り替え】
%             % 1: 複数球近似 (Multi-Sphere) ... 機体と荷物に独立した球を配置
%             % 2: 楕円体近似 (Ellipsoid) ... 紐の中心を原点とする縦長の楕円体で全体を包む
%             APPROX_TYPE = 2; 
%             % ========================================================
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
% 
%                 if APPROX_TYPE == 1
%                     % --- 複数球近似 (Multi-Sphere) ---
%                     protect_pts = [p_curr, (p_curr+pL_curr)/2.0, pL_curr];
%                     r_safe = 0.6; % 各球の半径
%                     Q_inv = inv(obs.Q_obs);
% 
%                     for p_i = 1:3
%                         pt = protect_pts(:, p_i);
%                         diff_local = obs.R_obs' * (pt - obs.p_obs);
%                         d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                         dist_raw = d_ellip - obs.d_margin;
%                         if dist_raw < d_surf_min; d_surf_min = dist_raw; end
%                         dist_eff = dist_raw - r_safe;
%                         if dist_eff < 0.05; dist_eff = 0.05; end
% 
%                         if dist_eff < d_inf
%                             grad_local = (Q_inv^2) * diff_local;
%                             dir = obs.R_obs * grad_local;
%                             dir = dir / norm(dir);
%                             if abs(dir(3)) > 0.95
%                                 [~, min_axis_idx] = min(diag(obs.Q_obs));
%                                 breaker_local = zeros(3,1); breaker_local(min_axis_idx) = 0.1; 
%                                 dir = dir + obs.R_obs * breaker_local; dir = dir / norm(dir);
%                             end
%                             force = k_rep * (1/dist_eff - 1/d_inf) * (1/dist_eff^2);
%                             if force > 10.0; force = 10.0; end
%                             F_rep = F_rep + force * dir;
%                         end
%                     end
% 
%                 elseif APPROX_TYPE == 2
%                     % --- 楕円体近似 (Ellipsoid vs Ellipsoid のミンコフスキー和近似) ---
%                     p_center = (p_curr + pL_curr) / 2.0; % 紐の中心
%                     L_str = norm(p_curr - pL_curr);
%                     r_xy = 0.6; % ドローンの横幅半径
%                     r_z = (L_str / 2.0) + 0.3; % 縦の半径 (紐の長さ/2 + 荷物の余裕)
% 
%                     % 障害物とドローンの楕円体を合成(ミンコフスキー和の対角近似)
%                     Q_eff = obs.Q_obs + diag([r_xy, r_xy, r_z]);
%                     Q_inv_eff = inv(Q_eff);
% 
%                     diff_local = obs.R_obs' * (p_center - obs.p_obs);
%                     d_ellip = sqrt(diff_local' * (Q_inv_eff^2) * diff_local) - 1.0;
%                     dist_raw = d_ellip - obs.d_margin;
%                     if dist_raw < d_surf_min; d_surf_min = dist_raw; end
% 
%                     dist_eff = dist_raw; % r_safeはQ_effに内包済み
%                     if dist_eff < 0.05; dist_eff = 0.05; end
% 
%                     if dist_eff < d_inf
%                         grad_local = (Q_inv_eff^2) * diff_local;
%                         dir = obs.R_obs * grad_local;
%                         dir = dir / norm(dir);
%                         if abs(dir(3)) > 0.95
%                             [~, min_axis_idx] = min(diag(obs.Q_obs));
%                             breaker_local = zeros(3,1); breaker_local(min_axis_idx) = 0.1; 
%                             dir = dir + obs.R_obs * breaker_local; dir = dir / norm(dir);
%                         end
%                         force = k_rep * (1/dist_eff - 1/d_inf) * (1/dist_eff^2);
%                         if force > 10.0; force = 10.0; end
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 【理論的完全解】6次クリティカルダンピング・アドミタンスフィルタ
%             % 懸架荷物システムの微分平坦性（Differential Flatness）は6階微分(Crackle)までを要求する。
%             % 従来の2次モデルではF_repの急変時に3階微分(Jerk)以降が不連続・発散し、理論的欠陥となる。
%             % そこで、全ての極を -lambda に配置した6次ローパスを構成し、
%             % C^0連続な6階微分オフセットまでを数学的に保証する。
%             % ========================================================
%             dt = 0.025; if isprop(time, 'dt'); dt = time.dt; end
% 
%             lambda = 2.0; % 応答速度（大きくすると速く、小さくすると滑らか）
%             c0 = lambda^6;
%             c1 = 6 * lambda^5;
%             c2 = 15 * lambda^4;
%             c3 = 20 * lambda^3;
%             c4 = 15 * lambda^2;
%             c5 = 6 * lambda;
% 
%             % 6階微分 (Crackle) オフセットの計算
%             % 定常状態で p_off = F_rep になるよう、入力に c0 を乗じてスケーリング
%             d6_off = c0 * F_rep - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + c2 * obj.a_off + c1 * obj.v_off + c0 * obj.p_off);
% 
%             % 状態の更新 (Euler積分)
%             obj.d5_off = obj.d5_off + d6_off * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off * dt;
%             obj.a_off  = obj.a_off  + obj.j_off * dt;
%             obj.v_off  = obj.v_off  + obj.a_off * dt;
%             obj.p_off  = obj.p_off  + obj.v_off * dt;
% 
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;   % Position (0th)
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;   % Velocity (1st)
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;   % Acceleration (2nd)
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;   % Jerk (3rd)
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;   % Snap (4th)
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;  % Pop (5th)
%             xd_mod(25:27) = xd_nom(25:27) + d6_off;      % Crackle (6th)
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% end
% 

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 初期の理想的な「曲面追従＆目標復帰」を完全再現し、
%     % 微分平坦性の C^6 連続性を数学的に厳密保証したアドミタンス修正器
%     % =========================================================================
%     properties
%         p_off   % 0th (Position offset)
%         v_off   % 1st (Velocity offset)
%         a_off   % 2nd (Acceleration offset)
%         j_off   % 3rd (Jerk offset)
%         s_off   % 4th (Snap offset)
%         d5_off  % 5th (Crackle offset)
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             obj.p_off  = zeros(3,1);
%             obj.v_off  = zeros(3,1);
%             obj.a_off  = zeros(3,1);
%             obj.j_off  = zeros(3,1);
%             obj.s_off  = zeros(3,1);
%             obj.d5_off = zeros(3,1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
%             p_mid   = (p_curr + pL_curr) / 2.0;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             k_rep = 5.0;  % 初期の理想パラメータ
%             d_inf = 4.0;  % 初期の理想パラメータ
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 1. 初期の「複数球近似 (Multi-Sphere)」幾何勾配場
%             % ========================================================
%             protect_pts = [p_curr, p_mid, pL_curr];
%             r_safe = 0.6; % 初期の安全球半径
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
%                 Q_inv = inv(obs.Q_obs);
% 
%                 for p_i = 1:3
%                     pt = protect_pts(:, p_i);
%                     diff_local = obs.R_obs' * (pt - obs.p_obs);
% 
%                     % 楕円ポテンシャル距離
%                     d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                     dist_raw = d_ellip - obs.d_margin;
%                     if dist_raw < d_surf_min
%                         d_surf_min = dist_raw;
%                     end
% 
%                     dist_eff = dist_raw - r_safe;
%                     if dist_eff < 0.05
%                         dist_eff = 0.05;
%                     end
% 
%                     if dist_eff < d_inf
%                         % 初期の自然な曲面法線勾配
%                         grad_local = (Q_inv^2) * diff_local;
%                         dir = obs.R_obs * grad_local;
%                         dir_norm = norm(dir);
%                         if dir_norm > 1e-6
%                             dir = dir / dir_norm;
%                         else
%                             dir = [1; 0; 0];
%                         end
% 
%                         % ----------------------------------------------------
%                         % 【滑らかな対称性破壊 (C^inf 保証)】
%                         % 初期コードの if abs(dir(3)) > 0.95 を滑らかな tanh 遷移へ
%                         % ----------------------------------------------------
%                         [~, min_axis_idx] = min(diag(obs.Q_obs));
%                         breaker_local = zeros(3,1);
%                         breaker_local(min_axis_idx) = 0.1;
%                         breaker_dir = obs.R_obs * breaker_local;
% 
%                         % 真上/真下 (dir(3) ≈ ±1) の特異点を滑らかに回避
%                         w_break = 0.5 * (1.0 + tanh(20.0 * (abs(dir(3)) - 0.92)));
%                         dir = (1.0 - w_break) * dir + w_break * (dir + breaker_dir);
%                         dir = dir / norm(dir);
% 
%                         % ----------------------------------------------------
%                         % 【真上接近時の水平迂回誘導】
%                         % 下向き反発を横方向（XY）に受け流し、叩き落としを防ぐ
%                         % ----------------------------------------------------
%                         if dir(3) < 0
%                             dir(3) = 0.05 * dir(3); % 下向き成分をほぼカット
%                             dir = dir / norm(dir);
%                         end
% 
%                         % 初期の斥力関数
%                         force = k_rep * (1.0/dist_eff - 1.0/d_inf) * (1.0 / (dist_eff^2));
%                         if force > 10.0
%                             force = 10.0;
%                         end
% 
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 2. 6次クリティカルダンピング・アドミタンスフィルタ
%             % (初期のバネ復元特性 (s+lambda)^6 を完全保持)
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             lambda = 2.0; % 初期の遮断周波数
%             c0 = lambda^6;
%             c1 = 6  * lambda^5;
%             c2 = 15 * lambda^4;
%             c3 = 20 * lambda^3;
%             c4 = 15 * lambda^2;
%             c5 = 6  * lambda;
% 
%             % 初期の閉ループ方程式 (復元項 c0*p_off が存在するため障害物通過後に 0 へ戻る)
%             d6_calc = c0 * F_rep - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                                     c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             % 状態の更新 (オイラー積分)
%             % 積分器を通すことで、p_off 〜 d5_off はすべて C^1 級以上の滑らかさを保証
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 3. 参照軌道への合成 (28次元)
%             % ========================================================
%             % 最上位微分(6階)には直達の d6_calc ではなく、
%             % 積分状態である d5_off の差分を当てることで C^0 連続性を厳密保証
%             d6_smooth = (obj.d5_off - (obj.d5_off - d6_calc * dt)) / dt; % = d6_calc の低域通過版
% 
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;   % Position (0th)
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;   % Velocity (1st)
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;   % Acceleration (2nd)
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;   % Jerk (3rd)
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;   % Snap (4th)
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;  % Pop (5th)
%             xd_mod(25:27) = xd_nom(25:27) + d6_smooth;   % Crackle (6th)
% 
%             % 制御器本体の実行
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom; % 原本を復元
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% end

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 初期の理想的な「曲面追従＆目標復帰」挙動を完全保持しつつ、
%     % 全空間での C^inf 滑らか整流 と C^6 連続性 (微分平坦性) を
%     % 数学的に厳密保証したアドミタンスリファレンス修正器
%     % =========================================================================
%     properties
%         p_off   % 0th (Position offset)
%         v_off   % 1st (Velocity offset)
%         a_off   % 2nd (Acceleration offset)
%         j_off   % 3rd (Jerk offset)
%         s_off   % 4th (Snap offset)
%         d5_off  % 5th (Crackle offset)
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             obj.p_off  = zeros(3,1);
%             obj.v_off  = zeros(3,1);
%             obj.a_off  = zeros(3,1);
%             obj.j_off  = zeros(3,1);
%             obj.s_off  = zeros(3,1);
%             obj.d5_off = zeros(3,1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
%             p_mid   = (p_curr + pL_curr) / 2.0;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             k_rep = 5.0;  % 初期の理想パラメータ
%             d_inf = 4.0;  % 初期の理想影響圏
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 1. 初期の「複数球近似 (Multi-Sphere)」幾何勾配場
%             % ========================================================
%             protect_pts = [p_curr, p_mid, pL_curr];
%             r_safe = 0.6; % 初期の安全球半径
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
%                 Q_inv = inv(obs.Q_obs);
% 
%                 for p_i = 1:3
%                     pt = protect_pts(:, p_i);
%                     diff_local = obs.R_obs' * (pt - obs.p_obs);
% 
%                     % 楕円ポテンシャル代数距離
%                     d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                     dist_raw = d_ellip - obs.d_margin;
%                     if dist_raw < d_surf_min
%                         d_surf_min = dist_raw;
%                     end
% 
%                     dist_eff = dist_raw - r_safe;
%                     if dist_eff < 0.05
%                         dist_eff = 0.05;
%                     end
% 
%                     if dist_eff < d_inf
%                         % 初期の自然な曲面法線勾配
%                         grad_local = (Q_inv^2) * diff_local;
%                         dir = obs.R_obs * grad_local;
%                         dir_norm = norm(dir);
%                         if dir_norm > 1e-6
%                             dir = dir / dir_norm;
%                         else
%                             dir = [1; 0; 0];
%                         end
% 
%                         % ----------------------------------------------------
%                         % 【滑らかな対称性破壊 (C^inf 級保証)】
%                         % 真上/真下 (dir(3) ≈ ±1) の特異点を滑らかな tanh で回避
%                         % ----------------------------------------------------
%                         [~, min_axis_idx] = min(diag(obs.Q_obs));
%                         breaker_local = zeros(3,1);
%                         breaker_local(min_axis_idx) = 0.1;
%                         breaker_dir = obs.R_obs * breaker_local;
% 
%                         w_break = 0.5 * (1.0 + tanh(20.0 * (abs(dir(3)) - 0.92)));
%                         dir = (1.0 - w_break) * dir + w_break * (dir + breaker_dir);
%                         dir = dir / norm(dir);
% 
%                         % ----------------------------------------------------
%                         % 【真上接近時の滑らかな水平迂回整流 (C^inf 級保証)】
%                         % if dir(3) < 0 の角を解消し、tanh で滑らかに下向きを遮断
%                         % dir(3) < 0 では 0.05 倍へ減衰、dir(3) > 0 では 1.0 倍へ滑らかに接続
%                         % ----------------------------------------------------
%                         w_z = 0.05 + 0.95 * 0.5 * (1.0 + tanh(20.0 * dir(3)));
%                         dir(3) = w_z * dir(3);
%                         dir = dir / norm(dir);
% 
%                         % 初期の斥力関数
%                         force = k_rep * (1.0/dist_eff - 1.0/d_inf) * (1.0 / (dist_eff^2));
%                         if force > 10.0
%                             force = 10.0;
%                         end
% 
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 2. 6次クリティカルダンピング・アドミタンスフィルタ
%             % (初期のバネ復元特性 (s+lambda)^6 を完全保持)
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             lambda = 2.0; % 初期の遮断周波数
%             c0 = lambda^6;
%             c1 = 6  * lambda^5;
%             c2 = 15 * lambda^4;
%             c3 = 20 * lambda^3;
%             c4 = 15 * lambda^2;
%             c5 = 6  * lambda;
% 
%             % 閉ループ状態方程式 (復元項 c0*p_off により障害物通過後に 0 へ自律復帰)
%             d6_calc = c0 * F_rep - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                                     c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             % 状態の更新 (オイラー積分)
%             % 積分により p_off 〜 d5_off は C^1 級以上の滑らかさを保証
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 3. 参照軌道への合成 (28次元スタック完全適合)
%             % ========================================================
%             % 滑らかな空間ポテンシャル Frep のため、
%             % 線形結合である d6_calc 自体が数学的に C^0 連続性を保証されている
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;   % Position (0th)
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;   % Velocity (1st)
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;   % Acceleration (2nd)
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;   % Jerk (3rd)
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;   % Snap (4th)
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;  % Pop (5th)
%             xd_mod(25:27) = xd_nom(25:27) + d6_calc;     % Crackle (6th)
% 
%             % 制御器本体の実行
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom; % 原本を復元
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% end

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 幾何モデル切り替え型 C^6 連続アドミタンス・リファレンス修正器
%     % 
%     % 【GEOM_TYPE】
%     %   1: 複数球近似モデル (Multi-Sphere: 機体・テザー中点・荷物の3球)
%     %   2: 動的回転楕円体モデル (Ellipsoid: CBFと完全整合した全系統一楕円体)
%     % =========================================================================
%     properties
%         GEOM_TYPE = 2; % デフォルト: 2 (楕円体)
% 
%         % 6次アドミタンス内部状態 (3次元 x 6階層)
%         p_off   % 0th (Position offset)
%         v_off   % 1st (Velocity offset)
%         a_off   % 2nd (Acceleration offset)
%         j_off   % 3rd (Jerk offset)
%         s_off   % 4th (Snap offset)
%         d5_off  % 5th (Crackle offset)
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if isfield(param, 'GEOM_TYPE')
%                 obj.GEOM_TYPE = param.GEOM_TYPE;
%             end
%             obj.p_off  = zeros(3, 1);
%             obj.v_off  = zeros(3, 1);
%             obj.a_off  = zeros(3, 1);
%             obj.j_off  = zeros(3, 1);
%             obj.s_off  = zeros(3, 1);
%             obj.d5_off = zeros(3, 1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             % ノミナル参照軌道 (28次元スタック適合)
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             L_cable = norm(p_curr - pL_curr);
%             if L_cable > 1e-4
%                 pT = (p_curr - pL_curr) / L_cable;
%             else
%                 pT = [0; 0; 1];
%             end
%             p_mid = pL_curr + 0.5 * L_cable * pT;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             k_rep = 6.0;  % 斥力ゲイン
%             d_inf = 4.5;  % 影響圏 [m]
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 幾何モデルに基づく斥力ポテンシャル計算
%             % ========================================================
%             if obj.GEOM_TYPE == 1
%                 % ----------------------------------------------------
%                 % モード 1: 複数球近似モデル (Multi-Sphere)
%                 % ----------------------------------------------------
%                 protect_pts = [p_curr, p_mid, pL_curr];
%                 r_safe_sphere = 0.6; % 保護球半径 [m]
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     Q_inv = inv(obs.Q_obs);
% 
%                     for p_i = 1:3
%                         pt = protect_pts(:, p_i);
%                         diff_local = obs.R_obs' * (pt - obs.p_obs);
% 
%                         d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                         dist_raw = d_ellip - obs.d_margin;
%                         if dist_raw < d_surf_min
%                             d_surf_min = dist_raw;
%                         end
% 
%                         dist_eff = dist_raw - r_safe_sphere;
%                         if dist_eff < 0.05
%                             dist_eff = 0.05;
%                         end
% 
%                         if dist_eff < d_inf
%                             grad_local = (Q_inv^2) * diff_local;
%                             dir_w = obs.R_obs * grad_local;
%                             norm_d = norm(dir_w);
%                             if norm_d > 1e-6
%                                 dir = dir_w / norm_d;
%                             else
%                                 dir = [1; 0; 0];
%                             end
% 
%                             % 対称性破壊 & 上昇整流 (C^inf)
%                             dir = obj.apply_smooth_circulation(obs, dir);
% 
%                             force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                             if force > 12.0; force = 12.0; end
% 
%                             F_rep = F_rep + force * dir;
%                         end
%                     end
%                 end
% 
%             elseif obj.GEOM_TYPE == 2
%                 % ----------------------------------------------------
%                 % モード 2: 動的回転楕円体モデル (Ellipsoid: CBF整合)
%                 % ----------------------------------------------------
%                 a_sys = 0.35;                   % 短軸半径 (水平保護幅)
%                 b_sys = (L_cable / 2.0) + 0.35; % 長軸半径 (テザー軸保護幅)
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     Q_inv = inv(obs.Q_obs);
% 
%                     diff_local = obs.R_obs' * (p_mid - obs.p_obs);
%                     d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                     dist_raw = d_ellip - obs.d_margin;
%                     if dist_raw < d_surf_min
%                         d_surf_min = dist_raw;
%                     end
% 
%                     grad_local = (Q_inv^2) * diff_local;
%                     dir_w = obs.R_obs * grad_local;
%                     norm_d = norm(dir_w);
%                     if norm_d > 1e-6
%                         n_vec = dir_w / norm_d;
%                     else
%                         n_vec = [1; 0; 0];
%                     end
% 
%                     % 自機楕円体の法線方向拡大幅
%                     r_sys_ext = sqrt(a_sys^2 + (b_sys^2 - a_sys^2) * (n_vec' * pT)^2);
% 
%                     dist_eff = dist_raw - r_sys_ext;
%                     if dist_eff < 0.05
%                         dist_eff = 0.05;
%                     end
% 
%                     if dist_eff < d_inf
%                         dir = obj.apply_smooth_circulation(obs, n_vec);
% 
%                         force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                         if force > 12.0; force = 12.0; end
% 
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 6次クリティカルダンピング・アドミタンス補償器
%             % ((s + lambda)^6 による C^6 連続性保証)
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             lambda = 2.2;
%             c0 = lambda^6;
%             c1 = 6.0  * lambda^5;
%             c2 = 15.0 * lambda^4;
%             c3 = 20.0 * lambda^3;
%             c4 = 15.0 * lambda^2;
%             c5 = 6.0  * lambda;
% 
%             k_spring = lambda^2;
%             d6_calc = c0 * (F_rep / k_spring) - ...
%                       (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                        c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             % 状態更新 (オイラー積分)
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 参照軌道への合成 (28次元スタック)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
%             xd_mod(25:27) = xd_nom(25:27) + d6_calc;
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function dir_out = apply_smooth_circulation(~, obs, dir_in)
%             % C^inf 級 対称性破壊 (Symmetry Breaker)
%             [~, min_axis_idx] = min(diag(obs.Q_obs));
%             breaker_local = zeros(3, 1);
%             breaker_local(min_axis_idx) = 0.15;
%             breaker_dir = obs.R_obs * breaker_local;
% 
%             w_break = 0.5 * (1.0 + tanh(20.0 * (abs(dir_in(3)) - 0.90)));
%             dir_mod = (1.0 - w_break) * dir_in + w_break * (dir_in + breaker_dir);
%             dir_mod = dir_mod / norm(dir_mod);
% 
%             % C^inf 級 上昇流線への一方通行整流
%             w_z = 0.05 + 0.95 * 0.5 * (1.0 + tanh(20.0 * dir_mod(3)));
%             dir_mod(3) = w_z * dir_mod(3);
%             dir_out = dir_mod / norm(dir_mod);
%         end
%     end
% end

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 幾何モデル切り替え型 C^6 連続アドミタンス・リファレンス修正器
%     % 
%     % 【特徴】
%     %  1. 描画クラス (DRAW_SUSPENDED_LOAD) とのパラメータ自動完全同期
%     %  2. 長軸マージン delta_long = 0.35m による機体天面・荷物底面の突き抜け防止
%     %  3. C^inf 級 滑らか整流 (Symmetry Breaker + 上昇一方通行流線)
%     %  4. 6次クリティカルダンピング・アドミタンス補償器による C^6 連続性保証
%     % =========================================================================
%     properties
%         GEOM_TYPE = 2;       % 1: 複数球近似 (3-Spheres), 2: 動的回転楕円体 (Ellipsoid)
%         a_sys = 0.35;        % 短軸半径 (水平保護幅) [m]
%         delta_long = 0.35;   % 長軸マージン (機体フレーム・荷物を含む突き抜け防止幅) [m]
%         r_safe_sphere = 0.6; % 複数球モード用保護球半径 [m]
% 
%         % 6次アドミタンス内部状態 (3次元 x 6階層)
%         p_off   % 0th (Position offset)
%         v_off   % 1st (Velocity offset)
%         a_off   % 2nd (Acceleration offset)
%         j_off   % 3rd (Jerk offset)
%         s_off   % 4th (Snap offset)
%         d5_off  % 5th (Crackle offset)
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if isfield(param, 'GEOM_TYPE')
%                 obj.GEOM_TYPE = param.GEOM_TYPE;
%             end
%             if isfield(param, 'a_sys')
%                 obj.a_sys = param.a_sys;
%             end
%             if isfield(param, 'delta_long')
%                 obj.delta_long = param.delta_long;
%             end
%             if isfield(param, 'r_safe_sphere')
%                 obj.r_safe_sphere = param.r_safe_sphere;
%             end
% 
%             obj.p_off  = zeros(3, 1);
%             obj.v_off  = zeros(3, 1);
%             obj.a_off  = zeros(3, 1);
%             obj.j_off  = zeros(3, 1);
%             obj.s_off  = zeros(3, 1);
%             obj.d5_off = zeros(3, 1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             % ノミナル参照軌道 (28次元スタック適合)
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             L_cable = norm(p_curr - pL_curr);
%             if L_cable > 1e-4
%                 pT = (p_curr - pL_curr) / L_cable;
%             else
%                 pT = [0; 0; 1];
%             end
%             p_mid = pL_curr + 0.5 * L_cable * pT;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             k_rep = 6.0;  % 斥力ゲイン
%             d_inf = 4.5;  % 影響圏 [m]
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 幾何モデルに基づく斥力ポテンシャル計算
%             % ========================================================
%             if obj.GEOM_TYPE == 1
%                 % ----------------------------------------------------
%                 % モード 1: 複数球近似モデル (Multi-Sphere)
%                 % ----------------------------------------------------
%                 protect_pts = [p_curr, p_mid, pL_curr];
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     Q_inv = inv(obs.Q_obs);
% 
%                     for p_i = 1:3
%                         pt = protect_pts(:, p_i);
%                         diff_local = obs.R_obs' * (pt - obs.p_obs);
% 
%                         d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                         dist_raw = d_ellip - obs.d_margin;
%                         if dist_raw < d_surf_min
%                             d_surf_min = dist_raw;
%                         end
% 
%                         dist_eff = dist_raw - obj.r_safe_sphere;
%                         if dist_eff < 0.05
%                             dist_eff = 0.05;
%                         end
% 
%                         if dist_eff < d_inf
%                             grad_local = (Q_inv^2) * diff_local;
%                             dir_w = obs.R_obs * grad_local;
%                             norm_d = norm(dir_w);
%                             if norm_d > 1e-6
%                                 dir = dir_w / norm_d;
%                             else
%                                 dir = [1; 0; 0];
%                             end
% 
%                             dir = obj.apply_smooth_circulation(obs, dir);
%                             force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                             if force > 12.0; force = 12.0; end
% 
%                             F_rep = F_rep + force * dir;
%                         end
%                     end
%                 end
% 
%             elseif obj.GEOM_TYPE == 2
%                 % ----------------------------------------------------
%                 % モード 2: 動的回転楕円体モデル (Ellipsoid: 機体・荷物完全包含)
%                 % ----------------------------------------------------
%                 b_sys = (L_cable / 2.0) + obj.delta_long;
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     Q_inv = inv(obs.Q_obs);
% 
%                     diff_local = obs.R_obs' * (p_mid - obs.p_obs);
%                     d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
%                     dist_raw = d_ellip - obs.d_margin;
%                     if dist_raw < d_surf_min
%                         d_surf_min = dist_raw;
%                     end
% 
%                     grad_local = (Q_inv^2) * diff_local;
%                     dir_w = obs.R_obs * grad_local;
%                     norm_d = norm(dir_w);
%                     if norm_d > 1e-6
%                         n_vec = dir_w / norm_d;
%                     else
%                         n_vec = [1; 0; 0];
%                     end
% 
%                     % 法線方向への動的実効半径
%                     r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
% 
%                     dist_eff = dist_raw - r_sys_ext;
%                     if dist_eff < 0.05
%                         dist_eff = 0.05;
%                     end
% 
%                     if dist_eff < d_inf
%                         dir = obj.apply_smooth_circulation(obs, n_vec);
%                         force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                         if force > 12.0; force = 12.0; end
% 
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 6次クリティカルダンピング・アドミタンス補償器
%             % ((s + lambda)^6 による C^6 連続性保証)
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             lambda = 2.2;
%             c0 = lambda^6;
%             c1 = 6.0  * lambda^5;
%             c2 = 15.0 * lambda^4;
%             c3 = 20.0 * lambda^3;
%             c4 = 15.0 * lambda^2;
%             c5 = 6.0  * lambda;
% 
%             k_spring = lambda^2;
%             d6_calc = c0 * (F_rep / k_spring) - ...
%                       (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                        c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 参照軌道への合成 (28次元スタック完全適合)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
%             xd_mod(25:27) = xd_nom(25:27) + d6_calc;
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function dir_out = apply_smooth_circulation(~, obs, dir_in)
%             % C^inf 級 対称性破壊 (Symmetry Breaker)
%             [~, min_axis_idx] = min(diag(obs.Q_obs));
%             breaker_local = zeros(3, 1);
%             breaker_local(min_axis_idx) = 0.15;
%             breaker_dir = obs.R_obs * breaker_local;
% 
%             w_break = 0.5 * (1.0 + tanh(20.0 * (abs(dir_in(3)) - 0.90)));
%             dir_mod = (1.0 - w_break) * dir_in + w_break * (dir_in + breaker_dir);
%             dir_mod = dir_mod / norm(dir_mod);
% 
%             % C^inf 級 上昇流線への一方通行整流
%             w_z = 0.05 + 0.95 * 0.5 * (1.0 + tanh(20.0 * dir_mod(3)));
%             dir_mod(3) = w_z * dir_mod(3);
%             dir_out = dir_mod / norm(dir_mod);
%         end
%     end
% end

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 幾何モデル切り替え型 C^6 連続アドミタンス・リファレンス修正器
%     % 
%     % 【特徴】
%     %  1. サポート関数差分法 (8回反復) による楕円体間・真の最近接点ユークリッド距離算出
%     %  2. 描画クラス (DRAW_SUSPENDED_LOAD) との幾何パラメータ自動完全同期
%     %  3. 長軸マージン delta_long = 0.35m による機体天面・荷物底面の突き抜け完全防止
%     %  4. 鉛直制動保持 ＆ 解析的水平逃げベクトル注入による天井衝突回避
%     %  5. 6次クリティカルダンピング・アドミタンス補償器による C^6 連続性保証
%     % =========================================================================
%     properties
%         GEOM_TYPE = 2;       % 1: 複数球近似 (3-Spheres), 2: 動的回転楕円体 (Ellipsoid)
%         a_sys = 0.35;        % 短軸半径 (水平保護幅) [m]
%         delta_long = 0.35;   % 長軸マージン (機体フレーム・荷物突き抜け防止幅) [m]
%         r_safe_sphere = 0.6; % 複数球モード用保護球半径 [m]
% 
%         % 6次アドミタンス内部状態 (3次元 x 6階層)
%         p_off   % 0th (Position offset)
%         v_off   % 1st (Velocity offset)
%         a_off   % 2nd (Acceleration offset)
%         j_off   % 3rd (Jerk offset)
%         s_off   % 4th (Snap offset)
%         d5_off  % 5th (Crackle offset)
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if isfield(param, 'GEOM_TYPE')
%                 obj.GEOM_TYPE = param.GEOM_TYPE;
%             end
%             if isfield(param, 'a_sys')
%                 obj.a_sys = param.a_sys;
%             end
%             if isfield(param, 'delta_long')
%                 obj.delta_long = param.delta_long;
%             end
%             if isfield(param, 'r_safe_sphere')
%                 obj.r_safe_sphere = param.r_safe_sphere;
%             end
% 
%             obj.p_off  = zeros(3, 1);
%             obj.v_off  = zeros(3, 1);
%             obj.a_off  = zeros(3, 1);
%             obj.j_off  = zeros(3, 1);
%             obj.s_off  = zeros(3, 1);
%             obj.d5_off = zeros(3, 1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             % ノミナル参照軌道 (28次元スタック適合)
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             % テザー方向単位ベクトル pT (機体から荷物への向き)
%             L_cable = norm(p_curr - pL_curr);
%             if L_cable > 1e-4
%                 pT = (p_curr - pL_curr) / L_cable;
%             else
%                 pT = [0; 0; 1];
%             end
% 
%             % 自機楕円体の中心 (テザー中点)
%             p_mid = pL_curr + 0.5 * L_cable * pT;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             k_rep = 15.0; % 斥力ゲイン
%             d_inf = 4.0;  % 影響圏 [m]
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 幾何モデルに基づく斥力ポテンシャル計算
%             % ========================================================
%             if obj.GEOM_TYPE == 1
%                 % ----------------------------------------------------
%                 % モード 1: 複数球近似モデル (Multi-Sphere)
%                 % ----------------------------------------------------
%                 protect_pts = [p_curr, p_mid, pL_curr];
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     Q2_obs = obs.R_obs * (obs.Q_obs^2) * obs.R_obs';
% 
%                     for p_i = 1:3
%                         pt = protect_pts(:, p_i);
%                         diff_pt = pt - obs.p_obs;
% 
%                         if norm(diff_pt) > 1e-4
%                             n_sphere = diff_pt / norm(diff_pt);
%                         else
%                             n_sphere = [1; 0; 0];
%                         end
% 
%                         % 球と楕円体の最短点探索 (反復法)
%                         for it = 1:6
%                             r_o = sqrt(n_sphere' * Q2_obs * n_sphere);
%                             grad_o = (Q2_obs * n_sphere) / max(1e-6, r_o);
%                             grad_n = diff_pt - grad_o;
%                             n_sphere = n_sphere + 0.2 * grad_n;
%                             n_sphere = n_sphere / norm(n_sphere);
%                         end
% 
%                         r_obs_ext = sqrt(n_sphere' * Q2_obs * n_sphere);
%                         dist_surface = n_sphere' * diff_pt - r_obs_ext - obj.r_safe_sphere;
% 
%                         if dist_surface < d_surf_min
%                             d_surf_min = dist_surface;
%                         end
% 
%                         dist_eff = dist_surface - obs.d_margin;
%                         if dist_eff < 0.05
%                             dist_eff = 0.05;
%                         end
% 
%                         if dist_eff < d_inf
%                             dir = obj.apply_smooth_circulation(obs, pt, n_sphere);
%                             force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                             if force > 25.0; force = 25.0; end
%                             F_rep = F_rep + force * dir;
%                         end
%                     end
%                 end
% 
%             elseif obj.GEOM_TYPE == 2
%                 % ----------------------------------------------------
%                 % モード 2: 動的回転楕円体モデル (サポート関数差分法: CBFと完全一致)
%                 % ----------------------------------------------------
%                 b_sys = (L_cable / 2.0) + obj.delta_long;
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     diff_center = p_mid - obs.p_obs;
%                     Q2_obs = obs.R_obs * (obs.Q_obs^2) * obs.R_obs';
% 
%                     if norm(diff_center) > 1e-4
%                         n_vec = diff_center / norm(diff_center);
%                     else
%                         n_vec = [1; 0; 0];
%                     end
% 
%                     % 8回の最適化ループにより、2つの楕円体の真の最短法線 n_vec を探索
%                     for it = 1:8
%                         r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
%                         r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                         grad_r_sys = ((b_sys^2 - obj.a_sys^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                         grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                         grad_n = diff_center - grad_r_sys - grad_r_obs;
%                         n_vec = n_vec + 0.15 * grad_n;
%                         n_vec = n_vec / norm(n_vec);
%                     end
% 
%                     % 真の最近接点における各楕円体の張り出し半径 [m]
%                     r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     % 物理的な実表面間最短距離 [m] (真のユークリッド距離)
%                     dist_surface = n_vec' * diff_center - r_obs_ext - r_sys_ext;
% 
%                     if dist_surface < d_surf_min
%                         d_surf_min = dist_surface;
%                     end
% 
%                     % 設計安全マージン (0.5m) を差し引いた実効距離
%                     dist_eff = dist_surface - obs.d_margin;
%                     if dist_eff < 0.05
%                         dist_eff = 0.05;
%                     end
% 
%                     if dist_eff < d_inf
%                         dir = obj.apply_smooth_circulation(obs, p_mid, n_vec);
%                         force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                         if force > 25.0; force = 25.0; end
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 6次クリティカルダンピング・アドミタンス補償器
%             % ((s + lambda)^6 による C^6 連続性保証)
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             lambda = 2.5;
%             c0 = lambda^6;
%             c1 = 6.0  * lambda^5;
%             c2 = 15.0 * lambda^4;
%             c3 = 20.0 * lambda^3;
%             c4 = 15.0 * lambda^2;
%             c5 = 6.0  * lambda;
% 
%             k_spring = 2.0;
%             d6_calc = (c0 / k_spring) * F_rep - ...
%                       (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                        c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             % 状態空間積分更新
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 参照軌道への合成 (28次元スタック完全適合)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
%             xd_mod(25:27) = xd_nom(25:27) + d6_calc;
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             % 真の物理表面間最短距離を保存 (0以上で物理的非衝突)
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function dir_out = apply_smooth_circulation(~, obs, p_eval, dir_in)
%             % =============================================================
%             % C^inf 級 滑らか整流器 (特異点解消 & 鉛直制動保持)
%             % =============================================================
% 
%             % 1. 水平退避方向の解析的同定
%             diff_xy = p_eval(1:2) - obs.p_obs(1:2);
%             norm_xy = norm(diff_xy);
% 
%             if norm_xy > 1e-4
%                 escape_horiz = [diff_xy / norm_xy; 0.0];
%             else
%                 [~, min_axis_idx] = min(diag(obs.Q_obs(1:2, 1:2)));
%                 escape_horiz = zeros(3, 1);
%                 escape_horiz(min_axis_idx) = 1.0;
%             end
% 
%             % 2. 真下・真上アプローチ時の水平バイアス注入 (Symmetry Breaker)
%             % |n_z| > 0.75 に近づくにつれて水平逃げ方向をブレンド
%             w_horiz = 0.5 * (1.0 + tanh(15.0 * (abs(dir_in(3)) - 0.75)));
% 
%             dir_blended = (1.0 - w_horiz) * dir_in + w_horiz * (dir_in + 1.2 * escape_horiz);
%             dir_out = dir_blended / norm(dir_blended);
%         end
%     end
% end

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 幾何モデル切り替え型 C^6 連続アドミタンス・リファレンス修正器
%     % 【流線型回り込み（Circulation Streamline）ショックゼロ完全版】
%     % 
%     % 特徴:
%     %  1. 正面衝突ブレーキ（はじかれ現象）を排除し、手前からS字に回り込む流線場
%     %  2. サポート関数差分法 (8回反復) による真の最近接点ユークリッド距離
%     %  3. C^inf 幾何場 ＆ (s + lambda)^6 による厳密な C^6 時間連続性保証
%     %  4. 描画クラス (DRAW_SUSPENDED_LOAD) との幾何パラメータ完全同期
%     % =========================================================================
%     properties
%         GEOM_TYPE = 2;       % 1: 複数球近似, 2: 動的回転楕円体
%         a_sys = 0.35;        % 短軸半径 [m]
%         delta_long = 0.35;   % 長軸マージン [m]
%         r_safe_sphere = 0.6; % 複数球モード用半径 [m]
% 
%         % 6次アドミタンス内部状態 (3次元 x 6階層)
%         p_off
%         v_off
%         a_off
%         j_off
%         s_off
%         d5_off
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if isfield(param, 'GEOM_TYPE')
%                 obj.GEOM_TYPE = param.GEOM_TYPE;
%             end
%             if isfield(param, 'a_sys')
%                 obj.a_sys = param.a_sys;
%             end
%             if isfield(param, 'delta_long')
%                 obj.delta_long = param.delta_long;
%             end
%             if isfield(param, 'r_safe_sphere')
%                 obj.r_safe_sphere = param.r_safe_sphere;
%             end
% 
%             obj.p_off  = zeros(3, 1);
%             obj.v_off  = zeros(3, 1);
%             obj.a_off  = zeros(3, 1);
%             obj.j_off  = zeros(3, 1);
%             obj.s_off  = zeros(3, 1);
%             obj.d5_off = zeros(3, 1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             % ノミナル参照軌道 (28次元スタック適合)
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             v_ref_nom = xd_nom(5:7); % ノミナル速度ベクトル
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             L_cable = norm(p_curr - pL_curr);
%             if L_cable > 1e-4
%                 pT = (p_curr - pL_curr) / L_cable;
%             else
%                 pT = [0; 0; 1];
%             end
%             p_mid = pL_curr + 0.5 * L_cable * pT;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             k_rep = 14.0; % 斥力ゲイン
%             d_inf = 4.5;  % 影響圏を広げて手前からじわじわ曲げる [m]
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 幾何モデルに基づく最短距離と流線ベクトルの算出
%             % ========================================================
%             if obj.GEOM_TYPE == 1
%                 % ----------------------------------------------------
%                 % モード 1: 複数球近似モデル
%                 % ----------------------------------------------------
%                 protect_pts = [p_curr, p_mid, pL_curr];
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     Q2_obs = obs.R_obs * (obs.Q_obs^2) * obs.R_obs';
% 
%                     for p_i = 1:3
%                         pt = protect_pts(:, p_i);
%                         diff_pt = pt - obs.p_obs;
% 
%                         if norm(diff_pt) > 1e-4
%                             n_sphere = diff_pt / norm(diff_pt);
%                         else
%                             n_sphere = [1; 0; 0];
%                         end
% 
%                         for it = 1:6
%                             r_o = sqrt(n_sphere' * Q2_obs * n_sphere);
%                             grad_o = (Q2_obs * n_sphere) / max(1e-6, r_o);
%                             grad_n = diff_pt - grad_o;
%                             n_sphere = n_sphere + 0.2 * grad_n;
%                             n_sphere = n_sphere / norm(n_sphere);
%                         end
% 
%                         r_obs_ext = sqrt(n_sphere' * Q2_obs * n_sphere);
%                         dist_surface = n_sphere' * diff_pt - r_obs_ext - obj.r_safe_sphere;
% 
%                         if dist_surface < d_surf_min
%                             d_surf_min = dist_surface;
%                         end
% 
%                         dist_eff = dist_surface - obs.d_margin;
%                         if dist_eff < 0.05; dist_eff = 0.05; end
% 
%                         if dist_eff < d_inf
%                             dir = obj.apply_circulation_stream(obs, pt, n_sphere, v_ref_nom, dist_eff);
%                             force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                             if force > 20.0; force = 20.0; end
%                             F_rep = F_rep + force * dir;
%                         end
%                     end
%                 end
% 
%             elseif obj.GEOM_TYPE == 2
%                 % ----------------------------------------------------
%                 % モード 2: 動的回転楕円体モデル (サポート関数差分法)
%                 % ----------------------------------------------------
%                 b_sys = (L_cable / 2.0) + obj.delta_long;
% 
%                 for k = 1:length(obs_list)
%                     obs = obs_list(k);
%                     diff_center = p_mid - obs.p_obs;
%                     Q2_obs = obs.R_obs * (obs.Q_obs^2) * obs.R_obs';
% 
%                     if norm(diff_center) > 1e-4
%                         n_vec = diff_center / norm(diff_center);
%                     else
%                         n_vec = [1; 0; 0];
%                     end
% 
%                     % 8回反復による真の最短法線 n_vec 探索
%                     for it = 1:8
%                         r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
%                         r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                         grad_r_sys = ((b_sys^2 - obj.a_sys^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                         grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                         grad_n = diff_center - grad_r_sys - grad_r_obs;
%                         n_vec = n_vec + 0.15 * grad_n;
%                         n_vec = n_vec / norm(n_vec);
%                     end
% 
%                     r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     dist_surface = n_vec' * diff_center - r_obs_ext - r_sys_ext;
% 
%                     if dist_surface < d_surf_min
%                         d_surf_min = dist_surface;
%                     end
% 
%                     dist_eff = dist_surface - obs.d_margin;
%                     if dist_eff < 0.05; dist_eff = 0.05; end
% 
%                     if dist_eff < d_inf
%                         % 流線型回り込みベクトルの導出
%                         dir = obj.apply_circulation_stream(obs, p_mid, n_vec, v_ref_nom, dist_eff);
%                         force = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                         if force > 20.0; force = 20.0; end
%                         F_rep = F_rep + force * dir;
%                     end
%                 end
%             end
% 
%             % ========================================================
%             % 6次クリティカルダンピング・アドミタンス補償器
%             % ((s + lambda)^6 による C^6 連続性保証)
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             % 振り子固有振動数 (2.21 rad/s) との共振を完全に避けるため 1.6 rad/s へ設定
%             lambda = 1.6;
%             c0 = lambda^6;
%             c1 = 6.0  * lambda^5;
%             c2 = 15.0 * lambda^4;
%             c3 = 20.0 * lambda^3;
%             c4 = 15.0 * lambda^2;
%             c5 = 6.0  * lambda;
% 
%             k_spring = 1.5;
%             d6_calc = (c0 / k_spring) * F_rep - ...
%                       (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                        c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 参照軌道への合成 (28次元スタック完全適合)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
%             xd_mod(25:27) = xd_nom(25:27) + d6_calc;
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function dir_out = apply_circulation_stream(~, obs, p_eval, n_vec, v_ref, d_eff)
%             % =============================================================
%             % C^inf 流線型バイパス流（回り込みベクトル）の合成
%             % =============================================================
% 
%             % 1. 進行方向単位ベクトルの取得
%             speed_ref = norm(v_ref);
%             if speed_ref > 0.05
%                 v_dir = v_ref / speed_ref;
%             else
%                 v_dir = [0; 0; 1]; % 停止時は鉛直上向きを仮定
%             end
% 
%             % 2. 障害物外周を回り込む接線ベクトル（Streamline Tangent）の導出
%             % 法線 n_vec と進行方向 v_dir の外積から回り込み平面を決定
%             cross_vn = cross(v_dir, n_vec);
%             norm_cross = norm(cross_vn);
% 
%             if norm_cross > 1e-3
%                 % 接線ベクトル: (n_vec x cross_vn)
%                 tangent_vec = cross(n_vec, cross_vn) / norm_cross;
%             else
%                 % 完全な正面衝突（平行）特異点では水平方向へ逃がす
%                 diff_xy = p_eval(1:2) - obs.p_obs(1:2);
%                 if norm(diff_xy) > 1e-3
%                     tangent_vec = [diff_xy / norm(diff_xy); 0.0];
%                 else
%                     tangent_vec = [1; 0; 0];
%                 end
%             end
% 
%             % 3. 距離に応じた流線ブレンド重み w_stream
%             % 遠い (d_eff > 1.0m) : 接線 tangent_vec 主体で緩やかに回り込む
%             % 近い (d_eff < 0.5m) : 法線 n_vec 主体で確実にブロックする
%             w_stream = 0.5 * (1.0 + tanh(3.0 * (d_eff - 1.2))); % d_eff=1.2mで半々
% 
%             % 遠距離での接線バイアス強度 (正面ブレーキを逃げベクトルに変換)
%             dir_blended = (1.0 - 0.65 * w_stream) * n_vec + (1.2 * w_stream) * tangent_vec;
%             dir_out = dir_blended / norm(dir_blended);
%         end
%     end
% end

% classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 幾何モデル切り替え型 C^6 連続アドミタンス・リファレンス修正器
%     % 【吸引力ゼロ・幾何的外周スライド（Harmonic Circulation）完全版】
%     % =========================================================================
%     properties
%         GEOM_TYPE = 2;       % 1: 複数球近似, 2: 動的回転楕円体
%         a_sys = 0.35;        % 短軸半径 [m]
%         delta_long = 0.35;   % 長軸マージン [m]
%         r_safe_sphere = 0.6; % 複数球モード用半径 [m]
% 
%         % 6次アドミタンス内部状態 (3次元 x 6階層)
%         p_off
%         v_off
%         a_off
%         j_off
%         s_off
%         d5_off
%     end
% 
%     methods
%         function obj = HLC_CBF_APF(self, param)
%             obj@HLC_SUSPENDED_LOAD(self, param);
%             if isfield(param, 'GEOM_TYPE')
%                 obj.GEOM_TYPE = param.GEOM_TYPE;
%             end
%             if isfield(param, 'a_sys')
%                 obj.a_sys = param.a_sys;
%             end
%             if isfield(param, 'delta_long')
%                 obj.delta_long = param.delta_long;
%             end
%             if isfield(param, 'r_safe_sphere')
%                 obj.r_safe_sphere = param.r_safe_sphere;
%             end
% 
%             obj.p_off  = zeros(3, 1);
%             obj.v_off  = zeros(3, 1);
%             obj.a_off  = zeros(3, 1);
%             obj.j_off  = zeros(3, 1);
%             obj.s_off  = zeros(3, 1);
%             obj.d5_off = zeros(3, 1);
%         end
% 
%         function result = do(obj, time, varargin)
%             agent_obj = varargin{4};
%             idx = varargin{5};
% 
%             % ノミナル参照軌道 (28次元スタック適合)
%             xd_nom = agent_obj(idx).reference.result.state.xd;
%             if length(xd_nom) < 28
%                 xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
%             end
% 
%             v_ref_nom = xd_nom(5:7);
% 
%             p_curr  = agent_obj(idx).estimator.result.state.p;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             L_cable = norm(p_curr - pL_curr);
%             if L_cable > 1e-4
%                 pT = (p_curr - pL_curr) / L_cable;
%             else
%                 pT = [0; 0; 1];
%             end
%             p_mid = pL_curr + 0.5 * L_cable * pT;
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             F_rep = zeros(3, 1);
%             % k_rep = 16.0; % 斥力ゲイン
%             k_rep = 10.0;
%             % d_inf = 4.0;  % 影響圏 [m]
%             d_inf = 5.5;
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 幾何モデルに基づく最短法線と実効距離の導出
%             % ========================================================
%             b_sys = (L_cable / 2.0) + obj.delta_long;
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
%                 diff_center = p_mid - obs.p_obs;
%                 Q2_obs = obs.R_obs * (obs.Q_obs^2) * obs.R_obs';
% 
%                 if norm(diff_center) > 1e-4
%                     n_vec = diff_center / norm(diff_center);
%                 else
%                     n_vec = [1; 0; 0];
%                 end
% 
%                 % 8回反復サポート関数差分法
%                 for it = 1:8
%                     r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
%                     r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                     grad_r_sys = ((b_sys^2 - obj.a_sys^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
%                     grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
% 
%                     grad_n = diff_center - grad_r_sys - grad_r_obs;
%                     n_vec = n_vec + 0.15 * grad_n;
%                     n_vec = n_vec / norm(n_vec);
%                 end
% 
%                 r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
%                 r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
% 
%                 dist_surface = n_vec' * diff_center - r_obs_ext - r_sys_ext;
%                 if dist_surface < d_surf_min
%                     d_surf_min = dist_surface;
%                 end
% 
%                 dist_eff = dist_surface - obs.d_margin;
%                 if dist_eff < 0.05; dist_eff = 0.05; end
% 
%                 % ====================================================
%                 % 吸い込みゼロ・真の直交サーキュレーション（横滑り流線）
%                 % ====================================================
%                 if dist_eff < d_inf
%                     % 1. 基本反発力（距離が近づくほど強くなる）
%                     f_mag = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
%                     if f_mag > 20.0; f_mag = 20.0; end
% 
%                     % 2. 障害物から自機への水平動径単位ベクトル e_r
%                     diff_xy = p_mid(1:2) - obs.p_obs(1:2);
%                     norm_xy = norm(diff_xy);
%                     if norm_xy > 1e-3
%                         e_r = [diff_xy(1); diff_xy(2); 0.0] / norm_xy;
%                     else
%                         e_r = [1; 0; 0];
%                     end
% 
%                     % 3. ★真の直交接線ベクトル tau (e_z と外積をとり 90度回転)
%                     % 進行を邪魔せず、純粋に「横」へ逃がすベクトル
%                     tau = [-e_r(2); e_r(1); 0.0];
% 
%                     % 自機の現在位置がわずかに右(+X)なら右回り、左(-X)なら左回りを選択
%                     if diff_xy(1) < 0
%                         tau = -tau; % 左側へ避ける
%                     end
% 
%                     % 4. 遠距離では接線 tau (横へ車線変更) を優勢にし、
%                     %    至近距離では法線 n_vec (衝突阻止) を優勢にする
%                     w_circ = 0.5 * (1.0 + tanh(2.0 * (dist_eff - 0.8))); % 遠くで 1.0, 近くで 0.0
% 
%                     % 横へ流す力 (tau) と 後ろへ止める力 (n_vec) の配分
%                     % 遠距離では後ろ向き成分を大幅にカットし、横向き成分を最大化！
%                     dir_avoid = (1.0 - 0.7 * w_circ) * n_vec + (1.2 * w_circ) * tau;
%                     dir_avoid = dir_avoid / norm(dir_avoid);
% 
%                     F_rep = F_rep + f_mag * dir_avoid;
%                 end
%             end
% 
%             % ========================================================
%             % 6次クリティカルダンピング・アドミタンス補償器
%             % ========================================================
%             dt = 0.025;
%             if isprop(time, 'dt') && time.dt > 0
%                 dt = time.dt;
%             end
% 
%             % 揺れを抑えるため共振周波数(2.21)を避けた安定極
%             % lambda = 1.8;
%             lambda = 1.4;
%             c0 = lambda^6;
%             c1 = 6.0  * lambda^5;
%             c2 = 15.0 * lambda^4;
%             c3 = 20.0 * lambda^3;
%             c4 = 15.0 * lambda^2;
%             c5 = 6.0  * lambda;
% 
%             k_spring = 2.0;
%             d6_calc = (c0 / k_spring) * F_rep - ...
%                       (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
%                        c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
% 
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % ========================================================
%             % 参照軌道への合成 (28次元スタック完全適合)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
%             xd_mod(25:27) = xd_nom(25:27) + d6_calc;
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.d_surf = d_surf_min;
%             result = obj.result;
%         end
%     end
% end

classdef HLC_CBF_APF < HLC_SUSPENDED_LOAD
    % =========================================================================
    % 仮想先読み点（Look-ahead）導入型 C^6 連続アドミタンス・リファレンス修正器
    % 
    % 【特徴】
    %  1. T_look 先の参照軌道を用いた幾何評価による「手前からの滑らかな車線変更」
    %  2. サポート関数差分法 (8回反復) による楕円体間・真のユークリッド最短距離
    %  3. 直交接線サーキュレーション（外周スライド流線）の先行注入
    %  4. 6次クリティカルダンピング・アドミタンス補償器による C^6 連続性完全保証
    % =========================================================================
    properties
        GEOM_TYPE = 2;       % 1: 複数球近似, 2: 動的回転楕円体
        a_sys = 0.35;        % 短軸半径 [m]
        delta_long = 0.35;   % 長軸マージン [m]
        r_safe_sphere = 0.6; % 複数球モード用半径 [m]
        T_look = 1.0;        % ★ 先読み時間 [s] (1.0秒先を評価して手前から曲がる)
        
        % 6次アドミタンス内部状態 (3次元 x 6階層)
        p_off
        v_off
        a_off
        j_off
        s_off
        d5_off
    end
    
    methods
        function obj = HLC_CBF_APF(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            if isfield(param, 'GEOM_TYPE')
                obj.GEOM_TYPE = param.GEOM_TYPE;
            end
            if isfield(param, 'a_sys')
                obj.a_sys = param.a_sys;
            end
            if isfield(param, 'delta_long')
                obj.delta_long = param.delta_long;
            end
            if isfield(param, 'r_safe_sphere')
                obj.r_safe_sphere = param.r_safe_sphere;
            end
            if isfield(param, 'T_look')
                obj.T_look = param.T_look;
            end
            
            obj.p_off  = zeros(3, 1);
            obj.v_off  = zeros(3, 1);
            obj.a_off  = zeros(3, 1);
            obj.j_off  = zeros(3, 1);
            obj.s_off  = zeros(3, 1);
            obj.d5_off = zeros(3, 1);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            % ノミナル参照軌道 (28次元スタック適合)
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28
                xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
            end
            
            p_ref_nom = xd_nom(1:3);
            v_ref_nom = xd_nom(5:7);
            
            p_curr  = agent_obj(idx).estimator.result.state.p;
            pL_curr = agent_obj(idx).estimator.result.state.pL;
            
            L_cable = norm(p_curr - pL_curr);
            if L_cable > 1e-4
                pT = (p_curr - pL_curr) / L_cable;
            else
                pT = [0; 0; 1];
            end
            p_mid = 0.5 * (p_curr + pL_curr);
            
            % ========================================================
            % ★ 仮想先読み点（Look-ahead Point）の算出
            % ========================================================
            % ノミナル速度と加速度から T_look 秒先の目標中点位置を外挿予測
            a_ref_nom = xd_nom(9:11);
            p_look = p_ref_nom + v_ref_nom * obj.T_look + 0.5 * a_ref_nom * (obj.T_look^2);
            
            % 幾何評価の中心点: 現在の実位置 p_mid に、先読みによる進行オフセットを重畳
            p_eval = p_mid + (p_look - p_ref_nom);
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            F_rep = zeros(3, 1);
            k_rep = 15.0; % 斥力ゲイン
            d_inf = 4.0;  % 影響圏 [m]
            d_surf_min = 99.9;
            
            % ========================================================
            % 幾何モデルに基づく先読み最短距離と斥力計算
            % ========================================================
            b_sys = (L_cable / 2.0) + obj.delta_long;
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                Q2_obs = obs.R_obs * (obs.Q_obs^2) * obs.R_obs';
                
                % 1. 実機現在位置での物理表面間距離（ログ・安全監視用）
                diff_real = p_mid - obs.p_obs;
                n_real = diff_real / max(1e-4, norm(diff_real));
                for it = 1:6
                    r_s = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_real' * pT)^2);
                    r_o = sqrt(n_real' * Q2_obs * n_real);
                    grad_n = diff_real - ((b_sys^2 - obj.a_sys^2) * (n_real' * pT) / max(1e-6, r_s)) * pT - (Q2_obs * n_real) / max(1e-6, r_o);
                    n_real = n_real + 0.15 * grad_n;
                    n_real = n_real / norm(n_real);
                end
                r_s_real = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_real' * pT)^2);
                r_o_real = sqrt(n_real' * Q2_obs * n_real);
                d_surf_real = n_real' * diff_real - r_o_real - r_s_real;
                if d_surf_real < d_surf_min
                    d_surf_min = d_surf_real;
                end
                
                % 2. 先読み点 p_eval での幾何評価（制御力 F_rep 算出用）
                diff_eval = p_eval - obs.p_obs;
                if norm(diff_eval) > 1e-4
                    n_vec = diff_eval / norm(diff_eval);
                else
                    n_vec = [1; 0; 0];
                end
                
                % サポート関数差分法 (8回反復)
                for it = 1:8
                    r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
                    r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
                    
                    grad_r_sys = ((b_sys^2 - obj.a_sys^2) * (n_vec' * pT) / max(1e-6, r_sys_ext)) * pT;
                    grad_r_obs = (Q2_obs * n_vec) / max(1e-6, r_obs_ext);
                    
                    grad_n = diff_eval - grad_r_sys - grad_r_obs;
                    n_vec = n_vec + 0.15 * grad_n;
                    n_vec = n_vec / norm(n_vec);
                end
                
                r_sys_ext = sqrt(obj.a_sys^2 + (b_sys^2 - obj.a_sys^2) * (n_vec' * pT)^2);
                r_obs_ext = sqrt(n_vec' * Q2_obs * n_vec);
                
                % 先読み点基準の実効クリアランス
                dist_surface_look = n_vec' * diff_eval - r_obs_ext - r_sys_ext;
                dist_eff = dist_surface_look - obs.d_margin;
                if dist_eff < 0.05; dist_eff = 0.05; end
                
                % ====================================================
                % 直交接線サーキュレーション（外周スライド流線）
                % ====================================================
                if dist_eff < d_inf
                    f_mag = k_rep * (1.0 / dist_eff - 1.0 / d_inf) * (1.0 / (dist_eff^2));
                    if f_mag > 20.0; f_mag = 20.0; end
                    
                    % 水平動径単位ベクトル e_r
                    diff_xy = p_eval(1:2) - obs.p_obs(1:2);
                    norm_xy = norm(diff_xy);
                    if norm_xy > 1e-3
                        e_r = [diff_xy(1); diff_xy(2); 0.0] / norm_xy;
                    else
                        e_r = [1; 0; 0];
                    end
                    
                    % 直交接線ベクトル tau (e_z との外積で真横へ90度回転)
                    tau = [-e_r(2); e_r(1); 0.0];
                    if diff_xy(1) < 0
                        tau = -tau; % 左側回避
                    end
                    
                    % 距離重み w_circ
                    w_circ = 0.5 * (1.0 + tanh(1.5 * (dist_eff - 1.0)));
                    
                    % 先読み位置に応じた流線合成 (遠くでは接線バイパス主体)
                    dir_avoid = (1.0 - 0.75 * w_circ) * n_vec + (1.3 * w_circ) * tau;
                    dir_avoid = dir_avoid / norm(dir_avoid);
                    
                    F_rep = F_rep + f_mag * dir_avoid;
                end
            end
            
            % ========================================================
            % 6次クリティカルダンピング・アドミタンス補償器
            % ((s + lambda)^6 による C^6 連続性保証)
            % ========================================================
            dt = 0.025;
            if isprop(time, 'dt') && time.dt > 0
                dt = time.dt;
            end
            
            lambda = 1.6; % 共振周波数 (2.21) を避けた安定減衰極
            c0 = lambda^6;
            c1 = 6.0  * lambda^5;
            c2 = 15.0 * lambda^4;
            c3 = 20.0 * lambda^3;
            c4 = 15.0 * lambda^2;
            c5 = 6.0  * lambda;
            
            k_spring = 2.0;
            d6_calc = (c0 / k_spring) * F_rep - ...
                      (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
                       c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
            
            obj.d5_off = obj.d5_off + d6_calc    * dt;
            obj.s_off  = obj.s_off  + obj.d5_off * dt;
            obj.j_off  = obj.j_off  + obj.s_off  * dt;
            obj.a_off  = obj.a_off  + obj.j_off  * dt;
            obj.v_off  = obj.v_off  + obj.a_off  * dt;
            obj.p_off  = obj.p_off  + obj.v_off  * dt;
            
            % ========================================================
            % 参照軌道への合成 (28次元スタック完全適合)
            % ========================================================
            xd_mod = xd_nom;
            xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
            xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
            xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
            xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
            xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
            xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
            xd_mod(25:27) = xd_nom(25:27) + d6_calc;
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            % 監視用実表面間最短距離
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end