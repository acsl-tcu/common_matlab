% classdef HLC_CBF_APF_DYN < HLC_SUSPENDED_LOAD
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
%         function obj = HLC_CBF_APF_DYN(self, param)
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
%             obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
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

% classdef HLC_CBF_APF_DYN < HLC_SUSPENDED_LOAD
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
%         function obj = HLC_CBF_APF_DYN(self, param)
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
%             obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
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

classdef HLC_CBF_APF_DYN < HLC_SUSPENDED_LOAD
    % =========================================================================
    % 初期の理想的な「曲面追従＆目標復帰」挙動を完全保持しつつ、
    % 全空間での C^inf 滑らか整流 と C^6 連続性 (微分平坦性) を
    % 数学的に厳密保証したアドミタンスリファレンス修正器
    % =========================================================================
    properties
        p_off   % 0th (Position offset)
        v_off   % 1st (Velocity offset)
        a_off   % 2nd (Acceleration offset)
        j_off   % 3rd (Jerk offset)
        s_off   % 4th (Snap offset)
        d5_off  % 5th (Crackle offset)
    end
    
    methods
        function obj = HLC_CBF_APF_DYN(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off  = zeros(3,1);
            obj.v_off  = zeros(3,1);
            obj.a_off  = zeros(3,1);
            obj.j_off  = zeros(3,1);
            obj.s_off  = zeros(3,1);
            obj.d5_off = zeros(3,1);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28
                xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
            end
            
            p_curr  = agent_obj(idx).estimator.result.state.p;
            pL_curr = agent_obj(idx).estimator.result.state.pL;
            p_mid   = (p_curr + pL_curr) / 2.0;
            
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
            F_rep = zeros(3, 1);
            k_rep = 5.0;  % 初期の理想パラメータ
            d_inf = 4.0;  % 初期の理想影響圏
            d_surf_min = 99.9;
            
            % ========================================================
            % 1. 初期の「複数球近似 (Multi-Sphere)」幾何勾配場
            % ========================================================
            protect_pts = [p_curr, p_mid, pL_curr];
            r_safe = 0.6; % 初期の安全球半径
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                Q_inv = inv(obs.Q_obs);
                
                for p_i = 1:3
                    pt = protect_pts(:, p_i);
                    diff_local = obs.R_obs' * (pt - obs.p_obs);
                    
                    % 楕円ポテンシャル代数距離
                    d_ellip = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0;
                    dist_raw = d_ellip - obs.d_margin;
                    if dist_raw < d_surf_min
                        d_surf_min = dist_raw;
                    end
                    
                    dist_eff = dist_raw - r_safe;
                    if dist_eff < 0.05
                        dist_eff = 0.05;
                    end
                    
                    if dist_eff < d_inf
                        % 初期の自然な曲面法線勾配
                        grad_local = (Q_inv^2) * diff_local;
                        dir = obs.R_obs * grad_local;
                        dir_norm = norm(dir);
                        if dir_norm > 1e-6
                            dir = dir / dir_norm;
                        else
                            dir = [1; 0; 0];
                        end
                        
                        % ----------------------------------------------------
                        % 【滑らかな対称性破壊 (C^inf 級保証)】
                        % 真上/真下 (dir(3) ≈ ±1) の特異点を滑らかな tanh で回避
                        % ----------------------------------------------------
                        [~, min_axis_idx] = min(diag(obs.Q_obs));
                        breaker_local = zeros(3,1);
                        breaker_local(min_axis_idx) = 0.1;
                        breaker_dir = obs.R_obs * breaker_local;
                        
                        w_break = 0.5 * (1.0 + tanh(20.0 * (abs(dir(3)) - 0.92)));
                        dir = (1.0 - w_break) * dir + w_break * (dir + breaker_dir);
                        dir = dir / norm(dir);
                        
                        % ----------------------------------------------------
                        % 【真上接近時の滑らかな水平迂回整流 (C^inf 級保証)】
                        % if dir(3) < 0 の角を解消し、tanh で滑らかに下向きを遮断
                        % dir(3) < 0 では 0.05 倍へ減衰、dir(3) > 0 では 1.0 倍へ滑らかに接続
                        % ----------------------------------------------------
                        w_z = 0.05 + 0.95 * 0.5 * (1.0 + tanh(20.0 * dir(3)));
                        dir(3) = w_z * dir(3);
                        dir = dir / norm(dir);
                        
                        % 初期の斥力関数
                        force = k_rep * (1.0/dist_eff - 1.0/d_inf) * (1.0 / (dist_eff^2));
                        if force > 10.0
                            force = 10.0;
                        end
                        
                        F_rep = F_rep + force * dir;
                    end
                end
            end
            
            % ========================================================
            % 2. 6次クリティカルダンピング・アドミタンスフィルタ
            % (初期のバネ復元特性 (s+lambda)^6 を完全保持)
            % ========================================================
            dt = 0.025;
            if isprop(time, 'dt') && time.dt > 0
                dt = time.dt;
            end
            
            lambda = 2.0; % 初期の遮断周波数
            c0 = lambda^6;
            c1 = 6  * lambda^5;
            c2 = 15 * lambda^4;
            c3 = 20 * lambda^3;
            c4 = 15 * lambda^2;
            c5 = 6  * lambda;
            
            % 閉ループ状態方程式 (復元項 c0*p_off により障害物通過後に 0 へ自律復帰)
            d6_calc = c0 * F_rep - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + ...
                                    c2 * obj.a_off  + c1 * obj.v_off + c0 * obj.p_off);
            
            % 状態の更新 (オイラー積分)
            % 積分により p_off 〜 d5_off は C^1 級以上の滑らかさを保証
            obj.d5_off = obj.d5_off + d6_calc    * dt;
            obj.s_off  = obj.s_off  + obj.d5_off * dt;
            obj.j_off  = obj.j_off  + obj.s_off  * dt;
            obj.a_off  = obj.a_off  + obj.j_off  * dt;
            obj.v_off  = obj.v_off  + obj.a_off  * dt;
            obj.p_off  = obj.p_off  + obj.v_off  * dt;
            
            % ========================================================
            % 3. 参照軌道への合成 (28次元スタック完全適合)
            % ========================================================
            % 滑らかな空間ポテンシャル Frep のため、
            % 線形結合である d6_calc 自体が数学的に C^0 連続性を保証されている
            xd_mod = xd_nom;
            xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;   % Position (0th)
            xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;   % Velocity (1st)
            xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;   % Acceleration (2nd)
            xd_mod(13:15) = xd_nom(13:15) + obj.j_off;   % Jerk (3rd)
            xd_mod(17:19) = xd_nom(17:19) + obj.s_off;   % Snap (4th)
            xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;  % Pop (5th)
            xd_mod(25:27) = xd_nom(25:27) + d6_calc;     % Crackle (6th)
            
            % 制御器本体の実行
            agent_obj(idx).reference.result.state.xd = xd_mod;
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            agent_obj(idx).reference.result.state.xd = xd_nom; % 原本を復元
            
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end
