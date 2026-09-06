% classdef HLC_CBF_BACKUP < HLC_SUSPENDED_LOAD
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
%         function obj = HLC_CBF_BACKUP(self, param)
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
%             v_curr = agent_obj(idx).estimator.result.state.v;
%             pL_curr = agent_obj(idx).estimator.result.state.pL;
% 
%             dec_max = 2.0; 
%             t_brake = norm(v_curr) / dec_max;
%             if t_brake < 0.5; t_brake = 0.5; end
% 
%             obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
%             danger = false;
%             d_surf_min = 99.9;
% 
%             % ========================================================
%             % 【形状近似の切り替え】
%             % 1: 複数球近似 (Multi-Sphere)
%             % 2: 楕円体近似 (Ellipsoid)
%             APPROX_TYPE = 2; 
%             % ========================================================
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
% 
%                 if APPROX_TYPE == 1
%                     protect_curr = [p_curr, (p_curr+pL_curr)/2.0, pL_curr];
%                     protect_stop = [p_curr + v_curr * t_brake, (p_curr+pL_curr)/2.0 + v_curr * t_brake, pL_curr + v_curr * t_brake];
%                     r_safe = 0.6; 
%                     Q_inv = inv(obs.Q_obs);
% 
%                     for p_i = 1:3
%                         diff_local = obs.R_obs' * (protect_curr(:, p_i) - obs.p_obs);
%                         dist_raw = sqrt(diff_local' * (Q_inv^2) * diff_local) - 1.0 - obs.d_margin;
%                         if dist_raw < d_surf_min; d_surf_min = dist_raw; end
% 
%                         diff_stop = obs.R_obs' * (protect_stop(:, p_i) - obs.p_obs);
%                         dist_stop = sqrt(diff_stop' * (Q_inv^2) * diff_stop) - 1.0 - obs.d_margin;
%                         if dist_stop < r_safe; danger = true; end
%                     end
% 
%                 elseif APPROX_TYPE == 2
%                     p_center_curr = (p_curr + pL_curr) / 2.0;
%                     p_center_stop = p_center_curr + v_curr * t_brake;
%                     L_str = norm(p_curr - pL_curr);
%                     r_xy = 0.6; r_z = (L_str / 2.0) + 0.3;
% 
%                     Q_eff = obs.Q_obs + diag([r_xy, r_xy, r_z]);
%                     Q_inv_eff = inv(Q_eff);
% 
%                     diff_local = obs.R_obs' * (p_center_curr - obs.p_obs);
%                     dist_raw = sqrt(diff_local' * (Q_inv_eff^2) * diff_local) - 1.0 - obs.d_margin;
%                     if dist_raw < d_surf_min; d_surf_min = dist_raw; end
% 
%                     diff_stop = obs.R_obs' * (p_center_stop - obs.p_obs);
%                     dist_stop = sqrt(diff_stop' * (Q_inv_eff^2) * diff_stop) - 1.0 - obs.d_margin;
%                     if dist_stop < 0.0; danger = true; end % r_safeはQ_effに内包済み
%                 end
%             end
% 
%             dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
% 
%             % ========================================================
%             % 1. 仮想目標加速度 (a_cmd) の計算
%             % dangerのON/OFFにより a_cmd は不連続(ステップ状)に変化する
%             % ========================================================
%             if danger
%                 v_des_off = -xd_nom(5:7);
%                 a_cmd = 3.0 * (v_des_off - obj.v_off);
%                 if norm(a_cmd) > dec_max; a_cmd = dec_max * (a_cmd / norm(a_cmd)); end
%             else
%                 a_cmd = -3.0 * obj.v_off - 1.0 * obj.p_off;
%             end
% 
%             % ========================================================
%             % 2. 4次クリティカルダンピングフィルタによる C^6 連続性の確保
%             % a_cmd がステップ変化しても、実際の a_off は滑らかに追従し、
%             % かつ微分平坦性に必要な6階微分(Crackle)までを安全に生成する。
%             % ========================================================
%             lambda = 4.0; % a_cmd への追従速度 (PD制御より十分速い極に設定)
%             c0 = lambda^4;
%             c1 = 4 * lambda^3;
%             c2 = 6 * lambda^2;
%             c3 = 4 * lambda;
% 
%             % 6階微分(Crackle)の計算 (a_cmdを目標値とする4次ローパスフィルタ)
%             d6_calc = c0 * a_cmd - (c3 * obj.d5_off + c2 * obj.s_off + c1 * obj.j_off + c0 * obj.a_off);
% 
%             % 状態の更新 (オイラー積分)
%             old_d5_off = obj.d5_off;
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
% 
%             % 6階微分の平滑化出力
%             d6_smooth = (obj.d5_off - old_d5_off) / dt;
% 
%             % ========================================================
%             % 3. 参照軌道への合成 (28次元)
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;   % Position (0th)
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;   % Velocity (1st)
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;   % Acceleration (2nd)
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;   % Jerk (3rd)
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;   % Snap (4th)
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;  % Pop (5th)
%             xd_mod(25:27) = xd_nom(25:27) + d6_smooth;   % Crackle (6th)
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

classdef HLC_CBF_BACKUP < HLC_SUSPENDED_LOAD
    % =========================================================================
    % 運動学的制動停止予測 (ICS近似) に基づく C^6 連続 安全バックアップ制御器
    % (位相遅れ発振根絶 & 解析的閉形式幾何判定 確定版)
    % =========================================================================
    properties
        p_off   % 0th (Position offset)
        v_off   % 1st (Velocity offset)
        a_off   % 2nd (Acceleration offset)
        j_off   % 3rd (Jerk offset)
        s_off   % 4th (Snap offset)
        d5_off  % 5th (Crackle offset)
        
        is_latched      % 制動停止ラッチフラグ
        p_stop_locked   % 固定された停止目標位置
    end
    
    methods
        function obj = HLC_CBF_BACKUP(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off  = zeros(3, 1);
            obj.v_off  = zeros(3, 1);
            obj.a_off  = zeros(3, 1);
            obj.j_off  = zeros(3, 1);
            obj.s_off  = zeros(3, 1);
            obj.d5_off = zeros(3, 1);
            
            obj.is_latched = false;
            obj.p_stop_locked = zeros(3, 1);
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28
                xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)];
            end
            
            p_curr  = agent_obj(idx).estimator.result.state.p;
            v_curr  = agent_obj(idx).estimator.result.state.v;
            pL_curr = agent_obj(idx).estimator.result.state.pL;
            v_norm  = norm(v_curr);
            
            dt = 0.025;
            if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            
            % 1. 運動学的制動パラメータ (テイクオフ時の誤検知防止)
            dec_max = 2.0;       % 最大減速度 [m/s^2]
            g_acc   = 9.81;
            t_brake = (v_norm / dec_max) * tanh(4.0 * v_norm);
            
            L_cable = norm(p_curr - pL_curr);
            r_swing = L_cable * min(0.3, dec_max / g_acc); 
            
            obs_list = ENVIRONMENT_OBSTACLE_ELLIPSOID();
            d_surf_min = 99.9;
            d_stop_min = 99.9;
            
            % ========================================================
            % 【幾何判定モードの切り替え】
            % 1: 複数球近似 (Multi-Sphere)
            % 2: 解析的等価楕円体距離法 (Rimon-Koditschek 準拠)
            APPROX_TYPE = 1; 
            % ========================================================
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                
                if APPROX_TYPE == 1
                    % --- 複数球モデル ---
                    p_mid_curr = 0.5 * (p_curr + pL_curr);
                    protect_curr = [p_curr, p_mid_curr, pL_curr];
                    protect_stop = [p_curr + v_curr * t_brake, ...
                                    p_mid_curr + v_curr * t_brake, ...
                                    pL_curr + v_curr * t_brake];
                    r_base = 0.5;
                    Q_inv = inv(obs.Q_obs);
                    
                    for p_i = 1:3
                        diff_curr = obs.R_obs' * (protect_curr(:, p_i) - obs.p_obs);
                        dist_curr = sqrt(diff_curr' * (Q_inv^2) * diff_curr) - 1.0 - obs.d_margin;
                        if dist_curr < d_surf_min; d_surf_min = dist_curr; end
                        
                        diff_stop = obs.R_obs' * (protect_stop(:, p_i) - obs.p_obs);
                        dist_stop_raw = sqrt(diff_stop' * (Q_inv^2) * diff_stop) - 1.0 - obs.d_margin;
                        
                        if p_i == 3
                            dist_stop_eff = dist_stop_raw - (r_base + r_swing);
                        else
                            dist_stop_eff = dist_stop_raw - r_base;
                        end
                        if dist_stop_eff < d_stop_min; d_stop_min = dist_stop_eff; end
                    end
                    
                elseif APPROX_TYPE == 2
                    % --- 解析的閉形式 楕円体等価距離 (Rimon-Koditschek) ---
                    p_c_curr = 0.5 * (p_curr + pL_curr);
                    p_c_stop = p_c_curr + v_curr * t_brake;
                    
                    r_xy = 0.5;
                    r_z  = (L_cable / 2.0) + 0.25 + r_swing;
                    % ドローン等価体積球半径
                    r_eff_drone = (r_xy * r_xy * r_z)^(1/3);
                    
                    % 障害物ローカル座標系への投影 (動的更新なし・解析的計算)
                    diff_curr_local = obs.R_obs' * (p_c_curr - obs.p_obs);
                    diff_stop_local = obs.R_obs' * (p_c_stop - obs.p_obs);
                    Q_inv = inv(obs.Q_obs);
                    
                    % 厳密な等位面代数距離
                    dist_curr = sqrt(diff_curr_local' * (Q_inv^2) * diff_curr_local) - 1.0 - obs.d_margin;
                    dist_stop = sqrt(diff_stop_local' * (Q_inv^2) * diff_stop_local) - 1.0 - obs.d_margin - r_eff_drone;
                    
                    if dist_curr < d_surf_min; d_surf_min = dist_curr; end
                    if dist_stop < d_stop_min; d_stop_min = dist_stop; end
                end
            end
            
            % ========================================================
            % 2. 位相遅れ発振を根絶する停止ラッチ判定
            % ========================================================
            % 危険域に入った瞬間にラッチ（ブレーキ固定）
            if (d_stop_min < 0.25) || (d_surf_min < 0.40)
                if ~obj.is_latched
                    obj.is_latched = true;
                    obj.p_stop_locked = p_curr; % 現在の安全位置を固定
                end
            end
            
            % ========================================================
            % 3. 発振しない加速度指令 a_cmd の設計
            % ========================================================
            if obj.is_latched
                % 【ラッチ時】進み続ける xd_nom を無視し、現在速度を 0 に落として静止
                % 仮想オフセットは「停止点 - 現在のノミナル位置」
                p_err = (obj.p_stop_locked - (xd_nom(1:3) + obj.p_off));
                
                % 臨界減衰（D項主体）でブレーキをかけ、オーバーシュート・跳ね返りを根絶
                a_cmd = -4.0 * (xd_nom(5:7) + obj.v_off) + 1.5 * p_err;
                
                if norm(a_cmd) > dec_max
                    a_cmd = dec_max * (a_cmd / norm(a_cmd));
                end
            else
                % 【通常時】平常復帰 (低ゲインで緩やかに追従)
                sigma_danger = 0.5 * (1.0 - tanh(4.0 * (d_stop_min - 0.25)));
                v_brake_des = -xd_nom(5:7);
                
                a_brake = 3.0 * (v_brake_des - obj.v_off);
                if norm(a_brake) > dec_max; a_brake = dec_max * (a_brake / norm(a_brake)); end
                
                a_nom = -2.0 * obj.v_off - 0.2 * obj.p_off;
                a_cmd = sigma_danger * a_brake + (1.0 - sigma_danger) * a_nom;
            end
            
            % ========================================================
            % 4. 4次クリティカルダンピングフィルタ (C^6 連続性保証)
            % ========================================================
            lambda = 3.0;
            c0 = lambda^4;
            c1 = 4.0 * lambda^3;
            c2 = 6.0 * lambda^2;
            c3 = 4.0 * lambda;
            
            d6_calc = c0 * a_cmd - (c3 * obj.d5_off + c2 * obj.s_off + c1 * obj.j_off + c0 * obj.a_off);
            
            obj.d5_off = obj.d5_off + d6_calc    * dt;
            obj.s_off  = obj.s_off  + obj.d5_off * dt;
            obj.j_off  = obj.j_off  + obj.s_off  * dt;
            obj.a_off  = obj.a_off  + obj.j_off  * dt;
            obj.v_off  = obj.v_off  + obj.a_off  * dt;
            obj.p_off  = obj.p_off  + obj.v_off  * dt;
            
            % ========================================================
            % 5. 参照軌道への合成
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
            
            obj.result.d_surf = d_surf_min;
            result = obj.result;
        end
    end
end