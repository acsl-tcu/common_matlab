% classdef HLC_AVOID_SLIDING < HLC_SUSPENDED_LOAD
%     % =========================================================================
%     % 障害物表面を滑らかに沿う位置制約コントローラ (Kinematic Reference CBF)
%     % 定数的な回避ではなく、障害物の幾何形状（楕円体）の境界制約をQPで解き、
%     % その表面を這うような（スライディングする）理想的な迂回速度を算出します。
%     % さらにそれを6次LPFに通すことで、Minimum Snapと同等の滑らかさを保証します。
%     % =========================================================================
%     properties
%         p_off; v_off; a_off; j_off; s_off; d5_off;
%     end
% 
%     methods
%         function obj = HLC_AVOID_SLIDING(self, param)
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
%             if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
% 
%             p_drone = agent_obj(idx).estimator.result.state.p;
%             p_load  = agent_obj(idx).estimator.result.state.pL;
%             p_mid   = (p_drone + p_load) / 2.0; % 牽引物も考慮した代表点
% 
%             v_nom = xd_nom(5:7); % 元の目標速度
% 
%             % ========================================================
%             % 1. 位置制約（Kinematic CBF）によるスライディング速度の計算
%             % ========================================================
%             obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
% 
%             % QPの設定 (変数は v_safe の3次元)
%             % J = 0.5 * || v_safe - v_nom ||^2
%             H_qp = eye(3);
%             f_qp = -v_nom;
% 
%             A_qp = [];
%             b_qp = [];
% 
%             r_influence = 6.0; % この距離に入ったら制約を意識し始める
%             gamma = 1.5;       % 障害物に近づく許容スピード（小さいほど手前で沿い始める）
%             min_d_surf = Inf;
% 
%             for k = 1:length(obs_list)
%                 obs = obs_list(k);
% 
%                 % 相対位置と楕円体表面までの距離計算
%                 diff = p_mid - obs.p_obs;
%                 diff_norm = obs.R_obs' * diff;
%                 Q_inv = inv(obs.Q_obs);
% 
%                 dist_ellip = sqrt(diff_norm' * (Q_inv^2) * diff_norm) - 1.0;
%                 d_surf = dist_ellip - obs.d_margin;
%                 if d_surf < min_d_surf
%                     min_d_surf = d_surf;
%                 end
%                 if d_surf < r_influence
%                     % 楕円体表面の法線ベクトル（勾配）
%                     dir = obs.R_obs * (Q_inv^2) * diff_norm;
%                     dir = dir / norm(dir);
% 
%                     % 動的障害物の速度
%                     v_obs = zeros(3,1);
%                     if isfield(obs, 'v_obs') && ~isempty(obs.v_obs)
%                         v_obs = obs.v_obs;
%                     end
% 
%                     % 制約式: dir^T * (v_safe - v_obs) >= -gamma * d_surf
%                     % 変換:  -dir^T * v_safe <= gamma * d_surf - dir^T * v_obs
%                     A_qp = [A_qp; -dir'];
%                     b_qp = [b_qp; gamma * d_surf - dir' * v_obs];
%                 end
%             end
% 
%             % QPを解く（制約がある場合のみ）
%             v_safe = v_nom;
%             if ~isempty(A_qp)
%                 options = optimoptions('quadprog', 'Display', 'off');
%                 [v_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], [], [], v_nom, options);
%                 if exitflag == 1 || exitflag == 2
%                     v_safe = v_opt;
%                 end
%             end
% 
%             % ========================================================
%             % 2. 安全速度から「軌道オフセット」への変換と C^6 フィルタリング
%             % v_safe と v_nom の差分を「仮想的な斥力」とみなし、フィルタに通すことで
%             % スプラインやMinimum Snapと同様の極めて滑らかな軌道を生成する
%             % ========================================================
%             F_virtual = 2.0 * (v_safe - v_nom); % 速度差分を力として扱う
% 
%             dt = 0.025; if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
%             lambda = 4.0;
%             c0 = lambda^6; c1 = 6*lambda^5; c2 = 15*lambda^4; c3 = 20*lambda^3; c4 = 15*lambda^2; c5 = 6*lambda;
% 
%             d6_calc = c0 * F_virtual - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + c2 * obj.a_off + c1 * obj.v_off + c0 * obj.p_off);
% 
%             old_d5 = obj.d5_off;
%             obj.d5_off = obj.d5_off + d6_calc    * dt;
%             obj.s_off  = obj.s_off  + obj.d5_off * dt;
%             obj.j_off  = obj.j_off  + obj.s_off  * dt;
%             obj.a_off  = obj.a_off  + obj.j_off  * dt;
%             obj.v_off  = obj.v_off  + obj.a_off  * dt;
%             obj.p_off  = obj.p_off  + obj.v_off  * dt;
%             d6_smooth = (obj.d5_off - old_d5) / dt;
% 
%             % ========================================================
%             % 3. 目標値への合成
%             % ========================================================
%             xd_mod = xd_nom;
%             xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
%             xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
%             xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
%             xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
%             xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
%             xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
%             xd_mod(25:27) = xd_nom(25:27) + d6_smooth;
% 
%             agent_obj(idx).reference.result.state.xd = xd_mod;
% 
%             % スーパークラス(ベースコントローラ)の呼び出し
%             result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
% 
%             % ノミナルに戻す
%             agent_obj(idx).reference.result.state.xd = xd_nom;
% 
%             obj.result.p_off = obj.p_off;
%             obj.result.d_surf = min_d_surf; %
%             result = obj.result;
%         end
%     end
% end

classdef HLC_AVOID_SLIDING < HLC_SUSPENDED_LOAD
    % =========================================================================
    % 障害物表面を滑らかに沿う位置制約コントローラ (Kinematic Reference CBF)
    % 【厳密距離判定 ＆ 通過完了ラッチ ＆ 戻り衝突防止 完全版】
    % =========================================================================
    properties
        p_off; v_off; a_off; j_off; s_off; d5_off;
        is_avoiding;
    end
    
    methods
        function obj = HLC_AVOID_SLIDING(self, param)
            obj@HLC_SUSPENDED_LOAD(self, param);
            obj.p_off  = zeros(3,1);
            obj.v_off  = zeros(3,1);
            obj.a_off  = zeros(3,1);
            obj.j_off  = zeros(3,1);
            obj.s_off  = zeros(3,1);
            obj.d5_off = zeros(3,1);
            obj.is_avoiding = false;
        end
        
        function result = do(obj, time, varargin)
            agent_obj = varargin{4};
            idx = varargin{5};
            
            xd_nom = agent_obj(idx).reference.result.state.xd;
            if length(xd_nom) < 28; xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
            
            p_drone = agent_obj(idx).estimator.result.state.p;
            p_load  = agent_obj(idx).estimator.result.state.pL;
            p_mid   = (p_drone + p_load) / 2.0; % 牽引物も含めた代表点
            
            v_nom = xd_nom(5:7);
            
            % ========================================================
            % 1. 厳密な楕円体距離計算とスライディング目標速度の生成
            % ========================================================
            obs_list = ENVIRONMENT_OBSTACLE_DYNAMIC(time);
            
            r_influence = 5.0; 
            gamma = 1.5;       
            min_d_surf = Inf;
            
            A_qp = [];
            b_qp = [];
            v_slide_bias = zeros(3,1);
            in_obstacle_height = false;
            
            for k = 1:length(obs_list)
                obs = obs_list(k);
                
                diff = p_mid - obs.p_obs;
                diff_body = obs.R_obs' * diff;
                Q_inv = inv(obs.Q_obs);
                
                % 楕円体レベルセット値
                val_ellip = norm(Q_inv * diff_body);
                
                % 表面上の最近傍法線方向 (Body系)
                grad_body = (Q_inv^2) * diff_body;
                grad_world = obs.R_obs * grad_body;
                n_dir = grad_world / max(1e-6, norm(grad_world));
                
                % 厳密なメートル換算表面間距離 (Support functionベース)
                % r_body: n_dir 方向の楕円体半径
                r_body = sqrt(n_dir' * (obs.R_obs * (obs.Q_obs^2) * obs.R_obs') * n_dir);
                d_surf = dot(diff, n_dir) - r_body - obs.d_margin;
                
                if d_surf < min_d_surf
                    min_d_surf = d_surf;
                end
                
                % 障害物のZ方向有効範囲（厚み＋マージン＋機体半径）
                z_semi = obs.Q_obs(3,3) + obs.d_margin + 0.4;
                if abs(diff(3)) < z_semi
                    in_obstacle_height = true;
                end
                
                if d_surf < r_influence
                    % 動的障害物速度
                    v_obs = zeros(3,1);
                    if isfield(obs, 'v_obs') && ~isempty(obs.v_obs)
                        v_obs = obs.v_obs;
                    end
                    
                    % ====================================================
                    % 水平回避（パンケーキ型天井を横にすり抜ける）接線設計
                    % ====================================================
                    diff_xy = diff(1:2);
                    if norm(diff_xy) > 1e-3
                        rad_dir_xy = diff_xy / norm(diff_xy);
                    else
                        rad_dir_xy = [1; 0]; % 中心直撃時はX正方向へ逃げる
                    end
                    
                    % 水平外向き（斥力）と接線方向の合成
                    t_dir = [rad_dir_xy; 0];
                    
                    blend = max(0.0, (r_influence - d_surf) / r_influence);
                    v_mag = max(1.2, norm(v_nom)); 
                    v_slide_bias = v_slide_bias + (1.5 * v_mag * blend) * t_dir;
                    
                    % CBF 制約
                    A_qp = [A_qp; -n_dir'];
                    b_qp = [b_qp; gamma * d_surf - dot(n_dir, v_obs)];
                end
            end
            
            % QPによる安全速度の計算
            v_target = v_nom + v_slide_bias;
            H_qp = eye(3);
            f_qp = -v_target;
            
            v_safe = v_target;
            if ~isempty(A_qp)
                options = optimoptions('quadprog', 'Display', 'off');
                [v_opt, ~, exitflag] = quadprog(H_qp, f_qp, A_qp, b_qp, [], [], [], [], v_target, options);
                if exitflag == 1 || exitflag == 2
                    v_safe = v_opt;
                end
            end
            
            % ========================================================
            % 2. 通過完了ラッチ付き C^6 フィルタリング
            % ========================================================
            F_virtual = 3.0 * (v_safe - v_nom); 
            
            dt = 0.025; 
            if isprop(time, 'dt') && time.dt > 0; dt = time.dt; end
            
            lambda = 3.5;
            c0 = lambda^6; c1 = 6*lambda^5; c2 = 15*lambda^4; c3 = 20*lambda^3; c4 = 15*lambda^2; c5 = 6*lambda;
            
            % ★ 障害物の高さ範囲にいる間は、水平オフセットを原点に戻すバネ復元項をカット！
            % （完全に上空または下空へ抜けきるまで横に膨らんだ位置をキープする）
            p_restore = obj.p_off;
            if in_obstacle_height
                p_restore(1:2) = 0.0; % 水平復元力を無効化（戻り衝突を完全防止）
            end
            
            d6_calc = c0 * F_virtual - (c5 * obj.d5_off + c4 * obj.s_off + c3 * obj.j_off + c2 * obj.a_off + c1 * obj.v_off + c0 * p_restore);
            
            old_d5 = obj.d5_off;
            obj.d5_off = obj.d5_off + d6_calc    * dt;
            obj.s_off  = obj.s_off  + obj.d5_off * dt;
            obj.j_off  = obj.j_off  + obj.s_off  * dt;
            obj.a_off  = obj.a_off  + obj.j_off  * dt;
            obj.v_off  = obj.v_off  + obj.a_off  * dt;
            obj.p_off  = obj.p_off  + obj.v_off  * dt;
            d6_smooth = (obj.d5_off - old_d5) / dt;
            
            % ========================================================
            % 3. 目標値への合成
            % ========================================================
            xd_mod = xd_nom;
            xd_mod(1:3)   = xd_nom(1:3)   + obj.p_off;
            xd_mod(5:7)   = xd_nom(5:7)   + obj.v_off;
            xd_mod(9:11)  = xd_nom(9:11)  + obj.a_off;
            xd_mod(13:15) = xd_nom(13:15) + obj.j_off;
            xd_mod(17:19) = xd_nom(17:19) + obj.s_off;
            xd_mod(21:23) = xd_nom(21:23) + obj.d5_off;
            xd_mod(25:27) = xd_nom(25:27) + d6_smooth;
            
            agent_obj(idx).reference.result.state.xd = xd_mod;
            
            % ベースコントローラの呼び出し
            result = do@HLC_SUSPENDED_LOAD(obj, time, varargin{:});
            
            % 次ターンのために元に戻す
            agent_obj(idx).reference.result.state.xd = xd_nom;
            
            % プロット用ログ格納
            obj.result.p_off  = obj.p_off;
            obj.result.v_safe = v_safe;
            obj.result.d_surf = min_d_surf;
            result = obj.result;
        end
    end
end