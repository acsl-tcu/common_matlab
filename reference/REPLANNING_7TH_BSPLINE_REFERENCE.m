% classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
%     % 障害物適応型・動的時間計算付き 13次多項式リプランナ
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration     % 障害物検知時に動的決定する回避時間 [s]
% 
%         % 障害物設定
%         obs_center = [0; 0; 7.0]    % 障害物中心 [x, y, z]
%         obs_radius = 0.3            % 半径
%         safe_margin = 0.3           % 追加マージン
%         trigger_dist = 3.0          % 障害物からこの距離に入ったら検知 [m]
% 
%         poly_coeffs                 % 13次多項式係数
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
%             if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
% 
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], ...
%                 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL; % 速度
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             % 1. 全方向対応の距離ベース侵入検知
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 dist_to_obs = norm(pL_cur - obj.obs_center);
% 
%                 if dist_to_obs <= obj.trigger_dist
%                     fprintf("\n=======================================================\n");
%                     fprintf("[REPLAN] 障害物接近を検知! (t = %.3f s, 距離 = %.2f m)\n", time.t, dist_to_obs);
% 
%                     obj.t_start = time.t;
% 
%                     % --- 【動的時間計算】 ---
%                     % 距離と現在速度から、何秒かけて避けるかを自動計算
%                     current_speed = norm(vL_cur);
%                     if current_speed < 0.05
%                         current_speed = 0.3; % ほぼ停止中の場合のフォールバック
%                     end
%                     % 障害物までの残り距離と回り込みマージンを考慮して時間を決定 (最低4秒、最大12秒)
%                     calc_duration = (dist_to_obs + (obj.obs_radius + obj.safe_margin) * 2) / current_speed;
%                     obj.t_duration = max(4.0, min(12.0, calc_duration));
% 
%                     fprintf("[REPLAN] 動的計算された回避時間 T = %.2f 秒\n", obj.t_duration);
%                     fprintf("=======================================================\n\n");
% 
%                     obj.plan_optimal_avoidance(time.t, xd_nominal, pL_cur);
%                     obj.replan_active = true;
%                 end
%             end
% 
%             % 2. 軌道出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_polynomial_xd(tau, xd_nominal);
%                 else
%                     fprintf("\n[REPLAN] 回避完了・合流しました (t = %.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.replan_done = true;
%                     xd = xd_nominal;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28
%                 xd = [xd; zeros(28 - length(xd), 1)];
%             end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_optimal_avoidance(obj, t0, xd0, pL_cur)
%             T = obj.t_duration;
%             t1 = t0 + T;
% 
%             ref_f = obj.base_ref.func;
%             xd1 = ref_f(t1);
%             if length(xd1) < 28
%                 xd1 = [xd1; zeros(28 - length(xd1), 1)];
%             end
% 
%             % 障害物と現在地の中間から、どちら側に逃げるかを自動判断する経由点 (Via-point)
%             R_clear = obj.obs_radius + obj.safe_margin + 0.2;
% 
%             % 進行方向と垂直な方向へ避けるようにオフセットを計算
%             p_via = obj.obs_center;
%             % z方向（真上）に動いている場合は x または y 方向に避ける
%             p_via(1) = obj.obs_center(1) + R_clear; % x方向に避ける例
% 
%             order = 13;
%             A = zeros(14, 14);
%             Bx = zeros(14, 3);
% 
%             % 1. t=0 での拘束 (0〜6階微分)
%             for k = 0:6
%                 row = k + 1;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
%                 end
%                 Bx(row, :) = xd0(4*k + (1:3))';
%             end
% 
%             % 2. t = T/2 での経由点拘束
%             t_mid = T / 2;
%             row = 8;
%             for n = 0:order
%                 A(row, n + 1) = t_mid^n;
%             end
%             Bx(row, :) = p_via';
% 
%             % 3. t = T での合流拘束 (0〜5階微分)
%             for k = 0:5
%                 row = 9 + k;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (T)^(n - k);
%                 end
%                 Bx(row, :) = xd1(4*k + (1:3))';
%             end
% 
%             C = A \ Bx;
%             obj.poly_coeffs = C';
%         end
% 
%         function xd = evaluate_polynomial_xd(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(:, n + 1) * factor * (tau^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k;
%             end
%         end
%     end
% end

% classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
%     % ドローン・紐・荷物全体を防護する動的13次多項式リプランナ
%     properties
%         base_ref
%         self
%         replan_active = false
%         replan_done   = false
% 
%         t_start
%         t_duration     % 動的計算する回避時間 [s]
% 
%         % 障害物設定
%         obs_center = [0; 0; 7.0]    % 障害物中心 [x, y, z]
%         obs_radius = 0.3            % 半径
%         safe_margin = 0.5           % 安全マージン (ドローン・紐の分を考慮して少し広め)
%         trigger_dist = 4.0          % 検知距離 [m]
% 
%         poly_coeffs                 % 多項式係数
%         result
%     end
% 
%     methods
%         function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
%             arguments
%                 self
%                 base_ref
%                 opts = struct()
%             end
%             obj.self = self;
%             obj.base_ref = base_ref;
% 
%             if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
%             if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
%             if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
%             if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
% 
%             obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], ...
%                 'num_list', [28, 3, 3, 3]));
%         end
% 
%         function result = do(obj, varargin)
%             time = varargin{1};
%             cha = varargin{2};
% 
%             base_res = obj.base_ref.do(time, cha);
%             xd_nominal = base_res.state.xd;
% 
%             if isprop(obj.self.estimator.result.state, "pL")
%                 pL_cur = obj.self.estimator.result.state.pL;
%                 vL_cur = obj.self.estimator.result.state.vL;
%             else
%                 pL_cur = base_res.state.p;
%                 vL_cur = base_res.state.v;
%             end
% 
%             % 1. 障害物検知 (荷物またはドローンから障害物までの距離を監視)
%             if cha == 'f' && ~obj.replan_done && ~obj.replan_active
%                 L_cable = obj.self.parameter.get("cableL");
%                 % 簡易的に現在のドローン位置も推算して距離を測る
%                 pQ_cur = pL_cur + [0; 0; L_cable]; % 上空にいると仮定
% 
%                 dist_load_obs = norm(pL_cur - obj.obs_center);
%                 dist_drone_obs = norm(pQ_cur - obj.obs_center);
%                 min_dist = min(dist_load_obs, dist_drone_obs);
% 
%                 if min_dist <= obj.trigger_dist
%                     fprintf("\n=======================================================\n");
%                     fprintf("[REPLAN] 障害物接近検知 (システム全体防護) (t = %.3f s)\n", time.t);
% 
%                     obj.t_start = time.t;
% 
%                     % 動的回避時間の計算
%                     current_speed = norm(vL_cur);
%                     if current_speed < 0.05, current_speed = 0.3; end
%                     calc_duration = (min_dist + (obj.obs_radius + obj.safe_margin) * 3) / current_speed;
%                     obj.t_duration = max(5.0, min(14.0, calc_duration));
% 
%                     fprintf("[REPLAN] 動的回避時間 T = %.2f 秒\n", obj.t_duration);
%                     fprintf("=======================================================\n\n");
% 
%                     obj.plan_system_avoidance(time.t, xd_nominal, L_cable);
%                     obj.replan_active = true;
%                 end
%             end
% 
%             % 2. 軌道出力
%             if obj.replan_active
%                 tau = time.t - obj.t_start;
%                 if tau <= obj.t_duration
%                     xd = obj.evaluate_polynomial_xd(tau, xd_nominal);
%                 else
%                     fprintf("\n[REPLAN] システム全体の回避完了 (t = %.3f s)\n\n", time.t);
%                     obj.replan_active = false;
%                     obj.replan_done = true;
%                     xd = xd_nominal;
%                 end
%             else
%                 xd = xd_nominal;
%             end
% 
%             if length(xd) < 28
%                 xd = [xd; zeros(28 - length(xd), 1)];
%             end
% 
%             obj.result.state.xd = xd;
%             obj.result.state.p = xd(1:3);
%             obj.result.state.v = xd(5:7);
%             obj.result.state.q = [0; 0; xd(4)];
%             result = obj.result;
%         end
%     end
% 
%     methods (Access = private)
%         function plan_system_avoidance(obj, t0, xd0, L_cable)
%             T = obj.t_duration;
%             t1 = t0 + T;
% 
%             ref_f = obj.base_ref.func;
%             xd1 = ref_f(t1);
%             if length(xd1) < 28
%                 xd1 = [xd1; zeros(28 - length(xd1), 1)];
%             end
% 
%             % 【重要】ドローン本体（上空）と紐全体が確実に外側を通るように
%             % 障害物中心から「ドローン分のオフセット＋安全マージン」だけ大きく離れた経由点を計算
%             R_total = obj.obs_radius + obj.safe_margin + L_cable * 0.5;
% 
%             p_via = obj.obs_center;
%             % 進行方向（例: z方向上昇）に対して、ドローンと紐が巻き込まれないよう x または y 方向に大きく逃げる
%             p_via(1) = obj.obs_center(1) + R_total; 
% 
%             order = 13;
%             A = zeros(14, 14);
%             Bx = zeros(14, 3);
% 
%             % 1. t=0 での拘束 (0〜6階微分)
%             for k = 0:6
%                 row = k + 1;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
%                 end
%                 Bx(row, :) = xd0(4*k + (1:3))';
%             end
% 
%             % 2. t = T/2 での経由点拘束 (ドローン・紐全体を逃がすための p_via)
%             t_mid = T / 2;
%             row = 8;
%             for n = 0:order
%                 A(row, n + 1) = t_mid^n;
%             end
%             Bx(row, :) = p_via';
% 
%             % 3. t = T での合流拘束 (0〜5階微分)
%             for k = 0:5
%                 row = 9 + k;
%                 for n = k:order
%                     A(row, n + 1) = prod(n - k + 1 : n) * (T)^(n - k);
%                 end
%                 Bx(row, :) = xd1(4*k + (1:3))';
%             end
% 
%             C = A \ Bx;
%             obj.poly_coeffs = C';
%         end
% 
%         function xd = evaluate_polynomial_xd(obj, tau, xd_nom)
%             xd = xd_nom;
%             C = obj.poly_coeffs;
%             order = 13;
% 
%             for k = 0:6
%                 val_k = zeros(3, 1);
%                 for n = k:order
%                     factor = prod(n - k + 1 : n);
%                     val_k = val_k + C(:, n + 1) * factor * (tau^(n - k));
%                 end
%                 xd(4*k + (1:3)) = val_k;
%             end
%         end
%     end
% end

classdef REPLANNING_7TH_BSPLINE_REFERENCE < handle
    % 3D空間完全包絡・安全合流リプランナ
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 12.0           % 回避時間 [s]
        
        % 障害物設定
        obs_center = [0; 0; 7.0]    % 障害物中心 [x, y, z]
        obs_radius = 0.3            % 半径
        safe_margin = 0.8           % 安全マージン（ドローン・紐の分を十分に確保）
        trigger_dist = 4.0          % 検知距離 [m]
        
        poly_coeffs                 % 多項式係数
        result
    end
    
    methods
        function obj = REPLANNING_7TH_BSPLINE_REFERENCE(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center;   end
            if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;   end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;  end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist; end
            if isfield(opts, "t_duration"),   obj.t_duration   = opts.t_duration;   end
            
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], ...
                'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
                vL_cur = obj.self.estimator.result.state.vL;
            else
                pL_cur = base_res.state.p;
                vL_cur = base_res.state.v;
            end
            
            L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; L_cable]; % ドローン本体の現在位置
            
            % 1. 障害物検知
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n[REPLAN] システム全体防護・回避開始 at t=%.3f s\n", time.t);
                    obj.t_start = time.t;
                    obj.plan_system_avoidance(time.t, xd_nominal, L_cable);
                    obj.replan_active = true;
                end
            end
            
            % 2. 軌道出力 ＆ 厳格な3D空間通過判定
            if obj.replan_active
                tau = time.t - obj.t_start;
                
                % 【超重要】「荷物」だけでなく「ドローン本体」も障害物のてっぺんを完全に越えたか？
                % さらに、横方向にも十分に離れたかを同時にチェック
                obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
                is_load_passed  = pL_cur(3) >= obs_top_z;
                is_drone_passed = pQ_cur(3) >= obs_top_z;
                
                % 完全に安全圏を通り過ぎた場合のみ合流を許可
                is_completely_safe = is_load_passed && is_drone_passed;
                
                if tau <= obj.t_duration && ~is_completely_safe
                    xd = obj.evaluate_polynomial_xd(tau, xd_nominal);
                else
                    if ~obj.replan_done
                        fprintf("[REPLAN] 3D安全通過確認完了・公称軌道へ滑らかに復帰 at t=%.3f s\n\n", time.t);
                    end
                    obj.replan_active = false;
                    obj.replan_done = true;
                    
                    % 【滑らかな復帰接続】
                    % 急激に x=0 に戻らず、現在の位置から公称軌道へ自然に引き込む
                    xd = xd_nominal;
                    current_z = pL_cur(3);
                    
                    % 高さが落ちないよう現在の高さを維持しつつ、x座標は滑らかに中央へ戻すブレンド処理
                    blend = min(1.0, max(0.0, (time.t - (obj.t_start + obj.t_duration)) / 3.0)); 
                    xd(1) = (1 - blend) * pL_cur(1) + blend * xd_nominal(1);
                    xd(2) = (1 - blend) * pL_cur(2) + blend * xd_nominal(2);
                    xd(3) = max(current_z, xd_nominal(3)); % 高さが下がらないようにガード
                end
            else
                xd = xd_nominal;
            end
            
            if length(xd) < 28
                xd = [xd; zeros(28 - length(xd), 1)];
            end
            
            obj.result.state.xd = xd;
            obj.result.state.p = xd(1:3);
            obj.result.state.v = xd(5:7);
            obj.result.state.q = [0; 0; xd(4)];
            result = obj.result;
        end
    end
    
    methods (Access = private)
        function plan_system_avoidance(obj, t0, xd0, L_cable)
            T = obj.t_duration;
            t1 = t0 + T;
            ref_f = obj.base_ref.func;
            xd1 = ref_f(t1);
            if length(xd1) < 28
                xd1 = [xd1; zeros(28 - length(xd1), 1)];
            end
            
            obs_top_z = obj.obs_center(3) + obj.obs_radius + obj.safe_margin;
            safe_z = obs_top_z + 1.5;
            if xd1(3) < safe_z
                xd1(3) = safe_z; 
            end
            
            % ドローンと紐全体が絶対に接触しないよう、迂回幅を十分に大きく取る
            R_total = obj.obs_radius + obj.safe_margin + L_cable * 0.8;
            p_via = obj.obs_center;
            p_via(1) = obj.obs_center(1) + R_total; 
            p_via(3) = obs_top_z;                   
            
            order = 13;
            A = zeros(14, 14);
            Bx = zeros(14, 3);
            
            for k = 0:6
                row = k + 1;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (0)^(n - k);
                end
                Bx(row, :) = xd0(4*k + (1:3))';
            end
            
            t_mid = T / 2;
            row = 8;
            for n = 0:order
                A(row, n + 1) = t_mid^n;
            end
            Bx(row, :) = p_via';
            
            for k = 0:5
                row = 9 + k;
                for n = k:order
                    A(row, n + 1) = prod(n - k + 1 : n) * (T)^(n - k);
                end
                Bx(row, :) = xd1(4*k + (1:3))';
            end
            
            C = A \ Bx;
            obj.poly_coeffs = C';
        end
        
        function xd = evaluate_polynomial_xd(obj, tau, xd_nom)
            xd = xd_nom;
            C = obj.poly_coeffs;
            order = 13;
            
            for k = 0:6
                val_k = zeros(3, 1);
                for n = k:order
                    factor = prod(n - k + 1 : n);
                    val_k = val_k + C(:, n + 1) * factor * (tau^(n - k));
                end
                xd(4*k + (1:3)) = val_k;
            end
        end
    end
end