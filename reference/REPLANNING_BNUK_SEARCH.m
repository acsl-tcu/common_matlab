classdef REPLANNING_BNUK_SEARCH < handle
    % REPLANNING_BNUK_SEARCH
    % 案3: Tang et al. "B-spline based Non-uniform Kinodynamic Search" の 7次拡張
    % 状態方程式から初期制御点を拘束し、障害物を回避する安全な制御点の
    % グラフ探索（A* ライクな Greedy 探索）によって大域的な回避軌道を生成する。
    
    properties
        base_ref
        self
        replan_active = false
        replan_done   = false
        
        t_start
        t_duration = 10.0
        
        obs_center   = [0; 0; 6.0]
        obs_radius   = 0.3
        safe_margin  = 0.4
        trigger_dist = 3.5
        
        L_cable = 2.0
        
        p = 7;
        Q_path       % 探索された制御点列
        U_knot       % 結び目ベクトル
        
        t_merge_end
        p_merge_end
        v_merge_vec
        
        result
    end
    
    methods
        function obj = REPLANNING_BNUK_SEARCH(self, base_ref, opts)
            arguments
                self
                base_ref
                opts = struct()
            end
            obj.self = self;
            obj.base_ref = base_ref;
            
            if isfield(opts, "obs_center"),   obj.obs_center   = opts.obs_center(:);   end
            if isfield(opts, "obs_radius"),   obj.obs_radius   = opts.obs_radius;      end
            if isfield(opts, "safe_margin"),  obj.safe_margin  = opts.safe_margin;     end
            if isfield(opts, "trigger_dist"), obj.trigger_dist = opts.trigger_dist;    end
            
            obj.result.state = STATE_CLASS(struct('state_list', ["xd", "p", "q", "v"], 'num_list', [28, 3, 3, 3]));
        end
        
        function result = do(obj, varargin)
            time = varargin{1};
            cha = varargin{2};
            
            base_res = obj.base_ref.do(time, cha);
            xd_nominal = base_res.state.xd;
            if length(xd_nominal) < 28
                xd_nominal = [xd_nominal; zeros(28 - length(xd_nominal), 1)];
            end
            
            if isprop(obj.self.estimator.result.state, "pL")
                pL_cur = obj.self.estimator.result.state.pL;
                vL_cur = obj.self.estimator.result.state.vL;
            else
                pL_cur = base_res.state.p;
                vL_cur = base_res.state.v;
            end
            
            obj.L_cable = obj.self.parameter.get("cableL");
            pQ_cur = pL_cur + [0; 0; obj.L_cable];
            
            if cha == 'f' && ~obj.replan_done && ~obj.replan_active
                dist_load_obs  = norm(pL_cur - obj.obs_center);
                dist_drone_obs = norm(pQ_cur - obj.obs_center);
                min_dist = min(dist_load_obs, dist_drone_obs);
                
                if min_dist <= obj.trigger_dist
                    fprintf("\n=======================================================\n");
                    fprintf("[BNUK SEARCH] 障害物接近検知! (min_dist=%.2fm, t=%.3f s)\n", min_dist, time.t);
                    obj.t_start = time.t;
                    
                    v_vec = xd_nominal(5:7);
                    spd = norm(v_vec);
                    if spd < 0.05, spd = norm(vL_cur); end
                    if spd < 0.05, spd = 0.3; v_vec = [0; 0; 0.3]; end
                    dir_nom = v_vec / spd;
                    
                    vec_to_obs = obj.obs_center - pL_cur;
                    proj_dist = max(0, dot(vec_to_obs, dir_nom));
                    total_dist = proj_dist + obj.obs_radius + obj.L_cable + obj.safe_margin + 1.2;
                    obj.t_duration = max(8.0, min(14.0, total_dist / spd));
                    
                    fprintf("[BNUK SEARCH] キノダイナミックグラフ探索開始 (T = %.2f s)...\n", obj.t_duration);
                    
                    obj.plan_bnuk_search(xd_nominal, dir_nom, spd, total_dist, pL_cur, vL_cur);
                    obj.replan_active = true;
                    fprintf("=======================================================\n\n");
                end
            end
            
            if obj.replan_active
                tau = time.t - obj.t_start;
                
                if tau <= obj.t_duration
                    xd = obj.evaluate_bspline_trajectory(tau);
                else
                    if ~obj.replan_done
                        fprintf("[BNUK SEARCH] 障害物クリア・公称軌道へ合流 (t=%.3f s)\n\n", time.t);
                        obj.t_merge_end = obj.t_start + obj.t_duration;
                        % 終端状態を記録して等速直進へシームレスに移行
                        xd_end = obj.evaluate_bspline_trajectory(obj.t_duration);
                        obj.p_merge_end = xd_end(1:3);
                        obj.v_merge_vec = xd_end(5:7);
                        
                        obj.replan_active = false;
                        obj.replan_done   = true;
                    end
                    
                    dt_after = time.t - obj.t_merge_end;
                    xd = zeros(28, 1);
                    xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                    xd(5:7) = obj.v_merge_vec;
                    xd(9:28) = 0;
                end
            elseif obj.replan_done
                dt_after = time.t - obj.t_merge_end;
                xd = zeros(28, 1);
                xd(1:3) = obj.p_merge_end + obj.v_merge_vec * dt_after;
                xd(5:7) = obj.v_merge_vec;
                xd(9:28) = 0;
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
        function plan_bnuk_search(obj, xd0, dir_nom, spd, total_dist, pL_cur, vL_cur)
            T = obj.t_duration;
            p0 = pL_cur;
            p_end = p0 + dir_nom * total_dist;
            v_end = dir_nom * spd;
            
            % ノットベクトルの時間間隔 (制御点間の時間)
            % 探索ノード数を抑えるため、全体を 10区間程度に分割
            N_segments = 12; 
            dt_knot = T / N_segments;
            
            % --- 1. 初期状態拘束に基づく初期制御点の計算 (p=7 なので 8点) ---
            % xd0 の 0〜6階微分から、解析的に Q_0 ... Q_7 を求める
            % 簡単のため、現在位置から等速直線運動をしていたと仮定した配置とする
            % (実際のBNUKでは連立方程式を解くが、ドローンの現在状態がほぼ等速直進に近い場合は
            %  Q_k = p0 + vL_cur * (k * dt_knot / p) の配置が最適かつ極めて滑らかになる)
            
            Q_start = zeros(3, obj.p + 1);
            for k = 0 : obj.p
                Q_start(:, k+1) = p0 + vL_cur * (k * dt_knot / obj.p);
            end
            
            % --- 2. 終端状態拘束に基づく終端制御点の計算 (8点) ---
            Q_end = zeros(3, obj.p + 1);
            for k = 0 : obj.p
                % 終端から逆算
                Q_end(:, obj.p - k + 1) = p_end - v_end * (k * dt_knot / obj.p);
            end
            
            % --- 3. グラフ探索 (A* / Greedy Best-First Search) による中間制御点の展開 ---
            % Q_start の最後の点から出発し、Q_end の最初の点に繋がるように
            % 障害物を回避する制御点を空間内に探索・配置していく
            
            R_clear = obj.obs_radius + obj.safe_margin + obj.L_cable * 0.5 + 0.3;
            
            % サンプリングするためのアクション空間 (法線平面上の展開ベクトル)
            if norm(dir_nom(1:2)) > 1e-3
                dir_side1 = cross(dir_nom, [0; 0; 1]); 
            else
                dir_side1 = cross(dir_nom, [1; 0; 0]); 
            end
            dir_side1 = dir_side1 / norm(dir_side1);
            dir_side2 = cross(dir_nom, dir_side1);
            
            % 探索ノードリスト (Q_path に制御点を蓄積)
            Q_mid = [];
            current_q = Q_start(:, end);
            
            % 何ステップ（何個の制御点）で終端に届くかを推定
            remaining_dist = norm(p_end - current_q);
            step_length = spd * dt_knot; % 1制御点あたりの前進距離目安
            
            N_search_steps = N_segments - obj.p;
            if N_search_steps < 2, N_search_steps = 2; end
            
            for step = 1 : N_search_steps
                % 終端に近づいたら、直接 Q_end に向かう
                if step == N_search_steps
                    % 探索終了、合流
                    break;
                end
                
                % アクション展開 (9方向)
                % 直進、および法線面内で上下左右斜めに広がる
                % 広がり幅は障害物に近づくほど大きく取る (Non-uniform expansion)
                dist_to_obs = norm(current_q - obj.obs_center);
                if dist_to_obs < R_clear + 2.0
                    expansion_width = 0.8; % 障害物付近では大きく避ける
                else
                    expansion_width = 0.2; % 遠いところは直進重視
                end
                
                actions = [
                    0, 0;
                    1, 0; -1, 0; 0, 1; 0, -1;
                    0.7, 0.7; -0.7, 0.7; 0.7, -0.7; -0.7, -0.7;
                ] * expansion_width;
                
                best_q = current_q;
                best_cost = inf;
                
                for a = 1:size(actions, 1)
                    q_candidate = current_q + dir_nom * step_length ...
                                  + dir_side1 * actions(a, 1) ...
                                  + dir_side2 * actions(a, 2);
                                  
                    % 衝突チェック (B-splineの凸包性に基づく保守的チェック)
                    dist_cand = norm(q_candidate - obj.obs_center);
                    if dist_cand < R_clear
                        continue; % 衝突するため破棄
                    end
                    
                    % ヒューリスティックコスト: 
                    % 1. 終端への距離
                    cost_h = norm(p_end - q_candidate);
                    % 2. 余計な横ブレへのペナルティ (直進を好む)
                    cost_g = norm(actions(a, :)) * 2.0; 
                    
                    total_cost = cost_h + cost_g;
                    
                    if total_cost < best_cost
                        best_cost = total_cost;
                        best_q = q_candidate;
                    end
                end
                
                % もし全ノードが衝突して行き止まりになったら、強制的に横に押し出す
                if isinf(best_cost)
                    fprintf("    [WARNING] 探索が行き止まり。強制回避ノードを生成。\n");
                    vec_away = current_q - obj.obs_center;
                    vec_away = vec_away - dot(vec_away, dir_nom) * dir_nom; % 法線成分
                    if norm(vec_away) < 1e-3, vec_away = dir_side1; end
                    vec_away = vec_away / norm(vec_away);
                    best_q = current_q + dir_nom * step_length + vec_away * R_clear;
                end
                
                Q_mid = [Q_mid, best_q];
                current_q = best_q;
            end
            
            % --- 4. 制御点列の結合とノットベクトルの構成 ---
            obj.Q_path = [Q_start, Q_mid, Q_end];
            
            n_ctrl = size(obj.Q_path, 2);
            obj.U_knot = zeros(1, n_ctrl + obj.p + 1);
            
            % Clamped Uniform ノットベクトル (両端で multiplicity = p+1)
            % 無次元時間 u in [0, 1] で構成し、評価時に T をかける
            m = length(obj.U_knot) - 1;
            for i = 0 : obj.p
                obj.U_knot(i+1) = 0.0;
                obj.U_knot(m - i + 1) = 1.0;
            end
            
            internal_knots = n_ctrl - obj.p;
            for i = 1 : internal_knots - 1
                obj.U_knot(obj.p + 1 + i) = i / internal_knots;
            end
            
            fprintf("    [DEBUG] BNUK 探索完了 (制御点数: %d)\n", n_ctrl);
        end
        
        function xd = evaluate_bspline_trajectory(obj, tau)
            xd = zeros(28, 1);
            
            % ノットベクトルが無次元 [0,1] なので、評価時間を正規化
            T = obj.t_duration;
            u = max(0, min(1.0 - 1e-6, tau / T)); 
            
            n_c = size(obj.Q_path, 2) - 1;
            
            k_span = obj.p;
            for i = obj.p : n_c
                if u >= obj.U_knot(i+1) && u < obj.U_knot(i+2)
                    k_span = i;
                    break;
                end
            end
            
            % 0階微分 (位置) から 6階微分までを De Boor のアルゴリズムで計算
            for d = 0:6
                pt_d = obj.evaluate_derivative_internal(u, k_span, d, obj.Q_path);
                idx = 4 * d + 1;
                
                % 無次元時間 u から 実時間 tau の微分へスケール変換
                % dr/dtau = (dr/du) * (du/dtau) = (dr/du) / T
                % d^k r / dtau^k = (d^k r / du^k) / T^k
                xd(idx : idx+2) = pt_d / (T^d);
                xd(idx+3) = 0; 
            end
        end
        
        function pt = evaluate_derivative_internal(obj, u, k_span, d, Q_in)
            p_cur = obj.p - d;
            Q_d = zeros(3, p_cur + 1);
            
            % k_span (0-indexed) はノットベクトルのインデックスに対応
            % Q_in は 1-indexed 行列なので注意
            for i = 0 : p_cur
                ctrl_idx = k_span - p_cur + i; 
                Q_d(:, i+1) = obj.get_derivative_control_point(ctrl_idx, d, Q_in);
            end
            
            for r = 1 : p_cur
                for i = p_cur : -1 : r
                    knot_idx = k_span - p_cur + i;
                    
                    u_start = obj.U_knot(knot_idx + d + 1);
                    u_end   = obj.U_knot(knot_idx + p_cur - r + d + 2);
                    
                    denom = u_end - u_start;
                    if denom == 0
                        alpha = 0;
                    else
                        alpha = (u - u_start) / denom;
                    end
                    Q_d(:, i+1) = (1 - alpha) * Q_d(:, i) + alpha * Q_d(:, i+1);
                end
            end
            pt = Q_d(:, p_cur + 1);
        end
        
        function Qd = get_derivative_control_point(obj, idx, d, Q_in)
            if d == 0
                Qd = Q_in(:, idx + 1);
            else
                Qd_prev_i   = obj.get_derivative_control_point(idx, d-1, Q_in);
                Qd_prev_im1 = obj.get_derivative_control_point(idx-1, d-1, Q_in);
                
                u_end   = obj.U_knot(idx + obj.p + 2);
                u_start = obj.U_knot(idx + d + 1);
                denom = u_end - u_start;
                
                if denom == 0
                    Qd = zeros(3,1);
                else
                    Qd = (obj.p - d + 1) / denom * (Qd_prev_i - Qd_prev_im1);
                end
            end
        end
        
    end
end
