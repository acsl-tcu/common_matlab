classdef DRAW_SUSPENDED_LOAD_MPC_CBF < DRAW_SUSPENDED_LOAD
    % DRAW_SUSPENDED_LOAD_MPC_CBF
    % - 楕円体障害物（原形コア＋外接楕円＋安全マージン）の可視化
    % - Zheng et al. (2025): 荷物〜ケーブル〜ドローンの 5連保護球描画
    % - MPC-CBF の将来 N ステップ最適予測軌道の最前面リアルタイムオーバーレイ表示
    
    properties
        replanner_ref
    end
    
    methods
        function obj = DRAW_SUSPENDED_LOAD_MPC_CBF(logger, varargin)
            obj = obj@DRAW_SUSPENDED_LOAD(logger, varargin{:});
            param = struct(varargin{:});
            
            obj.replanner_ref = [];
            if isfield(param, 'replanner')
                obj.replanner_ref = param.replanner;
            elseif isfield(param, 'self') && isprop(param.self, 'reference')
                try
                    func_list = param.self.reference.func;
                    if iscell(func_list)
                        for i = 1:length(func_list)
                            cls_name = class(func_list{i});
                            if contains(cls_name, 'CBF') || contains(cls_name, 'MPC') || ...
                               contains(cls_name, 'SMOOTH') || contains(cls_name, 'REPLAN')
                                obj.replanner_ref = func_list{i};
                                break;
                            end
                        end
                    end
                catch
                end
            end
        end
        
        function draw(obj, varargin)
            % 1. 基底クラスの機体・ワイヤ・荷物の通常描画
            draw@DRAW_SUSPENDED_LOAD(obj, varargin{:});
            
            % 2. 状態座標 (pQ: ドローン, pL: 荷物) の取得
            if nargin >= 6 && isnumeric(varargin{1})
                pQ = varargin{2}(:)';
                pL = varargin{5}(:)';
            else
                if isstruct(varargin{1}) && isfield(varargin{1}, 't')
                    time = varargin{1};
                    k = round(time.t / time.dt) + 1;
                else
                    k = obj.logger.k;
                end
                if k > obj.logger.k, k = obj.logger.k; end
                if k < 1, k = 1; end
                pL = obj.logger.data(k, "pL")';
                pQ = obj.logger.data(k, "p")';
            end
            
            % 3. パラメータの取得
            r_load  = 0.15;
            r_drone = 0.30;
            L_cable = 2.0;
            if ~isempty(obj.replanner_ref)
                if isprop(obj.replanner_ref, 'r_load'),  r_load  = obj.replanner_ref.r_load;  end
                if isprop(obj.replanner_ref, 'r_drone'), r_drone = obj.replanner_ref.r_drone; end
                if isprop(obj.replanner_ref, 'L_cable'), L_cable = obj.replanner_ref.L_cable; end
            end
            
            hold(obj.ax, 'on');
            
            % =============================================================
            % 静的障害物の描画（全障害物を初回のみ一括生成・固定）
            % =============================================================
            h_static = findobj(obj.ax, 'Tag', 'TAG_CBF_STATIC_OBSTACLE');
            if isempty(h_static)
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch
                    obs_list = [];
                end
                
                [sp_x, sp_y, sp_z] = sphere(20);
                for idx = 1:length(obs_list)
                    obs = obs_list(idx);
                    c = obs.p_center;
                    radii = obs.ellipsoid_radii;
                    R_obs = obs.R_obs;
                    d_marg = obs.d_margin;
                    
                    % A. 原形コア（赤）
                    obj.create_core_geometry(obs);
                    
                    % B. 外接ハード楕円体（オレンジ）
                    [X_hard, Y_hard, Z_hard] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii, R_obs, c);
                    surf(obj.ax, X_hard, Y_hard, Z_hard, ...
                        'FaceColor', [1.0 0.5 0.0], 'FaceAlpha', 0.18, ...
                        'EdgeColor', 'none', 'Tag', 'TAG_CBF_STATIC_OBSTACLE');
                    
                    % C. ソフトマージン境界（黄）
                    [X_soft, Y_soft, Z_soft] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii + d_marg, R_obs, c);
                    mesh(obj.ax, X_soft, Y_soft, Z_soft, ...
                        'EdgeColor', [0.9 0.8 0.1], 'FaceColor', [1.0 0.9 0.2], ...
                        'FaceAlpha', 0.04, 'LineStyle', ':', 'Tag', 'TAG_CBF_STATIC_OBSTACLE');
                end
            end
            
            % =============================================================
            % 動的保護球列の描画（Zheng 2025: 荷物〜ケーブル〜ドローン 5連球）
            % =============================================================
            delete(findobj(obj.ax, 'Tag', 'TAG_CBF_DYNAMIC_SPHERES'));
            
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            [sp_x, sp_y, sp_z] = sphere(12);
            
            for j = 1:num_spheres
                lam = lambdas(j);
                p_sphere = (1 - lam) * pL + lam * pQ;
                r_sphere = (1 - lam) * r_load + lam * r_drone;
                
                surf(obj.ax, ...
                    sp_x * r_sphere + p_sphere(1), ...
                    sp_y * r_sphere + p_sphere(2), ...
                    sp_z * r_sphere + p_sphere(3), ...
                    'FaceColor', [0.0 0.7 0.9], 'FaceAlpha', 0.12, ...
                    'EdgeColor', 'none', 'Tag', 'TAG_CBF_DYNAMIC_SPHERES');
            end
            
            % =============================================================
            % 【最前面強調表示】MPC-CBF 将来予測軌道（Horizon N ステップ）
            % =============================================================
            delete(findobj(obj.ax, 'Tag', 'TAG_MPC_PREDICTED_TRAJ'));
            
            P_pred = [];
            if ~isempty(obj.replanner_ref) && isprop(obj.replanner_ref, 'p_pred_cache')
                P_pred = obj.replanner_ref.p_pred_cache;
            end
            
            if ~isempty(P_pred) && size(P_pred, 2) >= 2
                % 1. 吊り荷の将来予測軌道（高コントラスト・蛍光グリーン実線）
                plot3(obj.ax, P_pred(1, :), P_pred(2, :), P_pred(3, :), ...
                    '-', 'Color', [0.0 1.0 0.3], 'LineWidth', 3.2, ...
                    'Tag', 'TAG_MPC_PREDICTED_TRAJ');
                
                % 2. 予測ウェイポイント点列（オレンジ丸マーカー）
                plot3(obj.ax, P_pred(1, :), P_pred(2, :), P_pred(3, :), ...
                    'o', 'MarkerEdgeColor', [1.0 0.3 0.0], 'MarkerFaceColor', [1.0 0.9 0.0], ...
                    'MarkerSize', 5.5, 'Tag', 'TAG_MPC_PREDICTED_TRAJ');
                
                % 3. ドローン本体の将来予測軌道（マゼンタ破線）
                P_pred_Q = P_pred + [0; 0; L_cable];
                plot3(obj.ax, P_pred_Q(1, :), P_pred_Q(2, :), P_pred_Q(3, :), ...
                    '--', 'Color', [1.0 0.1 0.8], 'LineWidth', 1.8, ...
                    'Tag', 'TAG_MPC_PREDICTED_TRAJ');
                
                % 4. 予測ホライズン末端（到達目標点）の星形マーカー
                plot3(obj.ax, P_pred(1, end), P_pred(2, end), P_pred(3, end), ...
                    'p', 'Color', [1.0 0.2 0.0], 'MarkerSize', 11, ...
                    'MarkerFaceColor', [1.0 0.8 0.0], ...
                    'Tag', 'TAG_MPC_PREDICTED_TRAJ');
            end
        end
    end
    
    methods (Access = private)
        function [X, Y, Z] = generate_ellipsoid_pts(~, sp_x, sp_y, sp_z, r_vec, R_obs, c)
            X_l = sp_x * r_vec(1);
            Y_l = sp_y * r_vec(2);
            Z_l = sp_z * r_vec(3);
            pts = R_obs * [X_l(:)'; Y_l(:)'; Z_l(:)'];
            X = reshape(pts(1, :), size(sp_x)) + c(1);
            Y = reshape(pts(2, :), size(sp_y)) + c(2);
            Z = reshape(pts(3, :), size(sp_z)) + c(3);
        end
        
        function h = create_core_geometry(obj, obs)
            c = obs.p_center;
            R = obs.R_obs;
            type = obs.type;
            raw  = obs.raw_param;
            
            switch lower(type)
                case 'cylinder'
                    r = raw(1); h_len = raw(2);
                    [cx, cy, cz] = cylinder(r, 24);
                    cz = (cz - 0.5) * h_len;
                    pts = R * [cx(:)'; cy(:)'; cz(:)'];
                    X = reshape(pts(1, :), size(cx)) + c(1);
                    Y = reshape(pts(2, :), size(cy)) + c(2);
                    Z = reshape(pts(3, :), size(cz)) + c(3);
                    h = surf(obj.ax, X, Y, Z, 'FaceColor', [0.85 0.15 0.15], ...
                        'FaceAlpha', 0.9, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_STATIC_OBSTACLE');
                    
                case 'box'
                    dx = raw(1)/2; dy = raw(2)/2; dz = raw(3)/2;
                    v_box = [-dx -dy -dz; dx -dy -dz; dx dy -dz; -dx dy -dz; ...
                             -dx -dy  dz; dx -dy  dz; dx dy  dz; -dx dy  dz]';
                    v_world = R * v_box + c;
                    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                    h = patch(obj.ax, 'Vertices', v_world', 'Faces', faces, ...
                        'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.9, ...
                        'EdgeColor', 'none', 'Tag', 'TAG_CBF_STATIC_OBSTACLE');
                    
                otherwise
                    [sx, sy, sz] = sphere(16);
                    r_sph = max(obs.ellipsoid_radii) * 0.7;
                    h = surf(obj.ax, sx*r_sph + c(1), sy*r_sph + c(2), sz*r_sph + c(3), ...
                        'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.9, ...
                        'EdgeColor', 'none', 'Tag', 'TAG_CBF_STATIC_OBSTACLE');
            end
        end
    end
end