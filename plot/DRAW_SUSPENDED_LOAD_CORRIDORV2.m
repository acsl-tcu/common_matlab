% classdef DRAW_SUSPENDED_LOAD_CORRIDORV2 < DRAW_SUSPENDED_LOAD
%     % DRAW_SUSPENDED_LOAD_CORRIDORV2
%     % 楕円体障害物・原形コア・システム側ダイナミクス保護球（タグ管理・完全残像防止版）
% 
%     properties
%         replanner_ref
%     end
% 
%     methods
%         function obj = DRAW_SUSPENDED_LOAD_CORRIDORV2(logger, varargin)
%             obj = obj@DRAW_SUSPENDED_LOAD(logger, varargin{:});
%             param = struct(varargin{:});
% 
%             obj.replanner_ref = [];
%             if isfield(param, 'replanner')
%                 obj.replanner_ref = param.replanner;
%             elseif isfield(param, 'self') && isprop(param.self, 'reference')
%                 try
%                     func_list = param.self.reference.func;
%                     if iscell(func_list)
%                         for i = 1:length(func_list)
%                             if contains(class(func_list{i}), 'MELLINGER') || ...
%                                contains(class(func_list{i}), 'SWING')
%                                 obj.replanner_ref = func_list{i};
%                                 break;
%                             end
%                         end
%                     end
%                 catch
%                 end
%             end
%         end
% 
%         function draw(obj, varargin)
%             % 1. 基底クラスの通常描画
%             draw@DRAW_SUSPENDED_LOAD(obj, varargin{:});
% 
%             % 2. 座標取得
%             if nargin >= 6 && isnumeric(varargin{1})
%                 pQ = varargin{2}(:)';
%                 pL = varargin{5}(:)';
%             else
%                 if isstruct(varargin{1}) && isfield(varargin{1}, 't')
%                     time = varargin{1};
%                     k = round(time.t / time.dt) + 1;
%                 else
%                     k = obj.logger.k;
%                 end
%                 if k > obj.logger.k, k = obj.logger.k; end
%                 if k < 1, k = 1; end
%                 pL = obj.logger.data(k, "pL")';
%                 pQ = obj.logger.data(k, "p")';
%             end
% 
%             % 3. 障害物パラメータの取得
%             if ~isempty(obj.replanner_ref) && isprop(obj.replanner_ref, 'obs_radii')
%                 c = obj.replanner_ref.obs_center;
%                 radii = obj.replanner_ref.obs_radii;
%                 R_obs = obj.replanner_ref.obs_R;
%                 d_marg = obj.replanner_ref.obs_margin;
%                 r_load = obj.replanner_ref.r_load;
%                 r_drone = obj.replanner_ref.r_drone;
%             else
%                 try
%                     obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                     c = obs_list(1).p_center;
%                     radii = obs_list(1).ellipsoid_radii;
%                     R_obs = obs_list(1).R_obs;
%                     d_marg = obs_list(1).d_margin;
%                     r_load = 0.15;
%                     r_drone = 0.30;
%                 catch
%                     return;
%                 end
%             end
% 
%             hold(obj.ax, 'on');
% 
%             % =============================================================
%             % 【最重要】前フレームの動的バブル（水色球）をタグ検索で完全一括削除
%             % =============================================================
%             delete(findobj(obj.ax, 'Tag', 'TAG_CORRIDOR_DYNAMIC_BUBBLE'));
% 
%             % 静的障害物の描画（存在しない初回のみ描画して固定）
%             h_static = findobj(obj.ax, 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
%             if isempty(h_static)
%                 [sp_x, sp_y, sp_z] = sphere(25);
%                 [X_hard, Y_hard, Z_hard] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii, R_obs, c);
%                 [X_soft, Y_soft, Z_soft] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii + d_marg, R_obs, c);
% 
%                 % A. 原形コア (赤)
%                 obj.create_core_geometry(c, R_obs, radii);
% 
%                 % B. ハード楕円体 (オレンジ)
%                 surf(obj.ax, X_hard, Y_hard, Z_hard, ...
%                     'FaceColor', [1.0 0.5 0.0], 'FaceAlpha', 0.20, ...
%                     'EdgeColor', 'none', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
% 
%                 % C. ソフトマージン帯 (黄)
%                 mesh(obj.ax, X_soft, Y_soft, Z_soft, ...
%                     'EdgeColor', [0.9 0.8 0.1], 'FaceColor', [1.0 0.9 0.2], ...
%                     'FaceAlpha', 0.05, 'LineStyle', ':', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
%             end
% 
%             % =============================================================
%             % 現在フレームの水色バブルを描画（タグを付与）
%             % =============================================================
%             [sp_x, sp_y, sp_z] = sphere(16);
% 
%             % 荷物保護バブル (水色)
%             surf(obj.ax, ...
%                 sp_x * r_load + pL(1), sp_y * r_load + pL(2), sp_z * r_load + pL(3), ...
%                 'FaceColor', [0.0 0.8 1.0], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
%                 'Tag', 'TAG_CORRIDOR_DYNAMIC_BUBBLE');
% 
%             % 機体保護バブル (水色)
%             surf(obj.ax, ...
%                 sp_x * r_drone + pQ(1), sp_y * r_drone + pQ(2), sp_z * r_drone + pQ(3), ...
%                 'FaceColor', [0.0 0.8 1.0], 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
%                 'Tag', 'TAG_CORRIDOR_DYNAMIC_BUBBLE');
%         end
%     end
% 
%     methods (Access = private)
%         function [X, Y, Z] = generate_ellipsoid_pts(~, sp_x, sp_y, sp_z, r_vec, R_obs, c)
%             X_l = sp_x * r_vec(1);
%             Y_l = sp_y * r_vec(2);
%             Z_l = sp_z * r_vec(3);
%             pts = R_obs * [X_l(:)'; Y_l(:)'; Z_l(:)'];
%             X = reshape(pts(1, :), size(sp_x)) + c(1);
%             Y = reshape(pts(2, :), size(sp_y)) + c(2);
%             Z = reshape(pts(3, :), size(sp_z)) + c(3);
%         end
% 
%         function h = create_core_geometry(obj, c, R, ~)
%             try
%                 obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
%                 type = obs_list(1).type;
%                 raw  = obs_list(1).raw_param;
%             catch
%                 type = 'cylinder';
%                 raw  = [0.5, 0.5];
%             end
% 
%             switch lower(type)
%                 case 'cylinder'
%                     r = raw(1); h_len = raw(2);
%                     [cx, cy, cz] = cylinder(r, 20);
%                     cz = (cz - 0.5) * h_len;
%                     pts = R * [cx(:)'; cy(:)'; cz(:)'];
%                     X = reshape(pts(1, :), size(cx)) + c(1);
%                     Y = reshape(pts(2, :), size(cy)) + c(2);
%                     Z = reshape(pts(3, :), size(cz)) + c(3);
%                     h = surf(obj.ax, X, Y, Z, 'FaceColor', [0.85 0.15 0.15], ...
%                         'FaceAlpha', 1.0, 'EdgeColor', 'none', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
%                 otherwise
%                     [sx, sy, sz] = sphere(20);
%                     h = surf(obj.ax, sx*0.4+c(1), sy*0.4+c(2), sz*0.4+c(3), ...
%                         'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 1.0, 'EdgeColor', 'none', ...
%                         'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
%             end
%         end
%     end
% end

classdef DRAW_SUSPENDED_LOAD_CORRIDORV2 < DRAW_SUSPENDED_LOAD
    % DRAW_SUSPENDED_LOAD_CORRIDORV2
    % 全障害物一元描画・計画軌道オーバーレイ・動的保護バブル描画
    
    properties
        replanner_ref
    end
    
    methods
        function obj = DRAW_SUSPENDED_LOAD_CORRIDORV2(logger, varargin)
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
                            if contains(class(func_list{i}), 'MELLINGER') || ...
                               contains(class(func_list{i}), 'SWING')
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
            draw@DRAW_SUSPENDED_LOAD(obj, varargin{:});
            
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
            
            r_load = 0.15;
            r_drone = 0.30;
            if ~isempty(obj.replanner_ref)
                if isprop(obj.replanner_ref, 'r_load'),  r_load  = obj.replanner_ref.r_load;  end
                if isprop(obj.replanner_ref, 'r_drone'), r_drone = obj.replanner_ref.r_drone; end
            end
            
            hold(obj.ax, 'on');
            
            % 前フレームの動的バブルを消去
            delete(findobj(obj.ax, 'Tag', 'TAG_CORRIDOR_DYNAMIC_BUBBLE'));
            
            % 全静的障害物の描画 (初回のみ生成)
            h_static = findobj(obj.ax, 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
            if isempty(h_static)
                obs_list = [];
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch
                end
                
                [sp_x, sp_y, sp_z] = sphere(25);
                for i = 1:length(obs_list)
                    tgt = obs_list(i);
                    c = tgt.p_center;
                    R_obs = tgt.R_obs;
                    radii = tgt.ellipsoid_radii;
                    d_marg = tgt.d_margin;
                    
                    [X_hard, Y_hard, Z_hard] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii, R_obs, c);
                    [X_soft, Y_soft, Z_soft] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii + d_marg, R_obs, c);
                    
                    % 原形コア (赤)
                    obj.create_core_geometry(tgt);
                    
                    % ハード外接楕円体 (オレンジ)
                    surf(obj.ax, X_hard, Y_hard, Z_hard, ...
                        'FaceColor', [1.0 0.5 0.0], 'FaceAlpha', 0.20, ...
                        'EdgeColor', 'none', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
                    
                    % ソフトマージン帯 (黄メッシュ)
                    mesh(obj.ax, X_soft, Y_soft, Z_soft, ...
                        'EdgeColor', [0.9 0.8 0.1], 'FaceColor', [1.0 0.9 0.2], ...
                        'FaceAlpha', 0.05, 'LineStyle', ':', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
                end
            end
            
            % 計画軌道オーバーレイ描画
            h_traj = findobj(obj.ax, 'Tag', 'TAG_CORRIDOR_PLANNED_TRAJ');
            if isempty(h_traj) && ~isempty(obj.replanner_ref) && obj.replanner_ref.replan_active
                obj.draw_planned_trajectories();
            end
            
            % 動的保護バブル (シアン)
            [sp_x, sp_y, sp_z] = sphere(16);
            surf(obj.ax, ...
                sp_x * r_load + pL(1), sp_y * r_load + pL(2), sp_z * r_load + pL(3), ...
                'FaceColor', [0.0 0.8 1.0], 'FaceAlpha', 0.35, 'EdgeColor', 'none', ...
                'Tag', 'TAG_CORRIDOR_DYNAMIC_BUBBLE');
            surf(obj.ax, ...
                sp_x * r_drone + pQ(1), sp_y * r_drone + pQ(2), sp_z * r_drone + pQ(3), ...
                'FaceColor', [0.0 0.8 1.0], 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
                'Tag', 'TAG_CORRIDOR_DYNAMIC_BUBBLE');
        end
    end
    
    methods (Access = private)
        function draw_planned_trajectories(obj)
            rep = obj.replanner_ref;
            N_pts = 100;
            taus = linspace(0, rep.t_duration, N_pts);
            pL_traj = zeros(3, N_pts);
            pQ_traj = zeros(3, N_pts);
            
            g_vec = [0; 0; rep.gravity];
            for i = 1:N_pts
                t_eval = rep.t_start + taus(i);
                nom_res = rep.base_ref.do(struct('t', t_eval, 'dt', 0.025), 'f');
                xd_nom = nom_res.state.xd;
                if length(xd_nom) < 28, xd_nom = [xd_nom; zeros(28 - length(xd_nom), 1)]; end
                
                xd_opt = rep.evaluate_smooth_trajectory(taus(i), xd_nom);
                pL_traj(:, i) = xd_opt(1:3);
                
                aL = xd_opt(9:11);
                thrust_dir = (aL + g_vec) / norm(aL + g_vec);
                pQ_traj(:, i) = pL_traj(:, i) + rep.L_cable * thrust_dir;
            end
            
            plot3(obj.ax, pL_traj(1, :), pL_traj(2, :), pL_traj(3, :), ...
                'Color', [0.1 0.8 0.2], 'LineWidth', 2.0, 'Tag', 'TAG_CORRIDOR_PLANNED_TRAJ');
            plot3(obj.ax, pQ_traj(1, :), pQ_traj(2, :), pQ_traj(3, :), ...
                'Color', [0.6 0.2 0.8], 'LineWidth', 1.8, 'LineStyle', '--', 'Tag', 'TAG_CORRIDOR_PLANNED_TRAJ');
        end
        
        function [X, Y, Z] = generate_ellipsoid_pts(~, sp_x, sp_y, sp_z, r_vec, R_obs, c)
            X_l = sp_x * r_vec(1);
            Y_l = sp_y * r_vec(2);
            Z_l = sp_z * r_vec(3);
            pts = R_obs * [X_l(:)'; Y_l(:)'; Z_l(:)'];
            X = reshape(pts(1, :), size(sp_x)) + c(1);
            Y = reshape(pts(2, :), size(sp_y)) + c(2);
            Z = reshape(pts(3, :), size(sp_z)) + c(3);
        end
        
        function h = create_core_geometry(obj, tgt)
            c = tgt.p_center;
            R = tgt.R_obs;
            type = tgt.type;
            raw  = tgt.raw_param;
            
            switch lower(type)
                case 'cylinder'
                    r = raw(1); h_len = raw(2);
                    [cx, cy, cz] = cylinder(r, 20);
                    cz = (cz - 0.5) * h_len;
                    pts = R * [cx(:)'; cy(:)'; cz(:)'];
                    X = reshape(pts(1, :), size(cx)) + c(1);
                    Y = reshape(pts(2, :), size(cy)) + c(2);
                    Z = reshape(pts(3, :), size(cz)) + c(3);
                    h = surf(obj.ax, X, Y, Z, 'FaceColor', [0.85 0.15 0.15], ...
                        'FaceAlpha', 1.0, 'EdgeColor', 'none', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
                case 'sphere'
                    r = raw;
                    [sx, sy, sz] = sphere(20);
                    h = surf(obj.ax, sx*r + c(1), sy*r + c(2), sz*r + c(3), ...
                        'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 1.0, 'EdgeColor', 'none', ...
                        'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
                case 'box'
                    dx = raw(1)/2; dy = raw(2)/2; dz = raw(3)/2;
                    vertices = [-dx -dy -dz; dx -dy -dz; dx dy -dz; -dx dy -dz; ...
                                -dx -dy  dz; dx -dy  dz; dx dy  dz; -dx dy  dz];
                    pts = (R * vertices')' + c';
                    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                    h = patch(obj.ax, 'Vertices', pts, 'Faces', faces, ...
                        'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 1.0, ...
                        'EdgeColor', 'none', 'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
                otherwise
                    [sx, sy, sz] = sphere(20);
                    h = surf(obj.ax, sx*0.4+c(1), sy*0.4+c(2), sz*0.4+c(3), ...
                        'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 1.0, 'EdgeColor', 'none', ...
                        'Tag', 'TAG_CORRIDOR_STATIC_OBSTACLE');
            end
        end
    end
end