classdef DRAW_SUSPENDED_LOAD_MPC_CBF < DRAW_SUSPENDED_LOAD
    % DRAW_SUSPENDED_LOAD_MPC_CBF
    % - 動的・静的楕円体障害物のリアルタイムアニメーション可視化
    % - Zheng et al. (2025): 荷物〜ケーブル〜ドローン 5連保護球列の動的レンダリング
    
    properties
        replanner_ref
        logger_ref          
        sim_dt = 0.025
        anim_k = 1          
        default_obs_mode = 2
    end
    
    methods (Access = public)
        function obj = DRAW_SUSPENDED_LOAD_MPC_CBF(logger, varargin)
            obj = obj@DRAW_SUSPENDED_LOAD(logger, varargin{:});
            obj.logger_ref = logger;
            obj.anim_k = 1;
            
            param = struct(varargin{:});
            obj.replanner_ref = [];
            if isfield(param, 'replanner')
                obj.replanner_ref = param.replanner;
            elseif isfield(param, 'self') && isprop(param.self, 'reference')
                try
                    if isprop(param.self.reference, 'base_function') && isprop(param.self.reference.base_function, 'function_class_list')
                        func_list = param.self.reference.base_function.function_class_list;
                        for i = 1:length(func_list)
                            if contains(class(func_list{i}), 'CBF') || contains(class(func_list{i}), 'MPC')
                                obj.replanner_ref = func_list{i};
                                break;
                            end
                        end
                    end
                catch
                end
            end
            if isfield(param, 'obs_mode'), obj.default_obs_mode = param.obs_mode; end
        end
        
        function draw(obj, varargin)
            draw@DRAW_SUSPENDED_LOAD(obj, varargin{:});
            
            sim_time_t = 0.0;
            pQ = [0, 0, 2];
            pL = [0, 0, 0];
            
            if nargin >= 6 && isnumeric(varargin{2}) && isnumeric(varargin{5})
                pQ = varargin{2}(:)';
                pL = varargin{5}(:)';
                if isempty(obj.ax.UserData) || ~isnumeric(obj.ax.UserData)
                    obj.ax.UserData = 1;
                end
                k_current = obj.ax.UserData;
                try
                    sim_time_t = obj.logger_ref.data(k_current, 't');
                catch
                    sim_time_t = (k_current - 1) * obj.sim_dt;
                end
                obj.ax.UserData = k_current + 1;
                if ~isempty(obj.logger_ref) && isprop(obj.logger_ref, 'k')
                    if obj.ax.UserData > obj.logger_ref.k, obj.ax.UserData = 1; end
                end
            elseif nargin >= 2 && isstruct(varargin{1}) && isfield(varargin{1}, 't')
                sim_time_t = varargin{1}.t;
                if isfield(varargin{1}, 'dt'), obj.sim_dt = varargin{1}.dt; end
                try
                    pL = obj.self.estimator.result.state.pL';
                    pQ = obj.self.estimator.result.state.p';
                catch
                end
            end
            
            r_load = 0.15; r_drone = 0.30; L_cable = 2.0;
            if ~isempty(obj.replanner_ref)
                if isprop(obj.replanner_ref, 'r_load'),  r_load  = obj.replanner_ref.r_load;  end
                if isprop(obj.replanner_ref, 'r_drone'), r_drone = obj.replanner_ref.r_drone; end
                if isprop(obj.replanner_ref, 'L_cable'), L_cable = obj.replanner_ref.L_cable; end
            end
            
            hold(obj.ax, 'on');
            delete(findobj(obj.ax, 'Tag', 'TAG_CBF_DYNAMIC_SPHERES'));
            delete(findobj(obj.ax, 'Tag', 'TAG_CBF_OBSTACLE'));
            delete(findobj(obj.ax, 'Tag', 'TAG_MPC_PREDICTED_TRAJ'));
            
            obj.render_obstacles_at_time(sim_time_t);
            
            num_spheres = 5;
            lambdas = linspace(0, 1, num_spheres);
            [sp_x, sp_y, sp_z] = sphere(14);
            for j = 1:num_spheres
                lam = lambdas(j);
                p_sphere = (1 - lam) * pL + lam * pQ;
                r_sphere = (1 - lam) * r_load + lam * r_drone;
                surf(obj.ax, ...
                    sp_x * r_sphere + p_sphere(1), sp_y * r_sphere + p_sphere(2), sp_z * r_sphere + p_sphere(3), ...
                    'FaceColor', [0.0 0.75 0.95], 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_DYNAMIC_SPHERES');
            end
            
            P_pred = [];
            if ~isempty(obj.replanner_ref) && isprop(obj.replanner_ref, 'p_pred_cache')
                P_pred = obj.replanner_ref.p_pred_cache;
            end
            if ~isempty(P_pred) && size(P_pred, 2) >= 2
                plot3(obj.ax, P_pred(1, :), P_pred(2, :), P_pred(3, :), '-', 'Color', [0.0 1.0 0.3], 'LineWidth', 3.0, 'Tag', 'TAG_MPC_PREDICTED_TRAJ');
                plot3(obj.ax, P_pred(1, :), P_pred(2, :), P_pred(3, :), 'o', 'MarkerEdgeColor', [1.0 0.3 0.0], 'MarkerFaceColor', [1.0 0.9 0.0], 'MarkerSize', 5.0, 'Tag', 'TAG_MPC_PREDICTED_TRAJ');
                P_pred_Q = P_pred + [0; 0; L_cable];
                plot3(obj.ax, P_pred_Q(1, :), P_pred_Q(2, :), P_pred_Q(3, :), '--', 'Color', [1.0 0.1 0.8], 'LineWidth', 1.8, 'Tag', 'TAG_MPC_PREDICTED_TRAJ');
                plot3(obj.ax, P_pred(1, end), P_pred(2, end), P_pred(3, end), 'p', 'Color', [1.0 0.2 0.0], 'MarkerSize', 11, 'MarkerFaceColor', [1.0 0.8 0.0], 'Tag', 'TAG_MPC_PREDICTED_TRAJ');
            end
        end
    end
    
    methods (Access = private)
        function render_obstacles_at_time(obj, t_now)
            obs_list = [];
            try
                obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t_now);
            catch
                try
                    obs_list = ENVIRONMENT_OBSTACLE_ELLIPSE();
                catch
                end
            end
            
            if isempty(obs_list), return; end
            
            [sp_x, sp_y, sp_z] = sphere(20);
            for i = 1:length(obs_list)
                tgt = obs_list(i);
                c = tgt.p_center;
                if isfield(tgt, 'R_obs') && ~isempty(tgt.R_obs), R_obs = tgt.R_obs;
                elseif isprop(tgt, 'R_obs') && ~isempty(tgt.R_obs), R_obs = tgt.R_obs;
                else, R_obs = eye(3); end
                
                radii = tgt.ellipsoid_radii;
                d_marg = tgt.d_margin;
                
                [X_hard, Y_hard, Z_hard] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii, R_obs, c);
                [X_soft, Y_soft, Z_soft] = obj.generate_ellipsoid_pts(sp_x, sp_y, sp_z, radii + d_marg, R_obs, c);
                
                obj.create_core_geometry(tgt);
                
                surf(obj.ax, X_hard, Y_hard, Z_hard, 'FaceColor', [1.0 0.5 0.0], 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_OBSTACLE');
                mesh(obj.ax, X_soft, Y_soft, Z_soft, 'EdgeColor', [0.9 0.8 0.1], 'FaceColor', [1.0 0.9 0.2], 'FaceAlpha', 0.04, 'LineStyle', ':', 'Tag', 'TAG_CBF_OBSTACLE');
            end
        end
        
        function [X, Y, Z] = generate_ellipsoid_pts(~, sp_x, sp_y, sp_z, r_vec, R_obs, c)
            X_l = sp_x * r_vec(1); Y_l = sp_y * r_vec(2); Z_l = sp_z * r_vec(3);
            pts = R_obs * [X_l(:)'; Y_l(:)'; Z_l(:)'];
            X = reshape(pts(1, :), size(sp_x)) + c(1);
            Y = reshape(pts(2, :), size(sp_y)) + c(2);
            Z = reshape(pts(3, :), size(sp_z)) + c(3);
        end
        
        function h = create_core_geometry(obj, tgt)
            c = tgt.p_center;
            if isfield(tgt, 'R_obs') && ~isempty(tgt.R_obs), R = tgt.R_obs;
            elseif isprop(tgt, 'R_obs') && ~isempty(tgt.R_obs), R = tgt.R_obs;
            else, R = eye(3); end
            
            type = tgt.type; raw = tgt.raw_param;
            
            switch lower(type)
                case 'cylinder'
                    r = raw(1); h_len = raw(2);
                    [cx, cy, cz] = cylinder(r, 20); cz = (cz - 0.5) * h_len;
                    pts = R * [cx(:)'; cy(:)'; cz(:)'];
                    X = reshape(pts(1, :), size(cx)) + c(1); Y = reshape(pts(2, :), size(cy)) + c(2); Z = reshape(pts(3, :), size(cz)) + c(3);
                    h = surf(obj.ax, X, Y, Z, 'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.95, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_OBSTACLE');
                case 'sphere'
                    r = raw; [sx, sy, sz] = sphere(20);
                    h = surf(obj.ax, sx * r + c(1), sy * r + c(2), sz * r + c(3), 'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.95, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_OBSTACLE');
                case 'box'
                    dx = raw(1)/2; dy = raw(2)/2; dz = raw(3)/2;
                    v_box = [-dx -dy -dz; dx -dy -dz; dx dy -dz; -dx dy -dz; -dx -dy dz; dx -dy dz; dx dy dz; -dx dy dz]';
                    v_world = R * v_box + c;
                    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                    h = patch(obj.ax, 'Vertices', v_world', 'Faces', faces, 'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.95, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_OBSTACLE');
                otherwise
                    [sx, sy, sz] = sphere(16); r_sph = max(tgt.ellipsoid_radii) * 0.7;
                    h = surf(obj.ax, sx * r_sph + c(1), sy * r_sph + c(2), sz * r_sph + c(3), 'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.95, 'EdgeColor', 'none', 'Tag', 'TAG_CBF_OBSTACLE');
            end
        end
    end
end
