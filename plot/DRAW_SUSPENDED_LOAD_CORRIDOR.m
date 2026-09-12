classdef DRAW_SUSPENDED_LOAD_CORRIDOR < DRAW_SUSPENDED_LOAD
    % DRAW_SUSPENDED_LOAD_CORRIDOR
    % Mellinger回廊制約の多層境界（本体・ハード境界・マージン・超平面）を完全可視化
    
    properties
        replanner_ref
        h_obs_body      % 障害物本体 (赤)
        h_obs_hard      % ハード境界: 本体 + 紐・機体防護 (橙・半透明)
        h_obs_soft      % ソフト境界: ハード + 安全マージン (黄・ワイヤー)
        h_corridor_wall % QP安全壁平面 (シアン・半透明)
        setup_done = false
    end
    
    methods
        function obj = DRAW_SUSPENDED_LOAD_CORRIDOR(logger, varargin)
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
                            if contains(class(func_list{i}), 'MELLINGER')
                                obj.replanner_ref = func_list{i};
                                break;
                            end
                        end
                    end
                catch
                end
            end
            
            if ~isempty(obj.replanner_ref)
                disp('==== [VISUALIZER] DRAW_SUSPENDED_LOAD_CORRIDOR: Replanner Linked ====');
            else
                disp('==== [VISUALIZER] DRAW_SUSPENDED_LOAD_CORRIDOR: Replanner NOT Found ====');
            end
        end
        
        function draw(obj, varargin)
            % 1. ドローン本体、荷物、紐の通常描画
            draw@DRAW_SUSPENDED_LOAD(obj, varargin{:});
            
            % 2. 回避空間・制約構造の描画
            if ~isempty(obj.replanner_ref) && isprop(obj.replanner_ref, 'obs_radius')
                r_body = obj.replanner_ref.obs_radius;
                
                if r_body > 0.01
                    c = obj.replanner_ref.obs_center;
                    d_marg = obj.replanner_ref.obs_margin;
                    L_cable = obj.replanner_ref.L_cable;
                    
                    % リプランナで計算されている厳密な半径定義
                    r_hard = r_body + L_cable * 0.4 + 0.15; % ハード境界
                    r_soft = r_hard + d_marg;               % ソフト境界 (マージン込)
                    
                    [sp_x, sp_y, sp_z] = sphere(30);
                    
                    if ~obj.setup_done
                        hold(obj.ax, 'on');
                        
                        % レイヤー1: 障害物本体 (Solid Red)
                        obj.h_obs_body = surf(obj.ax, ...
                            sp_x*r_body + c(1), sp_y*r_body + c(2), sp_z*r_body + c(3), ...
                            'FaceColor', [0.85 0.15 0.15], 'FaceAlpha', 0.9, ...
                            'EdgeColor', 'none', 'DisplayName', '障害物本体 (Core)');
                        
                        % レイヤー2: ハード制約境界 (本体 + 紐・ドローン防護領域, Orange)
                        obj.h_obs_hard = surf(obj.ax, ...
                            sp_x*r_hard + c(1), sp_y*r_hard + c(2), sp_z*r_hard + c(3), ...
                            'FaceColor', [1.0 0.5 0.0], 'FaceAlpha', 0.25, ...
                            'EdgeColor', 'none', 'DisplayName', 'ハード境界 (本体+紐防護)');
                        
                        % レイヤー3: 安全マージン境界 (ソフト目標面, Yellow Wire)
                        obj.h_obs_soft = mesh(obj.ax, ...
                            sp_x*r_soft + c(1), sp_y*r_soft + c(2), sp_z*r_soft + c(3), ...
                            'EdgeColor', [0.9 0.8 0.1], 'FaceColor', 'none', ...
                            'LineStyle', ':', 'DisplayName', '安全マージン外殻 (ソフト目標)');
                        
                        % レイヤー4: 回廊安全壁 (Corridor Hyperplane, Cyan)
                        obj.h_corridor_wall = patch(obj.ax, ...
                            'XData', [], 'YData', [], 'ZData', [], ...
                            'FaceColor', [0.0 0.8 0.9], 'FaceAlpha', 0.35, ...
                            'EdgeColor', [0.0 0.5 0.8], 'LineWidth', 1.5, ...
                            'DisplayName', '回廊安全壁 (Corridor Wall)');
                        
                        obj.setup_done = true;
                    else
                        % 球体ジオメトリの追従更新
                        set(obj.h_obs_body, 'XData', sp_x*r_body + c(1), 'YData', sp_y*r_body + c(2), 'ZData', sp_z*r_body + c(3));
                        set(obj.h_obs_hard, 'XData', sp_x*r_hard + c(1), 'YData', sp_y*r_hard + c(2), 'ZData', sp_z*r_hard + c(3));
                        set(obj.h_obs_soft, 'XData', sp_x*r_soft + c(1), 'YData', sp_y*r_soft + c(2), 'ZData', sp_z*r_soft + c(3));
                    end
                    
                    % リプランが発動している場合、QPで実際に切った安全壁（超平面）を更新
                    if obj.replanner_ref.replan_active || obj.replanner_ref.replan_done
                        p0 = obj.replanner_ref.result.state.p;
                        % 進行方向に直交する法線ベクトルを再構成
                        dir_nom = obj.replanner_ref.v_merge_vec;
                        if isempty(dir_nom) || norm(dir_nom) < 1e-3
                            dir_nom = [0; 0; 1];
                        else
                            dir_nom = dir_nom / norm(dir_nom);
                        end
                        
                        vec_to_obs = c - p0;
                        proj_on_line = dot(vec_to_obs, dir_nom) * dir_nom;
                        normal_vec = vec_to_obs - proj_on_line;
                        if norm(normal_vec) < 1e-3
                            if abs(dir_nom(3)) < 0.9, normal_vec = cross(dir_nom, [0; 0; 1]);
                            else, normal_vec = cross(dir_nom, [1; 0; 0]); end
                        end
                        dir_normal = normal_vec / norm(normal_vec);
                        
                        % 壁の位置（ハード境界の外周接平面）
                        p_wall_center = c + dir_normal * r_hard;
                        n_wall = -dir_normal;
                        
                        % 壁の接平面ベクトルを算出
                        if abs(n_wall(3)) < 0.9, u_w = cross(n_wall, [0; 0; 1]);
                        else, u_w = cross(n_wall, [1; 0; 0]); end
                        u_w = u_w / norm(u_w);
                        v_w = cross(n_wall, u_w);
                        
                        % 壁の矩形サイズ
                        w_size = 2.5;
                        pt1 = p_wall_center + w_size * u_w + w_size * v_w;
                        pt2 = p_wall_center - w_size * u_w + w_size * v_w;
                        pt3 = p_wall_center - w_size * u_w - w_size * v_w;
                        pt4 = p_wall_center + w_size * u_w - w_size * v_w;
                        
                        set(obj.h_corridor_wall, ...
                            'XData', [pt1(1) pt2(1) pt3(1) pt4(1)], ...
                            'YData', [pt1(2) pt2(2) pt3(2) pt4(2)], ...
                            'ZData', [pt1(3) pt2(3) pt3(3) pt4(3)]);
                    end
                end
            end
        end
    end
end