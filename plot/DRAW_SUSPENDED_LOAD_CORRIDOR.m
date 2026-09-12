classdef DRAW_SUSPENDED_LOAD_CORRIDOR < DRAW_SUSPENDED_LOAD
    % DRAW_SUSPENDED_LOAD_Cw(obj, t, k)ORRIDOR 障害物と安全回廊(Corridor)の壁を描画するクラス
    
    properties
        replanner_ref % リプランナの参照を保持
        h_obs         % 障害物のグラフィックハンドル
        h_wall        % 壁のグラフィックハンドル
        setup_done = false
    end
    
    methods
        function obj = DRAW_SUSPENDED_LOAD_CORRIDOR(logger, varargin)
            % 親クラスのコンストラクタ呼び出し
            obj = obj@DRAW_SUSPENDED_LOAD(logger, varargin{:});
            
            param = struct(varargin{:});
            
            % リプランナの検索
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
                disp('==== [DEBUG] DRAW_SUSPENDED_LOAD_CORRIDOR: Replanner linked ====');
            else
                disp('==== [DEBUG] DRAW_SUSPENDED_LOAD_CORRIDOR: Replanner NOT FOUND ====');
            end
        end
        
        function draw(obj, varargin)
            % 親クラスの描画（機体、荷物、紐など）を呼び出す
            draw@DRAW_SUSPENDED_LOAD(obj, varargin{:});
            
            % リプランナが保持している障害物情報を毎フレーム描画
            if ~isempty(obj.replanner_ref)
                % 障害物の描画
                if isprop(obj.replanner_ref, 'obs_center') && isprop(obj.replanner_ref, 'obs_radius')
                    c = obj.replanner_ref.obs_center;
                    r = obj.replanner_ref.obs_radius;
                    if r > 0.01
                        if ~obj.setup_done
                            hold(obj.ax, 'on');
                            [sp_x, sp_y, sp_z] = sphere(20);
                            obj.h_obs = surf(obj.ax, sp_x*r + c(1), sp_y*r + c(2), sp_z*r + c(3), ...
                                'FaceColor', [1.0 0.2 0.2], 'FaceAlpha', 0.8, 'EdgeColor', 'none', 'Tag', 'obs_sphere');
                            obj.h_wall = patch(obj.ax, 'XData', [], 'YData', [], 'ZData', [], ...
                                      'FaceColor', [1.0 0.8 0.2], 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'Tag', 'obs_wall');
                            obj.setup_done = true;
                        else
                            % 更新
                            [sp_x, sp_y, sp_z] = sphere(20);
                            set(obj.h_obs, 'XData', sp_x*r + c(1), 'YData', sp_y*r + c(2), 'ZData', sp_z*r + c(3));
                        end
                    end
                end
                
                % 壁(Corridor)の描画
                if obj.setup_done && isprop(obj.replanner_ref, 'hyperplane_n')
                    n_vec = obj.replanner_ref.hyperplane_n;
                    p_vec = obj.replanner_ref.hyperplane_p;
                    if norm(n_vec) > 0.1
                        if abs(n_vec(3)) < 0.9
                            u_vec = cross(n_vec, [0;0;1]);
                        else
                            u_vec = cross(n_vec, [1;0;0]);
                        end
                        u_vec = u_vec / norm(u_vec);
                        v_vec = cross(n_vec, u_vec);
                        
                        plane_size = 4.0;
                        p1 = p_vec + plane_size * u_vec + plane_size * v_vec;
                        p2 = p_vec - plane_size * u_vec + plane_size * v_vec;
                        p3 = p_vec - plane_size * u_vec - plane_size * v_vec;
                        p4 = p_vec + plane_size * u_vec - plane_size * v_vec;
                        
                        set(obj.h_wall, 'XData', [p1(1) p2(1) p3(1) p4(1)], ...
                                        'YData', [p1(2) p2(2) p3(2) p4(2)], ...
                                        'ZData', [p1(3) p2(3) p3(3) p4(3)]);
                    end
                end
            end
        end
    end
end

