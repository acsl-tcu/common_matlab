% classdef DRAW_DRONE_MOTION
%     % Animation class for drone motion (no load)
% 
%     properties
%         frame
%         thrust
%         ax
%         xlim
%         ylim
%         zlim
%         L
%         frame_size = [0.1170, 0.0932];
%         rotor_r = 0.0392;
%     end
% 
%     methods
%         function obj = DRAW_DRONE_MOTION(logger, varargin)
%             param = struct(varargin{:});
%             if ~isfield(param, "target")
%                 param.target = 1;
%             end
% 
%             if isfield(param, "self") && isprop(param.self, "parameter")
%                 if isprop(param.self.parameter, "Lx") && isprop(param.self.parameter, "Ly")
%                     obj.frame_size = [param.self.parameter.Lx, param.self.parameter.Ly];
%                 end
%                 if isprop(param.self.parameter, "rotor_r")
%                     obj.rotor_r = param.self.parameter.rotor_r;
%                 end
%             end
% 
%             p = obj.data_format(logger, param.target, "p", "p");
%             r = [];
%             try
%                 r = obj.data_format(logger, param.target, "p", "r");
%             catch
%                 r = [];
%             end
% 
%             data = p;
%             if ~isempty(r)
%                 data = [data; r];
%             end
% 
%             tM = max(data, [], 1);
%             tm = min(data, [], 1);
%             M = [max(tM(1:3:end)), max(tM(2:3:end)), max(tM(3:3:end))];
%             m = [min(tm(1:3:end)), min(tm(2:3:end)), min(tm(3:3:end))];
% 
%             if isfield(param, "frame_size")
%                 obj.L = param.frame_size;
%             else
%                 obj.L = obj.frame_size;
%             end
% 
%             obj.xlim = [m(1)-obj.L(1) M(1)+obj.L(1)];
%             obj.ylim = [m(2)-obj.L(2) M(2)+obj.L(2)];
%             obj.zlim = [min(0, m(3)-1) M(3)+1];
%             if isfield(param, "lims")
%                 obj.xlim = param.lims(1,:);
%                 obj.ylim = param.lims(2,:);
%                 obj.zlim = param.lims(3,:);
%             end
% 
%             if isfield(param, "ax")
%                 ax = param.ax;
%             else
%                 figure();
%                 ax = axes('XLim', obj.xlim, 'YLim', obj.ylim, 'ZLim', obj.zlim);
%                 if ~obj.has_key(varargin, "ax")
%                     varargin = [varargin(:)', {'ax'}, {ax}];
%                 end
%             end
%             if obj.has_key(varargin, "target")
%                 obj = obj.gen_frame(varargin{:});
%             else
%                 obj = obj.gen_frame("target", param.target, varargin{:});
%             end
% 
%             view(ax, 3)
%             grid(ax, 'on')
%             daspect(ax, [1 1 1]);
%         end
% 
%         function obj = gen_frame(obj, varargin)
%             param = struct(varargin{:});
%             obj.ax = param.ax;
%             xlabel(obj.ax, "x [m]");
%             ylabel(obj.ax, "y [m]");
%             zlabel(obj.ax, "z [m]");
%             hold(obj.ax, "on");
% 
%             if isfield(param, "rotor_r")
%                 r = param.rotor_r;
%             else
%                 r = obj.rotor_r;
%             end
%             [xr, yr, zr] = cylinder(obj.ax, [0 r]);
%             zr = 0.001*zr;
% 
%             [xb, yb, zb] = ellipsoid(obj.ax, 0, 0, 0, 1.2*obj.L(1)/4, 0.8*obj.L(2)/4, 0.02);
%             d = obj.L/2;
%             rp = [d(1), d(2), 0.02; -d(1), d(2), 0.02; d(1), -d(2), 0.02; -d(1), -d(2), 0.02];
%             c = ["red", "green", "blue", "cyan", '#4DBEEE'];
% 
%             [x, y, z] = cylinder(obj.ax, 0.01);
%             z = z*vecnorm(d)*2;
%             R1 = rotmat(quaternion([obj.L(2), -obj.L(1), 0]*pi/(vecnorm(obj.L)*2), "rotvec"), 'point');
%             R2 = rotmat(quaternion([obj.L(2), obj.L(1), 0]*pi/(vecnorm(obj.L)*2), "rotvec"), 'point');
%             F1 = [R1*[x(1,:);y(1,:);z(1,:)];R1*[x(2,:);y(2,:);z(2,:)]]+[d(1);d(2);0;d(1);d(2);0];
%             F2 = [R2*[x(1,:);y(1,:);z(1,:)];R2*[x(2,:);y(2,:);z(2,:)]]+[-d(1);d(2);0;-d(1);d(2);0];
% 
%             for n = param.target
%                 for i = 4:-1:1
%                     h(n,i) = surface(obj.ax, xr+rp(i,1), yr+rp(i,2), zr+rp(i,3), 'FaceColor', c(i));
%                     T(n,i) = quiver3(obj.ax, rp(i,1), rp(i,2), rp(i,3), 0, 0, 1, 'FaceColor', c(i));
%                     tt(n,i) = hgtransform('Parent', obj.ax); set(T(n,i), 'Parent', tt(n,i));
%                 end
%                 h(n,5) = surface(obj.ax, [F1(1,:);F1(4,:);NaN(1,size(F1,2));F2(1,:);F2(4,:)], ...
%                     [F1(2,:);F1(5,:);NaN(1,size(F1,2));F2(2,:);F2(5,:)], ...
%                     [F1(3,:);F1(6,:);NaN(1,size(F1,2));F2(3,:);F2(6,:)], 'FaceColor', c(5));
%                 h(n,6) = surface(obj.ax, xb, yb, zb, 'FaceColor', c(5));
% 
%                 t(n) = hgtransform('Parent', obj.ax); set(h(n,:), 'Parent', t(n));
%             end
%             obj.frame = t;
%             obj.thrust = tt;
%         end
% 
%         function draw(obj, target, p, q, u)
%             arguments
%                 obj
%                 target
%                 p
%                 q
%                 u = [1;1;1;1]
%             end
% 
%             for n = target
%                 frame = obj.frame(n);
%                 thrust = obj.thrust(n,:);
%                 R = makehgtform('axisrotate', q(1,1:3,n), q(1,4,n));
%                 Txyz = makehgtform('translate', p(1,:,n));
%                 for i = 1:4
%                     if u(1,i,n) > 0
%                         S = makehgtform('scale', [1,1,u(1,i,n)]);
%                     elseif u(i) < 0
%                         S1 = makehgtform('xrotate', pi);
%                         S = makehgtform('scale', [1,1,-u(1,i,n)])*S1;
%                     else
%                         S = eye(4);
%                         S(3,3) = 1e-5;
%                     end
%                     set(thrust(i), 'Matrix', Txyz*R*S);
%                 end
%                 set(frame, 'Matrix', Txyz*R);
%             end
%             drawnow
%         end
% 
%         function animation(obj, logger, varargin)
%             param = struct(varargin{:});
%             if ~isfield(param, "target")
%                 param.target = 1;
%             end
% 
%             p = obj.data_format(logger, param.target, "p", "p");
%             q = obj.data_format(logger, param.target, "q", "p");
%             u = logger.data(param.target, "input", "");
%             u = reshape(u, size(u,1), size(u,2), length(param.target));
%             Q = obj.gen_Q(param.target, q);
% 
%             r = [];
%             try
%                 r = obj.data_format(logger, param.target, "p", "r");
%             catch
%                 r = [];
%             end
%             if ~isempty(r)
%                 for n = 1:size(r, 3)
%                     plot3(obj.ax, r(:,1,n), r(:,2,n), r(:,3,n), 'r');
%                 end
%             end
% 
%             if isfield(param, "gif")
%                 sizen = 256;
%                 delaytime = 0;
%                 filename = strrep(strrep(strcat('Data/Movie(', string(datetime('now')), ').gif'), ':', '_'), ' ', '_');
%             end
%             if isfield(param, "mp4")
%                 sizen = 256;
%                 delaytime = 0;
%                 filename = strrep(strrep(strcat('Data/Movie(', string(datetime('now')), ').mp4'), ':', '_'), ' ', '_');
%                 v = VideoWriter(filename, "MPEG-4");
%                 if param.mp4
%                     open(v);
%                     writeAnimation(v);
%                 end
%             end
% 
%             t = logger.data(0, "t", "");
%             phase = [];
%             try
%                 phase = logger.data(0, "phase", "");
%             catch
%                 phase = [];
%             end
%             tRealtime = tic;
%             for i = 1:length(t)-1
%                 if isfield(param, "opt_plot")
%                     param.self.show(param.opt_plot, "logger", logger, "k", i, varargin{:});
%                 end
%                 if ~isvalid(obj.frame)
%                     obj = obj.gen_frame("target", param.target, "ax", obj.ax);
%                 end
%                 obj.draw(param.target, p(i,:,param.target), Q(i,:,param.target), u(i,:,param.target));
%                 timeText = sprintf("%05.2f", t(i));
%                 phaseChar = "";
%                 if ~isempty(phase)
%                     phaseChar = char(phase(i));
%                 end
%                 title(obj.ax, "time : " + timeText + "  phase : " + phaseChar);
%                 if isfield(param, "realtime")
%                     delta = toc(tRealtime);
%                     if t(i+1)-t(i) > delta
%                         pause(t(i+1)-t(i) - delta);
%                     end
%                     tRealtime = tic;
%                 else
%                     pause(0.01);
%                 end
%                 if isfield(param, "lims")
%                     obj.xlim = param.lims(1,:);
%                     obj.ylim = param.lims(2,:);
%                     obj.zlim = param.lims(3,:);
%                     obj.ax.XLimMode = "manual";
%                     obj.ax.YLimMode = "manual";
%                     obj.ax.ZLimMode = "manual";
%                     obj.ax.XLim = obj.xlim;
%                     obj.ax.YLim = obj.ylim;
%                     obj.ax.ZLim = obj.zlim;
%                 end
%                 if isfield(param, "gif")
%                     im = frame2im(getframe(obj.ax));
%                     [imind, cm] = rgb2ind(im, sizen);
%                     if i == 1
%                         imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delaytime);
%                     else
%                         imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delaytime);
%                     end
%                 end
%                 if isfield(param, "mp4")
%                     framev = getframe(obj.ax);
%                     writeVideo(v, framev);
%                 end
%             end
%             if isfield(param, "mp4")
%                 close(v);
%             end
%         end
%     end
% 
%     methods (Access = private)
%         function tf = has_key(~, args, key)
%             tf = false;
%             if isempty(args)
%                 return
%             end
%             if isstruct(args)
%                 tf = isfield(args, key);
%                 return
%             end
%             keys = string(args(1:2:end));
%             tf = any(keys == key);
%         end
% 
%         function p = data_format(~, logger, source, var, att)
%             q = logger.data(source, var, att);
%             p = reshape(q, size(q,1), size(q,2)/length(source), length(source));
%         end
% 
%         function Q = gen_Q(~, target, q)
%             Q = zeros(size(q,1), 4, length(target));
%             for n = 1:length(target)
%                 switch size(q(:,:,n),2)
%                     case 3
%                         Q1 = quaternion(Eul2Quat(q(:,:,n)')');
%                     case 4
%                         Q1 = quaternion(q(:,:,n));
%                     case 9
%                         Q1 = quaternion(q(:,:,n), 'rotmat', 'frame');
%                 end
%                 Q2 = rotvec(Q1);
%                 tmp = vecnorm(Q2, 2, 2);
%                 Q(tmp==0,:,n) = 0;
%                 Q(tmp==0,1,n) = 1;
%                 if sum(tmp~=0) ~= 0
%                     Q(tmp~=0,:,n) = [Q2(tmp~=0,:)./tmp(tmp~=0), tmp(tmp~=0)];
%                 end
%             end
%         end
%     end
% end

classdef DRAW_DRONE_MOTION_CBF
    % Animation class for drone motion with Obstacle & Safety Margin visualization
    properties
        frame
        thrust
        ax
        xlim
        ylim
        zlim
        L
        frame_size = [0.1170, 0.0932];
        rotor_r = 0.0392;
    end
    methods
        function obj = DRAW_DRONE_MOTION_CBF(logger, varargin)
            param = struct(varargin{:});
            if ~isfield(param, "target")
                param.target = 1;
            end
            if isfield(param, "self") && isprop(param.self, "parameter")
                if isprop(param.self.parameter, "Lx") && isprop(param.self.parameter, "Ly")
                    obj.frame_size = [param.self.parameter.Lx, param.self.parameter.Ly];
                end
                if isprop(param.self.parameter, "rotor_r")
                    obj.rotor_r = param.self.parameter.rotor_r;
                end
            end
            p = obj.data_format(logger, param.target, "p", "p");
            r = [];
            try
                r = obj.data_format(logger, param.target, "p", "r");
            catch
                r = [];
            end
            data = p;
            if ~isempty(r)
                data = [data; r];
            end
            tM = max(data, [], 1);
            tm = min(data, [], 1);
            M = [max(tM(1:3:end)), max(tM(2:3:end)), max(tM(3:3:end))];
            m = [min(tm(1:3:end)), min(tm(2:3:end)), min(tm(3:3:end))];
            if isfield(param, "frame_size")
                obj.L = param.frame_size;
            else
                obj.L = obj.frame_size;
            end
            obj.xlim = [m(1)-obj.L(1) M(1)+obj.L(1)];
            obj.ylim = [m(2)-obj.L(2) M(2)+obj.L(2)];
            obj.zlim = [min(0, m(3)-1) M(3)+1];
            if isfield(param, "lims")
                obj.xlim = param.lims(1,:);
                obj.ylim = param.lims(2,:);
                obj.zlim = param.lims(3,:);
            end
            if isfield(param, "ax")
                ax = param.ax;
            else
                figure();
                ax = axes('XLim', obj.xlim, 'YLim', obj.ylim, 'ZLim', obj.zlim);
                if ~obj.has_key(varargin, "ax")
                    varargin = [varargin(:)', {'ax'}, {ax}];
                end
            end
            if obj.has_key(varargin, "target")
                obj = obj.gen_frame(varargin{:});
            else
                obj = obj.gen_frame("target", param.target, varargin{:});
            end

            % --- 障害物および安全マージンの描画 ---
            obj.draw_obstacles();

            view(ax, 3)
            grid(ax, 'on')
            daspect(ax, [1 1 1]);
        end

        function draw_obstacles(obj)
            % ENVIRONMENT_OBSTACLE から情報を取得して描画
            if exist('ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY', 'file') == 2
                obs_list = ENVIRONMENT_OBSTACLE_HOCBF_LINK_XY();
                hold(obj.ax, 'on');
                for i = 1:length(obs_list)
                    obs = obs_list(i);
                    
                    % 1. 純粋な障害物の描画 (ソリッド表示)
                    switch lower(obs.type)
                        case 'cylinder'
                            r_cyl = obs.raw_param(1);
                            h_cyl = obs.raw_param(2);
                            [xc, yc, zc] = cylinder(r_cyl, 30);
                            zc = zc * h_cyl - (h_cyl/2) + obs.p_center(3);
                            xc = xc + obs.p_center(1);
                            yc = yc + obs.p_center(2);
                            surf(obj.ax, xc, yc, zc, 'FaceColor', [0.8 0.2 0.2], ...
                                 'EdgeColor', 'none', 'FaceAlpha', 0.9);
                            % 上下面のフタ
                            fill3(obj.ax, xc(1,:), yc(1,:), zc(1,:), [0.8 0.2 0.2]);
                            fill3(obj.ax, xc(2,:), yc(2,:), zc(2,:), [0.8 0.2 0.2]);

                        case 'sphere'
                            r_sp = obs.raw_param;
                            [xs, ys, zs] = sphere(30);
                            surf(obj.ax, xs*r_sp + obs.p_center(1), ...
                                         ys*r_sp + obs.p_center(2), ...
                                         zs*r_sp + obs.p_center(3), ...
                                 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.9);

                        case 'box'
                            dx = obs.raw_param(1); dy = obs.raw_param(2); dz = obs.raw_param(3);
                            % 簡易直方体描画
                            cx = obs.p_center(1); cy = obs.p_center(2); cz = obs.p_center(3);
                            plot3(obj.ax, cx, cy, cz, 'r.', 'MarkerSize', 20);

                        otherwise
                            % その他の形状は中心点をプロット
                            plot3(obj.ax, obs.p_center(1), obs.p_center(2), obs.p_center(3), ...
                                  'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
                    end

                    % 2. 安全マージン外接球の描画 (CBFが認識しているバリア境界)
                    if isfield(obs, 'p_obs') && isfield(obs, 'r_obs_margin')
                        [x_m, y_m, z_m] = sphere(30);
                        x_m = x_m * obs.r_obs_margin + obs.p_obs(1);
                        y_m = y_m * obs.r_obs_margin + obs.p_obs(2);
                        z_m = z_m * obs.r_obs_margin + obs.p_obs(3);
                        
                        % 半透明の黄色ワイヤーフレーム/サーフェスで描画
                        surf(obj.ax, x_m, y_m, z_m, 'FaceColor', [1.0 0.85 0.0], ...
                             'EdgeColor', [0.9 0.6 0.0], 'FaceAlpha', 0.18, 'LineStyle', ':');
                    end
                end
            end
        end

        function obj = gen_frame(obj, varargin)
            param = struct(varargin{:});
            obj.ax = param.ax;
            xlabel(obj.ax, "x [m]");
            ylabel(obj.ax, "y [m]");
            zlabel(obj.ax, "z [m]");
            hold(obj.ax, "on");
            if isfield(param, "rotor_r")
                r = param.rotor_r;
            else
                r = obj.rotor_r;
            end
            [xr, yr, zr] = cylinder(obj.ax, [0 r]);
            zr = 0.001*zr;
            [xb, yb, zb] = ellipsoid(obj.ax, 0, 0, 0, 1.2*obj.L(1)/4, 0.8*obj.L(2)/4, 0.02);
            d = obj.L/2;
            rp = [d(1), d(2), 0.02; -d(1), d(2), 0.02; d(1), -d(2), 0.02; -d(1), -d(2), 0.02];
            c = ["red", "green", "blue", "cyan", '#4DBEEE'];
            [x, y, z] = cylinder(obj.ax, 0.01);
            z = z*vecnorm(d)*2;
            R1 = rotmat(quaternion([obj.L(2), -obj.L(1), 0]*pi/(vecnorm(obj.L)*2), "rotvec"), 'point');
            R2 = rotmat(quaternion([obj.L(2), obj.L(1), 0]*pi/(vecnorm(obj.L)*2), "rotvec"), 'point');
            F1 = [R1*[x(1,:);y(1,:);z(1,:)];R1*[x(2,:);y(2,:);z(2,:)]]+[d(1);d(2);0;d(1);d(2);0];
            F2 = [R2*[x(1,:);y(1,:);z(1,:)];R2*[x(2,:);y(2,:);z(2,:)]]+[-d(1);d(2);0;-d(1);d(2);0];
            for n = param.target
                for i = 4:-1:1
                    h(n,i) = surface(obj.ax, xr+rp(i,1), yr+rp(i,2), zr+rp(i,3), 'FaceColor', c(i));
                    T(n,i) = quiver3(obj.ax, rp(i,1), rp(i,2), rp(i,3), 0, 0, 1, 'FaceColor', c(i));
                    tt(n,i) = hgtransform('Parent', obj.ax); set(T(n,i), 'Parent', tt(n,i));
                end
                h(n,5) = surface(obj.ax, [F1(1,:);F1(4,:);NaN(1,size(F1,2));F2(1,:);F2(4,:)], ...
                    [F1(2,:);F1(5,:);NaN(1,size(F1,2));F2(2,:);F2(5,:)], ...
                    [F1(3,:);F1(6,:);NaN(1,size(F1,2));F2(3,:);F2(6,:)], 'FaceColor', c(5));
                h(n,6) = surface(obj.ax, xb, yb, zb, 'FaceColor', c(5));
                t(n) = hgtransform('Parent', obj.ax); set(h(n,:), 'Parent', t(n));
            end
            obj.frame = t;
            obj.thrust = tt;
        end

        function draw(obj, target, p, q, u)
            arguments
                obj
                target
                p
                q
                u = [1;1;1;1]
            end
            for n = target
                frame = obj.frame(n);
                thrust = obj.thrust(n,:);
                R = makehgtform('axisrotate', q(1,1:3,n), q(1,4,n));
                Txyz = makehgtform('translate', p(1,:,n));
                for i = 1:4
                    if u(1,i,n) > 0
                        S = makehgtform('scale', [1,1,u(1,i,n)]);
                    elseif u(i) < 0
                        S1 = makehgtform('xrotate', pi);
                        S = makehgtform('scale', [1,1,-u(1,i,n)])*S1;
                    else
                        S = eye(4);
                        S(3,3) = 1e-5;
                    end
                    set(thrust(i), 'Matrix', Txyz*R*S);
                end
                set(frame, 'Matrix', Txyz*R);
            end
            drawnow
        end

        function animation(obj, logger, varargin)
            param = struct(varargin{:});
            if ~isfield(param, "target")
                param.target = 1;
            end
            p = obj.data_format(logger, param.target, "p", "p");
            q = obj.data_format(logger, param.target, "q", "p");
            u = logger.data(param.target, "input", "");
            u = reshape(u, size(u,1), size(u,2), length(param.target));
            Q = obj.gen_Q(param.target, q);
            r = [];
            try
                r = obj.data_format(logger, param.target, "p", "r");
            catch
                r = [];
            end
            if ~isempty(r)
                for n = 1:size(r, 3)
                    plot3(obj.ax, r(:,1,n), r(:,2,n), r(:,3,n), 'r--');
                end
            end
            
            % ドローンの実飛行軌跡 (青色実線)
            plot3(obj.ax, p(:,1,param.target), p(:,2,param.target), p(:,3,param.target), 'b-', 'LineWidth', 1.5);

            if isfield(param, "gif")
                sizen = 256;
                delaytime = 0;
                filename = strrep(strrep(strcat('Data/Movie(', string(datetime('now')), ').gif'), ':', '_'), ' ', '_');
            end
            if isfield(param, "mp4")
                sizen = 256;
                delaytime = 0;
                filename = strrep(strrep(strcat('Data/Movie(', string(datetime('now')), ').mp4'), ':', '_'), ' ', '_');
                v = VideoWriter(filename, "MPEG-4");
                if param.mp4
                    open(v);
                    writeAnimation(v);
                end
            end
            t = logger.data(0, "t", "");
            phase = [];
            try
                phase = logger.data(0, "phase", "");
            catch
                phase = [];
            end
            tRealtime = tic;
            for i = 1:length(t)-1
                if isfield(param, "opt_plot")
                    param.self.show(param.opt_plot, "logger", logger, "k", i, varargin{:});
                end
                if ~isvalid(obj.frame)
                    obj = obj.gen_frame("target", param.target, "ax", obj.ax);
                end
                obj.draw(param.target, p(i,:,param.target), Q(i,:,param.target), u(i,:,param.target));
                timeText = sprintf("%05.2f", t(i));
                phaseChar = "";
                if ~isempty(phase)
                    phaseChar = char(phase(i));
                end
                title(obj.ax, "time : " + timeText + "  phase : " + phaseChar);
                if isfield(param, "realtime")
                    delta = toc(tRealtime);
                    if t(i+1)-t(i) > delta
                        pause(t(i+1)-t(i) - delta);
                    end
                    tRealtime = tic;
                else
                    pause(0.01);
                end
                if isfield(param, "lims")
                    obj.xlim = param.lims(1,:);
                    obj.ylim = param.lims(2,:);
                    obj.zlim = param.lims(3,:);
                    obj.ax.XLimMode = "manual";
                    obj.ax.YLimMode = "manual";
                    obj.ax.ZLimMode = "manual";
                    obj.ax.XLim = obj.xlim;
                    obj.ax.YLim = obj.ylim;
                    obj.ax.ZLim = obj.zlim;
                end
                if isfield(param, "gif")
                    im = frame2im(getframe(obj.ax));
                    [imind, cm] = rgb2ind(im, sizen);
                    if i == 1
                        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delaytime);
                    else
                        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delaytime);
                    end
                end
                if isfield(param, "mp4")
                    framev = getframe(obj.ax);
                    writeVideo(v, framev);
                end
            end
            if isfield(param, "mp4")
                close(v);
            end
        end
    end

    methods (Access = private)
        function tf = has_key(~, args, key)
            tf = false;
            if isempty(args)
                return
            end
            if isstruct(args)
                tf = isfield(args, key);
                return
            end
            keys = string(args(1:2:end));
            tf = any(keys == key);
        end
        function p = data_format(~, logger, source, var, att)
            q = logger.data(source, var, att);
            p = reshape(q, size(q,1), size(q,2)/length(source), length(source));
        end
        function Q = gen_Q(~, target, q)
            Q = zeros(size(q,1), 4, length(target));
            for n = 1:length(target)
                switch size(q(:,:,n),2)
                    case 3
                        Q1 = quaternion(Eul2Quat(q(:,:,n)')');
                    case 4
                        Q1 = quaternion(q(:,:,n));
                    case 9
                        Q1 = quaternion(q(:,:,n), 'rotmat', 'frame');
                end
                Q2 = rotvec(Q1);
                tmp = vecnorm(Q2, 2, 2);
                Q(tmp==0,:,n) = 0;
                Q(tmp==0,1,n) = 1;
                if sum(tmp~=0) ~= 0
                    Q(tmp~=0,:,n) = [Q2(tmp~=0,:)./tmp(tmp~=0), tmp(tmp~=0)];
                end
            end
        end
    end
end