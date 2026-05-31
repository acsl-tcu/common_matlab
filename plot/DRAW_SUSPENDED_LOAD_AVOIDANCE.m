% classdef DRAW_SUSPENDED_LOAD_AVOIDANCE
%     % Animation class for single suspended-load (1 drone + 1 load)
% 
%     properties
%         frame
%         thrust
%         ax
%         xlim
%         ylim
%         zlim
%         L
%         frame_size = [];
%         rotor_r = [];
%         load
%         line
%         load_shape = "sphere";
%         load_size = [];
%     end
% 
%     methods
%         function obj = DRAW_SUSPENDED_LOAD_AVOIDANCE(logger, varargin)
%             param = struct(varargin{:});
%             if ~isfield(param, "target")
%                 param.target = 1;
%             end
%             if ~isfield(param, "self") || ~isprop(param.self, "parameter")
%                 error("DRAW_SUSPENDED_LOAD:MissingSelf", "self.parameter is required.");
%             end
%             if isprop(param.self.parameter, "Lx") && isprop(param.self.parameter, "Ly")
%                 obj.frame_size = [param.self.parameter.Lx, param.self.parameter.Ly];
%             else
%                 error("DRAW_SUSPENDED_LOAD:MissingFrameSize", "self.parameter.Lx/Ly are required.");
%             end
%             if isprop(param.self.parameter, "rotor_r")
%                 obj.rotor_r = param.self.parameter.rotor_r;
%             else
%                 error("DRAW_SUSPENDED_LOAD:MissingRotor", "self.parameter.rotor_r is required.");
%             end
%             if isprop(param.self.parameter, "Length")
%                 obj.load_size = repmat(param.self.parameter.Length, 1, 3);
%             else
%                 error("DRAW_SUSPENDED_LOAD:MissingLoadSize", "self.parameter.Length is required.");
%             end
% 
%             p = obj.data_format(logger, param.target, "p", "p");
%             pL = [];
%             try
%                 pL = obj.get_load_position(logger, param, p);
%             catch
%                 pL = [];
%             end
% 
%             data = p;
%             if ~isempty(pL)
%                 data = [p; pL];
%             end
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
%             if isfield(param, "load_shape")
%                 obj.load_shape = param.load_shape;
%             end
%             if isfield(param, "load_size")
%                 obj.load_size = param.load_size;
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
%             obj = obj.gen_load();
% 
%             p0 = p(1,:);
%             if isempty(pL)
%                 pL0 = p0 + [0 0 -obj.load_size(3)];
%             else
%                 pL0 = pL(1,:);
%             end
%             obj.line = plot3(obj.ax, [p0(1) pL0(1)], [p0(2) pL0(2)], [p0(3) pL0(3)], "k");
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
%         function obj = gen_load(obj)
%             switch obj.load_shape
%                 case {"cube", "cuboid"}
%                     scale = obj.load_size(:)'/2;
%                     v = [ ...
%                         -1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
%                         -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
%                     v = v .* scale;
%                     f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
%                     h = patch(obj.ax, "Faces", f, "Vertices", v, "FaceColor", "cyan", ...
%                         "FaceAlpha", 0.5, "EdgeColor", "none");
%                 otherwise
%                     [x, y, z] = sphere(12);
%                     x = x * obj.load_size(1);
%                     y = y * obj.load_size(2);
%                     z = z * obj.load_size(3);
%                     h = surf(obj.ax, x, y, z, "FaceColor", "cyan", "FaceAlpha", 0.5, "EdgeColor", "none");
%             end
%             t = hgtransform('Parent', obj.ax);
%             set(h, 'Parent', t);
%             obj.load = t;
%         end
% 
%         function draw(obj, target, p, q, u, pL)
%             arguments
%                 obj
%                 target
%                 p
%                 q
%                 u = [1;1;1;1]
%                 pL = []
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
%                     elseif u(1,i,n) < 0
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
% 
%             if ~isempty(pL)
%                 Tload = makehgtform('translate', pL(1,:));
%                 set(obj.load, 'Matrix', Tload);
%                 obj.line.XData = [p(1,1,1) pL(1,1)];
%                 obj.line.YData = [p(1,2,1) pL(1,2)];
%                 obj.line.ZData = [p(1,3,1) pL(1,3)];
%             end
%             drawnow
%         end
% 
%         function animation(obj, logger, varargin)
%             param = struct(varargin{:});
%             if ~isfield(param, "target");      param.target = 1; end
% 
%             % ---- save options ----
%             % mp4: true/false or "path/to/file.mp4"
%             % gif: true/false or "path/to/file.gif"
%             if ~isfield(param, "fps");         param.fps = 30; end
%             if ~isfield(param, "skip");        param.skip = 1; end        % 1=全フレーム, 2=間引き
%             if ~isfield(param, "pause");       param.pause = 0.01; end    % 再生表示用（保存だけなら0推奨）
%             if ~isfield(param, "outdir");      param.outdir = "Data"; end
%             if ~isfield(param, "quality");     param.quality = 100; end   % MP4品質(0-100)
%             if ~isfield(param, "resolution");  param.resolution = 200; end % exportgraphics DPI相当(150-300目安)
%             if ~isfield(param, "gif_delay");   param.gif_delay = 1/param.fps; end
% 
%             do_mp4 = false; mp4_name = "";
%             if isfield(param, "mp4") && ~isempty(param.mp4)
%                 do_mp4 = true;
%                 if ~(islogical(param.mp4) || isnumeric(param.mp4))
%                     mp4_name = string(param.mp4);
%                 end
%             end
% 
%             do_gif = false; gif_name = "";
%             if isfield(param, "gif") && ~isempty(param.gif)
%                 do_gif = true;
%                 if ~(islogical(param.gif) || isnumeric(param.gif))
%                     gif_name = string(param.gif);
%                 end
%             end
% 
%             if do_mp4 || do_gif
%                 if ~exist(param.outdir, "dir"); mkdir(param.outdir); end
%             end
% 
%             timestamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
%             if do_mp4 && mp4_name == ""
%                 mp4_name = fullfile(param.outdir, "Movie_" + timestamp + ".mp4");
%             end
%             if do_gif && gif_name == ""
%                 gif_name = fullfile(param.outdir, "Movie_" + timestamp + ".gif");
%             end
% 
%             % ---- data ----
%             p = obj.data_format(logger, param.target, "p", "p");
%             q = obj.data_format(logger, param.target, "q", "p");
%             u = logger.data(param.target, "input", "");
%             u = reshape(u, size(u,1), size(u,2), length(param.target));
%             Q = obj.gen_Q(param.target, q);
%             pL = obj.get_load_position(logger, param, p);
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
%             t = logger.data(0, "t", "");
%             phase = [];
%             try
%                 phase = logger.data(0, "phase", "");
%             catch
%                 phase = [];
%             end
% 
%             fig = ancestor(obj.ax,'figure');
%             fig.Units = "pixels";
%             fig.Position(3:4) = [1280 720];   % 偶数
%             % ---- writers ----
%             v = [];
%             if do_mp4
%                 v = VideoWriter(mp4_name, "MPEG-4");
%                 v.FrameRate = param.fps;
%                 v.Quality = param.quality;
%                 open(v);
%                 cleaner = onCleanup(@() safe_close(v));
%             end
% 
%             % ---- loop ----
%             first_gif_written = false;
% 
%             for i = 1:param.skip:(length(t)-1)
%                 if ~isvalid(obj.frame)
%                     obj = obj.gen_frame("target", param.target, "ax", obj.ax);
%                 end
% 
%                 obj.draw(param.target, p(i,:,param.target), Q(i,:,param.target), u(i,:,param.target), pL(i,:));
% 
%                 timeText = sprintf("%05.2f", t(i));
%                 phaseChar = "";
%                 if ~isempty(phase)
%                     phaseChar = char(phase(i));
%                 end
%                 title(obj.ax, "time : " + timeText + "  phase : " + phaseChar);
% 
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
% 
%                 % ---- capture by getframe (lightweight) ----
%                 if do_mp4 || do_gif
%                     fig = ancestor(obj.ax,'figure');   % 追加：UIAxesでも軸を拾いやすい
%                     fr = getframe(fig);                % ←ここがポイント（obj.ax じゃなく fig）
% 
%                     if do_mp4
%                         writeVideo(v, fr);
%                     end
% 
%                     if do_gif
%                         im = frame2im(fr);
%                         [imind, cm] = rgb2ind(im, 256);
%                         if ~first_gif_written
%                             imwrite(imind, cm, gif_name, 'gif', 'Loopcount', inf, 'DelayTime', param.gif_delay);
%                             first_gif_written = true;
%                         else
%                             imwrite(imind, cm, gif_name, 'gif', 'WriteMode', 'append', 'DelayTime', param.gif_delay);
%                         end
%                     end
%                 end
% 
%                 if param.pause > 0
%                     pause(param.pause);
%                 end
%             end
% 
%             % ---- close ----
%             if do_mp4
%                 safe_close(v);
%                 fprintf("[DRAW_SUSPENDED_LOAD] saved mp4: %s\n", mp4_name);
%             end
%             if do_gif
%                 fprintf("[DRAW_SUSPENDED_LOAD] saved gif: %s\n", gif_name);
%             end
% 
%             % ---- local helpers ----
%             function safe_close(vw)
%                 try
%                     if ~isempty(vw); close(vw); end
%                 catch
%                 end
%             end
% 
%             function out = pad_even(img)
%                 out = img;
%                 [h, w, ~] = size(out);
%                 hp = mod(h,2);
%                 wp = mod(w,2);
%                 if hp ~= 0
%                     out(end+1,:,:) = out(end,:,:); % 最終行を複製
%                 end
%                 if wp ~= 0
%                     out(:,end+1,:) = out(:,end,:); % 最終列を複製
%                 end
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
%         function pL = get_load_position(obj, logger, param, p)
%             if isfield(param, "load_source")
%                 load_source = param.load_source;
%             elseif isfield(param, "target")
%                 load_source = param.target;
%             else
%                 load_source = 1;
%             end
% 
%             if isfield(param, "load_var")
%                 load_att = "e";
%                 if isfield(param, "load_att")
%                     load_att = param.load_att;
%                 end
%                 pL = obj.data_format(logger, load_source, param.load_var, load_att);
%                 pL = reshape(pL, size(p,1), 3);
%                 return
%             end
% 
%             candidates = { ...
%                 {"estimator.result.state.pL", "e"}, ...
%                 {"pL", "p"} ...
%                 };
%             for k = 1:length(candidates)
%                 item = candidates{k};
%                 try
%                     pL = obj.data_format(logger, load_source, item{1}, item{2});
%                     pL = reshape(pL, size(p,1), 3);
%                     return
%                 catch
%                 end
%             end
% 
%             pT = [];
%             candidates = { ...
%                 {"estimator.result.state.pT", "e"}, ...
%                 {"pT", "p"} ...
%                 };
%             for k = 1:length(candidates)
%                 item = candidates{k};
%                 try
%                     pT = obj.data_format(logger, load_source, item{1}, item{2});
%                     pT = reshape(pT, size(p,1), 3);
%                     break
%                 catch
%                 end
%             end
%             if isempty(pT)
%                 error("DRAW_SUSPENDED_LOAD:MissingLoad", "Load position data is missing.");
%             end
% 
%             if isfield(param, "self") && isprop(param.self, "parameter")
%                 L = param.self.parameter.get("cableL");
%             else
%                 L = 0;
%             end
%             pL = p + pT * L;
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


classdef DRAW_SUSPENDED_LOAD_AVOIDANCE
    % 障害物回避対応版：ドローン＋吊り荷の3次元アニメーション表示クラス
    % 論文準拠の「機体・紐を包む複数の保護球」と、障害物の「本体・安全マージン・CBF領域」を半透明で完全可視化します。

    properties
        frame
        thrust
        ax
        xlim
        ylim
        zlim
        L
        frame_size = [];
        rotor_r = [];
        load
        line
        load_shape = "sphere";
        load_size = [];
        
        % --- 【新規追加】回避表示用のグラフィックス・ハンドル群 ---
        obs_handles = [];      % 障害物の表示オブジェクト
        protection_spheres = []; % ドローン・ワイヤー側を包む保護球のオブジェクト
    end

    methods
        function obj = DRAW_SUSPENDED_LOAD_AVOIDANCE(logger, varargin)
            param = struct(varargin{:});
            if ~isfield(param, "target")
                param.target = 1;
            end
            if ~isfield(param, "self") || ~isprop(param.self, "parameter")
                error("DRAW_SUSPENDED_LOAD_AVOIDANCE:MissingSelf", "self.parameter is required.");
            end
            if isprop(param.self.parameter, "Lx") && isprop(param.self.parameter, "Ly")
                obj.frame_size = [param.self.parameter.Lx, param.self.parameter.Ly];
            else
                error("DRAW_SUSPENDED_LOAD_AVOIDANCE:MissingFrameSize", "self.parameter.Lx/Ly are required.");
            end
            if isprop(param.self.parameter, "rotor_r")
                obj.rotor_r = param.self.parameter.rotor_r;
            else
                error("DRAW_SUSPENDED_LOAD_AVOIDANCE:MissingRotor", "self.parameter.rotor_r is required.");
            end
            if isprop(param.self.parameter, "Length")
                obj.load_size = repmat(param.self.parameter.Length, 1, 3);
            else
                error("DRAW_SUSPENDED_LOAD_AVOIDANCE:MissingLoadSize", "self.parameter.Length is required.");
            end

            p = obj.data_format(logger, param.target, "p", "p");
            pL = [];
            try
                pL = obj.get_load_position(logger, param, p);
            catch
                pL = [];
            end

            data = p;
            if ~isempty(pL)
                data = [p; pL];
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
            if isfield(param, "load_shape")
                obj.load_shape = param.load_shape;
            end
            if isfield(param, "load_size")
                obj.load_size = param.load_size;
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
            obj = obj.gen_load();

            p0 = p(1,:);
            if isempty(pL)
                pL0 = p0 + [0 0 -obj.load_size(3)];
            else
                pL0 = pL(1,:);
            end
            obj.line = plot3(obj.ax, [p0(1) pL0(1)], [p0(2) pL0(2)], [p0(3) pL0(3)], "k", 'LineWidth', 2);
            
            %% 🧱 【新規追加】静的障害物環境の3層マージンプロット初期化
            % ENVIRONMENT_OBSTACLE からデータを引っ張り、環境を描画する
            try
                obs_list = ENVIRONMENT_OBSTACLE();
                [sx, sy, sz] = sphere(24); % マージン球用の高密度メッシュ
                hold(obj.ax, "on");
                
                for i = 1:length(obs_list)
                    xo = obs_list(i).p_obs;
                    ro = obs_list(i).r_obs;
                    R_safe = obs_list(i).R_safe; 
                    R_cbf  = R_safe + 0.5; % CBFアクティブ警戒領域（最外層）
                    
                    % -----------------------------------------------------------
                    % 層①：障害物のオリジナル「元の幾何学形状」（不透明ソリッド赤）
                    % -----------------------------------------------------------
                    switch lower(obs_list(i).type)
                        case 'sphere'
                            % 1. 真球
                            r_sp = obs_list(i).raw_param;
                            pc = obs_list(i).p_center;
                            obj.obs_handles(end+1) = surf(obj.ax, sx*r_sp + pc(1), sy*r_sp + pc(2), sz*r_sp + pc(3), ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                                
                        case 'cylinder'
                            % 2. 円柱 (現在これが発動します)
                            r_cyl = obs_list(i).raw_param(1);
                            h_cyl = obs_list(i).raw_param(2);
                            pc = obs_list(i).p_center;
                            [cx, cy, cz] = cylinder(obj.ax, r_cyl, 24);
                            cz = cz * h_cyl + (pc(3) - h_cyl/2); % 高度の中心をパチッと合わせる
                            % 側面の描画
                            obj.obs_handles(end+1) = surf(obj.ax, cx + pc(1), cy + pc(2), cz, ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                            % 上面・底面の蓋の描画
                            patch(obj.ax, cx(1,:)+pc(1), cy(1,:)+pc(2), cz(1,:), [0.8 0.1 0.1], 'EdgeColor', 'none');
                            patch(obj.ax, cx(2,:)+pc(1), cy(2,:)+pc(2), cz(2,:), [0.8 0.1 0.1], 'EdgeColor', 'none');
                                
                        case 'box'
                            % 3. 直方体 / 四角柱
                            dx = obs_list(i).raw_param(1); dy = obs_list(i).raw_param(2); dz = obs_list(i).raw_param(3);
                            pc = obs_list(i).p_center;
                            v_box = [ -1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] .* [dx, dy, dz]/2 + pc';
                            f_box = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                            obj.obs_handles(end+1) = patch(obj.ax, 'Faces', f_box, 'Vertices', v_box, ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                                
                        case 'prism'
                            % 4. 正三角柱
                            a_tri = obs_list(i).raw_param(1); h_pri = obs_list(i).raw_param(2);
                            pc = obs_list(i).p_center;
                            r_tri = a_tri / sqrt(3);
                            th_tri = [0, 120, 240] * pi / 180;
                            tx = r_tri * cos(th_tri); ty = r_tri * sin(th_tri);
                            v_pri = [tx', ty', zeros(3,1)-h_pri/2; tx', ty', zeros(3,1)+h_pri/2] + pc';
                            f_pri = [1 2 3 NaN; 4 5 6 NaN; 1 2 5 4; 2 3 6 5; 3 1 4 6];
                            obj.obs_handles(end+1) = patch(obj.ax, 'Faces', f_pri, 'Vertices', v_pri, ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                                
                        case 'cone'
                            % 5. 円錐
                            r_co = obs_list(i).raw_param(1); h_co = obs_list(i).raw_param(2);
                            pc = obs_list(i).p_center;
                            [cx, cy, cz] = cylinder(obj.ax, [r_co, 0], 24);
                            cz = cz * h_co + (pc(3) - h_co/2);
                            obj.obs_handles(end+1) = surf(obj.ax, cx + pc(1), cy + pc(2), cz, ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                            patch(obj.ax, cx(1,:)+pc(1), cy(1,:)+pc(2), cz(1,:), [0.8 0.1 0.1], 'EdgeColor', 'none');
                                
                        case 'pyramid'
                            % 6. 正四角錐
                            a_py = obs_list(i).raw_param(1); h_py = obs_list(i).raw_param(2);
                            pc = obs_list(i).p_center;
                            d_py = a_py / 2;
                            v_py = [-d_py, -d_py, -h_py/2; d_py, -d_py, -h_py/2; d_py, d_py, -h_py/2; -d_py, d_py, -h_py/2; 0, 0, h_py/2] + pc';
                            f_py = [1 2 3 4; 1 2 5 NaN; 2 3 5 NaN; 3 4 5 NaN; 4 1 5 NaN];
                            obj.obs_handles(end+1) = patch(obj.ax, 'Faces', f_py, 'Vertices', v_py, ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                                
                        case 'custom'
                            % 7. カスタム任意多面体
                            v_cust = obs_list(i).raw_param;
                            pc = obs_list(i).p_center;
                            % 頂点データから凸包（3Dソリッド体）を自動ビルドしてプロット
                            [f_cust, v_cust_mod] = convhull(v_cust(1,:), v_cust(2,:), v_cust(3,:));
                            obj.obs_handles(end+1) = trisurf(f_cust, v_cust_mod(:,1), v_cust_mod(:,2), v_cust_mod(:,3), ...
                                'Parent', obj.ax, 'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                                
                        otherwise
                            obj.obs_handles(end+1) = surf(obj.ax, sx*ro + xo(1), sy*ro + xo(2), sz*ro + xo(3), ...
                                'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
                    end
                    
                    % -----------------------------------------------------------
                    % 層②：数式計算で丸め込んだ「近似真球バリア」（半透明オレンジ）
                    % -----------------------------------------------------------
                    obj.obs_handles(end+1) = surf(obj.ax, sx*ro + xo(1), sy*ro + xo(2), sz*ro + xo(3), ...
                        'FaceColor', [1.0 0.5 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.25);
                    
                    % -----------------------------------------------------------
                    % 層③：さらに外側の制御上の「安全マージン境界」（薄い半透明黄色）
                    % -----------------------------------------------------------
                    obj.obs_handles(end+1) = surf(obj.ax, sx*R_safe + xo(1), sy*R_safe + xo(2), sz*R_safe + xo(3), ...
                        'FaceColor', [1.0 1.0 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.10);
                end
            catch
                disp('⚠️ [Visualizer ERROR] 3層幾何学障害物グラフィックスの生成に失敗しました。');
            end

            %% 🛡️ 【新規追加】単機牽引システム側の「保護球（2つ）」のグラフィックス初期化
            % 論文の Case 1 仕様 (\lambda^1=0.25, \lambda^2=0.75, 半径0.25m) の器を生成
            [px, py, pz] = sphere(16);
            r_sphere = 0.25; % 論文Table 1 記載の保護球半径
            
            % 1つ目の保護球（機体側：半透明なシアン青）
            obj.protection_spheres(1) = surf(obj.ax, px*r_sphere, py*r_sphere, pz*r_sphere, ...
                'FaceColor', [0.2 0.8 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
            % 2つ目の保護球（吊り荷側：半透明なシアン青）
            obj.protection_spheres(2) = surf(obj.ax, px*r_sphere, py*r_sphere, pz*r_sphere, ...
                'FaceColor', [0.2 0.8 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.35);

            view(ax, 3)
            grid(ax, 'on')
            daspect(ax, [1 1 1]);
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

        function obj = gen_load(obj)
            switch obj.load_shape
                case {"cube", "cuboid"}
                    scale = obj.load_size(:)'/2;
                    v = [ ...
                        -1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
                        -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
                    v = v .* scale;
                    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                    h = patch(obj.ax, "Faces", f, "Vertices", v, "FaceColor", "cyan", ...
                        "FaceAlpha", 0.5, "EdgeColor", "none");
                otherwise
                    [x, y, z] = sphere(12);
                    x = x * obj.load_size(1);
                    y = y * obj.load_size(2);
                    z = z * obj.load_size(3);
                    h = surf(obj.ax, x, y, z, "FaceColor", "magenta", "FaceAlpha", 0.8, "EdgeColor", "none");
            end
            t = hgtransform('Parent', obj.ax);
            set(h, 'Parent', t);
            obj.load = t;
        end

        function draw(obj, target, p, q, u, pL)
            arguments
                obj
                target
                p
                q
                u = [1;1;1;1]
                pL = []
            end

            for n = target
                frame = obj.frame(n);
                thrust = obj.thrust(n,:);
                R = makehgtform('axisrotate', q(1,1:3,n), q(1,4,n));
                Txyz = makehgtform('translate', p(1,:,n));
                for i = 1:4
                    if u(1,i,n) > 0
                        S = makehgtform('scale', [1,1,u(1,i,n)]);
                    elseif u(1,i,n) < 0
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

            if ~isempty(pL)
                Tload = makehgtform('translate', pL(1,:));
                set(obj.load, 'Matrix', Tload);
                obj.line.XData = [p(1,1,1) pL(1,1)];
                obj.line.YData = [p(1,2,1) pL(1,2)];
                obj.line.ZData = [p(1,3,1) pL(1,3)];
                
                %% 🔄 【新規追加】機体・ワイヤー側の「2つの保護球」の同期スライド
                lambda = [0.25, 0.75];
                r_sphere = 0.25;
                [px, py, pz] = sphere(16);
                
                % ドローン機体位置 p と 吊り荷位置 pL から、現在の保護球中心をリアルタイム再計算
                xq_vec = p(1,:,1)';
                pL_vec = pL(1,:)';
                n_vec = (pL_vec - xq_vec); % ケーブルの方向ベクトル(長さ含む)
                
                for j = 1:2
                    % ケーブル線分上の割合に従って球体の中心位置を移動
                    pos_center = xq_vec + lambda(j) * n_vec;
                    
                    set(obj.protection_spheres(j), ...
                        'XData', px * r_sphere + pos_center(1), ...
                        'YData', py * r_sphere + pos_center(2), ...
                        'ZData', pz * r_sphere + pos_center(3));
                end
            end
            drawnow
        end

        function animation(obj, logger, varargin)
            param = struct(varargin{:});
            if ~isfield(param, "target");      param.target = 1; end

            if ~isfield(param, "fps");         param.fps = 30; end
            if ~isfield(param, "skip");        param.skip = 1; end        
            if ~isfield(param, "pause");       param.pause = 0.01; end    
            if ~isfield(param, "outdir");      param.outdir = "Data"; end
            if ~isfield(param, "quality");     param.quality = 100; end   
            if ~isfield(param, "resolution");  param.resolution = 200; end 
            if ~isfield(param, "gif_delay");   param.gif_delay = 1/param.fps; end

            do_mp4 = false; mp4_name = "";
            if isfield(param, "mp4") && ~isempty(param.mp4)
                do_mp4 = true;
                if ~(islogical(param.mp4) || isnumeric(param.mp4))
                    mp4_name = string(param.mp4);
                end
            end

            do_gif = false; gif_name = "";
            if isfield(param, "gif") && ~isempty(param.gif)
                do_gif = true;
                if ~(islogical(param.gif) || isnumeric(param.gif))
                    gif_name = string(param.gif);
                end
            end

            if do_mp4 || do_gif
                if ~exist(param.outdir, "dir"); mkdir(param.outdir); end
            end

            timestamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
            if do_mp4 && mp4_name == ""
                mp4_name = fullfile(param.outdir, "Movie_" + timestamp + ".mp4");
            end
            if do_gif && gif_name == ""
                gif_name = fullfile(param.outdir, "Movie_" + timestamp + ".gif");
            end

            p = obj.data_format(logger, param.target, "p", "p");
            q = obj.data_format(logger, param.target, "q", "p");
            u = logger.data(param.target, "input", "");
            u = reshape(u, size(u,1), size(u,2), length(param.target));
            Q = obj.gen_Q(param.target, q);
            pL = obj.get_load_position(logger, param, p);

            r = [];
            try
                r = obj.data_format(logger, param.target, "p", "r");
            catch
                r = [];
            end
            if ~isempty(r)
                for n = 1:size(r, 3)
                    plot3(obj.ax, r(:,1,n), r(:,2,n), r(:,3,n), 'r');
                end
            end

            t = logger.data(0, "t", "");
            phase = [];
            try
                phase = logger.data(0, "phase", "");
            catch
                phase = [];
            end

            fig = ancestor(obj.ax,'figure');
            fig.Units = "pixels";
            fig.Position(3:4) = [1280 720];   
            v = [];
            if do_mp4
                v = VideoWriter(mp4_name, "MPEG-4");
                v.FrameRate = param.fps;
                v.Quality = param.quality;
                open(v);
                cleaner = onCleanup(@() safe_close(v));
            end

            first_gif_written = false;

            for i = 1:param.skip:(length(t)-1)
                if ~isvalid(obj.frame)
                    obj = obj.gen_frame("target", param.target, "ax", obj.ax);
                end

                obj.draw(param.target, p(i,:,param.target), Q(i,:,param.target), u(i,:,param.target), pL(i,:));

                timeText = sprintf("%05.2f", t(i));
                phaseChar = "";
                if ~isempty(phase)
                    phaseChar = char(phase(i));
                end
                title(obj.ax, "time : " + timeText + "  phase : " + phaseChar);

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

                if do_mp4 || do_gif
                    fig = ancestor(obj.ax,'figure');   
                    fr = getframe(fig);                

                    if do_mp4
                        writeVideo(v, fr);
                    end

                    if do_gif
                        im = frame2im(fr);
                        [imind, cm] = rgb2ind(im, 256);
                        if ~first_gif_written
                            imwrite(imind, cm, gif_name, 'gif', 'Loopcount', inf, 'DelayTime', param.gif_delay);
                            first_gif_written = true;
                        else
                            imwrite(imind, cm, gif_name, 'gif', 'WriteMode', 'append', 'DelayTime', param.gif_delay);
                        end
                    end
                end

                if param.pause > 0
                    pause(param.pause);
                end
            end

            if do_mp4
                safe_close(v);
                fprintf("[DRAW_SUSPENDED_LOAD_AVOIDANCE] saved mp4: %s\n", mp4_name);
            end
            if do_gif
                fprintf("[DRAW_SUSPENDED_LOAD_AVOIDANCE] saved gif: %s\n", gif_name);
            end

            function safe_close(vw)
                try
                    if ~isempty(vw); close(vw); end
                catch
                end
            end
            function out = pad_even(img)
                out = img;
                [h, w, ~] = size(out);
                hp = mod(h,2);
                wp = mod(w,2);
                if hp ~= 0
                    out(end+1,:,:) = out(end,:,:); 
                end
                if wp ~= 0
                    out(:,end+1,:) = out(:,end,:); 
                end
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

        % (中略: 元のDRAW_SUSPENDED_LOADの共通関数を完全保持)
        function pL = get_load_position(obj, logger, param, p)
            if isfield(param, "load_source"); load_source = param.load_source;
            elseif isfield(param, "target"); load_source = param.target;
            else; load_source = 1; end
            if isfield(param, "load_var")
                load_att = "e"; if isfield(param, "load_att"); load_att = param.load_att; end
                pL = obj.data_format(logger, load_source, param.load_var, load_att);
                pL = reshape(pL, size(p,1), 3); return
            end
            candidates = {{"estimator.result.state.pL", "e"}, {"pL", "p"}};
            for k = 1:length(candidates)
                item = candidates{k};
                try; pL = obj.data_format(logger, load_source, item{1}, item{2});
                    pL = reshape(pL, size(p,1), 3); return; catch; end
            end
            pT = []; candidates = {{"estimator.result.state.pT", "e"}, {"pT", "p"}};
            for k = 1:length(candidates)
                item = candidates{k};
                try; pT = obj.data_format(logger, load_source, item{1}, item{2});
                    pT = reshape(pT, size(p,1), 3); break; catch; end
            end
            if isempty(pT); error("DRAW_SUSPENDED_LOAD_AVOIDANCE:MissingLoad", "Load position data is missing."); end
            if isfield(param, "self") && isprop(param.self, "parameter"); L = param.self.parameter.get("cableL"); else; L = 0; end
            pL = p + pT * L;
        end
        function Q = gen_Q(~, target, q)
            Q = zeros(size(q,1), 4, length(target));
            for n = 1:length(target)
                switch size(q(:,:,n),2)
                    case 3; Q1 = quaternion(Eul2Quat(q(:,:,n)')');
                    case 4; Q1 = quaternion(q(:,:,n));
                    case 9; Q1 = quaternion(q(:,:,n), 'rotmat', 'frame');
                end
                Q2 = rotvec(Q1); tmp = vecnorm(Q2, 2, 2);
                Q(tmp==0,:,n) = 0; Q(tmp==0,1,n) = 1;
                if sum(tmp~=0) ~= 0; Q(tmp~=0,:,n) = [Q2(tmp~=0,:)./tmp(tmp~=0), tmp(tmp~=0)]; end
            end
        end
    end
end