% classdef DRAW_SUSPENDED_LOAD
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
%                 load_size = [];
%         % CBF Safety Visualizations
%         cbf_type = 0; % 0:None, 1:3-Spheres, 2:Ellipsoid
%         cbf_handles = [];
%         cbf_r_safe = 0.6;
%     end
% 
%     methods
%         function obj = DRAW_SUSPENDED_LOAD(logger, varargin)
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
% 

% classdef DRAW_SUSPENDED_LOAD
%     % Animation class for single suspended-load (1 drone + 1 load)
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
% 
%         % =========================================================
%         % CBF Safety Visualizations (自機防護領域の可視化)
%         % =========================================================
%         cbf_type = 2;       % 0:なし, 1:3球近似, 2:動的回転楕円体 (推奨)
%         cbf_handles = [];
%         cbf_r_safe = 0.35;  % a_sys (短軸半径)
%         cableL = 1.0;
%     end
%     methods
%         function obj = DRAW_SUSPENDED_LOAD(logger, varargin)
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
%             if isprop(param.self.parameter, "cableL")
%                 obj.cableL = param.self.parameter.get("cableL");
%             end
% 
%             p = obj.data_format(logger, param.target, "p", "p");
%             pL = [];
%             try
%                 pL = obj.get_load_position(logger, param, p);
%             catch
%                 pL = [];
%             end
%             data = p;
%             if ~isempty(pL)
%                 data = [p; pL];
%             end
%             tM = max(data, [], 1);
%             tm = min(data, [], 1);
%             M = [max(tM(1:3:end)), max(tM(2:3:end)), max(tM(3:3:end))];
%             m = [min(tm(1:3:end)), min(tm(2:3:end)), min(tm(3:3:end))];
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
%             if isfield(param, "cbf_type")
%                 obj.cbf_type = param.cbf_type;
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
%             obj.line = plot3(obj.ax, [p0(1) pL0(1)], [p0(2) pL0(2)], [p0(3) pL0(3)], "k", 'LineWidth', 1.5);
% 
%             % =========================================================
%             % CBF 防護メッシュの初期生成 (hgtransform 登録)
%             % =========================================================
%             obj = obj.gen_cbf_mesh();
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
%             if isfield(param, "rotor_r")
%                 r = param.rotor_r;
%             else
%                 r = obj.rotor_r;
%             end
%             [xr, yr, zr] = cylinder(obj.ax, [0 r]);
%             zr = 0.001*zr;
%             [xb, yb, zb] = ellipsoid(obj.ax, 0, 0, 0, 1.2*obj.L(1)/4, 0.8*obj.L(2)/4, 0.02);
%             d = obj.L/2;
%             rp = [d(1), d(2), 0.02; -d(1), d(2), 0.02; d(1), -d(2), 0.02; -d(1), -d(2), 0.02];
%             c = ["red", "green", "blue", "cyan", '#4DBEEE'];
%             [x, y, z] = cylinder(obj.ax, 0.01);
%             z = z*vecnorm(d)*2;
%             R1 = rotmat(quaternion([obj.L(2), -obj.L(1), 0]*pi/(vecnorm(obj.L)*2), "rotvec"), 'point');
%             R2 = rotmat(quaternion([obj.L(2), obj.L(1), 0]*pi/(vecnorm(obj.L)*2), "rotvec"), 'point');
%             F1 = [R1*[x(1,:);y(1,:);z(1,:)];R1*[x(2,:);y(2,:);z(2,:)]]+[d(1);d(2);0;d(1);d(2);0];
%             F2 = [R2*[x(1,:);y(1,:);z(1,:)];R2*[x(2,:);y(2,:);z(2,:)]]+[-d(1);d(2);0;-d(1);d(2);0];
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
%                         "FaceAlpha", 0.8, "EdgeColor", "none");
%                 otherwise
%                     [x, y, z] = sphere(16);
%                     x = x * obj.load_size(1);
%                     y = y * obj.load_size(2);
%                     z = z * obj.load_size(3);
%                     h = surf(obj.ax, x, y, z, "FaceColor", "cyan", "FaceAlpha", 0.8, "EdgeColor", "none");
%             end
%             t = hgtransform('Parent', obj.ax);
%             set(h, 'Parent', t);
%             obj.load = t;
%         end
% 
%         % =========================================================
%         % 自機防護メッシュ生成関数 (半透明メッシュ)
%         % =========================================================
%         function obj = gen_cbf_mesh(obj)
%             if obj.cbf_type == 2
%                 % 動的回転楕円体 (Ellipsoid)
%                 a_sys = obj.cbf_r_safe;               % 短軸 0.35m
%                 b_sys = (obj.cableL / 2.0) + 0.15;    % 長軸
%                 [xe, ye, ze] = ellipsoid(obj.ax, 0, 0, 0, a_sys, a_sys, b_sys, 20);
%                 % 透過度 0.25 のライムグリーン色で表示
%                 h_cbf = surf(obj.ax, xe, ye, ze, 'FaceColor', [0.2 0.8 0.2], ...
%                     'FaceAlpha', 0.25, 'EdgeColor', [0.1 0.5 0.1], 'EdgeAlpha', 0.15);
%                 t_cbf = hgtransform('Parent', obj.ax);
%                 set(h_cbf, 'Parent', t_cbf);
%                 obj.cbf_handles = t_cbf;
% 
%             elseif obj.cbf_type == 1
%                 % 複数球 (3-Spheres: 機体, 中点, 荷物)
%                 [xs, ys, zs] = sphere(16);
%                 r_s = 0.6;
%                 obj.cbf_handles = [];
%                 for i = 1:3
%                     h_s = surf(obj.ax, xs*r_s, ys*r_s, zs*r_s, 'FaceColor', [0.2 0.6 1.0], ...
%                         'FaceAlpha', 0.2, 'EdgeColor', 'none');
%                     t_s = hgtransform('Parent', obj.ax);
%                     set(h_s, 'Parent', t_s);
%                     obj.cbf_handles = [obj.cbf_handles, t_s];
%                 end
%             end
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
% 
%                 % =====================================================
%                 % 自機防護領域のリアルタイム追従更新
%                 % =====================================================
%                 p_drone = p(1,:,1)';
%                 p_ld    = pL(1,:)';
%                 p_mid   = (p_drone + p_ld) / 2.0;
% 
%                 if obj.cbf_type == 2 && ~isempty(obj.cbf_handles)
%                     % 楕円体を中点 p_mid に配置し、テザー姿勢 pT に傾ける
%                     vec_tether = p_drone - p_ld;
%                     len_t = norm(vec_tether);
%                     if len_t > 1e-4
%                         pT = vec_tether / len_t;
%                     else
%                         pT = [0; 0; 1];
%                     end
% 
%                     % 基準Z軸 [0;0;1] から pT への回転行列を導出
%                     z_axis = [0; 0; 1];
%                     rot_axis = cross(z_axis, pT);
%                     norm_axis = norm(rot_axis);
% 
%                     T_mid = makehgtform('translate', p_mid');
%                     if norm_axis > 1e-4
%                         rot_angle = atan2(norm_axis, dot(z_axis, pT));
%                         R_ellip = makehgtform('axisrotate', rot_axis' / norm_axis, rot_angle);
%                         set(obj.cbf_handles, 'Matrix', T_mid * R_ellip);
%                     else
%                         if dot(z_axis, pT) < 0
%                             % 真下を向いている場合 (180度反転)
%                             R_ellip = makehgtform('xrotate', pi);
%                             set(obj.cbf_handles, 'Matrix', T_mid * R_ellip);
%                         else
%                             set(obj.cbf_handles, 'Matrix', T_mid);
%                         end
%                     end
% 
%                 elseif obj.cbf_type == 1 && length(obj.cbf_handles) == 3
%                     % 3球近似 (機体, 中点, 荷物)
%                     set(obj.cbf_handles(1), 'Matrix', makehgtform('translate', p_drone'));
%                     set(obj.cbf_handles(2), 'Matrix', makehgtform('translate', p_mid'));
%                     set(obj.cbf_handles(3), 'Matrix', makehgtform('translate', p_ld'));
%                 end
%             end
%             drawnow
%         end
% 
%         function animation(obj, logger, varargin)
%             param = struct(varargin{:});
%             if ~isfield(param, "target");      param.target = 1; end
%             if ~isfield(param, "fps");         param.fps = 30; end
%             if ~isfield(param, "skip");        param.skip = 1; end
%             if ~isfield(param, "pause");       param.pause = 0.01; end
%             if ~isfield(param, "outdir");      param.outdir = "Data"; end
%             if ~isfield(param, "quality");     param.quality = 100; end
%             if ~isfield(param, "resolution");  param.resolution = 200; end
%             if ~isfield(param, "gif_delay");   param.gif_delay = 1/param.fps; end
%             do_mp4 = false; mp4_name = "";
%             if isfield(param, "mp4") && ~isempty(param.mp4)
%                 do_mp4 = true;
%                 if ~(islogical(param.mp4) || isnumeric(param.mp4))
%                     mp4_name = string(param.mp4);
%                 end
%             end
%             do_gif = false; gif_name = "";
%             if isfield(param, "gif") && ~isempty(param.gif)
%                 do_gif = true;
%                 if ~(islogical(param.gif) || isnumeric(param.gif))
%                     gif_name = string(param.gif);
%                 end
%             end
%             if do_mp4 || do_gif
%                 if ~exist(param.outdir, "dir"); mkdir(param.outdir); end
%             end
%             timestamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
%             if do_mp4 && mp4_name == ""
%                 mp4_name = fullfile(param.outdir, "Movie_" + timestamp + ".mp4");
%             end
%             if do_gif && gif_name == ""
%                 gif_name = fullfile(param.outdir, "Movie_" + timestamp + ".gif");
%             end
% 
%             p = obj.data_format(logger, param.target, "p", "p");
%             q = obj.data_format(logger, param.target, "q", "p");
%             u = logger.data(param.target, "input", "");
%             u = reshape(u, size(u,1), size(u,2), length(param.target));
%             Q = obj.gen_Q(param.target, q);
%             pL = obj.get_load_position(logger, param, p);
%             r = [];
%             try
%                 r = obj.data_format(logger, param.target, "p", "r");
%             catch
%                 r = [];
%             end
%             if ~isempty(r)
%                 for n = 1:size(r, 3)
%                     plot3(obj.ax, r(:,1,n), r(:,2,n), r(:,3,n), 'r--');
%                 end
%             end
%             t = logger.data(0, "t", "");
%             phase = [];
%             try
%                 phase = logger.data(0, "phase", "");
%             catch
%                 phase = [];
%             end
%             fig = ancestor(obj.ax,'figure');
%             fig.Units = "pixels";
%             fig.Position(3:4) = [1280 720];
% 
%             v = [];
%             if do_mp4
%                 v = VideoWriter(mp4_name, "MPEG-4");
%                 v.FrameRate = param.fps;
%                 v.Quality = param.quality;
%                 open(v);
%                 cleaner = onCleanup(@() safe_close(v));
%             end
% 
%             first_gif_written = false;
%             for i = 1:param.skip:(length(t)-1)
%                 if ~isvalid(obj.frame)
%                     obj = obj.gen_frame("target", param.target, "ax", obj.ax);
%                 end
%                 obj.draw(param.target, p(i,:,param.target), Q(i,:,param.target), u(i,:,param.target), pL(i,:));
%                 timeText = sprintf("%05.2f", t(i));
%                 phaseChar = "";
%                 if ~isempty(phase)
%                     phaseChar = char(phase(i));
%                 end
%                 title(obj.ax, "time : " + timeText + "  phase : " + phaseChar);
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
%                 if do_mp4 || do_gif
%                     fig = ancestor(obj.ax,'figure');
%                     fr = getframe(fig);
%                     if do_mp4
%                         writeVideo(v, fr);
%                     end
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
%                 if param.pause > 0
%                     pause(param.pause);
%                 end
%             end
% 
%             if do_mp4
%                 safe_close(v);
%                 fprintf("[DRAW_SUSPENDED_LOAD] saved mp4: %s\n", mp4_name);
%             end
%             if do_gif
%                 fprintf("[DRAW_SUSPENDED_LOAD] saved gif: %s\n", gif_name);
%             end
% 
%             function safe_close(vw)
%                 try
%                     if ~isempty(vw); close(vw); end
%                 catch
%                 end
%             end
%         end
%     end
% 
%     methods (Access = private)
%         function tf = has_key(~, args, key)
%             tf = false;
%             if isempty(args); return; end
%             if isstruct(args); tf = isfield(args, key); return; end
%             keys = string(args(1:2:end));
%             tf = any(keys == key);
%         end
%         function p = data_format(~, logger, source, var, att)
%             q = logger.data(source, var, att);
%             p = reshape(q, size(q,1), size(q,2)/length(source), length(source));
%         end
%         function pL = get_load_position(obj, logger, param, p)
%             if isfield(param, "load_source")
%                 load_source = param.load_source;
%             elseif isfield(param, "target")
%                 load_source = param.target;
%             else
%                 load_source = 1;
%             end
%             if isfield(param, "load_var")
%                 load_att = "e";
%                 if isfield(param, "load_att")
%                     load_att = param.load_att;
%                 end
%                 pL = obj.data_format(logger, load_source, param.load_var, load_att);
%                 pL = reshape(pL, size(p,1), 3);
%                 return
%             end
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
%             if isfield(param, "self") && isprop(param.self, "parameter")
%                 L = param.self.parameter.get("cableL");
%             else
%                 L = 0;
%             end
%             pL = p + pT * L;
%         end
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

classdef DRAW_SUSPENDED_LOAD
    % Animation class for single suspended-load (1 drone + 1 load)
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
        
        % =========================================================
        % CBF Safety Visualizations (コントローラと完全同期)
        % =========================================================
        cbf_type = 2;       % 0:None, 1:3-Spheres, 2:Ellipsoid
        cbf_handles = [];
        cbf_r_safe = 0.35;  % a_sys (短軸半径)
        b_sys = [];         % 長軸半径 (自動計算)
        cableL = 2.0;       % ケーブル長
    end
    methods
        function obj = DRAW_SUSPENDED_LOAD(logger, varargin)
            param = struct(varargin{:});
            if ~isfield(param, "target")
                param.target = 1;
            end
            if ~isfield(param, "self") || ~isprop(param.self, "parameter")
                error("DRAW_SUSPENDED_LOAD:MissingSelf", "self.parameter is required.");
            end
            if isprop(param.self.parameter, "Lx") && isprop(param.self.parameter, "Ly")
                obj.frame_size = [param.self.parameter.Lx, param.self.parameter.Ly];
            else
                error("DRAW_SUSPENDED_LOAD:MissingFrameSize", "self.parameter.Lx/Ly are required.");
            end
            if isprop(param.self.parameter, "rotor_r")
                obj.rotor_r = param.self.parameter.rotor_r;
            else
                error("DRAW_SUSPENDED_LOAD:MissingRotor", "self.parameter.rotor_r is required.");
            end
            if isprop(param.self.parameter, "Length")
                obj.load_size = repmat(param.self.parameter.Length, 1, 3);
            else
                error("DRAW_SUSPENDED_LOAD:MissingLoadSize", "self.parameter.Length is required.");
            end
            if isprop(param.self.parameter, "cableL")
                obj.cableL = param.self.parameter.get("cableL");
            end
            
            % =====================================================
            % コントローラから幾何パラメータを自動抽出・同期
            % =====================================================
            delta_long_val = 0.35;
            try
                ctrl_hlc = param.self.controller.hlc_suspended;
                if isprop(ctrl_hlc, 'GEOM_TYPE')
                    obj.cbf_type = ctrl_hlc.GEOM_TYPE;
                end
                if isprop(ctrl_hlc, 'a_sys')
                    obj.cbf_r_safe = ctrl_hlc.a_sys;
                elseif isprop(ctrl_hlc, 'r_safe_sphere')
                    obj.cbf_r_safe = ctrl_hlc.r_safe_sphere;
                end
                if isprop(ctrl_hlc, 'delta_long')
                    delta_long_val = ctrl_hlc.delta_long;
                end
            catch
                % コントローラから直接取得できない場合のフォールバック値
            end
            obj.b_sys = (obj.cableL / 2.0) + delta_long_val;
            
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
            obj.line = plot3(obj.ax, [p0(1) pL0(1)], [p0(2) pL0(2)], [p0(3) pL0(3)], "k", 'LineWidth', 1.5);
            
            % 防護メッシュの生成
            obj = obj.gen_cbf_mesh();
            
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
                        "FaceAlpha", 0.8, "EdgeColor", "none");
                otherwise
                    [x, y, z] = sphere(16);
                    x = x * obj.load_size(1);
                    y = y * obj.load_size(2);
                    z = z * obj.load_size(3);
                    h = surf(obj.ax, x, y, z, "FaceColor", "cyan", "FaceAlpha", 0.8, "EdgeColor", "none");
            end
            t = hgtransform('Parent', obj.ax);
            set(h, 'Parent', t);
            obj.load = t;
        end
        
        function obj = gen_cbf_mesh(obj)
            if obj.cbf_type == 2
                % 動的回転楕円体 (Ellipsoid)
                a_rad = obj.cbf_r_safe;
                b_rad = obj.b_sys;
                [xe, ye, ze] = ellipsoid(obj.ax, 0, 0, 0, a_rad, a_rad, b_rad, 24);
                h_cbf = surf(obj.ax, xe, ye, ze, 'FaceColor', [0.2 0.8 0.2], ...
                    'FaceAlpha', 0.20, 'EdgeColor', [0.1 0.6 0.1], 'EdgeAlpha', 0.15);
                t_cbf = hgtransform('Parent', obj.ax);
                set(h_cbf, 'Parent', t_cbf);
                obj.cbf_handles = t_cbf;
                
            elseif obj.cbf_type == 1
                % 複数球 (3-Spheres)
                [xs, ys, zs] = sphere(16);
                r_s = obj.cbf_r_safe;
                obj.cbf_handles = [];
                for i = 1:3
                    h_s = surf(obj.ax, xs*r_s, ys*r_s, zs*r_s, 'FaceColor', [0.2 0.6 1.0], ...
                        'FaceAlpha', 0.20, 'EdgeColor', 'none');
                    t_s = hgtransform('Parent', obj.ax);
                    set(h_s, 'Parent', t_s);
                    obj.cbf_handles = [obj.cbf_handles, t_s];
                end
            end
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
                
                % =====================================================
                % 自機防護領域のリアルタイム追従更新
                % =====================================================
                p_drone = p(1,:,1)';
                p_ld    = pL(1,:)';
                p_mid   = (p_drone + p_ld) / 2.0;
                
                if obj.cbf_type == 2 && ~isempty(obj.cbf_handles)
                    vec_tether = p_drone - p_ld;
                    len_t = norm(vec_tether);
                    if len_t > 1e-4
                        pT = vec_tether / len_t;
                    else
                        pT = [0; 0; 1];
                    end
                    
                    z_axis = [0; 0; 1];
                    rot_axis = cross(z_axis, pT);
                    norm_axis = norm(rot_axis);
                    
                    T_mid = makehgtform('translate', p_mid');
                    if norm_axis > 1e-4
                        rot_angle = atan2(norm_axis, dot(z_axis, pT));
                        R_ellip = makehgtform('axisrotate', rot_axis' / norm_axis, rot_angle);
                        set(obj.cbf_handles, 'Matrix', T_mid * R_ellip);
                    else
                        if dot(z_axis, pT) < 0
                            R_ellip = makehgtform('xrotate', pi);
                            set(obj.cbf_handles, 'Matrix', T_mid * R_ellip);
                        else
                            set(obj.cbf_handles, 'Matrix', T_mid);
                        end
                    end
                    
                elseif obj.cbf_type == 1 && length(obj.cbf_handles) == 3
                    set(obj.cbf_handles(1), 'Matrix', makehgtform('translate', p_drone'));
                    set(obj.cbf_handles(2), 'Matrix', makehgtform('translate', p_mid'));
                    set(obj.cbf_handles(3), 'Matrix', makehgtform('translate', p_ld'));
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
                    plot3(obj.ax, r(:,1,n), r(:,2,n), r(:,3,n), 'r--');
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
                fprintf("[DRAW_SUSPENDED_LOAD] saved mp4: %s\n", mp4_name);
            end
            if do_gif
                fprintf("[DRAW_SUSPENDED_LOAD] saved gif: %s\n", gif_name);
            end
            
            function safe_close(vw)
                try
                    if ~isempty(vw); close(vw); end
                catch
                end
            end
        end
    end
    
    methods (Access = private)
        function tf = has_key(~, args, key)
            tf = false;
            if isempty(args); return; end
            if isstruct(args); tf = isfield(args, key); return; end
            keys = string(args(1:2:end));
            tf = any(keys == key);
        end
        function p = data_format(~, logger, source, var, att)
            q = logger.data(source, var, att);
            p = reshape(q, size(q,1), size(q,2)/length(source), length(source));
        end
        function pL = get_load_position(obj, logger, param, p)
            if isfield(param, "load_source")
                load_source = param.load_source;
            elseif isfield(param, "target")
                load_source = param.target;
            else
                load_source = 1;
            end
            if isfield(param, "load_var")
                load_att = "e";
                if isfield(param, "load_att")
                    load_att = param.load_att;
                end
                pL = obj.data_format(logger, load_source, param.load_var, load_att);
                pL = reshape(pL, size(p,1), 3);
                return
            end
            candidates = { ...
                {"estimator.result.state.pL", "e"}, ...
                {"pL", "p"} ...
                };
            for k = 1:length(candidates)
                item = candidates{k};
                try
                    pL = obj.data_format(logger, load_source, item{1}, item{2});
                    pL = reshape(pL, size(p,1), 3);
                    return
                catch
                end
            end
            pT = [];
            candidates = { ...
                {"estimator.result.state.pT", "e"}, ...
                {"pT", "p"} ...
                };
            for k = 1:length(candidates)
                item = candidates{k};
                try
                    pT = obj.data_format(logger, load_source, item{1}, item{2});
                    pT = reshape(pT, size(p,1), 3);
                    break
                catch
                end
            end
            if isempty(pT)
                error("DRAW_SUSPENDED_LOAD:MissingLoad", "Load position data is missing.");
            end
            if isfield(param, "self") && isprop(param.self, "parameter")
                L = param.self.parameter.get("cableL");
            else
                L = 0;
            end
            pL = p + pT * L;
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