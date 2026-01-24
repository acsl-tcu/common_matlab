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
            obj.line = plot3(obj.ax, [p0(1) pL0(1)], [p0(2) pL0(2)], [p0(3) pL0(3)], "k");
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
                    h = surf(obj.ax, x, y, z, "FaceColor", "cyan", "FaceAlpha", 0.5, "EdgeColor", "none");
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

            if ~isempty(pL)
                Tload = makehgtform('translate', pL(1,:));
                set(obj.load, 'Matrix', Tload);
                obj.line.XData = [p(1,1,1) pL(1,1)];
                obj.line.YData = [p(1,2,1) pL(1,2)];
                obj.line.ZData = [p(1,3,1) pL(1,3)];
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
            for i = 1:length(t)-1
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
                pause(0.01);
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
