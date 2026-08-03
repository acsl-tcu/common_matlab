classdef DRAW_SUSPENDED_LOAD_HOCBF_Z_LINK_EL
    % 障害物回避対応版：ドローン＋吊り荷の3次元アニメーション表示クラス
    % システム全体（ドローン＋ケーブル）を包む「姿勢追従型の回転楕円体バリア」と、
    % 障害物環境を半透明で可視化します。
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
        
        % --- 回避表示用のグラフィックス・ハンドル群 ---
        obs_handles = [];          % 障害物の表示オブジェクト
        protection_ellipsoid = []; % 姿勢連動するシステム回転楕円体バリア (hgtransform)
        sys_params = [];           % [a_sys; b_sys] 楕円体の半軸長
    end
    methods
        function obj = DRAW_SUSPENDED_LOAD_HOCBF_Z_LINK_EL(logger, varargin)
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
            
            %% 🧱 静的障害物環境のプロット初期化
            try
                obs_list = ENVIRONMENT_OBSTACLE_HOCBF_Z();
                [sx, sy, sz] = sphere(24);
                hold(obj.ax, "on");
                
                for i = 1:length(obs_list)
                    xo = obs_list(i).p_obs;
                    ro = obs_list(i).r_obs;
                    
                    % 障害物本体（赤）
                    obj.obs_handles(end+1) = surf(obj.ax, sx*ro + xo(1), sy*ro + xo(2), sz*ro + xo(3), ...
                        'FaceColor', [0.8 0.1 0.1], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
                    
                    % 近似領域（半透明オレンジ）
                    obj.obs_handles(end+1) = surf(obj.ax, sx*ro + xo(1), sy*ro + xo(2), sz*ro + xo(3), ...
                        'FaceColor', [1.0 0.5 0.0], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
                end
            catch
                disp('⚠️ [Visualizer ERROR] 障害物グラフィックスの生成に失敗しました。');
            end

            %% 🛡️ 姿勢連動型「回転楕円体バリア」グラフィックス初期化
            if isfield(param, "a_sys") && isfield(param, "b_sys")
                a_sys = param.a_sys;
                b_sys = param.b_sys;
            else
                % 引数未指定時のデフォルト（ケーブル長依存）
                L_cable = param.self.parameter.get("cableL");
                a_sys = L_cable / 2 + 0.3; % 長軸
                b_sys = 0.4;               % 短軸
            end
            obj.sys_params = [a_sys; b_sys];
            
            % 原点中心に回転楕円体メッシュを作成 (a_sys: Z方向長軸, b_sys: X/Y方向短軸)
            [ex, ey, ez] = ellipsoid(0, 0, 0, b_sys, b_sys, a_sys, 20);
            
            % 楕円体サーフェスオブジェクトを作成
            h_ellip = surf(obj.ax, ex, ey, ez, ...
                'FaceColor', [0.2 0.8 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
            
            % 姿勢回転・平行移動制御用の hgtransform をバインド
            t_ellip = hgtransform('Parent', obj.ax);
            set(h_ellip, 'Parent', t_ellip);
            obj.protection_ellipsoid = t_ellip;
            
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
                
                %% 🔄 【修正】ケーブル方向 vector pT に回転楕円体バリアをピッタリ同期
                p_drone = p(1,:,1)';
                p_load  = pL(1,:)';
                p_mid   = 0.5 * (p_drone + p_load); % リンク中点
                
                % ケーブル方向の単位ベクトル pT (ドローン -> 荷物)
                delta_p = p_load - p_drone;
                if norm(delta_p) > 1e-6
                    pT = delta_p / norm(delta_p);
                else
                    pT = [0; 0; -1]; % デフォルト真下
                end
                
                % 楕円のデフォルト長軸方向 [0; 0; 1] から pT への回転行列を計算
                z_axis = [0; 0; 1];
                v_rot = cross(z_axis, pT); % 回転軸
                s_rot = norm(v_rot);      % sin(theta)
                c_rot = dot(z_axis, pT);   % cos(theta)
                
                if s_rot > 1e-6
                    v_rot_unit = v_rot / s_rot;
                    theta_rot = atan2(s_rot, c_rot);
                    R_ellip = makehgtform('axisrotate', v_rot_unit, theta_rot);
                elseif c_rot < 0
                    % 180度反対（真下を向いている場合）
                    R_ellip = makehgtform('xrotate', pi);
                else
                    R_ellip = eye(4);
                end
                
                % 中点への平行移動行列
                T_mid = makehgtform('translate', p_mid);
                
                % 🌟 移動＋回転行列を結合して楕円バリアにセット
                set(obj.protection_ellipsoid, 'Matrix', T_mid * R_ellip);
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