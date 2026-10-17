classdef DRAW_SUSPENDED_LOAD_HOCBF_ELLIPSOID_LINK_XY < handle
    % 静止・任意姿勢楕円体障害物と紐姿勢楕円体を描画する専用DRAW。
    % 既存のコンストラクタ/animation(logger,"target",1,"self",agent,"rl",...)に互換。
    % 描画データはHLC.historyを使用し、LOGGERの内部構造には依存しない。
    % 黄色の境界は衝突面そのものではなく、保守的なCBF排除領域。
    properties
        self
        target
        rl
        maxFrames
        pause_s
        showExclusion
        mp4
    end

    methods
        function obj = DRAW_SUSPENDED_LOAD_HOCBF_ELLIPSOID_LINK_XY(~, varargin)
            obj.self = [];
            obj.target = 1;
            obj.rl = 0.04;
            obj.maxFrames = 240;
            obj.pause_s = 0.01;
            obj.showExclusion = true;
            obj.mp4 = false;
            obj.setOptions(varargin{:});
        end

        function animation(obj, ~, varargin)
            obj.setOptions(varargin{:});
            assert(~isempty(obj.self), 'DRAWには"self",agentが必要です。');
            ctrl = obj.self.controller.hlc_suspended;
            H = ctrl.history;
            assert(H.k>0, 'HLC.historyが空です。');
            obs = ENVIRONMENT_OBSTACLE_ELLIPSOID_STATIC();
            Nobs = numel(obs);
            N = H.k;
            L = obj.self.parameter.cableL;
            [sx,sy,sz] = sphere(16);
            un = [sx(:)';sy(:)';sz(:)'];
            meshSize = size(sx);

            fig = figure('Name','DRAW: Suspended Load / Oriented Ellipsoid HOCBF', ...
                'Color','w');
            ax = axes('Parent',fig);
            hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,3);
            xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');

            % 真の障害物形状：回転R_obsと主軸半径をそのまま使用。
            exclusionSurf = gobjects(1,Nobs);
            for i=1:Nobs
                R = obs(i).R_obs;
                r = obs(i).ellipsoid_radii(:);
                [XX,YY,ZZ] = transformSphere(obs(i).p_obs,R*diag(r),un,meshSize);
                surf(ax,XX,YY,ZZ,'FaceColor',[0.85 0.25 0.20], ...
                    'FaceAlpha',0.34,'EdgeColor',[0.7 0.2 0.2], ...
                    'EdgeAlpha',0.10,'DisplayName',sprintf('Obstacle %d (physical)',i));
                if obj.showExclusion
                    % h=0境界: p_midの禁入領域（障害物楕円体そのものではない）
                    exclusionSurf(i) = surf(ax,nan(size(sx)),nan(size(sy)),nan(size(sz)), ...
                        'FaceColor',[0.98 0.74 0.13],'FaceAlpha',0.075, ...
                        'EdgeColor',[0.75 0.55 0.1],'EdgeAlpha',0.035, ...
                        'DisplayName',sprintf('Obstacle %d CBF outer boundary',i));
                end
            end

            % 紐カプセルを包む長軸楕円体（参考表示）。
            % 表示上は全障害物の中で最大marginを使用し、CBFは障害物ごとに評価。
            margins = [obs.d_margin];
            if isempty(margins), rho=obj.rl; else, rho=obj.rl+max(margins); end
            a = L/2+rho;
            b = sqrt(rho*a);
            ropeEllip = surf(ax,nan(size(sx)),nan(size(sy)),nan(size(sz)), ...
                'FaceColor',[0.05 0.7 0.85],'FaceAlpha',0.16, ...
                'EdgeAlpha',0.04,'DisplayName','Cable envelope + margin');
            ropeLine = plot3(ax,nan,nan,nan,'k-','LineWidth',2.1, ...
                'DisplayName','Cable centerline');
            loadMark = plot3(ax,nan,nan,nan,'ko','MarkerFaceColor','k', ...
                'MarkerSize',6,'DisplayName','Load');
            quadMark = plot3(ax,nan,nan,nan,'k^','MarkerFaceColor',[0.2 0.2 0.2], ...
                'MarkerSize',7,'DisplayName','Quadrotor');
            traj = plot3(ax,nan,nan,nan,'b-','LineWidth',1.3, ...
                'DisplayName','Load trajectory');
            refLine = plot3(ax,H.refXYZ(1,:),H.refXYZ(2,:),H.refXYZ(3,:), ...
                'b--','LineWidth',0.7,'DisplayName','Reference (controller cycles)');
            legend(ax,'Location','bestoutside');

            % 全体が見える軸範囲を静的に決定。
            xyz=[H.pL,H.pQuad,H.refXYZ];
            for i=1:Nobs
                center=obs(i).p_obs(:);
                reach=norm(obs(i).ellipsoid_radii(:))+L+rho;
                xyz=[xyz,center+reach*eye(3),center-reach*eye(3)]; %#ok<AGROW>
            end
            amin=min(xyz,[],2)-0.5;
            amax=max(xyz,[],2)+0.5;
            xlim(ax,[amin(1),amax(1)]);
            ylim(ax,[amin(2),amax(2)]);
            zlim(ax,[amin(3),amax(3)]);

            % mp4は必要時だけ有効化。既定ではMAT/CSVだけ自動保存。
            vw = [];
            if obj.mp4
                outputDir=fullfile(pwd,'simulation_logs');
                if ~exist(outputDir,'dir'), mkdir(outputDir); end
                try
                    path=fullfile(outputDir, ['EllipsoidHOCBF_XY_' datestr(now,'yyyymmdd_HHMMSS') '.mp4']);
                    vw=VideoWriter(path,'MPEG-4');
                    vw.FrameRate=15;
                    open(vw);
                    fprintf('[DRAW] MP4保存: %s\n',path);
                catch ME
                    warning('MP4を開始できないためアニメーション表示のみ続行: %s',ME.message);
                    vw=[];
                end
            end

            step=max(1,ceil(N/obj.maxFrames));
            frames=unique([1:step:N,N]);
            for k=frames
                if ~isgraphics(fig), break; end
                pl=H.pL(:,k); p=H.pT(:,k); p=p/norm(p);
                qpos=pl-L*p;
                mid=pl-0.5*L*p;
                set(ropeLine,'XData',[pl(1),qpos(1)], ...
                    'YData',[pl(2),qpos(2)],'ZData',[pl(3),qpos(3)]);
                set(loadMark,'XData',pl(1),'YData',pl(2),'ZData',pl(3));
                set(quadMark,'XData',qpos(1),'YData',qpos(2),'ZData',qpos(3));
                set(traj,'XData',H.pL(1,1:k),'YData',H.pL(2,1:k), ...
                    'ZData',H.pL(3,1:k));

                % pに沿って長軸a、法線方向に短軸bをとる。
                [~,axisId]=min(abs(p));
                en=zeros(3,1); en(axisId)=1;
                v=cross(p,en);v=v/norm(v);
                w=cross(p,v);
                [X,Y,Z]=transformSphere(mid,[p,v,w]*diag([a b b]),un,meshSize);
                set(ropeEllip,'XData',X,'YData',Y,'ZData',Z);
                if obj.showExclusion
                    for i=1:Nobs
                        S=ctrl.shape_cache{i}.C+ctrl.shape_cache{i}.kappa*(p*p.');
                        [V,D]=eig((S+S.')/2);
                        transform=V*diag(sqrt(max(diag(D),0)));
                        [X,Y,Z]=transformSphere(obs(i).p_obs,transform,un,meshSize);
                        set(exclusionSurf(i),'XData',X,'YData',Y,'ZData',Z);
                    end
                end
                title(ax,sprintf(['t = %.2f s (since HLC start),  min h = %.3g,  ' ...
                    'QP active=%d,  margin=%.2f m'], ...
                    (k-1)*H.dt,H.minCBFH(k),H.qpActive(k),max([0 margins])));
                drawnow limitrate;
                if ~isempty(vw)
                    try
                        writeVideo(vw,getframe(fig));
                    catch ME
                        warning('動画保存を中止します: %s',ME.message);
                        close(vw); vw=[];
                    end
                end
                if obj.pause_s>0, pause(obj.pause_s); end
            end
            if ~isempty(vw), close(vw); end
        end
    end

    methods (Access=private)
        function setOptions(obj,varargin)
            % 元DRAWからの呼び出し互換に必要なname-valueのみ処理。
            if mod(numel(varargin),2)~=0
                error('オプションは名前と値のペアで指定してください。');
            end
            for j=1:2:numel(varargin)
                key=lower(char(varargin{j}));
                value=varargin{j+1};
                switch key
                    case 'self'
                        obj.self=value;
                    case 'target'
                        obj.target=value;
                    case 'rl'
                        obj.rl=value;
                    case 'maxframes'
                        obj.maxFrames=max(1,round(value));
                    case 'pause'
                        obj.pause_s=value;
                    case 'showexclusion'
                        obj.showExclusion=logical(value);
                    case 'mp4'
                        obj.mp4=logical(value);
                    otherwise
                        warning('DRAW:未使用のオプション %s を無視します。',key);
                end
            end
        end
    end
end

function [X,Y,Z]=transformSphere(center,transform,U,meshSize)
    V=transform*U+center(:);
    X=reshape(V(1,:),meshSize);
    Y=reshape(V(2,:),meshSize);
    Z=reshape(V(3,:),meshSize);
end
