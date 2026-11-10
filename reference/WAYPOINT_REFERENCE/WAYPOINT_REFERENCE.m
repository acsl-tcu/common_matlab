classdef WAYPOINT_REFERENCE < handle
    %WAYPOINT_REFERENCE Flight-only reference, same output contract as TIME_VARYING_REFERENCE.
    % 28x1 = [p(3);yaw; dp(3);dyaw; ... ; d6p(3);d6yaw].
    %
    % {generatorName, {generator parameters}, 6, progressSettings}
    % progressSettings (optional) has fields:
    %  enabled, slowdownStart, slowdownFull, minScale, filterTime
    %
    % Time profile starts/stops automatically based on derivative bounds.
    % Feedback governs VIRTUAL TIME based on estimated suspended LOAD position.
    % The 6-stage rate filter keeps virtual time C6 even when feedback changes.
    % NOTE: This is a reference governor, NOT a safety guarantee or CBF.
    properties
        self
        generator
        result
        t = []
        cha = 's'
        adaptiveProgress = true
        slowdownStart = 0.30
        slowdownFull = 0.90
        minScale = 0.05
        filterTime = 0.70
        virtualTime = 0
        rateState = ones(6,1)
        heldRateCommand = 1
        lastFlightTime = []
        lastPositionError = NaN
        lastProgressScale = 1
    end
    methods
        function obj = WAYPOINT_REFERENCE(self,args)
            if ~iscell(args) || numel(args)<2 || ~iscell(args{2})
                error('WaypointRef:Config', 'Use {generatorName,{name,value,...},6[,progressSettings]}.');
            end
            if numel(args)>2 && (~isscalar(args{3}) || args{3}~=6)
                error('WaypointRef:Order', 'Waypoint generator requires order=6.');
            end
            obj.self = self;
            genName = str2func(char(args{1}));
            routeParams = args{2};
            obj.generator = genName(routeParams{:});
            if ~isstruct(obj.generator) || ~isfield(obj.generator,'evaluate') || ...
                    ~isfield(obj.generator,'evaluateWarped') || ...
                    ~isfield(obj.generator,'order') || obj.generator.order~=6
                error('WaypointRef:InvalidGenerator','Generator must provide C6 evaluate / evaluateWarped.');
            end
            if numel(args)>=4
                opts=args{4};
                if ~isstruct(opts) || ~isscalar(opts)
                    error('WaypointRef:ProgressConfig','Fourth argument must be a scalar settings struct.');
                end
                if isfield(opts,'enabled'), obj.adaptiveProgress=logical(opts.enabled); end
                if isfield(opts,'slowdownStart'), obj.slowdownStart=opts.slowdownStart; end
                if isfield(opts,'slowdownFull'), obj.slowdownFull=opts.slowdownFull; end
                if isfield(opts,'minScale'), obj.minScale=opts.minScale; end
                if isfield(opts,'filterTime'), obj.filterTime=opts.filterTime; end
            end
            if ~isscalar(obj.adaptiveProgress) || ...
                    ~isscalar(obj.slowdownStart) || ~isfinite(obj.slowdownStart) || obj.slowdownStart<0 || ...
                    ~isscalar(obj.slowdownFull) || ~isfinite(obj.slowdownFull) || obj.slowdownFull<=obj.slowdownStart || ...
                    ~isscalar(obj.minScale) || ~isfinite(obj.minScale) || obj.minScale<0 || obj.minScale>1 || ...
                    ~isscalar(obj.filterTime) || ~isfinite(obj.filterTime) || obj.filterTime<=0
                error('WaypointRef:GovernorSettings','Invalid progress governor settings.');
            end
            xd=obj.generator.evaluate(0);
            if ~isequal(size(xd),[28,1]) || any(~isfinite(xd))
                error('WaypointRef:ReferenceShape','Generator must return finite 28x1 xd.');
            end
            obj.result.state=STATE_CLASS(struct(...
                'state_list',["xd","p","q","v"], 'num_list',[28,3,3,3]));
            obj.result.state.set_state('xd',xd);
            obj.result.state.set_state('p',xd(1:3));
            obj.result.state.set_state('v',xd(5:7));
            obj.result.state.set_state('q',[0;0;xd(4)]);
        end

        function result=do(obj,varargin)
            time=varargin{1};
            obj.cha=varargin{2};
            if strcmp(char(obj.cha),'f')
                if isempty(obj.t)
                    obj.t=time.t;
                    obj.lastFlightTime=time.t;
                    obj.virtualTime=0;
                    obj.rateState=ones(6,1);
                    obj.heldRateCommand=1;
                end
                if obj.adaptiveProgress
                    dt=max(0,time.t-obj.lastFlightTime);
                    [obj.rateState,obj.virtualTime,tauDer] = wp_governor_step( ...
                        obj.rateState,obj.virtualTime,obj.heldRateCommand,dt,obj.filterTime);
                    obj.lastFlightTime=time.t;
                    xd=obj.generator.evaluateWarped(obj.virtualTime,tauDer);
                    % Compare on SAME coordinates as SUSPENDED_LOAD_REF_ADJUST:
                    % Flight desired load height = raw waypoint height - cableL.
                    rawTargetLoad=xd(1:3);
                    L=obj.self.parameter.cableL;
                    rawTargetLoad(3)=rawTargetLoad(3)-L;
                    e=obj.load_error(rawTargetLoad);
                    obj.lastPositionError=e;
                    if isfinite(e)
                        u=max(0,min(1,(e-obj.slowdownStart)/ ...
                            (obj.slowdownFull-obj.slowdownStart)));
                        % Smoothstep blend avoids abrupt command changes.
                        u=u*u*(3-2*u);
                        obj.heldRateCommand=1-(1-obj.minScale)*u;
                    else
                        obj.heldRateCommand=1; % estimation unavailable
                    end
                    obj.lastProgressScale=obj.heldRateCommand;
                else
                    tau=max(0,time.t-obj.t);
                    xd=obj.generator.evaluate(tau);
                    obj.lastPositionError=NaN;
                    obj.lastProgressScale=1;
                end
            else
                xd=obj.generator.evaluate(0);
            end
            obj.result.state.xd=xd;
            obj.result.state.p=xd(1:3);
            obj.result.state.v=xd(5:7);
            obj.result.state.q=[0;0;xd(4)];
            result=obj.result;
        end

        function e=load_error(obj,pLoadTarget)
            e=NaN;
            try
                st=obj.self.estimator.result.state;
                if ~isempty(st) && isprop(st,'pL')
                    p=st.pL;
                    if isequal(size(p),[3,1]) && all(isfinite(p))
                        e=norm(pLoadTarget-p);
                    end
                end
            catch
                % No valid estimate: proceed nominally without adaptation.
            end
        end

        function show(~,logger)
            rp=logger.data(1,'p','r');
            ep=logger.data(1,'p','e');
            plot3(rp(:,1),rp(:,2),rp(:,3)); hold on;
            plot3(ep(:,1),ep(:,2),ep(:,3)); hold off;
            daspect([1 1 1]); legend('reference','estimate');
            xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
        end
    end
end
