classdef CBF_TRAJ_REFERENCE < handle
% REV38: direct causal displacement correction; one order-seven return. Only the
% five local base-reference constructions are selectable.
    properties
        self
        cfg
        mode='NOMINAL'
        quietCount=0
        lastFlightTime=[]
        previousP=[]
        previousReference=[]
        comparison=[]
        correction=zeros(3,7)
        center=zeros(3,1)
        anchor=zeros(3,1)
        probeCounter=0
        correctionTransition
    end
    methods
        function obj=CBF_TRAJ_REFERENCE(agent,cfg)
            obj.self=agent;
            req={'dt','cbfId','referenceMethod','waypointRef','hlc'};
            if ~isstruct(cfg) || ~all(isfield(cfg,req))
                error('CBFTraj:Configuration','Use REV38 Sim settings in README');
            end
            obsolete={'correctionMethod','returnMethod','method','nSubsteps', ...
                'returnDuration','referenceProbeFraction','paperActuator', ...
                'sensitivityEpsilon','shadowMaxOffsetM','shadowMaxVelocityMps', ...
                'returnOmega','returnQuietCycles','settlePositionM','settleVelocityMps','settleDerivativeScaledM'};
            if any(isfield(cfg,obsolete)),error('CBFTraj:ObsoleteConfiguration','Remove old settings; use REV38 README');end
            validateattributes(cfg.dt,{'numeric'},{'scalar','real','finite','positive'});
            validateattributes(cfg.cbfId,{'numeric'},{'scalar','integer','>=',0,'<=',3});
            validateattributes(cfg.referenceMethod,{'numeric'},{'scalar','integer','>=',1,'<=',5});
            if ~isa(cfg.hlc,'HLC_SUSPENDED_LOAD_CBF_REAL')
                error('CBFTraj:Configuration','REV38 uses the existing real-input HLC pure functions');
            end
            if cfg.hlc.cbfConfig.cbfId~=cfg.cbfId || abs(cfg.hlc.param.dt-cfg.dt)>1e-10
                error('CBFTraj:Configuration','HLC ID/dt mismatch');
            end
            obj.cfg=cfg;
            obj.correctionTransition=transition(obj.rate(),cfg.dt);
        end
        function out=do(obj,varargin)
            started=tic;time=varargin{1};cha=char(varargin{2});t=double(time.t);
            base=obj.self.reference.result;
            if ~isstruct(base) || ~isfield(base,'state') || ~isprop(base.state,'xd')
                error('CBFTraj:ReferenceMissing','Run after SLOAD');
            end
            before=double(base.state.xd(:));deltaPV=zeros(6,1);fromTime=NaN;
            prev=CBF_TRAJ_PHYSICAL_DIAG('FIRST_FLIGHT_CYCLE');status=prev.status;
            if ~strcmp(cha,'f')
                obj.resetCorrection();obj.lastFlightTime=[];obj.previousP=[];obj.previousReference=[];
                out=makeOutput(obj,base,before,t,'NON_FLIGHT',deltaPV,prev,fromTime,cha,started);return
            end
            if numel(before)~=28 || any(~isfinite(before))
                error('CBFTraj:Reference28Invalid','Expected finite post-SLOAD 28x1 jet');
            end
            if ~isempty(obj.lastFlightTime)
                elapsed=t-obj.lastFlightTime;
                if abs(elapsed-obj.cfg.dt)>max(1e-8,1e-4*obj.cfg.dt)
                    status='NONREGULAR_FLIGHT_PERIOD';obj.degrade();
                else
                    fromTime=obj.lastFlightTime;
                    ctrl=obj.self.controller.result;
                    % Keep the original-reference baseline alive during RETURN.
                    % Do not restart a lost baseline until NOMINAL.
                    track=strcmp(obj.mode,'CARRY') || ...
                        (strcmp(obj.mode,'RETURN') && ~isempty(obj.comparison));
                    allow=strcmp(obj.mode,'NOMINAL') || track;
                    % In DEGRADED, the comparison stays latched off until
                    % a valid quiet return; do not restart and sum errors.
                    comp=obj.comparison;
                    [deltaPV,prev,nextComparison]=CBF_TRAJ_PHYSICAL_HOLD_DIFF( ...
                        ctrl,obj.previousP,obj.cfg.dt,obj.cfg.cbfId,obj.cfg.hlc,comp,track,allow,obj.previousReference);
                    status=obj.applyInterval(prev,nextComparison);
                end
            end
            obj.lastFlightTime=t;
            % Capture the physical parameters used by the immediately
            % following HLC, rather than using next-sample mass estimates.
            obj.previousP=obj.self.parameter.get(["mass","jx","jy","jz","gravity","loadmass","cableL"]);
            % WAYPOINT has already advanced by the previously held command.
            % Update only the NEXT held command, using the CURRENT corrected
            % post-SLOAD load position. Do not advance virtual time again.
            % In NOMINAL, keep the original WAYPOINT behavior untouched.
            if ~strcmp(obj.mode,'NOMINAL')
                wp=obj.cfg.waypointRef;
                if wp.adaptiveProgress
                    e=wp.load_error(before(1:3)+obj.correction(:,1));
                    wp.lastPositionError=e;
                    wp.heldRateCommand=CBF_TRAJ_PROGRESS_COMMAND(e,wp.slowdownStart,wp.slowdownFull,wp.minScale);
                    wp.lastProgressScale=wp.heldRateCommand;
                end
            end
            out=makeOutput(obj,base,before,t,status,deltaPV,prev,fromTime,cha,started);
            % Save both jets from this exact sample for the next interval.
            obj.previousReference=struct('before28',out.cbfTraj.before28, ...
                'after28',out.cbfTraj.after28);
        end
        function status=applyInterval(obj,prev,nextComparison)
            % The pure update is independently testable without live HLC.
            st=struct('mode',obj.mode,'correction',obj.correction, ...
                'center',obj.center,'anchor',obj.anchor,'quietCount',obj.quietCount);
            [st,status,keepComparison]=CBF_TRAJ_CORRECTION_STEP(st,prev,obj.correctionTransition);
            obj.mode=st.mode;obj.correction=st.correction;obj.center=st.center;
            obj.anchor=st.anchor;obj.quietCount=st.quietCount;
            if keepComparison,obj.comparison=nextComparison;else,obj.comparison=[];end
            if strcmp(obj.mode,'NOMINAL'),obj.resetCorrection();end
        end
        function degrade(obj)
            obj.quietCount=0;obj.comparison=[];
            if strcmp(obj.mode,'NOMINAL'),return;end
            % Missing time interval: retain the last valid position; never
            % extrapolate a stale velocity through an unknown interval.
            obj.correction(:,2:7)=0;obj.center=obj.correction(:,1);
            obj.mode='DEGRADED';
        end
        function w=rate(obj)
            % Existing REV20 return pole; not a CARRY tracking filter.
            w=1.5;
        end
        function resetCorrection(obj)
            obj.mode='NOMINAL';obj.correction=zeros(3,7);obj.center=zeros(3,1);
            obj.anchor=zeros(3,1);obj.comparison=[];obj.quietCount=0;obj.probeCounter=0;
        end
    end
end
function E=transition(w,dt)
A=zeros(7);A(1:6,2:7)=eye(6);
for j=0:6,A(7,j+1)=-nchoosek(7,j)*w^(7-j);end
E=expm(A*dt);
end
function out=makeOutput(obj,base,before,t,status,dpv,prev,fromTime,cha,started)
out=base;jet=zeros(size(before));after=before;
probe=struct('status','NOT_SAMPLED','s',NaN,'selected28',[],'direct28',[], ...
    'baseError28',[],'maxErrorByOrder',nan(1,7),'yawErrorByOrder',nan(1,7));
if strcmp(cha,'f') && ~strcmp(obj.mode,'NOMINAL')
    j=CBF_TRAJ_REFERENCE_EVAL(obj.cfg.referenceMethod,0,obj.cfg.dt,before,[]);
    jet=reshape([obj.correction;zeros(1,7)],28,1);after=j+jet;
    st=state_copy(base.state);st.xd=after;
    if isprop(st,'p'),st.p=after(1:3);end
    if isprop(st,'v'),st.v=after(5:7);end
    if isprop(st,'q'),st.q=[0;0;after(4)];end
    out.state=st;
    % Fixed diagnostic cadence (first active tick and every 20 ticks).
    if mod(obj.probeCounter,20)==0,probe=referenceProbe(obj,before);end
    obj.probeCounter=obj.probeCounter+1;
end
b28=[];a28=[];d28=[];
if numel(before)==28,b28=before;a28=after;d28=jet;end
out.cbfTraj=struct('time',t,'sourceIntervalStart',fromTime,'sourceIntervalEnd',t, ...
    'sourceStatus',status,'cbfId',obj.cfg.cbfId,'mode',obj.mode, ...
    'before28',b28,'after28',a28,'delta28',d28, ...
    'signedCarryPosition',obj.correction(:,1),'signedCarryVelocity',obj.correction(:,2), ...
    'correctionTarget',obj.center,'episodeAnchor',obj.anchor, ...
    'lastIntervalDeltaPV',dpv,'quietCount',obj.quietCount, ...
    'physicalModel','RK4','physicalPrevious',prev, ...
    'interSampleContinuityRequired',false,'safetyCertified',false, ...
    'phase',cha,'enabled',true,'beforeXd',before,'afterXd',after, ...
    'referenceMethod',obj.cfg.referenceMethod,'referenceProbe',probe, ...
    'correctionRevision',38,'elapsedMs',1000*toc(started));
if strcmp(cha,'f')
    wp=obj.cfg.waypointRef;
    out.cbfTraj.governorRate=wp.rateState(6);
    out.cbfTraj.governorCommand=wp.heldRateCommand;
    out.cbfTraj.governorError=wp.lastPositionError;
    out.cbfTraj.virtualTime=wp.virtualTime;
end
end
function p=referenceProbe(obj,before)
% Optional diagnostic only. Evaluate the actual generator under its known
% current held governor command over the next local 25ms interval. Never
% feed the probe into HLC or use it to claim a future plant trajectory.
p=struct('status','NOT_EVALUATED','s',NaN,'selected28',[],'direct28',[], ...
    'baseError28',[],'maxErrorByOrder',nan(1,7),'yawErrorByOrder',nan(1,7));
try
    wp=obj.cfg.waypointRef;H=obj.cfg.dt;s=H/2;
    if wp.adaptiveProgress
        [~,tau,td]=wp_governor_step(wp.rateState,wp.virtualTime,wp.heldRateCommand,0,wp.filterTime);
        raw=wp.generator.evaluateWarped(tau,td);
        evaluator=@(u) warpedAt(wp,u);
    else
        tau=max(0,obj.lastFlightTime-wp.t);
        raw=wp.generator.evaluate(tau);
        evaluator=@(u) wp.generator.evaluate(tau+u);
    end
    off=before-raw(:);
    % SLOAD shifts only position; constant post-SLOAD offset over local step.
    if any(abs(off(4:end))>1e-8*(1+abs(before(4:end))))
        p.status='BASE_JET_MISMATCH';return
    end
    off(4:end)=0;fn=@(u) reshape(evaluator(u),28,1)+off;
    direct=fn(s);selected=CBF_TRAJ_REFERENCE_EVAL(obj.cfg.referenceMethod,s,H,before,fn);
    j=CBF_TRAJ_LOCAL_CORRECTION_JET(obj.mode,obj.correction,s,obj.rate());
    add=reshape([j;zeros(1,7)],28,1);
    p.s=s;p.direct28=direct+add;p.selected28=selected+add;
    p.baseError28=selected-direct;
    err=reshape(p.baseError28,4,7);
    p.maxErrorByOrder=max(abs(err(1:3,:)),[],1);
    p.yawErrorByOrder=abs(err(4,:));
    p.status='OK';
catch ME
    p.status=['PROBE_FAILED:' ME.identifier];
end
end
function v=warpedAt(wp,s)
[~,tau,td]=wp_governor_step(wp.rateState,wp.virtualTime,wp.heldRateCommand,s,wp.filterTime);
v=wp.generator.evaluateWarped(tau,td);
end
