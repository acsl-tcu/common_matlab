classdef CBF_TRAJ_REFERENCE < handle
% Sim-side WAYPOINT decoration. Runs AFTER WAYPOINT and SLOAD reference.
% Reads previous interval's already-existing controller.result.{x,tmp,input,cbf};
% updates signed [position;velocity] correction and passes a new 28-jet
% to the UNMODIFIED HLC. Works only in flight, never modifies controllers.
% Every phase emits cbfTraj; non-flight references retain original size.
    properties
        self
        cfg
        d=zeros(3,1)
        w=zeros(3,1)
        mode='NOMINAL'
        quietCount=0
        lastFlightTime=[]
    end
    methods
        function obj=CBF_TRAJ_REFERENCE(agent,cfg)
            obj.self=agent;
            must={'dt','cbfId','method','nSubsteps','returnOmega', ...
                'returnQuietCycles','settlePositionM','settleVelocityMps'};
            if ~isstruct(cfg) || ~all(isfield(cfg,must))
                error('CBFTraj:Configuration','Incomplete Sim trajectory correction configuration');
            end
            if ~isfield(cfg,'enabled'),cfg.enabled=true;end
            validateattributes(cfg.enabled,{'logical','numeric'},{'scalar','finite'});
            cfg.enabled=logical(cfg.enabled);
            obj.cfg=cfg;
        end
        function out=do(obj,varargin)
            time=varargin{1};
            cha=char(varargin{2});
            base=obj.self.reference.result;
            if ~isstruct(base) || ~isfield(base,'state') || ...
                    ~isprop(base.state,'xd')
                error('CBFTraj:ReferenceMissing', ...
                    'CBF trajectory reference must run AFTER SLOAD/waypoint');
            end
            beforeXd=double(base.state.xd(:));
            currentTime=double(time.t);
            % Non-flight generators supply 20 entries, not 28. Never resize
            % or overwrite their state, even when trajectory correction is ON.
            if ~strcmp(cha,'f') || ~obj.cfg.enabled
                obj.d=zeros(3,1);obj.w=zeros(3,1);
                obj.mode='NOMINAL';obj.quietCount=0;obj.lastFlightTime=[];
                if ~strcmp(cha,'f'),status='NON_FLIGHT';else,status='DISABLED';end
                out=makeOutput(obj,base,beforeXd,currentTime,status, ...
                    zeros(6,1),emptyPhysical(status),NaN,cha);
                return
            end
            before28=beforeXd;
            if numel(before28)~=28 || any(~isfinite(before28))
                error('CBFTraj:Reference28Invalid','Expected finite base 28x1 after SLOAD');
            end
            deltaPV=zeros(6,1);fromTime=NaN;
            previous=emptyPhysical('FIRST_FLIGHT_CYCLE');
            updateStatus='FIRST_FLIGHT_CYCLE';
            if ~isempty(obj.lastFlightTime)
                elapsed=currentTime-obj.lastFlightTime;
                if elapsed<0 || abs(elapsed-obj.cfg.dt)>max(1e-8,1e-4*obj.cfg.dt)
                    updateStatus='NONREGULAR_FLIGHT_PERIOD';
                    obj.quietCount=0;
                else
                    fromTime=obj.lastFlightTime;
                    ctrl=obj.self.controller.result;
                    P=obj.self.parameter.get( ...
                        ["mass","jx","jy","jz","gravity","loadmass","cableL"]);
                    [deltaPV,previous]=CBF_TRAJ_PHYSICAL_HOLD_DIFF( ...
                        ctrl,P,obj.cfg.dt,obj.cfg.cbfId, ...
                        obj.cfg.method,obj.cfg.nSubsteps);
                    updateStatus=previous.status;
                    obj=applyLastInterval(obj,deltaPV,previous);
                end
            end
            obj.lastFlightTime=currentTime;
            out=makeOutput(obj,base,before28,currentTime,updateStatus, ...
                deltaPV,previous,fromTime,cha);
        end
    end
end

function obj=applyLastInterval(obj,deltaPV,previous)
dt=obj.cfg.dt;
if ~previous.valid
    % Unknown preceding actuation: do not misclassify it as a clear interval.
    % Hold extrapolation only; mark explicitly uncertified/unaccounted.
    obj.quietCount=0;
    obj.d=obj.d+dt*obj.w;
    if ~strcmp(obj.mode,'NOMINAL'),obj.mode='CARRY';end
    return
end
if previous.active
    obj.quietCount=0;
    obj.d=obj.d+dt*obj.w+deltaPV(1:3);
    obj.w=obj.w+deltaPV(4:6);
    obj.mode='CARRY';
else
    obj.quietCount=obj.quietCount+1;
    if obj.quietCount>=obj.cfg.returnQuietCycles && ...
            ~strcmp(obj.mode,'NOMINAL')
        omega=obj.cfg.returnOmega;
        t=omega*dt;
        Phi=exp(-t)*[1+t,dt;-omega^2*dt,1-t];
        oldD=obj.d;oldW=obj.w;
        obj.d=Phi(1,1)*oldD+Phi(1,2)*oldW;
        obj.w=Phi(2,1)*oldD+Phi(2,2)*oldW;
        obj.mode='RETURN';
        if norm(obj.d,Inf)<=obj.cfg.settlePositionM && ...
                norm(obj.w,Inf)<=obj.cfg.settleVelocityMps
            obj.d=zeros(3,1);obj.w=zeros(3,1);obj.mode='NOMINAL';
        end
    else
        obj.d=obj.d+dt*obj.w;
    end
end
end

function out=makeOutput(obj,base,beforeXd,t,status,deltaPV,previous,fromTime,cha)
% Preserve the original state completely for pass-through. Dispatcher and
% LOGGER already copy the state. Only the active flight correction copies
% and modifies state here.
out=base;
jet=zeros(size(beforeXd));
if strcmp(cha,'f') && obj.cfg.enabled && ~strcmp(obj.mode,'NOMINAL')
    jet=CBF_TRAJ_C6_JET(obj.d,obj.w,obj.mode,obj.cfg.returnOmega);
    afterXd=beforeXd+jet;
    st=state_copy(base.state);
    st.xd=afterXd;
    if isprop(st,'p'),st.p=afterXd(1:3);end
    if isprop(st,'v'),st.v=afterXd(5:7);end
    if isprop(st,'q'),st.q=[0;0;afterXd(4)];end
    out.state=st;
else
    afterXd=beforeXd;
end
% The 28-entry fields refer only to actual 28-entry references. Do not
% fabricate higher derivatives for takeoff/landing. Existing plotting code
% skips samples whose before28/after28 are not 28 entries.
before28=[];after28=[];delta28=[];
if numel(beforeXd)==28
    before28=beforeXd;after28=afterXd;delta28=jet;
end
tele=struct('time',t,'sourceIntervalStart',fromTime, ...
    'sourceIntervalEnd',t,'sourceStatus',status,'cbfId',obj.cfg.cbfId, ...
    'mode',obj.mode,'before28',before28,'after28',after28, ...
    'delta28',delta28,'signedCarryPosition',obj.d,'signedCarryVelocity',obj.w, ...
    'lastIntervalDeltaPV',deltaPV,'quietCount',obj.quietCount, ...
    'physicalModel',obj.cfg.method,'physicalPrevious',previous, ...
    'interSampleContinuityRequired',false,'safetyCertified',false, ...
    'phase',cha,'enabled',obj.cfg.enabled, ...
    'beforeXd',beforeXd,'afterXd',afterXd);
out.cbfTraj=tele;
end

function p=emptyPhysical(status)
p=struct('valid',false,'active',false,'status',status, ...
    'deltaControl',nan(4,1),'baseline',nan(4,1),'applied',nan(4,1), ...
    'T0',NaN,'Tdot0',NaN,'TddotApplied',NaN,'TddotBaseline',NaN, ...
    'TendApplied',NaN,'TendBaseline',NaN, ...
    'TdotEndApplied',NaN,'TdotEndBaseline',NaN,'elapsedMs',NaN);
end
