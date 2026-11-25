classdef CBF_TRAJ_REFERENCE < handle
% Sim-side WAYPOINT decoration. Runs AFTER WAYPOINT and SLOAD reference.
% Reads previous interval's already-existing controller.result.{x,tmp,input,cbf};
% updates signed [position;velocity] correction and passes a new 28-jet
% to the UNMODIFIED HLC. Works only in flight, never modifies controllers.
% Every flight reference result has cbfTraj.before28 and cbfTraj.after28.
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
            before28=double(base.state.xd(:));
            if numel(before28)~=28 || any(~isfinite(before28))
                error('CBFTraj:Reference28Invalid','Expected finite base 28x1 after SLOAD');
            end
            currentTime=double(time.t);
            if ~strcmp(cha,'f')
                obj.d=zeros(3,1);obj.w=zeros(3,1);
                obj.mode='NOMINAL';obj.quietCount=0;obj.lastFlightTime=[];
                out=makeOutput(obj,base,before28,currentTime,'NON_FLIGHT', ...
                    zeros(6,1),[],NaN);
                return
            end
            deltaPV=zeros(6,1);fromTime=NaN;
            previous=struct('valid',false,'active',false,'status','FIRST_FLIGHT_CYCLE', ...
                'T0',NaN,'Tdot0',NaN,'TddotApplied',NaN, ...
                'TddotBaseline',NaN,'TendApplied',NaN,'TendBaseline',NaN, ...
                'TdotEndApplied',NaN,'TdotEndBaseline',NaN, ...
                'elapsedMs',NaN);
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
                deltaPV,previous,fromTime);
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

function out=makeOutput(obj,base,before28,t,status,deltaPV,previous,fromTime)
if strcmp(obj.mode,'NOMINAL')
    jet=zeros(28,1);
else
    jet=CBF_TRAJ_C6_JET(obj.d,obj.w,obj.mode,obj.cfg.returnOmega);
end
after28=before28+jet;
% Preserve previous reference result fields, and replace state by a COPY.
st=state_copy(base.state);
st.xd=after28;
if isprop(st,'p'),st.p=after28(1:3);end
if isprop(st,'v'),st.v=after28(5:7);end
if isprop(st,'q'),st.q=[0;0;after28(4)];end
tele=struct('time',t,'sourceIntervalStart',fromTime, ...
    'sourceIntervalEnd',t,'sourceStatus',status,'cbfId',obj.cfg.cbfId, ...
    'mode',obj.mode,'before28',before28,'after28',after28, ...
    'delta28',jet,'signedCarryPosition',obj.d,'signedCarryVelocity',obj.w, ...
    'lastIntervalDeltaPV',deltaPV,'quietCount',obj.quietCount, ...
    'physicalModel',obj.cfg.method,'physicalPrevious',previous, ...
    'interSampleContinuityRequired',false, ...
    'safetyCertified',false);
out=base;
out.state=st;
out.cbfTraj=tele;
end
