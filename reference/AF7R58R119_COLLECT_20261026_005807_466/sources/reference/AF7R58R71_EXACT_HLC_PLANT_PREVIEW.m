function R = AF7R58R71_EXACT_HLC_PLANT_PREVIEW(agent,rawEval,t0,H,dt,mL,x0)
% Independent, noncommittable model-prediction experiment.
% Model and HLC are the SAME classes/functions used by the working system;
% their OBJECTS are not modified. Existing HLC gains are read at invocation.
% Flight SUSPENDED_LOAD_REF_ADJUST: raw xd(3) -> payload xd(3)-cableL.
% This is a numerical rollout, NOT a continuous-time reachable set proof.
assert(isa(rawEval,'function_handle'),'[R71] missing active reference evaluator');
assert(isfinite(t0)&&isscalar(t0) && isfinite(H)&&H>0 && ...
    isfinite(dt)&&dt>0 && isfinite(mL)&&mL>0, ...
    '[R71] invalid time/horizon/dt/latest EKF payload mass.');
n=H/dt;N=round(n)+1;
assert(abs(n-round(n)) <= 1e-9*max(1,abs(n)), ...
    '[R71] caller must use the configured exact horizon/control grid.');
% R74: accept a frozen, read-only *struct* authority for postflight
% replay. Never require or mutate the live controller/plant instances.
plant=agent.plant;
assert(((isstruct(plant) && isfield(plant,'method') && isfield(plant,'solver')) || ...
    (isobject(plant) && isprop(plant,'method') && isprop(plant,'solver'))) && ...
    isa(plant.method,'function_handle') && isa(plant.solver,'function_handle'), ...
    '[R74] captured plant solver/model is unavailable.');
controller=agent.controller;
assert(((isstruct(controller) && isfield(controller,'hlc_suspended')) || ...
    (isobject(controller) && isprop(controller,'hlc_suspended'))) && ...
    ((isa(controller.hlc_suspended,'HLC_SUSPENDED_LOAD')) || ...
      (isstruct(controller.hlc_suspended) && ...
       isfield(controller.hlc_suspended,'param'))), ...
    '[R74] event-bound HLC gains are unavailable.');
P=double(agent.plant.param(:).');
order=string(agent.parameter.parameter_order);
ix=find(strcmpi(order,'loadmass'));
assert(numel(ix)==1 && numel(P)>=ix && ...
    numel(P)==double(agent.plant.dim(3)), ...
    '[R71] live parameter order/plant dimension changed; no fallback.');
frozenPlantMass=double(P(ix));
P(ix)=double(mL); % EKF estimate: model of real plant, not true plant mass.
% PARAMETER_CLASS.get iterates string ARRAYS of parameter names.
% A MATLAB char vector ('loadmass') may be treated as eight single-letter
% names, causing MATLAB:noSuchMethodOrField on property 'l'.
% These names must be MATLAB string scalars, as in the real HLC.
liveControllerML=double(agent.parameter.get("loadmass"));
assert(isfinite(liveControllerML)&&abs(liveControllerML-mL) <= ...
    4096*eps(max([1,abs(liveControllerML),abs(mL)])), ...
    ['[R71] current HLC parameter loadmass differs from latest EKF mL; ' ...
     'physical preview cannot claim the exact live controller.']);
L=double(agent.parameter.get("cableL"));
assert(isscalar(L)&&isfinite(L)&&L>0, ...
    '[R71] invalid runtime cable length.');
fields={'p','q','v','w','pL','vL','pT','wL'};
if nargin>=7 && ~isempty(x0)
    % Immutable EKF snapshot from the SAME real Flight trigger event.
    x=double(x0(:));
    assert(numel(x)==24 && all(isfinite(x)), ...
        '[R71R2] invalid frozen same-event EKF state.');
else
    state=agent.estimator.result.state;
    x=zeros(24,1);
    for k=1:8
        zi=state.(fields{k});
        zi=double(zi(:));
        assert(numel(zi)==3 && all(isfinite(zi)), ...
            '[R71] full EKF state not available in live plant coordinates.');
        x((3*k-2):(3*k))=zi;
    end
end
assert(double(agent.plant.dim(1))==numel(x), ...
    '[R71] live physical state dimension differs from source-bound EKF layout.');
% Shadow STATE_CLASS objects are independent of the live estimator/reference.
desc=struct('state_list',string(fields),'num_list',3*ones(1,8));
shadowState=STATE_CLASS(desc);
shadowState.set_state(x);
refDesc=struct('state_list',"xd",'num_list',28,'qlist',"");
shadowReferenceState=STATE_CLASS(refDesc);
shadow=struct();
shadow.parameter=agent.parameter; % read-only external authority
shadow.estimator=struct('result',struct('state',shadowState));
shadow.reference=struct('result',struct('state',shadowReferenceState));
% New HLC instance, original equations and original LIVE gains; no HLC edit.
shadowHLC=HLC_SUSPENDED_LOAD(shadow,agent.controller.hlc_suspended.param);
T=t0+(0:(N-1))*dt;
X=nan(24,N);U=nan(4,N-1);RawU=nan(4,N-1);
XD=nan(28,N);X(:,1)=x;
fun=agent.plant.method;
solver=agent.plant.solver;
for k=1:N
    raw=double(rawEval(T(k)));raw=raw(:);
    assert(numel(raw)>=28 && all(isfinite(raw(1:28))), ...
        '[R71] exact active reference must provide 0..6 world-time jets.');
    cmd=raw(1:28);
    cmd(3)=cmd(3)-L; % verified external Flight adapter mapping
    XD(:,k)=cmd;
    if k==N, break;end
    shadowState.set_state(x);
    shadowReferenceState.xd=cmd;
    out=shadowHLC.do();
    u=double(out.input(:));tmp=double(out.tmp(:));
    assert(numel(u)==4 && numel(tmp)==4 && all(isfinite([u;tmp])), ...
        '[R71] live HLC yielded invalid input.');
    U(:,k)=u;RawU(:,k)=tmp;
    [~,xx]=solver(@(~,s)fun(s,u,P),[T(k),T(k+1)],x);
    assert(size(xx,2)==24 && all(isfinite(xx(end,:))), ...
        '[R71] live physical plant rollout failed.');
    x=double(xx(end,:).');X(:,k+1)=x;
end
% Preserve the original physical-state convention, not a pL-offset trick.
pQ=X(1:3,:);pL=X(13:15,:);pT=X(19:21,:);
closure=sqrt(sum((pQ+L*pT-pL).^2,1));
R=struct();
R.status='EKF_INITIALIZED_EXISTING_HLC_AND_PLANT_NUMERIC_ONLY__NOT_CERTIFIED';
R.t=T;R.state=X;R.u=U;R.raw_input=RawU;
R.hlc_payload_xd=XD;R.pQ=pQ;R.pL=pL;R.pT=pT;
R.estimated_mass_kg=mL;R.cable_length_m=L;
R.event_plant_model_mass_kg=frozenPlantMass;
R.mass_authority='EVENT_EKF_ESTIMATE_FOR_PLANNING__PLANT_TRUE_MASS_NOT_ASSUMED';
R.input_limits_note='ACTUAL_SATURATED_HLC_INPUTS__NO_CONTINUOUS_TIME_PROOF';
R.max_cable_closure_residual_m=max(closure);
R.saturated_sample_count=sum(any(abs(U-RawU)>1e-9,1));
R.continuous_time_collision_proved=false;
R.future_collision_trigger_replaced=false;
R.execution_input_limits_proved=false;
R.tension_limits_proved=false;
R.safe_for_commit=false;
end
