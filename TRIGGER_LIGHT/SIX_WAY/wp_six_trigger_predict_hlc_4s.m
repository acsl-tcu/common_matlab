function out=wp_six_trigger_predict_hlc_4s(agent,model,options)
%WP_SIX_TRIGGER_PREDICT_HLC_4S Diagnostic ORIGINAL-HLC closed-loop rollout.
%
% USE: out=wp_six_trigger_predict_hlc_4s(agent,model);
%      out=wp_six_trigger_predict_hlc_4s(agent,model, ...
%             struct('sampleTimes',0:0.1:4));
%
% Uses EXACT original with_load_model_euler_for_HL (24-state) and the
% ORIGINAL HLC_SUSPENDED_LOAD class, including its 4-input saturation.
% Updates a PRIVATE COPY of the 6-stage waypoint governor every step, from
% the predicted load state. No constant-future-input assumption.
% Source agent/HLC/EKF/reference/parameters are NEVER mutated.
%
% Future measurements are unknown: predicted plant state is supplied as a
% PERFECT future estimator for this nominal rollout. This is NOT a complete
% prediction of the EKF nor a certified actual-trajectory error tube.
% The numerical ODE solver gives NO rigorous enclosure. rho=NaN by design;
% wp_six_trigger_evaluate MUST return UNKNOWN for this forecast until a
% mathematically justified error enclosure is separately supplied.
%
% Recommended for OFFLINE comparative Flight and model-equation audits.
% 160 controller evaluations + 160 ODE integrations are NOT expected to
% meet the 1-2 ms online trigger target without substantial further work.
if nargin<3,options=struct();end
if ~isfield(options,'dt'),options.dt=0.025;end
if ~isfield(options,'horizon'),options.horizon=4;end
if ~isfield(options,'sampleTimes'),options.sampleTimes=0:0.1:4;end
if ~isfield(options,'solver'),options.solver=@ode15s;end
if ~isfield(options,'relTol'),options.relTol=1e-7;end
if ~isfield(options,'absTol'),options.absTol=1e-9;end
if ~isfield(options,'profile'),options.profile=false;end
if ~(isscalar(options.dt)&&isfinite(options.dt)&&options.dt>0 && ...
        abs(options.horizon/options.dt-round(options.horizon/options.dt))<1e-10)
    error('LightTrigger:Time','Horizon must be an integer number of original controller steps');
end
h=options.dt;N=round(options.horizon/h);
requested=reshape(options.sampleTimes,1,[]);
if isempty(requested)||abs(requested(1))>1e-10 || ...
        abs(requested(end)-options.horizon)>1e-10 || any(diff(requested)<=0)
    error('LightTrigger:Time','Require sorted sample grid spanning 0 to horizon');
end
fullTimes=(0:N)*h;
requestedIndex=round(requested/h)+1;
if any(abs((requestedIndex-1)*h-requested)>1e-9)
    error('LightTrigger:Sampling','This diagnostic requires sample times on the controller hold grid');
end
binding=wp_six_trigger_original_model_binding(agent,model);
if ~isprop(agent.controller,'hlc_suspended') || ...
        ~isa(agent.controller.hlc_suspended,'HLC_SUSPENDED_LOAD')
    error('LightTrigger:HLC','Missing original HLC_SUSPENDED_LOAD controller');
end
if ~isprop(agent.reference,'waypoint') || ...
        ~isa(agent.reference.waypoint,'WAYPOINT_REFERENCE')
    error('LightTrigger:Reference','Missing original WAYPOINT_REFERENCE in flight');
end
if ~(exist('wp_governor_step','file')==2) || ~(exist('STATE_CLASS','class')==8)
    error('LightTrigger:Sources','Original STATE_CLASS and wp_governor_step are required');
end
if ~isprop(agent.estimator.result.state,'mL')
    error('LightTrigger:Estimator','Current EKF estimated mass state required');
end
currentState=agent.estimator.result.state;
keys={'p','q','v','w','pL','vL','pT','wL'};
x=zeros(24,1);
for j=1:8
    if ~isprop(currentState,keys{j}),error('LightTrigger:Estimator','Missing %s',keys{j});end
    z=currentState.(keys{j});
    if numel(z)~=3 || any(~isfinite(z(:)))
        error('LightTrigger:Estimator','Invalid %s (expected 3-vector)',keys{j});
    end
    x((j-1)*3+(1:3))=z(:);
end

% This proxy isolates all future simulated state/reference/controller calls
% from the actual Flight agent, including estimator, controller internals
% and parameter read access. Every callback uses the original HLC source.
stateCfg=struct('state_list',string(keys),'num_list',3*ones(1,8));
stateProxy=STATE_CLASS(stateCfg);
stateProxy.set_state(x);
refProxy=STATE_CLASS(struct('state_list',"xd",'num_list',28,'qlist',""));
refNow=agent.reference.result.state.xd;
if numel(refNow)~=28 || any(~isfinite(refNow(:)))
    error('LightTrigger:Reference','Current 28-value reference unavailable');
end
refProxy.set_state('xd',refNow(:));
pnames={'mass','Lx','Ly','lx','ly','jx','jy','jz','gravity', ...
    'km1','km2','km3','km4','k1','k2','k3','k4','rotor_r', ...
    'Length','loadmass','cableL'};
pvals=num2cell(binding.physicalParameterRow);
paramSnapshot=WP_TRIGGER_PARAMETER_VIEW(cell2struct(pvals,pnames,2));
proxy=struct();
proxy.estimator=struct('result',struct('state',stateProxy));
proxy.reference=struct('result',struct('state',refProxy));
proxy.parameter=paramSnapshot;
oldHlc=agent.controller.hlc_suspended;
controller=HLC_SUSPENDED_LOAD(proxy,oldHlc.param);
gov=agent.reference.waypoint;
if isempty(gov.lastFlightTime)
    error('LightTrigger:Governor','Waypoint reference must have entered flight phase');
end
tau=gov.virtualTime;
z=gov.rateState(:);
cmd=gov.heldRateCommand;
if numel(z)~=6 || any(~isfinite(z)) || ~isfinite(cmd)
    error('LightTrigger:Governor','Invalid current virtual-time filter');
end

xHist=zeros(24,N+1); xHist(:,1)=x;
inputHist=zeros(4,N);rawInputHist=nan(4,N);tensionAtKnots=nan(1,N);
tauHist=zeros(1,N+1);tauHist(1)=tau;
cmdHist=zeros(1,N+1);cmdHist(1)=cmd;
zHist=zeros(6,N+1);zHist(:,1)=z;
start=tic;
profileMs=struct('hlc',0,'rhsIntegration',0,'governor',0);
solverOpts=odeset('RelTol',options.relTol,'AbsTol',options.absTol);
for k=1:N
    % HLC recomputes all four inputs from PRIVATE predicted state + current
    % private reference. Original controller saturates thrust to [0,20],
    % torques to [-1,1]. No held-future-input shortcut.
    if options.profile,watch=tic;end
    stateProxy.set_state(x);
    try
        control=controller.do();
    catch ME
        error('LightTrigger:OriginalHLC', ...
            'Original HLC failed at forecast step %d: %s',k,ME.message);
    end
    u=control.input(:);
    if numel(u)~=4 || any(~isfinite(u))
        error('LightTrigger:InvalidInput','Original HLC gave invalid input at %d',k);
    end
    if options.profile,profileMs.hlc=profileMs.hlc+1000*toc(watch);end
    inputHist(:,k)=u;
    if isfield(control,'tmp'), rawInputHist(:,k)=control.tmp(:); end
    qk=x(19:21);wk=x(22:24);
    rot=RodriguesQuaternion(Eul2Quat(x(4:6)));
    bk=rot(:,3);
    mD=model.droneMass;mL=model.loadMass;LL=model.cableLength;
    tensionAtKnots(k)=(mD*mL/(mD+mL))*LL*norm(cross(wk,qk))^2 ...
        -(mL/(mD+mL))*u(1)*dot(qk,bk);
    if options.profile,watch=tic;end
    [~,states]=options.solver(@(tloc,xx) binding.rhs(xx,u,binding.physicalParameterRow), ...
        [0,h],x,solverOpts); %#ok<ASGLU>
    if options.profile,profileMs.rhsIntegration=profileMs.rhsIntegration+1000*toc(watch);end
    x=states(end,:)';
    if numel(x)~=24 || any(~isfinite(x))
        error('LightTrigger:OriginalRHS','Nonfinite original-model result at %d',k);
    end
    xHist(:,k+1)=x;

    % Advance governor using PREVIOUS held command; compute NEXT command
    % from NEW predicted load position. Equivalent ordering to source
    % WAYPOINT_REFERENCE.do during 'f' (not pre-frozen future command).
    if options.profile,watch=tic;end
    if gov.adaptiveProgress
        [z,tau,tauDer]=wp_governor_step(z,tau,cmd,h,gov.filterTime);
        raw=gov.generator.evaluateWarped(tau,tauDer);
        targetLoad=raw(1:3);targetLoad(3)=targetLoad(3)-model.cableLength;
        e=norm(targetLoad-x(13:15));
        v=max(0,min(1,(e-gov.slowdownStart)/ ...
            (gov.slowdownFull-gov.slowdownStart)));
        v=v*v*(3-2*v);
        cmd=1-(1-gov.minScale)*v;
    else
        tau=tau+h;
        raw=gov.generator.evaluate(tau);
        cmd=1;
    end
    raw(3)=raw(3)-model.cableLength; % original SUSPENDED_LOAD_REF_ADJUST flight rule
    refProxy.set_state('xd',raw(:));
    tauHist(k+1)=tau;
    cmdHist(k+1)=cmd;
    zHist(:,k+1)=z;
    if options.profile,profileMs.governor=profileMs.governor+1000*toc(watch);end
end
runtime=1000*toc(start);
positionsD=xHist(1:3,requestedIndex);
positionsL=xHist(13:15,requestedIndex);
% A guaranteed acceleration bound of the *actual* physical plant is not
% available. Even the nominal ODE integration error is not rigorous.
% Leave these NaN so the existing safety gate CANNOT declare CLEAR.
F=struct('timeOffsets',requested,'pDrone',positionsD,'pLoad',positionsL, ...
    'rhoDrone',nan(1,numel(requested)), ...
    'rhoLoad',nan(1,numel(requested)), ...
    'accDrone',nan(1,numel(requested)-1), ...
    'accLoad',nan(1,numel(requested)-1), ...
    'modelLoadMass',model.loadMass, ...
    'notCertified',true);
% Violations are DIAGNOSTICS, not criteria permitting unsafe clearance.
q=xHist(19:21,:);r=xHist(13:15,:)-xHist(1:3,:);
qNormError=max(abs(sqrt(sum(q.^2,1))-1));
cableResidual=max(sqrt(sum((r-model.cableLength*q).^2,1)));
out=struct('status','NOMINAL_HLC_ROLLOUT_NOT_CERTIFIED', ...
    'forecast',F,'fullTimes',fullTimes,'fullState',xHist, ...
    'heldInputs',inputHist,'rawUnclippedInputs',rawInputHist, ...
    'sampledModelTension',tensionAtKnots, ...
    'minSampledModelTension',min(tensionAtKnots), ...
    'modelSlackMayOccur',any(tensionAtKnots<=0), ...
    'inputSaturationSeen',any(abs(rawInputHist-inputHist)>1e-9,'all'), ...
    'virtualTime',tauHist, ...
    'governorCommand',cmdHist,'governorRateState',zHist, ...
    'governorSettings',struct('adaptiveProgress',gov.adaptiveProgress, ...
      'slowdownStart',gov.slowdownStart,'slowdownFull',gov.slowdownFull, ...
      'minScale',gov.minScale,'filterTime',gov.filterTime), ...
    'elapsedMs',runtime, ...
    'controllerSteps',N,'solverName',func2str(options.solver), ...
    'profileEnabled',logical(options.profile),'profileStageMs',profileMs, ...
    'originalRhsSource',binding.modelSource, ...
    'originalHlcSource',which('HLC_SUSPENDED_LOAD'), ...
    'initialStateSource','current post-update estimator state', ...
    'futureEstimatorAssumption','predicted state used as future estimator', ...
    'estimatedLoadMassUsed',model.loadMass, ...
    'plantTruthRead',false,'agentMutated',false, ...
    'numericalModelConsistencyMaxCableResidual',cableResidual, ...
    'numericalCableDirectionNormError',qNormError, ...
    'actualFutureErrorCertified',false,'safeToCommit',false);
end
