function R=TEST_TRIGGER_V3_ORIGINAL_HLC_4S()
%TEST_TRIGGER_V3_ORIGINAL_HLC_4S Test the ORIGINAL 24-state RHS/HLC.
% No production agent is touched. Creates a SEPARATE TEST-ONLY agent.
% Requires original common_matlab model/controller/waypoint sources on path.
% Uses EKF-like CURRENT estimated mL=0.14 but model parameter initially
% 0.01, to ensure rollout does NOT read plant truth or stale loadmass.
R=struct('status','NOT_RUN','reason','', 'originalRhsFound',false, ...
    'originalHlcFound',false,'modelBindingVerified',false, ...
    'full4sRolloutEvaluated',false,'modelEstimatedMassUsed',NaN, ...
    'referenceProgressChanged',false,'inputChangedOverHorizon',false, ...
    'rolloutMs',NaN,'modelKinematicResidual',NaN, ...
    'nominalNotCertified',true, ...
    'physicalTruthUsedByPrediction',false);
required={'DRONE','DRONE_PARAM_SUSPENDED_LOAD','HLC_SUSPENDED_LOAD', ...
    'Controller_HL_Suspended_Load','STATE_CLASS', ...
    'get_model_name','with_load_model_euler_for_HL', ...
    'WAYPOINT_REFERENCE','wp_governor_step','gen_wp_straight', ...
    'Eul2Quat','RodriguesQuaternion'};
missing={};
for j=1:numel(required)
    if isempty(which(required{j})),missing{end+1}=required{j};end %#ok<AGROW>
end
if ~isempty(missing)
    R.status='SKIPPED_ORIGINAL_SOURCE_NOT_ON_MATLAB_PATH';
    R.reason=strjoin(missing,', ');
    fprintf('SKIP ORIGINAL MODEL/HLC: add real source paths: %s\n',R.reason);
    return
end
R.originalRhsFound=true;R.originalHlcFound=true;

% Local test agent only. Matches the 25ms Flight configuration, not a
% substitute for an actual Flight log with future EKF measurement updates.
dt=.025;
agent=DRONE();
agent.parameter=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
agent.parameter.set("cableL",struct("cableL",2.0));
agent.parameter.set("loadmass",struct("loadmass",0.01)); % deliberately stale parameter
L=agent.parameter.get("cableL");
initial=struct('p',[0;0;3],'q',[0;0;0], 'v',[0;0;0], ...
    'w',[0;0;0],'pL',[0;0;3-L],'vL',[0;0;0], ...
    'pT',[0;0;-1],'wL',[0;0;0],'mL',0.14);
names=["p","q","v","w","pL","vL","pT","wL","mL"];
sizes=[3,3,3,3,3,3,3,3,1];
estState=STATE_CLASS(struct('state_list',names,'num_list',sizes));
estState.set_state(initial);
agent.estimator.result=struct('state',estState);
agent.plant=struct('name','independent_model_audit_placeholder');

agent.reference.set_function_class('waypoint', ...
    WAYPOINT_REFERENCE(agent,{'gen_wp_straight',{'speed',1.5},6}));
wp=agent.reference.waypoint;
wp.virtualTime=0;wp.rateState=ones(6,1);
wp.heldRateCommand=1;wp.lastFlightTime=5;wp.t=5;
raw=wp.generator.evaluate(0);
raw(3)=raw(3)-L;
wp.result.state.set_state('xd',raw);
agent.reference.result=wp.result;
agent.controller.set_function_class('hlc_suspended', ...
    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));

massBefore=agent.parameter.get("loadmass");
tauBefore=wp.virtualTime;
refBefore=agent.reference.result.state.xd;
model=wp_six_trigger_model_snapshot(agent);
assert(abs(model.loadMass-initial.mL)<1e-12);
binding=wp_six_trigger_original_model_binding(agent,model);
assert(abs(binding.physicalParameterRow(20)-initial.mL)<1e-12);
assert(abs(agent.parameter.get("loadmass")-massBefore)<1e-12);
R.modelBindingVerified=true;R.modelEstimatedMassUsed=model.loadMass;

x0=estState.get(["p","q","v","w","pL","vL","pT","wL"]);
assert(isequal(size(x0),[24,1]), ...
    'Original 24-state RHS must receive exactly 24 states; estimated mL is a parameter');
u0=[(model.droneMass+model.loadMass)*model.gravity;0;0;0];
dx=binding.rhs(x0,u0,binding.physicalParameterRow);
assert(isequal(size(dx),[24,1]) && all(isfinite(dx)));
assert(norm(dx(1:3)-x0(7:9))<1e-12);
assert(norm(dx(13:15)-x0(16:18))<1e-12);
assert(norm(dx(19:21)-cross(x0(22:24),x0(19:21)))<1e-10);
% The original source at exact vertical hover must return zero translational
% acceleration. This is a source-grounded equilibrium, NOT empirical tuning.
R.modelKinematicResidual=max([norm(dx(1:3)-x0(7:9)), ...
    norm(dx(13:15)-x0(16:18)), ...
    norm(dx(19:21)-cross(x0(22:24),x0(19:21)))]);
assert(norm(dx(7:9))<1e-7 && norm(dx(16:18))<1e-7, ...
    'Original source hover equilibrium differs from presumed ideal taut model');

try
    out=wp_six_trigger_predict_hlc_4s(agent,model, ...
        struct('dt',dt,'horizon',4,'sampleTimes',0:0.1:4,'profile',true));
catch ME
    R.status='FAIL_HLC_ROLLOUT';R.reason=ME.message;
    rethrow(ME);
end
assert(strcmp(out.status,'NOMINAL_HLC_ROLLOUT_NOT_CERTIFIED'));
assert(out.controllerSteps==160);
assert(all(isfinite(out.fullState(:))) && all(isfinite(out.heldInputs(:))));
assert(all(size(out.fullState)==[24,161]));
assert(abs(agent.parameter.get("loadmass")-massBefore)<1e-12, ...
    'Test original agent parameter was modified by predictor');
assert(isequal(agent.reference.result.state.xd,refBefore));
assert(wp.virtualTime==tauBefore);
assert(all(isnan(out.forecast.rhoDrone))); % CANNOT claim safe
assert(~out.actualFutureErrorCertified && ~out.safeToCommit);
R.full4sRolloutEvaluated=true;
R.referenceProgressChanged=abs(out.virtualTime(end)-out.virtualTime(1))>1e-5;
R.inputChangedOverHorizon=any(abs(diff(out.heldInputs,1,2))>1e-8,'all');
R.rolloutMs=out.elapsedMs;
R.rolloutStageMs=out.profileStageMs;
R.minSampledModelTension=out.minSampledModelTension;
R.inputSaturationSeen=out.inputSaturationSeen;
R.maxCableResidual=out.numericalModelConsistencyMaxCableResidual;
R.initialEstimatedMass=model.loadMass;
R.originalRhsSource=out.originalRhsSource;
R.originalHlcSource=out.originalHlcSource;
R.status='PASS_ORIGINAL_24STATE_AND_HLC_NOMINAL_ROLLOUT__UNCERTIFIED';
fprintf('PASS original RHS + original HLC 4s; %.4f ms; cable residual %.3g\n', ...
    R.rolloutMs,R.maxCableResidual);
end
