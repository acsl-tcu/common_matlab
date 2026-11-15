function R = TEST_TRIGGER_V4_HLC_STRESS()
%TEST_TRIGGER_V4_HLC_STRESS Offline ORIGINAL HLC/RHS 4s stress reference.
%
% Four scenarios: nominal tracking, off-route lateral offset, initial
% cable tilt & rate, and large offset likely to saturate. This is a
% COMPARISON BASELINE, NEVER an online 1-2ms predictor.
% No plant true load mass enters a predicted trajectory. The estimator's
% mL is passed to the original nominal RHS and the original HLC only.
% Future observations are UNKNOWN; this forecast supplies predicted state
% as the future estimator. An actual Flight log comparison is STILL needed.

cases={'nominal','lateral_rejoin','initial_swing','large_offset'};
R=struct('status','NOT_RUN','scenarios',{cases},'results',[], ...
   'allSucceeded',false,'actualFlightCompared',false, ...
   'physicalErrorCertified',false,'onlinePredictionQualified',false);
res=repmat(struct('caseName','','status','','elapsedMs',NaN, ...
    'maxDroneDisplacement',NaN,'maxLoadDisplacement',NaN, ...
    'maxTautResidual',NaN,'minSampledTension',NaN, ...
    'saturationSeen',false,'governorUpdated',false, ...
    'modelEstimatedMass',NaN,'errorText','', ...
    'loadReferenceErrorStart',NaN,'loadReferenceErrorEnd',NaN, ...
    'maxLoadReferenceError',NaN,'saturationMagnitude',NaN, ...
    'maxCableAngularRate',NaN,'nominalTrace',struct()),1,numel(cases));
required={'DRONE','DRONE_PARAM_SUSPENDED_LOAD','HLC_SUSPENDED_LOAD', ...
    'Controller_HL_Suspended_Load','STATE_CLASS','WAYPOINT_REFERENCE', ...
    'wp_governor_step','gen_wp_straight','with_load_model_euler_for_HL'};
missing={};
for i=1:numel(required)
    if isempty(which(required{i})),missing{end+1}=required{i};end %#ok<AGROW>
end
if ~isempty(missing)
    R.status='SKIPPED_ORIGINAL_SOURCES_MISSING';
    R.missingSources=missing;
    return
end
for i=1:numel(cases)
    res(i).caseName=cases{i};
    try
        [agent,model,initial]=makeScenario(cases{i});
        p0=initial.p(:);pl0=initial.pL(:);
        result=wp_six_trigger_predict_hlc_4s(agent,model,struct( ...
            'dt',.025,'horizon',4,'sampleTimes',0:.1:4, ...
            'profile',true));
        assert(result.controllerSteps==160);
        assert(~result.actualFutureErrorCertified && ~result.safeToCommit);
        assert(abs(result.estimatedLoadMassUsed-initial.mL)<1e-12);
        assert(~result.plantTruthRead);
        res(i).status='NOMINAL_COMPLETED';
        res(i).elapsedMs=result.elapsedMs;
        res(i).maxDroneDisplacement=max(vecnorm(result.fullState(1:3,:)-p0,2,1));
        res(i).maxLoadDisplacement=max(vecnorm(result.fullState(13:15,:)-pl0,2,1));
        res(i).maxTautResidual=result.numericalModelConsistencyMaxCableResidual;
        res(i).minSampledTension=result.minSampledModelTension;
        res(i).saturationSeen=result.inputSaturationSeen;
        res(i).governorUpdated=max(abs(diff(result.virtualTime)))>1e-8;
        res(i).modelEstimatedMass=result.estimatedLoadMassUsed;
        res(i).saturationMagnitude=max(abs( ...
            result.rawUnclippedInputs(:)-result.heldInputs(:)),[],'omitnan');
        res(i).maxCableAngularRate=max(vecnorm(result.fullState(22:24,:),2,1));
        % Measure the ACTUAL original-governor reference that this rollout
        % used, instead of conflating a fixed-time nominal reference with
        % the load's deviation during rejoining.
        actualRef=zeros(3,numel(result.virtualTime));
        for s=1:numel(result.virtualTime)
            jet=agent.reference.waypoint.generator.evaluate(result.virtualTime(s));
            actualRef(:,s)=jet(1:3);actualRef(3,s)=actualRef(3,s)-model.cableLength;
        end
        errors=vecnorm(result.fullState(13:15,:)-actualRef,2,1);
        res(i).loadReferenceErrorStart=errors(1);
        res(i).loadReferenceErrorEnd=errors(end);
        res(i).maxLoadReferenceError=max(errors);
        res(i).nominalTrace=struct('time',result.fullTimes, ...
             'pDrone',result.fullState(1:3,:), ...
             'pLoad',result.fullState(13:15,:), ...
             'pT',result.fullState(19:21,:), ...
             'wL',result.fullState(22:24,:), ...
             'refLoad',actualRef, ...
             'input',result.heldInputs, ...
             'governorTau',result.virtualTime, ...
             'source','original_HLC_24state_nominal_not_actual_Flight');
        % Do NOT apply an all-thrust-reachable 4-second ball here. Its
        % several-metre-scale radius would flag safe near obstacles and
        % does not model the actual HLC. No fake envelope is supplied.
    catch ME
        res(i).status='FAIL';res(i).errorText=ME.message;
    end
    fprintf('V4 %-18s %-20s %.3f ms, tension %.4g, sat=%d\n', ...
      res(i).caseName,res(i).status,res(i).elapsedMs, ...
      res(i).minSampledTension,res(i).saturationSeen);
end
R.results=res;
R.saturationActuallyObserved=any([res.saturationSeen]);
R.saturationCoverageEstablished=R.saturationActuallyObserved;
R.rejoinErrorDecreased=strcmp(res(2).status,'NOMINAL_COMPLETED') && ...
    (res(2).loadReferenceErrorEnd < res(2).loadReferenceErrorStart);
R.allSucceeded=all(strcmp({res.status},'NOMINAL_COMPLETED'));
if R.allSucceeded
    if R.saturationActuallyObserved
        R.status='PASS_FOUR_ORIGINAL_NOMINAL_STRESS_ROLLOUTS__SATURATION_OBSERVED';
    else
        R.status='PASS_FOUR_NOMINAL_STRESS__SATURATION_NOT_EXERCISED';
    end
else
    R.status='STRESS_CASE_FAILURES_REQUIRING_INSPECTION';
end
end

function [agent,model,initial]=makeScenario(whichCase)
dt=.025;agent=DRONE();
agent.parameter=DRONE_PARAM_SUSPENDED_LOAD('DIATONE');
agent.parameter.set("cableL",struct('cableL',2.0));
agent.parameter.set("loadmass",struct('loadmass',0.01)); % deliberately stale
L=agent.parameter.get("cableL");
initial=struct('p',[0;0;3],'q',[0;0;0],'v',[0;0;0], ...
    'w',[0;0;0],'pL',[0;0;3-L],'vL',[0;0;0], ...
    'pT',[0;0;-1],'wL',[0;0;0],'mL',.14);
switch whichCase
    case 'nominal'
        % baseline
    case 'lateral_rejoin'
        initial.p=initial.p+[.75;.35;0];
    case 'initial_swing'
        a=.15;
        initial.pT=[sin(a);0;-cos(a)];
        initial.wL=[0;.6;0];
    case 'large_offset'
        initial.p=initial.p+[1.4;.6;0];
        a=.20;
        initial.pT=[sin(a);0;-cos(a)];
        initial.wL=[0;.7;0];
end
initial.pL=initial.p+L*initial.pT;
initial.vL=initial.v+L*cross(initial.wL,initial.pT);
names=["p","q","v","w","pL","vL","pT","wL","mL"];
sizes=[3,3,3,3,3,3,3,3,1];
est=STATE_CLASS(struct('state_list',names,'num_list',sizes));
est.set_state(initial);
agent.estimator.result=struct('state',est);
agent.plant=struct('name','TEST_ONLY_FAKE_PLANT_NOT_USED_FOR_PREDICTION');
agent.reference.set_function_class('waypoint', ...
    WAYPOINT_REFERENCE(agent,{'gen_wp_straight',{'speed',1.5},6}));
wp=agent.reference.waypoint;
wp.virtualTime=0;
wp.rateState=ones(6,1);wp.heldRateCommand=1;wp.lastFlightTime=5;wp.t=5;
raw=wp.generator.evaluate(0);raw(3)=raw(3)-L;
wp.result.state.set_state('xd',raw);
agent.reference.result=wp.result;
agent.controller.set_function_class('hlc_suspended', ...
    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
model=wp_six_trigger_model_snapshot(agent);
assert(abs(model.loadMass-initial.mL)<1e-12);
end
