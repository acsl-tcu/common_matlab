function R=RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT(withSaturationProbe)
%RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT Offline source-HLC / causal forecast audit.
% USAGE: R=RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT();
%        R=RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT(true);  % optional one-step probe
%
% No source Flight/HLC/EKF/CBF is modified. No future recorded reference is
% passed into the candidate. Generator samples are KNOWN ROUTE, cached.
% NO physical error certificate, NO full25ms benchmark, NO safe CLEAR.
% A non-PASS is NOT silently promoted to a success.
if nargin<1,withSaturationProbe=false;end
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'SIX_WAY'));
addpath(fullfile(root,'TESTS'));
R=struct('status','NOT_RUN','errorText','', ...
    'actualFlightCompared',false,'physicalErrorCertified',false, ...
    'futureReferenceOracleUsedInCandidate',false, ...
    'requiresPastEstimatorHistory',true, ...
    'futurePlantUsedInCandidate',false, ...
    'onlineReady',false,'oneToTwoMsTotalQualified',false, ...
    'allSixModesIntegrated',false,'fullFlight25msQualified',false, ...
    'sampleIntervalSeconds',0.025,'referenceSimulationSeconds',6, ...
    'candidateForecastSeconds',4, ...
    'fitHistorySeconds',1,'startAtSeconds',[1,2], ...
    'saturationFourSecondQualified',false, ...
    'sourceContract',[],'governor',[],'original6',[],'causalForecast',[], ...
    'oneStepSaturation',[]);
try
    required={'DRONE','DRONE_PARAM_SUSPENDED_LOAD', ...
        'Controller_HL_Suspended_Load','HLC_SUSPENDED_LOAD', ...
        'with_load_model_euler_for_HL','WAYPOINT_REFERENCE', ...
        'gen_wp_straight','wp_governor_step','STATE_CLASS', ...
        'WP_TRIGGER_PARAMETER_VIEW', ...
        'wp_six_trigger_predict_hlc_4s'};
    missing={};
    for i=1:numel(required)
        if isempty(which(required{i})),missing{end+1}=required{i};end %#ok<AGROW>
    end
    if ~isempty(missing)
        error('V6:MissingSources','Missing original source: %s',strjoin(missing,', '));
    end
    fprintf('[V6.1] Checking loaded ORIGINAL-HLC rollout trace contract...\n');
    R.sourceContract=TEST_TRIGGER_V6_SOURCE_CONTRACT();
    fprintf('[V6] Source governor exact-ZOH equivalence...\n');
    R.governor=TEST_TRIGGER_V5_GOVERNOR_EQUIVALENCE(500);
    fprintf('[V6] Original HLC+24state 6s rollout: 4 cases (offline only)...\n');
    R.original6=TEST_TRIGGER_V6_HLC_STRESS_6S(6);
    if ~R.original6.allSucceeded
        R.status='FAIL_ORIGINAL_6S_ROLLOUT';
        bad=R.original6.results(~strcmp({R.original6.results.status},'NOMINAL_COMPLETED'));
        details=cell(1,numel(bad));
        for kb=1:numel(bad)
            details{kb}=sprintf('%s: %s',bad(kb).caseName,bad(kb).errorText);
        end
        error('V6:Original6', ...
            'Original HLC 6s cases failed: %s',strjoin(details,' | '));
    end
    fprintf('[V6] Causal modal + governor 4s predictions, t0=2s...\n');
    R.causalForecast=TEST_TRIGGER_V6_AUTONOMOUS_GOV_4S(R.original6);
    assert(~R.causalForecast.futureReferenceOracleUsed && ...
        ~R.causalForecast.physicalPredictionErrorCertified && ...
        ~R.causalForecast.productionIntegrationComplete && ...
        ~R.causalForecast.oneToTwoMsTotalQualified);
    if withSaturationProbe
        fprintf('[V6] Optional one-step saturation probe (NOT 4s)...\n');
        R.oneStepSaturation=TEST_TRIGGER_V5_SATURATION_EXCITATION();
    else
        R.oneStepSaturation=struct('status','NOT_RUN_OPTIONAL');
    end
    R.status='PASS_OFFLINE_CAUSAL_AUDIT__NOT_DEPLOYABLE';
catch ME
    if ~strcmp(R.status,'FAIL_ORIGINAL_6S_ROLLOUT')
        R.status='FAIL_V6_AUDIT';
    end
    R.errorText=sprintf('%s: %s',ME.identifier,ME.message);
    fprintf(2,'[V6] %s\n',R.errorText);
end
% Unique-name results even if another ZIP already exists.
outdir=fullfile(root,'TESTS','RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS');
base=['PURE_TRIGGER_V6_CAUSAL_AUDIT_' stamp];
ix=1;
while exist(fullfile(outdir,[base '.zip']),'file')
    ix=ix+1;base=['PURE_TRIGGER_V6_CAUSAL_AUDIT_' stamp '_' num2str(ix)];
end
matPath=fullfile(outdir,[base '.mat']);
txtPath=fullfile(outdir,[base '.txt']);
zipPath=fullfile(outdir,[base '.zip']);
save(matPath,'R','-v7');
fid=fopen(txtPath,'w');
assert(fid>0,'Cannot write V6 results');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'V6 OFFLINE CAUSAL HLC PREDICTION AUDIT\n');
fprintf(fid,'Status: %s\nError: %s\n',R.status,R.errorText);
if isstruct(R.sourceContract) && isfield(R.sourceContract,'resolvedPredictorPath')
    fprintf(fid,'Predictor source resolved: %s\n',R.sourceContract.resolvedPredictorPath);
end
if isstruct(R.original6) && isfield(R.original6,'results') && ~isempty(R.original6.results)
    fprintf(fid,'Original HLC 6s per-case report:\n');
    for kr=1:numel(R.original6.results)
        a=R.original6.results(kr);
        fprintf(fid,'  %s: %s, error=%s\n',a.caseName,a.status,a.errorText);
    end
end
fprintf(fid,'Original HLC and 24state used ONLY for 6s comparison baseline.\n');
fprintf(fid,'Four-second candidate: modal closed-loop+governor, using history until t0=1/2s.\n');
fprintf(fid,'Route cache from known ORIGINAL route (not future measured trajectory).\n');
fprintf(fid,'Estimated mass only. No plant true mass to predictor.\n');
fprintf(fid,'NOT physical error certified. NOT real-EKF validated.\n');
fprintf(fid,'NOT ready for actual-CBF safe clearance or online integration.\n');
fprintf(fid,'NOT 1-2ms qualified (geometry and preprocessing not included).\n');
if isstruct(R.causalForecast) && isfield(R.causalForecast,'records')
    fprintf(fid,'case,startTime_s,loadMax_m,droneMax_m,govTauMax_s,govCmdMax,refMax_m,cableAngleMax_rad,medianCandidateMs,modalHistoryCondition\n');
    for j=1:numel(R.causalForecast.records)
        a=R.causalForecast.records(j);
        fprintf(fid,'%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n', ...
            a.caseName,a.startTime_s,a.maxLoadError_m,a.maxDroneError_m, ...
            a.maxGovernorTauError_s,a.maxGovernorCommandError, ...
            a.maxReferencePositionError_m, ...
            a.maxCableDirectionError_rad,a.benchmarkMedianMs, ...
            a.modalHistoryCondition);
    end
end
if isstruct(R.oneStepSaturation) && isfield(R.oneStepSaturation,'status')
    fprintf(fid,'One-step saturation probe: %s\n',R.oneStepSaturation.status);
end
clear cleanup
zip(zipPath,{matPath,txtPath});
R.resultsZip=zipPath; R.resultsMat=matPath;
fprintf('[V6] Final status: %s\n',R.status);
fprintf('[V6] Results ZIP: %s\n',zipPath);
if isstruct(R.causalForecast) && isfield(R.causalForecast,'records')
    disp(struct2table(R.causalForecast.records));
end
end
