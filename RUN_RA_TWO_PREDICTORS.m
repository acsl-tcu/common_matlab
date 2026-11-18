function R=RUN_RA_TWO_PREDICTORS(root,mode,varargin)
% RUN_RA_TWO_PREDICTORS  V21 live-route optimized, exact 6th-jet torque audit; 2026-11-18.
% Two lightweight causal four-second forecast experiments from original Flight:
%   A: isolated audited V8 initial-EKF-anchored code embedded BELOW (1s PV history).
%   B: 4th-order slow-conjugate-pair reduced x/y modes and original 2nd-order z;
%      poles from ORIGINAL HLC gains, missing jerk fitted from past EKF PV,
%      current EKF p/v/q anchored. Prepared once/scenario; no HLC rollout.
%   Each predefined scenario is run TWICE with identical planted mass 0.14 kg:
%   initial estimator/HLC parameter 0.14 kg (matched) vs 0.01 kg (original).
%   EKF and state manager keep updating loadmass normally; NO fixed-mass oracle.
%   Neither forecasts future actuator saturation. B is a MODEL-REDUCTION
%   candidate, NOT equivalent to the nonlinear original HLC/plant.
%   Previous 240ms original-HLC brute-force B removed from ONLINE comparison.
% Reference/plant future and dynamic obstacle world future are SCORE-ONLY.
% The original Flight, HLC, plant, estimator and all source files are untouched.
% V18: time-first, single-pass vectorized C1 cubic separation, no torque surrogate.
% V16: preserve V15 all-window timing and forecasts; accelerate only the
%       nominal observed polyline screen by exact safe AABB separations.
% V15: measure current-observation + forecast + screen at EVERY forecast start;
%       data from same current simulated sensor, no future obstacle oracle.
%       exact full original HLC/EKF/plant/CBF control tick remains UNMEASURED.
%       future torque saturation is an OFFLINE scoring label, NEVER a forecast input.
% V12: compare a CURRENT-OBSERVATION-ONLY 4s nominal piecewise-linear
% continuous interval screen (exact endpoint sphere segment separation and
% conservative cable swept AABB), OFFLINE and separately timed. It does
% NOT certify the actual continuous model, dynamic obstacle future or flight.
% Result summary separately reports pointwise and per-window collision proxies.
% Adds an OFFLINE-only reference/tracking/Governor error decomposition and
% exact-ZOH versus continuous-feedback matrix diagnostic (not a third predictor).
% All persistent TEST RESULTS are written to ONE ZIP in root/RESULTS_RA_TWO_PREDICTORS.
% No MAT/TXT/CSV/fig/logs are written separately to that directory.
% Dynamic obstacles: movement is evaluated ONLY in the oracle score; this
% does NOT claim an online dynamic-obstacle motion predictor.
if nargin<1 || isempty(root),root=pwd;end
if nargin<2 || isempty(mode),mode='all';end
root=char(root);mode=char(mode);
assert(any(strcmp(mode,{'all','smoke','full','stress','saturation','replay'})), ...
    'mode must be all, smoke, full, stress, saturation, or replay');
assert(isfile(fullfile(root,'run_flight_SimSuspendedLoad_trj_obs_sns.m')), ...
    'Expected original run_flight_SimSuspendedLoad_trj_obs_sns.m in common_matlab');
assert(isfile(fullfile(root,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE', ...
    'SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m')) || ...
    isfile(fullfile(root,'reference','WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE', ...
    'SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m')), ...
    'Missing REFERENCE_OBSTACLE_AVOIDANCE installation');
oldPath=path;oldPwd=pwd;
restorePath=onCleanup(@()path(oldPath)); %#ok<NASGU>
restorePwd=onCleanup(@()cd(oldPwd)); %#ok<NASGU>
addpath(genpath(root));
SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
assert(exist('with_load_model_euler_for_HL','file')==2,'Missing ORIGINAL 24-state RHS');
assert(exist('HLC_SUSPENDED_LOAD','class')==8,'Missing ORIGINAL HLC_SUSPENDED_LOAD class');
assert(exist('obs_world_at_time','file')==2,'Missing DRONE_OBSTACLES dynamic code');
if strcmp(mode,'replay')
    % V19.2: no Flight or online forecast! Reuse V19 saved flightAuditRecords.
    sourceZip='';
    if ~isempty(varargin),sourceZip=char(varargin{1});end
    R=ra_v19_1_saved_torque_replay(root,sourceZip);
    return;
end
work=tempname;mkdir(work);
routeFiles=ra_write_test_open_route_sources(work); % temporary, test-only; snapshotted inside result ZIP
addpath(work,'-begin'); rehash;
cleanup=onCleanup(@()removeTmp(work)); %#ok<NASGU>
report=struct();report.timestamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
report.mode=mode;report.originalSourcesUnmodified=true;
report.safetyCertified=false;report.dynamicOracleIsScoringOnly=true;
report.noFutureInputHeldFixed=true;report.futureSaturationNotModeled=true;
report.futurePlantTruthUsedInPredictors=false;
report.twoModelsAreConditionalOnIdealUnsaturatedHLC=true;
report.methodBSourceDerivedNotEmpirical=true;
report.methodBNeedsPVHistoryForJerk=true;
report.referenceErrorSplitIsOfflineOracleScoringOnly=true;
report.knownRouteGeneratorIsUsedForOfflineJetAccuracyAudit=true;
report.discreteAuditDoesNotChangePredictor=true;
report.openNonperiodicC6RoutesIncluded=true;
report.actualAchievedSpeedReported=true;
report.routeIsPreplannedNotOnlineSwitched=true; % Flight scenario only; live evaluator itself accepts new current generator
report.routeJetContinuityAndCacheMeasuredOffTick=true; % now LIVE jet parity audit, NOT a trajectory table
report.flightAuditSupportsMixedPhaseReferenceLengths=true;
report.offlineFlightAuditFailureDoesNotDiscardPredictors=true;
report.methods={'A_ORIGINAL_HLC_6_2_MODAL_PV_HIST','B_HLC_TWO_SLOW_PAIRS_4_2_EKF_PV_HISTORY'};
report.testRevision='V21_LIVE_ROUTE_LOCAL_SNAPSHOT_EXACT_SIXTH_JET_AUDIT';
report.v21NoFutureTrajectoryPrecalculation=true;
report.v21RouteContextRebuiltForEveryTrigger=true;
report.v21RouteContextTimeIncludedInOnlinePipeline=true;
report.v21ExactTorqueSixthJetAuditIsOfflineOnly=true;
report.v21FutureTorquePredictionNotYetValidated=true;
report.v19TorqueFocusIsOfflineOnly=true;
report.v19OriginalHLCPerForecast=false;
report.v19GeometryAndPredictorsUnchangedFromV18=true;
report.v18GeometryIsSinglePassC1Cubic=true;
report.v18CubicHullIsExactContainmentForDefinedInterpolator=true;
report.v18NoCubicRecursion=true;
report.v18MaxActiveObservedParts=64;
report.v18OnlineTargetMs=2.0;
report.v18TorqueApproximationRemoved=true;
report.v18CurrentActuatorSaturationCausesUnknown=true;
report.v18FutureTorqueSaturationIsNotPredicted=true;
report.v18HardRealtimeNotCertified=true;
report.v18CubicUnitTest=ra_v18_fast_cubic_unit_test();
report.v20LiveRouteUnitTest=ra_v20_live_route_selftest();
report.allWindowObservedPipelineTimeMeasured=true;
report.broadphaseIsExactSafeForDefinedPolyline=false;
report.broadphaseIsExactSafeForDefinedCubic=true;
report.groundAndDynamicFutureStillUncertified=true;
report.broadphaseComparedAgainstV15ReferenceOffline=true;
report.pipelineIncludesCurrentSimulatedSensor=true;
report.pipelineExcludesOriginalFlightControllerEKFPlant=true;
report.futureTorqueSaturationUsedOnlyForOfflineScoring=true;
report.v20NoPregeneratedFutureRoute=true;
report.v20CurrentRouteQueriedOnDemand=true;
report.v20SupportedRouteClasses={'open_waypoints','periodic_fourier'};
report.v20FutureTorqueSaturationStillUnsolved=true;
report.v20OnlineSwitchedRouteFlightNotYetTested=true;
report.continuousCertificateRestrictedToDefinedCubicAndFrozenObservedObjects=true;
report.continuousCertificateRestrictedToDefinedPolylineAndFrozenObservedObjects=false;
report.truePlantLoadMassKg=0.14;
report.initialLoadMassCasesKg=[0.14,0.01];
report.massIsEstimatedOnlineInBothCases=true;
report.initialMassMatchedIsNotFixedDuringFlight=true;
report.predictionEquationsUnmodifiedFromV12=true;
report.referenceTrajectoryAndScenarioAreMatchedPerPair=true;
report.rngIsResetToSameSeedForTwoRunsInEachPair=true;
report.intervalDiagnosticsAreOfflineNotInPredictorTiming=true;
report.intervalClearAppliesOnlyToPolylineSurrogate=true;
report.obstacleObservationCurrentOnly=true;
report.currentEKFGeometryNotSilentlyProjected=true;
report.intervalSelfTest=ra_nominal_interval_self_test();
report.localPVCacheSHA256='68a5dcd6a10e50eac2a300d4d54f0809caad90f38762b3c634f18ea1f0c4a1cf';
report.localCachePreparedOncePerFlightScenario=true; % only fixed MODAL matrices, NOT future trajectory
report.referenceV8SourceSHA256Prefix='831e73d23aa4a1096225'; % archived original BEFORE local-function rename
report.methodAUsesOriginalHLCPerForecast=false;report.methodBUsesOriginalHLCPerForecast=false;
report.sourceResolution=struct('pv',which('wp_six_trigger_pv_forecast_v8'), ...
 'governor',which('wp_six_trigger_governor_fast_step'), ...
 'routeCache',which('wp_six_trigger_route_cache_prepare_v8'), ...
 'pvCache',which('wp_six_trigger_pv_cache_prepare'), ...
 'runner',mfilename('fullpath'), ...
 'testedA','LOCAL_FUNCTION_ra_test_modal_anchored_v8', ...
 'testedACache','LOCAL_FUNCTION_ra_test_pv_cache_prepare_v7');
try
  aa=fileread(report.sourceResolution.pv);
  report.sourceResolution.installedAnchorTokensSeen= ...
     contains(aa,'fixedEKFErrorP') && contains(aa,'initial.EKFLoadMismatch_m');
catch
  report.sourceResolution.installedAnchorTokensSeen=false;
end
report.note=['A: original ideal continuous-time HLC 6/2 modes and PV history. ' ...
 'B: original ideal continuous-time HLC two slow conjugate pairs (4/2) ' ...
 'with EKF current p/v/q anchors and one history-fitted jerk per horizontal axis. ' ...
 'Off-tick exact-ZOH diagnostic is NOT silently used as a replacement model. ' ...
 'Neither forecasts future saturation. V18 removes invalid torque surrogates and duplicated screens; ' ...
 'one bounded-cost C1 cubic hull screen replaces on-tick polyline screening. ' ...
 'Current active route coefficients used online; future plant and obstacle truth for SCORING ONLY.'];
if strcmp(mode,'saturation')
    % Focus ONLY on valid matched-mass examples with and without future
    % TORQUE saturation; per base case the original 0.01-kg run remains
    % available as a paired control, not a 'converged mass' claim.
    scenarios={ ...
        struct('name','straight_static','route','gen_wp_straight', ...
          'speed',1.50,'scene','scene_cylinders_high_side','te',17,'stride',20), ...
        struct('name','offset_curve_dynamic','route','gen_wp_gentle_turn', ...
          'speed',0.90,'scene','scene_dynamic_independent', ...
          'initialOffset',[0.8;-0.6;0],'te',17,'stride',20), ...
        struct('name','open_turn_v075_dynamic','route','ra_test_wp_straight_curve_c6', ...
          'speed',0.75,'scene','scene_dynamic_independent','te',25,'stride',28), ...
        struct('name','open_3dturn_v090_dynamic','route','ra_test_wp_3d_turn_c6', ...
          'speed',0.90,'scene','scene_dynamic_independent','te',24,'stride',28)};
    stride=20;te=17;
elseif strcmp(mode,'smoke')
    scenarios={struct('name','straight_static','route','gen_wp_straight','speed',1.5,'scene','scene_cylinders_high_side'), ...
               struct('name','curve_dynamic','route','gen_wp_gentle_turn','speed',0.35,'scene','scene_dynamic_independent')};
    stride=40;te=15;
elseif strcmp(mode,'stress')
    scenarios={struct('name','offset_curve_dynamic','route','gen_wp_gentle_turn', ...
            'speed',0.90,'scene','scene_dynamic_independent','initialOffset',[0.8;-0.6;0]), ...
        struct('name','offset_circle_dynamic','route','gen_wp_circle', ...
            'speed',0.75,'scene','scene_dynamic_independent','initialOffset',[-0.7;0.6;0]), ...
        struct('name','offset_straight_static','route','gen_wp_straight', ...
            'speed',2.5,'scene','scene_gate_square','initialOffset',[0.8;0.8;0])};
    stride=20;te=17;
else
    scenarios={struct('name','straight_static','route','gen_wp_straight','speed',1.5,'scene','scene_cylinders_high_side'), ...
               struct('name','straight_dynamic','route','gen_wp_straight','speed',1.5,'scene','scene_dynamic_independent'), ...
               struct('name','curve_static','route','gen_wp_gentle_turn','speed',0.35,'scene','scene_gate_square'), ...
               struct('name','curve_dynamic','route','gen_wp_gentle_turn','speed',0.35,'scene','scene_dynamic_independent'), ...
               struct('name','circle_dynamic','route','gen_wp_circle','speed',0.30,'scene','scene_dynamic_independent'), ...
               struct('name','saddle_static','route','gen_wp_saddle','speed',0.30,'scene','scene_cylinders_high_side')};
    stride=20;te=17;
end
% Default is ONE inclusive run: all six nominal cases plus all three stress
% cases. A failure in one scenario is recorded, not used to skip the rest.
if strcmp(mode,'all')
    scenarios=[scenarios, { ...
        struct('name','offset_curve_dynamic','route','gen_wp_gentle_turn', ...
          'speed',0.90,'scene','scene_dynamic_independent','initialOffset',[0.8;-0.6;0]), ...
        struct('name','offset_circle_dynamic','route','gen_wp_circle', ...
          'speed',0.75,'scene','scene_dynamic_independent','initialOffset',[-0.7;0.6;0]), ...
        struct('name','offset_straight_static','route','gen_wp_straight', ...
          'speed',2.5,'scene','scene_gate_square','initialOffset',[0.8;0.8;0])}];
end
% NEW V10 scenarios: same original smooth open route at different requested
% speeds, plus S-bends and 3D bends. They are single PREPLANNED nonperiodic
% C6 routes, not a runtime route replanning / cache-switch demonstration.
% Use only in 'all' / 'full', leaving old smoke and stress for regression.
if any(strcmp(mode,{'all','full'}))
    scenarios=[scenarios,{ ...
      struct('name','open_turn_v035_static','route','ra_test_wp_straight_curve_c6', ...
        'speed',0.35,'scene','scene_gate_square','te',38,'stride',52), ...
      struct('name','open_turn_v075_dynamic','route','ra_test_wp_straight_curve_c6', ...
        'speed',0.75,'scene','scene_dynamic_independent','te',25,'stride',28), ...
      struct('name','open_turn_v150_dynamic','route','ra_test_wp_straight_curve_c6', ...
        'speed',1.50,'scene','scene_dynamic_independent','te',20,'stride',24), ...
      struct('name','open_turn_v250_static','route','ra_test_wp_straight_curve_c6', ...
        'speed',2.50,'scene','scene_gate_square','te',19,'stride',24), ...
      struct('name','open_sbend_v060_dynamic','route','ra_test_wp_s_bend_c6', ...
        'speed',0.60,'scene','scene_dynamic_independent','te',29,'stride',36), ...
      struct('name','open_3dturn_v090_dynamic','route','ra_test_wp_3d_turn_c6', ...
        'speed',0.90,'scene','scene_dynamic_independent','te',24,'stride',28), ...
      struct('name','open_turn_offset_v120','route','ra_test_wp_straight_curve_c6', ...
        'speed',1.20,'scene','scene_dynamic_independent', ...
        'initialOffset',[0.65;-0.40;0],'te',21,'stride',28) ...
        }];
end
% Paired causal experiment: SAME plant truth, geometry, route, nominal speed,
% offsets and random seed. ONLY initial mass given to EKF/HLC changes.
baseScenarios=scenarios;
scenarios=cell(1,2*numel(baseScenarios));
for bi=1:numel(baseScenarios)
    for vi=1:2
        sc=baseScenarios{bi};
        sc.baseName=sc.name;
        sc.truePlantMassKg=0.14;
        sc.rngSeed=261118+bi;
        if vi==1
            sc.massCase='MATCHED_INITIAL_0p14kg';
            sc.initialEstimatorHLCMassKg=0.14;
            suffix='__MATCHED';
        else
            sc.massCase='ORIGINAL_INITIAL_0p01kg';
            sc.initialEstimatorHLCMassKg=0.01;
            suffix='__ORIGINAL';
        end
        sc.name=[sc.name suffix];
        scenarios{2*bi-2+vi}=sc;
    end
end
oldRandomState=rng;
restoreRandomState=onCleanup(@()rng(oldRandomState)); %#ok<NASGU>
report.massPairCount=numel(baseScenarios);
report.massRunCount=numel(scenarios);
report.scenarios=cell(1,numel(scenarios));
for ci=1:numel(scenarios)
    sc=scenarios{ci};
    item=struct('scenario',sc,'status','NOT_RUN','error','', ...
        'forecastSamples',0,'flightSamples',0,'methodA',[], 'methodB',[], ...
        'savedFlightTraceForAudit',false);
    try
        thisTe=te;thisStride=stride;
        if isfield(sc,'te'),thisTe=sc.te;end
        if isfield(sc,'stride'),thisStride=sc.stride;end
        % Use the same PRNG state for BOTH initial-mass branches of a pair.
        % The exact number of draws may differ later if model execution branches.
        rng(sc.rngSeed,'twister');
        tr=runFlightGenerated(root,work,sc,ci,thisTe);
        item.flightSamples=numel(tr.records);
        % Mass history is recorded before either forecast/scoring. A forecast
        % failure therefore cannot discard this separate mass audit.
        item.massAudit=ra_mass_trajectory_audit(tr.records,sc.truePlantMassKg);
        item.massAudit.plantConstructionMassKg=tr.truePlantMassKg;
        item.massAudit.initialEstimatorSetupMassKg=tr.initialEstimatorSetupMassKg;
        item=scoreBoth(tr,sc,item,thisStride);
        % V19: exact ORIGINAL HLC algebra is replayed ONLY AFTER Flight and
        % AFTER the two online forecasts. Never enters timed pipeline.
        try
            item.v19TorqueAudit=ra_v19_original_torque_audit(tr.records,tr.gains);
            if item.v19TorqueAudit.exactReplayPassed
                item.v19TorqueAuditStatus='PASS';
            else
                item.v19TorqueAuditStatus='MISMATCH_OR_NO_VALID_REPLAY';
            end
        catch MEtorque
            item.v19TorqueAuditStatus='FAILED';
            item.v19TorqueAuditError=getReport(MEtorque,'extended','hyperlinks','off');
        end
        % Honest post-convergence evaluation, entirely OFFLINE. Selected
        % windows start with 1s of past EKF mass within 10% of true mass.
        % Actual full-horizon admissibility is SCORE-ONLY (never an input
        % to either predictor, and not a real-time gating oracle).
        item.postConvergence=ra_mass_converged_window_scores(tr.records,item,sc.truePlantMassKg);
        if (isfield(item,'nominalIntervalAuditStatus') && ...
                strcmp(item.nominalIntervalAuditStatus,'FAILED')) || ...
           (isfield(item,'directKnownRouteAuditStatus') && ...
                strcmp(item.directKnownRouteAuditStatus,'FAILED'))
            item.status='PARTIAL_V12_DIAGNOSTIC_FAILURE';
            item.error='See nominalIntervalAuditError/directKnownRouteAuditError; A/B scores retained';
        elseif isfield(item,'flightDomainAuditStatus') && ...
                strcmp(item.flightDomainAuditStatus,'FAILED')
            item.status='PARTIAL_FLIGHT_AUDIT_FAILURE';
            item.error=item.flightDomainAuditError;
        elseif isfield(item,'v19TorqueAuditStatus') && ...
                ~strcmp(item.v19TorqueAuditStatus,'PASS')
            item.status='PARTIAL_V19_TORQUE_AUDIT_FAILURE';
            item.error='Original HLC torque replay did not validate; see v19TorqueAuditStatus/v19TorqueAuditError';
        elseif item.methodA.valid==0 || item.methodB.valid==0
            item.status='FAILED_PREDICTOR';
            item.error='At least one predictor produced zero valid forecasts; inspect internal errorMessages.';
        elseif item.methodA.failed>0 || item.methodB.failed>0
            item.status='PARTIAL_PREDICTOR_FAILURES';
        elseif strncmp(sc.route,'ra_test_wp_',11) && ...
                (~isfield(item,'routeAudit') || ...
                 item.routeAudit.bendCount==0 || ...
                 item.routeAudit.forecastWindowsThroughBend==0)
            item.status='INCOMPLETE_TRANSITION_COVERAGE';
            item.error='No 4s forecast window overlapped a C6 spatial turn; adjust timing/speed and rerun.';
        else
            item.status='SCORED_NOT_CERTIFIED';
        end
    catch ME
        item.status='FAILED';item.error=getReport(ME,'extended','hyperlinks','off');
    end
    report.scenarios{ci}=item;
    fprintf('[RA TWO] %-22s %s; forecasts=%d\n',sc.name,item.status,item.forecastSamples);
end
report.massPairs=cell(1,report.massPairCount);
for pi=1:report.massPairCount
    report.massPairs{pi}=ra_pair_mass_outcomes(report.scenarios{2*pi-1},report.scenarios{2*pi});
end
report.massNormalPairs=sum(cellfun(@(x)strcmp(x.pairClass,'BOTH_FLIGHTS_MODEL_CONSISTENT'),report.massPairs));
report.massInvalidPairs=sum(cellfun(@(x)strcmp(x.pairClass,'MODEL_INVALID_AT_LEAST_ONE_FLIGHT'),report.massPairs));
passed=sum(cellfun(@(x)strcmp(x.status,'SCORED_NOT_CERTIFIED'),report.scenarios));
if passed==numel(report.scenarios)
    report.status='COMPLETED_COMPARISON_NOT_SAFETY_CERTIFIED';
else
    report.status='INCOMPLETE_OR_FAILED';
end
outdir=fullfile(root,'RESULTS_RA_TWO_PREDICTORS');
if ~isfolder(outdir),mkdir(outdir);end
zipName=['RA_TWO_PREDICTORS_' report.timestamp '.zip'];
resultTemp=fullfile(work,'RA_TWO_PREDICTORS_RESULT.mat');
% Save even partial successes and full errors. Single ZIP is the only result.
save(resultTemp,'report','-v7.3');
summ=fullfile(work,'SUMMARY.txt');fid=fopen(summ,'w');
assert(fid>=0,'Could not create internal ZIP summary');
fprintf(fid,'Status: %s\n',report.status);
fprintf(fid,'Mode: %s | TEST V15: V14 paired TRUE plant mL=0.14 kg, INIT EKF/HLC=0.14 vs 0.01 kg, online EKF NEVER frozen.\n',mode);
fprintf(fid,'RUNS: %d runs = %d route/scene/velocity/offset pairs x 2 initial mass cases. Same PRNG seed within each pair.\n',numel(scenarios),report.massPairCount);
fprintf(fid,'A=HLC ideal 6/2 modal PV history. B=HLC ideal slow-pair 4/2, EKF p/v/q anchored, unknown jerk fitted from PAST PV only.\n');
fprintf(fid,'This experiment changes ONLY the initial estimator/HLC load mass after constructing the plant, never actual plant mass or EKF update laws.\n');
fprintf(fid,'V18: NO torque proxy, NO correction, NO double screen or recursive subdivision. Single-pass conservative cubic C1 Hermite hull test using 161 25ms knots. Separation is for DEFINED cubic with FROZEN observed shapes, not true HLC between knots. TIME >2ms means UNKNOWN, not real-time deadline preemption.\n');
fprintf(fid,'V18: ALL selected forecast starts: timing CURRENT simulated observation + PREDICTOR + single conservative 161-knot cubic-hull screen. Exact full original HLC/EKF/plant/CBF tick is NOT measured, and simulated sensing is NOT hardware sensor latency.\n');
fprintf(fid,'V19 SATURATION FOCUS: EXACT original HLC torque equations replayed on RECORDED EKF states at first torque clip and prior ticks OFFLINE ONLY. No future saturated commands or HLC evaluations entered A/B forecast, timing, or collision decision.\n');

fprintf(fid,'V20: NO PRECOMPUTED FUTURE ROUTE SAMPLES. The active generator geometry and analytic time law are evaluated in a vectorized batch for CURRENT predicted virtual times. Only modal matrices use offline preparation; route changes require passing the current generator snapshot. No future saturated-torque predictor is claimed.\n');
fprintf(fid,'V21: current generator is freshly snapshotted at EACH trigger (ONLY segment unit vectors / Fourier coefficients; NO FUTURE samples). Snapshot preparation is INCLUDED in all-window pipeline time. Smoothstep uses same exact analytic polynomial in faster factored/Horner form.\n');
fprintf(fid,'V21: exact SOURCE-DERIVED 6th-jet torque sensitivity matrix is audited OFFLINE at first clip, no future HLC rollout; future torque saturation remains UNSOLVED.\n');
fprintf(fid,'V21: first saved forecast input is re-run 5 times OFFLINE to diagnose JIT/timing spikes; these calls do NOT enter the online timing measurements.\n');
fprintf(fid,'V15: torque clipping onset for each future 4s window is OFFLINE score-only; neither predictor uses future actuator saturation, so this is a diagnosis, NOT saturation-corrected prediction.\n');
fprintf(fid,'VALIDITY: normal matched-vs-original comparisons are NOT mixed with invalid mass<=0 or gross EKF cable inconsistency/speed anomalies. Criteria below are diagnostic, not safety standards.\n');
fprintf(fid,'VALIDITY DIAGNOSTIC: flight-phase mL<=0/nonfinite, EKF cable endpoint residual>1cm, EKF load speed>5m/s means MODEL_INVALID. Results retained, not used to claim model accuracy.\n');
fprintf(fid,'MASS CONVERGENCE DIAGNOSTIC: first sustained >=1s within 10%% of 0.14kg; no demand or guarantee that EKF converges. Full mass traces saved.\n');
fprintf(fid,'A post-convergence scored 4-second window requires 1s of PAST EKF mass within 10pct of true AND future 4s mass stays within 10pct with physically consistent EKF state. Future check SCORING ONLY, never online.\n');
fprintf(fid,'Pairwise errors are comparable ONLY for same forecast start time where BOTH runs have valid postconvergence 4s windows. NO COMPARABLE WINDOWS is a scientific outcome.\n');
fprintf(fid,'PAIR SUMMARY: physically consistent (NOT necessarily mass-converged) pairs=%d, invalid pairs=%d, other incomplete=%d.\n',report.massNormalPairs,report.massInvalidPairs,report.massPairCount-report.massNormalPairs-report.massInvalidPairs);
for pi=1:report.massPairCount
    p=report.massPairs{pi};
    fprintf(fid,'PAIR [%s]: %s | RAW UNFILTERED A max load err MATCHED/ORIGINAL=%.6g/%.6g m | B=%.6g/%.6g m | saturation ticks=%g/%g | final flight mass=%.6g/%.6g kg | first sustained 10pct near truth=%.5g/%.5g s\n', ...
        p.baseName,p.pairClass,p.maxLoadErrorA_Matched_m,p.maxLoadErrorA_Original_m, ...
        p.maxLoadErrorB_Matched_m,p.maxLoadErrorB_Original_m, ...
        p.saturationTicksMatched,p.saturationTicksOriginal, ...
        p.massAtEndMatched_kg,p.massAtEndOriginal_kg, ...
        p.sustainedMassWithin10PctMatched_s,p.sustainedMassWithin10PctOriginal_s);
    fprintf(fid,'  mass compare: max estimator-vs-controller parameter diff=%.6g/%.6g kg; max true plant flight position difference=%.6g m (SCORING ONLY)\n', ...
        p.maxMassParameterDisagreementMatched_kg,p.maxMassParameterDisagreementOriginal_kg,p.maxTrueLoadTrajectoryDifference_m);
    fprintf(fid,'  POSTCONVERGENCE JOINTLY MATCHED WINDOWS=%d (of these unsaturated=%d): A matched/original=%.6g/%.6g m; B=%.6g/%.6g m; A unsat=%.6g/%.6g m | %s\n', ...
        p.comparableConvergedWindowCount,p.comparableConvergedUnsaturatedWindowCount, ...
        p.comparableA_MaxErrMatched_m,p.comparableA_MaxErrOriginal_m, ...
        p.comparableB_MaxErrMatched_m,p.comparableB_MaxErrOriginal_m, ...
        p.comparableA_NoSatMaxErrMatched_m,p.comparableA_NoSatMaxErrOriginal_m, ...
        p.convergenceComparison);
end
fprintf(fid,'\nPER-RUN RESULTS (both initial masses separately)\n');
fprintf(fid,'Neither A nor B forecasts future actuator saturation. B slow conjugate pairs are selected OFF-TICK from original HLC gains.\n');
fprintf(fid,'HLC gains are from lqrd: CONTINUOUS feedback matrix exp((A-BK)dt) is generally not EXACT ZOH (Ad-BdK). We report both, without switching the predictor.\n');
fprintf(fid,'Reference split is OFFLINE oracle-only scoring, not an online future reference input. Sum of signed vector components is checked numerically.\n');
fprintf(fid,'Dynamic obstacle future world is used ONLY for offline scoring; proxy is a diagnostic, NOT online obstacle prediction.\n');
fprintf(fid,'V12: adds current EKF cable endpoint consistency, current observed obstacle-only nominal LINEAR-SWEEP continuous interval sufficient exclusion (offline separately timed, not a physical guarantee). A/B forecast equations unchanged.\n');
fprintf(fid,'Interval diagnostic applies to the 41-knot PIECEWISE-LINEAR defined surrogate and currently observed frozen obstacle covers; cannot certify actual future continuous HLC motion or moving obstacle future.\n');
fprintf(fid,'Self-test of between-node encounter and interval screen: %s; V15-equivalence trials=%d\n',report.intervalSelfTest.status,report.intervalSelfTest.differentialTrials);
  fprintf(fid,'Requested speed, achieved route profile speed, FLIGHT-phase reference speed and full-Flight reference speed are DISTINCT; 0-5s takeoff contaminates a full-run max.\n');
fprintf(fid,'Open-route path C6 property derives from 13th-degree smoothstep; PCHIP cache is NOT C6. Route source files are bundled in this ZIP.\n');
fprintf(fid,'These are fixed known paths with mid-flight geometry transitions; online route REPLACEMENT / cache invalidation is not tested.\n');
fprintf(fid,'B reduced modes are original-gain-derived; its only unknown high-order initial condition uses past PV. No arbitrary fit coefficients or added safety radii.\n');
fprintf(fid,'Installed V8 at: %s\n',report.sourceResolution.pv);
fprintf(fid,'A evaluated using test-local audited V8 anchor and matching test-local PV cache; installed anchor tokens seen=%d\n', ...
    report.sourceResolution.installedAnchorTokensSeen);
for ci=1:numel(report.scenarios)
    item=report.scenarios{ci};
    fprintf(fid,'\n[%s] %s forecasts=%d | TRUE mL=%.5gkg INIT=%.5gkg | seed=%d\n', ...
        item.scenario.name,item.status,item.forecastSamples,item.scenario.truePlantMassKg, ...
        item.scenario.initialEstimatorHLCMassKg,item.scenario.rngSeed);
    if isfield(item,'massAudit')
        mm=item.massAudit;
        fprintf(fid,'MASS TREND: EKF initial=%.8gkg end-flight=%.8gkg min/max flight=%.8g/%.8gkg RMS deviation from plant=%.6gkg\n', ...
           mm.firstFlightMassKg,mm.lastFlightMassKg,mm.flightMinMassKg,mm.flightMaxMassKg,mm.flightMassRMSErrorKg);
        fprintf(fid,'MASS at physical times %s sec = %s kg; sustained 1s within 10pct at t=%g sec; within 5pct at t=%g sec\n', ...
           mat2str(mm.sampleTimes_s),mat2str(mm.massAtSampleTimes_kg,5),mm.firstSustained10PctTime_s,mm.firstSustained5PctTime_s);
        fprintf(fid,'Mass controller/estimator max disagreement %.7g kg, firstFlight controller mass=%.7g kg\n', ...
            mm.maxControllerEstimatorMassDifferenceKg,mm.firstFlightControllerMassKg);
        fprintf(fid,'Flight valid model state diagnostic=%d; invalid flags: nonpositive mass=%d cable endpoint>1cm=%d extreme speed>5m/s=%d\n', ...
            mm.flightIsModelConsistent,mm.flightBadMassCount,mm.flightCableEndpointOver1cmCount,mm.flightLoadSpeedOver5mpsCount);
        if isfield(item,'postConvergence')
          pc=item.postConvergence;
          fprintf(fid,'POSTCONVERGENCE 1s-past EKF mass within 10pct: start count=%d; full 4s mass+physical-state-consistent scoring count=%d; latter without future saturation=%d. OFFLINE masks, NOT causal forecast inputs.\n', ...
            pc.past1sConvergedStartCount,pc.fullHorizonScorableWindowCount,pc.fullHorizonScorableUnsaturatedWindowCount);
          fprintf(fid,'POSTCONVERGENCE A: maxLoad=%.6g m at1/2/4s=[%.6g %.6g %.6g] | B maxLoad=%.6g m at1/2/4s=[%.6g %.6g %.6g]; NaN=no eligible valid 4s forecasts.\n', ...
            pc.A.maxLoadErrorM,pc.A.maxAt1sM,pc.A.maxAt2sM,pc.A.maxAt4sM, ...
            pc.B.maxLoadErrorM,pc.B.maxAt1sM,pc.B.maxAt2sM,pc.B.maxAt4sM);
          fprintf(fid,'POSTCONVERGENCE WITHOUT FUTURE SATURATION A/B maxLoad=%.6g/%.6g m (OFFLINE DIAGNOSTIC).\n', ...
            pc.A.noSatMaxLoadErrorM,pc.B.noSatMaxLoadErrorM);
        end
    end
    if isstruct(item.methodA) && isfield(item.methodA,'valid')
        if isfield(item,'routeAudit') && ~isempty(item.routeAudit)
            d=item.routeAudit;
            fprintf(fid,'ROUTE: loop=%d requestedSpeed=%.4g m/s actualPeak=%.4g m/s pathLength=%.4g m duration=%.4g s\n', ...
                d.isLoop,d.requestedSpeed,d.actualPeakSpeed,d.pathLength,d.duration);
            fprintf(fid,'ROUTE: max Flight HLC-reference speed=%.4g m/s; max EKF load speed=%.4g m/s (not forecast inputs)\n', ...
                d.maxFlightReferenceLoadSpeed_mps,d.maxFlightEKFLoadSpeed_mps);
            fprintf(fid,'ROUTE FLIGHT-PHASE ONLY: max HLC-reference speed=%.4g m/s; max EKF load speed=%.4g m/s (takeoff excluded)\n', ...
              d.maxFlightPhaseReferenceLoadSpeed_mps,d.maxFlightPhaseEKFLoadSpeed_mps);
            fprintf(fid,'ROUTE: bendCount=%d forecastWindowsThroughBend=%d transitionObservedInFlight=%d\n', ...
                d.bendCount,d.forecastWindowsThroughBend,d.transitionObservedInFlight);
            fprintf(fid,'C6 path boundary (numerical diagnostic, 0..6): normalizedMaxJump=%.6g ; malformedJetCount=%d\n', ...
                d.maxRelativePathJetBoundaryJump,d.malformedJetCount);
            fprintf(fid,'Route PCHIP cache vs exact generator: max |p|=%.7g m, |v|=%.7g m/s, |a|=%.7g m/s^2 (NOT certified)\n', ...
                d.maxCachePositionError_m,d.maxCacheVelocityError_mps,d.maxCacheAccelerationError_mps2);
            if d.bendCount>0 && d.forecastWindowsThroughBend==0
                fprintf(fid,'WARNING: no scored 4s window crosses the C6 spatial turn; results do not exercise midflight bending.\n');
            end
        end
        fprintf(fid,'A: valid=%d failed=%d medianMs=%.4f p95Ms=%.4f maxMs=%.4f maxErrL=%.5g maxErrD=%.5g\n', ...
           item.methodA.valid,item.methodA.failed,item.methodA.medianMs,item.methodA.p95Ms,item.methodA.maxMs, ...
           item.methodA.maxLoadErrorM,item.methodA.maxDroneErrorM);
        fprintf(fid,'B: valid=%d failed=%d medianMs=%.4f p95Ms=%.4f maxMs=%.4f maxErrL=%.5g maxErrD=%.5g\n', ...
           item.methodB.valid,item.methodB.failed,item.methodB.medianMs,item.methodB.p95Ms,item.methodB.maxMs, ...
           item.methodB.maxLoadErrorM,item.methodB.maxDroneErrorM);
        fprintf(fid,'A: load max at 1/2/4s = %.5g / %.5g / %.5g m; pointwise proxy misses=%d\n', ...
            item.methodA.maxLoadErrorAt1s_m,item.methodA.maxLoadErrorAt2s_m, ...
            item.methodA.maxLoadErrorAt4s_m,item.methodA.pointwiseScoringOnlyProxyFalseNegativeSamples);
        fprintf(fid,'B: load max at 1/2/4s = %.5g / %.5g / %.5g m; pointwise proxy misses=%d\n', ...
            item.methodB.maxLoadErrorAt1s_m,item.methodB.maxLoadErrorAt2s_m, ...
            item.methodB.maxLoadErrorAt4s_m,item.methodB.pointwiseScoringOnlyProxyFalseNegativeSamples);
        fprintf(fid,'A OFFLINE split: max reference=%.6g m, tracking=%.6g m, tau=%.6g s, decomposition residual=%.4g m\n', ...
            item.methodA.maxReferenceErrorM,item.methodA.maxTrackingErrorM, ...
            item.methodA.maxGovernorTauErrorS,item.methodA.maxDecompositionResidualM);
        fprintf(fid,'B OFFLINE split: max reference=%.6g m, tracking=%.6g m, tau=%.6g s, decomposition residual=%.4g m\n', ...
            item.methodB.maxReferenceErrorM,item.methodB.maxTrackingErrorM, ...
            item.methodB.maxGovernorTauErrorS,item.methodB.maxDecompositionResidualM);
        fprintf(fid,'A future saturation/no-saturation load error max: %.6g / %.6g m; B: %.6g / %.6g m (NaN = no eligible windows)\n', ...
            item.methodA.maxLoadErrorFutureSaturationM,item.methodA.maxLoadErrorNoFutureSaturationM, ...
            item.methodB.maxLoadErrorFutureSaturationM,item.methodB.maxLoadErrorNoFutureSaturationM);
        fprintf(fid,'A split at 1/2/4s ref=[%.5g %.5g %.5g] tracking=[%.5g %.5g %.5g] (m)\n', ...
            item.methodA.refErrorAt1sM,item.methodA.refErrorAt2sM,item.methodA.refErrorAt4sM, ...
            item.methodA.trackingErrorAt1sM,item.methodA.trackingErrorAt2sM,item.methodA.trackingErrorAt4sM);
        fprintf(fid,'B split at 1/2/4s ref=[%.5g %.5g %.5g] tracking=[%.5g %.5g %.5g] (m)\n', ...
            item.methodB.refErrorAt1sM,item.methodB.refErrorAt2sM,item.methodB.refErrorAt4sM, ...
            item.methodB.trackingErrorAt1sM,item.methodB.trackingErrorAt2sM,item.methodB.trackingErrorAt4sM);
        fprintf(fid,'OFF-TICK original integrator-chain audit: max ||Phi_cont-Phi_ZOH||_inf=%.7g ; max discrete-vs-cont pole modulus difference=%.7g\n', ...
            item.zohVsContinuous.maxMatrixDifferenceInf,item.zohVsContinuous.maxPoleModulusDifference);
        fprintf(fid,'A: satHistoryWindows=%d satFutureWindows=%d proxyFN=%d proxyFP=%d\n', ...
           item.methodA.pastSaturationWindows,item.methodA.trueFutureSaturationWindows, ...
           item.methodA.sampledProxyFalseNegativeFutureWorld,item.methodA.sampledProxyFalsePositiveFutureWorld);
        fprintf(fid,'B: satHistoryWindows=%d satFutureWindows=%d proxyFN=%d proxyFP=%d\n', ...
           item.methodB.pastSaturationWindows,item.methodB.trueFutureSaturationWindows, ...
           item.methodB.sampledProxyFalseNegativeFutureWorld,item.methodB.sampledProxyFalsePositiveFutureWorld);
        if item.methodA.trueFutureSaturationWindows==0 && item.methodB.trueFutureSaturationWindows==0
            fprintf(fid,'SATURATION_FORECAST_UNTESTED: no selected forecast window contained applied HLC saturation\n');
        end
        fprintf(fid,'Flight saturationTicks=%d; scored proxy contacts A truth=%d pred=%d; B truth=%d pred=%d\n', ...
           item.hlcSaturationSamplesInWholeFlight, ...
           item.methodA.sampledTruthProxyContactCount,item.methodA.sampledPredictionProxyContactCount, ...
           item.methodB.sampledTruthProxyContactCount,item.methodB.sampledPredictionProxyContactCount);
        if isfield(item,'flightDomainAudit')
            x=item.flightDomainAudit;
            fprintf(fid,'FLIGHT INPUT AUDIT: firstAnySat=%.5g s firstThrustSat=%.5g s firstTorqueSat=%.5g s; saturation axis counts [F tx ty tz]=%s\n', ...
                x.firstAnySaturation_s,x.firstThrustSaturation_s,x.firstTorqueSaturation_s,mat2str(x.saturationCountsByAxis));
            fprintf(fid,'FLIGHT INPUT AUDIT: max|raw HLC [F tx ty tz]|=%s; max|applied HLC|=%s\n', ...
                mat2str(x.peakAbsoluteRawInput,5),mat2str(x.peakAbsoluteAppliedInput,5));
            fprintf(fid,'FLIGHT STATE AUDIT: peak EKF load speed %.6g m/s at %.5g s; peak EKF mL %.6g kg at %.5g s; range [%.6g %.6g] kg; nonpositive/nonfinite mass ticks=%d\n', ...
                x.peakEKFLoadSpeed_mps,x.peakEKFLoadSpeedTime_s, ...
                x.peakEstimatedMass_kg,x.peakEstimatedMassTime_s, ...
                x.minEstimatedMass_kg,x.maxEstimatedMass_kg,x.badEstimatedMassTicks);
            fprintf(fid,'FLIGHT STATE AUDIT: max physical cable length residual %.6g m; max EKF cable endpoint residual %.6g m; peak true load tracking distance %.6g m\n', ...
                x.maxPhysicalCableLengthResidual_m,x.maxEKFCableEndpointResidual_m,x.maxTruthLoadTrackingDistance_m);
            fprintf(fid,'FLIGHT REFERENCE: jet lengths across all phases=%s; non-28 ticks=%d; flight-phase non-28 ticks=%d\n', ...
                mat2str(x.referenceJetLengths),x.referenceNon28Ticks,x.flightReferenceNon28Ticks);
            fprintf(fid,'DIAGNOSTIC ONLY: large raw control values or EKF drift do NOT prove which mechanism caused the instability. No predictor formula or HLC is modified here.\n');
        elseif isfield(item,'flightDomainAuditStatus') && strcmp(item.flightDomainAuditStatus,'FAILED')
            fprintf(fid,'FLIGHT_DOMAIN_AUDIT_FAILED (A/B forecasts preserved):\n%s\n',item.flightDomainAuditError);
        end
        if ~isempty(item.methodA.sampledProxyMissStartTimes_s)
            fprintf(fid,'A proxy miss window starts (s): %s\n',mat2str(item.methodA.sampledProxyMissStartTimes_s(:)',5));
        end
        if ~isempty(item.methodB.sampledProxyMissStartTimes_s)
            fprintf(fid,'B proxy miss window starts (s): %s\n',mat2str(item.methodB.sampledProxyMissStartTimes_s(:)',5));
        end
        fprintf(fid,'EKF vs plant over full Flight: maxLoad=%.6g m maxDrone=%.6g m\n', ...
            max(item.ekfPlantLoadDifferenceM),max(item.ekfPlantDroneDifferenceM));
        fprintf(fid,'Initial load EKF mismatch A(max)=%.8g m, B(max)=%.8g m\n', ...
           maxFinite(item.methodA.initialLoadAnchorMismatchM),maxFinite(item.methodB.initialLoadAnchorMismatchM));
        fprintf(fid,'Input history/current EKF mismatch A(max)=%.8g m, B(max)=%.8g m\n', ...
           maxFinite(item.methodA.inputHistoryMismatchM),maxFinite(item.methodB.inputHistoryMismatchM));
        fprintf(fid,'A V8 internal-reported initial EKF mismatch(max)=%.8g m\n', ...
           maxFinite(item.methodA.initialModelReportedAnchorMismatchM));
        fprintf(fid,'Initial drone EKF mismatch A(max)=%.8g m, B(max)=%.8g m\n', ...
           maxFinite(item.methodA.initialDroneMismatchM),maxFinite(item.methodB.initialDroneMismatchM));
        if isfield(item,'directKnownRouteAudit')
            k=item.directKnownRouteAudit;
            fprintf(fid,'KNOWN C6 ROUTE direct evaluateWarped vs cached forecast reference: max difference A/B=%.6g/%.6g m; offline direct evaluation median per 4s window A/B=%.4g/%.4g ms. Evaluated only after forecast; not in forecast timing.\n', ...
                k.A.maxPositionDifferenceM,k.B.maxPositionDifferenceM, ...
                k.A.medianMs,k.B.medianMs);
        end
        if isfield(item,'nominalIntervalAuditStatus') && strcmp(item.nominalIntervalAuditStatus,'FAILED')
            fprintf(fid,'V12 interval diagnostic FAILED (forecasts retained): %s\n',item.nominalIntervalAuditError);
        end
        if isfield(item,'directKnownRouteAuditStatus') && strcmp(item.directKnownRouteAuditStatus,'FAILED')
            fprintf(fid,'V12 original C6 route diagnostic FAILED (forecasts retained): %s\n',item.directKnownRouteAuditError);
        end
        if isfield(item,'nominalIntervalAudit')
            q=item.nominalIntervalAudit;
            fprintf(fid,'NOMINAL CONTINUOUS 41-knot polyline vs CURRENT observed frozen obstacles ONLY, diagnostic: A clear/unresolved=%d/%d, B=%d/%d; A/B median ms=%.4g/%.4g (OFFLINE not in forecast time)\n', ...
                q.A.clear,q.A.unresolved,q.B.clear,q.B.unresolved, ...
                q.A.medianMs,q.B.medianMs);
            fprintf(fid,'Nonempty current observation windows=%d; A/B clear despite observed parts=%d/%d (only nominal static snapshot).\n', ...
                q.nonEmptyObservationWindows,q.clearWithObservedPartsA,q.clearWithObservedPartsB);
            fprintf(fid,'Obstacle count current observation selected audit windows=%s; max current EKF cable endpoint inconsistency %.6g m; max initial predicted drone mismatch A/B=%.6g/%.6g m. Both are recorded, NEVER hidden or recentered.\n', ...
                mat2str(q.observedPartCounts),q.maxCurrentEKFEndpointResidualM, ...
                q.maxInitialADroneMismatchM,q.maxInitialBDroneMismatchM);
            fprintf(fid,'Current EKF cable endpoint VELOCITY inconsistency max %.6g m/s (uses omega cross q, never projected).\n',q.maxCurrentEKFEndpointVelocityResidualMps);
            fprintf(fid,'Interpretation: CLEAR_FOR_FROZEN_OBSERVED_POLYLINE_ONLY is NOT actual trajectory continuous-time CLEAR; unresolved means not proved separate, not confirmed collision.\n');
        end
        if item.methodA.valid>0 && isfield(item.methodA,'stageMedianMs') && ~isempty(item.methodA.stageMedianMs)
            tm=item.methodA.stageMedianMs;
            fprintf(fid,'A stages median(ms) fit=%.4f modeAndGov=%.4f route=%.4f reconstruction=%.4f\n', ...
                tm.identification,tm.modeAndGovernor,tm.routeInterpolation,tm.tautReconstruction);
        end
        if item.methodB.valid>0 && isfield(item.methodB,'stageMedianMs') && ~isempty(item.methodB.stageMedianMs)
            tm=item.methodB.stageMedianMs;
            fprintf(fid,'B stages median(ms) offTickPoleBuild=EXCLUDED init=%.4f modeAndGov=%.4f route=%.4f reconstruction=%.4f\n', ...
                tm.identification,tm.modeAndGovernor,tm.routeInterpolation,tm.tautReconstruction);
        end
        % V19 offline-only original HLC identity and perturbation audit.
        if isfield(item,'v19TorqueAudit')
            a19=item.v19TorqueAudit;
            fprintf(fid,'V19 ORIGINAL HLC TORQUE AUDIT OFFLINE: hasClip=%d firstClip=%g s, axis=[%s], replayOK=%d, max abs raw mismatch=%g Nm, max time/call=%g ms.\n', ...
              a19.hasTorqueClip,a19.firstTorqueClipTime_s,num2str(a19.firstTorqueClipAxis(:)'), ...
              a19.exactReplayPassed,a19.maxReplayAbsoluteErrorNm,a19.maxExactReplayTimeMs);
            fprintf(fid,'V19 first clipped or unsaturated-control raw=%s Nm, applied=%s Nm; angular acceleration deficit=%s rad/s2.\n', ...
              mat2str(a19.firstRawTorqueNm(:)',5),mat2str(a19.firstAppliedTorqueNm(:)',5), ...
              mat2str(a19.firstAngularAccelerationDeficitRadS2(:)',5));
            fprintf(fid,'V19 current-state counterfactual: same EKF x, previous reference-jet torque=%s Nm, current jet torque=%s Nm; jet perturbation is OFFLINE NONCAUSAL DIAGNOSIS ONLY.\n', ...
              mat2str(a19.counterfactualPreviousJetTorqueNm(:)',5), ...
              mat2str(a19.counterfactualCurrentJetTorqueNm(:)',5));
            fprintf(fid,'V19 exact HLC audit per-sample details are stored in RESULT.mat. No original HLC replay inside time budget.\n');
            fprintf(fid,'V21 high-order exact HLC torque decomposition (OFFLINE): validated=%d residual=%.9g Nm, pure 6th jet contribution=%s Nm, remainder=%s Nm. NOT a future torque predictor.\n', ...
               a19.highJetDecompositionPassed,a19.highJetAnalyticResidualNm, ...
               mat2str(a19.highJetTorqueNm(:)',5),mat2str(a19.highJetBaseTorqueNm(:)',5));
        elseif isfield(item,'v19TorqueAuditStatus') && strcmp(item.v19TorqueAuditStatus,'FAILED')
            fprintf(fid,'V19 ORIGINAL HLC TORQUE AUDIT FAILED (other A/B results kept): %s\n',item.v19TorqueAuditError);
        end
        if isfield(item,'v21SameInputRepeat')
            q=item.v21SameInputRepeat;
            if strcmp(q.status,'PASS_REPEAT_DETERMINISTIC')
                fprintf(fid,'V21 SAME INPUT OFFLINE REPLAY: original A/B=%.5g/%.5g ms, subsequent repeat A=%s ms B=%s ms. Excluded from pipeline.\n', ...
                    q.firstAOriginalMs,q.firstBOriginalMs,mat2str(q.repeatA_ms,4),mat2str(q.repeatB_ms,4));
            else
                fprintf(fid,'V21 SAME INPUT REPLAY: %s (diagnostic only).\n',q.status);
            end
        end
        % V15: all selected windows, sensor + forecast + nominal interval screen.
        if isfield(item.methodA,'pipeline') && isfield(item.methodB,'pipeline')
            pa=item.methodA.pipeline;pb=item.methodB.pipeline;
            fprintf(fid,'V21 ROUTE CONTEXT: median per-trigger prepare A/B=%.5g/%.5g ms; included in EACH online pipeline time.\n', ...
                pa.medianCurrentRouteSnapshotMs,pb.medianCurrentRouteSnapshotMs);
            fprintf(fid,'V18 TIME-FIRST ONLINE PIPELINE (all selected windows, simulated sensor + A forecast + cubic hull screen): valid=%d failed=%d median/p95/max=%.5g/%.5g/%.5g ms; warm excluding first median/p95/max=%.5g/%.5g/%.5g ms; sensor median=%.5g ms; screen median=%.5g ms; clear/unresolved=%d/%d; observed parts max=%.5g. Not full HLC/EKF/CBF tick.\n', ...
                pa.validSamples,pa.failedObservationOrScreen,pa.medianMs,pa.p95Ms,pa.maxMs, ...
                pa.warmMedianMs,pa.warmP95Ms,pa.warmMaxMs, ...
                pa.medianCurrentSimulatedSensorMs,pa.medianNominalIntervalMs, ...
                pa.clearDefinedCubicWithin2msCount,pa.unresolvedOrOverBudgetCubicCount,pa.maxObservedParts);
            fprintf(fid,'V18 TIME-FIRST ONLINE PIPELINE (all selected windows, simulated sensor + B forecast + cubic hull screen): valid=%d failed=%d median/p95/max=%.5g/%.5g/%.5g ms; warm excluding first median/p95/max=%.5g/%.5g/%.5g ms; sensor median=%.5g ms; screen median=%.5g ms; clear/unresolved=%d/%d; observed parts max=%.5g. Not full HLC/EKF/CBF tick.\n', ...
                pb.validSamples,pb.failedObservationOrScreen,pb.medianMs,pb.p95Ms,pb.maxMs, ...
                pb.warmMedianMs,pb.warmP95Ms,pb.warmMaxMs, ...
                pb.medianCurrentSimulatedSensorMs,pb.medianNominalIntervalMs, ...
                pb.clearDefinedCubicWithin2msCount,pb.unresolvedOrOverBudgetCubicCount,pb.maxObservedParts);
            fprintf(fid,'V18 2ms TOTAL TARGET: A exceeded=%d/%d, B exceeded=%d/%d; 25ms exceed A/B=%d/%d. A/B max are not hard realtime guarantees; HLC/EKF/plant excluded.\n', ...
                pa.exceeded2msCount,pa.validSamples,pb.exceeded2msCount,pb.validSamples, ...
                pa.exceeded25msCount,pb.exceeded25msCount);
            fprintf(fid,'V18 TORQUE CLIP oracle-only 4s windows A/B count=%d/%d; A max load err with/without torque clipping=%.6g/%.6g m, B=%.6g/%.6g m. Not used to generate a prediction.\n', ...
                item.methodA.trueFutureTorqueSaturationWindows,item.methodB.trueFutureTorqueSaturationWindows, ...
                item.methodA.maxErrorFutureTorqueSaturationM,item.methodA.maxErrorWithoutFutureTorqueSaturationM, ...
                item.methodB.maxErrorFutureTorqueSaturationM,item.methodB.maxErrorWithoutFutureTorqueSaturationM);
        end
        if item.methodA.failed>0
            bad=find(~cellfun(@isempty,item.methodA.errorMessages),1,'first');
            if ~isempty(bad)
                fprintf(fid,'A FIRST FAILURE [forecast %d]\n%s\n',bad,item.methodA.errorMessages{bad});
            end
        end
        if item.methodB.failed>0
            bad=find(~cellfun(@isempty,item.methodB.errorMessages),1,'first');
            if ~isempty(bad)
                fprintf(fid,'B FIRST FAILURE [forecast %d]\n%s\n',bad,item.methodB.errorMessages{bad});
            end
        end
    else
        fprintf(fid,'%s\n',item.error);
    end
end
fclose(fid);
% The source snapshot must not discard completed Flight/forecast results.
% MATLAB mfilename('fullpath') returns a path WITHOUT the .m extension.
sourceSnapshot=fullfile(work,'RUN_RA_TWO_PREDICTORS_USED.m');
sourceFile=mfilename('fullpath');
if ~isfile(sourceFile), sourceFile=[sourceFile '.m']; end
snapshotOK=false;
snapshotMessage='';
if isfile(sourceFile)
    try
        [snapshotOK,snapshotMessage]=copyfile(sourceFile,sourceSnapshot);
    catch ME
        snapshotMessage=getReport(ME,'basic','hyperlinks','off');
    end
else
    snapshotMessage=['Cannot find executing source file: ' sourceFile];
end
entries=[{'RA_TWO_PREDICTORS_RESULT.mat','SUMMARY.txt'},routeFiles];
if snapshotOK
    entries{end+1}='RUN_RA_TWO_PREDICTORS_USED.m'; %#ok<AGROW>
else
    fid=fopen(summ,'a');
    if fid>=0
        fprintf(fid,'\nSOURCE_SNAPSHOT_SKIPPED: %s\n',snapshotMessage);
        fclose(fid);
    end
    warning('RA:SnapshotSkipped', ...
        'Source snapshot skipped; ZIP still contains result MAT and summary: %s',snapshotMessage);
end
% First create a complete ZIP in the temporary workspace. The results
% directory receives only ZIPs, never loose MAT/TXT/M files or partial ZIPs.
zipPath=fullfile(outdir,zipName);
assert(~isfile(zipPath),'RA:ResultNameCollision', ...
    'Refusing to overwrite an existing result ZIP: %s',zipPath);
tmpZip=fullfile(work,zipName);
zip(tmpZip,entries,work);
assert(isfile(tmpZip),'RA:ZipNotCreated','Temporary ZIP not created');
[moveOK,moveMessage]=movefile(tmpZip,zipPath);
assert(moveOK,'RA:ZipMoveFailed','Cannot finalize result ZIP: %s',moveMessage);
R=struct('status',report.status,'zipPath',zipPath, ...
 'scenarioStatuses',{cellfun(@(x)x.status,report.scenarios,'UniformOutput',false)}, ...
 'mode',mode,'nScenarios',numel(scenarios));
fprintf('[RA TWO] %s\n[RA TWO] Result ZIP: %s\n',R.status,R.zipPath);
end

function a=ra_v19_original_torque_audit(rec,g)
% Re-evaluate ORIGINAL HLC control-law methods at RECORDed EKF states.
% OFFLINE only. This is NOT a future torque predictor, and its cost is
% NEVER counted as an on-tick cost. Objective: prove a faithful expression
% for raw torque before attempting a lightweight causal future evaluator.
a=struct('hasTorqueClip',false,'firstTorqueClipTime_s',NaN, ...
 'firstTorqueClipAxis',[false false false], ...
 'firstRawTorqueNm',nan(3,1),'firstAppliedTorqueNm',nan(3,1), ...
 'firstAngularAccelerationDeficitRadS2',nan(3,1), ...
 'counterfactualPreviousJetTorqueNm',nan(3,1), ...
 'counterfactualCurrentJetTorqueNm',nan(3,1), ...
 'highJetTorqueNm',nan(3,1),'highJetBaseTorqueNm',nan(3,1), ...
 'highJetAnalyticResidualNm',NaN,'highJetContributionMatrix',nan(3,3), ...
 'highJetDecompositionPassed',false, ...
 'exactReplayPassed',false,'maxReplayAbsoluteErrorNm',NaN, ...
 'maxExactReplayTimeMs',NaN, ...
 'times_s',[],'rawLoggedNm',[],'rawExactReplayedNm',[], ...
 'replayMs',[],'perTickAbsErrorNm',[], ...
 'notOnlineForecast',true);
raw=reshape([rec.hlcRaw],4,[]);
applied=reshape([rec.hlcApplied],4,[]);
clip=any(abs(raw(2:4,:)-applied(2:4,:))>1e-8,1);
active=find(([rec.time]>=5) & clip,1,'first');
if isempty(active)
   % For the unsaturated control, still verify the exact algebra at one
   % representative Flight tick (never run 160 original HLC calls).
   active=find([rec.time]>=10,1,'first');
   if isempty(active),return;end
else
   a.hasTorqueClip=true;
   a.firstTorqueClipTime_s=rec(active).time;
   a.firstTorqueClipAxis=abs(raw(2:4,active)-applied(2:4,active))>1e-8;
   a.firstRawTorqueNm=raw(2:4,active);
   a.firstAppliedTorqueNm=applied(2:4,active);
   J=rec(active).param(6:8);J=J(:);
   a.firstAngularAccelerationDeficitRadS2= ...
      (applied(2:4,active)-raw(2:4,active))./J;
end
idx=unique(max(1,active-2):min(numel(rec),active+1));
N=numel(idx);a.times_s=reshape([rec(idx).time],1,[]);
a.rawLoggedNm=raw(2:4,idx);
a.rawExactReplayedNm=nan(3,N);a.replayMs=nan(1,N);
a.perTickAbsErrorNm=nan(1,N);
hlc=HLC_SUSPENDED_LOAD([],[]);
for j=1:N
    sw=tic;
    exact=ra_v19_replay_original_hlc_raw(hlc,rec(idx(j)),g,[]);
    a.replayMs(j)=1000*toc(sw);
    a.rawExactReplayedNm(:,j)=exact(2:4);
    a.perTickAbsErrorNm(j)=max(abs(exact(2:4)-raw(2:4,idx(j))));
end
a.maxReplayAbsoluteErrorNm=max(a.perTickAbsErrorNm);
a.maxExactReplayTimeMs=max(a.replayMs);
a.exactReplayPassed=isfinite(a.maxReplayAbsoluteErrorNm) && ...
   a.maxReplayAbsoluteErrorNm<=1e-6;
if a.hasTorqueClip
    % With SAME current EKF x, compare original current reference and the
    % available previous reference jet. This is diagnostic only: replacing
    % a reference jet does not claim a valid alternative future Flight.
    current=ra_v19_replay_original_hlc_raw(hlc,rec(active),g,[]);
    prior=ra_v19_replay_original_hlc_raw(hlc,rec(active),g,rec(max(1,active-2)).reference);
    a.counterfactualCurrentJetTorqueNm=current(2:4);
    a.counterfactualPreviousJetTorqueNm=prior(2:4);
    % V21 OFFLINE ONLY: exact affine dependence of ORIGINAL V2_alpha2
    % on reference derivatives d6Xd1,d6Xd2,d6Xd3. Source has
    % [d6Xd1-(pT1/pT3)d6Xd3; d6Xd2-(pT2/pT3)d6Xd3; 0].
    % beta2 is independent of d6Xd1..3, hence raw torque Jacobian is
    % beta2\[1 0 -pT1/pT3; 0 1 -pT2/pT3; 0 0 0].
    [matrix,rawBase]=ra_v21_exact_sixth_jet_audit(hlc,rec(active),g);
    p6Now=rec(active).reference(:);
    contribution=matrix*p6Now(25:27);
    residual=max(abs((current(2:4)-rawBase(2:4))-contribution));
    a.highJetTorqueNm=contribution;
    a.highJetBaseTorqueNm=rawBase(2:4);
    a.highJetAnalyticResidualNm=residual;
    a.highJetContributionMatrix=matrix;
    a.highJetDecompositionPassed=isfinite(residual) && residual<1e-6;
    a.exactReplayPassed=a.exactReplayPassed && a.highJetDecompositionPassed;
end
end

function [H,u0]=ra_v21_exact_sixth_jet_audit(hlc,r,g)
% OFFLINE: derive the exact 6th-jet torque Jacobian from original beta2.
% Does not evaluate future paths or run inside the forecast/timed pipeline.
xd=r.reference(:);assert(numel(xd)==28,'RA:V21Jet','Reference must have 28 entries');
p6=xd(25:27);xd(25:27)=0;
u0=ra_v19_replay_original_hlc_raw(hlc,r,g,xd);
base=r.param(:);
param=[base(1),base(6:9).',r.activeControlMass,base(21),0,0];
x=[r.estQcompact(:);r.estW(:);r.estLoad(:);r.estLoadVelocity(:); ...
   r.estCableDir(:);r.estCableOmega(:)];
assert(isequal(size(param),[1,9]) && isequal(size(x),[19,1]) && ...
       abs(x(16))>1e-6, 'RA:V21SixthJet','HLC geometry singular');
vf=hlc.Vfd_SuspendedLoadxyDst(g.dt,x,xd',g.F1);
beta2=hlc.Beta2_SuspendedLoadxyDst(x,xd',vf,param);
assert(isequal(size(beta2),[3,3]) && all(isfinite(beta2(:))), ...
       'RA:V21SixthJet','Invalid beta2');
rawJac=[1 0 -x(14)/x(16);0 1 -x(15)/x(16);0 0 0];
H=beta2\rawJac;
assert(all(isfinite(H(:))) && all(isfinite(p6)), ...
       'RA:V21SixthJet','Nonfinite analytic high-jet Jacobian');
end

function u=ra_v19_replay_original_hlc_raw(hlc,r,g,alternateReference)
% EXACT algebra and yaw wrapping from ORIGINAL HLC_SUSPENDED_LOAD.do.
% Uses only recorded CURRENT EKF state x and original reference jet xd.
% If alternateReference is provided, use it with SAME x for counterfactual.
if isempty(alternateReference),xd=r.reference(:);
else,xd=alternateReference(:);end
if numel(xd)<28,xd=[xd;zeros(28-numel(xd),1)];end
assert(numel(xd)==28 && numel(r.estQcompact)==4, ...
 'RA:V19HLCReplayInput','Unexpected 28-jet or EKF compact quaternion length');
% MATLAB agent.parameter.get() may be a ROW. Original HLC uses named
% parameter extraction; vectorize once before concatenating the 9x1 P.
baseParam=r.param(:);
assert(numel(baseParam)>=21 && isscalar(r.activeControlMass), ...
    'RA:V19HLCReplayParam','Expected original 21+ parameter vector and scalar mass');
% IMPORTANT: generated ORIGINAL HLC functions index P as in5(:,7),
% so P must be a 1x9 ROW as in HLC_SUSPENDED_LOAD.do.
% The saved global 21-parameter vector may be a 21x1 column or a row.
p=[baseParam(1),baseParam(6:9).',r.activeControlMass,baseParam(21),0,0];
x=[r.estQcompact(:);r.estW(:);r.estLoad(:);r.estLoadVelocity(:); ...
   r.estCableDir(:);r.estCableOmega(:)];
assert(isequal(size(p),[1,9]) && isequal(size(x),[19,1]) && ...
       isequal(size(xd),[28,1]), ...
       'RA:V19_2HLCInputShapes', ...
       'HLC requires P 1x9 ROW, x 19x1 COLUMN, xd 28x1 COLUMN');
yaw=mod(r.estQ(3)+pi,2*pi)-pi;
yawd=xd(4);
yawUnit=[cos(yaw);sin(yaw);0];
yawdUnit=[cos(yawd);sin(yawd);0];
cy=cross(yawdUnit,yawUnit);
deltaYaw3=sign(cy(3))*acos(yawdUnit'*yawUnit);
xd(4)=-deltaYaw3+yaw;
assert(isequal(size(g.F1),[1,2]) && isequal(size(g.F2),[1,6]) && ...
       isequal(size(g.F3),[1,6]) && isequal(size(g.F4),[1,2]), ...
       'RA:V19_2GainShapes','Original HLC gain dimensions are invalid');
vf=hlc.Vfd_SuspendedLoadxyDst(g.dt,x,xd',g.F1);
assert(isequal(size(vf),[1,6]),'RA:V19_2VfShape','Expected Vfd 1x6 ROW');
vs=hlc.Vs_SuspendedLoadxyDst(x,xd',vf,p,g.F2,g.F3,g.F4);
assert(isequal(size(vs),[1,3]),'RA:V19_2VsShape','Expected Vs 1x3 ROW');
uf=hlc.Uf_SuspendedLoadxyDst(x,xd',vf,p);
beta2=hlc.Beta2_SuspendedLoadxyDst(x,xd',vf,p);
vsAlpha=hlc.V2_alpha2_SuspendedLoadxyDst(x,xd',vf,vs',p);
assert(isequal(size(beta2),[3,3]) && isequal(size(vsAlpha),[3,1]), ...
       'RA:V19_2TorqueShapes','Expected Beta2 3x3 and v2_alpha2 3x1');
u=[uf(1);beta2\vsAlpha];
assert(numel(u)==4 && all(isfinite(u)),'RA:V19HLCReplayNonfinite', ...
 'Exact raw HLC control replay produced a nonfinite output');
end

function routeFiles=ra_write_test_open_route_sources(work)
% Test-only generators are transient. All three rely on ORIGINAL
% gen_ref_waypoint_smooth, which returns a 28x1 jet of order six and uses
% degree-13 smoothstep to join non-collinear open waypoints without stops.
% They do NOT modify permanent waypoint or HLC source files.
names={'ra_test_wp_straight_curve_c6','ra_test_wp_s_bend_c6','ra_test_wp_3d_turn_c6'};
points={ ...
    [0 0 3; 0 1.5 3;0 3.0 3;0 4.5 3;0.8 6 3;2.3 7.5 3;3.5 9 3;4.1 10.5 3;4.1 12 3;4.1 14 3], ...
    [0 0 3;0 1.5 3;0 3 3;0.8 4.5 3;1.9 6 3;1.1 7.5 3;0 9 3;-0.5 10.5 3;-0.5 12.5 3], ...
    [0 0 3;0 1.5 3;0 3 3;0.5 4.5 3.15;1.25 6 3.5;1.9 7.5 3.9;2 9 4.0;1.8 10.5 4.0;1.2 12 3.8]};
routeFiles=cell(1,numel(names));
for i=1:numel(names)
    f=[names{i} '.m'];routeFiles{i}=f;
    filePath=fullfile(work,f);fid=fopen(filePath,'w');
    assert(fid>=0,'RA:OpenRouteFile','Cannot write temporary open route generator');
    fprintf(fid,'function ref=%s(varargin)\n',names{i});
    fprintf(fid,'%% TEST ONLY - nonperiodic, preplanned C6 open route.\n');
    fprintf(fid,'p=inputParser; addParameter(p,''speed'',0.75,@(v)isnumeric(v)&&isscalar(v)&&isfinite(v)&&v>0); parse(p,varargin{:});\n');
    fprintf(fid,'pointsXYZ=%s;\n',mat2str(points{i},17));
    % The requested speed is NOT necessarily achieved. The physical
    % derivative-profile limits are identical across all speed sweeps.
    fprintf(fid, ['ref=gen_ref_waypoint_smooth(''pointsXYZ'',pointsXYZ,''speed'',p.Results.speed,' ...
        '''blendRadius'',0.55,''maxAccel'',0.8,''maxJerk'',1.2,''maxD6'',3.0);\n']);
    fprintf(fid,'ref.routeName=''%s'';\n',names{i});
    fprintf(fid,'assert(ref.order==6 && ~ref.isLoop);\n');
    fprintf(fid,'end\n');fclose(fid);
end
end

function d=ra_audit_route_c6(g,rec,cache,use,stride)
% OFFLINE reference properties. No true future from Flight is supplied to
% the forecast. Future oracle is score-only elsewhere in this script.
d=struct('isLoop',logical(g.isLoop),'requestedSpeed',NaN, ...
 'actualPeakSpeed',NaN,'pathLength',NaN,'duration',NaN, ...
 'bendCount',0,'forecastWindowsThroughBend',0, ...
 'transitionObservedInFlight',false,'maxRelativePathJetBoundaryJump',NaN, ...
 'malformedJetCount',0,'maxCachePositionError_m',NaN, ...
 'maxCacheVelocityError_mps',NaN,'maxCacheAccelerationError_mps2',NaN, ...
 'sampleStride',stride,'allWindowsIncludeFourSeconds',true, ...
 'maxFlightReferenceLoadSpeed_mps',NaN,'maxFlightEKFLoadSpeed_mps',NaN, ...
  'maxFlightPhaseReferenceLoadSpeed_mps',NaN,'maxFlightPhaseEKFLoadSpeed_mps',NaN);
if isfield(g,'speedPeak'),d.actualPeakSpeed=g.speedPeak;end
if isfield(g,'pathLength'),d.pathLength=g.pathLength;end
if isfield(g,'duration'),d.duration=g.duration;end
% Profile peak (before governor) is not necessarily the actual flown speed.
% Read reference+EKF speed from current/PAST Flight observations for
% diagnostics only, never as input to any future forecast.
refSpeeds=arrayfun(@(v)norm(v.reference(5:7)),rec);
estSpeeds=arrayfun(@(v)norm(v.estLoadVelocity(:)),rec);
d.maxFlightReferenceLoadSpeed_mps=max(refSpeeds);
d.maxFlightEKFLoadSpeed_mps=max(estSpeeds);
flightMask=arrayfun(@(x)strcmp(char(x.phase),'f'),rec);
if any(flightMask)
    d.maxFlightPhaseReferenceLoadSpeed_mps=max(refSpeeds(flightMask));
    d.maxFlightPhaseEKFLoadSpeed_mps=max(estSpeeds(flightMask));
end
% speed is passed by the scenario; caller sets actual requested separately.
if ~g.isLoop && isfield(g,'evaluatePath') && ...
    isfield(g,'cumulativeS') && isfield(g,'blendRadiusEffective')
    S=g.cumulativeS;rad=g.blendRadiusEffective;W=g.waypoints;
    seg=diff(W,1,2);directions=seg./vecnorm(seg,2,1);
    bendS=[];bendRad=[];jumps=[];
    for i=2:size(W,2)-1
        angleChange=norm(directions(:,i)-directions(:,i-1));
        if angleChange>1e-5 && rad(i)>0
            bendS(end+1)=S(i); %#ok<AGROW>
            bendRad(end+1)=rad(i); %#ok<AGROW>
            for b=[S(i)-rad(i),S(i)+rad(i)]
                epsS=min(1e-11,rad(i)*1e-9);
                dl=g.evaluatePath(b-epsS);dr=g.evaluatePath(b+epsS);
                if ~isequal(size(dl),[3,7]) || ~isequal(size(dr),[3,7]) || ...
                        any(~isfinite([dl(:);dr(:)]))
                    d.malformedJetCount=d.malformedJetCount+1;
                else
                    for k=1:7
                        jumps(end+1)=norm(dl(:,k)-dr(:,k))/ ...
                           max([1,norm(dl(:,k)),norm(dr(:,k))]); %#ok<AGROW>
                    end
                end
            end
        end
    end
    d.bendCount=numel(bendS);
    if ~isempty(jumps),d.maxRelativePathJetBoundaryJump=max(jumps);end
    physicalS=nan(1,numel(rec));
    for i=1:numel(rec)
        tmp=g.timeLaw(rec(i).tau);
        physicalS(i)=tmp(1);
    end
    nThrough=0;
    for k=1:numel(use)
        startS=physicalS(use(k));endS=physicalS(use(k)+160);
        for j=1:numel(bendS)
            if min(startS,endS)<bendS(j)+bendRad(j) && ...
                    max(startS,endS)>bendS(j)-bendRad(j)
                nThrough=nThrough+1;break;
            end
        end
    end
    d.forecastWindowsThroughBend=nThrough;
    for j=1:numel(bendS)
        if min(physicalS)<bendS(j) && max(physicalS)>bendS(j)
            d.transitionObservedInFlight=true;break;
        end
    end
end
% A C6 ORIGINAL generator and its PCHIP table are not the same thing:
% independent PCHIP of p, v, a is only an approximation. Quantify it.
if isfield(cache,'liveDynamic') && cache.liveDynamic
    tauMax=max([rec.tau]);
else
    tauMax=min(max([rec.tau]),cache.tauGrid(end));
end
testTau=linspace(0,tauMax,171);
if isfield(cache,'liveDynamic') && cache.liveDynamic
    [pCheck,vCheck,aCheck]=ra_v20_live_route_xyz(cache.generator,testTau);
end
eP=zeros(1,numel(testTau));eV=eP;eA=eP;
for k=1:numel(testTau)
    jet=g.evaluate(testTau(k));
    if ~isequal(size(jet),[28,1]) || any(~isfinite(jet))
        d.malformedJetCount=d.malformedJetCount+1;continue;
    end
    if isfield(cache,'liveDynamic') && cache.liveDynamic
        eP(k)=norm(pCheck(:,k)-jet(1:3));
        eV(k)=norm(vCheck(:,k)-jet(5:7));
        eA(k)=norm(aCheck(:,k)-jet(9:11));
    else
        eP(k)=norm(ppval(cache.ppP,testTau(k))-jet(1:3));
        eV(k)=norm(ppval(cache.ppDP,testTau(k))-jet(5:7));
        eA(k)=norm(ppval(cache.ppDDP,testTau(k))-jet(9:11));
    end
end
d.maxCachePositionError_m=max(eP);
d.maxCacheVelocityError_mps=max(eV);
d.maxCacheAccelerationError_mps2=max(eA);
end

function tr=runFlightGenerated(root,work,sc,index,te)
s=fileread(fullfile(root,'run_flight_SimSuspendedLoad_trj_obs_sns.m'));
s=strrep(strrep(s,sprintf('\r\n'),sprintf('\n')),sprintf('\r'),sprintf('\n'));
s=req(s,'clear all;','% AUDIT isolated function; do not clear workspace',1);
s=req(s,'close all hidden;','% AUDIT no close',1);
s=req(s,'clc;','% AUDIT no clc',1);
s=req(s,'script_dir = fileparts(mfilename(''fullpath''));', ...
    ['script_dir = ''' strrep(root,'''','''''') ''';'],1);
s=req(s,'userpath(''clear'');','% AUDIT do not alter userpath',1);
s=req(s,'logger.display_on = true;','logger.display_on = false;',1);
s=req(s,'te = 50;',sprintf('te = %.17g;',te),1);
s=req(s,'flightRouteGenerator = "gen_wp_straight";', ...
    sprintf('flightRouteGenerator = "%s";',sc.route),1);
s=req(s,'flightNominalSpeed = 1.5;', ...
    sprintf('flightNominalSpeed = %.17g;',sc.speed),1);
s=req(s,'sceneName = ''scene_cylinders_high_side'';', ...
    sprintf('sceneName = ''%s'';',sc.scene),1);
% This is the ONLY physical/estimator mass-setup variation:
% the ORIGINAL plant is first built with mL=0.14 kg in BOTH runs.
% The original next line initializes EKF/HLC to 0.01; replace that value
% only in the MATCHED INITIAL run. The online mass state is never clamped.
assert(sc.truePlantMassKg==0.14,'RA:TruePlantMass','Expected untouched original plant mass 0.14 kg');
assert(~isempty(strfind(s,'agent.parameter.set("loadmass",0.14);')), ...
   'RA:MissingPlantMass','Original 0.14 kg plant construction not found');
s=req(s,'agent.parameter.set("loadmass",0.01);', ...
    sprintf('agent.parameter.set("loadmass",%.17g);',sc.initialEstimatorHLCMassKg),1);
if isfield(sc,'initialOffset')
    off=sc.initialOffset(:);
    assert(numel(off)==3 && all(isfinite(off)),'Invalid test-only offset');
    s=req(s,'initial_state.p  = [0; 0; 0];', ...
       sprintf('initial_state.p  = [%.17g; %.17g; %.17g];',off(1),off(2),off(3)),1);
end
for x={'obstacleDrawSceneViews','obstacleDrawFlightViews','obstacleDrawFlightMovie'}
    s=req(s,[x{1} ' = true;'],[x{1} ' = false;'],1);
end
% Current EKF, ref, governor and actual HLC saturation, before plant step.
s=req(s,'%% Arming at t = 0 (one log entry)', ...
    sprintf('raCells = {};\n%% Arming at t = 0 (one log entry)'),1);
hook='do_prop_local(time, cha, cha, logger, env, agent, "controller",      1, true);';
s=req(s,hook,[hook sprintf('\nraCells{end+1} = ra_test_capture(agent,time);')],2); %#ok<AGROW>
post=strfind(s,'%% SimSuspendedLoad.m: postprocessing without app.UIAxes');
loc=strfind(s,'%% Local functions, moved to end as required for MATLAB scripts');
assert(numel(post)==1 && numel(loc)==1 && post<loc,'Original runner template changed');
endBody=sprintf(['tr=struct(); tr.records=[raCells{:}];\n' ...
 'tr.generator=agent.reference.waypoint.generator;\n' ...
 'tr.world=worldObstacles;\n' ...
 'tr.route=char(flightRouteGenerator);\n' ...
 'tr.scene=char(sceneName);\n' ...
 'tr.requestedSpeed=flightNominalSpeed;\n' ...
 'tr.trueMassKgDefinedByPlantSetup=0.14;\n' ...
 'tr.plantModelMassKg=agent.plant.param(20);\n' ...
 'tr.gains=struct(''F1'',agent.controller.hlc_suspended.param.F1,' ...
 '''F2'',agent.controller.hlc_suspended.param.F2,' ...
 '''F3'',agent.controller.hlc_suspended.param.F3,' ...
 '''F4'',agent.controller.hlc_suspended.param.F4,' ...
 '''dt'',agent.controller.hlc_suspended.param.dt);\n' ...
 'end\n\n']);
s=[s(1:post-1),endBody,s(loc:end)];
name=sprintf('RA_TMP_FLIGHT_%02d',index);
s=[sprintf('function tr=%s()\n',name),s,sprintf('\n%s\n',localCaptureSource())];
f=fullfile(work,[name '.m']);fid=fopen(f,'w');assert(fid>=0);fwrite(fid,s,'char');fclose(fid);
addpath(work,'-begin');rehash;
tr=feval(name);
tr.truePlantMassKg=sc.truePlantMassKg;
assert(abs(tr.plantModelMassKg-sc.truePlantMassKg)<1e-12, ...
 'RA:PlantMassDrift','Actual plant mass %.8g != prescribed ground truth %.8g', ...
 tr.plantModelMassKg,sc.truePlantMassKg);
tr.initialEstimatorSetupMassKg=sc.initialEstimatorHLCMassKg;
end

function textOut=localCaptureSource()
textOut=[ ...
 'function r=ra_test_capture(agent,time)' newline ...
 'e=agent.estimator.result.state; p=agent.plant.state; g=agent.reference.waypoint;' newline ...
 'ctrl=agent.controller.result;' newline ...
 'r=struct(); r.time=time.t; r.phase=agent.cha;' newline ...
 'r.estP=e.p(:); r.estV=e.v(:); r.estQ=e.getq(''euler'');r.estQcompact=e.getq(''compact'');r.estW=e.w(:);' newline ...
 'r.estLoad=e.pL(:);r.estLoadVelocity=e.vL(:);r.estCableDir=e.pT(:);r.estCableOmega=e.wL(:);' newline ...
 'r.estMass=e.mL;r.activeControlMass=agent.parameter.get("loadmass");r.param=agent.parameter.get();' newline ...
 'r.oracleDrone=p.p(:);r.oracleLoad=p.pL(:);' newline ...
 'r.reference=agent.reference.result.state.xd(:);' newline ...
 'r.tau=g.virtualTime;r.rateState=g.rateState(:);r.heldCommand=g.heldRateCommand;' newline ...
 'r.governorSettings=[g.slowdownStart,g.slowdownFull,g.minScale,g.filterTime];' newline ...
 'r.hlcRaw=ctrl.tmp(:);r.hlcApplied=ctrl.input(:);' newline ...
 'r.wasSaturated=any(abs(r.hlcRaw-r.hlcApplied)>1e-8);' newline ...
 'end' newline];
end

function a=ra_mass_converged_window_scores(rec,item,truthKg)
% Convergence criterion uses ONLY past/current EKF mass at forecast start.
% Future mass/state/actuator values are inspected AFTER each forecast for
% objective, offline eligibility; never leak these into a future predictor.
t=[rec.time]; m=[rec.estMass];
N=numel(item.windowStartTimes_s);
maskPast=false(N,1);maskFull=false(N,1);maskNoSat=false(N,1);
normalStart=false(N,1);
normal=arrayfun(@(x)all(isfinite([x.estP(:);x.estV(:);x.estLoad(:); ...
 x.estLoadVelocity(:);x.estCableDir(:);x.estMass])) && ...
 x.estMass>0 && norm(x.estLoad(:)-x.estP(:)-x.param(21)*x.estCableDir(:))<=0.01 && ...
 norm(x.estLoadVelocity(:))<=5,rec);
goodMass=isfinite(m) & (abs(m-truthKg)<=0.10*truthKg);
a=struct();a.toleranceRelative=0.10;a.pastDuration_s=1;
a.forecastDuration_s=4;
a.thresholdsAreDiagnosticNotSafetyCertified=true;
a.pastOnlyConvergenceGateIsCausal=true;
a.futureEligibilityIsScoringOnly=true;
for j=1:N
  ii=find(abs(t-item.windowStartTimes_s(j))<1e-7,1,'first');
  if isempty(ii) || ii<=40 || ii+160>numel(rec),continue;end
  if ~all(abs(diff(t(ii-40:ii))-.025)<1e-6),continue;end
  normalStart(j)=normal(ii);
  maskPast(j)=all(goodMass(ii-40:ii)) && normalStart(j);
  % Exclude scored windows containing future physical breakdown or mass
  % drift out of the near-true regime, exactly as requested, OFFLINE.
  maskFull(j)=maskPast(j) && all(goodMass(ii:ii+160)) && all(normal(ii:ii+160));
  maskNoSat(j)=~any([rec(ii:ii+160).wasSaturated]);
end
a.past1sConvergedStartCount=nnz(maskPast);
a.fullHorizonScorableWindowCount=nnz(maskFull);
a.fullHorizonScorableUnsaturatedWindowCount=nnz(maskFull&maskNoSat);
a.past1sConvergedStartMask=maskPast;
a.fullHorizonScorableMask=maskFull;
a.noSaturationFutureMask=maskNoSat;
a.A=ra_mass_method_subset(item.methodA,maskFull,maskNoSat);
a.B=ra_mass_method_subset(item.methodB,maskFull,maskNoSat);
end

function q=ra_mass_method_subset(s,eligible,nonSat)
q=struct('validCount',0,'noSatCount',0,'maxLoadErrorM',NaN, ...
  'maxAt1sM',NaN,'maxAt2sM',NaN,'maxAt4sM',NaN, ...
  'noSatMaxLoadErrorM',NaN,'medianRuntimeMs',NaN,'p95RuntimeMs',NaN);
if ~isstruct(s) || ~isfield(s,'errorLoad_m'),return;end
ok=eligible(:) & all(isfinite(s.errorLoad_m),2) & isfinite(s.timeMs(:));
q.validCount=nnz(ok);
if ~any(ok),return;end
q.maxLoadErrorM=ra_max_window_error(s.errorLoad_m,ok);
q.maxAt1sM=max(s.errorLoad_m(ok,41),[],'all');
q.maxAt2sM=max(s.errorLoad_m(ok,81),[],'all');
q.maxAt4sM=max(s.errorLoad_m(ok,161),[],'all');
q.medianRuntimeMs=median(s.timeMs(ok));
q.p95RuntimeMs=quantileSimple(s.timeMs(ok),.95);
okNoSat=ok & nonSat(:);
q.noSatCount=nnz(okNoSat);
if any(okNoSat)
 q.noSatMaxLoadErrorM=ra_max_window_error(s.errorLoad_m,okNoSat);
end
end

function v=ra_max_window_error(E,mask)
if ~any(mask),v=NaN;return;end
x=E(mask,:);v=max(x(:));
end

function d=ra_mass_trajectory_audit(rec,truthKg)
% DIAGNOSTIC / SCORING ONLY: mass history is NEVER used as a future oracle.
t=[rec.time]; m=[rec.estMass]; active=[rec.activeControlMass];
flightMask=arrayfun(@(x)strcmp(char(x.phase),'f'),rec);
assert(any(flightMask),'RA:MissingFlightPhase','No flight samples for mass trajectory audit');
tf=t(flightMask);mf=m(flightMask);uf=active(flightMask);
d=struct();
d.truePlantMassKg=truthKg;
d.firstFlightMassKg=mf(1);d.lastFlightMassKg=mf(end);
d.firstFlightControllerMassKg=uf(1);
d.flightMinMassKg=min(mf);d.flightMaxMassKg=max(mf);
d.flightMassRMSErrorKg=sqrt(mean((mf-truthKg).^2));
d.maxControllerEstimatorMassDifferenceKg=max(abs(active-m));
d.sampleTimes_s=[0 5 8 10 12 15 17 20 25 38];
d.massAtSampleTimes_kg=nan(size(d.sampleTimes_s));
for k=1:numel(d.sampleTimes_s)
  ix=find(abs(t-d.sampleTimes_s(k))<1e-8,1,'first');
  if ~isempty(ix),d.massAtSampleTimes_kg(k)=m(ix);end
end
% First FULL 1-second block close to truth. This is convergence evidence,
% not a statement that later mass values must stay close or be frozen.
d.firstSustained10PctTime_s=ra_first_sustained_mass(tf,mf,truthKg,0.10);
d.firstSustained5PctTime_s=ra_first_sustained_mass(tf,mf,truthKg,0.05);
d.flightBadMassCount=nnz(~isfinite(mf) | mf<=0);
endpointResidual=arrayfun(@(x)norm(x.estLoad(:)-x.estP(:)-x.param(21)*x.estCableDir(:)),rec);
flightResidual=endpointResidual(flightMask);
flightSpeed=arrayfun(@(x)norm(x.estLoadVelocity),rec);
flightSpeed=flightSpeed(flightMask);
d.flightCableEndpointOver1cmCount=nnz(~isfinite(flightResidual) | flightResidual>0.01);
d.flightLoadSpeedOver5mpsCount=nnz(~isfinite(flightSpeed) | flightSpeed>5);
d.flightIsModelConsistent=d.flightBadMassCount==0 && d.flightCableEndpointOver1cmCount==0 && d.flightLoadSpeedOver5mpsCount==0;
d.massAndFlightAuditIsOfflineOnly=true;
end

function ts=ra_first_sustained_mass(t,m,truth,tolerance)
ts=NaN;
steps=round(1/0.025)+1;
if numel(m)<steps,return,end
valid=isfinite(m) & abs(m-truth)<=tolerance*truth;
for k=1:numel(m)-steps+1
    if all(valid(k:k+steps-1))
        ts=t(k);return
    end
end
end

function p=ra_pair_mass_outcomes(matched,original)
p=struct();p.baseName=matched.scenario.baseName;
assert(strcmp(matched.scenario.baseName,original.scenario.baseName), ...
 'RA:MassPairMismatch','Paired base routes differ');
assert(matched.scenario.truePlantMassKg==original.scenario.truePlantMassKg && ...
       matched.scenario.rngSeed==original.scenario.rngSeed, ...
       'RA:MassPairMismatch','Plant mass and PRNG must match across paired runs');
p.massCaseA=matched.scenario.massCase;p.massCaseB=original.scenario.massCase;
p.initialMatchedKg=matched.scenario.initialEstimatorHLCMassKg;
p.initialOriginalKg=original.scenario.initialEstimatorHLCMassKg;
p.truePlantKg=matched.scenario.truePlantMassKg;
p.rngSeed=matched.scenario.rngSeed;
p.maxLoadErrorA_Matched_m=NaN;p.maxLoadErrorA_Original_m=NaN;
p.maxLoadErrorB_Matched_m=NaN;p.maxLoadErrorB_Original_m=NaN;
p.saturationTicksMatched=NaN;p.saturationTicksOriginal=NaN;
p.massAtEndMatched_kg=NaN;p.massAtEndOriginal_kg=NaN;
p.sustainedMassWithin10PctMatched_s=NaN;p.sustainedMassWithin10PctOriginal_s=NaN;
p.maxMassParameterDisagreementMatched_kg=NaN;
p.maxMassParameterDisagreementOriginal_kg=NaN;
p.maxTrueLoadTrajectoryDifference_m=NaN;
p.comparableConvergedWindowCount=0;
p.comparableConvergedUnsaturatedWindowCount=0;
p.comparableA_MaxErrMatched_m=NaN;
p.comparableA_MaxErrOriginal_m=NaN;
p.comparableB_MaxErrMatched_m=NaN;
p.comparableB_MaxErrOriginal_m=NaN;
p.comparableA_NoSatMaxErrMatched_m=NaN;
p.comparableA_NoSatMaxErrOriginal_m=NaN;
p.convergenceComparison='NOT_AVAILABLE';
p.pairClass='INCOMPLETE_MASS_COMPARISON';
if ~(isfield(matched,'massAudit') && isfield(original,'massAudit')),return,end
ma=matched.massAudit;mb=original.massAudit;
p.massAtEndMatched_kg=ma.lastFlightMassKg;
p.massAtEndOriginal_kg=mb.lastFlightMassKg;
p.sustainedMassWithin10PctMatched_s=ma.firstSustained10PctTime_s;
p.sustainedMassWithin10PctOriginal_s=mb.firstSustained10PctTime_s;
p.maxMassParameterDisagreementMatched_kg=ma.maxControllerEstimatorMassDifferenceKg;
p.maxMassParameterDisagreementOriginal_kg=mb.maxControllerEstimatorMassDifferenceKg;
if isfield(matched,'methodA') && isstruct(matched.methodA) && isfield(matched.methodA,'maxLoadErrorM')
  p.maxLoadErrorA_Matched_m=matched.methodA.maxLoadErrorM;
  p.maxLoadErrorB_Matched_m=matched.methodB.maxLoadErrorM;
  p.saturationTicksMatched=matched.hlcSaturationSamplesInWholeFlight;
end
if isfield(original,'methodA') && isstruct(original.methodA) && isfield(original.methodA,'maxLoadErrorM')
  p.maxLoadErrorA_Original_m=original.methodA.maxLoadErrorM;
  p.maxLoadErrorB_Original_m=original.methodB.maxLoadErrorM;
  p.saturationTicksOriginal=original.hlcSaturationSamplesInWholeFlight;
end
if matched.savedFlightTraceForAudit && original.savedFlightTraceForAudit
  a=matched.flightAuditRecords;b=original.flightAuditRecords;
  if numel(a)==numel(b) && max(abs([a.time]-[b.time]))<1e-8
    delta=sqrt(sum(([a.oracleLoad]-[b.oracleLoad]).^2,1));
    p.maxTrueLoadTrajectoryDifference_m=max(delta);
  end
end
if isfield(matched,'postConvergence') && isfield(original,'postConvergence') && ...
   isfield(matched,'methodA') && isstruct(matched.methodA) && ...
   isfield(original,'methodA') && isstruct(original.methodA) && ...
   isfield(matched.methodA,'errorLoad_m') && isfield(original.methodA,'errorLoad_m')
  ax=matched.postConvergence; bx=original.postConvergence;
  if isequal(size(matched.windowStartTimes_s),size(original.windowStartTimes_s)) && ...
       all(abs(matched.windowStartTimes_s(:)-original.windowStartTimes_s(:))<1e-7)
    cv=ax.fullHorizonScorableMask(:) & bx.fullHorizonScorableMask(:);
    bothNoSat=cv & ax.noSaturationFutureMask(:) & bx.noSaturationFutureMask(:);
    cv=cv & all(isfinite(matched.methodA.errorLoad_m),2) & all(isfinite(original.methodA.errorLoad_m),2) & ...
      all(isfinite(matched.methodB.errorLoad_m),2) & all(isfinite(original.methodB.errorLoad_m),2);
    bothNoSat=bothNoSat & cv;
    p.comparableConvergedWindowCount=nnz(cv);
    p.comparableConvergedUnsaturatedWindowCount=nnz(bothNoSat);
    if any(cv)
      p.comparableA_MaxErrMatched_m=ra_max_window_error(matched.methodA.errorLoad_m,cv);
      p.comparableA_MaxErrOriginal_m=ra_max_window_error(original.methodA.errorLoad_m,cv);
      p.comparableB_MaxErrMatched_m=ra_max_window_error(matched.methodB.errorLoad_m,cv);
      p.comparableB_MaxErrOriginal_m=ra_max_window_error(original.methodB.errorLoad_m,cv);
      p.convergenceComparison='AVAILABLE_AFTER_BOTH_EKF_MASSES_CONVERGED';
    else
      p.convergenceComparison='NO_JOINT_POSTCONVERGENCE_WINDOWS';
    end
    if any(bothNoSat)
      p.comparableA_NoSatMaxErrMatched_m=ra_max_window_error(matched.methodA.errorLoad_m,bothNoSat);
      p.comparableA_NoSatMaxErrOriginal_m=ra_max_window_error(original.methodA.errorLoad_m,bothNoSat);
    end
  else
    p.convergenceComparison='MISMATCHED_TIME_GRIDS';
  end
end
if ~ma.flightIsModelConsistent || ~mb.flightIsModelConsistent
    p.pairClass='MODEL_INVALID_AT_LEAST_ONE_FLIGHT';
elseif all(isfinite([p.maxLoadErrorA_Matched_m,p.maxLoadErrorA_Original_m,p.maxLoadErrorB_Matched_m,p.maxLoadErrorB_Original_m]))
    p.pairClass='BOTH_FLIGHTS_MODEL_CONSISTENT';
end
end

function d=ra_flight_domain_audit(rec)
% V11 OFFLINE diagnostics: recorded original Flight only. Never used by predictor.
% Preserve the exact applied input and all original model/estimator behavior.
n=numel(rec);
tt=[rec.time];
raw=reshape([rec.hlcRaw],4,n);
applied=reshape([rec.hlcApplied],4,n);
clip=abs(raw-applied)>1e-8;
d=struct();
d.saturationCountsByAxis=sum(clip,2).';
d.peakAbsoluteRawInput=max(abs(raw),[],2).';
d.peakAbsoluteAppliedInput=max(abs(applied),[],2).';
d.firstAnySaturation_s=ra_first_event_time(tt,any(clip,1));
d.firstThrustSaturation_s=ra_first_event_time(tt,clip(1,:));
d.firstTorqueSaturation_s=ra_first_event_time(tt,any(clip(2:4,:),1));
estv=reshape([rec.estLoadVelocity],3,n);
speeds=sqrt(sum(estv.^2,1));
[d.peakEKFLoadSpeed_mps,ix]=max(speeds);
d.peakEKFLoadSpeedTime_s=tt(ix);
masses=[rec.estMass];
[d.peakEstimatedMass_kg,ix]=max(masses);
d.peakEstimatedMassTime_s=tt(ix);
d.minEstimatedMass_kg=min(masses);
d.maxEstimatedMass_kg=max(masses);
d.badEstimatedMassTicks=nnz(~isfinite(masses) | masses<=0);
trueD=reshape([rec.oracleDrone],3,n);
trueL=reshape([rec.oracleLoad],3,n);
estD=reshape([rec.estP],3,n);
estL=reshape([rec.estLoad],3,n);
estq=reshape([rec.estCableDir],3,n);
% HLC reference jet lengths need not agree across takeoff/flight/landing.
% For physical tracking distance only the three position entries are needed.
% Concatenating the entire reference vectors across all phases can throw
% MATLAB:horzcat when different control phases expose different jet lengths.
refP=nan(3,n);
refLengths=zeros(1,n);
for k=1:n
    jet=rec(k).reference(:);
    refLengths(k)=numel(jet);
    if numel(jet)<3 || any(~isfinite(jet(1:3)))
        error('RA:FlightAuditReferencePosition', ...
            'Missing/nonfinite three-position reference at t=%.6g s, phase=%s, jetLength=%d', ...
            tt(k),char(rec(k).phase),numel(jet));
    end
    refP(:,k)=jet(1:3);
end
d.referenceJetLengths=unique(refLengths);
d.referenceNon28Ticks=nnz(refLengths~=28);
flightMask=arrayfun(@(x)strcmp(char(x.phase),'f'),rec);
d.flightReferenceNon28Ticks=nnz(refLengths(flightMask)~=28);
len=arrayfun(@(x)x.param(21),rec);
d.maxPhysicalCableLengthResidual_m=max(abs(sqrt(sum((trueL-trueD).^2,1))-len));
d.maxEKFCableEndpointResidual_m=max(sqrt(sum((estL-estD-estq.*len).^2,1)));
d.maxTruthLoadTrackingDistance_m=max(sqrt(sum((trueL-refP).^2,1)));
d.isDiagnosticOnly=true;
end

function t=ra_first_event_time(times,flags)
idx=find(flags,1,'first');
if isempty(idx),t=NaN;else,t=times(idx);end
end

function item=scoreBoth(tr,~,item,stride)
r=tr.records;tt=[r.time];n=numel(r);
assert(n>=400 && max(abs(diff(tt)-.025))<1e-7,'Expected uniform 25ms original Flight');
use=find(tt>=6 & tt<=tt(end)-4.0001 & tt<47);
use=use(1:stride:end);
assert(~isempty(use),'No eligible four-second windows');
item.forecastSamples=numel(use);
cache=struct('liveDynamic',true,'generator',tr.generator); % V20: NO future route samples stored
K=ra_test_pv_cache_prepare_v7(tr.gains,-1:.025:0,r(use(1)).governorSettings(4),.1);
Bpar=ra_hlc_four_pole_prepare(tr.gains,K);
item.zohVsContinuous=ra_hlc_sample_hold_audit(tr.gains);
assert(isfield(K,'WfreePV') && isfield(K,'WfreePVA') && ...
       isfield(K,'BfixedPV') && isfield(K,'BfixedPVA') && ...
       isfield(K,'Bfull') && K.initialPVExactlyAnchoredToCurrentEKF, ...
       'RA:InternalCacheVersion','Test-local V8 initial-EKF cache contract is invalid');
M=numel(use);N=161;
A=newRecord(M,N);B=newRecord(M,N);
geometryCfg=wp_system_geometry_config('B');
geometryCfg.checkGround=false; % V15 nominal obstacle screen does not certify ground
sensorCfg=ra_sim_sensor_config(20.0);
for m=1:M
    ii=use(m); cur=r(ii);
    % In a real control tick, use agent.reference.waypoint.generator HERE.
    % The present Flight replay has one generator per run (no online switch).
    routeContextWatch=tic;
    cache.generator=ra_v21_current_route_snapshot(tr.generator);
    currentRouteSnapshotMs=1000*toc(routeContextWatch);
    hi=ii-40:ii;
    H=struct('pastTimes',K.pastRelativeTimes,'pastLoad',[r(hi).estLoad], ...
     'pastVelocity',[r(hi).estLoadVelocity],'pastRefLoad',zeros(3,41), ...
     'pastRefVelocity',zeros(3,41),'tauNow',cur.tau,'rateNow',cur.rateState, ...
     'heldCommandNow',cur.heldCommand,'estimatedML',cur.estMass, ...
     'cableLength',cur.param(21),'gravity',cur.param(9), ...
     'slowdownStart',cur.governorSettings(1),'slowdownFull',cur.governorSettings(2), ...
     'minScale',cur.governorSettings(3),'filterTime',cur.governorSettings(4), ...
     'currentEKFDronePosition',cur.estP,'currentEKFCableDirection',cur.estCableDir);
    historyRef=[r(hi).reference];H.pastRefLoad=historyRef(1:3,:);H.pastRefVelocity=historyRef(5:7,:);H.pastRefAcceleration=historyRef(9:11,:);
    % Both forecasts start at EXACTLY the same current EKF state. No oracle.
    % A history-index mismatch is captured explicitly, rather than silently
    % blaming the modal dynamics for a t=0 discrepancy.
    A.inputHistoryMismatchM(m)=norm(H.pastLoad(:,end)-cur.estLoad(:));
    B.inputHistoryMismatchM(m)=A.inputHistoryMismatchM(m);
    % CURRENT observation only, computed once per trigger, shared by A and B.
    % This is the provided simulated sensor, not future obstacle truth.
    currentObservation=struct('parts',[]); currentSensorMs=NaN;
    try
        sensorWatch=tic;
        worldNow=obs_world_at_time(tr.world,cur.time);
        [currentObservation,~]=ra_sim_sensor_detect( ...
            worldNow,cur.oracleDrone(:),sensorCfg);
        currentSensorMs=1000*toc(sensorWatch);
    catch ME
        % The predictor is still evaluated: an observation failure must never
        % be silently interpreted as an obstacle-free scene.
        item.observationAuditErrors{m}=getReport(ME,'extended','hyperlinks','off');
    end
    if m==1
        firstInputH=H;firstInputCache=cache; % measured independently AFTER normal forecasts
    end
    A=oneForecast(A,m,@()ra_test_modal_anchored_v8(H,K,cache),r,ii,tr.world, ...
        currentObservation,geometryCfg,currentSensorMs,currentRouteSnapshotMs);
    B=oneForecast(B,m,@()ra_hlc_four_pole_forecast(H,Bpar,K.governor,cache),r,ii,tr.world, ...
        currentObservation,geometryCfg,currentSensorMs,currentRouteSnapshotMs);
    A.startTimes(m)=cur.time;B.startTimes(m)=cur.time;
    A.pastSaturationCount(m)=nnz([r(hi).wasSaturated]);
    B.pastSaturationCount(m)=A.pastSaturationCount(m);
    A.truthFutureSaturationCount(m)=nnz([r(ii:min(n,ii+160)).wasSaturated]);
    B.truthFutureSaturationCount(m)=A.truthFutureSaturationCount(m);
    % OFFLINE oracle label, AFTER both forecasts have completed.
    futureRec=r(ii:min(n,ii+160));
    rawFuture=reshape([futureRec.hlcRaw],4,[]);
    appliedFuture=reshape([futureRec.hlcApplied],4,[]);
    torqueClipped=any(abs(rawFuture(2:4,:)-appliedFuture(2:4,:))>1e-8,1);
    A.truthFutureTorqueSaturationCount(m)=nnz(torqueClipped);
    B.truthFutureTorqueSaturationCount(m)=A.truthFutureTorqueSaturationCount(m);
    idxClip=find(torqueClipped,1,'first');
    if ~isempty(idxClip)
        A.truthFirstFutureTorqueSaturationOffset_s(m)=(idxClip-1)*.025;
        B.truthFirstFutureTorqueSaturationOffset_s(m)=A.truthFirstFutureTorqueSaturationOffset_s(m);
    end
end
% OFFLINE same-input timing diagnostic: isolate MATLAB/JIT/OS spikes
% without spending time on extra calls inside any measured control tick.
item.v21SameInputRepeat=struct('status','NOT_RUN');
try
    ta=nan(1,5);tb=nan(1,5);
    for j=1:5
        watch=tic;
        checkA=ra_test_modal_anchored_v8(firstInputH,K,firstInputCache);
        ta(j)=1000*toc(watch);
        watch=tic;
        checkB=ra_hlc_four_pole_forecast(firstInputH,Bpar,K.governor,firstInputCache);
        tb(j)=1000*toc(watch);
        origA=A.predLoad(:,:,1);origB=B.predLoad(:,:,1);
        assert(max(abs(checkA.pLoad(:)-origA(:)))<1e-9, ...
            'RA:V21ReplayA','Repeated A forecast not deterministic');
        assert(max(abs(checkB.pLoad(:)-origB(:)))<1e-9, ...
            'RA:V21ReplayB','Repeated B forecast not deterministic');
    end
    item.v21SameInputRepeat=struct('status','PASS_REPEAT_DETERMINISTIC', ...
       'firstAOriginalMs',A.timeMs(1),'firstBOriginalMs',B.timeMs(1), ...
       'repeatA_ms',ta,'repeatB_ms',tb,'notIncludedInOnlineTiming',true);
catch MEtiming
    item.v21SameInputRepeat=struct('status','FAILED_REPEAT_DIAGNOSTIC', ...
       'error',getReport(MEtiming,'extended','hyperlinks','off'), ...
       'notIncludedInOnlineTiming',true);
end
item.methodA=reduceMethod(A);item.methodB=reduceMethod(B);
% ONLY three representative windows per scenario are checked against the
% currently observed shapes. This expensive OFFLINE audit is excluded from
% the measured A/B predictor time. No future dynamic obstacle truth enters.
try
    item.nominalIntervalAudit=ra_nominal_interval_scenario_audit(r,use,A,B,tr.world);
    item.nominalIntervalAuditStatus='PASS';
catch ME
    item.nominalIntervalAuditStatus='FAILED';
    item.nominalIntervalAuditError=getReport(ME,'extended','hyperlinks','off');
end
try
    item.directKnownRouteAudit=ra_direct_known_future_route_audit(tr.generator,r,use,A,B);
    item.directKnownRouteAuditStatus='PASS';
catch ME
    item.directKnownRouteAuditStatus='FAILED';
    item.directKnownRouteAuditError=getReport(ME,'extended','hyperlinks','off');
end
item.methodBModel=struct('orderXY',4,'orderZ',2, ...
 'polesX',Bpar.polesX,'polesY',Bpar.polesY, ...
 'poleSelection','two slowest stable conjugate pairs of ORIGINAL CONTINUOUS A6-B6*K', ...
 'gainSource','original HLC F1/F2/F3', ...
 'initialPVAndCableFromEKF',true, ...
 'initialJerkFromPastPV',true, ...
 'notExactUnSaturated6thOrderModel',true, ...
 'notExactZOH6thOrderModel',true);
item.windowStartTimes_s=tt(use);
item.routeAudit=ra_audit_route_c6(tr.generator,tr.records,cache,use,stride);
item.routeAudit.requestedSpeed=tr.requestedSpeed;
% This OFFLINE-only diagnostic must never discard completed A/B
% forecast results. A diagnostic failure is nevertheless a PARTIAL failure.
try
    item.flightDomainAudit=ra_flight_domain_audit(tr.records);
    item.flightDomainAuditStatus='PASS';
catch auditME
    item.flightDomainAuditStatus='FAILED';
    item.flightDomainAuditError=getReport(auditME,'extended','hyperlinks','off');
end
item.routeAudit.isKnownPreplannedRoute=true;
item.routeAudit.isOnlineRouteReplanningTest=false;
item.hlcSaturationSamplesInWholeFlight=nnz([r.wasSaturated]);
item.ekfPlantLoadDifferenceM=arrayfun(@(t)norm(t.estLoad-t.oracleLoad),r);
item.ekfPlantDroneDifferenceM=arrayfun(@(t)norm(t.estP-t.oracleDrone),r);
item.savedFlightTraceForAudit=true;item.flightAuditRecords=r;
item.dynamicWorldIsOracleOnlyInScoring=true;
item.intermediateGeometryUsesBoundingSphereOnly=true;
item.note='Collision proxy is outer-sphere sampled, NOT C/B/D certified geometry.';
end

function d=ra_direct_known_future_route_audit(generator,rec,use,A,B)
% Evaluate a future reference defined by the available C6 route and the
% PREDICTED governor's future tau and tau derivatives. This is NOT future
% plant truth, and does not use the Flight's future governor measurements.
% Only three window audits per scenario; off the 25ms forecast timer.
valid=[];
for k=1:numel(use)
    if all(isfinite(A.predTauDer(:,:,k)),'all') && ...
       all(isfinite(B.predTauDer(:,:,k)),'all') && ...
       all(isfinite(A.predRefLoad(:,:,k)),'all') && ...
       all(isfinite(B.predRefLoad(:,:,k)),'all')
        valid(end+1)=k; %#ok<AGROW>
    end
end
pick=[];
if ~isempty(valid),pick=unique(valid([1 ceil(numel(valid)/2) numel(valid)]),'stable');end
d=struct('A',struct('maxPositionDifferenceM',NaN,'medianMs',NaN,'timesMs',[]), ...
         'B',struct('maxPositionDifferenceM',NaN,'medianMs',NaN,'timesMs',[]), ...
         'isOffTick',true,'usesOnlyKnownRouteAndPredictedGovernor',true);
for method={'A','B'}
    name=method{1};
    if strcmp(name,'A'),R=A;else,R=B;end
    diffs=[];tt=[];
    for l=1:numel(pick)
        k=pick(l);watch=tic;
        for j=1:161
            jet=generator.evaluateWarped(R.predTau(k,j),R.predTauDer(:,j,k));
            assert(numel(jet)==28 && all(isfinite(jet)), ...
                'RA:DirectKnownRoute','Original generator returned invalid future jet');
            rp=jet(1:3); % Original original route, not load height-adjusted
            rp(3)=rp(3)-rec(use(k)).param(21); % Current source cable length
            diffs(end+1)=norm(R.predRefLoad(:,j,k)-rp); %#ok<AGROW>
        end
        tt(end+1)=1000*toc(watch); %#ok<AGROW>
    end
    if ~isempty(diffs)
        d.(name).maxPositionDifferenceM=max(diffs);
        d.(name).medianMs=median(tt);
        d.(name).timesMs=tt;
    end
end
end

function audit=ra_nominal_interval_scenario_audit(rec,use,A,B,world)
% V12. Diagnostics ONLY. Select first/middle/last valid common windows.
% Use detector with current plant position as in the provided simulated
% sensor. Do not pass the future dynamic world to the predictor or gate.
% MATLAB compatibility: avoid implicit squeeze differences with a loop.
valid=[];
for k=1:numel(use)
    if all(isfinite(A.predLoad(:,:,k)),'all') && ...
       all(isfinite(B.predLoad(:,:,k)),'all') && ...
       all(isfinite(A.predDrone(:,:,k)),'all') && ...
       all(isfinite(B.predDrone(:,:,k)),'all')
        valid(end+1)=k; %#ok<AGROW>
    end
end
pick=[];
if ~isempty(valid)
    pick=unique(valid([1,ceil(numel(valid)/2),numel(valid)]),'stable');
end
audit=struct('scoredStartTimes',nan(1,numel(pick)), ...
    'nonEmptyObservationWindows',0,'clearWithObservedPartsA',0, ...
    'clearWithObservedPartsB',0, ...
    'observedPartCounts',zeros(1,numel(pick)), ...
    'maxCurrentEKFEndpointResidualM',NaN, ...
    'maxCurrentEKFEndpointVelocityResidualMps',NaN, ...
    'maxInitialADroneMismatchM',NaN,'maxInitialBDroneMismatchM',NaN, ...
    'A',struct('clear',0,'unresolved',0,'medianMs',NaN,'timesMs',nan(1,numel(pick))), ...
    'B',struct('clear',0,'unresolved',0,'medianMs',NaN,'timesMs',nan(1,numel(pick))));
geo=wp_system_geometry_config('B');
geo.checkGround=false; % Ground is NOT certified by this diagnostic.
a=[];b=[];rr=[];rv=[];
for j=1:numel(pick)
    k=pick(j);idx=use(k);r=rec(idx);
    audit.scoredStartTimes(j)=r.time;
    % Same current simulated sensor semantics as source Flight. No future data.
    now=obs_world_at_time(world,r.time);
    [observed,~]=ra_sim_sensor_detect(now,r.oracleDrone(:),ra_sim_sensor_config(20.0));
    audit.observedPartCounts(j)=numel(observed.parts);
    audit.nonEmptyObservationWindows=audit.nonEmptyObservationWindows+double(numel(observed.parts)>0);
    rr(end+1)=norm(r.estLoad(:)-r.estP(:)-r.param(21)*r.estCableDir(:)); %#ok<AGROW>
    rv(end+1)=norm((r.estLoadVelocity(:)-r.estV(:))- ...
        r.param(21)*cross(r.estCableOmega(:),r.estCableDir(:))); %#ok<AGROW>
    a(end+1)=norm(A.predDrone(:,1,k)-r.estP(:)); %#ok<AGROW>
    b(end+1)=norm(B.predDrone(:,1,k)-r.estP(:)); %#ok<AGROW>
    watch=tic;
    [clearA,~]=ra_nominal_linear_interval_clear( ...
      A.predDrone(:,1:4:161,k),A.predLoad(:,1:4:161,k),observed,geo);
    audit.A.timesMs(j)=1000*toc(watch);
    watch=tic;
    [clearB,~]=ra_nominal_linear_interval_clear( ...
      B.predDrone(:,1:4:161,k),B.predLoad(:,1:4:161,k),observed,geo);
    audit.B.timesMs(j)=1000*toc(watch);
    audit.A.clear=audit.A.clear+double(clearA);
    audit.clearWithObservedPartsA=audit.clearWithObservedPartsA+double(clearA && numel(observed.parts)>0);
    audit.A.unresolved=audit.A.unresolved+double(~clearA);
    audit.B.clear=audit.B.clear+double(clearB);
    audit.clearWithObservedPartsB=audit.clearWithObservedPartsB+double(clearB && numel(observed.parts)>0);
    audit.B.unresolved=audit.B.unresolved+double(~clearB);
end
if ~isempty(pick)
    audit.A.medianMs=median(audit.A.timesMs);
    audit.B.medianMs=median(audit.B.timesMs);
    audit.maxCurrentEKFEndpointResidualM=max(rr);
    audit.maxCurrentEKFEndpointVelocityResidualMps=max(rv);
    audit.maxInitialADroneMismatchM=max(a);
    audit.maxInitialBDroneMismatchM=max(b);
end
audit.isOffTick=true;
audit.onlyFrozenObservedObstacleSnapshot=true;
audit.notActualContinuousSystemCertified=true;
audit.groundNotChecked=true;
audit.ellipsoidsCoveredByBoundingSpheres=true;
audit.cableUsesFourEndpointHullAABBSeparation=true;
audit.noAddedMarginBeyondExistingProvisionalGeometryCfg=true;
end

function [clear,firstUnresolved] = ra_nominal_linear_interval_clear(D,L,observed,geo)
% V16 exact-preserving broad-phase for the DEFINED piecewise-linear surrogate.
% Entire swept drone/load/cable lies inside the AABB of all D/L knots.
% If distance(obstacle center, WHOLE AABB) > obstacle bound radius + the
% LARGEST moving-body radius, this part cannot touch ANY of the intervals.
% Then interval-specific AABB skips proven-distant intervals. For all
% remaining candidates use the UNCHANGED V15 exact drone/load segment
% distance and conservative four-corner swept-cable hull AABB tests.
% This is NOT a bound on the true smooth HLC trajectory or future dynamic
% obstacle motion, and no unproven radius is added.
clear=false;firstUnresolved=[NaN NaN];
assert(isequal(size(D),size(L)) && size(D,1)==3 && size(D,2)>=2);
if any(~isfinite([D(:);L(:)])) || ~isfield(observed,'parts')
    return;
end
parts=observed.parts;
if isempty(parts)
    clear=true;return;
end
D0=D(:,1:end-1);D1=D(:,2:end);
L0=L(:,1:end-1);L1=L(:,2:end);
lo=min(min(D0,D1),min(L0,L1));
hi=max(max(D0,D1),max(L0,L1));
wholeLo=min(lo,[],2);wholeHi=max(hi,[],2);
maxMovingRadius=max([geo.droneRadius,geo.loadRadius,geo.cableRadius])+geo.safetyMargin;
active=false(1,numel(parts));
partRad=nan(1,numel(parts));
for j=1:numel(parts)
    part=parts(j);c=part.center(:);
    if strcmp(char(part.kind),'sphere')
        rad=part.params.radius;
    elseif strcmp(char(part.kind),'ellipsoid')
        rad=max(part.params.axes);
    else
        % Unknown shape must remain UNRESOLVED, never excluded.
        active(j)=true;continue;
    end
    partRad(j)=rad;
    if numel(c)~=3 || any(~isfinite([c;rad])) || rad<0
        active(j)=true;continue;
    end
    delta=max(max(wholeLo-c,0),c-wholeHi);
    % The global AABB CONTAINS the complete defined surrogate sweep.
    active(j)=dot(delta,delta)<=(rad+maxMovingRadius)^2;
end
js=find(active);
if isempty(js)
    clear=true;return;
end
for i=1:size(D,2)-1
    for jj=1:numel(js)
        j=js(jj);part=parts(j);c=part.center(:);
        rad=partRad(j);
        if ~isfinite(rad) || numel(c)~=3 || any(~isfinite(c)) || rad<0
            firstUnresolved=[i j];return;
        end
        % AABB separation is only a skip; original exact tests are retained
        % for every unexcluded candidate.
        delta=max(max(lo(:,i)-c,0),c-hi(:,i));
        if dot(delta,delta)>(rad+maxMovingRadius)^2
            continue;
        end
        rD=rad+geo.droneRadius+geo.safetyMargin;
        rL=rad+geo.loadRadius+geo.safetyMargin;
        rC=rad+geo.cableRadius+geo.safetyMargin;
        if ra_segment_dist2(c,D0(:,i),D1(:,i))<=rD^2 || ...
           ra_segment_dist2(c,L0(:,i),L1(:,i))<=rL^2
            firstUnresolved=[i j];return;
        end
        if dot(delta,delta)<=rC^2
            firstUnresolved=[i j];return;
        end
    end
end
clear=true;
end

function d2=ra_segment_dist2(c,a,b)
v=b-a;
vv=dot(v,v);
if vv<=0
    d2=dot(c-a,c-a);
else
    alpha=max(0,min(1,dot(c-a,v)/vv));
    v=c-(a+alpha*v);
    d2=dot(v,v);
end
end

function [clear,firstUnresolved] = ra_nominal_linear_interval_clear_v15_reference(D,L,observed,geo)
% Proven set containment for the DEFINED 41-node piecewise-linear surrogate:
% D(t) and L(t) linearly interpolate endpoints in each time interval.
% Every intermediate straight cable is contained in conv(D_i,D_i+1,L_i,L_i+1)
% and therefore in the AABB of those four endpoints. Sphere/ellipsoid
% parts are conservatively covered by their bounding balls (R=max(axes)).
% Early positive separation: endpoint swept segments vs obstacle balls,
% plus cable hull AABB vs obstacle ball. This only proves NOMINAL POLYLINE
% separate from the frozen observed outer balls, NOT true HLC trajectory.
clear=false;firstUnresolved=[NaN NaN];
assert(isequal(size(D),size(L)) && size(D,1)==3 && size(D,2)>=2);
if any(~isfinite([D(:);L(:)])) || ~isfield(observed,'parts')
    return;
end
parts=observed.parts;
for i=1:size(D,2)-1
    pts=[D(:,i),D(:,i+1),L(:,i),L(:,i+1)];
    lo=min(pts,[],2);hi=max(pts,[],2);
    for j=1:numel(parts)
        part=parts(j);c=part.center(:);
        if strcmp(char(part.kind),'sphere')
            rad=part.params.radius;
        elseif strcmp(char(part.kind),'ellipsoid')
            rad=max(part.params.axes);
        else
            firstUnresolved=[i j];return;
        end
        if any(~isfinite([c;rad])) || rad<0
            firstUnresolved=[i j];return;
        end
        % Separate drone and load swept balls using exact segment distance.
        rD=rad+geo.droneRadius+geo.safetyMargin;
        rL=rad+geo.loadRadius+geo.safetyMargin;
        rC=rad+geo.cableRadius+geo.safetyMargin;
        if ra_segment_dist2(c,D(:,i),D(:,i+1))<=rD^2 || ...
           ra_segment_dist2(c,L(:,i),L(:,i+1))<=rL^2
            firstUnresolved=[i j];return;
        end
        % Cable occupies a bilinear sweep subset of the four-point hull.
        % dist(c,AABB) is a lower bound on dist(c,hull).
        delta=max(max(lo-c,0),c-hi);
        if dot(delta,delta)<=rC^2
            firstUnresolved=[i j];return;
        end
    end
end
clear=true;
end

function r=ra_nominal_interval_self_test()
% Numerical sanity test; not a floating point formal proof.
g=struct('droneRadius',.25,'loadRadius',.16, ...
   'cableRadius',.015,'safetyMargin',.05);
w=struct();w.parts=struct('kind','sphere','center',[0;0;0], ...
   'params',struct('radius',.12));
% Endpoints of D alone are 2m apart from obstacle; interior hits it.
D=[-2,2;0,0;0,0];L=D+[0;0;-2];
[clearHit,idx]=ra_nominal_linear_interval_clear(D,L,w,g);
assert(~clearHit && isequal(idx,[1 1]), ...
    'RA:IntervalSelfTest','Between-node event not detected');
D=[-2,2;4,4;0,0];L=D+[0;0;-2];
[clearFar,~]=ra_nominal_linear_interval_clear(D,L,w,g);
assert(clearFar,'RA:IntervalSelfTest','Clearly separated polyline incorrectly unresolved');
% Offline differential test with original V15 algorithm; no on-tick cost.
% Restoring RNG is essential for identical paired flights.
prevRng=rng;restoreRng=onCleanup(@()rng(prevRng)); %#ok<NASGU>
rng(91473);
compared=0;clearCount=0;unresolvedCount=0;
for trial=1:120
    DD=cumsum(0.25*randn(3,41),2);
    LL=DD+repmat([0;0;-2],1,41)+0.03*randn(3,41);
    nObj=1+mod(trial,17);
    parts=repmat(struct('kind','sphere','center',[0;0;0], ...
        'params',struct('radius',.12,'axes',[.12,.12,.12])),1,nObj);
    for jj=1:nObj
        if mod(jj,3)==0
            parts(jj).kind='ellipsoid';
            parts(jj).params.axes=[.1,.2,.3]+.05*rand(1,3);
        else
            parts(jj).params.radius=.1+.25*rand;
        end
        if mod(jj,4)==0
            parts(jj).center=DD(:,min(41,1+mod(3*jj,40)))+.05*randn(3,1);
        else
            parts(jj).center=3*randn(3,1);
        end
    end
    obs=struct('parts',parts);
    [fastClear,fastIdx]=ra_nominal_linear_interval_clear(DD,LL,obs,g);
    [oldClear,oldIdx]=ra_nominal_linear_interval_clear_v15_reference(DD,LL,obs,g);
    assert(isequal(fastClear,oldClear) && isequaln(fastIdx,oldIdx), ...
        'RA:FastScreenDifferential','V16 filter changed V15 outcome at trial %d',trial);
    compared=compared+1;
    clearCount=clearCount+double(fastClear);
    unresolvedCount=unresolvedCount+double(~fastClear);
end
r=struct('status','PASS_INTERIOR_SEPARATION_AND_V15_DIFFERENTIAL', ...
  'isProofOnlyForDefinedPiecewiseLinearSurrogate',true, ...
  'notFloatingPointFormalCertification',true, ...
  'differentialTrials',compared,'differentialClear',clearCount, ...
  'differentialUnresolved',unresolvedCount);

end

function S=newRecord(M,N)
S=struct('startTimes',nan(M,1),'timeMs',nan(M,1), ...
 'errorLoad_m',nan(M,N),'errorDrone_m',nan(M,N), ...
 'referenceError_m',nan(M,N),'trackingError_m',nan(M,N), ...
 'decompositionResidual_m',nan(M,N),'governorTauError_s',nan(M,N), ...
 'sampledTruthProxyContact',false(M,1),'sampledPredProxyContact',false(M,1), ...
 'sampledPredProxyVsFutureWorldContact',false(M,1), ...
 'sampledPointwiseProxyFN',zeros(M,1),'sampledPointwiseProxyFP',zeros(M,1), ...
 'firstTruthContactOffset_s',nan(M,1),'firstPredContactOffset_s',nan(M,1), ...
 'pastSaturationCount',nan(M,1),'truthFutureSaturationCount',nan(M,1), ...
 'futureSaturationCountPredicted',nan(M,1), ...
 'truthFutureTorqueSaturationCount',nan(M,1), ...
 'truthFirstFutureTorqueSaturationOffset_s',nan(M,1), ...
 'currentObservationMs',nan(M,1), ...
 'onlineRouteSnapshotMs',nan(M,1), ...
 'onlineIntervalScreenMs',nan(M,1), ...
 'onlinePipelineMs',nan(M,1), ...
 'onlineNominalClear',false(M,1), ...
 'onlineScreenStatus',{cell(M,1)}, ...
 'observedPartCount',nan(M,1), ...
 'maxTautProjectionCorrection',nan(M,1),'errorMessages',{cell(M,1)}, ...
 'stageTimingsMs',{cell(M,1)}, ...
 'initialLoadAnchorMismatchM',nan(M,1),'initialDroneMismatchM',nan(M,1), ...
 'initialModelReportedAnchorMismatchM',nan(M,1),'inputHistoryMismatchM',nan(M,1), ...
 'predLoad',nan(3,N,M),'predDrone',nan(3,N,M), ...
 'predRefLoad',nan(3,N,M),'predTau',nan(M,N), ...
 'predTauDer',nan(7,N,M), ...
 'identifiedCoefficients',nan(3,2,M),'pastFitRMS',nan(3,M), ...
 'futureSaturationNotModeled',true);
end
function S=oneForecast(S,row,fn,rec,i,world,observed,geo,currentSensorMs,routeSnapshotMs)
tick=tic;
try
  f=fn();S.timeMs(row)=1000*toc(tick);
  assert(isequal(size(f.pLoad),[3,161]) && isequal(size(f.pDrone),[3,161]),'Forecast must have 161 knots');
  assert(all(isfinite([f.pLoad(:);f.pDrone(:)])),'Nonfinite forecast');
  S.initialLoadAnchorMismatchM(row)=norm(f.pLoad(:,1)-rec(i).estLoad(:));
  S.initialDroneMismatchM(row)=norm(f.pDrone(:,1)-rec(i).estP(:));
  if isfield(f,'initialAnchor') && isfield(f.initialAnchor,'EKFLoadMismatch_m')
      S.initialModelReportedAnchorMismatchM(row)=f.initialAnchor.EKFLoadMismatch_m;
  end
  assert(S.initialLoadAnchorMismatchM(row)<1e-6, ...
      'RA:InitialEKFAnchor','Forecast load at t=0 does not equal current EKF (%.8g m). Inspect test-local model and original trace.', ...
      S.initialLoadAnchorMismatchM(row));
  S.predLoad(:,:,row)=f.pLoad;S.predDrone(:,:,row)=f.pDrone;
  % ALL-WINDOW nominal interval screen. Current observation and all
  % predicted positions are causal; no true future HLC/plant/world data.
  S.currentObservationMs(row)=currentSensorMs;
  S.onlineRouteSnapshotMs(row)=routeSnapshotMs;
  if isfield(observed,'parts') && isfinite(currentSensorMs)
      S.observedPartCount(row)=numel(observed.parts);
      try
          screenWatch=tic;
          % V18: exactly ONE online geometry pass; NO V16 polyline screen,
          % NO V17 expensive recursion, NO poorly predictive torque surrogate.
          [nominalClear,unresolved]=ra_v18_fast_cubic_clear( ...
              f.pDrone,f.pLoad,observed,geo,64);
          S.onlineIntervalScreenMs(row)=1000*toc(screenWatch);
          % Time includes the forecast and only this geometric pass, plus
          % one SHARED simulated 20m observation. It excludes offline scoring.
          S.onlinePipelineMs(row)=1000*toc(tick)+currentSensorMs+routeSnapshotMs;
          if S.onlinePipelineMs(row)>2.0
              S.onlineNominalClear(row)=false;
              S.onlineScreenStatus{row}='UNKNOWN_OVER_2MS_TARGET_NOT_HARD_PREEMPTION';
          elseif any(abs(rec(i).hlcRaw(2:4)-rec(i).hlcApplied(2:4))>1e-8)
              % Actual CURRENT HLC torque saturation is directly measured.
              % It is NOT an oracle about future torque saturation.
              S.onlineNominalClear(row)=false;
              S.onlineScreenStatus{row}='UNKNOWN_CURRENT_TORQUE_SATURATION';
          elseif nominalClear
              S.onlineNominalClear(row)=true;
              S.onlineScreenStatus{row}='CLEAR_DEFINED_C1_CUBIC_VS_FROZEN_OBSERVATION_ONLY';
          else
              S.onlineNominalClear(row)=false;
              if isequal(unresolved(1),-2)
                  S.onlineScreenStatus{row}='UNKNOWN_TOO_MANY_RELEVANT_OBSTACLES';
              else
                  S.onlineScreenStatus{row}='UNKNOWN_CUBIC_SEPARATION_NOT_PROVEN';
              end
          end
      catch MEonline
          S.onlineScreenStatus{row}=getReport(MEonline,'extended','hyperlinks','off');
      end
  else
      S.onlineScreenStatus{row}='CURRENT_SENSOR_FAILED_NOT_CLEAR';
  end
  S.predRefLoad(:,:,row)=f.pLoad-f.predTrackingError;
  S.predTau(row,:)=reshape(f.virtualTime,1,[]);
  if isfield(f,'governorTauDer') && isequal(size(f.governorTauDer),[7,161])
      S.predTauDer(:,:,row)=f.governorTauDer;
  end
  % After the model has fully predicted, use saved future data ONLY for
  % OFFLINE diagnosis; do not pass it back to either prediction function.
  assert(isfield(f,'predTrackingError') && isequal(size(f.predTrackingError),[3,161]), ...
      'RA:SplitMissing','Predictor must expose its own tracking-error output');
  assert(isfield(f,'virtualTime') && numel(f.virtualTime)==161, ...
      'RA:GovMissing','Predictor must expose its own virtual-time output');
  for j=1:161
    tru=rec(i+j-1);
    S.errorLoad_m(row,j)=norm(f.pLoad(:,j)-tru.oracleLoad(:));
    S.errorDrone_m(row,j)=norm(f.pDrone(:,j)-tru.oracleDrone(:));
    % Same physical meaning as pastRefLoad: original saved HLC load target.
    referenceTruth=tru.reference(1:3);
    forecastRef=f.pLoad(:,j)-f.predTrackingError(:,j);
    dr=forecastRef-referenceTruth;
    de=f.predTrackingError(:,j)-(tru.oracleLoad(:)-referenceTruth);
    S.referenceError_m(row,j)=norm(dr);
    S.trackingError_m(row,j)=norm(de);
    S.decompositionResidual_m(row,j)=norm( ...
        (f.pLoad(:,j)-tru.oracleLoad(:))-(dr+de));
    S.governorTauError_s(row,j)=abs(f.virtualTime(j)-tru.tau);
  end
  assert(max(S.decompositionResidual_m(row,:))<=1e-9, ...
      'RA:SplitIdentity','Signed reference+tracking split failed the numeric identity');
  if isfield(f,'predictedSaturationCount')
    S.futureSaturationCountPredicted(row)=f.predictedSaturationCount;
  end
  if isfield(f,'identificationCoefficients')
    S.identifiedCoefficients(:,:,row)=f.identificationCoefficients;
    S.pastFitRMS(:,row)=f.pastAccelerationFitRMS(:);
  end
  if isfield(f,'stageTimesMs')
    S.stageTimingsMs{row}=f.stageTimesMs;
  elseif isfield(f,'stageMs')
    % Reporting only: map original modal timing keys to shared comparison names.
    t=f.stageMs;
    S.stageTimingsMs{row}=struct( ...
      'identification',t.modalFit, ...
      'modeAndGovernor',t.modalAndGovernor160, ...
      'routeInterpolation',t.routeInterpolation, ...
      'tautReconstruction',t.tautReconstruction);
  end
  if isfield(f,'maxTautProjectionCorrection')
    S.maxTautProjectionCorrection(row)=f.maxTautProjectionCorrection;
  end
  % Future obstacle pose is accessed ONLY after the forecast, for scoring.
  % This tests the CURRENT/PREDICTED TRAJECTORY only, not an online motion predictor.
  cTruth=false;cPred=false;cPredFrozen=false;
  frozen=obs_world_at_time(world,rec(i).time);
  for j=1:4:161
    t=rec(i+j-1).time;
    dynamic=obs_world_at_time(world,t);
    truthNow=proxyContacts(rec(i+j-1).oracleDrone(:),rec(i+j-1).oracleLoad(:),dynamic);
    predictedNow=proxyContacts(f.pDrone(:,j),f.pLoad(:,j),dynamic);
    if truthNow && isnan(S.firstTruthContactOffset_s(row)), S.firstTruthContactOffset_s(row)=(j-1)*.025; end
    if predictedNow && isnan(S.firstPredContactOffset_s(row)), S.firstPredContactOffset_s(row)=(j-1)*.025; end
    S.sampledPointwiseProxyFN(row)=S.sampledPointwiseProxyFN(row)+double(truthNow && ~predictedNow);
    S.sampledPointwiseProxyFP(row)=S.sampledPointwiseProxyFP(row)+double(~truthNow && predictedNow);
    cTruth=cTruth || truthNow;
    cPred=cPred || predictedNow;
    cPredFrozen=cPredFrozen || proxyContacts(f.pDrone(:,j),f.pLoad(:,j),frozen);
  end
  S.sampledTruthProxyContact(row)=cTruth;
  S.sampledPredProxyVsFutureWorldContact(row)=cPred;
  S.sampledPredProxyContact(row)=cPredFrozen;
catch ME
  S.timeMs(row)=1000*toc(tick);
  S.errorMessages{row}=getReport(ME,'extended','hyperlinks','off');
end
end

function yes=proxyContacts(pd,pl,world)
% DIAGNOSTIC ONLY: conservative enclosing sphere of every part against
% endpoints and full cable segment; no exact ellipsoid/capsule geometry.
yes=false;
parts=world.parts;
for j=1:numel(parts)
   q=parts(j); c=q.center(:);
   if strcmp(q.kind,'sphere')
      rr=q.params.radius;
   elseif strcmp(q.kind,'ellipsoid')
      rr=max(q.params.axes);
   else
      error('RA:UnsupportedPart','Unexpected obstacle kind: %s',q.kind);
   end
   dp=pl-pd;
   if dot(dp,dp)>0
       s=max(0,min(1,dot(c-pd,dp)/dot(dp,dp)));
       d=norm(c-(pd+s*dp));
   else
       d=norm(c-pd);
   end
   % Current provisional radii in wp_system_geometry_config are NOT used
   % as safety proof; these fixed numbers are DIAGNOSTIC proxy only.
   if d<=rr+0.30 || norm(c-pd)<=rr+0.30 || norm(c-pl)<=rr+0.21
       yes=true;return
   end
end
end

function B=ra_hlc_four_pole_prepare(gains,K)
% OFF-TICK ONLY. Ideal, UNSATURATED HLC horizontal 6-state companion has
% two slow conjugate pairs and two faster REAL poles in the tested sim.
% Select slow pairs by REAL PART. No arbitrary gain/pole tuning.
% This 4-state model is an approximate modal truncation, NOT exact for all
% initial states, sampled-data HLC nor the nonlinear/slack/saturated plant.
B=struct();
B.polesX=ra_two_slow_pairs(gains.F2);
B.polesY=ra_two_slow_pairs(gains.F3);
B.F1=reshape(gains.F1,1,[]);
B.Phi=cell(1,3); B.A2=cell(1,3);B.Bfull=cell(1,2);
B.WfreePVA=cell(1,2);B.BfixedPVA=cell(1,2);
B.fitCondition=nan(1,2);B.velocityWeight=K.velocityWeight;
for j=1:2
    if j==1,p=B.polesX;else,p=B.polesY;end
    c=real(poly(p)); % s^4+c(2)s^3+c(3)s^2+c(4)s+c(5)
    M=[0 1 0 0;0 0 1 0;0 0 0 1;-c(5) -c(4) -c(3) -c(2)];
    assert(all(real(eig(M))<0),'RA:FourPoleUnstable','Nonstable reduced roots');
    B.Phi{j}=expm(K.samplePeriod_s*M);
    B.A2{j}=M*M;
    t=K.pastRelativeTimes;m=numel(t);
    HP=zeros(m,4);HV=zeros(m,4);
    for a=1:m
       XX=expm(M*t(a));
       HP(a,:)=XX(1,:);HV(a,:)=XX(2,:);
    end
    Q=[HP;B.velocityWeight*HV];
    B.Bfull{j}=Q;
    B.BfixedPVA{j}=Q(:,1:3);
    U=Q(:,4); % only missing jerk is estimated from PAST PV
    B.fitCondition(j)=cond(Q); % full four-state PV-history condition
    if ~isfinite(norm(U)) || norm(U)<=eps*size(Q,1)*norm(Q,'fro')
        error('RA:FourPoleFitCondition','Jerk coefficient cannot be recovered reliably from the PV history');
    end
    B.WfreePVA{j}=U\eye(2*m);
end
B.Phi{3}=K.Phi{3};B.A2{3}=K.A2{3};
end

function p=ra_two_slow_pairs(gains)
% Choose two complex conjugate pairs with largest REAL PART; preserves
% the slower decaying oscillatory components of the original HLC.
A=diag(ones(5,1),1);A(6,:)=-reshape(gains,1,[]);
ev=eig(A);
assert(all(isfinite(ev)) && all(real(ev)<0),'RA:OriginalModalUnstable', ...
    'Ideal original HLC mode must be stable');
positiveImag=ev(imag(ev)>1e-7);
assert(numel(positiveImag)>=2,'RA:NoTwoComplexPairs', ...
    'Cannot build this mode-preserving four-state reduction');
[~,ord]=sort(real(positiveImag),'descend');
positiveImag=positiveImag(ord(1:2));
for i=1:2
    assert(any(abs(ev-conj(positiveImag(i)))<1e-6),'RA:ConjugateNotFound', ...
       'Missing conjugate pole in original HLC sixth-order matrix');
end
p=[positiveImag(1),conj(positiveImag(1)), ...
   positiveImag(2),conj(positiveImag(2))];
end

function f=ra_hlc_four_pole_forecast(H,B,Cgov,routeCache)
% Ideal ORIGINAL HLC slow-pair reduced x/y model (4th order); original z 2nd.
% Current EKF position, velocity, cable direction anchor e,e',e''; the
% missing jerk state is least-squares fitted using PAST EKF PV only.
% No current/held constant physical input, no HLC calls, no future oracle.
watch=tic;phase=tic;
h=.025;N=160;
assert(all(isfinite([H.pastLoad(:);H.pastVelocity(:); ...
    H.pastRefLoad(:);H.pastRefVelocity(:);H.rateNow(:); ...
    H.currentEKFCableDirection(:)])) && H.estimatedML>0 && ...
    H.cableLength>0 && H.gravity>0,'RA:FourPoleEKF','Invalid current EKF/history');
assert(isfield(routeCache,'liveDynamic') && routeCache.liveDynamic, ...
    'RA:V20LiveRouteRequired','No pregenerated route cache is accepted');
[pref0,dp0,ddp0]=ra_v20_live_route_xyz(routeCache.generator,H.tauNow);
pref0=pref0(:);dp0=dp0(:);ddp0=ddp0(:);
pref0(3)=pref0(3)-H.cableLength;
rate0=Cgov.D*H.rateNow(:);
refVelocity0=dp0*rate0(1);
refAcc0=dp0*rate0(2)+ddp0*rate0(1)^2;
e0=H.pastLoad(:,end)-pref0;
ev0=H.pastVelocity(:,end)-refVelocity0;
ezdd=-B.F1*[e0(3);ev0(3)];
restZ=refAcc0(3)+ezdd+H.gravity;
q=H.currentEKFCableDirection(:);
qNorm=norm(q);
if ~isfinite(qNorm) || qNorm<=sqrt(eps)
    error('RA:FourPoleCable','Current EKF cable direction is invalid');
end
q=q/qNorm;
if abs(q(3))<=1e-3
    error('RA:FourPoleCable','Current EKF cable horizontal: cannot anchor load acceleration');
end
lambda=-restZ/q(3); % per-mass positive load tension, taut branch only
if ~isfinite(lambda) || lambda<=sqrt(eps)*H.gravity
    error('RA:FourPoleTension','Current estimated cable q disagrees with positive tension');
end
acc0=[-lambda*q(1:2);restZ-H.gravity];
eacc0=acc0-refAcc0;
states=cell(1,3);rmsFit=nan(1,2);
for j=1:2
    y=[(H.pastLoad(j,:)-H.pastRefLoad(j,:))'; ...
       B.velocityWeight*(H.pastVelocity(j,:)-H.pastRefVelocity(j,:))'];
    known=[e0(j);ev0(j);eacc0(j)];
    missingJerk=B.WfreePVA{j}*(y-B.BfixedPVA{j}*known);
    states{j}=[known;missingJerk];
    if ~all(isfinite(states{j}))
        error('RA:FourPoleFit','Nonfinite fitted jerk');
    end
    rmsFit(j)=norm(B.Bfull{j}*states{j}-y)/sqrt(numel(y));
end
states{3}=[e0(3);ev0(3)];
fitMs=1000*toc(phase);phase=tic;
E=zeros(3,N+1);E1=E;E2=E;
for j=1:3
    E(j,1)=states{j}(1);E1(j,1)=states{j}(2);
    E2(j,1)=B.A2{j}(1,:)*states{j};
end
z=H.rateNow(:);tau=H.tauNow;cmd=H.heldCommandNow;
tauHist=zeros(1,N+1);dr=zeros(7,N+1);
tauHist(1)=tau;dr(:,1)=[tau;Cgov.D*z];
for k=1:N
    for j=1:3
        states{j}=B.Phi{j}*states{j};
        E(j,k+1)=states{j}(1);E1(j,k+1)=states{j}(2);
        E2(j,k+1)=B.A2{j}(1,:)*states{j};
    end
    [z,tau,d]=wp_six_trigger_governor_fast_step(z,tau,cmd,Cgov);
    tauHist(k+1)=tau;dr(:,k+1)=d;
    ee=norm(E(:,k+1));
    if ee<=H.slowdownStart,u=0;
    elseif ee>=H.slowdownFull,u=1;
    else,u=(ee-H.slowdownStart)/(H.slowdownFull-H.slowdownStart);
    end
    u=u*u*(3-2*u);
    cmd=1-(1-H.minScale)*u;
end
modeGovMs=1000*toc(phase);
if any(diff(tauHist)<-1e-10)
    error('RA:FourPoleRoute','Predicted governor virtual time was nonmonotone');
end
phase=tic;
[pref,dref,ddref]=ra_v20_live_route_xyz(routeCache.generator,tauHist);
interpMs=1000*toc(phase);phase=tic;
pref(3,:)=pref(3,:)-H.cableLength;
refAcc=dref.*repmat(dr(3,:),3,1)+ddref.*repmat(dr(2,:).^2,3,1);
pL=pref+E;
aL=refAcc+E2;
rest=aL+repmat([0;0;H.gravity],1,N+1);
rr=vecnorm(rest,2,1);
if any(~isfinite(rr)) || min(rr)<=sqrt(eps)*H.gravity
    error('RA:FourPoleTaut','Future reconstructed taut cable is singular');
end
qPred=-rest./repmat(rr,3,1);
pD=pL-H.cableLength*qPred;
recMs=1000*toc(phase);
f=struct('pLoad',pL,'pDrone',pD,'predTrackingError',E,'virtualTime',tauHist, ...
    'timeOffsets',(0:N)*h,'governorTauDer',dr, ...
    'physicalSafetyCertified',false, ...
    'isEligibleForSafeClear',false,'futureSaturationNotModeled',true, ...
    'predictedSaturationCount',NaN,'estimatedModelMass',H.estimatedML, ...
    'initialEKFLoadMismatchM',norm(pL(:,1)-H.pastLoad(:,end)), ...
    'initialEKFDroneMismatchM',norm(pD(:,1)-H.currentEKFDronePosition(:)), ...
    'pastFitWeightedRMS',rmsFit,'fitCondition',B.fitCondition, ...
    'stageTimesMs',struct('identification',fitMs,'modeAndGovernor',modeGovMs, ...
       'routeInterpolation',interpMs,'tautReconstruction',recMs), ...
    'timeMs',1000*toc(watch));
end

function D=ra_hlc_sample_hold_audit(gains)
% OFF-TICK THEORY DIAGNOSTIC: controller's LQR gains were generated by lqrd
% with dt. Actual original HLC is nonlinear; the ZOH linear mode is not
% automatically the true nonlinear plant trajectory. Do NOT use in gate.
D=struct();dt=gains.dt;
for j=1:3
    if j==1
        A=[0 1;0 0];B=[0;1];K=reshape(gains.F1,1,[]); key='vertical';
    elseif j==2
        A=diag(ones(5,1),1);B=[zeros(5,1);1];K=reshape(gains.F2,1,[]);key='horizontalX';
    else
        A=diag(ones(5,1),1);B=[zeros(5,1);1];K=reshape(gains.F3,1,[]);key='horizontalY';
    end
    n=size(A,1);
    Z=expm([A B;zeros(1,n+1)]*dt);
    Ad=Z(1:n,1:n);Bd=Z(1:n,n+1);
    PhiZOH=Ad-Bd*K;
    PhiCont=expm((A-B*K)*dt);
    D.(key)=struct('normInf',norm(PhiCont-PhiZOH,inf), ...
        'zohEigen',eig(PhiZOH),'continuousEigen',eig(PhiCont), ...
        'zohMaxEigenModulus',max(abs(eig(PhiZOH))), ...
        'continuousMaxEigenModulus',max(abs(eig(PhiCont))));
end
D.maxMatrixDifferenceInf=max([D.vertical.normInf, ...
    D.horizontalX.normInf,D.horizontalY.normInf]);
D.maxPoleModulusDifference=max(abs([ ...
    D.vertical.zohMaxEigenModulus-D.vertical.continuousMaxEigenModulus, ...
    D.horizontalX.zohMaxEigenModulus-D.horizontalX.continuousMaxEigenModulus, ...
    D.horizontalY.zohMaxEigenModulus-D.horizontalY.continuousMaxEigenModulus]));
D.onlyAnAuditNotAnOnlinePredictor=true;
end

function v=maxFinite(x)
x=x(isfinite(x));
if isempty(x),v=NaN;else,v=max(x);end
end

function T=reduceMethod(S)
T=S;
valid=all(isfinite(S.errorLoad_m),2)&all(isfinite(S.errorDrone_m),2);
T.valid=nnz(valid);T.failed=numel(valid)-T.valid;
v=S.timeMs(valid);if isempty(v)
   T.medianMs=NaN;T.p95Ms=NaN;T.maxMs=NaN;
   T.maxLoadErrorM=NaN;T.maxDroneErrorM=NaN;
else
   T.medianMs=median(v);T.p95Ms=quantileSimple(v,.95);T.maxMs=max(v);
   a=S.errorLoad_m(valid,:);b=S.errorDrone_m(valid,:);
   T.maxLoadErrorM=max(a(:));T.maxDroneErrorM=max(b(:));
end
T.sampledProxyFalseNegativeFutureWorld=nnz(S.sampledTruthProxyContact(valid) & ~S.sampledPredProxyVsFutureWorldContact(valid));
T.pointwiseScoringOnlyProxyFalseNegativeSamples=sum(S.sampledPointwiseProxyFN(valid));
T.pointwiseScoringOnlyProxyFalsePositiveSamples=sum(S.sampledPointwiseProxyFP(valid));
T.maxFirstContactTimeDifference_s=maxFinite(abs(S.firstTruthContactOffset_s(valid)-S.firstPredContactOffset_s(valid)));
T.maxLoadErrorAt1s_m=maxFinite(S.errorLoad_m(valid,41));
T.maxLoadErrorAt2s_m=maxFinite(S.errorLoad_m(valid,81));
T.maxLoadErrorAt4s_m=maxFinite(S.errorLoad_m(valid,161));
T.maxDroneErrorAt4s_m=maxFinite(S.errorDrone_m(valid,161));
T.maxReferenceErrorM=maxFinite(S.referenceError_m(valid,:));
T.maxTrackingErrorM=maxFinite(S.trackingError_m(valid,:));
T.maxLoadErrorFutureSaturationM=maxFinite(S.errorLoad_m(valid & S.truthFutureSaturationCount>0,:));
T.maxLoadErrorNoFutureSaturationM=maxFinite(S.errorLoad_m(valid & S.truthFutureSaturationCount==0,:));
T.maxGovernorTauErrorS=maxFinite(S.governorTauError_s(valid,:));
T.maxDecompositionResidualM=maxFinite(S.decompositionResidual_m(valid,:));
T.refErrorAt1sM=maxFinite(S.referenceError_m(valid,41));
T.refErrorAt2sM=maxFinite(S.referenceError_m(valid,81));
T.refErrorAt4sM=maxFinite(S.referenceError_m(valid,161));
T.trackingErrorAt1sM=maxFinite(S.trackingError_m(valid,41));
T.trackingErrorAt2sM=maxFinite(S.trackingError_m(valid,81));
T.trackingErrorAt4sM=maxFinite(S.trackingError_m(valid,161));
T.sampledProxyFalsePositiveFutureWorld=nnz(~S.sampledTruthProxyContact(valid) & S.sampledPredProxyVsFutureWorldContact(valid));
T.trueFutureSaturationWindows=nnz(S.truthFutureSaturationCount(valid)>0);
T.pastSaturationWindows=nnz(S.pastSaturationCount(valid)>0);
T.predictedSaturationWindows=nnz(S.futureSaturationCountPredicted(valid)>0);
T.trueFutureTorqueSaturationWindows=nnz(S.truthFutureTorqueSaturationCount(valid)>0);
T.maxErrorFutureTorqueSaturationM=maxFinite(S.errorLoad_m(valid & S.truthFutureTorqueSaturationCount>0,:));
T.maxErrorWithoutFutureTorqueSaturationM=maxFinite(S.errorLoad_m(valid & S.truthFutureTorqueSaturationCount==0,:));
T.firstFutureTorqueSaturationOffsets_s=S.truthFirstFutureTorqueSaturationOffset_s(valid & S.truthFutureTorqueSaturationCount>0);
T.pipeline=ra_v15_pipeline_stats(S,valid);
T.runtimeExcludesOriginalEKFAndFlight=true;
T.runtimeIncludesSelectedPredictorOnly=true;
T.allCollisionProxyScoresAreSampledAndNotCertified=true;
T.positionErrorsComparedToSavedPlantAfterPrediction=true;
T.sampledTruthProxyContactCount=nnz(S.sampledTruthProxyContact(valid));
T.sampledPredictionProxyContactCount=nnz(S.sampledPredProxyVsFutureWorldContact(valid));
missWindow=valid & S.sampledTruthProxyContact & ~S.sampledPredProxyVsFutureWorldContact;
T.sampledProxyMissStartTimes_s=S.startTimes(missWindow);
T.stageMedianMs=[];
if isfield(S,'stageTimingsMs') && any(valid)
    found=find(valid & ~cellfun(@isempty,S.stageTimingsMs));
    if ~isempty(found)
        names={'identification','modeAndGovernor','routeInterpolation','tautReconstruction'};
        med=struct();
        for j=1:numel(names)
            v=zeros(numel(found),1);
            for k=1:numel(found)
                v(k)=S.stageTimingsMs{found(k)}.(names{j});
            end
            med.(names{j})=median(v);
        end
        T.stageMedianMs=med;
    end
end
end
function p=ra_v15_pipeline_stats(S,valid)
% Simulated on-tick-like stage sum; BOTH methods would NOT run together online.
% No Flight/plant/EKF, no CBF optimization, and no physical sensor latency.
p=struct('validSamples',0,'failedObservationOrScreen',0, ...
 'medianMs',NaN,'p95Ms',NaN,'maxMs',NaN, ...
 'warmMedianMs',NaN,'warmP95Ms',NaN,'warmMaxMs',NaN, ...
 'medianCurrentSimulatedSensorMs',NaN, ...
 'medianCurrentRouteSnapshotMs',NaN, ...
 'medianNominalIntervalMs',NaN, ...
 'clearDefinedCubicWithin2msCount',0,'unresolvedOrOverBudgetCubicCount',0, ...
 'maxObservedParts',NaN,'includesEKFControllerAndPlant',false, ...
 'includesCBF',false,'includesPhysicalSensorLatency',false, ...
 'coversAllSelectedForecastWindows',true);
valid=valid(:);
ok=valid & isfinite(S.onlinePipelineMs(:)) & ...
   isfinite(S.onlineIntervalScreenMs(:)) & isfinite(S.currentObservationMs(:));
p.validSamples=nnz(ok);
p.failedObservationOrScreen=nnz(valid)-p.validSamples;
if ~any(ok),return;end
v=S.onlinePipelineMs(ok);
p.medianMs=median(v);p.p95Ms=quantileSimple(v,.95);p.maxMs=max(v);
p.medianCurrentSimulatedSensorMs=median(S.currentObservationMs(ok));
p.medianCurrentRouteSnapshotMs=median(S.onlineRouteSnapshotMs(ok));
p.medianNominalIntervalMs=median(S.onlineIntervalScreenMs(ok));
p.clearDefinedCubicWithin2msCount=nnz(S.onlineNominalClear(ok));
p.unresolvedOrOverBudgetCubicCount=nnz(~S.onlineNominalClear(ok));
p.maxObservedParts=max(S.observedPartCount(ok));
p.targetMs=2.0;
p.exceeded2msCount=nnz(v>2.0);
p.exceeded25msCount=nnz(v>25.0);
p.pass2msFraction=nnz(v<=2.0)/numel(v);
% First selected start in each scenario includes MATLAB JIT/warmup effects.
warm=ok;warm(1)=false;
if any(warm)
 v=S.onlinePipelineMs(warm);
 p.warmMedianMs=median(v);p.warmP95Ms=quantileSimple(v,.95);p.warmMaxMs=max(v);
end
end


function [clear,firstUnresolved]=ra_v18_fast_cubic_clear(D,L,observed,geo,maxActiveParts)
% V18: time-first conservative interval screen, NO recursive subdivisions.
% A C1 cubic Hermite interpolant is DEFINED through ALL 161 predicted knots.
% Each 25ms cubic is enclosed by its 4 Bernstein/Bezier control points.
% The full cable at each instant is enclosed by the combined drone/load
% control-point AABBs (a convexity argument); these boxes guarantee
% noncontact for the DEFINED nominal cubic only, not true HLC motion.
% Every rejected shape is proven separate; UNKNOWN for remaining near cases.
% At most maxActiveParts non-globally-excluded obstacles are inspected.
clear=false;firstUnresolved=[NaN NaN];
if nargin<5,maxActiveParts=64;end
if ~isequal(size(D),[3,161]) || ~isequal(size(L),[3,161]) || ...
   any(~isfinite(D(:))) || any(~isfinite(L(:))) || ...
   ~isfield(observed,'parts')
    return;
end
h=.025;
vd=zeros(3,161);vl=zeros(3,161);
vd(:,2:160)=(D(:,3:161)-D(:,1:159))/(2*h);
vl(:,2:160)=(L(:,3:161)-L(:,1:159))/(2*h);
vd(:,1)=(D(:,2)-D(:,1))/h;vd(:,161)=(D(:,161)-D(:,160))/h;
vl(:,1)=(L(:,2)-L(:,1))/h;vl(:,161)=(L(:,161)-L(:,160))/h;
% Four Bezier controls for all 160 segments: vectorized without loops.
Dd0=D(:,1:160);Dd1=Dd0+(h/3)*vd(:,1:160);
Dd3=D(:,2:161);Dd2=Dd3-(h/3)*vd(:,2:161);
Ll0=L(:,1:160);Ll1=Ll0+(h/3)*vl(:,1:160);
Ll3=L(:,2:161);Ll2=Ll3-(h/3)*vl(:,2:161);
loD=min(min(Dd0,Dd1),min(Dd2,Dd3));
hiD=max(max(Dd0,Dd1),max(Dd2,Dd3));
loL=min(min(Ll0,Ll1),min(Ll2,Ll3));
hiL=max(max(Ll0,Ll1),max(Ll2,Ll3));
loC=min(loD,loL);hiC=max(hiD,hiL);
% Ground: an unknown interval MUST NOT be called clear.
if any(loD(3,:)<=geo.droneRadius+geo.safetyMargin) || ...
   any(loL(3,:)<=geo.loadRadius+geo.safetyMargin) || ...
   any(loC(3,:)<=geo.cableRadius+geo.safetyMargin)
    firstUnresolved=[0 0];return;
end
parts=observed.parts;
if isempty(parts),clear=true;return;end
wholeLo=min(loC,[],2);wholeHi=max(hiC,[],2);
Rmax=max([geo.droneRadius,geo.loadRadius,geo.cableRadius])+geo.safetyMargin;
active=0;
for j=1:numel(parts)
    p=parts(j);c=p.center(:);
    if strcmp(char(p.kind),'sphere')
        r=p.params.radius;
    elseif strcmp(char(p.kind),'ellipsoid')
        r=max(p.params.axes);
    else
        firstUnresolved=[-1 j];return;
    end
    if numel(c)~=3 || any(~isfinite(c)) || ...
       ~isscalar(r) || ~isfinite(r) || r<0
        firstUnresolved=[-1 j];return;
    end
    g=max(max(wholeLo-c,0),c-wholeHi);
    if dot(g,g)>(r+Rmax)^2
        continue;
    end
    active=active+1;
    if active>maxActiveParts
        firstUnresolved=[-2 j];return;
    end
    % All 160 25ms segments at once; conservative AABB separation.
    % No recursion, no 40xM nested MATLAB loops, no future-world oracle.
    deltaD=max(max(bsxfun(@minus,loD,c),0),bsxfun(@minus,c,hiD));
    deltaL=max(max(bsxfun(@minus,loL,c),0),bsxfun(@minus,c,hiL));
    deltaC=max(max(bsxfun(@minus,loC,c),0),bsxfun(@minus,c,hiC));
    sepD=sum(deltaD.^2,1)>(r+geo.droneRadius+geo.safetyMargin)^2;
    sepL=sum(deltaL.^2,1)>(r+geo.loadRadius+geo.safetyMargin)^2;
    sepC=sum(deltaC.^2,1)>(r+geo.cableRadius+geo.safetyMargin)^2;
    k=find(~(sepD & sepL & sepC),1,'first');
    if ~isempty(k)
        firstUnresolved=[k j];return;
    end
end
clear=true;
end

function T=ra_v18_fast_cubic_unit_test()
% Offline test checks both a curved separation and conservative unknown.
D=repmat([0;3;3],1,161);L=repmat([0;3;2],1,161);
D(1,:)=linspace(-2,2,161);L(1,:)=D(1,:);
geo=struct('droneRadius',.18,'loadRadius',.12, ...
           'cableRadius',.01,'safetyMargin',0);
obs=struct('parts',struct('kind','sphere','center',[0;0;2], ...
                          'params',struct('radius',.25)));
[ok,~]=ra_v18_fast_cubic_clear(D,L,obs,geo,64);
assert(ok,'RA:V18Test','Separated cubic must be clear');
D(2,:)=0;L(2,:)=0;
[ok,~]=ra_v18_fast_cubic_clear(D,L,obs,geo,64);
assert(~ok,'RA:V18Test','Intersecting cable must be unresolved');
D(3,:)=.05;L(3,:)=.03;
[ok,~]=ra_v18_fast_cubic_clear(D,L,struct('parts',[]),geo,64);
assert(~ok,'RA:V18Test','Ground-intersecting path must be unresolved');
T=struct('status','PASS_BASIC_CURVE_GROUND', ...
 'notProofOfTrueHLCBetweenKnots',true);
end

function q=quantileSimple(x,p)
x=sort(x(:));n=numel(x);if n==0,q=NaN;return,end
z=1+(n-1)*p;lo=floor(z);hi=ceil(z);q=x(lo)+(z-lo)*(x(hi)-x(lo));
end
function out=req(s,a,b,k)
n=numel(strfind(s,a));
if n~=k,error('RA:RunnerTemplate','Expected %d occurrences, got %d: %s',k,n,a);end
out=strrep(s,a,b);
end
function removeTmp(folder)
if isfolder(folder),try,rmdir(folder,'s');catch,end,end
end

% ===== AUDITED TEST-LOCAL V8 REFERENCE =====
% Extracted byte-for-byte (except MATLAB local function name) from
% supplied WAYPOINT_REFERENCE.zip, SHA256-prefix 831e73d23aa4a1096225.
% Encapsulated here to avoid MATLAB path/version drift; original installed
% production predictor, EKF, HLC, plant and governor remain untouched.
function F=ra_test_modal_anchored_v8(H,K,routeCache)
%WP_SIX_TRIGGER_PV_FORECAST Causal PV-history 4s candidate; ONE 25ms tick.
%
% Inputs contain ONLY CURRENT source-model/estimator/governor values,
% past EKF load pL/vL, original HLC gains, and CURRENT route generator.
% V20 does NOT pre-evaluate or store a future route table. They contain NO future HLC or plant.
% Recomputes six-stage virtual-time governor command each 25ms based on
% predicted TRACKING ERROR, with exact original ZOH state transition.
% Current EKF load p/v are EXACT initial conditions. Only the missing
% high-order error modes are fitted to past EKF/reference samples.
% When provided, current EKF cable direction also constrains x/y load
% acceleration at t=0, consistent with the taut-cable reconstruction.
% No post-hoc position snapping or future truth is used.
% Modal output error is derived from ORIGINAL dynamic feedback linearization,
% conditional on taut cable, nonsingular geometry, and unsaturated HLC.
% Tension/physical prediction error are NOT rigorously enclosed.
%
% Do NOT attach finite rho/acc envelopes from this unqualified model;
% downstream geometry must not return physically-certified clear.
required={'pastTimes','pastLoad','pastVelocity','pastRefLoad', ...
    'pastRefVelocity','tauNow','rateNow','heldCommandNow', ...
    'estimatedML','cableLength','gravity','slowdownStart', ...
    'slowdownFull','minScale','filterTime'};
for i=1:numel(required)
    if ~isfield(H,required{i})
        error('LightTrigger:PVInput','Missing current/past data: %s',required{i});
    end
end
m=numel(K.pastRelativeTimes);
if ~isequal(size(H.pastLoad),[3,m]) || ...
   ~isequal(size(H.pastVelocity),[3,m]) || ...
   ~isequal(size(H.pastRefLoad),[3,m]) || ...
   ~isequal(size(H.pastRefVelocity),[3,m]) || ...
   ~isequal(size(H.rateNow),[6,1]) || ...
   numel(H.pastTimes)~=m || ...
   max(abs(reshape(H.pastTimes,1,[])-K.pastRelativeTimes))>1e-9
    error('LightTrigger:PVInput','History or governor state shape is inconsistent');
end
if any(~isfinite([H.pastLoad(:);H.pastVelocity(:); ...
        H.pastRefLoad(:);H.pastRefVelocity(:);H.rateNow(:)])) || ...
        ~isfinite(H.estimatedML) || H.estimatedML<=0 || ...
        ~isfinite(H.cableLength) || H.cableLength<=0 || ...
        ~isfinite(H.gravity) || H.gravity<=0 || ...
        H.slowdownFull<=H.slowdownStart || H.minScale<0 || ...
        H.minScale>1 || ...
        abs(K.governor.filterTime-H.filterTime)>1e-12
    error('LightTrigger:PVInput','Invalid physical/governor data');
end
h=.025;N=160;
Cgov=K.governor;
start=tic;
profileWatch=tic;
if ~isfield(K,'WfreePV') || ~isfield(K,'WfreePVA')
    error('LightTrigger:PVCache','Rebuild the initial-value-constrained PV cache off-tick');
end
errHist=H.pastLoad-H.pastRefLoad;
errVel=H.pastVelocity-H.pastRefVelocity;
% The predicted route is evaluated from the active generator, whereas the
% historical reference is the ORIGINAL HLC reference. They can differ
% slightly even at current tau because the current original HLC jet may
% reflect a different governor/replan phase. Anchor against SAME live reference that will be used in the forecast,
% otherwise even an exact error fit would not give exact current EKF p/v.
assert(isfield(routeCache,'liveDynamic') && routeCache.liveDynamic, ...
    'RA:V20LiveRouteRequired','No pregenerated route cache is accepted');
[pref0,dp0,ddp0]=ra_v20_live_route_xyz(routeCache.generator,H.tauNow);
pref0=pref0(:);dp0=dp0(:);ddp0=ddp0(:);
pref0(3)=pref0(3)-H.cableLength;
gDer=K.governor.D*H.rateNow(:);  % tau' and tau'' at t=0
vref0=dp0*gDer(1);
accRef0=dp0*gDer(2)+ddp0*gDer(1)^2;
fixedEKFErrorP=H.pastLoad(:,end)-pref0;
fixedEKFErrorV=H.pastVelocity(:,end)-vref0;
states=cell(1,3);
yObs=cell(1,3);
fitRMS=zeros(1,3);
for j=1:3
    yObs{j}=[errHist(j,:)';K.velocityWeight*errVel(j,:)'];
    fixed=[fixedEKFErrorP(j);fixedEKFErrorV(j)]; % exact EKF output
    unknown=K.WfreePV{j}*(yObs{j}-K.BfixedPV{j}*fixed);
    states{j}=[fixed;unknown];
    if any(~isfinite(states{j})),error('LightTrigger:PVFit','Nonfinite estimated mode');end
end
% The taut model gives q=-(aLoad+g)/||aLoad+g||. With the vertical
% 2nd-order mode fixed by EKF p/v, the z-acceleration is no longer free.
% If EKF q is available and feasible, constrain horizontal e_ddot(0)
% and fit ONLY the remaining 3 higher modes in each 6th-order axis.
% This maintains pLoad(0), vLoad(0), and (conditionally) q(0) without a jump.
initial=struct('exactEKFLoadPV',true,'cableAnchorRequested',false, ...
    'cableAnchorApplied',false,'cableAnchorStatus','NOT_PROVIDED', ...
    'EKFSourceGeometryMismatch_m',NaN,'EKFDroneMismatch_m',NaN, ...
    'EKFCableAngleMismatch_deg',NaN,'EKFLoadMismatch_m',NaN, ...
    'EKFLoadVelocityMismatch_mps',NaN,'modelFitWeightedRMS',nan(1,3), ...
    'estimatedInitialTension_N',NaN, ...
    'routeCacheCurrentReferenceMismatch_m',norm(pref0-H.pastRefLoad(:,end)), ...
    'routeCacheCurrentVelocityMismatch_mps',norm(vref0-H.pastRefVelocity(:,end)));
qEKF=[];
if isfield(H,'currentEKFCableDirection')
    initial.cableAnchorRequested=true;
    qEKF=H.currentEKFCableDirection(:);
    if numel(qEKF)~=3 || any(~isfinite(qEKF)) || norm(qEKF)<sqrt(eps)
        initial.cableAnchorStatus='INVALID_EKF_CABLE_DIRECTION';
    else
        qEKF=qEKF/norm(qEKF);
        restZ=accRef0(3)+K.A2{3}(1,:)*states{3}+H.gravity;
        % A taut cable requires rest=-lambda*q with lambda>0.
        % q_z near zero makes horizontal acceleration determination unstable.
        if abs(qEKF(3))<=1e-3
            initial.cableAnchorStatus='CABLE_Z_COMPONENT_SINGULAR';
        else
            forcePerMass=-restZ/qEKF(3);
            if ~isfinite(forcePerMass) || forcePerMass<=sqrt(eps)*H.gravity
                initial.cableAnchorStatus='NO_POSITIVE_TAUT_TENSION_CONSISTENCY';
            else
                for j=1:2
                    accNeeded=-forcePerMass*qEKF(j);
                    fixed=[states{j}(1:2);accNeeded-accRef0(j)];
                    free=K.WfreePVA{j}*(yObs{j}-K.BfixedPVA{j}*fixed);
                    states{j}=[fixed;free];
                end
                initial.cableAnchorApplied=true;
                initial.cableAnchorStatus='EXACT_EKF_CABLE_DIRECTION_TAUT_CONSISTENT';
                initial.estimatedInitialTension_N=H.estimatedML*forcePerMass;
            end
        end
    end
end
for j=1:3
    fitRMS(j)=norm(K.Bfull{j}*states{j}-yObs{j})/sqrt(numel(yObs{j}));
end
initial.modelFitWeightedRMS=fitRMS;
fitMs=1000*toc(profileWatch);
Phi=K.Phi;A2=K.A2;
E=zeros(3,N+1); E1=E; E2=E;
tauHist=zeros(1,N+1);dr=zeros(7,N+1);cmdHist=zeros(1,N+1);
z=H.rateNow(:);tau=H.tauNow;cmd=H.heldCommandNow;
tauHist(1)=tau;dr(:,1)=[tau;Cgov.D*z];cmdHist(1)=cmd;
for j=1:3
    E(j,1)=states{j}(1);E1(j,1)=states{j}(2);
    E2(j,1)=A2{j}(1,:)*states{j};
end
profileWatch=tic;
for k=1:N
    for j=1:3
        states{j}=Phi{j}*states{j};
        E(j,k+1)=states{j}(1);
        E1(j,k+1)=states{j}(2);
        E2(j,k+1)=A2{j}(1,:)*states{j};
    end
    [z,tau,tauDer]=wp_six_trigger_governor_fast_step(z,tau,cmd,Cgov);
    tauHist(k+1)=tau;dr(:,k+1)=tauDer;
    ee=norm(E(:,k+1));
    if ee<=H.slowdownStart
        u=0;
    elseif ee>=H.slowdownFull
        u=1;
    else
        u=(ee-H.slowdownStart)/(H.slowdownFull-H.slowdownStart);
    end
    u=u*u*(3-2*u);
    cmd=1-(1-H.minScale)*u;
    cmdHist(k+1)=cmd;
end
modeGovMs=1000*toc(profileWatch);
if any(diff(tauHist)<-1e-10)
    error('RA:V20Governor','Predicted virtual time moved backward');
end
profileWatch=tic;
[ref,dref,d2ref]=ra_v20_live_route_xyz(routeCache.generator,tauHist);
routeInterpMs=1000*toc(profileWatch);
profileWatch=tic;
ref(3,:)=ref(3,:)-H.cableLength;
refAcc=dref.*repmat(dr(3,:),3,1) + ...
       d2ref.*repmat(dr(2,:).^2,3,1);
pLoad=ref+E;
vLoad=dref.*repmat(dr(2,:),3,1)+E1;
aLoad=refAcc+E2;
gvec=repmat([0;0;H.gravity],1,N+1);
rest=aLoad+gvec;
forceNorm=vecnorm(rest,2,1);
if any(~isfinite(forceNorm)) || ...
    min(forceNorm)<=sqrt(eps)*H.gravity
    error('V5:TautReconstruction','Cannot reconstruct positive-tension cable');
end
q=-rest./repmat(forceNorm,3,1);
pDrone=pLoad-H.cableLength*q;
initial.EKFLoadMismatch_m=norm(pLoad(:,1)-H.pastLoad(:,end));
initial.EKFLoadVelocityMismatch_mps=norm(vLoad(:,1)-H.pastVelocity(:,end));
if ~isempty(qEKF) && numel(qEKF)==3 && all(isfinite(qEKF)) && norm(qEKF)>0
    initial.EKFCableAngleMismatch_deg=acos(max(-1,min(1,dot(q(:,1),qEKF))))*180/pi;
end
if isfield(H,'currentEKFDronePosition')
    pd=H.currentEKFDronePosition(:);
    if numel(pd)==3 && all(isfinite(pd))
        initial.EKFDroneMismatch_m=norm(pDrone(:,1)-pd);
        if ~isempty(qEKF) && numel(qEKF)==3 && all(isfinite(qEKF)) && norm(qEKF)>0
            initial.EKFSourceGeometryMismatch_m=norm(H.pastLoad(:,end)-pd-H.cableLength*qEKF);
        end
    end
end
reconstructMs=1000*toc(profileWatch);
F=struct('status','NOMINAL_PV_MODAL_GOVERNOR_NOT_CERTIFIED', ...
    'timeOffsets',(0:N)*h,'pLoad',pLoad,'vLoad',vLoad,'pDrone',pDrone, ...
    'predCableDirection',q,'predTrackingError',E, ...
    'virtualTime',tauHist, ...
    'governorHeldCommand',cmdHist,'governorTauDer',dr, ...
    'modelEstimatedMassUsed',H.estimatedML,'modelLoadMass',H.estimatedML, ...
    'predictedModelTension_N',H.estimatedML*forceNorm, ...
    'rhoDrone',nan(1,N+1),'rhoLoad',nan(1,N+1), ...
    'accDrone',nan(1,N),'accLoad',nan(1,N), ...
    'notCertified',true,'isEligibleForSafeClear',false, ...
    'futureReferenceOracleUsed',false, ...
    'futurePlantUsed',false,'futureHlcUsed',false, ...
    'futureReferenceFromCachedOriginalRoute',true, ...
    'inputSaturationRuledOut',false,'requiresPastEstimatedOutputHistory',true, ...
    'requiresPastEstimatedVelocityHistory',true, ...
    'usesCurrentEKFPositionAndVelocity',true, ...
    'timeMs',1000*toc(start), ...
    'stageMs',struct('modalFit',fitMs,'modalAndGovernor160',modeGovMs, ...
      'routeInterpolation',routeInterpMs,'tautReconstruction',reconstructMs), ...
    'cacheBuiltInTick',false,'fitCondition',K.condition, ...
    'initialAnchor',initial,'initialConditionsTakenFromLatestEKF',true);
end


% ===== TEST-LOCAL MATCHED V8 PV CACHE (OFF TICK) =====
% Function body copied without model/matrix changes from the same
% user-supplied WAYPOINT_REFERENCE.zip as ra_test_modal_anchored_v8.
% Renamed ONLY to avoid a stale installed cache function on MATLAB path.
function K=ra_test_pv_cache_prepare_v7(gains,pastRelativeTimes,filterTime,velocityWeight)
%WP_SIX_TRIGGER_PV_CACHE_PREPARE Precompute all history-fit matrices OFF tick.
% Original HLC F1/F2/F3 => 2nd/6th order EXACT ideal error modes.
% Current EKF pL and vL histories are inputs; weight has units of seconds
% and is the assumed POSITION/VELOCITY measurement std ratio (NOT a
% collision radius, NOT a physical error bound). Caller supplies it.
% Do not rebuild this for each 25-ms tick. Refresh on gain/T/history changes.
if nargin~=4 || ~isscalar(velocityWeight) || ~isfinite(velocityWeight) || velocityWeight<=0
    error('LightTrigger:PVWeight','Provide positive estimator position/velocity noise ratio (seconds)');
end
t=reshape(pastRelativeTimes,1,[]);
if numel(t)<8 || any(~isfinite(t)) || any(diff(t)<=0) || abs(t(end))>1e-9
    error('LightTrigger:PVHistory','Need ordered past times ending at 0');
end
A6=diag(ones(5,1),1);
Ax=A6;Ax(6,:)=-reshape(gains.F2,1,[]);
Ay=A6;Ay(6,:)=-reshape(gains.F3,1,[]);
Az=[0 1;-reshape(gains.F1,1,[])];
As={Ax,Ay,Az};
K=struct();K.W=cell(1,3);K.Phi=cell(1,3);K.A2=cell(1,3);
% Reduced least-squares maps with EKF e(0), e_dot(0) fixed exactly.
% Also precompute the x/y map with e_ddot(0) fixed by EKF cable
% direction, if the taut-cable condition is feasible at this tick.
K.BfixedPV=cell(1,3);K.WfreePV=cell(1,3);
K.BfixedPVA=cell(1,3);K.WfreePVA=cell(1,3);
K.Bfull=cell(1,3);
K.reducedConditionPV=nan(1,3);
K.reducedConditionPVA=nan(1,3);
K.condition=zeros(1,3);
for j=1:3
    A=As{j};n=size(A,1);m=numel(t);
    HM=zeros(m,n);HV=zeros(m,n);
    for k=1:m
        X=expm(A*t(k));
        HM(k,:)=X(1,:);HV(k,:)=X(2,:);
    end
    B=[HM;velocityWeight*HV];
    % This solve happens ONCE at setup, never inside the 25ms call.
    K.W{j}=B\eye(2*m); % retained for compatible offline diagnostics
    K.Bfull{j}=B;
    K.BfixedPV{j}=B(:,1:2);
    if n>2
        U=B(:,3:end);
        K.WfreePV{j}=U\eye(2*m);
        K.reducedConditionPV(j)=cond(U);
    else
        K.WfreePV{j}=zeros(0,2*m); % vertical 2-state: exact EKF p/v
        K.reducedConditionPV(j)=1;
    end
    if n>3
        K.BfixedPVA{j}=B(:,1:3);
        U=B(:,4:end);
        K.WfreePVA{j}=U\eye(2*m);
        K.reducedConditionPVA(j)=cond(U);
    else
        K.BfixedPVA{j}=zeros(2*m,0);
        K.WfreePVA{j}=zeros(0,2*m);
        K.reducedConditionPVA(j)=NaN;
    end
    K.condition(j)=cond(B);
    K.Phi{j}=expm(A*.025);
    K.A2{j}=A*A;
end
K.pastRelativeTimes=t;
K.velocityWeight=velocityWeight;
K.gainsF1=gains.F1;K.gainsF2=gains.F2;K.gainsF3=gains.F3;
K.governor=wp_six_trigger_governor_fast_coeff(.025,filterTime);
K.samplePeriod_s=.025;
K.precomputedOutsideTick=true;
K.initialPVExactlyAnchoredToCurrentEKF=true;
K.cableDirectionCanBeDynamicallyAnchored=true;
K.modelIsConditionalOnUnSaturatedOriginalHLC=true;
K.physicalPredictionErrorCertified=false;
end



function R=ra_v19_1_saved_torque_replay(root,sourceZip)
% V19.2 OFFLINE fix: original generated HLC uses P as a 1x9 ROW. No Flight or predictor.
% Original HLC methods are called only at a few recorded EKF snapshots.
outputDir=fullfile(root,'RESULTS_RA_TWO_PREDICTORS');
assert(isfolder(outputDir),'RA:V19_1NoResults','RESULTS_RA_TWO_PREDICTORS not found');
if isempty(sourceZip)
    d=dir(fullfile(outputDir,'RA_TWO_PREDICTORS_*.zip'));
    assert(~isempty(d),'RA:V19_1NoSource','No original V19 results ZIP under %s',outputDir);
    [~,ind]=max([d.datenum]);
    sourceZip=fullfile(d(ind).folder,d(ind).name);
end
assert(isfile(sourceZip),'RA:V19_1NoSource','Cannot read %s',sourceZip);
work=tempname;mkdir(work);
cleanup=onCleanup(@()removeTmp(work)); %#ok<NASGU>
unzip(sourceZip,work);
matFile=fullfile(work,'RA_TWO_PREDICTORS_RESULT.mat');
assert(isfile(matFile),'RA:V19_1NoSource','ZIP has no RA_TWO_PREDICTORS_RESULT.mat');
tmp=load(matFile,'report');report=tmp.report;clear tmp;
assert(isfield(report,'testRevision') && ...
    strcmp(report.testRevision,'V19_SATURATION_ONLY_ORIGINAL_HLC_OFFLINE_IDENTITY'), ...
    'RA:V19_1WrongRevision','Source results are not the V19 saturation-focus run');
audit=struct();
audit.sourceResultsZip=sourceZip;
audit.kind='V19_2_SAVED_TORQUE_REPLAY_SHAPE_FIX';
audit.neverRunsFlight=true;
audit.neverRunsOnlineForecast=true;
audit.replayIsOffline=true;
audit.gainMethod='reconstructed from original Controller_HL_Suspended_Load SIM lqrd Q,R,dt; poles cross-checked with saved methodBModel';
audit.cases=cell(1,numel(report.scenarios));
for i=1:numel(report.scenarios)
    item=report.scenarios{i};
    one=struct('name',item.scenario.name,'status','NOT_RUN','error','', ...
        'audit',[],'originalFlightModelValid',item.massAudit.flightIsModelConsistent);
    try
        assert(isfield(item,'flightAuditRecords') && ~isempty(item.flightAuditRecords), ...
            'RA:V19_1NoHistory','Source results lack flightAuditRecords');
        rec=item.flightAuditRecords;
        dt=median(diff([rec.time]));
        g=ra_v19_1_reconstruct_original_sim_gains(dt,item);
        one.audit=ra_v19_original_torque_audit(rec,g);
        if one.audit.exactReplayPassed
            one.status='PASS';
        else
            one.status='MISMATCH';
            one.error=sprintf('Original raw HLC torque mismatch %.9g Nm; source reference/state sync or algebra must be examined', ...
                one.audit.maxReplayAbsoluteErrorNm);
        end
    catch ME
        one.status='FAILED';one.error=getReport(ME,'extended','hyperlinks','off');
    end
    audit.cases{i}=one;
end
if all(cellfun(@(x)strcmp(x.status,'PASS'),audit.cases))
    audit.status='PASS_ORIGINAL_HLC_RAW_TORQUE_REPLAY';
else
    audit.status='INCOMPLETE_OR_MISMATCH';
end
audit.generatedOn=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
resultMat=fullfile(work,'RA_V19_2_SAVED_TORQUE_AUDIT.mat');
save(resultMat,'audit','-v7.3');
summary=fullfile(work,'SUMMARY_V19_2.txt');fid=fopen(summary,'w');
assert(fid>=0,'RA:V19_1Summary','Cannot write summary');
summaryCleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'V19.2 OFFLINE SAVED-RESULTS HLC TORQUE REPLAY (P ROW SHAPE FIX)\n');
fprintf(fid,'Status: %s\n',audit.status);
fprintf(fid,'Input: %s\n',audit.sourceResultsZip);
fprintf(fid,'No flight reruns. No on-tick HLC calls. No prediction/geometry changes.\n');
fprintf(fid,'The gains are reconstructed from the original SIM lqrd expressions, with horizontal pole cross-check against saved result.\n');
fprintf(fid,'This is a raw torque REPLAY on recorded CURRENT states, not a future saturation predictor.\n\n');
for i=1:numel(audit.cases)
    q=audit.cases{i};
    fprintf(fid,'[%s] %s\n',q.name,q.status);
    if strcmp(q.status,'FAILED')
        fprintf(fid,'  ERROR: %s\n',q.error);continue;
    end
    a=q.audit;
    fprintf(fid,'  firstTorqueClip_s=%g, axis=%s, replayOK=%d, max abs replay mismatch=%.9g Nm\n', ...
        a.firstTorqueClipTime_s,mat2str(a.firstTorqueClipAxis(:)'),a.exactReplayPassed,a.maxReplayAbsoluteErrorNm);
    fprintf(fid,'  first raw torque=%s Nm; first applied torque=%s Nm\n', ...
        mat2str(a.firstRawTorqueNm(:)',8),mat2str(a.firstAppliedTorqueNm(:)',8));
    fprintf(fid,'  actuator angular accel deficit=(applied - raw)/J=%s rad/s2\n', ...
        mat2str(a.firstAngularAccelerationDeficitRadS2(:)',8));
    fprintf(fid,'  previous reference jet (same EKF state)=%s Nm, current=%s Nm\n', ...
        mat2str(a.counterfactualPreviousJetTorqueNm(:)',8), ...
        mat2str(a.counterfactualCurrentJetTorqueNm(:)',8));
    fprintf(fid,'  sampled time=%s, per-tick raw mismatch=%s Nm\n', ...
        mat2str(a.times_s(:)',8),mat2str(a.perTickAbsErrorNm(:)',8));
    if ~strcmp(q.status,'PASS'),fprintf(fid,'  PROBLEM: %s\n',q.error);end
end
clear summaryCleanup; % flush the summary BEFORE zip
snap=fullfile(work,'RUN_RA_TWO_PREDICTORS_USED.m');
copyfile([mfilename('fullpath') '.m'],snap);
zipName=['RA_V19_2_SAVED_TORQUE_AUDIT_' audit.generatedOn '.zip'];
zipFile=fullfile(outputDir,zipName);
assert(~isfile(zipFile),'RA:V19_1Duplicate','Already exists: %s',zipFile);
tmpZip=fullfile(work,zipName);
zip(tmpZip,{'RA_V19_2_SAVED_TORQUE_AUDIT.mat','SUMMARY_V19_2.txt', ...
    'RUN_RA_TWO_PREDICTORS_USED.m'},work);
[ok,message]=movefile(tmpZip,zipFile);
assert(ok,'RA:V19_1Output','Cannot publish one ZIP: %s',message);
R=struct('status',audit.status,'zipPath',zipFile,'mode','replay', ...
    'sourceZip',sourceZip,'nScenarios',numel(report.scenarios));
fprintf('[RA V19.2 SAVED REPLAY] %s\nResult ZIP: %s\n',R.status,R.zipPath);
end

function g=ra_v19_1_reconstruct_original_sim_gains(dt,item)
% EXACT Q/R choices in original Controller_HL_Suspended_Load.m SIM branch.
% Do not silently assume these are current active gains: verify saved poles.
assert(abs(dt-0.025)<1e-7,'RA:V19_1Dt','Unexpected HLC dt %.9g',dt);
A2=diag(1,1);B2=[0;1];
A6=diag([1 1 1 1 1],1);B6=[0;0;0;0;0;1];
g=struct('dt',dt, ...
    'F1',lqrd(A2,B2,diag([150,10]),0.01,dt), ...
    'F2',lqrd(A6,B6,diag([100,150,100,10,1,0.1]),0.001,dt), ...
    'F3',lqrd(A6,B6,diag([100,150,100,10,1,0.1]),0.001,dt), ...
    'F4',lqrd(A2,B2,diag([10,1]),0.01,dt));
assert(isfield(item,'methodBModel') && isfield(item.methodBModel,'polesX'), ...
    'RA:V19_1NoGainEvidence','Saved B poles needed for gain validation');
originalPolesX=eig(A6-B6*g.F2);
originalPolesY=eig(A6-B6*g.F3);
for c={'X','Y'}
  if strcmp(c{1},'X'), saved=item.methodBModel.polesX(:);pred=originalPolesX;
  else,saved=item.methodBModel.polesY(:);pred=originalPolesY;end
  assert(numel(saved)==4,'RA:V19_1UnexpectedPoles','Expected 4 retained conjugate poles');
  % each of four slow eigenvalues must match one of full six, no ordering assumption
  mismatches=arrayfun(@(z)min(abs(pred(:)-z)),saved);
  assert(max(mismatches)<1e-7,'RA:V19_1GainMismatch', ...
      'Original controller lqrd gains fail saved %s pole check (max %.9g)',c{1},max(mismatches));
end
end


% ===== V20 ON-DEMAND EXACT WAYPOINT JETS: NO FUTURE ROUTE CACHE =====
function [p,dp,ddp]=ra_v20_live_route_xyz(generator,tau)
% Evaluate the CURRENT generator's analytic parameters ONLY WHEN CALLED.
% No predicted timestamps/position table is prepared or saved. The
% generator may be replaced between calls; no persistent/hidden cache.
% For the supplied gen_ref_waypoint_smooth/periodic, outputs implement
% EXACT analytic derivatives p,p_tau,p_tautau (up to roundoff).
% The governor uses the chain rule separately. No heavy 28-jet generator
% calls or high-order symbolic computations for each of the 161 samples.
assert(isstruct(generator) && isfield(generator,'curveType'), ...
 'RA:V20LiveRoute','Generator must expose the active analytic waypoint type');
tau=reshape(tau,1,[]);n=numel(tau);
assert(n>0 && all(isfinite(tau)),'RA:V20Tau','Invalid on-demand route times');
assert(isfield(generator,'pathLength') && isfield(generator,'speedPeak') && ...
       isfield(generator,'rampTimeEffective') && isfield(generator,'waypoints'), ...
       'RA:V20LiveRoute','Original waypoint generator parameters missing');
D=generator.pathLength;v=generator.speedPeak;Ta=generator.rampTimeEffective;
assert(D>0 && v>0 && Ta>0,'RA:V20LiveRoute','Invalid analytic route profile');
s=zeros(1,n);sd=zeros(1,n);sdd=zeros(1,n);
if generator.isLoop
    ramp=(tau>0 & tau<Ta);cruise=(tau>=Ta);
    if any(ramp)
        u=tau(ramp)/Ta;
        si=ra_v20_smooth13(u,-1);
        s(ramp)=v*Ta*si;
        sd(ramp)=v*ra_v20_smooth13(u,0);
        sdd(ramp)=v/Ta*ra_v20_smooth13(u,1);
    end
    s(cruise)=v*(tau(cruise)-Ta/2);sd(cruise)=v;
else
    Tc=max(0,D/v-Ta);T=2*Ta+Tc;
    rise=(tau>0 & tau<Ta);
    cruise=(tau>=Ta & tau<Ta+Tc);
    fall=(tau>=Ta+Tc & tau<T);
    stop=(tau>=T);
    if any(rise)
        u=tau(rise)/Ta;
        s(rise)=v*Ta*ra_v20_smooth13(u,-1);
        sd(rise)=v*ra_v20_smooth13(u,0);
        sdd(rise)=v/Ta*ra_v20_smooth13(u,1);
    end
    s(cruise)=v*(tau(cruise)-Ta/2);sd(cruise)=v;
    if any(fall)
        u=(tau(fall)-Ta-Tc)/Ta;
        s(fall)=v*(Ta/2+Tc+Ta*(u-ra_v20_smooth13(u,-1)));
        sd(fall)=v*(1-ra_v20_smooth13(u,0));
        sdd(fall)=-v/Ta*ra_v20_smooth13(u,1);
    end
    s(stop)=D;
    s=max(0,min(D,s));
end
if strcmp(generator.curveType,'open_waypoints')
    W=generator.waypoints;S=generator.cumulativeS;
    b=generator.blendRadiusEffective;
    assert(size(W,1)==3 && size(W,2)==numel(S) && numel(S)==numel(b), ...
       'RA:V20OpenShape','Open-route analytic data mismatch');
    if isfield(generator,'raV21Derived') && ...
            strcmp(generator.raV21Derived.curveType,'open_waypoints')
        dirs=generator.raV21Derived.unitDirections;
    else
        seg=diff(W,1,2);len=sqrt(sum(seg.^2,1));dirs=seg./len;
    end
    % The number of waypoints is small; this index computation is batched.
    ix=sum(bsxfun(@ge,s(:),S(1:end-1)),2);
    ix=max(1,min(size(dirs,2),ix));ix=reshape(ix,1,[]);
    local=bsxfun(@minus,s,S(ix));
    P=W(:,ix)+bsxfun(@times,dirs(:,ix),local);
    Ps=dirs(:,ix);Pss=zeros(3,n);
    for j=2:size(W,2)-1
        if b(j)<=0,continue;end
        hit=abs(s-S(j))<=b(j);
        if ~any(hit),continue;end
        x=s(hit)-S(j);u=(x+b(j))/(2*b(j));
        H0=ra_v20_smooth13(u,0);
        H1=ra_v20_smooth13(u,1);
        H2=ra_v20_smooth13(u,2);
        dd=dirs(:,j)-dirs(:,j-1);
        P(:,hit)=bsxfun(@plus,W(:,j),bsxfun(@times, ...
            bsxfun(@plus,dirs(:,j-1),bsxfun(@times,dd,H0)),x));
        Ps(:,hit)=bsxfun(@plus,dirs(:,j-1),bsxfun(@times,dd, ...
            H0+x.*H1/(2*b(j))));
        Pss(:,hit)=bsxfun(@times,dd,2*H1/(2*b(j)) + ...
            x.*H2/(2*b(j))^2);
    end
elseif strcmp(generator.curveType,'periodic_fourier')
    W=generator.waypoints;N=size(W,2);
    assert(size(W,1)==3 && N>=15,'RA:V20PeriodicShape','Invalid Fourier route samples');
    if isfield(generator,'raV21Derived') && ...
            strcmp(generator.raV21Derived.curveType,'periodic_fourier')
        C=generator.raV21Derived.fourierC;
        freq=generator.raV21Derived.fourierFreq;
    else
        C=fft(W,[],2)/N;
        if mod(N,2)==0
            freq=[0:N/2,-N/2+1:-1];
        else
            freq=[0:(N-1)/2,-(N-1)/2:-1];
        end
        freq=freq(:);
    end
    omega=2*pi/D;
    phase=2*pi*mod(s,D)/D;
    z=exp(1i*(freq*phase));
    q=1i*freq*omega;
    P=real(C*z);
    Ps=real(C*bsxfun(@times,q,z));
    Pss=real(C*bsxfun(@times,q.^2,z));
else
    error('RA:V20UnsupportedRoute', ...
      'Active route class %s needs an analytic batch evaluator; no silent 161-call fallback', ...
      char(generator.curveType));
end
p=P;dp=bsxfun(@times,Ps,sd);
ddp=bsxfun(@times,Ps,sdd)+bsxfun(@times,Pss,sd.^2);
assert(isequal(size(p),[3,n]) && all(isfinite([p(:);dp(:);ddp(:)])), ...
   'RA:V20RouteFinite','Nonfinite analytic future reference');
end

function y=ra_v20_smooth13(u,k)
% V21 EXACT algebraic rewriting of original degree-13 smoothstep.
% S13'(u)=12012*u^6*(1-u)^6, S13''=72072*u^5*(1-u)^5*(1-2*u).
% The primitive and S13 use Horner (same coefficients as original).
% Symmetry about 1/2 avoids endpoint cancellation for values near 1.
assert(any(k==[-1,0,1,2]),'RA:V21SmoothOrder','Only derivatives -1..2 required');
u=max(0,min(1,u));
flip=u>0.5;w=u;w(flip)=1-u(flip);
if k==1
    a=w.*(1-w);
    y=12012*a.^6;
elseif k==2
    a=w.*(1-w);
    y=72072*a.^5.*(1-2*w);
    y(flip)=-y(flip);
else
    coeff=[1716,-9009,20020,-24024,16380,-6006,924];
    if k==-1
        coeff=coeff./(8:14);power=8;
    else
        power=7;
    end
    y=coeff(7)*ones(size(w));
    for j=6:-1:1
        y=y.*w+coeff(j);
    end
    y=y.*(w.^power);
    if k==-1
        y(flip)=u(flip)-0.5+y(flip);
    else
        y(flip)=1-y(flip);
    end
end
end

function g=ra_v21_current_route_snapshot(generator)
% Per CURRENT trigger. Only algebraic geometry coefficients are prepared.
% NEVER evaluates future trajectory samples, never uses persistent cache.
% A changed generator is consumed on the very next prediction call.
g=generator;
assert(isstruct(g) && isfield(g,'curveType') && ...
       isfield(g,'waypoints'),'RA:V21Route','Missing current route');
r=struct('curveType',char(g.curveType));
W=g.waypoints;
if strcmp(g.curveType,'open_waypoints')
    d=diff(W,1,2);
    a=sqrt(sum(d.^2,1));
    assert(all(isfinite(a)) && all(a>0),'RA:V21Route','Bad segment length');
    r.unitDirections=bsxfun(@rdivide,d,a);
elseif strcmp(g.curveType,'periodic_fourier')
    N=size(W,2);
    assert(N>=15,'RA:V21Route','Too few Fourier input waypoints');
    r.fourierC=fft(W,[],2)/N;
    if mod(N,2)==0
        r.fourierFreq=[0:N/2,-N/2+1:-1]';
    else
        r.fourierFreq=[0:(N-1)/2,-(N-1)/2:-1]';
    end
else
    error('RA:V21UnsupportedRoute','Unsupported current route %s',char(g.curveType));
end
g.raV21Derived=r;
end

function t=ra_v20_live_route_selftest()
% Off-tick parity test vs ORIGINAL generator, including fast changes.
% No future route table preparation; only a few independently chosen probes.
a=gen_ref_waypoint_smooth('pointsXYZ',[0 0 3;0 1 3;1 2 3;1 3 3], ...
  'speed',0.75,'blendRadius',0.4);
b=gen_ref_waypoint_smooth('pointsXYZ',[0 0 3;0 1 3;-1 2 3;-1 3 3], ...
  'speed',0.75,'blendRadius',0.4);
c=gen_ref_waypoint_periodic('pointsXYZ',[cos((0:19)'*2*pi/20), ...
  sin((0:19)'*2*pi/20),3+0.1*cos((0:19)'*4*pi/20)],'speed',0.4);
probes=[-0.5 0 .1 1 2 4 8 12 25 100];err=0;changes=false;
for g={a,b,c}
    [P,V,A]=ra_v20_live_route_xyz(g{1},probes);
    snapshot=ra_v21_current_route_snapshot(g{1});
    [Ps,Vs,As]=ra_v20_live_route_xyz(snapshot,probes);
    assert(max(abs([P(:)-Ps(:);V(:)-Vs(:);A(:)-As(:)]))<1e-12, ...
       'RA:V21SnapshotParity','Current route snapshot changed analytic values');
    for k=1:numel(probes)
        x=g{1}.evaluate(probes(k));
        err=max([err,norm(P(:,k)-x(1:3)), ...
          norm(V(:,k)-x(5:7)),norm(A(:,k)-x(9:11))]);
    end
end
probeChange=0.55*min(a.duration,b.duration);
[pa,~,~]=ra_v20_live_route_xyz(ra_v21_current_route_snapshot(a),probeChange);
[pb,~,~]=ra_v20_live_route_xyz(ra_v21_current_route_snapshot(b),probeChange);
changes=norm(pa-pb)>1e-8;
assert(err<5e-7 && changes,'RA:V20Parity', ...
    'Live route parity/update failed, err=%g routeChanged=%d',err,changes);
t=struct('status','PASS_OFFTICK_REFERENCE_PARITY', ...
 'maxPVAccelDifference',err,'routeChangeVisibleWithoutCache',changes, ...
 'isMatlabRealtimeBenchmark',false);
end
