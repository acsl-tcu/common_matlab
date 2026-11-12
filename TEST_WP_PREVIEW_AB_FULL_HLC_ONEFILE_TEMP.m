function [AB_Results, AB_Details, BatchSummary] = TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP(lookAheadSeconds, samplingConfig)
% One MATLAB file. NO ARGUMENTS = run ALL 14 functional regression cases:
% original 6 cases + C/B/D geometry, circle/saddle, moving obstacles,
% a synthetic capped 36-part stress, and 0.25s monitoring benchmarks.
% Explicit TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP(4,opts) still runs ONE case.
% The original HLC, EKF, V3 and plant remain unchanged; no output files saved.
if nargin==0 || (nargin>=1 && (ischar(lookAheadSeconds) || ...
        (isstring(lookAheadSeconds)&&isscalar(lookAheadSeconds))) && ...
        strcmpi(char(lookAheadSeconds),'batch'))
    cases = {
        '4s_default_sphere', 4, struct('probeMode','grazing', ...
            'probeShape','sphere','obstacleMethod','multiple_spheres');
        '6s_default_sphere', 6, struct('probeMode','grazing', ...
            'probeShape','sphere','obstacleMethod','multiple_spheres');
        '4s_overlap_ellipsoid_approx', 4, struct('probeMode','overlap', ...
            'probeShape','sphere','obstacleMethod','multiple_ellipsoids');
        '4s_rotated_ellipsoid_grazing', 4, struct('probeMode','grazing', ...
            'probeShape','ellipsoid','obstacleMethod','multiple_ellipsoids');
        '4s_rotated_ellipsoid_overlap', 4, struct('probeMode','overlap', ...
            'probeShape','ellipsoid','obstacleMethod','multiple_ellipsoids');
        '4s_real_mixed_ellipsoids', 4, struct('probeMode','none', ...
            'sceneName','scene_mixed_obstacles', ...
            'obstacleMethod','multiple_ellipsoids');
        '4s_B_36parts', 4, struct('probeMode','none', ...
            'routeMode','quick','geometryMethod','B', ...
            'sceneName','scene_showcase_13','obstacleMethod','multiple_ellipsoids', ...
            'stressTo36Parts',true,'runPeriodicBenchmark',true);
        '4s_circle_C_36parts', 4, struct('probeMode','none', ...
            'routeMode','circle','geometryMethod','C', ...
            'sceneName','scene_showcase_13','obstacleMethod','multiple_ellipsoids', ...
            'stressTo36Parts',true,'runPeriodicBenchmark',true);
        '4s_saddle_D_36parts', 4, struct('probeMode','none', ...
            'routeMode','saddle','geometryMethod','D', ...
            'sceneName','scene_showcase_13','obstacleMethod','multiple_ellipsoids', ...
            'stressTo36Parts',true,'runPeriodicBenchmark',true);
        '4s_dynamic_B_circle_saddle', 4, struct('probeMode','none', ...
            'routeMode','circle_saddle','geometryMethod','B', ...
            'sceneName','scene_dynamic_independent', ...
            'obstacleMethod','multiple_ellipsoids','runPeriodicBenchmark',true);
        '4s_dynamic_C_circle', 4, struct('probeMode','none', ...
            'routeMode','circle','geometryMethod','C', ...
            'sceneName','scene_dynamic_independent', ...
            'obstacleMethod','multiple_ellipsoids');
        '4s_dynamic_D_saddle', 4, struct('probeMode','none', ...
            'routeMode','saddle','geometryMethod','D', ...
            'sceneName','scene_dynamic_independent', ...
            'obstacleMethod','multiple_ellipsoids');
        '4s_rejoin_straight_B',4,struct('probeMode','none', ...
            'routeMode','straight','geometryMethod','B', ...
            'sceneName','scene_cylinders_high_side', ...
            'obstacleMethod','multiple_ellipsoids', ...
            'rejoinTime_s',9.0,'rejoinShift_m',[0;0.55;0], ...
            'checkpointTimes_s',[9.0 9.5 10.5 13.0], ...
            'runPeriodicBenchmark',true,'monitorWindow_s',[9 10]);
        '4s_rejoin_turn_D',4,struct('probeMode','none', ...
            'routeMode','turn','geometryMethod','D', ...
            'sceneName','scene_cylinders_high_side', ...
            'obstacleMethod','multiple_ellipsoids', ...
            'rejoinTime_s',9.0,'rejoinShift_m',[0.45;-0.45;0], ...
            'checkpointTimes_s',[9.0 9.5 10.5 13.0], ...
            'runPeriodicBenchmark',true,'monitorWindow_s',[9 10])
    };
    AB_Results=table();AB_Details=struct([]);BatchSummary=table();
    allAdaptive=table();allProbe=table();allPeriodic=table();
    allSphereGate=table();allRejoin=table();
    fprintf('\n=== AUTOMATIC BATCH: %d independent cases / one command ===\n',size(cases,1));
    for i=1:size(cases,1)
        tag=string(cases{i,1});h=cases{i,2};cfg=cases{i,3};
        fprintf('\n>>>>>>>> CASE %d/%d: %s <<<<<<<<\n',i,size(cases,1),char(tag));
        try
            [r,d,a,q,m,g,rejoinTbl]=run_single_ab_scenario_onefile(h,cfg);
        catch ME
            % One failed test must not prevent the remaining independent
            % feature checks. Failures remain visible in the summary.
            warning('ABPreview:BatchCaseFailed', ...
                'Batch case %s failed: %s',char(tag),ME.message);
            failed=table(tag,h,false,string(ME.message),0,0,0,0,0,0,0,0,0,0,NaN, ...
                'VariableNames',{'TestCase','Horizon_s','Completed', ...
                'ErrorMessage','ABRows','Snapshots','BaselineProbeHits', ...
                'AdaptiveProbeHits','DenseNominalProbeHitCases', ...
                'TotalExtraNodes','TotalUnrefinedNearIntervals','TotalAdaptiveWall_s', ...
                'MonitorTicks','MonitorOverruns','MonitorMaxWall_ms'});
            if isempty(BatchSummary),BatchSummary=failed;
            else,BatchSummary=[BatchSummary;failed];end %#ok<AGROW>
            continue
        end
        r.TestCase=repmat(tag,height(r),1);
        if isempty(AB_Results),AB_Results=r;else,AB_Results=[AB_Results;r];end %#ok<AGROW>
        for idxD=1:numel(d),d(idxD).TestCase=char(tag);end
        if isempty(AB_Details),AB_Details=d;else,AB_Details=[AB_Details,d];end %#ok<AGROW>
        if ~isempty(a)
            a.TestCase=repmat(tag,height(a),1);
            if isempty(allAdaptive),allAdaptive=a;else,allAdaptive=[allAdaptive;a];end %#ok<AGROW>
        end
        if ~isempty(q)
            q.TestCase=repmat(tag,height(q),1);
            if isempty(allProbe),allProbe=q;else,allProbe=[allProbe;q];end %#ok<AGROW>
        end
        if ~isempty(m)
            m.TestCase=repmat(tag,height(m),1);
            if isempty(allPeriodic),allPeriodic=m;
            else,allPeriodic=[allPeriodic;m];end %#ok<AGROW>
        end
        if ~isempty(g)
            g.TestCase=repmat(tag,height(g),1);
            if isempty(allSphereGate),allSphereGate=g;
            else,allSphereGate=[allSphereGate;g];end %#ok<AGROW>
        end
        if ~isempty(rejoinTbl)
            rejoinTbl.TestCase=repmat(tag,height(rejoinTbl),1);
            if isempty(allRejoin),allRejoin=rejoinTbl;
            else,allRejoin=[allRejoin;rejoinTbl];end %#ok<AGROW>
        end
        if isempty(q)
            nBase=0;nAd=0;nDense=0;
        else
            nBase=sum(q.BaselineAnyHit);nAd=sum(q.AdaptiveAnyHit);
            nDense=sum(q.DenseNominalHits>0);
        end
        nMonitor=0;nOver=0;maxMonitorMs=NaN;
        if ~isempty(m)
            nMonitor=height(m);nOver=sum(m.Overrun);maxMonitorMs=max(m.Wall_ms);
        end
        one=table(tag,h,true,string(''),height(r),numel(d),nBase,nAd,nDense, ...
            sum(a.ExtraSamples),sum(a.UnrefinedCandidateIntervals), ...
            sum(a.PredictionWall_s),nMonitor,nOver,maxMonitorMs, ...
            'VariableNames',{'TestCase','Horizon_s','Completed', ...
            'ErrorMessage','ABRows','Snapshots','BaselineProbeHits', ...
            'AdaptiveProbeHits','DenseNominalProbeHitCases', ...
            'TotalExtraNodes','TotalUnrefinedNearIntervals','TotalAdaptiveWall_s', ...
            'MonitorTicks','MonitorOverruns','MonitorMaxWall_ms'});
        if isempty(BatchSummary),BatchSummary=one;
        else,BatchSummary=[BatchSummary;one];end %#ok<AGROW>
    end
    assignin('base','WP_AB_Results',AB_Results);
    assignin('base','WP_AB_Details',AB_Details);
    assignin('base','WP_Adaptive_Results',allAdaptive);
    assignin('base','WP_Probe_Results',allProbe);
    assignin('base','WP_AB_BatchSummary',BatchSummary);
    assignin('base','WP_MonitorCycle_Results',allPeriodic);
    assignin('base','WP_C_SphereGate_Results',allSphereGate);
    assignin('base','WP_Rejoin_Results',allRejoin);
    fprintf('\n=== ALL BATCH CASES FINISHED ===\n');disp(BatchSummary);
else
    if nargin<1 || isempty(lookAheadSeconds),lookAheadSeconds=4;end
    if nargin<2 || isempty(samplingConfig),samplingConfig=struct();end
    [AB_Results,AB_Details]=run_single_ab_scenario_onefile( ...
        lookAheadSeconds,samplingConfig);
    BatchSummary=table();
end
end

function [AB_Results, AB_Details, Adaptive_Results, Probe_Results, Periodic_Results, SphereGate_Results, Rejoin_Results] = run_single_ab_scenario_onefile(lookAheadSeconds, samplingConfig)
% TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP
% ========================================================================
% Waypoint V3 + unchanged HLC: configurable-horizon A versus B evaluation.
% ONE FILE ONLY. Put this file at commonmatlab/ root and run:
%   [AB_Results,AB_Details,BatchSummary] = TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP;
% No-argument invocation automatically performs fourteen functional tests. 
%
% A = existing flatness-based nominal future path from the *live V3 state*.
% B = integrate the EXISTING 24-state suspended-load plant model, starting
%     from the CURRENT estimator state (not an ad hoc pendulum).
%     C_closed: local closed-loop output-error forecast based on ORIGINAL
%       HLC feedback-linearized channels (x,y: 6th; z: 2nd).
%       The active V3 governor is propagated with predicted tracking error.
%       NO held physical input, NO future measured input, NO MPC.
%       This is model-conditional, not a safety or accuracy certificate.
%     B_replay_est: REPLAY FUTURE RECORDED HLC INPUTS from estimated state.
%       ***NON-CAUSAL ORACLE DIAGNOSTIC ONLY. NOT an online predictor.***
%     B_replay_true: same replay from actual PLANT state, for model / time
%       alignment validation only. Uses privileged simulation information.
% Estimated mass: for C_closed and B_replay_est, the model parameters are
% read from the live vehicle parameter object, and the unknown payload mass
% is replaced by the EKF's current mL at each checkpoint. The real plant
% mL is used ONLY in the B_replay_true offline diagnostic.
% Adaptive sampling is TEST ONLY: coarse samples depend on horizon
% and speed; extra nodes have <=12/obstacle and <=36/global defaults.
% Whole-route static obstacle exclusions are mathematical for the NOMINAL
% reference, while nearby time intervals remain only sample-checked.
% No closed-loop safety guarantee is claimed.
% CURRENT-POSE VALIDATION: The actual EKF state at each checkpoint is fed
% into wp_preview_4s via opts.currentPose. This checks the CURRENT system
% shape, not the future closed-loop state. The tracking threshold is a
% diagnostic flag, NOT a theoretically proven safe tracking bound.
%
% Every method is compared with the actual unchanged HLC/plant flight.
% No MPC, no replacement controller, no CBF, no replanning, no new files,
% and NO automatic figures, videos, MAT files or CSV outputs.
%
% References: Sreenath/Lee/Kumar (IEEE CDC 2013) use differential flatness
% for nominal suspended-load trajectories. Potdar/de Croon/Alonso-Mora
% (Autonomous Robots 2020) propagate estimated states using dynamics in
% their NMPC; here we ONLY test non-optimizing state propagation.
%
% MATLAB FIX: use string scalars for DRONE_PARAM_SUSPENDED_LOAD.set;
% store true x(t_k) BEFORE plant.do and replay u(t_k...k+n-1).
% IMPORTANT: No 4s input-hold predictor is run. B_replay is a NONCAUSAL
% diagnostic using recorded future inputs. C_closed predicts only the
% LOCAL feedback-linearized nominal closed-loop output response; its
% actual-flight accuracy must be measured, especially under mass mismatch,
% saturation, and flight-mode changes.
% ========================================================================

% External configuration, with the familiar default of 4 seconds:
%   TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP
%   TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP(6)
%   TEST_WP_PREVIEW_AB_FULL_HLC_ONEFILE_TEMP(4,struct( ...
%       'coarseTravel_m',0.25,'maxExtraTotal',36,'sceneName','scene_cylinders_high_side'))
% Uses mathematical whole-route spatial enclosures to exclude static far
% obstacles. Nearby intervals are sampled and remain UNVERIFIED in between.
if nargin<1 || isempty(lookAheadSeconds),lookAheadSeconds=4.0;end
if nargin<2 || isempty(samplingConfig),samplingConfig=struct();end
assert(isstruct(samplingConfig)&&isscalar(samplingConfig), ...
    'samplingConfig must be a scalar struct');
assert(isnumeric(lookAheadSeconds) && isscalar(lookAheadSeconds) && ...
    isfinite(lookAheadSeconds) && lookAheadSeconds>0, ...
    'lookAheadSeconds must be a finite positive scalar');

%% EDIT ONLY THIS SECTION
selfDir = fileparts(mfilename('fullpath'));
addpath(genpath(selfDir));

% For a shorter first run comment out some rows. Each row is an independent
% simulation using the ORIGINAL HLC, EKF, TAKEOFF and V3 Waypoint.
routeCases = {
    'gen_wp_straight',       0.50;
    'gen_wp_straight',       1.50;
    'gen_wp_gentle_turn',    1.00;
    'gen_wp_circle',         0.50;
    'gen_wp_saddle',         1.00;
};

% Route selection is configured below, not by a separate manual edit.
% Full five-route regression uses routeMode='all'.


plantDt       = 0.025;   % Exactly as run_flight_SimSuspendedLoad_trj.m
checkTimes    = []; % set from configuration below
lookAhead = lookAheadSeconds;
assert(abs(lookAhead/plantDt-round(lookAhead/plantDt))<1e-9, ...
    'In this one-file diagnostic horizon must be a multiple of 0.025s');

% Sampling settings: the baseline never disappears. The baseline step is
% chosen per route from nominal speed and a travel-per-sample preference.
sampleDefaults=struct('coarseTravel_m',0.25,'maxCoarseStep_s',0.20, ...
    'minCoarseStep_s',0.025,'fineStep_s',0.05, ...
    'nearBand_m',0.55,'groundNearBand_m',0.10, ...
    'maxExtraPerObstacle',12,'maxGroundExtra',6,'maxExtraTotal',36, ...
    'includeGroundAdaptive',true, ...
    'probeMode','grazing','probeShape','sphere', ... % shape: sphere/ellipsoid
    'probeRadius_m',0.06,'probePenetration_m',0.003, ...
    'probeIntervalFraction',0.30, ...
    'sceneName','scene_cylinders_high_side', ...
    'obstacleMethod','multiple_spheres', ...
    'denseTruthCheck',true, ...
    'geometryMethod','B','routeMode','quick', ...
    'stressTo36Parts',false, ... % test-only: duplicate real parts to exactly 36
    'validateFastPath',true, ... % one off-timing comparison per real flight
    'runPeriodicBenchmark',false, ...
    'monitorPeriod_s',0.25,'monitorWindow_s',[7.50 8.50], ...
    'checkpointTimes_s',[7.5,10.5,13.0], ...
    'rejoinTime_s',NaN, ...
    'rejoinShift_m',[0;0;0], ...
    'rejoinVelocityShift_mps',[0;0;0]);
fn=fieldnames(samplingConfig);
for fi=1:numel(fn)
    assert(isfield(sampleDefaults,fn{fi}), ...
        'Unknown samplingConfig field: %s',fn{fi});
    sampleDefaults.(fn{fi})=samplingConfig.(fn{fi});
end
checkTimes=sampleDefaults.checkpointTimes_s(:)';
assert(isnumeric(checkTimes)&&~isempty(checkTimes)&& ...
    all(isfinite(checkTimes))&&all(diff(checkTimes)>0)&& ...
    all(abs(checkTimes/plantDt-round(checkTimes/plantDt))<1e-8), ...
    'checkpointTimes_s must be increasing multiples of 0.025s');
assert(isnumeric(sampleDefaults.rejoinShift_m)&& ...
    numel(sampleDefaults.rejoinShift_m)==3&& ...
    all(isfinite(sampleDefaults.rejoinShift_m(:))) && ...
    isnumeric(sampleDefaults.rejoinVelocityShift_mps)&& ...
    numel(sampleDefaults.rejoinVelocityShift_mps)==3&& ...
    all(isfinite(sampleDefaults.rejoinVelocityShift_mps(:))), ...
    'Rejoin shifts must be finite 3-vectors');
if isfinite(sampleDefaults.rejoinTime_s)
    assert(sampleDefaults.rejoinTime_s>=5 && ...
        abs(sampleDefaults.rejoinTime_s/plantDt - ...
        round(sampleDefaults.rejoinTime_s/plantDt))<1e-8 && ...
        checkTimes(1)>=sampleDefaults.rejoinTime_s, ...
        'Rejoin perturbation must begin during flight, before first checkpoint');
end
flightEndTime=max(17.50,ceil((max(checkTimes)+lookAhead+0.5)/plantDt)*plantDt);
assert(flightEndTime<47, ...
    'Selected checkpoints and horizon must remain during flight');
switch char(sampleDefaults.routeMode)
    case 'quick', routeCases=routeCases([1 3],:);
    case 'all', routeCases=routeCases;  % original five-route grid
    case 'circle', routeCases=routeCases(4,:);
    case 'saddle', routeCases=routeCases(5,:);
    case 'circle_saddle', routeCases=routeCases([4 5],:);
    case 'straight', routeCases=routeCases(1,:);
    case 'turn', routeCases=routeCases(3,:);
    otherwise,error('Unknown routeMode');
end
assert(any(strcmp(char(sampleDefaults.geometryMethod),{'C','B','D'})), ...
    'geometryMethod must be C, B or D');
assert(isnumeric(sampleDefaults.monitorWindow_s) && ...
    numel(sampleDefaults.monitorWindow_s)==2 && ...
    all(isfinite(sampleDefaults.monitorWindow_s)) && ...
    sampleDefaults.monitorWindow_s(2)>=sampleDefaults.monitorWindow_s(1), ...
    'monitorWindow_s must be [start end]');
assert(isnumeric(sampleDefaults.monitorPeriod_s) && ...
    isscalar(sampleDefaults.monitorPeriod_s) && ...
    sampleDefaults.monitorPeriod_s>=plantDt && ...
    abs(sampleDefaults.monitorPeriod_s/plantDt- ...
    round(sampleDefaults.monitorPeriod_s/plantDt))<1e-8, ...
    'monitorPeriod_s must be an integer multiple of 0.025s');
assert(any(strcmp(char(sampleDefaults.probeMode),{'none','grazing','overlap'})), ...
    'probeMode must be none, grazing or overlap');
assert(any(strcmp(char(sampleDefaults.probeShape),{'sphere','ellipsoid'})), ...
    'probeShape must be sphere or ellipsoid');
assert(sampleDefaults.probeRadius_m>0 && sampleDefaults.probePenetration_m>0 && ...
    sampleDefaults.probeIntervalFraction>0 && sampleDefaults.probeIntervalFraction<1, ...
    'Invalid synthetic probe settings');
assert(sampleDefaults.maxExtraPerObstacle>=0 && ...
    mod(sampleDefaults.maxExtraPerObstacle,1)==0 && ...
    sampleDefaults.maxGroundExtra>=0 && ...
    mod(sampleDefaults.maxGroundExtra,1)==0 && ...
    sampleDefaults.maxExtraTotal>=0 && ...
    mod(sampleDefaults.maxExtraTotal,1)==0, ...
    'Sample budgets must be nonnegative integers');

% Collision comparisons use the same one geometry method for all A/B
% predictions. This is a GEOMETRY choice C/B/D, NOT prediction A/B.
geometryMethod = char(sampleDefaults.geometryMethod);  % C/B/D, external
sceneName      = sampleDefaults.sceneName;
obstacleMethod = sampleDefaults.obstacleMethod;  % multiple_spheres/ellipsoids
checkCollisions = true;
% Warning threshold only; NOT a safety guarantee or future error bound.
trackingWarningDistance_m = 0.30;
showComparisonPlot = false;  % true: figure appears; never saved

% The following are actual SIMULATION setup values, NOT information
% disclosed to the online estimator / online dynamic prediction.
% The drone's other physical parameters are read from its real parameter
% object (DRONE_PARAM_SUSPENDED_LOAD), not retyped in this test.
physicalSetup = struct('plantLoadMass_kg',0.14, ...
    'ekfInitialLoadMass_kg',0.01,'cableLength_m',2.0);

% TEST-ONLY: adaptive preview sampling near obstacles. The existing
% wp_preview_4s.m and future CBF/replanner code are NOT modified.
% Keep a coarse grid across the entire horizon; its step is automatically
% selected from nominal speed, travel-per-sample, and horizon divisibility.
% Extra points are assigned per ORIGINAL OBSTACLE, not per approximation part.
adaptiveSamplingOn = true;
adaptiveOpts=sampleDefaults;
assert(adaptiveOpts.fineStep_s>=plantDt, ...
    'For exact offline plant-grid comparison, fineStep_s >= 0.025s');
% The caps are hard upper limits; they are NOT automatically raised
% for longer horizons. Baseline point count grows with the horizon.
forecastDt = choose_forecast_dt_onefile( ...
    lookAhead,0.50,plantDt,adaptiveOpts); % overwritten per route

assert(abs(round(forecastDt/plantDt)*plantDt-forecastDt)<1e-10);
test_inter_sample_guard_onefile();
% No whole-flight trajectory preview before takeoff. The deterministic
% geometry self-test is available as a local helper but is NOT run here.
test_rotated_ellipsoid_fixture_onefile();
test_c_sphere_gate_enclosure_onefile(adaptiveOpts,plantDt);
assert(all(checkTimes>=5 & checkTimes+lookAhead<flightEndTime+1e-9), ...
    'All t0+lookAhead must remain in the flight interval');
assert(all(abs(checkTimes/plantDt-round(checkTimes/plantDt))<1e-8), ...
    'checkTimes must align with the actual simulation time grid');

cfg=wp_system_geometry_config(geometryMethod, ...
    struct('cableLength',physicalSetup.cableLength_m));
ob=struct('maxPartsPerObstacle',6,'maxTotalParts',36, ...
          'targetPartSize',3,'polySides',8,'hollowSides',16);
% A always needs this world to evaluate its built-in 4s lookahead.
world=DRONE_OBSTACLES(sceneName,obstacleMethod,'ground_z0',ob);
worldBenchmark=struct('naturalParts',numel(world.parts),'syntheticAdded',0);
if sampleDefaults.stressTo36Parts
    [world,worldBenchmark]=stress_to_36_parts_onefile(world);
end
assert(numel(world.parts)<=36,'Obstacle approximation exceeded 36 parts');

fprintf('\n=============================================================\n');
fprintf('A (flatness V3) vs B (24-state plant), REAL HLC flight\n');
fprintf('C_closed = ORIGINAL HLC feedback-linearized error dynamics + live V3 governor\n');
fprintf('No future physical input held. B_replay uses NONCAUSAL controls for diagnosis only.\n');
fprintf('Horizon %.2fs / baseline dt=auto by route speed / full HLC dt %.3fs\n', ...
    lookAhead,plantDt);
fprintf('Prediction mass: latest EKF mL at t0 (true mass ONLY for ORACLE diagnostic)\n');
fprintf('Adaptive preview: %d (coarse grid retained, max +%d/object and +%d total)\n', ...
    adaptiveSamplingOn,adaptiveOpts.maxExtraPerObstacle, ...
    adaptiveOpts.maxExtraTotal);
fprintf('Geometry=%s scene=%s parts=%d/36 (synthetic stress added=%d)\n', ...
    geometryMethod,char(sceneName),numel(world.parts),worldBenchmark.syntheticAdded);
fprintf('Monitoring starts ONLY during flight, at the CURRENT time; no preflight full-trajectory prediction.\n');
fprintf('Monitor benchmark period %.0fms; the original HLC is 25ms.\n', ...
    sampleDefaults.monitorPeriod_s*1000);
fprintf('No MPC / No new controller / No auto save\n');
fprintf('=============================================================\n');

AB_Results=table();
Adaptive_Results=table();
Probe_Results=table();
Periodic_Results=table();
SphereGate_Results=table();
Rejoin_Results=table();
AB_Details=struct('route',{},'speed',{},'checkTime',{},'times',{}, ...
    'actual',{},'A',{},'C_closed',{},'B_replay_est',{},'B_replay_true',{}, ...
    'estimatedInitial',{},'trueInitial',{},'uAtStart',{}, ...
    'estimationErrorInitial',{},'cableConstraintErrorInitial',{}, ...
    'collisionActual',{},'collisionA',{},'collisionCClosed',{}, ...
    'collisionBReplayEst',{},'collisionBReplayTrue',{}, ...
    'nominalStartDeviation_m',{},'estimatedStartDroneError_m',{}, ...
    'estimatedStartLoadError_m',{},'estimatedStartDroneVelocityError_mps',{}, ...
    'estimatedStartLoadVelocityError_mps',{}, ...
    'monitorStatus',{},'currentPoseChecked',{}, ...
    'estimatedLoadMass_kg',{},'trueLoadMass_kg',{}, ...
    'A_adaptive',{},'adaptiveDiagnostics',{}, ...
    'probeDiagnostics',{},'C_sphereGate',{});

for runIdx=1:size(routeCases,1)
    routeName=routeCases{runIdx,1};
    speed=routeCases{runIdx,2};
    fprintf('\n========== RUN %d/%d  %s  speed=%.2f m/s ==========\n', ...
        runIdx,size(routeCases,1),routeName,speed);
    forecastDt=choose_forecast_dt_onefile( ...
        lookAhead,speed,plantDt,adaptiveOpts);
    fprintf('  Auto sample: horizon=%.2fs, baseline dt=%.3fs, base points=%d, desired fine<=%.3fs\n', ...
        lookAhead,forecastDt,round(lookAhead/forecastDt)+1, ...
        adaptiveOpts.fineStep_s);
    [logData, snaps]=run_original_hlc_test( ...
        routeName,speed,plantDt,flightEndTime,checkTimes, ...
        lookAhead,forecastDt,cfg,world,trackingWarningDistance_m, ...
        physicalSetup,adaptiveSamplingOn,adaptiveOpts);
    fprintf('  Original HLC executed: %d flight-log samples; snapshots=%d\n', ...
        numel(logData.times),numel(snaps));
    if ~isempty(logData.periodicMonitor)
        q=logData.periodicMonitor;
        pm=table(repmat(string(routeName),numel(q),1), ...
            repmat(string(geometryMethod),numel(q),1), ...
            [q.time_s]',[q.wall_s]'*1000, ...
            repmat(sampleDefaults.monitorPeriod_s*1000,numel(q),1), ...
            [q.overrun]',[q.totalSamples]',[q.parts]', ...
            'VariableNames',{'Route','GeometryMethod','Time_s', ...
            'Wall_ms','Period_ms','Overrun','TotalSamples','ObstacleParts'});
        if isempty(Periodic_Results),Periodic_Results=pm;
        else,Periodic_Results=[Periodic_Results;pm];end %#ok<AGROW>
        fprintf('  PERIODIC MONITOR %.0fms (single-thread simulation): ticks=%d median=%.1fms worst=%.1fms overruns=%d\n', ...
            sampleDefaults.monitorPeriod_s*1000,numel(q),median(pm.Wall_ms), ...
            max(pm.Wall_ms),sum(pm.Overrun));
    end

    for si=1:numel(snaps)
        s=snaps(si);
        sampleStride=round(forecastDt/plantDt);
        futureIndices=s.index+(0:round(lookAhead/forecastDt))*sampleStride;
        assert(futureIndices(end)<=numel(logData.times), ...
            'Not enough subsequent actual flight data');
        timeVec=logData.times(futureIndices);
        actual=logData.xTrue(:,futureIndices);
        assert(numel(timeVec)==numel(s.A.times), 'A/actual sample mismatch');
        assert(max(abs(timeVec(:)-s.A.times(:)))<1e-7, ...
            'Live V3 reference and actual plant timeline do not match');
        % Apply u(t_k),...,u(t_{k+n-1}) to propagate x(t_k)->x(t_{k+n}).
        inputSequence=logData.u(:,s.index:futureIndices(end)-1);
        nSteps=size(inputSequence,2);
        assert(nSteps==(numel(timeVec)-1)*sampleStride);
        dyn=logData.plantDynamics;
        parTrue=logData.plantParameters; % simulation truth; ORACLE ONLY
        parEstimated=s.predictionParameters; % CURRENT EKF mass (no future truth)
        xE=s.xEst(:);xT=s.xTrue(:);
        assert(numel(xE)==24 && numel(xT)==24, ...
            'Expected 24-state plant model for A/B experiment');

        % CAUSAL HLC: propagate the original feedback-linearized TRACKING
        % ERROR channels and the actual V3 governor feedback law.
        % Do NOT assume the plant input remains constant.
        Cclosed=hlc_error_closed_loop_preview_onefile( ...
            s,cfg,plantDt,sampleStride,numel(timeVec));
        wallClosed=Cclosed.wallSeconds;
        % C is the CURRENT-STATE closed-loop forecast. Gate its actual
        % predicted geometry; NEVER screen it using the nominal A route.
        Cgate=c_sphere_gate_monitor_onefile( ...
            s,Cclosed,cfg,world,adaptiveOpts,plantDt,forecastDt);
        fprintf(['   C SPHERE-FIRST: candidate intervals=%d/%d, ', ...
            'extra=%d, narrow=%d, earlyExit=%d, ', ...
            'broad=%.2fms narrow/refine=%.2fms C+gate=%.2fms\n'], ...
            Cgate.candidateIntervals,Cgate.baseIntervals, ...
            Cgate.extraSamples,Cgate.exactChecks,Cgate.earlyExit, ...
            Cgate.broadWall_s*1e3,Cgate.narrowWall_s*1e3, ...
            (Cclosed.wallSeconds+Cgate.wallSeconds)*1e3);
        gr=table(string(routeName),string(geometryMethod),speed,s.t0, ...
            lookAhead,Cgate.baseSamples,Cgate.candidateIntervals, ...
            Cgate.candidateObjects,Cgate.extraSamples,Cgate.exactChecks, ...
            Cgate.earlyExit,Cgate.unrefinedIntervals, ...
            Cgate.firstHitTime,Cgate.broadWall_s*1000, ...
            Cgate.narrowWall_s*1000,Cgate.wallSeconds*1000, ...
            Cclosed.wallSeconds*1000, ...
            (Cclosed.wallSeconds+Cgate.wallSeconds)*1000, ...
            'VariableNames',{'Route','GeometryMethod','Speed_mps', ...
            't0_s','Horizon_s','BaseNodes','CandidateIntervals', ...
            'CandidateObjects','AddedNodes','ExactCollisionChecks', ...
            'EarlyExit','UnrefinedIntervals','FirstHitTime_s', ...
            'BroadPhase_ms','NarrowPhase_ms','GateTotal_ms', ...
            'PredictionC_ms','CplusGate_ms'});
        if isempty(SphereGate_Results),SphereGate_Results=gr;
        else,SphereGate_Results=[SphereGate_Results;gr];end %#ok<AGROW>

        % B REPLAY EST: FUTURE true controls provided AFTER the flight.
        % It isolates input prediction / initial state estimation errors.
        tB=tic;
        BReplayEst=integrate_existing_dynamics(dyn,parEstimated,xE, ...
            inputSequence,sampleStride,plantDt);
        wallReplay=toc(tB);

        % B REPLAY TRUE: privileged truth at t0 + future controls.
        % A model-consistency check ONLY; no causal prediction capability.
        tB=tic;
        BReplayTrue=integrate_existing_dynamics(dyn,parTrue,xT, ...
            inputSequence,sampleStride,plantDt);
        wallTrue=toc(tB);

        % Compare positions of BOTH aircraft/load, and cable angle.
        % PredictionWall includes geometry checks for every method when ON.
        predictionNames={'A_flatness','C_HLC_closed_loop', ...
            'B_replay_est_NONCAUSAL','B_replay_true_ORACLE'};
        predPDrone={s.A.pDrone, Cclosed.pDrone, ...
            BReplayEst(1:3,:),BReplayTrue(1:3,:)};
        predPLoad={s.A.pLoad, Cclosed.pLoad, ...
            BReplayEst(13:15,:),BReplayTrue(13:15,:)};
        seconds=[s.A.totalElapsedSeconds,wallClosed,wallReplay,wallTrue];
        if checkCollisions
            truthHits=collisions_from_positions(cfg,world,timeVec, ...
                actual(1:3,:),actual(13:15,:));
            predHits=cell(1,4);predHits{1}=double(s.A.collisionSamples(:)');
            predHits{1}(~s.A.validSamples(:)')=NaN;
            % A's first sample merges the CURRENT EKF collision. Separate
            % that from the nominal future reference for fair A/actual.
            if s.A.validSamples(1)
                [nom0,~]=initial_collision_pair_onefile(cfg,world, ...
                    s.A.pDrone(:,1),s.A.pLoad(:,1), ...
                    s.xEst(1:3),s.xEst(13:15),s.t0);
                predHits{1}(1)=double(nom0);
            end
            % A necessary (and testable) broad-phase property: at every
            % sampled C node, any genuine fine-shape hit must have been
            % selected by the exact-geometry enclosing spheres.
            for pi=2:4
                tCollision=tic;
                predHits{pi}=collisions_from_positions(cfg,world,timeVec, ...
                    predPDrone{pi},predPLoad{pi});
                seconds(pi)=seconds(pi)+toc(tCollision);
            end
        else
            truthHits=nan(1,numel(timeVec));
            predHits=repmat({nan(1,numel(timeVec))},1,4);
        end

        if checkCollisions && strcmp(Cclosed.status,'calculated')
            cTrue=(predHits{2}>0);
            assert(~any(cTrue(:) & ~Cgate.baseCandidateNodes(:)), ...
                'Sphere broad phase missed an exact collision at a C baseline node');
            assert(~any(cTrue(:) & ~Cgate.baseHits(:)), ...
                'Sphere-gated C collision failed to detect baseline contact');
        end
        for j=1:4
            [md,ml,ca,rmseL,good]=position_errors( ...
                predPDrone{j},predPLoad{j},actual(1:3,:),actual(13:15,:));
            common=isfinite(truthHits)&isfinite(predHits{j});
            miss=sum(truthHits(common)==1 & predHits{j}(common)==0);
            extra=sum(truthHits(common)==0 & predHits{j}(common)==1);
            if ~checkCollisions,miss=NaN;extra=NaN;end
            invalidGeom=sum(~common);
            if ~checkCollisions,invalidGeom=NaN;end
            massByPredictor=[NaN,s.mLoadEstimated_kg, ...
                s.mLoadEstimated_kg,s.mLoadTrue_kg];
            r=table(string(routeName),speed,s.t0,string(predictionNames{j}), ...
                md,ml,rmseL,ca,miss,extra,invalidGeom,seconds(j),good, ...
                massByPredictor(j), ...
                'VariableNames',{'Route','Speed_mps','t0_s','Predictor', ...
                'MaxDroneError_m','MaxLoadError_m','RMSLoadError_m', ...
                'MaxCableAngle_deg','MissedSamples','ExtraWarnings', ...
                'InvalidCollisionSamples', ...
                'PredictionWall_s','PredictionValid','ModelLoadMass_kg'});
            if isempty(AB_Results)
                AB_Results=r;
            else
                AB_Results=[AB_Results;r]; %#ok<AGROW>
            end
        end

        fprintf(' t0=%5.2f: Max load error [m]: A=%7.3f, C-HLC=%7.3f,', ...
            s.t0,AB_Results.MaxLoadError_m(end-3), ...
            AB_Results.MaxLoadError_m(end-2));
        fprintf(' B-Replay-Est=%7.3f, B-Replay-True=%7.3f\n', ...
            AB_Results.MaxLoadError_m(end-1),AB_Results.MaxLoadError_m(end));
        fprintf('   C-HLC: status=%s, current pose mismatch=%.4fm, current input saturation=%.4g, wall=%.2fms\n', ...
            Cclosed.status,Cclosed.startPoseDiscrepancy_m, ...
            Cclosed.inputSaturation,Cclosed.wallSeconds*1e3);
        if ~isempty(Cclosed.reason)
            fprintf('   C-HLC diagnostic: %s\n',Cclosed.reason);
        end
        if all(isfinite(Cclosed.refLoad(:)))
            actualRef=logData.refHLC(1:3,futureIndices);
            refDiff=Cclosed.refLoad-actualRef;
            fprintf('   C-HLC V3 future-reference RMS difference [offline diagnostic]: %.4fm\n', ...
                sqrt(mean(sum(refDiff.^2,1))));
        end
        if all(isfinite(Cclosed.pLoad(:)))
            targetH=[0.5,1.0,2.0,4.0];
            subA=nan(1,numel(targetH));subC=subA;
            for hi=1:numel(targetH)
                if targetH(hi)>lookAhead,continue;end
                [~,kk]=min(abs(timeVec-(s.t0+targetH(hi))));
                subA(hi)=norm(s.A.pLoad(:,kk)-actual(13:15,kk));
                subC(hi)=norm(Cclosed.pLoad(:,kk)-actual(13:15,kk));
            end
            fprintf('   Load error A vs C at +0.5/1/2/4s [m, nearest sample]:\n');
            fprintf('     A: %7.3f %7.3f %7.3f %7.3f\n',subA);
            fprintf('     C: %7.3f %7.3f %7.3f %7.3f\n',subC);
        end
        fprintf(['   INITIAL EKF vs plant [m]: drone=%.4f load=%.4f', ...
            ' ; velocity [m/s]: drone=%.4f load=%.4f\n'], ...
            norm(xE(1:3)-xT(1:3)),norm(xE(13:15)-xT(13:15)), ...
            norm(xE(7:9)-xT(7:9)),norm(xE(16:18)-xT(16:18)));
        fprintf('   DYNAMICS MASS: EKF/current-prediction=%.5f kg; true-plant=%.5f kg [offline only]\n', ...
            s.mLoadEstimated_kg,s.mLoadTrue_kg);
        fprintf('   LIVE CURRENT POSE: checked=%d deviation=%.4fm status=%s\n', ...
            s.A.currentPoseChecked,s.A.trackingDeviation,char(s.A.status));
        if isfinite(s.A.firstHitTime)
            fprintf('   A first merged collision (nominal OR current): %.3fs (+%.3fs) / part=%s / obstacle=%s\n', ...
                s.A.firstHitTime,s.A.firstHitTime-s.t0, ...
                char(string(s.A.firstHitPart)),char(string(s.A.firstHitObstacle)));
        end
        if adaptiveSamplingOn
            ad=s.A_adaptive;
            [adIds,adTruth]=truth_at_adaptive_times(logData,ad.times,plantDt);
            adHits=nan(size(ad.times));
            adHits(ad.validSamples)=double(ad.collisionSamples(ad.validSamples));
            adCommon=isfinite(adHits)&isfinite(adTruth);
            adMiss=sum(adTruth(adCommon)==1 & adHits(adCommon)==0);
            adExtra=sum(adTruth(adCommon)==0 & adHits(adCommon)==1);
            [nominalInitialHit,actualInitialHit]=initial_collision_pair_onefile( ...
                cfg,world,s.A.pDrone(:,1),s.A.pLoad(:,1), ...
                xE(1:3),xE(13:15),s.t0);
            assert(Cgate.currentChecked && ...
                Cgate.currentHit==logical(actualInitialHit), ...
                'C sphere-first monitor missed the CURRENT EKF collision');
            fprintf('   COLLISION ORIGIN: nominal(t0)=%d, current-EKF(t0)=%d (A merges these in sample 1)\n', ...
                nominalInitialHit,actualInitialHit);
            if isfinite(ad.firstHitTime)
                fprintf('   A nominal sampled collision: t=%.3fs (in %.3fs)\n', ...
                    ad.firstHitTime,ad.firstHitTime-s.t0);
            else
                fprintf('   A nominal sampled collision: none in current horizon\n');
            end
            certTic=tic;
            intervalCert=interval_certificate_onefile(cfg,world,ad,adaptiveOpts);
            certWallSeconds=toc(certTic);
            fprintf('   ROUTE-SET CHECK: completely-excluded=%d contact-at-node=%d unresolved=%d (total=%d); %s\n', ...
                intervalCert.certified,intervalCert.nodeContact, ...
                intervalCert.unresolved,intervalCert.total,intervalCert.reason);
            diag=struct('base',numel(s.A.times),'extra',numel(ad.times)-numel(s.A.times), ...
                'total',numel(ad.times),'budgetUnresolved',ad.budgetUnresolved, ...
                'perObstacleNames',{ad.obstacleNames}, ...
                'perObstacleAdded',ad.addedByObstacle, ...
                'hitFirst',ad.firstHitTime,'missed',adMiss, ...
                'falseAlarmSamples',adExtra, ...
                'baseWallSeconds',s.A.totalElapsedSeconds, ...
                'refineWallSeconds',ad.totalElapsedSeconds, ...
                'wallSeconds',s.A.totalElapsedSeconds+ad.totalElapsedSeconds+certWallSeconds, ...
                'certificateWallSeconds',certWallSeconds, ...
                'actualIndices',adIds,'intervalCertificate',intervalCert, ...
                'nominalInitialCollision',nominalInitialHit, ...
                'currentEKFCollision',actualInitialHit, ...
                'wholeRouteScreen',ad.globalScreen);
            dense=struct('denseHits',NaN,'missedDenseHits',NaN, ...
                'unsampledHitCount',NaN);
            if adaptiveOpts.denseTruthCheck
                dense=truth_dense_diagnostic_onefile( ...
                    logData,s.t0,lookAhead,ad.times,adTruth,plantDt);
                diag.denseTruth=dense;
                fprintf('   OFFLINE actual @25ms: hits=%d, unsampled-hit-instances=%d, entirely hidden collisions=%d\n', ...
                    dense.denseHits,dense.unsampledHitCount,dense.missedDenseHits);
            end
            fprintf('   GLOBAL GEOMETRY: proved-excluded static parts=%d/%d, ground-excluded=%d, scope=%s\n', ...
                ad.globalScreen.excludedParts,ad.globalScreen.originalParts, ...
                ad.globalScreen.groundExcluded,ad.globalScreen.source);
            fprintf('   ADAPTIVE: base=%d + extra=%d -> total=%d; unresolved-near-intervals=%d; sampled-misses=%d; total-wall=%.4fs\n', ...
                diag.base,diag.extra,diag.total,diag.budgetUnresolved, ...
                diag.missed,diag.wallSeconds);
            if si==1
                fprintf('   Per-obstacle extra-node allocation (cap=%d/object):\n', ...
                    adaptiveOpts.maxExtraPerObstacle);
                for jj=1:numel(ad.obstacleNames)
                    fprintf('     %-24s  %2d\n',ad.obstacleNames{jj},ad.addedByObstacle(jj));
                end
            end
            ar=table(string(routeName),speed,s.t0,lookAhead,forecastDt, ...
                diag.base,diag.extra,diag.total,diag.budgetUnresolved, ...
                intervalCert.certified,intervalCert.unresolved, ...
                dense.unsampledHitCount,dense.missedDenseHits, ...
                diag.missed,diag.falseAlarmSamples, ...
                ad.globalScreen.excludedParts,ad.globalScreen.originalParts, ...
                ad.globalScreen.groundExcluded, ...
                s.A.totalElapsedSeconds,ad.totalElapsedSeconds,diag.wallSeconds, ...
                'VariableNames',{'Route','Speed_mps','t0_s','Horizon_s', ...
                'BaseDt_s','BaseSamples','ExtraSamples','TotalSamples', ...
                'UnrefinedCandidateIntervals','GloballyExcludedIntervals', ...
                'UnresolvedIntervals','TruthUnsampledHitInstances', ...
                'TruthEntirelyHiddenHits','MissedSampledCollisions', ...
                'ExtraSampledWarnings','FarExcludedParts','OriginalParts', ...
                'GroundGloballyExcluded','BasePredictionWall_s', ...
                'AdaptiveOnlyWall_s','PredictionWall_s'});
            if isempty(Adaptive_Results),Adaptive_Results=ar;
            else,Adaptive_Results=[Adaptive_Results;ar];end %#ok<AGROW>
        else
            diag=struct();
        end
        % Separate synthetic geometry-only test fixture: an obstacle is
        % placed AFTER finding the nominal reference. This is an offline
        % TEST of time sampling, not an obstacle detected by the controller.
        probeDiag=struct();
        if isfield(s,'probe') && ~isempty(s.probe)
            probeDiag=evaluate_probe_onefile(s.probe,logData,plantDt);
            % Run the SAME C-sphere gate on the synthetic sphere or
            % rotated ellipsoid. This is not the A nominal trajectory.
            probeGate=c_sphere_gate_monitor_onefile( ...
                s,Cclosed,cfg,s.probe.world,adaptiveOpts,plantDt,forecastDt);
            probeDiag.C_sphereGate=probeGate;
            fprintf('   C PROBE SPHERE-FIRST (%s): candidate=%d/%d extra=%d earlyExit=%d wall=%.2fms\n', ...
                s.probe.shape,probeGate.candidateIntervals, ...
                probeGate.baseIntervals,probeGate.extraSamples, ...
                probeGate.earlyExit,probeGate.wallSeconds*1000);
            if strcmp(Cclosed.status,'calculated')
                truthC=collisions_from_positions( ...
                    cfg,s.probe.world,timeVec,Cclosed.pDrone,Cclosed.pLoad);
                assert(~any(truthC(:)>0 & ~probeGate.baseCandidateNodes(:)), ...
                    'C sphere screening missed a probe collision at baseline node');
                assert(~any(truthC(:)>0 & ~probeGate.baseHits(:)), ...
                    'C sphere gated narrow phase missed a probe collision');
            end
            fprintf('   PROBE(%s/%s): baselineHit=%d adaptiveHit=%d denseNomHit=%d densePlantHit=%d hiddenBaseline=%d hiddenAdaptive=%d extra=%d unresolved=%d screenExclusion=%d wall=%.4fs\n', ...
                s.probe.mode,s.probe.shape,probeDiag.baseAny,probeDiag.adaptiveAny, ...
                probeDiag.denseNominalHits,probeDiag.densePlantHits, ...
                probeDiag.hiddenBaselineHits,probeDiag.hiddenNominalHits,probeDiag.extraPoints, ...
                probeDiag.budgetUnresolved,probeDiag.excludedParts, ...
                probeDiag.wallSeconds);
            if strcmpi(s.probe.mode,'grazing') && ~probeDiag.grazingFixtureValid
                fprintf('   PROBE NOTE: midpoint-only grazing placement was not achieved; interpret sampling detection descriptively.\n');
            end
            pr=table(string(routeName),speed,s.t0,string(s.probe.mode),string(s.probe.shape), ...
                probeDiag.baseAny,probeDiag.adaptiveAny,probeDiag.denseNominalHits, ...
                probeDiag.densePlantHits,probeDiag.hiddenBaselineHits, ...
                probeDiag.hiddenNominalHits, ...
                probeDiag.extraPoints,probeDiag.budgetUnresolved, ...
                probeDiag.wallSeconds,probeDiag.grazingFixtureValid, ...
                'VariableNames',{'Route','Speed_mps','t0_s','ProbeType','ProbeShape', ...
                'BaselineAnyHit','AdaptiveAnyHit','DenseNominalHits', ...
                'DensePlantHits','HiddenBaselineHitInstances', ...
                'HiddenAdaptiveHitInstances','AddedNodes', ...
                'UnrefinedNearIntervals','PredictionWall_s', ...
                'GrazingFixtureValid'});
            if isempty(Probe_Results),Probe_Results=pr;
            else,Probe_Results=[Probe_Results;pr];end %#ok<AGROW>
        end
        if AB_Results.MaxLoadError_m(end)>0.15
            warning('ABPreview:ModelReplayMismatch', ...
                ['True-state input replay differs from actual plant by >0.15 m. ', ...
                 'Check time alignment, integrator and model constraints.']);
        end
        if isfinite(adaptiveOpts.rejoinTime_s)
            tDiag=s.t0+[0 0.5 1 2 4];
            tDiag=tDiag(tDiag<=s.t0+lookAhead+1e-9);
            for ij=1:numel(tDiag)
                [~,ik]=min(abs(data.times-tDiag(ij)));
                [~,ic]=min(abs(timeVec-tDiag(ij)));
                devTrue=norm(data.xTrue(13:15,ik)-data.refHLC(1:3,ik));
                devPred=norm(Cclosed.pLoad(:,ic)-Cclosed.refLoad(:,ic));
                rr=table(string(routeName),string(geometryMethod), ...
                    s.t0,tDiag(ij)-s.t0,devTrue,devPred, ...
                    data.hlcSaturation(ik), ...
                    'VariableNames',{'Route','GeometryMethod','t0_s', ...
                    'Ahead_s','TrueTrackingError_m', ...
                    'PredictedTrackingError_m','ActualHLCSaturation'});
                if isempty(Rejoin_Results),Rejoin_Results=rr;
                else,Rejoin_Results=[Rejoin_Results;rr];end %#ok<AGROW>
            end
            fprintf('   REJOIN: actual/predicted load-to-HLC-reference deviation [m]\n');
            fprintf('     Ahead(s):');fprintf('  %.2g',tDiag-s.t0);fprintf('\n');
            block=Rejoin_Results(Rejoin_Results.t0_s==s.t0 & ...
                Rejoin_Results.Route==string(routeName),:);
            fprintf('     Actual :');fprintf('  %.3f',block.TrueTrackingError_m);fprintf('\n');
            fprintf('     C-HLC  :');fprintf('  %.3f',block.PredictedTrackingError_m);fprintf('\n');
        end
        d=struct('route',routeName,'speed',speed,'checkTime',s.t0, ...
            'times',timeVec,'actual',actual,'A',s.A,'C_closed',Cclosed, ...
            'B_replay_est',BReplayEst,'B_replay_true',BReplayTrue, ...
            'estimatedInitial',xE,'trueInitial',xT,'uAtStart',s.uNow, ...
            'estimationErrorInitial',norm(xE(13:15)-xT(13:15)), ...
            'cableConstraintErrorInitial',norm(xE(13:15)-xE(1:3)-cfg.cableLength*xE(19:21)), ...
            'collisionActual',truthHits,'collisionA',predHits{1}, ...
            'collisionCClosed',predHits{2},'collisionBReplayEst',predHits{3}, ...
            'collisionBReplayTrue',predHits{4}, ...
            'nominalStartDeviation_m',s.A.trackingDeviation, ...
            'estimatedStartDroneError_m',norm(xE(1:3)-xT(1:3)), ...
            'estimatedStartLoadError_m',norm(xE(13:15)-xT(13:15)), ...
            'estimatedStartDroneVelocityError_mps',norm(xE(7:9)-xT(7:9)), ...
            'estimatedStartLoadVelocityError_mps',norm(xE(16:18)-xT(16:18)), ...
            'monitorStatus',char(s.A.status), ...
            'currentPoseChecked',s.A.currentPoseChecked, ...
            'estimatedLoadMass_kg',s.mLoadEstimated_kg, ...
            'trueLoadMass_kg',s.mLoadTrue_kg, ...
            'A_adaptive',s.A_adaptive,'adaptiveDiagnostics',diag, ...
            'probeDiagnostics',probeDiag,'C_sphereGate',Cgate);
        AB_Details(end+1)=d; %#ok<AGROW>
        if showComparisonPlot && runIdx==1 && si==1
            show_error_comparison(d);
        end
    end
end

% Output available in MATLAB workspace without writing any new file.
assignin('base','WP_AB_Results',AB_Results);
assignin('base','WP_AB_Details',AB_Details);
assignin('base','WP_Adaptive_Results',Adaptive_Results);
assignin('base','WP_Probe_Results',Probe_Results);
assignin('base','WP_MonitorCycle_Results',Periodic_Results);
assignin('base','WP_C_SphereGate_Results',SphereGate_Results);
assignin('base','WP_Rejoin_Results',Rejoin_Results);
fprintf('\n=== A/B COMPARISON FINISHED (%d rows / %d snapshots) ===\n', ...
    height(AB_Results),numel(AB_Details));
disp(AB_Results(:,{'Route','Speed_mps','t0_s','Predictor', ...
    'MaxLoadError_m','MaxCableAngle_deg','MissedSamples', ...
    'InvalidCollisionSamples','PredictionWall_s','ModelLoadMass_kg'}));
if adaptiveSamplingOn
    fprintf('\n=== ADAPTIVE SAMPLING EXPERIMENT (NOT A SAFETY CERTIFICATE) ===\n');
    disp(Adaptive_Results);
    if ~isempty(Probe_Results)
        fprintf('\n=== OFFLINE SYNTHETIC PROBE (INTER-SAMPLE DETECTION) ===\n');
        disp(Probe_Results);
    end
end
fprintf('\n=== CURRENT-STATE MONITOR SNAPSHOT DIAGNOSTICS ===\n');
for kk=1:numel(AB_Details)
    q=AB_Details(kk);
    fprintf('%-20s t=%.2f  currentCheck=%d  nominalDev=%.3fm  EKF loadErr=%.3fm  EKF vLoadErr=%.3fm/s  status=%s\n', ...
        q.route,q.checkTime,q.currentPoseChecked,q.nominalStartDeviation_m, ...
        q.estimatedStartLoadError_m,q.estimatedStartLoadVelocityError_mps, ...
        q.monitorStatus);
end
if ~isempty(Periodic_Results)
    fprintf('\n=== PERIODIC MONITOR BENCHMARK (flight only; separate from 25ms HLC schedule) ===\n');
    disp(Periodic_Results);
end
fprintf('The tracking threshold is ONLY a warning, never a certified bound.\n');
fprintf(['\nInterpretation: C-HLC uses original closed-loop virtual error channels and updated V3 governor.\n', ...
    'B-Replay-Est and B-Replay-True used FUTURE measured control inputs: OFFLINE ONLY.\n', ...
    'Static far-obstacle exclusion uses route-wide mathematical enclosures; no speed bound.\n', ...
    'Nearby time intervals are sampled, NOT continuously certified.\n', ...
    'A 4s prediction validity cannot be inferred just from MATLAB execution.\n']);
end

%% Original HLC simulation. This is deliberately inside the SAME file.
function [data,snapshots]=run_original_hlc_test( ...
    routeName,speed,dt,te,checkTimes,horizon,sampleDt,cfg,world,trackingWarningDistance_m, ...
    physicalSetup,adaptiveSamplingOn,adaptiveOpts)
ts=0;
time=TIME(ts,dt,te);
N=numel(ts:dt:te);
logger=LOGGER(1,N,0,[],[]);
logger.display_on=false;
logger.display_func=@(~,~) [];

agent=DRONE;
agent.parameter=DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
agent.parameter.set("loadmass",physicalSetup.plantLoadMass_kg);
agent.parameter.set("cableL",physicalSetup.cableLength_m);
initial=struct('q',[0;0;0],'w',[0;0;0],'vL',[0;0;0], ...
    'p',[0;0;0],'pT',[0;0;-1],'wL',[0;0;0],'v',[0;0;0]);
initial.pL=initial.p+agent.parameter.cableL*initial.pT;
agent.plant=MODEL_CLASS(agent,Model_Suspended_Load(dt,initial,1,agent));
% Preserve original known loadmass-parameter construction order.
agent.parameter.set("loadmass",physicalSetup.ekfInitialLoadMass_kg);

motive=Connector_Natnet_sim(dt,{{1,"p","q"},{1,"pL","pT"}});
motive.getData(agent);
agent.sensor.set_function_class('motive',MOTIVE(agent,motive, ...
    'output_func',@motive_output_onefile,'rigid_id',[1,2], ...
    'state_list',{["p","q"],"p"}));
agent.estimator.set_function_class('ekf',EKF(agent, ...
    Estimator_EKF_SuspendedLoad(agent,dt, ...
    MODEL_CLASS(agent,Model_Suspended_Load(dt,initial,1,agent,'Load_mL_HL')), ...
    ["p","q","pL","pT"])));
agent.estimator.set_function_class('loadstate',SUSPENDED_LOAD_STATE_MANAGER(agent));

L=agent.parameter.cableL;
params={"speed",speed};
agent.reference.set_function_class('waypoint', ...
    WAYPOINT_REFERENCE(agent,{char(routeName),params,6}));
agent.reference.set_function_class('sload',SUSPENDED_LOAD_REF_ADJUST(agent));
agent.reference.set_function_class('takeoff',TAKEOFF_REFERENCE(agent,'zd',3.0,'te',5));
agent.reference.set_function_class('landing',LANDING_REFERENCE(agent,'dt',dt,'zd',-L,'te',3));
agent.controller.set_function_class('hlc_suspended', ...
    HLC_SUSPENDED_LOAD(agent,Controller_HL_Suspended_Load(dt,agent)));
agent.set_cha_allocation_for_all("sensor","motive");
agent.set_cha_allocation_for_all("estimator",["ekf","loadstate"]);
agent.cha_allocation.a.reference=["takeoff","sload"];
agent.cha_allocation.t.reference=["takeoff","sload"];
agent.cha_allocation.f.reference=["waypoint","sload"];
agent.cha_allocation.l.reference=["landing","sload"];
env=[];

for chaInit=['a','t','f','l']
    agent.cha=chaInit;
    dispatch_onefile(time,chaInit,'0',logger,env,agent,'sensor',false);
    dispatch_onefile(time,chaInit,'0',logger,env,agent,'estimator',false);
    dispatch_onefile(time,chaInit,'0',logger,env,agent,'reference',false);
    dispatch_onefile(time,chaInit,'0',logger,env,agent,'controller',false);
    dispatch_onefile(time,chaInit,'0',logger,env,agent,'input_transform',false);
end

data.geometryConfig=cfg;
data.world=world;
data.times=(ts:dt:te);
data.xTrue=nan(24,N);
data.u=nan(4,N);
data.refHLC=nan(28,N);  % actual future reference for OFFLINE diagnosis only
data.hlcSaturation=nan(1,N);  % measured original HLC saturation
data.plantDynamics=agent.plant.method;
data.periodicMonitor=struct('time_s',{},'wall_s',{},'overrun',{},'totalSamples',{},'parts',{});
data.plantParameters=agent.plant.param;
data.parameterNames=string(agent.parameter.parameter_order);
data.loadMassIndex=find(data.parameterNames=="loadmass");
assert(isscalar(data.loadMassIndex), ...
    'Cannot resolve loadmass index from DRONE_PARAM_SUSPENDED_LOAD.parameter_order');
assert(abs(data.plantParameters(data.loadMassIndex)-physicalSetup.plantLoadMass_kg)<1e-12, ...
    'True plant parameter must preserve the construction-time loadmass');
assert(abs(cfg.cableLength-agent.parameter.cableL)<1e-12, ...
    'System safety geometry must use CURRENT physical cable length');
snapshots=struct('index',{},'t0',{},'xEst',{},'xTrue',{},'uNow',{},'A',{}, ...
    'hlcSnapshot',{}, ...
    'predictionParameters',{},'mLoadEstimated_kg',{},'mLoadTrue_kg',{}, ...
    'A_adaptive',{},'probe',{});

for k=1:N
    time.t=data.times(k);
    if k==1, cha='a';
    elseif time.t<5,cha='t';
    elseif time.t<47,cha='f';
    else,cha='l';end
    agent.cha=cha;
    % OFFLINE REJOIN TEST ONLY: a geometrically consistent state reset.
    % Drone and suspended load are translated together; cable length and
    % cable orientation stay unchanged. This models an initial offset,
    % NOT a physically modeled gust or a new controller.
    if cha=='f' && isfinite(adaptiveOpts.rejoinTime_s) && ...
            abs(time.t-adaptiveOpts.rejoinTime_s)<dt/3
        xx=agent.plant.state.get();xx=xx(:);
        shift=adaptiveOpts.rejoinShift_m(:);
        dv=adaptiveOpts.rejoinVelocityShift_mps(:);
        xx(1:3)=xx(1:3)+shift;
        xx(13:15)=xx(13:15)+shift;
        xx(7:9)=xx(7:9)+dv;
        xx(16:18)=xx(16:18)+dv;
        agent.plant.set_state(xx);
        fprintf('  REJOIN STATE RESET at t=%.3f: shift=[%.2f %.2f %.2f]m; common dv=[%.2f %.2f %.2f]m/s (offline fixture)\n', ...
            time.t,shift,dv);
    end
    motive.getData(agent);
    ticAll=tic;
    dispatch_onefile(time,cha,cha,logger,env,agent,'sensor',true);
    dispatch_onefile(time,cha,cha,logger,env,agent,'estimator',true);
    dispatch_onefile(time,cha,cha,logger,env,agent,'reference',true);
    dispatch_onefile(time,cha,cha,logger,env,agent,'controller',true);
    dispatch_onefile(time,cha,cha,logger,env,agent,'input_transform',true);

    % CRITICAL time alignment: the plant state BEFORE plant.do is x(t_k).
    % Controller input u(t_k) acts over [t_k, t_k+dt]. The earlier test
    % accidentally stored x(t_k+dt) under t_k while using the EKF's x(t_k),
    % making the A/B comparison inconsistent by one integration step.
    xt=agent.plant.state.get();xt=xt(:);
    assert(numel(xt)==24 && all(isfinite(xt)), ...
        'Plant must provide 24 finite states; route=%s time=%.3f', ...
        routeName,time.t);
    data.xTrue(:,k)=xt;
    if cha=='f'
        data.refHLC(:,k)=agent.controller.result.xd(:);
        data.hlcSaturation(k)=norm( ...
            agent.controller.result.tmp(:)- ...
            agent.controller.result.input(:),inf);
    end
    uk=agent.controller.result.input;uk=uk(:);
    if numel(uk)==4 && all(isfinite(uk)),data.u(:,k)=uk;end

    isCheckpoint=cha=='f' && any(abs(time.t-checkTimes)<dt/3);
    isPeriodic=false;
    if cha=='f' && adaptiveOpts.runPeriodicBenchmark && ...
            time.t>=adaptiveOpts.monitorWindow_s(1)-dt/3 && ...
            time.t<=adaptiveOpts.monitorWindow_s(2)+dt/3 && ...
            time.t+horizon<=te+1e-9
        stepId=round((time.t-adaptiveOpts.monitorWindow_s(1))/dt);
        isPeriodic=abs(stepId*dt- ...
            (time.t-adaptiveOpts.monitorWindow_s(1)))<dt/3 && ...
            mod(stepId,round(adaptiveOpts.monitorPeriod_s/dt))==0;
    end
    if isCheckpoint||isPeriodic
        monitorWallClock=tic;
        es=agent.estimator.result.state;
        xe=estimator_to_24state(es);
        assert(all(isfinite(xe)), ...
            'Estimator state invalid at t=%.3f; do not substitute reference',time.t);
        assert(all(isfinite(data.u(:,k))), ...
            'HLC output invalid at t=%.3f',time.t);
        options=struct('horizon',horizon,'sampleDt',sampleDt, ...
            'predictionMode','flatness','flightStartTime',5);
        % IMPORTANT: use the CURRENT EKF geometry as an actual input.
        % This is a CURRENT collision/deviation check, not future propagation.
        options.currentPose=struct('pDrone',xe(1:3), ...
                                   'pLoad',xe(13:15));
        options.maxTrackingDeviation=trackingWarningDistance_m;
        % PREDICTION: copy the real model's full physical parameter vector,
        % then use the CURRENT EKF estimate for the unknown load mass.
        % NEVER take the plant's privileged true load mass here.
        if isstruct(es)
            assert(isfield(es,'mL'), ...
                'EKF must expose current mL estimate for online mass prediction');
        else
            assert(isprop(es,'mL'), ...
                'EKF must expose current mL estimate for online mass prediction');
        end
        mLHat=es.mL;
        assert(isnumeric(mLHat) && isscalar(mLHat) && ...
            isfinite(mLHat) && mLHat>=0, ...
            'EKF estimated mL must be a finite nonnegative scalar');
        predParams=agent.parameter.get("all","row");
        predParams(data.loadMassIndex)=mLHat;
        assert(all(isfinite(predParams)) && ...
            numel(predParams)==numel(data.plantParameters));
        % Snapshot the EXACT HLC virtual state and CURRENT V3 governor for
        % a lightweight closed-loop tracking-error forecast. Do not read
        % privileged plant truth or future HLC inputs.
        controllerObj=agent.controller.hlc_suspended;
        v3=agent.reference.waypoint;
        hState=[es.getq('compact');es.w;es.pL;es.vL;es.pT;es.wL];
        assert(numel(hState)==19 && all(isfinite(hState)));
        hParam=[agent.parameter.get(["mass","jx","jy","jz", ...
             "gravity","loadmass","cableL"]),0,0];
        hParam(6)=mLHat;
        hlcSnapshot=struct( ...
            'controller',controllerObj, ...
            'state19',hState(:),'xdNow',agent.controller.result.xd(:), ...
            'P',hParam,'F1',controllerObj.param.F1(:)', ...
            'F2',controllerObj.param.F2(:)', ...
            'F3',controllerObj.param.F3(:)', ...
            'inputSaturation',norm(agent.controller.result.tmp(:)- ...
                 agent.controller.result.input(:),inf), ...
            'generator',v3.generator, ...
            'virtualTime',v3.virtualTime,'rateState',v3.rateState(:), ...
            'heldRateCommand',v3.heldRateCommand, ...
            'filterTime',v3.filterTime, ...
            'slowdownStart',v3.slowdownStart, ...
            'slowdownFull',v3.slowdownFull,'minScale',v3.minScale);
        % Screen static far obstacles ON THIS FLIGHT TICK only. No preflight
        % evaluation or predicted entire HLC flight is ever run.
        % The periodic online timing path evaluates ONLY C+the sphere gate.
        % It never spends CPU on the A/B diagnostics or synthetic probes.
        if isPeriodic && ~isCheckpoint
            live=struct('t0',time.t,'xEst',xe,'hlcSnapshot',hlcSnapshot);
            Cp=hlc_error_closed_loop_preview_onefile( ...
                live,cfg,dt,round(sampleDt/dt),round(horizon/sampleDt)+1);
            Gp=c_sphere_gate_monitor_onefile( ...
                live,Cp,cfg,world,adaptiveOpts,dt,sampleDt);
            elapsed=Cp.wallSeconds+Gp.wallSeconds;
            data.periodicMonitor(end+1)=struct( ...
                'time_s',time.t,'wall_s',elapsed, ...
                'overrun',elapsed>adaptiveOpts.monitorPeriod_s, ...
                'totalSamples',Gp.baseSamples+Gp.extraSamples, ...
                'parts',numel(world.parts)); %#ok<AGROW>
        else
        [A,tightWorld,screen]=screened_nominal_preview_onefile( ...
            agent.reference.waypoint,world,cfg,time.t,options);
        if adaptiveSamplingOn
            A_adaptive=adaptive_preview_test_onefile( ...
                agent.reference.waypoint,world,cfg,A,options,adaptiveOpts, ...
                tightWorld,screen,true);
        else
            A_adaptive=struct();
        end
        if isPeriodic
            % Measure the proposed C+gate production path, excluding
            % A and offline diagnostics computed for this checkpoint.
            live=struct('t0',time.t,'xEst',xe,'hlcSnapshot',hlcSnapshot);
            Cp=hlc_error_closed_loop_preview_onefile( ...
                live,cfg,dt,round(sampleDt/dt),round(horizon/sampleDt)+1);
            Gp=c_sphere_gate_monitor_onefile( ...
                live,Cp,cfg,world,adaptiveOpts,dt,sampleDt);
            elapsed=Cp.wallSeconds+Gp.wallSeconds;
            data.periodicMonitor(end+1)=struct( ...
                'time_s',time.t,'wall_s',elapsed, ...
                'overrun',elapsed>adaptiveOpts.monitorPeriod_s, ...
                'totalSamples',Gp.baseSamples+Gp.extraSamples, ...
                'parts',numel(world.parts)); %#ok<AGROW>
        end
        if isCheckpoint
        % Regression of optimized path against the untouched collision
        % monitor once per FLIGHT, not on every online monitor tick.
        if adaptiveOpts.validateFastPath && ...
                abs(time.t-checkTimes(1))<dt/3
            legacy=wp_preview_4s(world,cfg,agent.reference.waypoint,time.t,options);
            assert(isequal(A.collisionSamples,legacy.collisionSamples), ...
                'Fast path changed nominal+current collision flags');
            assert(max(abs(A.pDrone(:)-legacy.pDrone(:)))<1e-7 && ...
                max(abs(A.pLoad(:)-legacy.pLoad(:)))<1e-7, ...
                'Fast path changed nominal reference trajectory');
            fprintf('  CACHE VALIDATION PASS: optimized preview preserves full-world collision flags.\n');
        end
        probe=[];
        if ~strcmpi(char(adaptiveOpts.probeMode),'none') && ...
                (~isfield(world,'dynamicIndices') || isempty(world.dynamicIndices))
            probe=make_and_run_nominal_probe_onefile( ...
                agent.reference.waypoint,world,cfg,A,options,adaptiveOpts,dt);
        end
        if A.validSamples(1)
            assert(A.currentPoseChecked, ...
                'Current EKF pose was NOT passed into preview at t=%.3f',time.t);
        end
        if ~all(A.validSamples)
            warning('ABPreview:NominalInvalid', ...
                'A invalid on %s t=%.3f: %s',routeName,time.t,A.invalidReason);
        end
        snapshots(end+1)=struct('index',k,'t0',time.t, ...
            'xEst',xe,'xTrue',xt,'uNow',data.u(:,k),'A',A, ...
            'hlcSnapshot',hlcSnapshot, ...
            'predictionParameters',predParams, ...
            'mLoadEstimated_kg',mLHat, ...
            'mLoadTrue_kg',data.plantParameters(data.loadMassIndex), ...
            'A_adaptive',A_adaptive,'probe',probe); %#ok<AGROW>
        end % checkpoint-only; periodic tick does not create an A/B snapshot
        end % checkpoint path; C-only at noncheckpoint periodic ticks
    end

    % Advance actual plant only AFTER recording x(t_k) and u(t_k).
    dispatch_onefile(time,cha,cha,logger,env,agent,'plant',true);
    agent.controller.result.controllertime_total=toc(ticAll);
    logger.logging(time,cha,agent,[]);
    time.k=logger.k;

end
assert(numel(snapshots)==numel(checkTimes), ...
    'Expected %d snapshots but captured %d',numel(checkTimes),numel(snapshots));
% MATLAB assert requires one scalar logical, not a 1-by-N vector.
% Check all four input channels at every logged control step.
uReplayCheck = data.u(:,2:end);
assert(all(isfinite(uReplayCheck(:))), ...
    'Some HLC inputs were not finite; cannot perform future replay');
end


function [A,tightWorld,screen]=screened_nominal_preview_onefile(source,world,cfg,t0,options)
% One preview of the upcoming horizon, started at t0 DURING flight only.
% Screen static remote obstacles before expensive base collisions; reuse
% its results in refinement. The screen is an OUTER reference envelope,
% not a current-state error bound and not a preflight trajectory forecast.
clock=tic;
[tightWorld,screen]=static_route_screen_onefile(source,world,cfg);
opt=options;
if isfield(opt,'currentPose'),opt=rmfield(opt,'currentPose');end
A=wp_preview_4s(tightWorld,cfg,source,t0,opt);
A.nominalCollisionSamples=A.collisionSamples;
A.nominalMinClearance=A.minClearance;
A.currentPoseChecked=false;A.trackingDeviation=NaN;
if isfield(options,'currentPose') && ~isempty(options.currentPose)
    cp=options.currentPose;
    try
        shp=wp_system_shape_at_pose(cfg,cp.pDrone,cp.pLoad);
        % Current configuration uses the ORIGINAL scene, never the nominal
        % route's potentially over-aggressive culling for the real state.
        rr=wp_system_collision(cfg,shp,world,t0);
        A.currentPoseChecked=true;
        A.trackingDeviation=max(norm(cp.pDrone(:)-A.pDrone(:,1)), ...
             norm(cp.pLoad(:)-A.pLoad(:,1)));
        if rr.collision
            A.collisionSamples(1)=true;
            A.minClearance(1)=min(A.minClearance(1),rr.minClearance);
            A.firstHitObstacle=rr.worstObstacle;
            A.firstHitPart=rr.worstSystemPart;
        end
    catch ex
        A.validSamples(1)=false;
        A.invalidReason=sprintf('Current-pose collision failed: %s',ex.message);
    end
end
idx=find(A.collisionSamples&A.validSamples,1,'first');
if isempty(idx)
    A.collisionDetected=false;A.firstHitTime=NaN;
    A.firstHitObstacle='';A.firstHitPart='';
else
    A.collisionDetected=true;A.firstHitTime=A.times(idx);
end
if any(~A.validSamples)
    A.status='prediction_invalid';
elseif A.collisionDetected
    A.status='sampled_collision';
elseif A.currentPoseChecked && A.trackingDeviation>options.maxTrackingDeviation
    A.status='tracking_deviation_unbounded';
else
    A.status='sampled_clear_not_guaranteed';
end
A.totalElapsedSeconds=toc(clock);
end

function ad=adaptive_preview_test_onefile(source,world,cfg,base,options,budget,preWorld,preScreen,reuseBase)
% Extra samples are selected after one base preview. Never predict anything
% before flight. Reuse the base node collision results in the REAL scene.
% Synthetic probe worlds require fresh baseline collisions (different scene).
clock=tic;
if nargin>=9 && reuseBase
    tightWorld=preWorld;screen=preScreen;
    hit=logical(base.nominalCollisionSamples);
    clr=base.nominalMinClearance;
else
    [tightWorld,screen]=static_route_screen_onefile(source,world,cfg);
    t0=base.times(:)';N0=numel(t0);
    hit=false(1,N0);clr=nan(1,N0);
    for k=1:N0
        shp=wp_system_shape_at_pose(cfg,base.pDrone(:,k),base.pLoad(:,k));
        rep=wp_system_collision(cfg,shp,tightWorld,t0(k));
        hit(k)=logical(rep.collision);clr(k)=rep.minClearance;
    end
end
t=base.times(:)';N=numel(t);I=N-1;
assert(all(base.validSamples),'Nominal baseline must be valid');
if screen.enclosureAvailable
    a=screen.systemEnvelopeMin+screen.systemEnvelopeRadius;
    b=screen.systemEnvelopeMax-screen.systemEnvelopeRadius;
    assert(all(all(base.pLoad>=a-1e-7 & base.pLoad<=b+1e-7)), ...
        'Route envelope excludes a nominal load sample');
end
h=t(2)-t(1);dt=0.025;nSteps=round(h/dt);
assert(abs(nSteps*dt-h)<1e-8);
choices=1:nSteps;choices=choices(mod(nSteps,choices)==0);
valid=choices((nSteps./choices)*dt<=budget.fineStep_s+1e-9);
if isempty(valid),nSub=nSteps;else,nSub=min(valid);end
nAdd=nSub-1;
Mids=(t(1:end-1)+t(2:end))/2;
[X,~]=wp_preview_reference_series(source,Mids,options);
midD=nan(3,I);midL=nan(3,I);
for k=1:I
    [midD(:,k),midL(:,k),pi]=wp_pose_from_reference(X(:,k),cfg,options);
    assert(pi.valid,'Nominal midpoint invalid');
end
if isempty(tightWorld.parts), names=string.empty(1,0);
else,names=string({tightWorld.parts.obstacleName});end
objNames=unique(names,'stable');
if budget.includeGroundAdaptive && cfg.checkGround && ~isempty(tightWorld.ground)
    objNames=[objNames,"__ground__"];
end
nObj=numel(objNames);
% Static part-to-obstacle mapping avoids repeated finds in time loops.
partGroups=cell(1,nObj);
for o=1:nObj
    if objNames(o)=="__ground__",partGroups{o}=[];
    else,partGroups{o}=find(names==objNames(o));end
end
rows=zeros(0,4); % [priority, exact-risk, interval, object]
% Cheap geometry broad-phase, followed by exact current-time narrow phase
% ONLY for shortlisted candidates. No guessed global velocity bound.
for k=1:I
    threeT=[t(k),Mids(k),t(k+1)];
    ds=[base.pDrone(:,k),midD(:,k),base.pDrone(:,k+1)];
    ls=[base.pLoad(:,k),midL(:,k),base.pLoad(:,k+1)];
    wMid=[];shapeMid=[];
    if isfield(tightWorld,'dynamicIndices') && ~isempty(tightWorld.dynamicIndices)
        w3={obs_world_at_time(tightWorld,threeT(1)), ...
            obs_world_at_time(tightWorld,threeT(2)), ...
            obs_world_at_time(tightWorld,threeT(3))};
    else
        w3={tightWorld,tightWorld,tightWorld};
    end
    for o=1:nObj
        gaps=inf(1,3);
        for j=1:3
            if objNames(o)=="__ground__"
                gaps(j)=ground_narrow_gap_onefile(cfg,tightWorld.ground,ds(:,j),ls(:,j));
            else
                w=w3{j};
                for z=partGroups{o}(:)'
                    gaps(j)=min(gaps(j), ...
                        object_body_gap_onefile(cfg,w.parts(z),ds(:,j),ls(:,j)));
                end
            end
        end
        band=budget.nearBand_m;
        if objNames(o)=="__ground__",band=budget.groundNearBand_m;end
        crosses=any(gaps(1:2).*gaps(2:3)<=0);
        near=any(gaps<=band); % IMPORTANT: negative can mean deep overlap
        if ~(crosses||near),continue;end
        if hit(k)&&hit(k+1)&&~crosses,continue;end
        if isempty(shapeMid)
            shapeMid=wp_system_shape_at_pose(cfg,midD(:,k),midL(:,k));
        end
        if isempty(wMid),wMid=w3{2};end
        narrow=wMid;
        narrow.dynamicIndices=[];narrow.ground=[];
        if objNames(o)=="__ground__"
            narrow.parts=narrow.parts([]);narrow.ground=wMid.ground;
        else
            narrow.parts=wMid.parts(partGroups{o});
        end
        % The exact collision kernel handles rotated ellipsoids.
        r=wp_system_collision(cfg,shapeMid,narrow,Mids(k));
        risk=r.minClearance;
        if ~isfinite(risk),risk=inf;end
        priority=double(~crosses);
        if r.collision && ~(hit(k)&&hit(k+1)),priority=-2;end
        if objNames(o)=="__ground__",priority=priority+3;end
        % Prefer genuinely near ellipsoid surfaces, not their loose
        % circumsphere gaps. Collision priority always precedes distance.
        rows(end+1,:)=[priority,abs(risk),k,o]; %#ok<AGROW>
    end
end
if ~isempty(rows),rows=sortrows(rows,[1 2 3 4]);end
perObject=zeros(nObj,1);refined=false(1,I);extra=[];
for j=1:size(rows,1)
    k=rows(j,3);o=rows(j,4);
    if refined(k)||nAdd<=0,continue;end
    perCap=budget.maxExtraPerObstacle;
    if objNames(o)=="__ground__",perCap=budget.maxGroundExtra;end
    if perObject(o)+nAdd>perCap || ...
       numel(extra)+nAdd>budget.maxExtraTotal,continue;end
    extra=[extra,t(k)+(1:nSub-1)*(h/nSub)]; %#ok<AGROW>
    refined(k)=true;perObject(o)=perObject(o)+nAdd;
end
if isempty(rows),budgetUnresolved=0;
else,budgetUnresolved=sum(~refined(unique(rows(:,3))));end
T=sort(unique([t,extra]));M=numel(T);
pD=nan(3,M);pL=nan(3,M);H=false(1,M);V=false(1,M);C=nan(1,M);
[isBase,kBase]=ismember(T,t);
for z=find(isBase)
    j=kBase(z);pD(:,z)=base.pDrone(:,j);pL(:,z)=base.pLoad(:,j);
    H(z)=hit(j);V(z)=true;C(z)=clr(j);
end
idx=find(~isBase);
if ~isempty(idx)
    [XX,~]=wp_preview_reference_series(source,T(idx),options);
    for z=1:numel(idx)
        k=idx(z);
        [dd,ll,pi]=wp_pose_from_reference(XX(:,z),cfg,options);
        if ~pi.valid,continue;end
        try
            shp=wp_system_shape_at_pose(cfg,dd,ll);
            r=wp_system_collision(cfg,shp,tightWorld,T(k));
            pD(:,k)=dd;pL(:,k)=ll;H(k)=logical(r.collision);
            C(k)=r.minClearance;V(k)=true;
        catch
            % Invalid samples must remain explicitly UNVERIFIED.
        end
    end
end
first=find(H&V,1,'first');if isempty(first),firstHit=NaN;
else,firstHit=T(first);end
ad=struct('times',T,'pDrone',pD,'pLoad',pL,...
    'collisionSamples',H,'validSamples',V,'minClearance',C,...
    'firstHitTime',firstHit,'obstacleNames',{cellstr(objNames)},...
    'addedByObstacle',perObject,'candidateCount',size(rows,1),...
    'budgetUnresolved',budgetUnresolved,'refinedIntervals',find(refined),...
    'globalScreen',screen,'totalElapsedSeconds',toc(clock),...
    'discreteOnly',true,'continuousTimeCertified',false);
end

function [world,info]=stress_to_36_parts_onefile(world)
% Test-only capped 36-part worst-case load. If the scene has fewer than 36
% parts, repeat its existing geometries with SMALL deterministic offsets.
% Never claim these duplicates are original scene objects or physical truth.
N=numel(world.parts);assert(N>=1 && N<=36);
info=struct('naturalParts',N,'syntheticAdded',36-N);
assert(~isfield(world,'dynamicIndices') || isempty(world.dynamicIndices), ...
    'Synthetic 36-part stress is STATIC ONLY; use a dynamic scene separately');
if N==36,return;end
base=world.parts;
for j=N+1:36
    i=mod(j-N-1,N)+1;
    q=base(i);
    q.center=q.center+0.035*[mod(j,3)-1,mod(j,5)-2,0];
    % Keep parent original obstacleName; per-object budget is still capped.
    world.parts(end+1,1)=q; %#ok<AGROW>
end
world.baseParts=world.parts;
assert(numel(world.parts)==36 && numel(world.baseParts)==36);
end

function [reduced,meta]=static_route_screen_onefile(source,world,cfg)
% Exact *outer* spatial set for supplied V3 generators; conservative
% enough to include the WHOLE nominal route, not just sampled nodes.
% It is NOT a physical tracking tube and no measured speeds enter here.
reduced=world;
meta=struct('enclosureAvailable',false,'source','none',...
    'originalParts',numel(world.parts),'excludedParts',0,...
    'excludedNames',{{}},'remainingParts',numel(world.parts),...
    'allObstaclePartsExcluded',false,'groundExcluded',false);
if isobject(source) && isprop(source,'generator')
    g=source.generator;
elseif isstruct(source) && isfield(source,'generator')
    g=source.generator;  % internal pure geometry regression only
else
    return;
end
if ~isfield(g,'waypoints')||~isfield(g,'curveType'),return;end
W=double(g.waypoints);
if size(W,1)~=3||any(~isfinite(W(:))),return;end
switch char(g.curveType)
    case 'open_waypoints'
        % Along straight sections: within waypoint coordinate min/max.
        % In a blend, p=W_j+x*((1-S)*a+S*b), |x|<=blend(j),
        % a,b unit directions and 0<=S<=1 (13th smoothstep).
        % Thus ||p-W_j|| <= blend(j). Expand each coordinate by max blend.
        if ~isfield(g,'blendRadiusEffective'),return;end
        b=max([0,g.blendRadiusEffective]);
        xyzLo=min(W,[],2)-b;xyzHi=max(W,[],2)+b;
        meta.source='waypoints_plus_blend_outer_AABB';
    case 'periodic_fourier'
        % x(theta)=real(sum C_k exp(i*k*theta)). For any phase, each
        % coordinate is inside real(C_0) +/- sum_{k>0}|C_k|.
        C=fft(W,[],2)/size(W,2);
        radius=sum(abs(C(:,2:end)),2);
        xyzLo=real(C(:,1))-radius;
        xyzHi=real(C(:,1))+radius;
        meta.source='Fourier_coefficient_triangle_outer_AABB';
    otherwise
        return;
end
L=cfg.cableLength;rc=cfg.cableRadius+cfg.safetyMargin;
rD=cfg.droneRadius+cfg.safetyMargin;
rL=cfg.loadRadius+cfg.safetyMargin;
switch cfg.method
    case 'C'
        chain=hypot(L/(2*cfg.cableSegments),rc);
        wholeRadius=max([L+rD,rL,L+chain]);
    case 'B'
        wholeRadius=max([L+rD,rL,L+rc]);
    case 'D'
        h=L/2;eta=cfg.ellipsoidEta;
        aLong=sqrt((1+eta)*h^2+(1+1/eta)*rc^2);
        aShort=sqrt((1+1/eta)*rc^2);
        wholeRadius=max([L+rD,rL,L/2+max(aLong,aShort)]);
    otherwise,error('Unexpected geometry');
end
% Current wp_pose_from_reference maps raw waypoint z to load z by -L.
xyzLo(3)=xyzLo(3)-L;xyzHi(3)=xyzHi(3)-L;
lo=xyzLo-wholeRadius;hi=xyzHi+wholeRadius;
assert(all(lo<hi)&&all(isfinite([lo;hi])));
meta.enclosureAvailable=true;
meta.systemEnvelopeMin=lo;meta.systemEnvelopeMax=hi;
meta.systemEnvelopeRadius=wholeRadius;
% Static parts: remove ONLY when entire containing sphere misses AABB.
% Dynamic obstacle parts may move: never remove them from a t=0 pose.
dynNames=string.empty(1,0);
if isfield(world,'dynamicIndices') && ~isempty(world.dynamicIndices)
    dynNames=string({world.objects(world.dynamicIndices).name});
end
keep=true(size(world.parts));excluded={};
for j=1:numel(world.parts)
    part=world.parts(j);
    if any(dynNames==string(part.obstacleName)),continue;end
    r=obstacle_bounding_radius_onefile(part);
    sep=norm(max(max(lo-part.center(:),part.center(:)-hi),0));
    if sep>r+1e-9
        keep(j)=false;excluded{end+1}=char(part.obstacleName); %#ok<AGROW>
    end
end
reduced.parts=world.parts(keep);
if isfield(reduced,'baseParts'),reduced.baseParts=world.baseParts(keep);end
meta.excludedParts=sum(~keep);
meta.excludedNames=unique(excluded,'stable');
meta.remainingParts=sum(keep);
meta.allObstaclePartsExcluded=~any(keep);
% A halfspace can be removed only if whole box lies above its boundary.
if cfg.checkGround && ~isempty(world.ground)
    ground=world.ground;
    n=ground.normal(:)/norm(ground.normal(:));
    xCorner=lo;
    xCorner(n<0)=hi(n<0);
    safety=0;if isfield(ground,'safetyMargin'),safety=ground.safetyMargin;end
    groundSafe=dot(n,xCorner-ground.point(:))>safety+1e-9;
    % Stronger (still rigorous) vertical-route bound for B/C geometry.
    % If all control heights are identical, raw z(t) is constant for the
    % V3 open spline and Fourier path. Then a_L,z=0 and q_z<0, so the
    % nominal drone is never below the load. The cable centers lie
    % between the two endpoints. No maximum-velocity assumption is used.
    constantHeight=all(W(3,:)==W(3,1));
    horizontalGround=isequal(n,[0;0;1]);
    if ~groundSafe && constantHeight && horizontalGround && ...
            (cfg.method=='B'||cfg.method=='C')
        rCable=rc;
        if cfg.method=='C'
            rCable=hypot(L/(2*cfg.cableSegments),rc);
        end
        lowSystemZ=W(3,1)-L-max([rD,rL,rCable]);
        groundSafe=(lowSystemZ>ground.point(3)+safety+1e-9);
    end
    if groundSafe
        reduced.ground=[];
        meta.groundExcluded=true;
    end
else
    meta.groundExcluded=true;
end
end

function test_global_outer_screen_onefile()
% Static, deterministic geometry regression; no HLC run and no estimated
% velocities. These sampled checks supplement the algebra, not prove it.
for mode={'gen_wp_straight','gen_wp_circle'}
    gen=feval(mode{1},'speed',0.5);
    src=struct('generator',gen);
    cfg=wp_system_geometry_config('B',struct('cableLength',2));
    p=obs_new_part('sphere',[100,100,100],eye(3),struct('radius',1));
    p.obstacleName='regression_far';
    w=struct('parts',p,'baseParts',p,'ground',[],...
        'dynamicIndices',[]);
    [wf,meta]=static_route_screen_onefile(src,w,cfg);
    assert(meta.enclosureAvailable && meta.excludedParts==1 && ...
        isempty(wf.parts),'Known far obstacle must be excluded');
    sMax=gen.pathLength;
    for sk=linspace(0,sMax,23)
        derivatives=gen.evaluatePath(sk);
        pL=derivatives(:,1)-[0;0;cfg.cableLength];
        % The nominal *load position* must be within route envelope.
        a=meta.systemEnvelopeMin+meta.systemEnvelopeRadius;
        b=meta.systemEnvelopeMax-meta.systemEnvelopeRadius;
        assert(all(pL>=a-1e-7 & pL<=b+1e-7), ...
            'Route envelope omitted a known curve position');
    end
end
end


function test_rotated_ellipsoid_fixture_onefile()
% Numeric regression for the REAL supported ellipsoid obstacle kind.
% Not a claim about continuous collision between samples.
R=obs_rpy_matrix([18 30 55]);
assert(norm(R'*R-eye(3),'fro')<1e-9 && det(R)>0.999, ...
    'Rotated-ellipsoid orientation is invalid');
assert(norm(R-eye(3),'fro')>0.01,'Rotation test must not use identity');
for model={'B','C','D'}
    cfg=wp_system_geometry_config(model{1},struct('cableLength',2));
    shape=wp_system_shape_at_pose(cfg,[0;0;3],[0;0;1]);
    e=obs_new_part('ellipsoid',[0.05 0 3],R, ...
        struct('axes',[0.08 0.12 0.22]));
    e.obstacleName='rotated_ellipsoid_selftest';
    w=struct('parts',e,'ground',[],'dynamicIndices',[]);
    a=wp_system_collision(cfg,shape,w,0);
    assert(a.collision,'Rotated ellipsoid near drone not detected for %s',model{1});
    w.parts.center=[100;100;100];
    b=wp_system_collision(cfg,shape,w,0);
    assert(~b.collision,'Far rotated ellipsoid spuriously collided for %s',model{1});
end
end

function [p,fixture]=make_grazing_probe_part_onefile(~,cfg,reference,options,alpha,mode,budget)
% A deliberately synthetic, OFFLINE obstacle. Find an actual contact
% boundary from the existing collision library rather than treating an
% ellipsoid support radius as a point-to-surface distance.
t=reference.times(:)';N=numel(t)-1;
k=max(1,min(N,round(alpha*N)));
tm=(t(k)+t(k+1))/2;
if isfield(options,'probePlantDt')
    tm=t(k)+round((tm-t(k))/options.probePlantDt)*options.probePlantDt;
end
[XX,~]=wp_preview_reference_series(options.sourceHandle,tm,options);
[p0,pL0,ii]=wp_pose_from_reference(XX(:,1),cfg,options);
assert(ii.valid);
v=reference.pDrone(:,k+1)-reference.pDrone(:,k);
normal=cross(v,[0;0;1]);
if norm(normal)<1e-9,normal=cross(v,[1;0;0]);end
if norm(normal)<1e-9,normal=[1;0;0];end
normal=normal/norm(normal);
rad=budget.probeRadius_m;
shape=char(budget.probeShape);
if strcmp(shape,'ellipsoid')
    R=obs_rpy_matrix([18 30 55]);  % arbitrary fixed 3D orientation, non-spherical
    axes=[rad,0.7*rad,1.6*rad];
else
    R=eye(3);axes=[rad rad rad];
end
w=struct('parts',[],'ground',[],'dynamicIndices',[]);
shp=wp_system_shape_at_pose(cfg,p0,pL0);
% At zero displacement the drone overlaps the obstacle.
p=make_part(0);
assert(would_hit(p,shp,tm), 'Probe must collide at drone center');
hi=cfg.droneRadius+cfg.loadRadius+cfg.cableLength+max(axes)+1;
p=make_part(hi);
for i=1:8
    if ~would_hit(p,shp,tm),break;end
    hi=2*hi;p=make_part(hi);
end
assert(~would_hit(p,shp,tm), 'Could not bracket a non-contact probe offset');
lo=0;
for i=1:27
    mid=(lo+hi)/2;
    p=make_part(mid);
    if would_hit(p,shp,tm),lo=mid;else,hi=mid;end
end
boundary=lo; % greatest known colliding offset
if strcmpi(mode,'overlap')
    penetration=max(0.05,budget.probePenetration_m);
else
    penetration=budget.probePenetration_m;
end
% For grazing, seek contact at the mid-interval and clearance at BOTH
% coarse endpoints. If impossible, record invalid fixture explicitly.
baseClear=false;midHit=false;
for attempt=1:18
    offset=max(0,boundary-penetration);
    p=make_part(offset);
    midHit=would_hit(p,shp,tm);
    shp1=wp_system_shape_at_pose(cfg,reference.pDrone(:,k),reference.pLoad(:,k));
    shp2=wp_system_shape_at_pose(cfg,reference.pDrone(:,k+1),reference.pLoad(:,k+1));
    baseClear=~would_hit(p,shp1,t(k)) && ~would_hit(p,shp2,t(k+1));
    if strcmpi(mode,'overlap') || (midHit&&baseClear),break;end
    penetration=penetration/2;
end
fixture=struct('midTime',tm,'baselineInterval',k, ...
    'midpointHit',logical(midHit),'bracketingEndpointsClear',logical(baseClear), ...
    'placementPenetration_m',penetration, ...
    'grazingFixtureValid',logical(midHit&&baseClear), ...
    'rotation',R,'axes_m',axes);
if strcmpi(mode,'grazing') && ~fixture.grazingFixtureValid
    warning('ABProbe:NotIsolated', ...
        'Grazing fixture cannot guarantee a contact exclusively between two coarse nodes');
end

    function part=make_part(displacement)
        c=p0+normal*displacement;
        if strcmp(shape,'ellipsoid')
            part=obs_new_part('ellipsoid',c.',R,struct('axes',axes));
        else
            part=obs_new_part('sphere',c.',eye(3),struct('radius',rad));
        end
        part.obstacleName='synthetic_probe';part.method='TEST_ONLY';
    end
    function yes=would_hit(part,systemShape,when)
        w.parts=part;
        rep=wp_system_collision(cfg,systemShape,w,when);
        yes=logical(rep.collision);
    end
end

function probe=make_and_run_nominal_probe_onefile(source,world,cfg,base,opt,budget,plantDt)
% Create a second, controlled, geometrically near obstacle, with no
% changes to the actual HLC trajectory or real scene.
assert(isempty(world.dynamicIndices), ...
    'Synthetic probe fixture currently expects static obstacle scenes');
qOpt=opt;
qOpt.sourceHandle=source; % only used locally for the fixture construction
qOpt.probePlantDt=plantDt;
[part,fixture]=make_grazing_probe_part_onefile(world,cfg,base,qOpt, ...
    budget.probeIntervalFraction,char(budget.probeMode),budget);
w=world;
w.parts=[w.parts;part];
if isfield(w,'baseParts'),w.baseParts=[w.baseParts;part];end
ad=adaptive_preview_test_onefile(source,w,cfg,base,opt,budget);
% For nominal inter-sample detection, use a 25ms DENSE NOMINAL trajectory.
% Only the probe sphere is tested; it is not a real-time computation.
T=base.times(1):plantDt:base.times(end);
[XX,~]=wp_preview_reference_series(source,T,opt);
probeOnly=w;probeOnly.parts=part;probeOnly.ground=[];
truth=false(size(T));
for k=1:numel(T)
    [pd,pl,info]=wp_pose_from_reference(XX(:,k),cfg,opt);
    assert(info.valid);
    rr=wp_system_collision(cfg,wp_system_shape_at_pose(cfg,pd,pl),probeOnly,T(k));
    truth(k)=rr.collision;
end
% Re-evaluate baseline against probe-only: the original scene may have
% other contacts, not relevant to this sampling isolation test.
baseProbe=false(size(base.times));
for k=1:numel(base.times)
    rr=wp_system_collision(cfg,wp_system_shape_at_pose(cfg, ...
        base.pDrone(:,k),base.pLoad(:,k)),probeOnly,base.times(k));
    baseProbe(k)=rr.collision;
end
extraProbe=false(size(ad.times));
for k=1:numel(ad.times)
    if ~ad.validSamples(k),continue;end
    rr=wp_system_collision(cfg,wp_system_shape_at_pose(cfg, ...
        ad.pDrone(:,k),ad.pLoad(:,k)),probeOnly,ad.times(k));
    extraProbe(k)=rr.collision;
end
probe=struct('mode',char(budget.probeMode), ...
    'shape',char(budget.probeShape),'fixture',fixture,'world',probeOnly, ...
    'denseTimes',T,'denseNominalHit',truth,'adaptive',ad, ...
    'baselineTimes',base.times, ...
    'baselineProbeHit',baseProbe,'adaptiveProbeHit',extraProbe, ...
    'center',part.center,'part',part);
end

function diag=evaluate_probe_onefile(probe,logData,plantDt)
% No future state or future input is used by the online monitoring design.
% The dense nominal reference and plant truth are OFFLINE diagnostics.
ad=probe.adaptive;
idx=round(probe.denseTimes/plantDt)+1;
truePlant=false(size(idx));
for j=1:numel(idx)
    k=idx(j);x=logData.xTrue(:,k);
    rr=wp_system_collision(logData.geometryConfig, ...
        wp_system_shape_at_pose(logData.geometryConfig,x(1:3),x(13:15)), ...
        probe.world,logData.times(k));
    truePlant(j)=rr.collision;
end
hit=probe.denseNominalHit;
ids=round((ad.times-probe.denseTimes(1))/plantDt)+1;
notSampled=true(size(hit));notSampled(ids)=false;
hidden=0;
for k=find(hit&notSampled)
    lo=find(ids<k,1,'last');hi=find(ids>k,1,'first');
    if ~isempty(lo)&&~isempty(hi)&& ...
            ~probe.adaptiveProbeHit(lo)&&~probe.adaptiveProbeHit(hi)
        hidden=hidden+1;
    end
end
idsBase=round((probe.baselineTimes-probe.denseTimes(1))/plantDt)+1;
% Count nominal 25ms hit samples bracketed by TWO CLEAR baseline nodes.
hiddenBaseline=0;
for k=find(hit)
    lo=find(idsBase<k,1,'last');hi=find(idsBase>k,1,'first');
    if ~isempty(lo)&&~isempty(hi)&& ...
            ~probe.baselineProbeHit(lo)&&~probe.baselineProbeHit(hi)
        hiddenBaseline=hiddenBaseline+1;
    end
end
diag=struct('baseAny',any(probe.baselineProbeHit),...
    'adaptiveAny',any(probe.adaptiveProbeHit),...
    'denseNominalHits',sum(hit),'densePlantHits',sum(truePlant),...
    'hiddenBaselineHits',hiddenBaseline,'hiddenNominalHits',hidden,'extraPoints',numel(ad.times)-numel(probe.baselineProbeHit),...
    'budgetUnresolved',ad.budgetUnresolved,...
    'excludedParts',ad.globalScreen.excludedParts,...
    'fixtureMidHit',probe.fixture.midpointHit, ...
    'fixtureEndpointsClear',probe.fixture.bracketingEndpointsClear, ...
    'grazingFixtureValid',probe.fixture.grazingFixtureValid, ...
    'wallSeconds',ad.totalElapsedSeconds);
end

function gap=object_body_gap_onefile(cfg,part,pD,pL)
% Cheap closest-feature broad phase. This is a CANDIDATE SCORE, not an
% exact collision distance. World ellipsoids are enclosed by circumspheres.
c=part.center(:);R=obstacle_bounding_radius_onefile(part);
gD=norm(pD-c)-(cfg.droneRadius+cfg.safetyMargin+R);
gL=norm(pL-c)-(cfg.loadRadius+cfg.safetyMargin+R);
rc=cfg.cableRadius+cfg.safetyMargin;
if cfg.method=='C'
    rc=hypot(norm(pL-pD)/(2*cfg.cableSegments),rc);
elseif cfg.method=='D'
    h=norm(pL-pD)/2;eta=cfg.ellipsoidEta;
    aLong=sqrt((1+eta)*h^2+(1+1/eta)*rc^2);
    aShort=sqrt((1+1/eta)*rc^2);
    % Heuristic only: allowance for the outward-cover cable ellipsoid.
    rc=rc+max(aLong-h,aShort-rc);
end
gCable=wp_segment_point_distance(pD,pL,c)-(rc+R);
gap=min([gD,gL,gCable]);
end

function gap=ground_narrow_gap_onefile(cfg,ground,pD,pL)
if isempty(ground),gap=inf;return;end
n=ground.normal(:);n=n/norm(n);origin=ground.point(:);
if isfield(ground,'safetyMargin'),margin=ground.safetyMargin;
else,margin=0;end
rD=cfg.droneRadius+cfg.safetyMargin;
rL=cfg.loadRadius+cfg.safetyMargin;
gD=dot(n,pD-origin)-rD-margin;
gL=dot(n,pL-origin)-rL-margin;
rc=cfg.cableRadius+cfg.safetyMargin;
if cfg.method=='C'
    rc=hypot(norm(pD-pL)/(2*cfg.cableSegments),rc);
    gCable=min(dot(n,pD-origin),dot(n,pL-origin))-rc-margin;
elseif cfg.method=='D'
    h=norm(pD-pL)/2;eta=cfg.ellipsoidEta;
    aLong=sqrt((1+eta)*h^2+(1+1/eta)*rc^2);
    aShort=sqrt((1+1/eta)*rc^2);
    axis=(pL-pD)/norm(pL-pD);
    along=dot(n,axis);
    extent=sqrt((aLong*along)^2+aShort^2*(1-along^2));
    gCable=dot(n,(pD+pL)/2-origin)-extent-margin;
else
    gCable=min(dot(n,pD-origin),dot(n,pL-origin))-rc-margin;
end
gap=min([gD,gL,gCable]);
end

function [hitNominal,hitCurrent]=initial_collision_pair_onefile( ...
    cfg,world,pNomD,pNomL,pEstD,pEstL,t0)
r1=wp_system_collision(cfg,wp_system_shape_at_pose(cfg,pNomD,pNomL),world,t0);
r2=wp_system_collision(cfg,wp_system_shape_at_pose(cfg,pEstD,pEstL),world,t0);
hitNominal=logical(r1.collision);hitCurrent=logical(r2.collision);
end

function cert=interval_certificate_onefile(~,~,ad,~)
% This checks ONLY whole-route mathematical set exclusion; there is no
% empirical speed bound and NO assertion about closed-loop tracking.
% Global model envelope is derived from route geometry and cable length.
N=numel(ad.times)-1;
contact=(ad.collisionSamples(1:end-1)|ad.collisionSamples(2:end));
invalid=(~ad.validSamples(1:end-1)|~ad.validSamples(2:end));
sc=ad.globalScreen;
cert=struct('total',N,'certified',0,'nodeContact',sum(contact),...
    'unresolved',0,'reason','');
if sc.enclosureAvailable && sc.allObstaclePartsExcluded && ...
   sc.groundExcluded && ~any(contact) && ~any(invalid)
    cert.certified=N;
    cert.reason='All nominal model geometry excluded by whole-route sets';
else
    cert.reason='Partial spatial exclusion only; nearby/ground intervals are sampled';
end
cert.unresolved=N-cert.certified-cert.nodeContact;
end

function dense=truth_dense_diagnostic_onefile(data,t0,horizon, ...
    testedTimes,testedHit,dt)
% OFFLINE DIAGNOSTIC ONLY: compare 25ms actual PLANT collision samples
% against the actual result at the ADAPTIVE sampling nodes. Distinguish
% unsampled collision points from crossings whose BOTH bracketing sampled
% nodes are clear (the latter is a genuine discretization miss on the
% physical log). Neither count is a continuous-time collision guarantee.
ids=round((t0:dt:t0+horizon)/dt)+1;
num=numel(ids);hits=false(1,num);
for k=1:num
    idx=ids(k);x=data.xTrue(:,idx);
    shp=wp_system_shape_at_pose(data.geometryConfig,x(1:3),x(13:15));
    rep=wp_system_collision(data.geometryConfig,shp,data.world,data.times(idx));
    hits(k)=rep.collision;
end
sampledIdx=round((testedTimes-t0)/dt)+1;
assert(all(sampledIdx>=1 & sampledIdx<=num));
actualAtNodes=logical(testedHit);
unsampled=true(1,num);unsampled(sampledIdx)=false;
hidden=0;
for k=1:num
    if ~(hits(k)&&unsampled(k)),continue;end
    lo=find(sampledIdx<k,1,'last');
    hi=find(sampledIdx>k,1,'first');
    if ~isempty(lo) && ~isempty(hi) && ...
            ~actualAtNodes(lo) && ~actualAtNodes(hi)
        hidden=hidden+1;
    end
end
dense=struct('denseHits',sum(hits), ...
    'unsampledHitCount',sum(hits & unsampled), ...
    'missedDenseHits',hidden,'totalFineSamples',num, ...
    'onlyPlantGrid',true,'continuousTimeCertified',false);
end

function dtOut=choose_forecast_dt_onefile(horizon,nominalSpeed,plantDt,opt)
% Choose an integer submultiple of the horizon's PLANT-grid step count.
% This keeps every baseline/adaptive node aligned with HLC log data.
maxStep=min(opt.maxCoarseStep_s, ...
    opt.coarseTravel_m/max(nominalSpeed,1e-3));
maxStep=max(opt.minCoarseStep_s,maxStep);
N=round(horizon/plantDt);
sub=1:N;sub=sub(mod(N,sub)==0);
usable=sub(sub*plantDt<=maxStep+1e-12);
assert(~isempty(usable),'No valid baseline interval for requested horizon');
dtOut=max(usable)*plantDt;
end

function test_inter_sample_guard_onefile()
% Deterministic reminder: TWO clear endpoints do not prove a clear path.
% No assumed velocity upper bound is used by this experiment.
x0=-1;x1=1;r=0.1;
assert(min(abs([x0,x1]))-r>0,'Endpoint should be clear');
assert(abs((x0+x1)/2)<r,'Midpoint must intersect');
end

function r=obstacle_bounding_radius_onefile(part)
switch char(part.kind)
    case 'sphere'
        r=part.params.radius;
    case 'ellipsoid'
        r=max(part.params.axes);
    otherwise
        error('ABAdaptive:ObstacleKind','Expected sphere/ellipsoid approximations');
end
end

function [ids,hits]=truth_at_adaptive_times(logData,times,plantDt)
% This diagnostic uses future PLANT GROUND TRUTH only after the flight.
% All adaptive times must be on the physical 0.025s grid.
ids=round(times/plantDt)+1;
assert(all(ids>=1 & ids<=numel(logData.times)), ...
    'Adaptive time outside recorded flight');
assert(max(abs(logData.times(ids)-times))<1e-8, ...
    'Adaptive times must align with physical plant samples');
hits=nan(size(times));
for i=1:numel(ids)
    x=logData.xTrue(:,ids(i));
    pD=x(1:3);pL=x(13:15);
    try
        shp=wp_system_shape_at_pose(logData.geometryConfig,pD,pL);
        r=wp_system_collision(logData.geometryConfig,shp, ...
            logData.world,times(i));
        hits(i)=double(r.collision);
    catch
        hits(i)=NaN;
    end
end
end

function test_c_sphere_gate_enclosure_onefile(budget,plantDt)
% Node-level completeness regression for both sphere and ROTATED ellipsoid
% obstacles, tested against the unchanged precise C/B/D collision kernel.
% No trajectory tracking claim; no inter-sample safety claim.
R=obs_rpy_matrix([18 30 55]);
for m={'B','C','D'}
    cfg=wp_system_geometry_config(m{1},struct('cableLength',2));
    c=struct('status','calculated', ...
        'pDrone',repmat([0;0;3],1,3), ...
        'pLoad',repmat([0;0;1],1,3));
    snap=struct('t0',0);
    tiny=budget;tiny.maxExtraTotal=0;
    tiny.maxExtraPerObstacle=0;tiny.maxGroundExtra=0;
    for kind={'sphere','ellipsoid'}
        if strcmp(kind{1},'sphere')
            part=obs_new_part('sphere',[0.05 0 3],eye(3), ...
                struct('radius',0.08));
        else
            part=obs_new_part('ellipsoid',[0.05 0 3],R, ...
                struct('axes',[0.08 0.12 0.22]));
        end
        part.obstacleName='gate_geometry_selftest';
        w=struct('parts',part,'ground',[], 'dynamicIndices',[]);
        hit=wp_system_collision(cfg, ...
            wp_system_shape_at_pose(cfg,c.pDrone(:,1),c.pLoad(:,1)),w,0);
        assert(hit.collision,'Sphere-gate fixture must have a true collision');
        g=c_sphere_gate_monitor_onefile( ...
            snap,c,cfg,w,tiny,plantDt,0.2);
        assert(~g.earlyExit && all(g.baseCandidateNodes) && ...
            all(g.baseHits),'C broadphase missed an actual sample collision');
        w.parts.center=[100;100;100];
        gFar=c_sphere_gate_monitor_onefile( ...
            snap,c,cfg,w,tiny,plantDt,0.2);
        assert(gFar.earlyExit && gFar.exactChecks==0, ...
            'C broadphase did not skip clearly remote obstacle');
    end
end
end

function G=c_sphere_gate_monitor_onefile(s,C,cfg,world,budget,plantDt,baseDt)
% C prediction + two-stage collision evaluation.
% 1) Pairwise ENClOSING SPHERES for drone, load and cable swept between
%    adjacent C nodes (the sweep is an outer bound only for INTERPOLATED
%    endpoint primitives; no assumption on true between-node motion).
% 2) Only interval candidates are sent to original C/B/D exact kernels,
%    with the existing extra-node caps and requested small sample interval.
% If the spheres miss all obstacles, we stop the expensive collision work.
% This says only "no collision at evaluated C nodes"; NOT continuous-time
% nominal safety, nor actual HLC safety. No arbitrary speed error bound.
clock=tic;
t= s.t0+(0:size(C.pLoad,2)-1)*baseDt;
N=numel(t);I=N-1;
G=struct('status','invalid','baseSamples',N,'baseIntervals',I, ...
    'candidateIntervals',0,'candidateObjects',0, ...
    'baseCandidateNodes',true(1,N),'baseHits',false(1,N), ...
    'currentChecked',false,'currentHit',false, ...
    'extraSamples',0,'exactChecks',0,'earlyExit',false, ...
    'unrefinedIntervals',I,'firstHitTime',NaN, ...
    'broadWall_s',NaN,'narrowWall_s',NaN,'wallSeconds',NaN, ...
    'continuousCertified',false,'reason','');
if ~strcmp(C.status,'calculated') || ...
        ~all(isfinite([C.pDrone(:);C.pLoad(:)]))
    G.reason='C predictor invalid: no coarse pruning allowed';
    G.wallSeconds=toc(clock);return;
end
% Explicitly keep moving obstacles in the narrow phase: we do not have a
% mathematically validated bound on their motion BETWEEN samples.
parts=world.parts;
if isempty(parts),objNames=string.empty(1,0);
else,objNames=unique(string({parts.obstacleName}),'stable');end
hasGround=cfg.checkGround && ~isempty(world.ground);
if hasGround,objNames=[objNames,"__ground__"];end
J=numel(objNames);
partNames=string.empty(1,0);
if ~isempty(parts),partNames=string({parts.obstacleName});end
isDynamic=false(1,J);
if isfield(world,'dynamicIndices') && ~isempty(world.dynamicIndices)
    % Until each obstacle has a validated motion enclosure, NEVER screen
    % it using static endpoint locations. Retain ALL objects in the
    % dynamic scene (deliberately conservative, no guessed speed bound).
    isDynamic(1:J)=true;
end
candidate=false(J,I);score=inf(J,I);
% Exact enclosing radii of the provisionally chosen C/B/D geometric
% primitives at each sample: two spheres and the cable's bound sphere.
centers=zeros(3,3,N);radii=zeros(3,N);
for k=1:N
    pD=C.pDrone(:,k);pL=C.pLoad(:,k);
    d=norm(pD-pL);h=d/2;rc=cfg.cableRadius+cfg.safetyMargin;
    centers(:,:,k)=[pD,pL,(pD+pL)/2];
    radii(1,k)=cfg.droneRadius+cfg.safetyMargin;
    radii(2,k)=cfg.loadRadius+cfg.safetyMargin;
    switch cfg.method
        case 'B',rCable = h+rc;
        case 'C',rCable = h+hypot(d/(2*cfg.cableSegments),rc);
        case 'D'
            eta=cfg.ellipsoidEta;
            aLong=sqrt((1+eta)*h^2+(1+1/eta)*rc^2);
            aShort=sqrt((1+1/eta)*rc^2);
            rCable =max(aLong,aShort);
        otherwise,error('Unknown C/B/D system geometry');
    end
    radii(3,k)=rCable;
end
% The current EKF pose is a separate geometric object from the C flatness
% reconstruction at t0. Include BOTH in the t0 broad-phase enclosure.
% This avoids culling an obstacle that intersects the current estimated
% configuration even when the C reconstructed pose starts elsewhere.
if isfield(s,'xEst') && numel(s.xEst)>=15 && ...
        all(isfinite([s.xEst(1:3);s.xEst(13:15)]))
    p0D=s.xEst(1:3);p0L=s.xEst(13:15);
    d0=norm(p0D-p0L);h0=d0/2;
    rc=cfg.cableRadius+cfg.safetyMargin;
    c0=[p0D,p0L,(p0D+p0L)/2];
    r0=[cfg.droneRadius+cfg.safetyMargin; ...
        cfg.loadRadius+cfg.safetyMargin;0];
    switch cfg.method
        case 'B',r0(3)=h0+rc;
        case 'C',r0(3)=h0+hypot(d0/(2*cfg.cableSegments),rc);
        case 'D'
            eta=cfg.ellipsoidEta;
            aLong=sqrt((1+eta)*h0^2+(1+1/eta)*rc^2);
            aShort=sqrt((1+1/eta)*rc^2);
            r0(3)=max(aLong,aShort);
    end
    for i=1:3
        a=centers(:,i,1);b=c0(:,i);
        centers(:,i,1)=0.5*(a+b);
        radii(i,1)=max(radii(i,1),r0(i))+0.5*norm(a-b);
    end
end
% Form a spatial ball that contains both endpoint balls and their linear
% interpolation (not an unproven ball around the true nonlinear path).
for k=1:I
    mid=0.5*(centers(:,:,k)+centers(:,:,k+1));
    outer=max(radii(:,k),radii(:,k+1)) + ...
        0.5*vecnorm(centers(:,:,k+1)-centers(:,:,k),2,1)';
    for j=1:J
        if objNames(j)=="__ground__"
            n=world.ground.normal(:);n=n/norm(n);
            point=world.ground.point(:);
            margin=0;
            if isfield(world.ground,'safetyMargin')
                margin=world.ground.safetyMargin;
            end
            gap=min(n'*(mid-point)-outer')-margin;
            candidate(j,k)=(gap<=budget.groundNearBand_m);
            score(j,k)=gap;
            continue;
        end
        if isDynamic(j)
            % Dynamic obstacle swept motion is not bounded by endpoints.
            candidate(j,k)=true;score(j,k)=-inf;continue;
        end
        js=find(partNames==objNames(j));
        minGap=inf;
        for z=js(:)'
            p=parts(z);r=obstacle_bounding_radius_onefile(p);
            for m=1:3
                gap=norm(mid(:,m)-p.center(:))-(outer(m)+r);
                minGap=min(minGap,gap);
            end
        end
        candidate(j,k)=minGap<=budget.nearBand_m;
        score(j,k)=minGap;
    end
end
G.broadWall_s=toc(clock);
G.candidateIntervals=sum(any(candidate,1));
G.candidateObjects=sum(any(candidate,2));
baseNodes=false(1,N);
for k=find(any(candidate,1))
    baseNodes(k:k+1)=true;
end
G.baseCandidateNodes=baseNodes;
if ~any(baseNodes)
    G.earlyExit=true;G.unrefinedIntervals=0;
    G.currentChecked=isfield(s,'xEst');
    G.status='coarse_nodes_clear_only';
    G.reason='Broad spheres excluded evaluated nodes and linear sweeps only';
    G.narrowWall_s=0;G.wallSeconds=toc(clock);return;
end
narrowClock=tic;
G.unrefinedIntervals=G.candidateIntervals;
for k=find(baseNodes)
    try
        shp=wp_system_shape_at_pose(cfg,C.pDrone(:,k),C.pLoad(:,k));
        r=wp_system_collision(cfg,shp,world,t(k));
        G.baseHits(k)=logical(r.collision);
        G.exactChecks=G.exactChecks+1;
        if k==1 && isfield(s,'xEst')
            % Actual estimated current geometry, not C's reconstruction.
            sh0=wp_system_shape_at_pose(cfg, ...
                s.xEst(1:3),s.xEst(13:15));
            r0=wp_system_collision(cfg,sh0,world,t(1));
            G.currentChecked=true;
            G.currentHit=logical(r0.collision);
            G.exactChecks=G.exactChecks+1;
        end
    catch err
        G.reason=['Baseline collision kernel invalid: ',err.message];
        G.narrowWall_s=toc(narrowClock);
        G.wallSeconds=toc(clock);return;
    end
end
% Assign only optional extra time nodes to candidate intervals. Reuse the
% exact original per-obstacle and global budgets. Prioritize geometry gaps.
nSteps=round(baseDt/plantDt);
assert(abs(nSteps*plantDt-baseDt)<1e-8);
subdiv=1:nSteps;
subdiv=subdiv(mod(nSteps,subdiv)==0 & ...
    baseDt./subdiv<=budget.fineStep_s+1e-9);
if isempty(subdiv),nSub=nSteps;else,nSub=min(subdiv);end
nAdd=nSub-1;
perObj=zeros(J,1);used=false(1,I);extra=[];
[rj,rk]=find(candidate);
if ~isempty(rj)
    ranks=[score(sub2ind(size(score),rj,rk)),rk,rj];
    ranks=sortrows(ranks,[1 2 3]);
    for ii=1:size(ranks,1)
        k=ranks(ii,2);j=ranks(ii,3);
        if used(k) || nAdd==0,continue;end
        cap=budget.maxExtraPerObstacle;
        if objNames(j)=="__ground__",cap=budget.maxGroundExtra;end
        if perObj(j)+nAdd>cap || ...
                numel(extra)+nAdd>budget.maxExtraTotal,continue;end
        extra=[extra,t(k)+(1:nSub-1)*baseDt/nSub]; %#ok<AGROW>
        used(k)=true;perObj(j)=perObj(j)+nAdd;
    end
end
G.unrefinedIntervals=sum(any(candidate,1)&~used);
extra=sort(unique(extra));G.extraSamples=numel(extra);
if ~isempty(extra)
    requested=round((extra-s.t0)/plantDt);
    extraC=hlc_error_closed_loop_preview_onefile( ...
        s,cfg,plantDt,1,numel(requested),requested);
    if ~strcmp(extraC.status,'calculated')
        G.reason=['Extra C predictor invalid: ',extraC.reason];
        G.narrowWall_s=toc(narrowClock);
        G.wallSeconds=toc(clock);return;
    end
    for i=1:numel(extra)
        try
            shp=wp_system_shape_at_pose(cfg, ...
                extraC.pDrone(:,i),extraC.pLoad(:,i));
            r=wp_system_collision(cfg,shp,world,extra(i));
            G.exactChecks=G.exactChecks+1;
            if r.collision
                if isnan(G.firstHitTime),G.firstHitTime=extra(i);
                else,G.firstHitTime=min(G.firstHitTime,extra(i));end
            end
        catch err
            G.reason=['Extra collision kernel invalid: ',err.message];
            G.narrowWall_s=toc(narrowClock);
            G.wallSeconds=toc(clock);return;
        end
    end
end
first=find(G.baseHits,1,'first');
if G.currentHit
    if isnan(G.firstHitTime),G.firstHitTime=t(1);
    else,G.firstHitTime=min(G.firstHitTime,t(1));end
end
if ~isempty(first)
    if isnan(G.firstHitTime)
        G.firstHitTime=t(first);
    else
        G.firstHitTime=min(G.firstHitTime,t(first));
    end
end
G.status='checked_candidate_samples';
G.reason='Nonlinear inter-node motion not certified; sample-only monitor';
G.narrowWall_s=toc(narrowClock);
G.wallSeconds=toc(clock);
end

function C=hlc_error_closed_loop_preview_onefile(s,cfg,dt,outputStride,nOutputs,requestSteps)
% CAUSAL LOCAL PREDICTOR, no physical input assumption.
% Uses the EXACT HLC generated Vs() expression to extract the six tracking
% error coordinates of x/y: Vs is affine and homogeneous in F2/F3/F4.
% For unit basis gain j, Vs_j = desired output derivative_j - actual_j.
% This is NOT a numerical differentiation of noisy signals.
% Nominal exact I/O feedback linearization yields:
%   de_xy/dt = (A6-B6*F2/F3)e_xy,  de_z/dt=(A2-B2*F1)e_z.
% It is MODEL-CONDITIONAL (mass estimate, nonsingular decoupling,
% unsaturated actuators, taut cable, exact transformed state). Not an
% arbitrary closed-loop model for all operating regimes.
% Future V3 progress commands are recomputed from predicted output error
% at each ORIGINAL 25ms flight step. This exactly integrates the six-stage
% governor's linear filter for each held 25ms command, rather than
% freezing the progress command for 4 seconds.
clock=tic;
if nargin<6
    requestSteps=(0:nOutputs-1)*outputStride;
else
    requestSteps=requestSteps(:)';
    assert(numel(requestSteps)==nOutputs && ...
        all(isfinite(requestSteps)) && ...
        all(requestSteps>=0) && ...
        all(abs(requestSteps-round(requestSteps))<1e-8) && ...
        all(diff(requestSteps)>0), ...
        'C predictor requested steps must be increasing integer indices');
end
C=struct('pDrone',nan(3,nOutputs),'pLoad',nan(3,nOutputs), ...
    'status','invalid','reason','', 'startPoseDiscrepancy_m',NaN, ...
    'inputSaturation',NaN,'loadStateMismatch_m',NaN, ...
    'refLoad',nan(3,nOutputs), ...
    'wallSeconds',NaN,'errorInitial',nan(14,1));
try
    h=s.hlcSnapshot;
    C.inputSaturation=h.inputSaturation;
    f1=h.F1(:)'; f2=h.F2(:)'; f3=h.F3(:)';
    assert(numel(f1)==2 && numel(f2)==6 && numel(f3)==6);
    x=h.state19(:);xd=h.xdNow(:);P=h.P(:)';
    assert(numel(x)==19 && numel(xd)==28 && numel(P)==9);
    assert(all(isfinite([x;xd;P(:);f1(:);f2(:);f3(:)])));
    % The HLC's own symbolic transformation, with its actual current vf.
    vf=h.controller.Vfd_SuspendedLoadxyDst(dt,x,xd',f1);
    ex=zeros(6,1);ey=zeros(6,1);
    for j=1:6
        basis=zeros(1,6);basis(j)=1;
        feedback=h.controller.Vs_SuspendedLoadxyDst( ...
            x,xd',vf,P,basis,basis,zeros(1,2));
        ex(j)=-feedback(1); ey(j)=-feedback(2);
    end
    ez=[s.xEst(15)-xd(3);s.xEst(18)-xd(7)];
    C.errorInitial=[ex;ey;ez];
    % Independently validate the symbolic extraction against measured
    % load position and velocity (no empirical tolerance as safety bound).
    direct=[s.xEst(13)-xd(1);s.xEst(16)-xd(5); ...
            s.xEst(14)-xd(2);s.xEst(17)-xd(6)];
    fromSymbol=[ex(1);ex(2);ey(1);ey(2)];
    C.loadStateMismatch_m=max(abs(direct-fromSymbol));
    if any(~isfinite(C.errorInitial)) || C.loadStateMismatch_m>1e-7
        C.reason='HLC transformed state extraction inconsistent with EKF position/velocity';
        C.wallSeconds=toc(clock);return;
    end
    % The exact matrix exponentials are unchanged as long as the HLC
    % gains, governor time constant and dt are identical. Cache them to
    % avoid repeated expm calls on each monitoring tick; invalidate when
    % any entry differs. This is an algebraic optimization, not a model
    % approximation or an empirical dynamics assumption.
    T=h.filterTime;
    assert(isfinite(T) && T>0);
    persistent matrixKey matrixCache
    keyNow=[dt;T;f1(:);f2(:);f3(:)];
    if isempty(matrixKey) || ~isequal(matrixKey,keyNow)
        A6=diag(ones(5,1),1);B6=[zeros(5,1);1];
        A2=[0 1;0 0];B2=[0;1];
        cache.Dx=expm((A6-B6*f2)*dt);
        cache.Dy=expm((A6-B6*f3)*dt);
        cache.Dz=expm((A2-B2*f1)*dt);
        Ag=zeros(7,7);Ag(1:6,1:6)=-eye(6)/T;
        Ag(2:6,1:5)=Ag(2:6,1:5)+eye(5)/T;
        Ag(7,6)=1; Bg=[1/T;zeros(6,1)];
        M=expm([Ag Bg; zeros(1,8)]*dt);
        cache.Ad=M(1:7,1:7);cache.Bd=M(1:7,8);
        matrixKey=keyNow;matrixCache=cache;
    end
    Dx=matrixCache.Dx;Dy=matrixCache.Dy;Dz=matrixCache.Dz;
    Ad=matrixCache.Ad;Bd=matrixCache.Bd;
    governor=[h.rateState(:);h.virtualTime];
    cmd=h.heldRateCommand;
    assert(numel(governor)==7 && all(isfinite(governor)));
    assert(all(isfinite([cmd,h.slowdownStart,h.slowdownFull,h.minScale])));
    assert(h.slowdownFull>h.slowdownStart && h.minScale>=0 && h.minScale<=1);
    nSteps=requestSteps(end);
    nextOutput=1;
    for step=0:nSteps
        if nextOutput<=nOutputs && step==requestSteps(nextOutput)
            k=nextOutput;
            nextOutput=nextOutput+1;
            if step==0
                raw=xd;
                % The HLC flight reference is AFTER SUSPENDED_LOAD_REF_ADJUST.
                % wp_pose_from_reference expects BEFORE adjustment.
                raw(3)=raw(3)+cfg.cableLength;
            else
                derivatives=hlc_governor_derivatives_onefile(governor,T);
                raw=h.generator.evaluateWarped(governor(7),derivatives);
            end
            raw=raw(:);
            C.refLoad(:,k)=[raw(1:2);raw(3)-cfg.cableLength];
            % Output error drives the actual load position; the second
            % load derivative determines the cable direction under the
            % same taut-cable flatness map as the original A preview.
            raw(1:3)=raw(1:3)+[ex(1);ey(1);ez(1)];
            raw(9:11)=raw(9:11)+[ex(3);ey(3);-f1*ez];
            [pD,pL,info]=wp_pose_from_reference(raw,cfg, ...
                struct('gravity',P(5),'predictionMode','flatness'));
            if ~info.valid || any(~isfinite([pD;pL]))
                C.reason='Flatness inverse or future reference invalid';
                C.wallSeconds=toc(clock);return;
            end
            C.pDrone(:,k)=pD;C.pLoad(:,k)=pL;
            if step==0
                C.startPoseDiscrepancy_m=max( ...
                    norm(pD-s.xEst(1:3)),norm(pL-s.xEst(13:15)));
            end
        end
        if step==nSteps,break;end
        % Predict the HLC output error; feed it back to ORIGINAL V3
        % governor (same smoothstep rule), without holding any command
        % across multiple controller cycles.
        ex=Dx*ex;ey=Dy*ey;ez=Dz*ez;
        governor=Ad*governor+Bd*cmd;
        posErr=norm([ex(1);ey(1);ez(1)]);
        eta=max(0,min(1,(posErr-h.slowdownStart)/ ...
            (h.slowdownFull-h.slowdownStart)));
        eta=eta*eta*(3-2*eta);
        cmd=1-(1-h.minScale)*eta;
    end
    C.status='calculated';
    if C.inputSaturation>1e-9
        C.reason='Current HLC actuator command was saturated: nominal error dynamics conditional';
    elseif C.startPoseDiscrepancy_m>1e-3
        C.reason='Flatness current pose does not match current EKF geometry: inspect model assumptions';
    else
        C.reason='Nominal closed-loop error prediction; no empirical or formal true-plant error bound';
    end
catch exn
    C.reason=sprintf('%s: %s',exn.identifier,exn.message);
end
C.wallSeconds=toc(clock);
end

function tauDer=hlc_governor_derivatives_onefile(gov,T)
% Same derivative recurrence used in wp_governor_step.m.
z=gov(1:6);tauDer=zeros(7,1);tauDer(1)=gov(7);
tauDer(2)=z(6);
for k=1:5
    v=0;
    for j=0:k
        v=v+(-1)^j*nchoosek(k,j)*z(6-k+j);
    end
    tauDer(k+2)=v/T^k;
end
end

function x=estimator_to_24state(es)
% Order exactly matches Model_Suspended_Load / Load_HL:
% p(1:3), q(4:6), v(7:9), w(10:12), pL(13:15), vL(16:18),
% pT(19:21), wL(22:24). The EKF extra mL state is intentionally omitted.
fields={'p','q','v','w','pL','vL','pT','wL'};
x=zeros(24,1);
for i=1:8
    key=fields{i};
    if isstruct(es)
        assert(isfield(es,key),'Estimator missing %s',key);
        val=es.(key);
    else
        assert(isprop(es,key),'Estimator missing %s',key);
        val=es.(key);
    end
    assert(isnumeric(val)&&numel(val)==3,'Estimator %s must be 3x1',key);
    x((i-1)*3+(1:3))=val(:);
end
end

function X=integrate_existing_dynamics(f,param,x0,U,outputStride,dt)
% Fourth-order Runge-Kutta applied to the exact original Model_Suspended_Load
% function handle. For each 0.025s step, the input is the logged HLC value
% (or the current HLC value held). Actual plant uses ode15s, so replay
% from true state is included to quantify numerical / timeline differences.
N=size(U,2);
assert(mod(N,outputStride)==0,'integrator output interval mismatch');
X=nan(24,N/outputStride+1);
x=x0(:);X(:,1)=x;
for k=1:N
    u=U(:,k);
    if ~all(isfinite(x)) || ~all(isfinite(u)) || max(abs(x))>1e7
        return;
    end
    try
        a=f(x,u,param);a=a(:);
        b=f(x+dt/2*a,u,param);b=b(:);
        c=f(x+dt/2*b,u,param);c=c(:);
        d=f(x+dt*c,u,param);d=d(:);
        x=x+(dt/6)*(a+2*b+2*c+d);
    catch ex
        warning('ABPreview:IntegrationFailed', ...
            'Nonlinear plant integration failed at k=%d: %s',k,ex.message);
        return;
    end
    if mod(k,outputStride)==0
        X(:,k/outputStride+1)=x;
    end
end
end

function [maxD,maxL,maxAngle,rmsL,valid]=position_errors(pD,pL,tD,tL)
valid=isequal(size(pD),size(tD))&&isequal(size(pL),size(tL))&& ...
    all(isfinite(pD(:)))&&all(isfinite(pL(:)));
if ~valid,maxD=NaN;maxL=NaN;maxAngle=NaN;rmsL=NaN;return;end
d=sqrt(sum((pD-tD).^2,1));
l=sqrt(sum((pL-tL).^2,1));
maxD=max(d);maxL=max(l);rmsL=sqrt(mean(l.^2));
q1=pL-pD;q2=tL-tD;
q1=q1./max(vecnorm(q1),1e-12);q2=q2./max(vecnorm(q2),1e-12);
ang=acosd(max(-1,min(1,sum(q1.*q2,1))));
maxAngle=max(ang);
end

function hits=collisions_from_positions(cfg,world,times,pD,pL)
N=numel(times);hits=nan(1,N);
for k=1:N
    if ~all(isfinite([pD(:,k);pL(:,k)])),continue;end
    try
        shp=wp_system_shape_at_pose(cfg,pD(:,k),pL(:,k));
        res=wp_system_collision(cfg,shp,world,times(k));
        hits(k)=double(res.collision);
    catch
        % A failed geometric check must NOT be reported as collision-free.
        hits(k)=NaN;
    end
end
end

function show_error_comparison(d)
t=d.times-d.checkTime;
labels={'A flatness','C HLC closed-loop error','B replay estimated (oracle)', ...
    'B replay true (oracle)'};
Pdr={d.A.pDrone,d.C_closed.pDrone,d.B_replay_est(1:3,:),d.B_replay_true(1:3,:)};
Pld={d.A.pLoad,d.C_closed.pLoad,d.B_replay_est(13:15,:),d.B_replay_true(13:15,:)};
figure('Name','4s A vs B / NO SAVING');
subplot(2,1,1);hold on;
for j=1:4
    plot(t,vecnorm(Pdr{j}-d.actual(1:3,:)),'LineWidth',1.3);
end
xlabel('Seconds after checkpoint');ylabel('Drone position error [m]');grid on;
legend(labels,'Location','best');title(sprintf('%s speed %.2f t0 %.2f', ...
    d.route,d.speed,d.checkTime),'Interpreter','none');
subplot(2,1,2);hold on;
for j=1:4
    plot(t,vecnorm(Pld{j}-d.actual(13:15,:)),'LineWidth',1.3);
end
xlabel('Seconds after checkpoint');ylabel('Load position error [m]');grid on;
end

function y=motive_output_onefile(obj,data)
p=data.rigid(obj.rigid_id(1)).p;
pT=data.rigid(obj.rigid_id(2)).p-p;pT=pT/norm(pT);
y=[p;Quat2Eul(data.rigid(obj.rigid_id(1)).q);data.rigid(obj.rigid_id(2)).p;pT];
end

function dispatch_onefile(time,cha,fCha,logger,env,agent,prop,isReady)
if ~isReady && strcmp(prop,'plant'),return;end
if isfield(agent(1).cha_allocation,cha) && ...
        isfield(agent(1).cha_allocation.(cha),prop)
    list=agent(1).cha_allocation.(cha).(prop);
else
    list=[];
end
if isempty(list)
    agent(1).(prop).result=agent(1).(prop).do( ...
        time,fCha,logger,env,agent,1);
else
    agent(1).(prop).result=agent(1).(prop).(list(1)).do( ...
        time,fCha,logger,env,agent,1);
    for j=2:length(list)
        agent(1).(prop).result=merge_result( ...
            agent(1).(prop).result, ...
            agent(1).(prop).(list(j)).do(time,fCha,logger,env,agent,1));
    end
end
end
