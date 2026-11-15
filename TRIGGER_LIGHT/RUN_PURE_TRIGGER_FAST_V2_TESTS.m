function R=RUN_PURE_TRIGGER_FAST_V2_TESTS(randomCases,benchmarkRepeats)
%RUN_PURE_TRIGGER_FAST_V2_TESTS One-mode A/B, detailed costs and theory.
% Cd to common_matlab/TRIGGER_LIGHT, then:
%   R=RUN_PURE_TRIGGER_FAST_V2_TESTS(60,100);
% No Flight source is changed; only staging functions and tests are used.
if nargin<1,randomCases=60;end
if nargin<2,benchmarkRepeats=100;end
root=fileparts(mfilename('fullpath'));
oldPath=path;restore=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(root,fullfile(root,'SIX_WAY'),fullfile(root,'TESTS'), ...
    fullfile(root,'TESTS','BASELINE'));
addpath(genpath(fullfile(root,'TESTS','REFERENCE_GEOMETRY')));
projectRoot=fileparts(root);
if exist('DRONE_OBSTACLES','file')~=2
    official=fullfile(projectRoot,'DRONE_OBSTACLES');
    if exist(fullfile(official,'DRONE_OBSTACLES.m'),'file')==2
        addpath(genpath(official));
    else
        referenceCopy=fullfile(root,'TESTS','REFERENCE_DRONE_OBSTACLES','DRONE_OBSTACLES');
        if exist(fullfile(referenceCopy,'DRONE_OBSTACLES.m'),'file')==2
            addpath(genpath(referenceCopy));
        end
    end
end
fprintf('\n=== SELECTED ONE-MODE TRIGGER: FAST V2 / THEORETICAL AUDIT ===\n');
R=struct();
R.resultFields=TEST_TRIGGER_V2_RESULT_FIELDS();
R.basic=TEST_SIX_TRIGGER_BASIC();
R.inclusion=TEST_SIX_TRIGGER_SWEEP_INCLUSION();
R.obstacleGenerator=TEST_SIX_TRIGGER_DRONE_OBSTACLES();
R.batched=TEST_TRIGGER_V2_FAST_GATES(randomCases);
R.tautEquations=TEST_TRIGGER_V2_MODEL_FORMULAS(500);
R.originalSource=TEST_TRIGGER_V2_SOURCE_DISCOVERY();
R.random=TEST_SIX_TRIGGER_RANDOM(randomCases);
R.benchmark=BENCH_TRIGGER_V2_STAGE_TIMES(benchmarkRepeats,30);
R.syntheticTick=BENCH_TRIGGER_V2_TICK_STAGES(30);
R.status='PASS_V2_IF_ALL_ASSERTIONS_PASSED__REAL_HLC_AND_NUMERIC_PROOF_PENDING';
R.oneSelectedModePerCall=true;
R.estimatedMassIsOnlyForecastModelMass=true;
R.actualPhysicalSafetyCertified=false;
R.actualHlc4sPredictionIntegrated=false;
R.certifiedTrueErrorBoundsProvided=false;
R.fullFlight25msMeasured=false;
R.selectedModeTotal1to2msDemonstrated=false;
resultDir=fullfile(root,'TESTS','RESULTS');
if ~exist(resultDir,'dir'),mkdir(resultDir);end
stem='PURE_TRIGGER_V2_PROFILE_RESULTS_FOR_CHATGPT';
save(fullfile(resultDir,[stem '.mat']),'R');
fid=fopen(fullfile(resultDir,[stem '.txt']),'w');
if fid<0,error('LightTrigger:Test','Cannot write test summary');end
cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'SELECTED ONE-MODE TRIGGER V2 / STAGE PROFILE / USER PC\n');
fprintf(fid,'STATUS %s\n',R.status);
fprintf(fid,'Original obstacle generator %s\n',R.obstacleGenerator.status);
fprintf(fid,'Original RHS resolver %s, identifier %s\n',R.originalSource.resolverStatus,R.originalSource.resolvedDynamicsIdentifier);
fprintf(fid,'Selected one mode ONLY; estimated mL ONLY; no HLC/CBF/Flight timing.\n');
fprintf(fid,'A/B and separate instrumented stage costs, units milliseconds.\n');
keys={'empty','distant_36','cable_contact','near_noncontact_36','thin_ellipsoid_36'};
fields={'validation','inflation','globalBroadphase', ...
    'intervalBroadphase','sweepBuild','componentBroadphase','supportSeparation'};
for s=1:numel(keys)
    key=keys{s};
    fprintf(fid,'\nSCENARIO %s\n',key);
    records=R.benchmark.(key);
    for mi=1:3
        for ci=1:2
            x=records(mi,ci);
            fprintf(fid,'%s/%s old_median %.6f old_p95 %.6f old_max %.6f ', ...
                x.mode,x.cover,x.oldMedianMs,x.oldP95Ms,x.oldMaxMs);
            fprintf(fid,'new_median %.6f new_p95 %.6f new_max %.6f status %s\n', ...
                x.newMedianMs,x.newP95Ms,x.newMaxMs,x.newStatus);
            fprintf(fid,'  counts component_pairs %d narrow_pairs %d global_AABB %d interval_AABB %d component_AABB %d\n', ...
                x.componentCandidatePairs,x.narrowphasePairs, ...
                x.globalAabbCleared,x.intervalAabbCleared,x.componentAabbCleared);
            fprintf(fid,'  instrumented_total_median %.6f',x.stageMedianMeasuredTotalMs);
            for j=1:numel(fields)
                fprintf(fid,' %s=%.6f',fields{j},x.stageMedianMs.(fields{j}));
            end
            fprintf(fid,'\n');
        end
    end
end
fprintf(fid,'\nSYNTHETIC selected-tick stage medians (NO real HLC predictor)\n');
for mi=1:3
  for ci=1:2
    x=R.syntheticTick(mi,ci);
    fprintf(fid,'%s/%s total_median %.6f profile_GEOM %.6f callback %.6f snapshot %.6f config %.6f cache %.6f\n', ...
        x.mode,x.cover,x.totalMedianMs,x.stageMedianMs.geometryEvaluation, ...
        x.stageMedianMs.forecastProvider,x.stageMedianMs.modelSnapshot, ...
        x.stageMedianMs.configuration,x.stageMedianMs.obstacleCache);
  end
end
fprintf(fid,'\nNOTE: instrumented stages incur tic/toc overhead. Uninstrumented total is authoritative.\n');
fprintf(fid,'The analytic taut acceleration envelope is for estimated-mass model ONLY; it is NOT a true-plant or HLC error certificate.\n');
clear cleaner;
zip(fullfile(resultDir,[stem '.zip']),{[stem '.mat'],[stem '.txt']},resultDir);
fprintf('Saved %s\n',fullfile(resultDir,[stem '.zip']));
end
