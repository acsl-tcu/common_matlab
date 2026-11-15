function R=RUN_PURE_TRIGGER_FAST_V3_TESTS(randomCases,benchRepetitions,runOriginalHlc)
%RUN_PURE_TRIGGER_FAST_V3_TESTS Staging V3, one selected mode per run.
%  cd('.../common_matlab/TRIGGER_LIGHT')
%  R=RUN_PURE_TRIGGER_FAST_V3_TESTS(60,100,true);
% No production code is modified by this test suite.
if nargin<1,randomCases=60;end
if nargin<2,benchRepetitions=100;end
if nargin<3,runOriginalHlc=true;end
here=fileparts(mfilename('fullpath'));
addpath(fullfile(here,'SIX_WAY'));
addpath(fullfile(here,'TESTS'));
addpath(fullfile(here,'TESTS','BASELINE'));
% The original preview/config/obstacle sources must be on the project's
% normal MATLAB path. No arbitrary source clones are installed in Flight.
start=tic;
R=struct();
R.geometry=TEST_TRIGGER_V3_GEOMETRY_AUDIT(randomCases);
R.sourceEquations=TEST_TRIGGER_V3_24STATE_FORMULAS(300);
R.benchmark=BENCH_TRIGGER_V3_ONE_MODE_AB(benchRepetitions,20);
if runOriginalHlc
    R.originalHlc=TEST_TRIGGER_V3_ORIGINAL_HLC_4S();
else
    R.originalHlc=struct('status','NOT_RUN_BY_USER');
end
R.full25msFlightMeasured=false;
R.realEKFFutureErrorCertified=false;
R.geometricSweepIsConditionallyCertified=true;
R.perSelectedModeGoalMs=2;
R.sixModesAreNeverSimultaneouslyApplied=true;
R.originalHlcRolloutIsOfflineReference=true;
R.totalTestRuntimeSeconds=toc(start);
if contains(R.originalHlc.status,'FAIL')
    R.status='FAIL_ORIGINAL_HLC_ROLLOUT';
elseif startsWith(R.originalHlc.status,'SKIPPED') || ...
        startsWith(R.sourceEquations.status,'SKIPPED')
    R.status='PASS_GEOMETRY__ORIGINAL_HLC_SOURCE_PATH_REQUIRED';
else
    R.status='PASS_V3_AUDIT__ONLINE_PREDICTION_AND_2MS_PENDING';
end
fprintf('\n%s\n',R.status);
results=fullfile(here,'TESTS','RESULTS');
if ~isfolder(results),mkdir(results);end
save(fullfile(results,'PURE_TRIGGER_V3_RESULTS.mat'),'R','-v7');
summaryFile=fullfile(results,'PURE_TRIGGER_V3_RESULTS.txt');
fid=fopen(summaryFile,'w');
fprintf(fid,'PURE PORT V3 - SELECTED MODE / ONE MODE PER CALL\n');
fprintf(fid,'STATUS: %s\n',R.status);
fprintf(fid,'Geometry: %s\n',R.geometry.status);
fprintf(fid,'Original HLC: %s\n',R.originalHlc.status);
fprintf(fid,'Original 24-state RHS: %s\n',R.sourceEquations.status);
if isfield(R.originalHlc,'reason'),fprintf(fid,'Original HLC reason: %s\n',R.originalHlc.reason);end
fprintf(fid,'Goal: 1-2ms including prediction, NOT YET ACHIEVED\n');
fprintf(fid,'True physical error enclosure: NOT PROVEN\n');
for k=1:numel(R.benchmark.records)
    v=R.benchmark.records(k);
    fprintf(fid,'%s %s %s V2median=%.6f V3median=%.6f V3P95=%.6f V3MAX=%.6f preclear=%d\n', ...
        v.scenario,v.mode,v.cover,v.v2MedianMs,v.v3MedianMs,v.v3P95Ms, ...
        v.v3MaxMs,v.precomputedPairsCleared);
end
fclose(fid);
zip(fullfile(results,'PURE_TRIGGER_V3_RESULTS_FOR_CHATGPT.zip'), ...
    {'PURE_TRIGGER_V3_RESULTS.mat','PURE_TRIGGER_V3_RESULTS.txt'},results);
fprintf('Saved %s\n',fullfile(results,'PURE_TRIGGER_V3_RESULTS_FOR_CHATGPT.zip'));
end
