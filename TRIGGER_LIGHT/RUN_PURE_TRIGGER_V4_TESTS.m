function R=RUN_PURE_TRIGGER_V4_TESTS(nRandom,nTiming,runHlcStress)
%RUN_PURE_TRIGGER_V4_TESTS Geometry V3/V4, model and original-HLC audits.
% Example, from common_matlab/TRIGGER_LIGHT after addpath(genpath(...)):
%   R=RUN_PURE_TRIGGER_V4_TESTS(100,60,true);
%
% All six variants are separate tests; a real tick uses EXACTLY ONE.
% This does not change the Flight run, controller, EKF or CBF.
% HLC 4-s diagnostics are OFFLINE and are NOT included in the V4 geometry
% benchmark, and are NOT a usable 1-2 ms future predictor.
if nargin<1,nRandom=100;end
if nargin<2,nTiming=60;end
if nargin<3,runHlcStress=true;end
R=struct('status','NOT_RUN','sixModesAlternatives',true, ...
    'onlineFullTrigger2msProven',false,'realFlightErrorCertified',false, ...
    'sourceRhsVerified',false,'hlcStressVerified',false);
R.orientedBox=execute(@()TEST_TRIGGER_V4_ORIENTED_AABB(nRandom));
R.selectedTick=execute(@TEST_TRIGGER_V4_SELECTED_TICK);
R.sourceEquations=execute(@()TEST_TRIGGER_V3_24STATE_FORMULAS(300));
R.parameterSweep=execute(@()TEST_TRIGGER_V4_SOURCE_PARAM_SWEEP(400));
if runHlcStress
    R.hlcStress=execute(@TEST_TRIGGER_V4_HLC_STRESS);
else
    R.hlcStress=struct('status','SKIPPED_BY_CALLER');
end
R.benchmark=execute(@()BENCH_TRIGGER_V4_ONE_MODE_AB(nTiming));
R.sourceRhsVerified=startsWith(string(R.sourceEquations.status),'PASS');
R.hlcStressVerified=startsWith(string(R.hlcStress.status), ...
    'PASS_FOUR_ORIGINAL_NOMINAL_STRESS_ROLLOUTS');
if startsWith(string(R.orientedBox.status),'PASS') && ...
        startsWith(string(R.selectedTick.status),'PASS') && ...
        startsWith(string(R.benchmark.status),'PASS')
    R.status='PASS_V4_GEOMETRY_AUDIT__LIGHT_HLC_PREDICTOR_PENDING';
else
    R.status='FAIL_V4_AUDIT_INSPECT_RESULTS';
end
folder=fullfile(fileparts(mfilename('fullpath')),'TESTS','RESULTS');
if ~exist(folder,'dir'),mkdir(folder);end
save(fullfile(folder,'PURE_TRIGGER_V4_RESULTS.mat'),'R');
fid=fopen(fullfile(folder,'PURE_TRIGGER_V4_RESULTS.txt'),'w');
if fid<0,error('LightTrigger:Results','Unable to write result text');end
fprintf(fid,'Result: %s\n6 alternatives: one mode per controller cycle\n',R.status);
sections={'orientedBox','selectedTick','sourceEquations','parameterSweep','hlcStress','benchmark'};
for j=1:numel(sections)
    name=sections{j};part=R.(name);
    fprintf(fid,'%s: %s\n',name,part.status);
    if isfield(part,'errorMessage')
        fprintf(fid,'  Error: %s\n',part.errorMessage);
    end
end
if isfield(R.benchmark,'records')
    for j=1:numel(R.benchmark.records)
        q=R.benchmark.records(j);
        fprintf(fid,'%-19s %s x %-19s V3 %.5f V4 %.5f P95 %.5f MAX %.5f ms tight=%d\n', ...
            q.scenario,q.mode,q.cover,q.v3MedianMs,q.v4MedianMs, ...
            q.v4P95Ms,q.v4MaxMs,q.v4TightBoxCleared);
    end
end
if isfield(R.hlcStress,'results') && ~isempty(R.hlcStress.results)
    for j=1:numel(R.hlcStress.results)
        q=R.hlcStress.results(j);
        fprintf(fid,'HLC %-18s %s %.4fms; sat=%d minT=%g; %s\n', ...
           q.caseName,q.status,q.elapsedMs,q.saturationSeen, ...
           q.minSampledTension,q.errorText);
    end
end
fprintf(fid,'ONLINE 1-2ms includes actual forecast? NO.\n');
fprintf(fid,'TRUE physical prediction-error certificate? NO.\n');
fclose(fid);
files={'PURE_TRIGGER_V4_RESULTS.mat','PURE_TRIGGER_V4_RESULTS.txt'};
zip(fullfile(folder,'PURE_TRIGGER_V4_RESULTS_FOR_CHATGPT.zip'), ...
    fullfile(folder,files));
fprintf('V4 result %s, saved %s\n',R.status,folder);
end
function r=execute(f)
try
    r=f();
    if ~isstruct(r) || ~isfield(r,'status')
        r=struct('status','FAIL_RETURN_CONTRACT');
    end
catch ME
    r=struct('status','FAIL_EXCEPTION','errorMessage',ME.message, ...
        'errorIdentifier',ME.identifier);
    fprintf('V4 test failure: %s : %s\n',ME.identifier,ME.message);
end
end
