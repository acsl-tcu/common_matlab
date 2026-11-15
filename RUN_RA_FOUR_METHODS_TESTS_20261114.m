function R=RUN_RA_FOUR_METHODS_TESTS_20261114(commonRoot,runPerformance,runOriginalReplay)
%RUN_RA_FOUR_METHODS_TESTS_20261114 Run four original obstacle methods.
% Test entry only: no unzip, no source changes, no modification to C/B/D,
% sensor or trigger. Prefer official environment/DRONE_OBSTACLES over a
% historical test fixture under TRIGGER_LIGHT/TESTS.
% Every attempted test receives a .status, even if it throws an exception.
% Failures include FULL identifier, message and MATLAB stack information.
if nargin<1 || isempty(commonRoot),commonRoot=pwd;end
if nargin<2 || isempty(runPerformance),runPerformance=true;end
if nargin<3 || isempty(runOriginalReplay),runOriginalReplay=false;end
commonRoot=char(commonRoot);
R=struct('status','NOT_RUN','root',commonRoot,'moduleRoot','', ...
    'testRoot','','cases',struct(), 'failures',{{}}, ...
    'failureDetails',struct('test',{},'identifier',{},'message',{},'stack',{}), ...
    'officialGenerator','', 'activeGenerator','', 'generatorPriority','', ...
    'resultsZip','');

sources={fullfile(commonRoot,'reference','WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE'), ...
    fullfile(commonRoot,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE')};
module='';
for j=1:numel(sources)
    if isfile(fullfile(sources{j},'SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m'))
        module=sources{j};break
    end
end
if isempty(module)
    R.status='SOURCE_NOT_FOUND';
    fprintf(2,'[RA] Source folder not found below %s\n',commonRoot);
    return
end
R.moduleRoot=module;
testRoot=fullfile(commonRoot,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS');
R.testRoot=testRoot;
if ~isfile(fullfile(testRoot,'09_FOUR_METHODS','TEST_RA_FOUR_PART_KIND_UNIT.m'))
    R.status='TESTS_NOT_FOUND';
    fprintf(2,'[RA] Test folder not found: %s\n',testRoot);
    return
end

addpath(module,'-begin');
try
    R.setup=SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
catch ME
    R.status='SETUP_FAILED';
    [R.failures,R.failureDetails]=addFailure(R.failures,R.failureDetails,'setup',ME);
    printFailure('setup',ME);
    saveResult(R,testRoot);
    return
end

% Each test is isolated from the production folder.
testLeaves={'SUPPORT','09_FOUR_METHODS','01_INSTALLATION','02_GEOMETRY', ...
    '03_MEX','04_TRIGGER','05_PERFORMANCE','06_ORIGINAL_TRACE_REPLAY', ...
    '07_SIMULATED_SENSOR','08_SINGLE_SHAPE_AND_FLIGHT'};
for j=1:numel(testLeaves)
    p=fullfile(testRoot,testLeaves{j});
    if isfolder(p),addpath(p,'-begin');end
end

% Explicitly prioritize the actual source used by the Flight project.
% Never allow the historical TRIGGER_LIGHT/TESTS generator to shadow it.
officialDir=fullfile(commonRoot,'environment','DRONE_OBSTACLES');
if isfile(fullfile(officialDir,'DRONE_OBSTACLES.m'))
    addpath(genpath(officialDir),'-begin');
    clear DRONE_OBSTACLES
    rehash
    R.officialGenerator=fullfile(officialDir,'DRONE_OBSTACLES.m');
    R.generatorPriority='OFFICIAL_ENVIRONMENT';
else
    R.generatorPriority='NO_OFFICIAL_ENVIRONMENT_GENERATOR_FOUND';
    fprintf(2,'[RA] Official environment/DRONE_OBSTACLES not found.\n');
end
R.activeGenerator=which('DRONE_OBSTACLES');
fprintf('[RA] DRONE_OBSTACLES -> %s\n',R.activeGenerator);
if ~isempty(R.officialGenerator) && ...
        ~strcmpi(R.activeGenerator,R.officialGenerator)
    R.status='ORIGINAL_GENERATOR_PATH_CONFLICT';
    fprintf(2,'[RA] Original generator path still shadowed.\n');
    return
end

% All task names exist in the result, including when the test fails.
tasks={ ...
    'installation',@()TEST_RA_INSTALLATION(); ...
    'geometrySix',@()TEST_RA_GEOMETRY_6(); ...
    'kind',@()TEST_RA_FOUR_PART_KIND_UNIT(); ...
    'originalGenerator',@()TEST_RA_FOUR_ORIGINAL_GENERATOR(); ...
    'sensorLegacy',@()TEST_RA_SIMULATED_SENSOR(); ...
    'sensorGeneratorLegacy',@()TEST_RA_SIMULATED_SENSOR_GENERATOR(); ...
    'fourStream',@()TEST_RA_FOUR_25MS_STREAM(); ...
    'stream',@()TEST_RA_TICK_25MS_HISTORY(); ...
    'mex',@()TEST_RA_MEX_AUTOBUILD()};
if runPerformance
    tasks(end+1,:)={'geometryPerformance',@()TEST_RA_GEOMETRY_BENCHMARK(25)};
    tasks(end+1,:)={'sensorPerformance',@()TEST_RA_FOUR_SENSOR_BENCHMARK(25)};
else
    R.cases.geometryPerformance=struct('status','SKIPPED_BY_OPTION');
    R.cases.sensorPerformance=struct('status','SKIPPED_BY_OPTION');
end
if runOriginalReplay
    tasks(end+1,:)={'originalReplay',@()RUN_RA_OPTIONAL_96_TRACE_REPLAY(3)};
else
    R.cases.originalReplay=struct('status','SKIPPED_BY_OPTION');
end

for j=1:size(tasks,1)
    name=tasks{j,1};fn=tasks{j,2};
    try
        R.cases.(name)=fn();
        if ~isstruct(R.cases.(name)) || ~isfield(R.cases.(name),'status')
            R.cases.(name)=struct('status','RESULT_MISSING_STATUS');
        end
        fprintf('[RA] %-22s %s\n',name,R.cases.(name).status);
    catch ME
        R.cases.(name)=struct('status','FAIL', ...
            'errorIdentifier',ME.identifier,'errorText',ME.message);
        [R.failures,R.failureDetails]=addFailure(R.failures,R.failureDetails,name,ME);
        printFailure(name,ME);
    end
end
R.actualFlightTested=false;
R.physicalSafetyCertified=false;
R.full25msDeadlineCertified=false;
if isempty(R.failures)
    if startsWith(R.cases.originalGenerator.status,'SKIPPED')
        R.status='PASS_SELF_CONTAINED__ORIGINAL_GENERATOR_NOT_ON_PATH';
    else
        R.status='PASS_FOUR_METHODS_AND_CORE_TESTS__NOT_FLIGHT';
    end
else
    R.status='FAILED_SEE_FAILURES';
end
R=saveResult(R,testRoot);
fprintf('[RA] %s ; failures=%d\n',R.status,numel(R.failures));
end

function [failures,details]=addFailure(failures,details,name,ME)
if isempty(ME.identifier),id='UNIDENTIFIED';else,id=ME.identifier;end
failures{end+1}=sprintf('%s: %s %s',name,id,ME.message); %#ok<AGROW>
stack=cell(numel(ME.stack),1);
for k=1:numel(ME.stack)
    stack{k}=sprintf('%s:%d (%s)',ME.stack(k).file,ME.stack(k).line,ME.stack(k).name);
end
details(end+1)=struct('test',name,'identifier',id, ...
    'message',ME.message,'stack',{stack}); %#ok<AGROW>
end
function printFailure(name,ME)
fprintf(2,'[RA] FAIL %-22s %s : %s\n',name,ME.identifier,ME.message);
for k=1:min(5,numel(ME.stack))
    s=ME.stack(k);
    fprintf(2,'      %s line %d\n',s.file,s.line);
end
end
function R=saveResult(R,testRoot)
outdir=fullfile(testRoot,'RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
fileName=['RA_FOUR_METHODS_DIAG_' datestr(now,'yyyymmdd_HHMMSS_FFF')];
matPath=fullfile(outdir,[fileName '.mat']);
zipPath=fullfile(outdir,[fileName '.zip']);
R.resultsZip=zipPath;
save(matPath,'R');
zip(zipPath,matPath);
end
