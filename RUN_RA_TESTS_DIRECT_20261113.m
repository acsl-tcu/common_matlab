function R = RUN_RA_TESTS_DIRECT_20261113(commonRoot, runPerformance, runOriginalReplay)
%RUN_RA_TESTS_DIRECT_20261113 Run obstacle-avoidance tests from separate paths.
% Unique launcher name prevents an old AUTOLOCATE function from shadowing it.
% No ZIP extraction, no folder moves, no fixed sibling-folder assertion.
%
% Example (file located in common_matlab):
%   cd('C:\Users\student\Documents\GitHub\common_matlab')
%   R = RUN_RA_TESTS_DIRECT_20261113(pwd,true,false);
%
% This is only an offline test entry. NEVER call it in a 25-ms Flight loop.
if nargin < 1 || isempty(commonRoot)
    commonRoot = fileparts(mfilename('fullpath'));
end
if nargin < 2 || isempty(runPerformance), runPerformance = true; end
if nargin < 3 || isempty(runOriginalReplay), runOriginalReplay = false; end
commonRoot = char(commonRoot);
R = struct('status','NOT_RUN','commonRoot',commonRoot, ...
    'moduleRoot','','testRoot','','errorText','', ...
    'resultsZip','','failures',{{}});
if ~isfolder(commonRoot)
    R.status = 'COMMON_ROOT_NOT_FOUND';
    R.errorText = ['Directory not found: ' commonRoot];
    fprintf(2,'%s\n',R.errorText);
    return;
end

% Independent discovery: the production source and tests need not be siblings.
sourceCandidates = dir(fullfile(commonRoot,'**','SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m'));
testCandidates = dir(fullfile(commonRoot,'**','TEST_RA_INSTALLATION.m'));
sourceFolders = validParent(sourceCandidates,'REFERENCE_OBSTACLE_AVOIDANCE');
testFolders = validParent(testCandidates,'01_INSTALLATION');
if isempty(sourceFolders)
    R.status = 'SOURCE_NOT_FOUND';
    R.errorText = ['SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m not found under ' commonRoot];
    fprintf(2,'%s\n',R.errorText);
    return;
end
R.moduleRoot = selectSource(sourceFolders,commonRoot);
if isempty(testFolders)
    R.status = 'TESTS_NOT_FOUND';
    R.errorText = ['TEST_RA_INSTALLATION.m not found under ' commonRoot];
    fprintf(2,'Source found: %s\n%s\n',R.moduleRoot,R.errorText);
    fprintf(['Place the manually extracted REFERENCE_OBSTACLE_AVOIDANCE_TESTS ' ...
        'folder anywhere below common_matlab.\n']);
    return;
end
% The discovered 01_INSTALLATION directory is immediately below test root.
rootOptions = cellfun(@fileparts,testFolders,'UniformOutput',false);
R.testRoot = selectTest(rootOptions,commonRoot);
fprintf('[Direct tests] Source: %s\n',R.moduleRoot);
fprintf('[Direct tests] Tests:  %s\n',R.testRoot);

% Explicit priority of the source that actually exists, not a guessed path.
addpath(R.moduleRoot,'-begin');
rehash;
try
    R.setup = SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
catch ME
    R.status = 'SETUP_FAILED';
    R.errorText = getReport(ME,'extended','hyperlinks','off');
    fprintf(2,'%s\n',R.errorText);
    return;
end

sub = {'01_INSTALLATION','02_GEOMETRY','03_MEX', ...
    '04_TRIGGER','05_PERFORMANCE','06_ORIGINAL_TRACE_REPLAY','07_SIMULATED_SENSOR','SUPPORT'};
for k = numel(sub):-1:1
    p = fullfile(R.testRoot,sub{k});
    if isfolder(p), addpath(p,'-begin'); end
end
rehash;

% These test functions are called DIRECTLY: the old
% RUN_REFERENCE_AVOIDANCE_TESTS and AUTOLOCATE are never invoked.
cases = { ...
    'installation','TEST_RA_INSTALLATION'; ...
    'sixModeSmoke','TEST_REFERENCE_AVOIDANCE_INSTALL'; ...
    'geometry','TEST_RA_GEOMETRY_6'; ...
    'mex','TEST_RA_MEX_AUTOBUILD'; ...
    'history','TEST_RA_TICK_25MS_HISTORY'; ...
    'simulatedSensor','RUN_RA_SIMULATED_SENSOR_TESTS'};
if runPerformance
    cases(end+1,:) = {'performance','TEST_RA_GEOMETRY_BENCHMARK'};
end
if runOriginalReplay
    cases(end+1,:) = {'originalReplay','RUN_RA_OPTIONAL_96_TRACE_REPLAY'};
end
for i = 1:size(cases,1)
    fieldName = cases{i,1};
    funcName = cases{i,2};
    resolved = which(funcName);
    if isempty(resolved) || ~startsWith(resolved,[R.testRoot filesep],'IgnoreCase',true)
        R.(fieldName) = struct('status','MISSING_OR_SHADOWED', ...
            'errorText',['Expected test function inside test folder; resolved: ' resolved]);
        R.failures{end+1} = fieldName; %#ok<AGROW>
        fprintf(2,'[FAIL] %s: %s\n',fieldName,R.(fieldName).errorText);
        continue;
    end
    try
        switch fieldName
            case 'performance'
                result = feval(funcName,30);
            case 'simulatedSensor'
                result = feval(funcName,false);
            case 'originalReplay'
                result = feval(funcName,3);
            otherwise
                result = feval(funcName);
        end
        R.(fieldName) = result;
        if isstruct(result) && isfield(result,'status')
            fprintf('[%s] %s\n',fieldName,result.status);
            % An optional SKIPPED original replay is not a core failure.
            if ~startsWith(result.status,'PASS') && ...
                    ~startsWith(result.status,'INFORMATIONAL') && ...
                    ~(strcmp(fieldName,'originalReplay') && startsWith(result.status,'SKIPPED'))
                R.failures{end+1} = fieldName; %#ok<AGROW>
            end
        else
            R.failures{end+1} = fieldName; %#ok<AGROW>
            fprintf(2,'[FAIL] %s: no status field\n',fieldName);
        end
    catch ME
        R.(fieldName) = struct('status','FAIL','errorText', ...
            getReport(ME,'extended','hyperlinks','off'));
        R.failures{end+1} = fieldName; %#ok<AGROW>
        fprintf(2,'[FAIL] %s: %s\n',fieldName,ME.message);
    end
end
R.physicalSafetyCertified = false;
R.fullPrediction2msProved = false;
R.flight25msProved = false;
if isempty(R.failures)
    R.status = 'PASS_CORE_TESTS_NOT_FLIGHT';
else
    R.status = 'FAILED_TESTS';
end

% Save results without unpacking or moving any user folders. A failure to
% save does NOT override the test results or falsely claim test success.
try
    outdir = fullfile(R.testRoot,'RESULTS');
    if ~isfolder(outdir), mkdir(outdir); end
    stamp = datestr(now,'yyyymmdd_HHMMSS_FFF');
    matPath = fullfile(outdir,['RA_DIRECT_' stamp '.mat']);
    zipPath = fullfile(outdir,['RA_DIRECT_' stamp '.zip']);
    R.resultsZip = zipPath;
    save(matPath,'R');
    zip(zipPath,matPath);
catch ME
    R.saveWarning = ME.message;
    fprintf(2,'Tests ran, but saving failed: %s\n',ME.message);
end
fprintf('[Direct tests] %s (failed: %d)\n',R.status,numel(R.failures));
end

function folders = validParent(entries,requiredLeaf)
folders = {};
for i=1:numel(entries)
    p = entries(i).folder;
    [~,leaf] = fileparts(p);
    if strcmpi(leaf,requiredLeaf) && ~any(strcmpi(p,folders))
        folders{end+1} = p; %#ok<AGROW>
    end
end
end

function pathOut=selectSource(options,commonRoot)
preferred = { ...
    fullfile(commonRoot,'reference','WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE'), ...
    fullfile(commonRoot,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE')};
pathOut=selectPreferred(options,preferred);
end

function pathOut=selectTest(options,commonRoot)
preferred = { ...
    fullfile(commonRoot,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS'), ...
    fullfile(commonRoot,'reference','REFERENCE_OBSTACLE_AVOIDANCE_TESTS')};
pathOut=selectPreferred(options,preferred);
end

function pathOut=selectPreferred(options,preferred)
pathOut='';
for j=1:numel(preferred)
    idx=find(strcmpi(options,preferred{j}),1);
    if ~isempty(idx),pathOut=options{idx};return;end
end
[~,idx]=min(cellfun(@numel,options));
pathOut=options{idx};
end
