function R = RUN_REFERENCE_AVOIDANCE_AUTOLOCATE(runPerformance, runOriginalReplay, commonRoot)
%RUN_REFERENCE_AVOIDANCE_AUTOLOCATE Find manually extracted source and tests.
% No ZIP extraction, no assert, and no fixed assumption about an extra ZIP folder.
% Usage:
%   R = RUN_REFERENCE_AVOIDANCE_AUTOLOCATE(true, false);
%   R = RUN_REFERENCE_AVOIDANCE_AUTOLOCATE(true, false, 'C:\path\common_matlab');
if nargin < 1 || isempty(runPerformance), runPerformance = true; end
if nargin < 2 || isempty(runOriginalReplay), runOriginalReplay = false; end
if nargin < 3 || isempty(commonRoot)
    commonRoot = fileparts(mfilename('fullpath'));
end
R = struct('status','NOT_RUN','commonRoot',commonRoot, ...
    'moduleRoot','','testRoot','','errorText','');
if ~isfolder(commonRoot)
    R.status = 'COMMON_ROOT_NOT_FOUND';
    R.errorText = sprintf('Folder not found: %s', commonRoot);
    fprintf(2,'%s\n',R.errorText);
    return;
end
% Find the module even if Windows extracted the package one level deeper.
found = dir(fullfile(commonRoot,'**','SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m'));
if isempty(found)
    R.status = 'MODULE_NOT_FOUND';
    R.errorText = ['SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m was not found below ' commonRoot];
    fprintf(2,'%s\n',R.errorText);
    fprintf('Please verify the actual folder location after manual extraction.\n');
    return;
end
% Prefer the intended layout directly beneath common_matlab, then the first
% fully paired nested layout. Do not combine unrelated copies.
preferred = fullfile(commonRoot,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE');
chosen = '';
chosenTests = '';
for pass = 1:2
    for i = 1:numel(found)
        module = found(i).folder;
        if pass == 1 && ~strcmpi(module, preferred), continue; end
        if pass == 2 && strcmpi(module, preferred), continue; end
        packageRoot = fileparts(fileparts(module));
        tests = fullfile(packageRoot,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS');
        if isfile(fullfile(tests,'RUN_REFERENCE_AVOIDANCE_TESTS.m'))
            chosen = module;
            chosenTests = tests;
            break;
        end
    end
    if ~isempty(chosen), break; end
end
if isempty(chosen)
    R.status = 'SOURCE_AND_TESTS_NOT_PAIRED';
    R.errorText = 'Found the source but not a matching sibling test folder.';
    fprintf(2,'%s\n',R.errorText);
    for i = 1:numel(found)
        fprintf('Source found: %s\n',found(i).folder);
        fprintf('Expected test folder beside WAYPOINT_REFERENCE: %s\n', ...
            fullfile(fileparts(fileparts(found(i).folder)), ...
                'REFERENCE_OBSTACLE_AVOIDANCE_TESTS'));
    end
    return;
end
R.moduleRoot = chosen;
R.testRoot = chosenTests;
fprintf('Detected module: %s\n',chosen);
fprintf('Detected tests:  %s\n',chosenTests);
addpath(chosenTests,'-begin');
rehash;
try
    R = RUN_REFERENCE_AVOIDANCE_TESTS(runPerformance, runOriginalReplay);
    fprintf('Test status: %s\n',R.status);
catch ME
    R.status = 'TEST_EXECUTION_FAILED';
    R.errorText = getReport(ME,'extended','hyperlinks','off');
    R.moduleRoot = chosen;
    R.testRoot = chosenTests;
    fprintf(2,'%s\n',R.errorText);
end
end
