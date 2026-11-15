function R=RUN_RA_SENSOR_TESTS_20261113(commonRoot,runPerformance)
%RUN_RA_SENSOR_TESTS_20261113 Independent test launcher for manual install.
% Does not unzip, move folders, or require source/tests to be siblings.
if nargin<1 || isempty(commonRoot),commonRoot=fileparts(mfilename('fullpath'));end
if nargin<2,runPerformance=true;end
R=struct('status','NOT_RUN','moduleRoot','','testRoot','', ...
    'testResult',struct(),'errorText','');
if ~isfolder(commonRoot)
    R.status='COMMON_ROOT_NOT_FOUND';return;
end
S=dir(fullfile(commonRoot,'**','SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m'));
T=dir(fullfile(commonRoot,'**','RUN_RA_SIMULATED_SENSOR_TESTS.m'));
if isempty(S),R.status='SOURCE_NOT_FOUND';return;end
if isempty(T),R.status='SENSOR_TESTS_NOT_FOUND';return;end
expected=fullfile(commonRoot,'reference','WAYPOINT_REFERENCE', ...
    'REFERENCE_OBSTACLE_AVOIDANCE');
idx=find(strcmpi({S.folder},expected),1);
if isempty(idx)
    expected=fullfile(commonRoot,'WAYPOINT_REFERENCE', ...
        'REFERENCE_OBSTACLE_AVOIDANCE');
    idx=find(strcmpi({S.folder},expected),1);
end
if isempty(idx),idx=1;end
R.moduleRoot=S(idx).folder;
R.testRoot=T(1).folder;
addpath(R.moduleRoot,'-begin');
try
    info=SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
    if ~strcmp(info.root,R.moduleRoot)
        error('RaSensor:WrongSetupSource', ...
            'The setup is resolving from a different module copy.');
    end
    addpath(R.testRoot,'-begin');
    R.testResult=RUN_RA_SIMULATED_SENSOR_TESTS(runPerformance);
    R.status=R.testResult.status;
catch ME
    R.status='FAIL_SENSOR_SUITE';
    R.errorText=getReport(ME,'extended','hyperlinks','off');
    fprintf(2,'%s\n',R.errorText);
end
fprintf('[Sensor launch] %s\nSource: %s\nTests: %s\n', ...
    R.status,R.moduleRoot,R.testRoot);
end
