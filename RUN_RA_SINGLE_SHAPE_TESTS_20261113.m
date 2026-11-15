function R=RUN_RA_SINGLE_SHAPE_TESTS_20261113(commonRoot)
%RUN_RA_SINGLE_SHAPE_TESTS_20261113 Placement-independent manual-unzip test.
% NO zip extraction and NO forced sibling-folder layout.
if nargin<1 || isempty(commonRoot),commonRoot=pwd;end
moduleDirs={fullfile(commonRoot,'reference','WAYPOINT_REFERENCE', ...
   'REFERENCE_OBSTACLE_AVOIDANCE'), ...
   fullfile(commonRoot,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE')};
root='';
for i=1:numel(moduleDirs)
    if isfile(fullfile(moduleDirs{i},'SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m'))
        root=moduleDirs{i};break;
    end
end
if isempty(root)
    error('RA:Layout','Cannot find reference/WAYPOINT_REFERENCE/REFERENCE_OBSTACLE_AVOIDANCE');
end
addpath(root,'-begin'); SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
testDir=fullfile(commonRoot,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS');
if ~isfolder(testDir)
    error('RA:Layout','Cannot find tests at %s',testDir);
end
addpath(fullfile(testDir,'SUPPORT'),'-begin');
addpath(fullfile(testDir,'08_SINGLE_SHAPE_AND_FLIGHT'),'-begin');
% This source is original, not duplicated into the sensor patch.
if exist('DRONE_OBSTACLES','file')~=2
    error('RA:Layout','Add your ORIGINAL DRONE_OBSTACLES folder to MATLAB path.');
end
R=RUN_RA_SINGLE_SHAPE_TESTS();
disp(R.status);disp(R.result);disp(R.resultsZip);
end
