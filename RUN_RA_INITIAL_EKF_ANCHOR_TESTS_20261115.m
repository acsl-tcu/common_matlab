function OUT=RUN_RA_INITIAL_EKF_ANCHOR_TESTS_20261115(root)
% Test ONLY. No changes to run, HLC, EKF or original DRONE_OBSTACLES.
if nargin<1 || isempty(root),root=pwd;end
root=char(root);
modu=fullfile(root,'reference','WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE');
addpath(genpath(fullfile(root,'reference')),'-end');
addpath(modu,'-begin');
SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
testDir=fullfile(root,'REFERENCE_OBSTACLE_AVOIDANCE_TESTS','10_FORECAST_AUDIT');
addpath(testDir,'-begin');
OUT=TEST_RA_V8_INITIAL_EKF_ANCHOR();
disp(OUT);
end
