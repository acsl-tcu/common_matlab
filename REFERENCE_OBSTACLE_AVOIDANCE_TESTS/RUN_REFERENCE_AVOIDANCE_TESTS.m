function R=RUN_REFERENCE_AVOIDANCE_TESTS(runPerformance,runOriginalReplay)
%RUN_REFERENCE_AVOIDANCE_TESTS Independent common_matlab test suite.
% NO Flight, CBF, sensor simulation, or physics safety certification.
if nargin<1,runPerformance=false;end
if nargin<2,runOriginalReplay=false;end
here=fileparts(mfilename('fullpath'));
parent=fileparts(here);
root=fullfile(parent,'WAYPOINT_REFERENCE','REFERENCE_OBSTACLE_AVOIDANCE');
setupFile=fullfile(root,'SETUP_REFERENCE_OBSTACLE_AVOIDANCE.m');
if ~isfile(setupFile)
    error('RefAvoidance:IncorrectPackageLayout', [ ...
        'Installation files are not next to REFERENCE_OBSTACLE_AVOIDANCE_TESTS.\n' ...
        'Expected setup file: %s\n' ...
        'The ZIP must be extracted into common_matlab (not into an extra ZIP-name folder),\n' ...
        'or RUN_REFERENCE_AVOIDANCE_QUICKSTART must be called inside the extracted package.'],setupFile);
end
% The tests can run from a nested archive extraction for diagnosis, but
% report the mismatch instead of silently claiming common_matlab install.
[~,installationParent]=fileparts(parent);
layoutInCommon=strcmpi(installationParent,'common_matlab');
if ~layoutInCommon
    warning('RefAvoidance:NestedExtraction', ...
        'Tests are executing from %s, not common_matlab. Move both top-level folders into common_matlab before Flight integration.',parent);
end
addpath(root,'-begin');
setupInfo=SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
sub={'01_INSTALLATION','02_GEOMETRY','03_MEX','04_TRIGGER', ...
    '05_PERFORMANCE','06_ORIGINAL_TRACE_REPLAY','SUPPORT'};
for k=1:numel(sub),addpath(fullfile(here,sub{k}),'-begin');end
R=struct();R.root=setupInfo.root;
R.layoutInCommonMatlab=layoutInCommon;
R.packageParent=parent;
R.installation=TEST_RA_INSTALLATION();
R.legacySixModeSmoke=TEST_REFERENCE_AVOIDANCE_INSTALL();
R.geometry=TEST_RA_GEOMETRY_6();
R.mex=TEST_RA_MEX_AUTOBUILD();
R.history=TEST_RA_TICK_25MS_HISTORY();
if runPerformance,R.performance=TEST_RA_GEOMETRY_BENCHMARK(30);end
if runOriginalReplay
    R.originalReplay=RUN_RA_OPTIONAL_96_TRACE_REPLAY(3);
end
R.status='PASS_CORE_FUNCTIONAL_NO_REAL_FLIGHT';
if runOriginalReplay && isfield(R,'originalReplay')
    if isfield(R.originalReplay,'status') && startsWith(R.originalReplay.status,'SKIPPED')
        R.status='PASS_CORE_ORIGINAL_REPLAY_SKIPPED';
    end
end
R.physicalSafetyCertified=false;
R.fullPrediction2msProved=false;
R.flight25msProved=false;
outdir=fullfile(here,'RESULTS');
if ~isfolder(outdir),mkdir(outdir);end
stamp=datestr(now,'yyyymmdd_HHMMSS_FFF');
matPath=fullfile(outdir,['REFERENCE_AVOIDANCE_TEST_' stamp '.mat']);
zipPath=fullfile(outdir,['REFERENCE_AVOIDANCE_TEST_' stamp '.zip']);
save(matPath,'R');
zip(zipPath,matPath);
R.resultsZip=zipPath;
fprintf('[ReferenceAvoidance] %s, results=%s\\n',R.status,R.resultsZip);
end
