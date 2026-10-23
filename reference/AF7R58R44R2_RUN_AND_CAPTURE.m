% AF7R58R44R2_RUN_AND_CAPTURE (SCRIPT, not a function)
% Execute the UNMODIFIED headless simulation and capture the actual active
% REPAIRED correction polygons immediately after it returns.
% No files are written from the Flight loop by this script.
% run_flight_headless.m begins with `clearvars`; therefore determine paths
% afresh AFTER the run; do not rely on local values created before it.
run(fullfile(fileparts(which('AF7R58R44R2_RUN_AND_CAPTURE')), ...
    '..','run_flight_headless.m'));
if exist('replanner_instance','var')~=1
    error(['[R44R2] The unmodified headless script finished without ' ...
        'a replanner_instance. Cannot capture actual Bezier polygons.']);
end
R44R2_capture=AF7R58R44R2_CAPTURE(replanner_instance, ...
    fullfile(fileparts(which('AF7R58R44R2_RUN_AND_CAPTURE')), ...
    'AF7R58R44R2_ACTIVE_SNAPSHOT.mat'));
disp('========== R44R2 ACTUAL POSTRUN CAPTURE ==========');
disp(R44R2_capture);
if R44R2_capture.return_hull_available
    disp('========== R44R2 ACTUAL CONTROL POINT HULL ==========');
    disp(R44R2_capture.return_hull.segments);
else
    fprintf(['[R44R2] Snapshot saved, but active polygons unavailable (%s).\n' ...
        '         Do not infer missing control points from R23 timing MAT.\n'], ...
        R44R2_capture.status);
end
