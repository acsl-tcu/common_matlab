function R=AF7R58R44R2_CHECK()
% First try existing actual saved snapshot; then a live base-workspace handle.
% Never create fictitious x_est/Creturn/REPLANNING_SYSTEM objects.
fn=fullfile(fileparts(mfilename('fullpath')), ...
    'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
if isfile(fn)
    R=AF7R58R44R2_RETURN_HULL(fn);
    fprintf('[R44R2] Existing snapshot: %s | status=%s\n',fn,R.status);
    return;
end
if evalin('base','exist(''replanner_instance'',''var'') == 1')
    P=evalin('base','replanner_instance');
    R=AF7R58R44R2_CAPTURE(P,fn);
    return;
end
R=struct('schema','AF7R58R44R2_CHECK_V1', ...
    'status','NO_SNAPSHOT_AND_NO_LIVE_INSTANCE__REPLAY_REQUIRED', ...
    'saved',false,'live',false, ...
    'event_log_contains_control_points',false, ...
    'what_to_run','AF7R58R44R2_RUN_AND_CAPTURE');
fprintf(['[R44R2] MAT not present and current workspace has no replanner.\n' ...
    '        The R23 timing MAT records event times, not Bezier polygons.\n' ...
    '        Run AF7R58R44R2_RUN_AND_CAPTURE once.\n']);
end
