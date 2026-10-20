function AF7R58ZZY_R21_ROLLBACK
% R21 rollback target is the validated R19 numerical/runtime authority.
runnerPath=which('run_flight_headless');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R21 rollback] runner not found.');
root=fileparts(runnerPath);
assert(exist('REPLANNING_SYSTEM_R19_FAST','class')==8, ...
    '[R21 rollback] R19 class unavailable; reinstall R19 package first.');
txt=fileread(runnerPath);
r21='replanner_instance = REPLANNING_SYSTEM_R21_BALANCED(agent,nominal_ref,replan_opts);';
r19='replanner_instance = REPLANNING_SYSTEM_R19_FAST(agent,nominal_ref,replan_opts);';
if contains(txt,r21)
    assert(count(txt,r21)==1,'[R21 rollback] R21 anchor not unique.');
    txt=strrep(txt,r21,r19);
    fid=fopen(runnerPath,'w','n','UTF-8'); assert(fid>=0); c=onCleanup(@()fclose(fid)); fwrite(fid,txt,'char');
elseif contains(txt,r19)
    fprintf('[R21 rollback] runner is already R19.\n');
    return;
else
    error('[R21 rollback] runner is neither R21 nor R19; refusing blind rewrite.');
end
rehash;
fprintf('[R21 rollback] runner restored to REPLANNING_SYSTEM_R19_FAST.\n');
end
