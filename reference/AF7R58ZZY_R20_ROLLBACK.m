function AF7R58ZZY_R20_ROLLBACK
runnerPath=which('run_flight_headless');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R20] runner missing.');
root=fileparts(runnerPath);
backup=fullfile(root,'run_flight_headless.R20_PREINSTALL_BACKUP.m');
assert(isfile(backup),'[R20] R20 preinstall runner backup is missing.');
copyfile(backup,runnerPath,'f');
rehash;
fprintf('[R20] Runner restored from %s\n',backup);
fprintf('[R20] REPLANNING_SYSTEM.m was never modified by R20.\n');
end
