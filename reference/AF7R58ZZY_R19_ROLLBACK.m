function AF7R58ZZY_R19_ROLLBACK
runnerPath=which('run_flight_headless');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R19] runner missing.');
root=fileparts(runnerPath);
backup=fullfile(root,'run_flight_headless.R19_PREINSTALL_BACKUP.m');
assert(isfile(backup),'[R19] R19 preinstall runner backup is missing.');
copyfile(backup,runnerPath,'f');
rehash;
fprintf('[R19] Runner restored from %s\n',backup);
fprintf('[R19] REPLANNING_SYSTEM.m was never modified by R19.\n');
end
