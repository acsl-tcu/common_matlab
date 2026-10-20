function AF7R58ZZY_R16_ROLLBACK
% Restore exactly the runner state that existed immediately before R16.
runnerPath=which('run_flight_headless');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R16] run_flight_headless.m not found.');
root=fileparts(runnerPath);
backupPath=fullfile(root,'run_flight_headless.R16_PREINSTALL_BACKUP.m');
assert(isfile(backupPath),'[R16] R16 preinstall backup is missing.');
copyfile(backupPath,runnerPath,'f');
rehash;
fprintf('[R16] Runner restored from %s\n',backupPath);
fprintf('[R16] REPLANNING_SYSTEM.m was never modified by R16.\n');
end
