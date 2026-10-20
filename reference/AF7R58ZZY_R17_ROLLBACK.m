function AF7R58ZZY_R17_ROLLBACK
runnerPath=which('run_flight_headless');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[R17] run_flight_headless.m not found.');
root=fileparts(runnerPath);
backupPath=fullfile(root,'run_flight_headless.R17_PREINSTALL_BACKUP.m');
assert(isfile(backupPath),'[R17] R17 preinstall backup is missing.');
cur=fileread(runnerPath);
anchor='replanner_instance = REPLANNING_SYSTEM_R17_FAST(agent,nominal_ref,replan_opts);';
assert(count(cur,anchor)==1,'[R17] current runner is not uniquely R17-installed.');
copyfile(backupPath,runnerPath,'f');
rehash;
fprintf('[R17] runner restored from %s\n',backupPath);
fprintf('[R17] REPLANNING_SYSTEM.m was never modified by R17.\n');
end
