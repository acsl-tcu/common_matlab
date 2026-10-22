function Q = AF7R58R26_DISABLE_SIM_COMMIT()
% Revert only the R26 headless simulation opt-in.  Production deadline logic
% and hard certificates are never modified by this function.
runner=which('run_flight_headless');
assert(~isempty(runner)&&isfile(runner),'[R26] Headless runner missing.');
s=fileread(runner);
old='replan_opts.R26_SIMULATION_ONLY_EXPERIMENT = true;';
new='replan_opts.R26_SIMULATION_ONLY_EXPERIMENT = false;';
assert(numel(strfind(s,old))==1, ...
    '[R26] Expected exactly one enabled simulation token.');
s=strrep(s,old,new);
backup=fullfile(tempdir,['AF7R58R26_DISABLED_BACKUP_' ...
    datestr(now,'yyyymmdd_HHMMSS_FFF') '.m']);
[ok,msg]=copyfile(runner,backup);
assert(ok,'[R26] Backup failed: %s',msg);
fid=fopen(runner,'w','n','UTF-8');
assert(fid>=0,'[R26] Cannot open runner.');
cleanup=onCleanup(@()fclose(fid));
fwrite(fid,s,'char');
clear cleanup;
rehash;
Q=struct('runner',runner,'simulation_commit_enabled',false,'backup',backup);
fprintf('[R26] Simulation-only reference commit DISABLED in the runner.\n');
end
