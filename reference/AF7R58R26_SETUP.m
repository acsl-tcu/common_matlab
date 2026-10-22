function Q = AF7R58R26_SETUP()
% R26 simulation-only commit experiment plus deferred flight diagnostics.
% Only modifies the headless simulator's constructor options; never rewrites
% the real controller/plant, timing thresholds, or physical certificates.
% Use the installed file as the path authority instead of hardcoded paths.
cls=which('REPLANNING_SYSTEM');
assert(~isempty(cls) && isfile(cls), '[R26] REPLANNING_SYSTEM not on path.');
ref=fileparts(cls);
src=fileread(cls);
assert(contains(src,'r26_materialize_deferred_diagnostics') && ...
       contains(src,'R26_SIMULATION_ONLY_EXPERIMENT') && ...
       contains(src,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS'), ...
    '[R26] R26 REPLANNING_SYSTEM.m is not installed. No runner modified.');
assert(exist('af7r36_single_core_mex','file') == 3, ...
    '[R26] Current R23+ compiled native MEX missing.');
runner=which('run_flight_headless');
if isempty(runner)
    runner=fullfile(fileparts(ref),'run_flight_headless.m');
end
assert(isfile(runner), '[R26] headless simulator not found.');
code=fileread(runner);
assert(contains(code,'% AF7R58R25_LOGGER_INSTALLED') && ...
       contains(code,'replanner_instance.r25_publish_capture();') && ...
       contains(code,'SIM_NUMERIC_TRUTH'), ...
    '[R26] R25 diagnostic logger/flush not installed. No modification made.');
anchor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
assert(numel(strfind(code,anchor))==1, ...
    '[R26] headless constructor anchor not unique. No modification made.');
token='replan_opts.R26_SIMULATION_ONLY_EXPERIMENT = true;';
if ~contains(code,token)
    add=['% R26: HEADLESS SIMULATION ONLY. Exercises original atomic reference-only' newline ...
         '% commit after all reference-level gates. NOT real-time/flight-safe.' newline ...
         token newline anchor];
    edited=strrep(code,anchor,add);
    assert(numel(strfind(edited,token))==1,'[R26] Installer postcondition.');
    backup=fullfile(tempdir, ['AF7R58R26_HEADLESS_BACKUP_' ...
        datestr(now,'yyyymmdd_HHMMSS_FFF') '.m']);
    [ok,msg]=copyfile(runner,backup);
    assert(ok,'[R26] Backup failed: %s',msg);
    fid=fopen(runner,'w','n','UTF-8');
    assert(fid>=0,'[R26] Unable to open runner after backup.');
    cleanup=onCleanup(@()fclose(fid));
    fwrite(fid,edited,'char');
    clear cleanup;
    patched=true;
else
    patched=false;
    backup='';
end
rehash;
Q=struct('class_source',cls,'runner',runner, ...
    'runner_patched',logical(patched),'backup',backup, ...
    'sim_only_commit_opt_in',true,'execution_tracking_tube_complete',false, ...
    'stage1_stage2_deadline_bypass',true, ...
    'flight_suitable',false,'mex_rebuild_required',false);
fprintf('[R26] SIM ONLY | deferred Stage1 diagnostics | optional atomic reference commit ENABLED.\n');
fprintf('[R26] Original stage safety certificates retained; 25-ms violation is NOT flight-safe.\n');
end
