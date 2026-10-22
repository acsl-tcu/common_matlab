function Q = AF7R58R27_SETUP()
% R27: source-bound observer failure telemetry + sim-only commit opt-in audit.
% Does not modify observer acceptance, deadlines, or the real flight system.
cls = which('REPLANNING_SYSTEM');
assert(~isempty(cls) && isfile(cls), '[R27] REPLANNING_SYSTEM.m missing.');
src = fileread(cls);
assert(contains(src,'AF7R58R27_ROLL_LIFT_FAILURE_V1') && ...
       contains(src,'R27_STAGE3_RUNTIME_SIM_FLAG_FALSE') && ...
       contains(src,'R27_STAGE3_RUNTIME_SIM_FLAG_TRUE'), ...
       '[R27] REPLANNING_SYSTEM.m is not the R27 file.');
assert(exist('AF7R58R26_SETUP','file') == 2, ...
       '[R27] Keep AF7R58R26_SETUP.m from R26 installed.');
Q26 = AF7R58R26_SETUP();
runner = Q26.runner;
a = fileread(runner);
token = 'replan_opts.R26_SIMULATION_ONLY_EXPERIMENT = true;';
anchor = 'replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
assert(contains(a,token) && numel(strfind(a,anchor)) == 1 && ...
       strfind(a,token) < strfind(a,anchor), ...
       '[R27] Headless opt-in was not placed before construction.');
marker = '% AF7R58R27_RUNTIME_SIM_FLAG_ASSERT';
patched = false; backup='';
if ~contains(a,marker)
    addition = [anchor newline ...
        marker newline ...
        'assert(logical(replanner_instance.R26_SIMULATION_ONLY_EXPERIMENT), ...' newline ...
        '    ''[R27] Sim-only commit opt-in was lost during construction.'');' newline ...
        'fprintf(''[R27] effective sim-only commit flag = %d\n'', ...' newline ...
        '    double(replanner_instance.R26_SIMULATION_ONLY_EXPERIMENT));'];
    b = strrep(a,anchor,addition);
    assert(numel(strfind(b,marker)) == 1, '[R27] Runner patch mismatch.');
    backup = fullfile(tempdir,['AF7R58R27_RUNNER_BACKUP_' ...
        datestr(now,'yyyymmdd_HHMMSS_FFF') '.m']);
    [ok,msg]=copyfile(runner,backup);
    assert(ok,'[R27] Cannot back up simulator: %s',msg);
    fid=fopen(runner,'w','n','UTF-8');
    assert(fid>=0,'[R27] Cannot open runner.');
    cc=onCleanup(@()fclose(fid));
    fwrite(fid,b,'char');
    clear cc;
    patched=true;
end
rehash;
% Old snapshot belongs to earlier failed runs; prevent misleading results.
if isappdata(0,'AF7R58R27_ROLL_FAILURE')
    rmappdata(0,'AF7R58R27_ROLL_FAILURE');
end
Q=struct('source',cls,'runner',runner,'runner_patched',patched, ...
    'backup',backup,'rebuild_mex_required',false, ...
    'observer_fail_closed_unchanged',true, ...
    'sim_only_late_continue',true,'flight_suitable',false);
fprintf('[R27] SETUP PASS: observer proof unchanged, sim-only runtime flag asserted.\n');
end
