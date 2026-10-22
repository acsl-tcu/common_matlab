function Q = AF7R58R23_INSTALL_SPIKE_PROBE()
% One-time, source-checked setup for timing the ACTUAL full Flight loop.
% The headless runner is not replaced by an archived copy: the current
% run_flight_headless.m is patched ONLY if all required anchors match once.
% Does not alter any reference, controller, plant or safety acceptance logic.
runner=which('run_flight_headless');
if isempty(runner)
    replannerSource=which('REPLANNING_SYSTEM');
    if ~isempty(replannerSource)
        candidate=fullfile(fileparts(fileparts(replannerSource)), ...
            'run_flight_headless.m');
        if isfile(candidate),runner=candidate;end
    end
end
assert(~isempty(runner) && endsWith(runner,'.m'), ...
    '[R23] Current run_flight_headless.m not found.');
S=fileread(runner);
tag='% AF7R58R23_SPIKE_PROBE_INSTALLED';
if contains(S,tag)
    Q=struct('runner',runner,'already_installed',true,'patched',false);
    fprintf('[R23] Full-loop probe already installed: %s\n',runner);
    return;
end
startTag='%% 8. Headless simulation loop';
stopTag='% Trim preallocated timing arrays to actual logged length.';
si=strfind(S,startTag); ei=strfind(S,stopTag);
assert(numel(si)==1 && numel(ei)==1 && si<ei, ...
    '[R23] Flight-loop source structure differs; runner NOT modified.');
head=S(1:si-1); loop=S(si:ei-1); tail=S(ei:end);
assert(contains(loop,'while time.t <= time.te'), ...
    '[R23] Flight loop not found; no change.');
% Add exactly one set of arrays, outside the Flight loop.
base='runtime_profile.k = 0;';
new=[base newline ...
    'runtime_profile.r23_sensor_s = nan(size(ts:dt:te));' newline ...
    'runtime_profile.r23_estimator_s = nan(size(ts:dt:te));' newline ...
    'runtime_profile.r23_reference_s = nan(size(ts:dt:te));' newline ...
    'runtime_profile.r23_controller_s = nan(size(ts:dt:te));' newline ...
    'runtime_profile.r23_input_s = nan(size(ts:dt:te));' newline ...
    'runtime_profile.r23_other_s = nan(size(ts:dt:te));' newline tag];
head=r23_replace_once(head,base,new,'runtime-profile setup');
props={'sensor','estimator','reference','controller','input_transform'};
fields={'r23_sensor_s','r23_estimator_s','r23_reference_s','r23_controller_s','r23_input_s'};
for j=1:numel(props)
    p=props{j}; v=fields{j};
    old=sprintf('    do_prop_local(time,cha,cha,logger,env,agent,"%s",1,true);',p);
    repl=sprintf('    r23_step=tic;\n%s\n    %s=toc(r23_step);',old,v);
    loop=r23_replace_once(loop,old,repl,p);
end
% Save the exact time series, not just an overwritten last-event scalar.
needle='    runtime_profile.full_iteration_s(rk)=full_iteration_s;';
extra={};
for j=1:numel(fields)
    f=fields{j};
    extra{end+1}=sprintf('    runtime_profile.%s(rk)=%s;',f,f); %#ok<AGROW>
end
extra{end+1}=['    runtime_profile.r23_other_s(rk)=' ...
    '(full_iteration_s-(sensor_acquisition_s+' ...
    'r23_sensor_s+r23_estimator_s+r23_reference_s+' ...
    'r23_controller_s+r23_input_s+simulation_plant_s+logger_s));'];
loop=r23_replace_once(loop,needle,[needle newline strjoin(extra,newline)],'per-cycle time-series recording');
% All file I/O occurs strictly AFTER the Flight loop has finished.
anchor='disp("Simulation complete! Drawing plots...");';
append=['r23_fields={''r23_sensor_s'',''r23_estimator_s'',''r23_reference_s'',' newline ...
    '    ''r23_controller_s'',''r23_input_s'',''r23_other_s''};' newline ...
    'for r23_j=1:numel(r23_fields)' newline ...
    '    r23_f=r23_fields{r23_j};' newline ...
    '    r23_v=runtime_profile.(r23_f);' newline ...
    '    runtime_profile.(r23_f)=r23_v(1:Krt);' newline ...
    'end' newline ...
    'AF7R58R23_SPIKE_REPORT(runtime_profile);' newline newline anchor];
tail=r23_replace_once(tail,anchor,append,'post-Flight reporting');
% Validate every edit BEFORE overwriting the user's source.
patched=[head loop tail];
assert(contains(patched,tag) && ...
    numel(strfind(patched,'AF7R58R23_SPIKE_REPORT(runtime_profile)'))==1 && ...
    numel(strfind(patched,'runtime_profile.r23_reference_s(rk)='))==1, ...
    '[R23] Incomplete patch; runner NOT modified.');
backup=fullfile(tempdir,['AF7R58R23_RUNNER_BACKUP_' datestr(now,'yyyymmdd_HHMMSS_FFF') '.m']);
[ok,msg]=copyfile(runner,backup);
assert(ok,'[R23] Cannot back up runner: %s',msg);
fid=fopen(runner,'w','n','UTF-8');
assert(fid>=0,'[R23] Cannot write runner; backup: %s',backup);
fprintf(fid,'%s',patched);fclose(fid);
rehash;
setappdata(0,'AF7R58R23_RUNNER_BACKUP',backup);
Q=struct('runner',runner,'backup',backup, ...
    'already_installed',false,'patched',true, ...
    'safety_logic_modified',false,'flight_disk_io',false);
fprintf('[R23] LOOP PROBE INSTALLED (source-bound).\nRunner: %s\nBackup: %s\n',runner,backup);
end

function S=r23_replace_once(S,from,to,what)
n=numel(strfind(S,from));
assert(n==1,'[R23] %s source anchor count=%d, expected 1; no write.',what,n);
S=strrep(S,from,to);
end
