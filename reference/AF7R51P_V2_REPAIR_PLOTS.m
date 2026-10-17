function OUT = AF7R51P_V2_REPAIR_PLOTS()
% Repair ONLY run_flight_headless post-run plotting.
% REPLANNING_SYSTEM / AF7R51 safety or timing code is not touched.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R51P V2 | WINDOWS-SAFE PLOT REPAIR\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] restoreKnownBackupFirst=1 | postRunPlotOnly=1 | runtimeLoopUnchanged=1 | safetyLogicUnchanged=1\n');

src=which('run_flight_headless');
assert(~isempty(src)&&isfile(src),'[AF7R51P V2] run_flight_headless.m unresolved.');
folder=fileparts(src);

% The V1 installer created the backup BEFORE writing the broken plot block.
D=dir(fullfile(folder,'run_flight_headless_pre_AF7R51P_*.m'));
assert(~isempty(D), ...
    '[AF7R51P V2] no pre-AF7R51P backup found; no write performed.');
[~,ix]=max([D.datenum]);
bak=fullfile(D(ix).folder,D(ix).name);

raw0=fileread(bak);
startMarker='disp("Simulation complete! Drawing plots...");';
endMarker='%% LOCAL FUNCTIONS';
a=strfind(raw0,startMarker);
b=strfind(raw0,endMarker);
assert(numel(a)==1,'[AF7R51P V2] start marker count=%d, expected 1.',numel(a));
assert(numel(b)==1,'[AF7R51P V2] end marker count=%d, expected 1.',numel(b));
assert(b(1)>a(1),'[AF7R51P V2] marker order invalid.');

% Find the actual end of the "Simulation complete" line in either CRLF/LF.
tail=raw0(a(1):end);
lf=find(tail==char(10),1,'first');
assert(~isempty(lf),'[AF7R51P V2] line ending after start marker not found.');
replaceStart=a(1)+lf;   % first char after LF
replaceEnd=b(1)-1;      % char immediately before %% LOCAL FUNCTIONS

NL=newline;
lines={ ...
    ''; ...
    '%% AF7R51 single post-run dashboard -- replaces fragmented Figures 1--11.'; ...
    'assert(exist(''replanner_instance'',''var'')==1, ...'; ...
    '    ''[AF7R51P] replanner_instance is unavailable after the run.'');'; ...
    'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);'; ...
    'drawnow;'; ...
    ''; ...
    ''};
newBlock=strjoin(lines,NL);

patched=[raw0(1:replaceStart-1) newBlock raw0(replaceEnd+1:end)];

% Resolve-before-write structural checks.
assert(~contains(patched,'\n%% AF7R51'), ...
    '[AF7R51P V2] literal backslash-n remains in generated source.');
assert(contains(patched, ...
    'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);'));
assert(~contains(patched, ...
    'logger.plot({1,"controller.result.simulation_plant_total"'));
assert(~contains(patched,'plot_runtime_profile(runtime_profile,10,11);'));
assert(~contains(patched,'plot_suspended_load_trajectory(logger,dt, ...'));

% Ensure local functions still exist after the replacement.
assert(contains(patched,'function y = motive_output(obj,data)'));
assert(contains(patched,'function do_prop_local('));
assert(contains(patched,'function plot_runtime_profile('));
assert(contains(patched,'function plot_suspended_load_trajectory('));

% Write a temporary candidate first.
tmp=[tempname(folder) '.m'];
fid=fopen(tmp,'w');
assert(fid>=0,'[AF7R51P V2] temp file open failed.');
c=onCleanup(@()fclose(fid));
fwrite(fid,patched,'char');
clear c;

% Lightweight lexical sanity checks before replacing the real script.
tmpRaw=fileread(tmp);
assert(~contains(tmpRaw,'\n%% AF7R51'));
assert(numel(strfind(tmpRaw,startMarker))==1);
assert(numel(strfind(tmpRaw,endMarker))==1);

% Put the dashboard helper beside run_flight_headless.
here=fileparts(mfilename('fullpath'));
helper=fullfile(here,'AF7R51_POSTRUN_DASHBOARD.m');
assert(isfile(helper),'[AF7R51P V2] dashboard helper missing.');
helperDst=fullfile(folder,'AF7R51_POSTRUN_DASHBOARD.m');
[ok,msg]=copyfile(helper,helperDst,'f');
assert(ok,'[AF7R51P V2] helper copy failed: %s',msg);

% Preserve the current broken V1 file for forensic recovery, then write V2.
stamp=datestr(now,'yyyymmdd_HHMMSS');
brokenBak=fullfile(folder,['run_flight_headless_broken_AF7R51P_V1_' stamp '.m']);
[ok,msg]=copyfile(src,brokenBak,'f');
assert(ok,'[AF7R51P V2] broken-file backup failed: %s',msg);

[ok,msg]=copyfile(tmp,src,'f');
assert(ok,'[AF7R51P V2] repaired source write failed: %s',msg);
delete(tmp);
rehash;

check=fileread(src);
closed=contains(check, ...
        'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);') && ...
    ~contains(check,'\n%% AF7R51') && ...
    ~contains(check,'logger.plot({1,"controller.result.simulation_plant_total"') && ...
    ~contains(check,'plot_runtime_profile(runtime_profile,10,11);') && ...
    contains(check,'%% LOCAL FUNCTIONS') && ...
    contains(check,'function y = motive_output(obj,data)');

fprintf(' repaired from clean backup              = %s\n',bak);
fprintf(' broken V1 preserved                     = %s\n',brokenBak);
fprintf(' run script                              = %s\n',src);
fprintf(' dashboard helper                        = %s\n',helperDst);
fprintf(' literal \\n in generated MATLAB source  = 0\n');
fprintf(' runtime loop modified                   = 0\n');
fprintf(' REPLANNING_SYSTEM modified              = 0\n');
fprintf(' AF7R51P V2 repair closed                = %d\n',closed);
fprintf('====================================================================================================\n');
assert(closed,'[AF7R51P V2] post-write audit failed.');

OUT=struct( ...
    'schema','AF7R51P_V2_PLOT_REPAIR', ...
    'closed',closed, ...
    'source',src, ...
    'restored_from',bak, ...
    'broken_backup',brokenBak, ...
    'dashboard_helper',helperDst);
assignin('base','AF7R51P_V2_REPAIR_OUT',OUT);
end
