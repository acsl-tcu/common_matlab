function OUT = AF7R51_INSTALL_PLOT_REPLACEMENT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R51P | RUN_FIGHT_HEADLESS PLOT REPLACEMENT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] postRunPlotOnly=1 | runtimeLoopUnchanged=1 | safetyLogicUnchanged=1\n');

src=which('run_flight_headless');
assert(~isempty(src)&&isfile(src),'[AF7R51P] run_flight_headless.m unresolved.');
raw=fileread(src);

startMarker='disp("Simulation complete! Drawing plots...");';
endMarker='%% LOCAL FUNCTIONS';
a=strfind(raw,startMarker);
b=strfind(raw,endMarker);
assert(numel(a)==1,'[AF7R51P] plot-start marker count=%d, expected 1.',numel(a));
assert(numel(b)==1,'[AF7R51P] LOCAL FUNCTIONS marker count=%d, expected 1.',numel(b));
assert(b(1)>a(1),'[AF7R51P] marker order invalid.');

lineEnd=find(raw(a(1):end)==newline,1,'first');
assert(~isempty(lineEnd),'[AF7R51P] plot-start line end unresolved.');
replaceStart=a(1)+lineEnd; % immediately after newline
replaceEnd=b(1)-1;

newBlock=sprintf([ ...
    '\\n%% AF7R51 single post-run dashboard -- replaces fragmented Figures 1--11.\\n' ...
    'assert(exist(''replanner_instance'',''var'')==1, ...\\n' ...
    '    ''[AF7R51P] replanner_instance is unavailable after the run.'');\\n' ...
    'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);\\n' ...
    'drawnow;\\n\\n']);

patched=[raw(1:replaceStart-1) newBlock raw(replaceEnd+1:end)];

% Resolve-before-write audit.
assert(contains(patched,'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);'));
assert(~contains(patched,'logger.plot({1,"controller.result.simulation_plant_total"'));
assert(~contains(patched,'plot_runtime_profile(runtime_profile,10,11);'));
assert(~contains(patched,'plot_suspended_load_trajectory(logger,dt, ...'));

here=fileparts(mfilename('fullpath'));
helper=fullfile(here,'AF7R51_POSTRUN_DASHBOARD.m');
assert(isfile(helper),'[AF7R51P] dashboard helper missing.');

stamp=datestr(now,'yyyymmdd_HHMMSS');
bak=fullfile(fileparts(src),['run_flight_headless_pre_AF7R51P_' stamp '.m']);
[ok,msg]=copyfile(src,bak,'f'); assert(ok,'[AF7R51P] backup failed: %s',msg);

helperDst=fullfile(fileparts(src),'AF7R51_POSTRUN_DASHBOARD.m');
[ok,msg]=copyfile(helper,helperDst,'f'); assert(ok,'[AF7R51P] helper copy failed: %s',msg);

fid=fopen(src,'w'); assert(fid>=0,'[AF7R51P] cannot open run script for write.');
c=onCleanup(@()fclose(fid)); fwrite(fid,patched,'char'); clear c;
rehash;

check=fileread(src);
closed=contains(check,'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);') && ...
    ~contains(check,'logger.plot({1,"controller.result.simulation_plant_total"') && ...
    ~contains(check,'plot_runtime_profile(runtime_profile,10,11);');

fprintf(' run script                              = %s\n',src);
fprintf(' backup                                  = %s\n',bak);
fprintf(' dashboard helper                        = %s\n',helperDst);
fprintf(' old fragmented runtime/trajectory plots = REMOVED\n');
fprintf(' new Figure 5                            = integrated trajectory/avoidance\n');
fprintf(' new Figure 10                           = integrated timing dashboard\n');
fprintf(' AF7R51 plot replacement closed          = %d\n',closed);
fprintf('====================================================================================================\n');
assert(closed,'[AF7R51P] post-write audit failed.');

OUT=struct('schema','AF7R51P_PLOT_INSTALL_V1','closed',closed, ...
    'source',src,'backup',bak,'helper',helperDst);
assignin('base','AF7R51_PLOT_INSTALL_OUT',OUT);
end
