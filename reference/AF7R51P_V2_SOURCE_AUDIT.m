function OUT = AF7R51P_V2_SOURCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R51P V2 | RUN SCRIPT SOURCE AUDIT\n');
fprintf('====================================================================================================\n');

src=which('run_flight_headless');
assert(~isempty(src)&&isfile(src));
raw=fileread(src);

C=struct();
C.dashboard_call=contains(raw, ...
    'AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner_instance);');
C.no_literal_backslash_n=~contains(raw,'\n%% AF7R51');
C.local_functions=contains(raw,'%% LOCAL FUNCTIONS');
C.motive_function=contains(raw,'function y = motive_output(obj,data)');
C.prop_function=contains(raw,'function do_prop_local(');
C.old_figure9_call=contains(raw, ...
    'logger.plot({1,"controller.result.simulation_plant_total"');
C.old_runtime_profile_call=contains(raw,'plot_runtime_profile(runtime_profile,10,11);');
C.old_trajectory_call=contains(raw,'plot_suspended_load_trajectory(logger,dt, ...');

closed=C.dashboard_call && C.no_literal_backslash_n && C.local_functions && ...
    C.motive_function && C.prop_function && ...
    ~C.old_figure9_call && ~C.old_runtime_profile_call && ~C.old_trajectory_call;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-40s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' audit closed                             = %d\n',closed);
fprintf('====================================================================================================\n');
assert(closed,'[AF7R51P V2] source audit failed.');

OUT=struct('schema','AF7R51P_V2_SOURCE_AUDIT','closed',closed,'checks',C);
assignin('base','AF7R51P_V2_SOURCE_AUDIT_OUT',OUT);
end
