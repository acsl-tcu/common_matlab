function S = RUN_CHECK_TRIGGER_MODEL_SOURCE_BINDING()
% RUN_CHECK_TRIGGER_MODEL_SOURCE_BINDING
% One-command, self-contained diagnostic using the ORIGINAL flight runner.
% Put this file and CHECK_TRIGGER_MODEL_SOURCE_BINDING.m at common_matlab root.
% Runs the original Flight without editing its source, then audits live objects.
% This can take the normal full-flight runtime and opens its usual postrun UI.
% Save files in the same folder as this one, not in production modules.

root = fileparts(mfilename('fullpath'));
runner = fullfile(root,'run_flight_headless.m');
assert(isfile(runner),'Missing original run_flight_headless.m beside the audit.');
% This is a MATLAB SCRIPT. It begins with clearvars and creates agent/time
% in THIS function workspace, which is why the diagnostic can read them.
run(runner);
assert(exist('agent','var')==1 && exist('time','var')==1, ...
    'Flight runner did not leave live agent/time in its script workspace.');
S = CHECK_TRIGGER_MODEL_SOURCE_BINDING(agent,time);
folder = fileparts(which('RUN_CHECK_TRIGGER_MODEL_SOURCE_BINDING'));
if isempty(folder), folder=pwd; end
save(fullfile(folder,'TRIGGER_MODEL_SOURCE_BINDING_RESULT.mat'),'S');
fid=fopen(fullfile(folder,'TRIGGER_MODEL_SOURCE_BINDING_RESULT.txt'),'w');
if fid~=-1
    closer=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\n',S.status);
    fprintf(fid,'active_rhs=%s\n',S.active_rhs);
    fprintf(fid,'active_solver=%s\n',S.active_solver);
    fprintf(fid,'ekf_mass_kg=%.17g\n',S.ekf_mass_kg);
    fprintf(fid,'hlc_mass_source_kg=%.17g\n',S.hlc_mass_source_kg);
    fprintf(fid,'plant_configured_mass_kg=%.17g\n',S.plant_configured_mass_kg);
    fprintf(fid,'controller_mass_equals_estimator=%d\n',S.controller_mass_equals_estimator);
    fprintf(fid,'diagnostic_error=%s\n',S.diagnostic_error);
    clear closer
end
disp(S);
end
