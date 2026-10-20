function OUT=AF7R58ZZY_R31_RUN_CANONICAL()
root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
expectedClass='REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE';
assert(isfile(runner),'[R31 RUN] canonical runner missing.');
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;
txt=fileread(runner);
assert(contains(txt,[expectedClass '(agent,nominal_ref,replan_opts)']), ...
    '[R31 RUN] canonical runner is not bound to R31.');
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R31 | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('runner = %s\n',runner);
fprintf('class  = %s\n',expectedClass);
assignin('base','AF7R58ZZY_R31_ABSOLUTE_RUNNER',runner);
evalin('base','run(AF7R58ZZY_R31_ABSOLUTE_RUNNER);');
OUT=struct('closed',true,'runner',runner,'class',expectedClass);
assignin('base','AF7R58ZZY_R31_RUN_CANONICAL_OUT',OUT);
end
