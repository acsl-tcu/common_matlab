function OUT=AF7R58ZZY_R29_RUN_CANONICAL()
% Run canonical R29 runner by absolute path in the base workspace.
root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
expectedClass='REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT';
assert(isfile(runner),'[R29 RUN] canonical runner missing.');
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;
txt=fileread(runner);
assert(contains(txt,[expectedClass '(agent,nominal_ref,replan_opts)']), ...
    '[R29 RUN] canonical runner is not bound to R29.');
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R29 | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('runner = %s\n',runner);
fprintf('class  = %s\n',expectedClass);
assignin('base','AF7R58ZZY_R29_ABSOLUTE_RUNNER',runner);
evalin('base','run(AF7R58ZZY_R29_ABSOLUTE_RUNNER);');
OUT=struct('closed',true,'runner',runner,'class',expectedClass);
assignin('base','AF7R58ZZY_R29_RUN_CANONICAL_OUT',OUT);
end
