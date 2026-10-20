function OUT=AF7R58ZZY_R30_RUN_CANONICAL()
% Run canonical R30 runner by absolute path in BASE workspace.
root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
expectedClass='REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT';
assert(isfile(runner),'[R30 RUN] canonical runner missing.');
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;
txt=fileread(runner);
assert(contains(txt,[expectedClass '(agent,nominal_ref,replan_opts)']), ...
    '[R30 RUN] canonical runner is not bound to R30.');
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R30 | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('runner = %s\n',runner);
fprintf('class  = %s\n',expectedClass);
assignin('base','AF7R58ZZY_R30_ABSOLUTE_RUNNER',runner);
evalin('base','run(AF7R58ZZY_R30_ABSOLUTE_RUNNER);');
OUT=struct('closed',true,'runner',runner,'class',expectedClass);
assignin('base','AF7R58ZZY_R30_RUN_CANONICAL_OUT',OUT);
end
