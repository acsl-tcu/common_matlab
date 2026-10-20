function OUT=AF7R58ZZY_R33_RUN_CANONICAL()
root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
expectedClass='REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW';
assert(isfile(runner),'[R33 RUN] canonical runner missing.');
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;
txt=fileread(runner);
assert(contains(txt,[expectedClass '(agent,nominal_ref,replan_opts)']), ...
    '[R33 RUN] canonical runner is not bound to R33.');
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R33 | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('runner = %s\n',runner);
fprintf('class  = %s\n',expectedClass);
assignin('base','AF7R58ZZY_R33_ABSOLUTE_RUNNER',runner);
evalin('base','run(AF7R58ZZY_R33_ABSOLUTE_RUNNER);');
OUT=struct('closed',true,'runner',runner,'class',expectedClass);
assignin('base','AF7R58ZZY_R33_RUN_CANONICAL_OUT',OUT);
end
