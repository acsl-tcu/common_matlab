function OUT = AF7R58ZZY_R26_R3_RUN_CANONICAL()
% Run the exact canonical R26-R3 runner by ABSOLUTE PATH in BASE workspace.
% No bare-name dispatch is used for the runner.

root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
prod=fullfile(root,'reference','REPLANNING_SYSTEM.m');
r26=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');

expectedRunner='cfa9f9d61b2762fdb899515b2a6d6ddf582d550c1eaf4046574e360b9d28fbe5';
expectedProd='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
expectedR26='2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6';

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R3_RUN_CANONICAL_V2';
OUT.closed=false;
OUT.root=root;
OUT.runner=runner;

assert(isfile(runner)&&strcmp(sha_file(runner),expectedRunner), ...
    '[R26-R3 RUN] canonical runner missing/SHA mismatch.');
assert(isfile(prod)&&strcmp(sha_file(prod),expectedProd), ...
    '[R26-R3 RUN] production SHA mismatch.');
assert(isfile(r26)&&strcmp(sha_file(r26),expectedR26), ...
    '[R26-R3 RUN] R26 class SHA mismatch.');

cd(root);
addpath(fullfile(root,'reference'),'-begin');
addpath(root,'-begin');
rehash;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | CANONICAL FLIGHT LAUNCH V2\n');
fprintf('====================================================================================================\n');
fprintf('canonical root                          = %s\n',root);
fprintf('which(run_flight_headless)              = %s\n',which('run_flight_headless'));
fprintf('absolute runner                         = %s\n',runner);
fprintf('runner SHA                              = %s\n',sha_file(runner));
fprintf('launch mode                             = ABSOLUTE PATH / BASE WORKSPACE\n');
fprintf('====================================================================================================\n');

% Avoid quote construction entirely: pass the absolute path through BASE.
assignin('base','AF7R58ZZY_R26_R3_ABS_RUNNER_PATH',runner);
evalin('base','run(AF7R58ZZY_R26_R3_ABS_RUNNER_PATH);');

OUT.closed=true;
OUT.status='CANONICAL_FLIGHT_COMPLETED';
assignin('base','AF7R58ZZY_R26_R3_RUN_CANONICAL_OUT',OUT);
end

function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
