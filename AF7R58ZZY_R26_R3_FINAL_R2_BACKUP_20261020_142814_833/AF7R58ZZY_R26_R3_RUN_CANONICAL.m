function OUT = AF7R58ZZY_R26_R3_RUN_CANONICAL()
% Run the exact canonical R26-R3 runner by ABSOLUTE PATH in the base workspace.
% This prevents any residual MATLAB-path shadow from selecting a captured copy.

clc;
root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
prod=fullfile(root,'reference','REPLANNING_SYSTEM.m');

expectedRunner='cfa9f9d61b2762fdb899515b2a6d6ddf582d550c1eaf4046574e360b9d28fbe5';
expectedProd='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
expectedClass='2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6';

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R3_RUN_CANONICAL_V1';
OUT.closed=false;
OUT.root=root;
OUT.runner=runner;

assert(isfile(runner),'[R26-R3 RUN] canonical runner missing.');
assert(strcmp(sha_file(runner),expectedRunner), ...
    '[R26-R3 RUN] canonical runner SHA mismatch.');
assert(isfile(prod)&&strcmp(sha_file(prod),expectedProd), ...
    '[R26-R3 RUN] production REPLANNING_SYSTEM SHA mismatch.');
cp=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
assert(isfile(cp)&&strcmp(sha_file(cp),expectedClass), ...
    '[R26-R3 RUN] R26 class SHA mismatch.');

cd(root);
addpath(fullfile(root,'reference'),'-begin');
addpath(root,'-begin');
rehash;

w=which('run_flight_headless');
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('canonical root                          = %s\n',root);
fprintf('which(run_flight_headless)              = %s\n',w);
fprintf('absolute runner                         = %s\n',runner);
fprintf('runner SHA                              = %s\n',sha_file(runner));
fprintf('launch mode                             = ABSOLUTE PATH / BASE WORKSPACE\n');
fprintf('====================================================================================================\n');

% The script begins with clearvars. Execute in BASE workspace so its output
% variables (replanner_instance/runtime_profile/etc.) remain available to report.
cmd=sprintf('run(''%s'');',strrep(runner,,''));
evalin('base',cmd);

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
