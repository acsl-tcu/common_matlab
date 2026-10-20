function OUT = AF7R58ZZY_R27_RUN_CANONICAL()
% Run exact canonical R27 runner by absolute path in BASE workspace.
root=fileparts(mfilename('fullpath'));
runner=fullfile(root,'run_flight_headless.m');
prod=fullfile(root,'reference','REPLANNING_SYSTEM.m');
r27=fullfile(root,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
expectedRunner='5f0e4ad0230d4e5cebcbf9de2ad1525d454a6fe178a9694c9323eebed5aa048c';
expectedProd='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
expectedR27='be3bd20dd0c1acc71d1968727124918b9300b6542d97b197510e4b89442f06da';
OUT=struct('schema','AF7R58ZZY_R27_RUN_CANONICAL_V1','closed',false,'root',root,'runner',runner);
assert(isfile(runner)&&strcmp(sha_file(runner),expectedRunner),'[R27 RUN] runner SHA mismatch.');
assert(isfile(prod)&&strcmp(sha_file(prod),expectedProd),'[R27 RUN] production SHA mismatch.');
assert(isfile(r27)&&strcmp(sha_file(r27),expectedR27),'[R27 RUN] R27 class SHA mismatch.');
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R27 | CANONICAL FLIGHT LAUNCH\n');
fprintf('====================================================================================================\n');
fprintf('root                                    = %s\n',root);
fprintf('runner                                  = %s\n',runner);
fprintf('runner SHA                              = %s\n',sha_file(runner));
fprintf('class                                   = %s\n',which('REPLANNING_SYSTEM_R27_LEAN_EXACT'));
fprintf('launch                                  = ABSOLUTE PATH / BASE WORKSPACE\n');
fprintf('====================================================================================================\n');
assignin('base','AF7R58ZZY_R27_ABS_RUNNER_PATH',runner);
evalin('base','run(AF7R58ZZY_R27_ABS_RUNNER_PATH);');
OUT.closed=true; OUT.status='R27_CANONICAL_FLIGHT_COMPLETED';
assignin('base','AF7R58ZZY_R27_RUN_CANONICAL_OUT',OUT);
end
function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0); c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8'); md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8')); d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
