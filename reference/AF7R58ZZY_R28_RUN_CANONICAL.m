function OUT=AF7R58ZZY_R28_RUN_CANONICAL()
root=fileparts(mfilename('fullpath')); runner=fullfile(root,'run_flight_headless.m');
assert(isfile(runner)&&strcmp(sha_file(runner),'4122e1079eaa703d113a522099ef6aa5e376ed42c79c33e226ff61ccafe35c3a'),'[R28 RUN] canonical runner SHA mismatch.');
assert(isfile(fullfile(root,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m'))&&strcmp(sha_file(fullfile(root,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m')),'4edda21ff4a071754005a2c3aba174210d923542bf6b390ce2a21185f3faee2e'),'[R28 RUN] class SHA mismatch.');
rev=af7r36_single_core_mex(5);assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1'),'[R28 RUN] native MEX revision mismatch.');
cd(root); addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;
fprintf('\n====================================================================================================\n AF7R58ZZY-R28 | CANONICAL FLIGHT LAUNCH\n====================================================================================================\n');
fprintf('runner = %s\nSHA    = %s\nMEX rev= %s\n',runner,sha_file(runner),rev);
assignin('base','AF7R58ZZY_R28_ABS_RUNNER',runner);
evalin('base','run(AF7R58ZZY_R28_ABS_RUNNER);');
OUT=struct('schema','AF7R58ZZY_R28_RUN_CANONICAL_V1','closed',true,'runner',runner);assignin('base','AF7R58ZZY_R28_RUN_CANONICAL_OUT',OUT);
end
function h=sha_file(p)
f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
