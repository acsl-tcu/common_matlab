function OUT = AF7R58ZZY_R26_R3_INSTALL()
% AF7R58ZZY-R26-R3 transactional install.
% Fixes R26-R2 MATLAB superclass-dispatch parse error.
%
% Root cause:
%   R26-R2 called
%     prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED(...)
%   from inside advance_staged_replan_pipeline().
% MATLAB rejects that different-name direct superclass dispatch.
%
% R3 uses a legal same-name override bridge:
%   advance() -> obj.prepare_staged_reference_only_bundle(...)
%   same-name override -> prepare_staged_reference_only_bundle@R21(...)
% and a private core-only flag returns before the inherited R25 preflight
% compile. Mathematical/certificate authority is unchanged.
%
% Expected starting runner: exact R25, because both failed R26 installers
% reported runner rollback PASS.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | TRANSACTIONAL FOUR-STAGE INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] matlabDispatchFixed=1 | runnerPatch=1 | productionReplannerModified=0 | controllerModified=0 | safetyMathChanged=0\n');

runnerPath=which('run_flight_headless');
assert(~isempty(runnerPath),'[R26-R3 INSTALL] run_flight_headless unresolved.');
root=fileparts(runnerPath);
srcdir=fileparts(mfilename('fullpath'));

runner=fullfile(root,'run_flight_headless.m');
pProd=which('REPLANNING_SYSTEM');
pR25=which('REPLANNING_SYSTEM_R25_LEAN');
pR21=which('REPLANNING_SYSTEM_R21_BALANCED');
pR19=which('REPLANNING_SYSTEM_R19_FAST');
pCompile=which('af7r76_compile_correction_packet_R23');
pMerge=which('af7r77_merge_result_reference_fast_R25');

expected=struct();
expected.runner='0581116b243151e9b6855aacb676c704ea4b15991620213833f8ba914b9a3451';
expected.production='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
expected.r25='f2f13902bcf1589bacde9092527f3d0c8cb68f9d330d819b5282ad7b002f616c';
expected.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
expected.r19='928677701a06d2c8aabf96982276fb4ada5daf9c67c8ec1354182e823c85e482';
expected.compile='e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177';
expected.merge='f7b5f4c46710c0bc4ee64c3a7e839717a14bc9821291bb8e15bddefeef976a36';

fprintf('\n[A. EXACT R25 SOURCE BINDING AFTER R26-R2 ROLLBACK]\n');
show_hash('run_flight_headless',runner,expected.runner);
show_hash('REPLANNING_SYSTEM',pProd,expected.production);
show_hash('REPLANNING_SYSTEM_R25_LEAN',pR25,expected.r25);
show_hash('REPLANNING_SYSTEM_R21_BALANCED',pR21,expected.r21);
show_hash('REPLANNING_SYSTEM_R19_FAST',pR19,expected.r19);
show_hash('R23 compile helper',pCompile,expected.compile);
show_hash('R25 fast merge',pMerge,expected.merge);

assert(strcmp(sha_file(runner),expected.runner), ...
    '[R26-R3 INSTALL] runner is not exact R25 input. Do not patch unknown source.');
assert(strcmp(sha_file(pProd),expected.production),'[R26-R3 INSTALL] production source changed.');
assert(strcmp(sha_file(pR25),expected.r25),'[R26-R3 INSTALL] R25 class changed.');
assert(strcmp(sha_file(pR21),expected.r21),'[R26-R3 INSTALL] R21 class changed.');
assert(strcmp(sha_file(pR19),expected.r19),'[R26-R3 INSTALL] R19 class changed.');
assert(strcmp(sha_file(pCompile),expected.compile),'[R26-R3 INSTALL] R23 compile helper changed.');
assert(strcmp(sha_file(pMerge),expected.merge),'[R26-R3 INSTALL] R25 merge helper changed.');

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backupDir=fullfile(root,['AF7R58ZZY_R26_R3_BACKUP_' stamp]);
[ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
copyfile(runner,fullfile(backupDir,'run_flight_headless.m'));

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R3_INSTALL_V1';
OUT.closed=false;
OUT.runner_modified=false;
OUT.production_replanner_modified=false;
OUT.controller_modified=false;
OUT.backup_directory=backupDir;
OUT.status='INIT';

try
    fprintf('\n[B. COPY R26-R3 FILES]\n');
    names={'REPLANNING_SYSTEM_R26_BALANCED.m', ...
        'AF7R58ZZY_R26_R3_SELFTEST.m','AF7R58ZZY_R26_R3_REPORT.m'};
    for k=1:numel(names)
        src=fullfile(srcdir,names{k}); dst=fullfile(root,names{k});
        assert(isfile(src),'[R26-R3 INSTALL] package file missing: %s',names{k});
        copyfile(src,dst);
        fprintf(' %-48s SHA=%s\n',names{k},sha_file(dst));
    end
    rehash;

    fprintf('\n[C. MATLAB-DISPATCH STRUCTURAL SELFTEST]\n');
    OUT.selftest=AF7R58ZZY_R26_R3_SELFTEST();
    assert(OUT.selftest.closed,'[R26-R3 INSTALL] selftest did not close.');

    fprintf('\n[D. R26-R3 CLASS PARSE]\n');
    try, clear REPLANNING_SYSTEM_R26_BALANCED; catch, end
    rehash;
    mc=meta.class.fromName('REPLANNING_SYSTEM_R26_BALANCED');
    assert(~isempty(mc),'[R26-R3 INSTALL] class parse failed.');
    fprintf(' class                                  = %s\n',mc.Name);
    C=fileread(fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m'));
    assert(contains(C,'AF7R58ZZY_R26_BALANCED_R3'), ...
        '[R26-R3 INSTALL] build id missing.');

    fprintf('\n[E. PATCH RUNNER ONLY]\n');
    R=fileread(runner);
    oldClass='REPLANNING_SYSTEM_R25_LEAN(agent,nominal_ref,replan_opts)';
    newClass='REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)';
    assert(numel(strfind(R,oldClass))==1,'[R26-R3 INSTALL] R25 constructor line not unique.');

    oldProf='replan_opts.ENABLE_PROFILING = true;';
    newProf='replan_opts.ENABLE_PROFILING = false;';
    oldLight='replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;';
    newLight='replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;';
    oldThresh='replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 0.0;';
    newThresh='replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 2.0;';
    assert(numel(strfind(R,oldProf))==1,'[R26-R3 INSTALL] profiling option line not unique.');
    assert(numel(strfind(R,oldLight))==1,'[R26-R3 INSTALL] light-audit option line not unique.');
    assert(numel(strfind(R,oldThresh))==1,'[R26-R3 INSTALL] spike threshold line not unique.');

    assert(numel(strfind(R,'af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)'))==1, ...
        '[R26-R3 INSTALL] exact R25 fast merge binding missing.');

    R=strrep(R,oldClass,newClass);
    R=strrep(R,oldProf,newProf);
    R=strrep(R,oldLight,newLight);
    R=strrep(R,oldThresh,newThresh);

    write_utf8(runner,R);
    rehash;

    R2=fileread(runner);
    assert(numel(strfind(R2,newClass))==1,'[R26-R3 INSTALL] R26 class binding failed.');
    assert(numel(strfind(R2,newProf))==1&&numel(strfind(R2,newLight))==1, ...
        '[R26-R3 INSTALL] clean profiling flags failed.');

    OUT.runner_modified=true;
    OUT.runner_sha_after=sha_file(runner);
    OUT.production_sha_after=sha_file(pProd);
    assert(strcmp(OUT.production_sha_after,expected.production), ...
        '[R26-R3 INSTALL] production source changed unexpectedly.');

    fprintf(' runner modified                         = 1\n');
    fprintf(' detailed profiling OFF                 = 1\n');
    fprintf(' light runtime audit OFF                = 1\n');
    fprintf(' exact R25 fast merge retained          = 1\n');
    fprintf(' controller source modified             = 0\n');
    fprintf(' production REPLANNING_SYSTEM modified  = 0\n');
    fprintf(' runner SHA after                        = %s\n',OUT.runner_sha_after);
    fprintf(' production SHA                          = %s\n',OUT.production_sha_after);

    OUT.closed=true;
    OUT.status='R26_R3_INSTALLED__RUN_CLEAN_FLIGHT_NEXT';

catch ME
    fprintf('\n[R26-R3 INSTALL FAILURE] %s\n',ME.message);
    try
        copyfile(fullfile(backupDir,'run_flight_headless.m'),runner);
        rehash;
        OUT.runner_modified=false;
        fprintf(' runner rollback                         = PASS\n');
    catch MER
        fprintf(' runner rollback                         = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R26_R3_INSTALL_OUT',OUT);
    rethrow(ME);
end

fprintf('\n====================================================================================================\n');
fprintf(' R26-R3 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('status                                  = %s\n',OUT.status);
fprintf('backup                                  = %s\n',backupDir);
fprintf('research timing target                  = replanner/reference ~15 ms, <=17 ms immediate goal\n');
fprintf('controller                              = measured separately, not modified\n');
fprintf('R26 scheduling                          = generate / ref-cert / core-prep / final-commit\n');
fprintf('scheduled activation                    = trigger + 3*dt (75 ms at dt=25 ms)\n');
fprintf('fixed collision horizon                 = 4 s (unchanged)\n');
fprintf('Next:\n');
fprintf('  1) run_flight_headless\n');
fprintf('  2) AF7R58ZZY_R26_R3_REPORT\n');
fprintf('Return AF7R58ZZY_R26_R3_REPORT.txt and AF7R58ZZY_R26_R3_OUT.mat.\n');
fprintf('====================================================================================================\n');
assignin('base','AF7R58ZZY_R26_R3_INSTALL_OUT',OUT);
end

function show_hash(name,p,expected)
assert(~isempty(p)&&isfile(p),'[R26-R3 INSTALL] missing source: %s',name);
h=sha_file(p);
fprintf(' %-38s = %s\n',name,p);
fprintf('   SHA=%s | expected=%d\n',h,strcmp(h,expected));
end

function write_utf8(p,s)
f=fopen(p,'wt','n','UTF-8'); assert(f>=0,'cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,s,'char');
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
