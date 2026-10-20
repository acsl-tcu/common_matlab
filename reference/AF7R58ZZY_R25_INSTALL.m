function OUT = AF7R58ZZY_R25_INSTALL()
% AF7R58ZZY-R25 transactional installation.
%
% Installs a NEW R25 subclass/helper/MEX and patches only run_flight_headless:
%   R23 class -> R25 class
%   Flight reference merge_result -> exact shared-state fast merge helper
%
% REPLANNING_SYSTEM.m / R21 / R19 / controller / estimator / safety MEX are
% NOT modified.  The normal 4-ms stop-geometry certificate is also untouched;
% R25 treats 4 ms only as the existing certificate parameter, not as a new
% normal replanning gate.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R25 | TRANSACTIONAL LEAN ONLINE PIPELINE INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] newSubclass=1 | runnerPatch=1 | productionReplannerModified=0 | safetyMathChanged=0\n');

OUT=struct();
OUT.schema='AF7R58ZZY_R25_INSTALL_V1';
OUT.closed=false;
OUT.production_replanner_modified=false;
OUT.runner_modified=false;
OUT.backup_directory='';
OUT.mex_built=false;
OUT.selftest=struct();

pkgRoot=fileparts(mfilename('fullpath'));
runner=which('run_flight_headless');
assert(~isempty(runner)&&isfile(runner), ...
    '[R25 INSTALL] run_flight_headless.m not found on MATLAB path.');
root=fileparts(runner);

expected=struct();
expected.runner='22a4b5fa6d5c2ccfc9a3798780782535e101424173e5eec81c8ddfb9c6f49949';
expected.production='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
expected.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
expected.r19='928677701a06d2c8aabf96982276fb4ada5daf9c67c8ec1354182e823c85e482';
expected.r23helper='e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177';

pProd=which('REPLANNING_SYSTEM');
p21=which('REPLANNING_SYSTEM_R21_BALANCED');
p19=which('REPLANNING_SYSTEM_R19_FAST');
pOldHelper=which('af7r76_compile_correction_packet_R23');
assert(~isempty(pProd)&&~isempty(p21)&&~isempty(p19)&&~isempty(pOldHelper), ...
    '[R25 INSTALL] current R23/R21/R19 authority chain is incomplete.');

fprintf('\n[A. EXACT SOURCE BINDING]\n');
show_hash('run_flight_headless',runner,expected.runner);
show_hash('REPLANNING_SYSTEM',pProd,expected.production);
show_hash('REPLANNING_SYSTEM_R21_BALANCED',p21,expected.r21);
show_hash('REPLANNING_SYSTEM_R19_FAST',p19,expected.r19);
show_hash('R23 compile helper',pOldHelper,expected.r23helper);

assert(strcmp(sha_file(runner),expected.runner), ...
    '[R25 INSTALL] runner SHA differs from the audited R23 runner. No write performed.');
assert(strcmp(sha_file(pProd),expected.production), ...
    '[R25 INSTALL] production REPLANNING_SYSTEM SHA differs. No write performed.');
assert(strcmp(sha_file(p21),expected.r21), ...
    '[R25 INSTALL] R21 SHA differs. No write performed.');
assert(strcmp(sha_file(p19),expected.r19), ...
    '[R25 INSTALL] R19 SHA differs. No write performed.');
assert(strcmp(sha_file(pOldHelper),expected.r23helper), ...
    '[R25 INSTALL] R23 compiler helper SHA differs. No write performed.');

required={ ...
    'REPLANNING_SYSTEM_R25_LEAN.m', ...
    'af7r77_compile_correction_packet_R25.m', ...
    'af7r77_compile_dcp_mex.cpp', ...
    'af7r77_merge_result_reference_fast_R25.m', ...
    'AF7R58ZZY_R25_SELFTEST.m', ...
    'AF7R58ZZY_R25_REPORT.m'};
for k=1:numel(required)
    assert(isfile(fullfile(pkgRoot,required{k})), ...
        '[R25 INSTALL] package file missing: %s',required{k});
end

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backupDir=fullfile(root,['AF7R58ZZY_R25_BACKUP_' stamp]);
[ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
OUT.backup_directory=backupDir;
copyfile(runner,fullfile(backupDir,'run_flight_headless.m'));

installNames=required;
written={};
try
    fprintf('\n[B. COPY NEW R25 FILES]\n');
    for k=1:numel(installNames)
        src=fullfile(pkgRoot,installNames{k});
        dst=fullfile(root,installNames{k});
        if ~strcmpi(canonical(src),canonical(dst))
            if isfile(dst)
                copyfile(dst,fullfile(backupDir,installNames{k}));
            end
            copyfile(src,dst);
            written{end+1}=dst; %#ok<AGROW>
        end
        fprintf(' %-48s SHA=%s\n',installNames{k},sha_file(dst));
    end

    addpath(root,'-begin');
    rehash;

    fprintf('\n[C. BUILD EXACT DERIVATIVE MEX]\n');
    cpp=fullfile(root,'af7r77_compile_dcp_mex.cpp');
    buildLog=evalc("mex('-R2018a','-O','-outdir',root,cpp)");
    OUT.mex_build_log=buildLog;
    mexPath=fullfile(root,['af7r77_compile_dcp_mex.' mexext]);
    assert(isfile(mexPath),'[R25 INSTALL] MEX output not found after build.');
    clear af7r77_compile_dcp_mex;
    rehash;
    assert(exist('af7r77_compile_dcp_mex','file')==3, ...
        '[R25 INSTALL] compiled MEX is not resolvable.');
    OUT.mex_built=true;
    fprintf(' MEX                                    = %s\n',mexPath);
    fprintf(' MEX SHA                                = %s\n',sha_file(mexPath));

    fprintf('\n[D. BITWISE / VALUE EQUIVALENCE SELFTEST]\n');
    OUT.selftest=AF7R58ZZY_R25_SELFTEST();
    assert(isstruct(OUT.selftest)&&isfield(OUT.selftest,'closed')&&OUT.selftest.closed, ...
        '[R25 INSTALL] selftest did not close.');

    fprintf('\n[E. R25 CLASS PARSE]\n');
    try, clear REPLANNING_SYSTEM_R25_LEAN; catch, end
    rehash;
    mc=meta.class.fromName('REPLANNING_SYSTEM_R25_LEAN');
    assert(~isempty(mc),'[R25 INSTALL] R25 class parse failed.');
    fprintf(' class                                  = %s\n',mc.Name);

    fprintf('\n[F. PATCH RUNNER ONLY]\n');
    R=fileread(runner);
    oldClass='REPLANNING_SYSTEM_R23_COMPILED(agent,nominal_ref,replan_opts)';
    newClass='REPLANNING_SYSTEM_R25_LEAN(agent,nominal_ref,replan_opts)';
    oldMerge='merge_result(agent.reference.result,r22_sload_result)';
    newMerge='af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)';
    assert(numel(strfind(R,oldClass))==1,'[R25 INSTALL] expected R23 constructor line not unique.');
    assert(numel(strfind(R,oldMerge))==1,'[R25 INSTALL] expected Flight merge line not unique.');
    R=strrep(R,oldClass,newClass);
    R=strrep(R,oldMerge,newMerge);
    write_utf8(runner,R);
    rehash;
    R2=fileread(runner);
    assert(numel(strfind(R2,newClass))==1&&numel(strfind(R2,newMerge))==1, ...
        '[R25 INSTALL] post-write runner verification failed.');
    OUT.runner_modified=true;

    OUT.runner_sha_after=sha_file(runner);
    OUT.production_sha_after=sha_file(pProd);
    assert(strcmp(OUT.production_sha_after,expected.production), ...
        '[R25 INSTALL] production REPLANNING_SYSTEM changed unexpectedly.');

    fprintf(' runner modified                         = 1\n');
    fprintf(' runner SHA after                        = %s\n',OUT.runner_sha_after);
    fprintf(' production REPLANNING_SYSTEM modified   = 0\n');
    fprintf(' production SHA                          = %s\n',OUT.production_sha_after);

    OUT.closed=true;
    OUT.status='R25_INSTALLED__RUN_CLEAN_FLIGHT_NEXT';

catch ME
    fprintf('\n[R25 INSTALL FAILURE] %s\n',ME.message);
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
    assignin('base','AF7R58ZZY_R25_INSTALL_OUT',OUT);
    rethrow(ME);
end

fprintf('\n====================================================================================================\n');
fprintf(' R25 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('status                                  = %s\n',OUT.status);
fprintf('backup                                  = %s\n',backupDir);
fprintf('Next:\n');
fprintf('  1) run_flight_headless\n');
fprintf('  2) AF7R58ZZY_R25_REPORT\n');
fprintf('Return the R25 report/result to ChatGPT.\n');
fprintf('====================================================================================================\n');
assignin('base','AF7R58ZZY_R25_INSTALL_OUT',OUT);
end

function show_hash(name,p,expected)
h=sha_file(p);
fprintf(' %-38s = %s\n',name,p);
fprintf('   SHA=%s | expected=%d\n',h,strcmp(h,expected));
end

function p=canonical(p)
try
    p=char(java.io.File(p).getCanonicalPath());
catch
    p=char(p);
end
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
