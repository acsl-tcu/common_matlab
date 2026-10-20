function OUT=AF7R58ZZY_R28_R2_INSTALL()
% AF7R58ZZY-R28-R2 safe transactional native-once install.
% =========================================================================
% R2 fixes the R28 V1 packaging defect: V1 placed
%   af7r36_single_core_mex.cpp
% at ZIP top level.  If the ZIP was extracted directly into reference/, the
% package payload itself overwrote the R27 baseline source BEFORE the installer
% could verify it.  The observed SHA bd4474... is exactly the intended R28
% payload, not an unknown external modification.
%
% R2 stores all live-name payloads under payload/ and the exact R27 baseline
% cpp under baseline/.  Extraction alone therefore cannot overwrite live code.
%
% If the canonical cpp is already the known accidental R28 payload SHA, R2
% first restores the exact exported R27 baseline source (4f9778...) so runner,
% source and current MEX are coherent before any shadow build/test.
%
% No Flight is run here.  Production REPLANNING_SYSTEM.m and controller are
% not modified.  The R28 mathematical design is unchanged from V1.
% =========================================================================
clc;
fprintf('
====================================================================================================
');
fprintf(' AF7R58ZZY-R28-R2 | SAFE TRANSACTIONAL NATIVE-ONCE INSTALL
');
fprintf('====================================================================================================
');
fprintf('[AUTHORITY] packageCollisionFixed=1 | generatorMathChanged=0 | certificateMathChanged=0 | scheduleChanged=0 | controllerModified=0
');

EXP=struct();
EXP.prod='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r27runner='5f0e4ad0230d4e5cebcbf9de2ad1525d454a6fe178a9694c9323eebed5aa048c';
EXP.r27class='be3bd20dd0c1acc71d1968727124918b9300b6542d97b197510e4b89442f06da';
EXP.cpp0='4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea';
EXP.hpp='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
EXP.r25helper='8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44';
EXP.r27helper='2a008e5ac8abf248416ffe0ecef9ab0ebb71dac28901aa6c9467b9a823772533';
EXP.r28runner='4122e1079eaa703d113a522099ef6aa5e376ed42c79c33e226ff61ccafe35c3a';
EXP.r28class='4edda21ff4a071754005a2c3aba174210d923542bf6b390ce2a21185f3faee2e';
EXP.r28cpp='bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a';
EXP.report='f104d8f8d37e64ea070caa59e270a54e91ca0acfc65d762fb60253794268d5fa';
EXP.selftest='dae3094b64d53135566fb3fb5df01e1b7289bb06968ae8cb020b43ca2fd716e4';
EXP.launcher='ecb618749dd6c1a46ebf828d395a53a44ca8b4efd3eabc1c346ad1f6a839fa98';

pkg=fileparts(mfilename('fullpath'));
payload=fullfile(pkg,'payload');
baseline=fullfile(pkg,'baseline');
assert(isfolder(payload)&&isfolder(baseline),'[R28-R2 INSTALL] payload/baseline folders missing.');

% Package bytes are checked BEFORE touching the repository.
pCpp=fullfile(payload,'af7r36_single_core_mex.cpp');
pHpp=fullfile(payload,'af7r36_core.hpp');
pClass=fullfile(payload,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m');
pRunner=fullfile(payload,'run_flight_headless_R28_FINAL_SOURCE.m');
pReport=fullfile(payload,'AF7R58ZZY_R28_REPORT.m');
pSelf=fullfile(payload,'AF7R58ZZY_R28_SELFTEST.m');
pLaunch=fullfile(payload,'AF7R58ZZY_R28_RUN_CANONICAL.m');
pBaseCpp=fullfile(baseline,'af7r36_single_core_mex.cpp');

fprintf('
[A. PACKAGE BYTE AUTHORITY]
');
show_exact('R27 baseline cpp payload',pBaseCpp,EXP.cpp0);
show_exact('R28 modified cpp payload',pCpp,EXP.r28cpp);
show_exact('unchanged AF7R36 hpp payload',pHpp,EXP.hpp);
show_exact('R28 class payload',pClass,EXP.r28class);
show_exact('R28 runner payload',pRunner,EXP.r28runner);
show_exact('R28 report payload',pReport,EXP.report);
show_exact('R28 selftest payload',pSelf,EXP.selftest);
show_exact('R28 launcher payload',pLaunch,EXP.launcher);

root=find_root(EXP); ref=fullfile(root,'reference');
C=struct('runner',fullfile(root,'run_flight_headless.m'), ...
 'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
 'r27',fullfile(root,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m'), ...
 'cpp',fullfile(ref,'af7r36_single_core_mex.cpp'), ...
 'hpp',fullfile(ref,'af7r36_core.hpp'), ...
 'r25helper',fullfile(root,'af7r77_compile_correction_packet_R25.m'), ...
 'helper',fullfile(root,'af7r78_incremental_compile_R27.m'), ...
 'mex',fullfile(ref,['af7r36_single_core_mex.' mexext]), ...
 'r28class',fullfile(root,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m'), ...
 'report',fullfile(root,'AF7R58ZZY_R28_REPORT.m'), ...
 'selftest',fullfile(root,'AF7R58ZZY_R28_SELFTEST.m'), ...
 'launcher',fullfile(root,'AF7R58ZZY_R28_RUN_CANONICAL.m'));

cd(root); addpath(ref,'-begin'); addpath(root,'-begin'); rehash;

fprintf('
[B. EXACT ACTIVE R27 AUTHORITY -- WITH KNOWN V1 EXTRACTION-COLLISION RECOGNITION]
');
show_exact('runner',C.runner,EXP.r27runner);
show_exact('R27 class',C.r27,EXP.r27class);
show_exact('production',C.prod,EXP.prod);
show_exact('AF7R36 hpp',C.hpp,EXP.hpp);
show_exact('R25 packet helper',C.r25helper,EXP.r25helper);
show_exact('R27 helper',C.helper,EXP.r27helper);
assert(paths_equal(which('af7r77_compile_correction_packet_R25'),C.r25helper),'[R28-R2 INSTALL] R25 helper path shadowed.');
assert(paths_equal(which('af7r78_incremental_compile_R27'),C.helper),'[R28-R2 INSTALL] R27 helper path shadowed.');
assert(exist('af7r77_compile_dcp_mex','file')==3,'[R28-R2 INSTALL] R25 DCP reference MEX missing.');
assert(isfile(C.mex),'[R28-R2 INSTALL] current AF7R36 MEX missing.');

cppNow=sha_file(C.cpp);
if strcmp(cppNow,EXP.cpp0)
    sourceState='R27_BASELINE'; collision=false;
elseif strcmp(cppNow,EXP.r28cpp)
    sourceState='KNOWN_R28_V1_EXTRACTION_COLLISION'; collision=true;
else
    error('[R28-R2 INSTALL] canonical AF7R36 cpp is neither exact R27 baseline nor exact known R28 V1 payload: %s',cppNow);
end
fprintf(' AF7R36 cpp current SHA                  = %s
',cppNow);
fprintf(' AF7R36 cpp state                        = %s
',sourceState);
fprintf(' known package-extraction collision      = %d
',collision);

% Backup before any repository write.
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,['AF7R58ZZY_R28_R2_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
backup_one(C.runner,backup,'1_run_flight_headless.m');
backup_one(C.r27,backup,'2_REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
backup_one(C.cpp,backup,'3_current_af7r36_single_core_mex.cpp');
backup_one(C.hpp,backup,'4_af7r36_core.hpp');
backup_one(C.mex,backup,['5_af7r36_single_core_mex.' mexext]);
backup_optional(C.r28class,backup,'6_pre_R28_class.m');
backup_optional(C.report,backup,'7_pre_R28_report.m');
backup_optional(C.selftest,backup,'8_pre_R28_selftest.m');
backup_optional(C.launcher,backup,'9_pre_R28_launcher.m');

OUT=struct();
OUT.schema='AF7R58ZZY_R28_R2_INSTALL_V1';
OUT.closed=false; OUT.backup=backup;
OUT.production_modified=false; OUT.controller_modified=false;
OUT.package_extraction_collision_detected=collision;
OUT.preinstall_cpp_sha=cppNow;
OUT.preinstall_cpp_state=sourceState;
OUT.status='INIT';
liveModified=false;

try
    % V1 extraction may already have replaced only the .cpp source while the
    % runner/MEX remain R27. Normalize SOURCE back to exact R27 before testing.
    fprintf('
[C. NORMALIZE PREINSTALL SOURCE AUTHORITY]
');
    if collision
        copybin(pBaseCpp,C.cpp);
        assert(strcmp(sha_file(C.cpp),EXP.cpp0),'[R28-R2 INSTALL] baseline cpp recovery failed.');
        fprintf(' accidental R28 source overwrite repaired = 1
');
    else
        fprintf(' accidental R28 source overwrite repaired = 0 (not needed)
');
    end
    fprintf(' canonical cpp baseline SHA              = %s
',sha_file(C.cpp));

    % Build the R28 candidate in an isolated directory. The live source and
    % live MEX are still R27 at this point.
    fprintf('
[D. BUILD SHADOW R28 MEX -- LIVE MEX UNCHANGED]
');
    builddir=fullfile(root,['AF7R58ZZY_R28_R2_BUILD_' stamp]);
    [ok,msg]=mkdir(builddir); assert(ok,'%s',msg);
    testName='af7r36_single_core_mex_R28_R2_TEST';
    build_mex(pCpp,payload,builddir,testName);
    addpath(payload,'-begin'); addpath(builddir,'-begin'); rehash;
    cleanupPath=onCleanup(@()cleanup_shadow_paths(builddir,payload)); %#ok<NASGU>
    T=AF7R58ZZY_R28_SELFTEST(testName,pClass);
    assert(T.closed,'[R28-R2 INSTALL] shadow selftest did not close.');
    OUT.shadow_selftest=T;
    clear cleanupPath;
    cleanup_shadow_paths(builddir,payload);

    fprintf('
[E. INSTALL EXACT R28 SOURCE / CLASS / RUNNER]
');
    copybin(pCpp,C.cpp);
    % Header is mathematically/source identical. Copying exact bytes is a no-op
    % but closes source provenance for the live build.
    copybin(pHpp,C.hpp);
    copybin(pClass,C.r28class);
    copybin(pRunner,C.runner);
    copybin(pReport,C.report);
    copybin(pSelf,C.selftest);
    copybin(pLaunch,C.launch);
    liveModified=true;
    assert(strcmp(sha_file(C.cpp),EXP.r28cpp),'[R28-R2 INSTALL] installed cpp mismatch.');
    assert(strcmp(sha_file(C.hpp),EXP.hpp),'[R28-R2 INSTALL] installed hpp mismatch.');
    assert(strcmp(sha_file(C.runner),EXP.r28runner),'[R28-R2 INSTALL] installed runner mismatch.');
    assert(strcmp(sha_file(C.r28class),EXP.r28class),'[R28-R2 INSTALL] installed R28 class mismatch.');

    fprintf('
[F. BUILD LIVE AF7R36 MEX]
');
    try, clear af7r36_single_core_mex; catch, end
    build_mex(C.cpp,ref,ref,'af7r36_single_core_mex');
    rehash; addpath(ref,'-begin'); addpath(root,'-begin');
    assert(paths_equal(which('af7r36_single_core_mex'),C.mex),'[R28-R2 INSTALL] live AF7R36 MEX path shadowed.');
    rev=af7r36_single_core_mex(5);
    assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1'),'[R28-R2 INSTALL] live MEX revision mismatch.');

    fprintf('
[G. LIVE EXACT EQUIVALENCE SELFTEST]
');
    T2=AF7R58ZZY_R28_SELFTEST('af7r36_single_core_mex',C.r28class);
    assert(T2.closed,'[R28-R2 INSTALL] live selftest did not close.');
    OUT.live_selftest=T2;

    fprintf('
[H. CLASS / RUNNER / PRODUCTION IMMUTABILITY]
');
    try, clear REPLANNING_SYSTEM_R28_NATIVE_ONCE; catch, end
    rehash;
    mc=meta.class.fromName('REPLANNING_SYSTEM_R28_NATIVE_ONCE');
    assert(~isempty(mc),'[R28-R2 INSTALL] R28 class parse failed.');
    R=fileread(C.runner);
    assert(count(R,'REPLANNING_SYSTEM_R28_NATIVE_ONCE(agent,nominal_ref,replan_opts)')==1, ...
        '[R28-R2 INSTALL] R28 runner binding missing/nonunique.');
    assert(~contains(R,'REPLANNING_SYSTEM_R27_LEAN_EXACT(agent,nominal_ref,replan_opts)'), ...
        '[R28-R2 INSTALL] stale R27 runner binding remains.');
    assert(strcmp(sha_file(C.prod),EXP.prod),'[R28-R2 INSTALL] production MATLAB source changed.');
    fprintf(' R28 class parse                         = PASS
');
    fprintf(' production MATLAB source modified       = 0
');
    fprintf(' controller source modified              = 0
');

    OUT.closed=true;
    OUT.status='R28_R2_INSTALLED__RUN_CANONICAL_NEXT';

catch ME
    fprintf('
[R28-R2 INSTALL FAILURE] %s
',ME.message);
    % Always leave a coherent R27 state on failure, including the known V1
    % extraction-collision case.  The exact baseline cpp comes from the user's
    % read-only R28 source export.
    try
        try, clear af7r36_single_core_mex; catch, end
        copybin(fullfile(backup,'1_run_flight_headless.m'),C.runner);
        copybin(pBaseCpp,C.cpp);
        copybin(fullfile(backup,'4_af7r36_core.hpp'),C.hpp);
        copybin(fullfile(backup,['5_af7r36_single_core_mex.' mexext]),C.mex);
        restore_optional(backup,'6_pre_R28_class.m',C.r28class);
        restore_optional(backup,'7_pre_R28_report.m',C.report);
        restore_optional(backup,'8_pre_R28_selftest.m',C.selftest);
        restore_optional(backup,'9_pre_R28_launcher.m',C.launcher);
        rehash; addpath(ref,'-begin'); addpath(root,'-begin');
        assert(strcmp(sha_file(C.runner),EXP.r27runner),'rollback runner mismatch');
        assert(strcmp(sha_file(C.cpp),EXP.cpp0),'rollback cpp mismatch');
        assert(strcmp(sha_file(C.hpp),EXP.hpp),'rollback hpp mismatch');
        fprintf(' rollback to coherent R27 source/binary = PASS
');
    catch MER
        fprintf(' rollback                               = FAILED: %s
',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R28_R2_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R28_R2_INSTALL_OUT',OUT);
fprintf('
====================================================================================================
');
fprintf(' R28-R2 INSTALL COMPLETE
');
fprintf('====================================================================================================
');
fprintf('status                                  = %s
',OUT.status);
fprintf('backup                                  = %s
',backup);
fprintf('package extraction collision detected   = %d
',collision);
fprintf('production MATLAB source modified       = 0
');
fprintf('controller source modified              = 0
');
fprintf('NEXT: AF7R58ZZY_R28_RUN_CANONICAL
');
fprintf('Then: AF7R58ZZY_R28_REPORT
');
fprintf('====================================================================================================
');
end

function root=find_root(EXP)
starts=[{pwd,fileparts(mfilename('fullpath'))},strsplit(path,pathsep)]; cand={};
for k=1:numel(starts)
    p=starts{k}; if isempty(p)||~isfolder(p),continue;end
    for j=1:7
        cand{end+1}=p; %#ok<AGROW>
        q=fileparts(p); if isempty(q)||strcmp(q,p),break;end; p=q;
    end
end
cand=unique(cand); good={};
for k=1:numel(cand)
    r=cand{k}; p=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    rr=fullfile(r,'run_flight_headless.m'); c=fullfile(r,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
    if isfile(p)&&isfile(rr)&&isfile(c)&&strcmp(sha_file(p),EXP.prod)&& ...
            strcmp(sha_file(rr),EXP.r27runner)&&strcmp(sha_file(c),EXP.r27class)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good),'[R28-R2 INSTALL] exact canonical R27 root not found.');
score=zeros(size(good));
for k=1:numel(good)
    [~,n]=fileparts(good{k});
    score(k)=100*strcmpi(n,'common_matlab')+20*isfolder(fullfile(good{k},'.git'));
end
[~,i]=max(score); root=good{i};
end

function build_mex(src,inc,outdir,name)
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"',['-I' inc],'-output',name,'-outdir',outdir,src);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"',['-I' inc],'-output',name,'-outdir',outdir,src);
end
rehash;
assert(isfile(fullfile(outdir,[name '.' mexext])),'[R28-R2 INSTALL] MEX build output missing.');
end

function cleanup_shadow_paths(builddir,payload)
try, rmpath(builddir); catch, end
try, rmpath(payload); catch, end
rehash;
end

function backup_one(src,dirp,name)
assert(isfile(src),'[R28-R2 INSTALL] backup source missing: %s',src);
copyfile(src,fullfile(dirp,name),'f');
end

function backup_optional(src,dirp,name)
marker=fullfile(dirp,[name '.ABSENT']);
if isfile(src)
    copyfile(src,fullfile(dirp,name),'f');
else
    f=fopen(marker,'wt'); assert(f>=0); fprintf(f,'ABSENT
'); fclose(f);
end
end

function restore_optional(dirp,name,dst)
src=fullfile(dirp,name); marker=fullfile(dirp,[name '.ABSENT']);
if isfile(src)
    copyfile(src,dst,'f');
elseif isfile(marker) && isfile(dst)
    delete(dst);
end
end

function show_exact(n,p,h)
assert(isfile(p),'[R28-R2 INSTALL] missing %s: %s',n,p);
q=sha_file(p);
fprintf(' %-38s SHA=%s | exact=%d
',n,q,strcmp(q,h));
assert(strcmp(q,h),'[R28-R2 INSTALL] %s SHA mismatch.',n);
end

function copybin(a,b)
f=fopen(a,'rb'); assert(f>=0,'cannot open %s',a); c=onCleanup(@()fclose(f)); x=fread(f,Inf,'*uint8'); clear c;
f=fopen(b,'wb'); assert(f>=0,'cannot write %s',b); c=onCleanup(@()fclose(f)); assert(fwrite(f,x,'uint8')==numel(x));
end

function tf=paths_equal(a,b)
if isempty(a)||isempty(b),tf=false;return;end
try,a=char(java.io.File(a).getCanonicalPath());b=char(java.io.File(b).getCanonicalPath());catch,end
if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end

function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0,'cannot open %s',p); c=onCleanup(@()fclose(f)); b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(typecast(b,'int8')); d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
