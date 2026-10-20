function OUT=AF7R58ZZY_R28_R4_INSTALL()
% AF7R58ZZY-R28-R4 safe transactional native-once install.
% =========================================================================
% R4 fixes the R3 installer-only defects found after the successful shadow selftest:
%   1) destination field typo (launcher field).
%   2) duplicate shadow-path cleanup that produced harmless rmpath warnings.
% R3 rolled back to coherent R27 after the field typo, so this install starts
% from and re-verifies exact R27 authority before any live R28 activation.
%
% This installer still handles both valid preinstall states:
%   (A) exact R27 baseline cpp
%   (B) exact known R28-V1 extraction-collision cpp
%
% The R28 mathematical design/payload is unchanged:
%   - trajectory generator mathematics unchanged
%   - continuous certificate mathematics unchanged
%   - four-cycle measurement schedule unchanged
%   - controller unchanged
%   - native dependent representation added only after successful candidate
%
% No Flight is run here.
% =========================================================================
clc;
disp('====================================================================================================');
disp(' AF7R58ZZY-R28-R4 | SAFE TRANSACTIONAL NATIVE-ONCE INSTALL');
disp('====================================================================================================');
disp('[AUTHORITY] installerSyntaxFixed=1 | packageCollisionFixed=1 | generatorMathChanged=0 | certificateMathChanged=0 | scheduleChanged=0 | controllerModified=0');

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
assert(isfolder(payload)&&isfolder(baseline), ...
    '[R28-R4 INSTALL] payload/baseline folders missing.');

pCpp=fullfile(payload,'af7r36_single_core_mex.cpp');
pHpp=fullfile(payload,'af7r36_core.hpp');
pClass=fullfile(payload,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m');
pRunner=fullfile(payload,'run_flight_headless_R28_FINAL_SOURCE.m');
pReport=fullfile(payload,'AF7R58ZZY_R28_REPORT.m');
pSelf=fullfile(payload,'AF7R58ZZY_R28_SELFTEST.m');
pLaunch=fullfile(payload,'AF7R58ZZY_R28_RUN_CANONICAL.m');
pBaseCpp=fullfile(baseline,'af7r36_single_core_mex.cpp');

disp('');
disp('[A. PACKAGE BYTE AUTHORITY]');
show_exact('R27 baseline cpp payload',pBaseCpp,EXP.cpp0);
show_exact('R28 modified cpp payload',pCpp,EXP.r28cpp);
show_exact('unchanged AF7R36 hpp payload',pHpp,EXP.hpp);
show_exact('R28 class payload',pClass,EXP.r28class);
show_exact('R28 runner payload',pRunner,EXP.r28runner);
show_exact('R28 report payload',pReport,EXP.report);
show_exact('R28 selftest payload',pSelf,EXP.selftest);
show_exact('R28 launcher payload',pLaunch,EXP.launcher);

root=find_root(EXP);
ref=fullfile(root,'reference');
C=struct( ...
    'runner',fullfile(root,'run_flight_headless.m'), ...
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

requiredC={'runner','prod','r27','cpp','hpp','r25helper','helper','mex', ...
    'r28class','report','selftest','launcher'};
assert(all(isfield(C,requiredC)), ...
    '[R28-R4 INSTALL] internal destination-field contract is incomplete.');

cd(root);
addpath(ref,'-begin');
addpath(root,'-begin');
rehash;

disp('');
disp('[B. EXACT ACTIVE R27 AUTHORITY -- WITH KNOWN V1 EXTRACTION-COLLISION RECOGNITION]');
show_exact('runner',C.runner,EXP.r27runner);
show_exact('R27 class',C.r27,EXP.r27class);
show_exact('production',C.prod,EXP.prod);
show_exact('AF7R36 hpp',C.hpp,EXP.hpp);
show_exact('R25 packet helper',C.r25helper,EXP.r25helper);
show_exact('R27 helper',C.helper,EXP.r27helper);

assert(paths_equal(which('af7r77_compile_correction_packet_R25'),C.r25helper), ...
    '[R28-R4 INSTALL] R25 helper path shadowed.');
assert(paths_equal(which('af7r78_incremental_compile_R27'),C.helper), ...
    '[R28-R4 INSTALL] R27 helper path shadowed.');
assert(exist('af7r77_compile_dcp_mex','file')==3, ...
    '[R28-R4 INSTALL] R25 DCP reference MEX missing.');
assert(isfile(C.mex),'[R28-R4 INSTALL] current AF7R36 MEX missing.');

cppNow=sha_file(C.cpp);
if strcmp(cppNow,EXP.cpp0)
    sourceState='R27_BASELINE';
    collision=false;
elseif strcmp(cppNow,EXP.r28cpp)
    sourceState='KNOWN_R28_V1_EXTRACTION_COLLISION';
    collision=true;
else
    error('[R28-R4 INSTALL] canonical AF7R36 cpp is neither exact R27 baseline nor exact known R28 V1 payload: %s',cppNow);
end
fprintf(' AF7R36 cpp current SHA                  = %s\n',cppNow);
fprintf(' AF7R36 cpp state                        = %s\n',sourceState);
fprintf(' known package-extraction collision      = %d\n',collision);

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,['AF7R58ZZY_R28_R4_BACKUP_' stamp]);
[ok,msg]=mkdir(backup);
assert(ok,'%s',msg);
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
OUT.schema='AF7R58ZZY_R28_R4_INSTALL_V1';
OUT.closed=false;
OUT.backup=backup;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.package_extraction_collision_detected=collision;
OUT.preinstall_cpp_sha=cppNow;
OUT.preinstall_cpp_state=sourceState;
OUT.status='INIT';

try
    disp('');
    disp('[C. NORMALIZE PREINSTALL SOURCE AUTHORITY]');
    if collision
        copybin(pBaseCpp,C.cpp);
        assert(strcmp(sha_file(C.cpp),EXP.cpp0), ...
            '[R28-R4 INSTALL] baseline cpp recovery failed.');
        disp(' accidental R28 source overwrite repaired = 1');
    else
        disp(' accidental R28 source overwrite repaired = 0 (not needed)');
    end
    fprintf(' canonical cpp baseline SHA              = %s\n',sha_file(C.cpp));

    disp('');
    disp('[D. BUILD SHADOW R28 MEX -- LIVE MEX UNCHANGED]');
    builddir=fullfile(root,['AF7R58ZZY_R28_R4_BUILD_' stamp]);
    [ok,msg]=mkdir(builddir);
    assert(ok,'%s',msg);
    testName='af7r36_single_core_mex_R28_R4_TEST';
    build_mex(pCpp,payload,builddir,testName);

    addpath(payload,'-begin');
    addpath(builddir,'-begin');
    rehash;
    cleanupPath=onCleanup(@()cleanup_shadow_paths(builddir,payload)); %#ok<NASGU>

    T=AF7R58ZZY_R28_SELFTEST(testName,pClass);
    assert(T.closed,'[R28-R4 INSTALL] shadow selftest did not close.');
    OUT.shadow_selftest=T;

    clear cleanupPath; % executes cleanup exactly once

    disp('');
    disp('[E. INSTALL EXACT R28 SOURCE / CLASS / RUNNER]');
    copybin(pCpp,C.cpp);
    copybin(pHpp,C.hpp);
    copybin(pClass,C.r28class);
    copybin(pRunner,C.runner);
    copybin(pReport,C.report);
    copybin(pSelf,C.selftest);
    copybin(pLaunch,C.launcher);

    assert(strcmp(sha_file(C.cpp),EXP.r28cpp), ...
        '[R28-R4 INSTALL] installed cpp mismatch.');
    assert(strcmp(sha_file(C.hpp),EXP.hpp), ...
        '[R28-R4 INSTALL] installed hpp mismatch.');
    assert(strcmp(sha_file(C.runner),EXP.r28runner), ...
        '[R28-R4 INSTALL] installed runner mismatch.');
    assert(strcmp(sha_file(C.r28class),EXP.r28class), ...
        '[R28-R4 INSTALL] installed R28 class mismatch.');
    assert(strcmp(sha_file(C.report),EXP.report), ...
        '[R28-R4 INSTALL] installed report mismatch.');
    assert(strcmp(sha_file(C.selftest),EXP.selftest), ...
        '[R28-R4 INSTALL] installed selftest mismatch.');
    assert(strcmp(sha_file(C.launcher),EXP.launcher), ...
        '[R28-R4 INSTALL] installed launcher mismatch.');

    disp('');
    disp('[F. BUILD LIVE AF7R36 MEX]');
    try
        clear af7r36_single_core_mex;
    catch
    end
    build_mex(C.cpp,ref,ref,'af7r36_single_core_mex');
    rehash;
    addpath(ref,'-begin');
    addpath(root,'-begin');

    assert(paths_equal(which('af7r36_single_core_mex'),C.mex), ...
        '[R28-R4 INSTALL] live AF7R36 MEX path shadowed.');
    rev=af7r36_single_core_mex(5);
    assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1'), ...
        '[R28-R4 INSTALL] live MEX revision mismatch.');

    disp('');
    disp('[G. LIVE EXACT EQUIVALENCE SELFTEST]');
    T2=AF7R58ZZY_R28_SELFTEST('af7r36_single_core_mex',C.r28class);
    assert(T2.closed,'[R28-R4 INSTALL] live selftest did not close.');
    OUT.live_selftest=T2;

    disp('');
    disp('[H. CLASS / RUNNER / PRODUCTION IMMUTABILITY]');
    try
        clear REPLANNING_SYSTEM_R28_NATIVE_ONCE;
    catch
    end
    rehash;
    mc=meta.class.fromName('REPLANNING_SYSTEM_R28_NATIVE_ONCE');
    assert(~isempty(mc),'[R28-R4 INSTALL] R28 class parse failed.');

    R=fileread(C.runner);
    assert(count(R,'REPLANNING_SYSTEM_R28_NATIVE_ONCE(agent,nominal_ref,replan_opts)')==1, ...
        '[R28-R4 INSTALL] R28 runner binding missing/nonunique.');
    assert(~contains(R,'REPLANNING_SYSTEM_R27_LEAN_EXACT(agent,nominal_ref,replan_opts)'), ...
        '[R28-R4 INSTALL] stale R27 runner binding remains.');
    assert(strcmp(sha_file(C.prod),EXP.prod), ...
        '[R28-R4 INSTALL] production MATLAB source changed.');
    assert(paths_equal(which('AF7R58ZZY_R28_REPORT'),C.report)&& ...
        strcmp(sha_file(C.report),EXP.report), ...
        '[R28-R4 INSTALL] report binding mismatch.');
    assert(paths_equal(which('AF7R58ZZY_R28_SELFTEST'),C.selftest)&& ...
        strcmp(sha_file(C.selftest),EXP.selftest), ...
        '[R28-R4 INSTALL] selftest binding mismatch.');
    assert(paths_equal(which('AF7R58ZZY_R28_RUN_CANONICAL'),C.launcher)&& ...
        strcmp(sha_file(C.launcher),EXP.launcher), ...
        '[R28-R4 INSTALL] launcher binding mismatch.');

    disp(' R28 class parse                         = PASS');
    disp(' production MATLAB source modified       = 0');
    disp(' controller source modified              = 0');

    OUT.closed=true;
    OUT.status='R28_R4_INSTALLED__RUN_CANONICAL_NEXT';

catch ME
    fprintf('\n[R28-R3 INSTALL FAILURE] %s\n',ME.message);
    try
        try
            clear af7r36_single_core_mex;
        catch
        end
        copybin(fullfile(backup,'1_run_flight_headless.m'),C.runner);
        copybin(pBaseCpp,C.cpp);
        copybin(fullfile(backup,'4_af7r36_core.hpp'),C.hpp);
        copybin(fullfile(backup,['5_af7r36_single_core_mex.' mexext]),C.mex);
        restore_optional(backup,'6_pre_R28_class.m',C.r28class);
        restore_optional(backup,'7_pre_R28_report.m',C.report);
        restore_optional(backup,'8_pre_R28_selftest.m',C.selftest);
        restore_optional(backup,'9_pre_R28_launcher.m',C.launcher);
        rehash;
        addpath(ref,'-begin');
        addpath(root,'-begin');

        assert(strcmp(sha_file(C.runner),EXP.r27runner), ...
            'rollback runner mismatch');
        assert(strcmp(sha_file(C.cpp),EXP.cpp0), ...
            'rollback cpp mismatch');
        assert(strcmp(sha_file(C.hpp),EXP.hpp), ...
            'rollback hpp mismatch');
        disp(' rollback to coherent R27 source/binary = PASS');
    catch MER
        fprintf(' rollback                               = FAILED: %s\n',MER.message);
    end

    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R28_R4_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R28_R4_INSTALL_OUT',OUT);
disp('');
disp('====================================================================================================');
disp(' R28-R4 INSTALL COMPLETE');
disp('====================================================================================================');
fprintf('status                                  = %s\n',OUT.status);
fprintf('backup                                  = %s\n',backup);
fprintf('package extraction collision detected   = %d\n',collision);
disp('production MATLAB source modified       = 0');
disp('controller source modified              = 0');
disp('NEXT: AF7R58ZZY_R28_RUN_CANONICAL');
disp('Then: AF7R58ZZY_R28_REPORT');
disp('====================================================================================================');
end

function root=find_root(EXP)
starts=[{pwd,fileparts(mfilename('fullpath'))},strsplit(path,pathsep)];
cand={};
for k=1:numel(starts)
    p=starts{k};
    if isempty(p)||~isfolder(p)
        continue;
    end
    for j=1:7
        cand{end+1}=p; %#ok<AGROW>
        q=fileparts(p);
        if isempty(q)||strcmp(q,p)
            break;
        end
        p=q;
    end
end
cand=unique(cand);
good={};
for k=1:numel(cand)
    r=cand{k};
    p=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    rr=fullfile(r,'run_flight_headless.m');
    c=fullfile(r,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
    if isfile(p)&&isfile(rr)&&isfile(c)&& ...
            strcmp(sha_file(p),EXP.prod)&& ...
            strcmp(sha_file(rr),EXP.r27runner)&& ...
            strcmp(sha_file(c),EXP.r27class)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good), ...
    '[R28-R4 INSTALL] exact canonical R27 root not found.');
score=zeros(size(good));
for k=1:numel(good)
    [~,n]=fileparts(good{k});
    score(k)=100*strcmpi(n,'common_matlab')+ ...
        20*isfolder(fullfile(good{k},'.git'));
end
[~,i]=max(score);
root=good{i};
end

function build_mex(src,inc,outdir,name)
if ispc
    mex('-R2018a','-O', ...
        'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' inc],'-output',name,'-outdir',outdir,src);
else
    mex('-R2018a','-O', ...
        'CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' inc],'-output',name,'-outdir',outdir,src);
end
rehash;
assert(isfile(fullfile(outdir,[name '.' mexext])), ...
    '[R28-R4 INSTALL] MEX build output missing.');
end

function cleanup_shadow_paths(builddir,payload)
% Idempotent: remove only entries that are currently on MATLAB path.
if path_has_entry(builddir)
    rmpath(builddir);
end
if path_has_entry(payload)
    rmpath(payload);
end
rehash;
end

function tf=path_has_entry(p)
entries=strsplit(path,pathsep);
tf=false;
for k=1:numel(entries)
    if paths_equal(entries{k},p)
        tf=true;
        return;
    end
end
end

function backup_one(src,dirp,name)
assert(isfile(src), ...
    '[R28-R4 INSTALL] backup source missing: %s',src);
copyfile(src,fullfile(dirp,name),'f');
end

function backup_optional(src,dirp,name)
marker=fullfile(dirp,[name '.ABSENT']);
if isfile(src)
    copyfile(src,fullfile(dirp,name),'f');
else
    f=fopen(marker,'wt');
    assert(f>=0,'cannot create absent marker');
    fprintf(f,'%s\n','ABSENT');
    fclose(f);
end
end

function restore_optional(dirp,name,dst)
src=fullfile(dirp,name);
marker=fullfile(dirp,[name '.ABSENT']);
if isfile(src)
    copyfile(src,dst,'f');
elseif isfile(marker)&&isfile(dst)
    delete(dst);
end
end

function show_exact(n,p,h)
assert(isfile(p),'[R28-R4 INSTALL] missing %s: %s',n,p);
q=sha_file(p);
fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));
assert(strcmp(q,h),'[R28-R4 INSTALL] %s SHA mismatch.',n);
end

function copybin(a,b)
f=fopen(a,'rb');
assert(f>=0,'cannot open %s',a);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
x=fread(f,Inf,'*uint8');
clear c;

f=fopen(b,'wb');
assert(f>=0,'cannot write %s',b);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
n=fwrite(f,x,'uint8');
assert(n==numel(x),'short write: %s',b);
end

function tf=paths_equal(a,b)
if isempty(a)||isempty(b)
    tf=false;
    return;
end
try
    a=char(java.io.File(a).getCanonicalPath());
    b=char(java.io.File(b).getCanonicalPath());
catch
end
if ispc
    tf=strcmpi(a,b);
else
    tf=strcmp(a,b);
end
end

function h=sha_file(p)
f=fopen(p,'rb');
assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
