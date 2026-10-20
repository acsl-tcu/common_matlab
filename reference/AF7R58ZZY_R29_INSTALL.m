function OUT=AF7R58ZZY_R29_INSTALL()
% AF7R58ZZY-R29 theory-preserving Stage3 lean install.
% No production REPLANNING_SYSTEM.m or controller source is modified.
%
% R29 changes only:
%   1) exact canonical resource append avoids rewalking already-certified
%      historical control-point cells;
%   2) obstacle-axis retarget uses a bitwise-equivalent native MEX;
%   3) native/canonical binding consumes the already-closed resource metadata.
%
% Continuous geometry/C7/speed/sensor theorem, freshness gates, fail-closed
% rollback, R28 native trajectory representation, and four-cycle schedule remain.
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R29 | THEORY-PRESERVING STAGE3-LEAN INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | controllerModified=0 | certificateMathChanged=0 | scheduleChanged=0\n');

EXP=struct();
EXP.prod='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
EXP.r28runner='4122e1079eaa703d113a522099ef6aa5e376ed42c79c33e226ff61ccafe35c3a';
EXP.r28class='4edda21ff4a071754005a2c3aba174210d923542bf6b390ce2a21185f3faee2e';
EXP.r28cpp='bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a';
EXP.hpp='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
EXP.r25helper='8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44';
EXP.r29class='0e5eb4a63a1fffc57776034531bc5b78203b3935cb235f24e9fdd16177c2c184';
EXP.r29batch='78a686b9f33d85f224d734a9b6a9f7be63f602ad3b3d06096b46f75ac911b797';
EXP.r29mexsrc='794194666d68dead1a884467c82bb3dee484528a0f8a6f62f3cdab0570d07a60';
EXP.selftest='c1c304e726e5a6d13116f2d8f964ceb1cc4e8f1ff87e764d4c07a253a396a58d';
EXP.runner='967f7972d142db1f52c8fadb5d6421bfbb67c8ab98e4309e2800702f64dbd32a';
EXP.launcher='5a1b3810cda04655f2271293a95424416a3fd5707e70d1d37085bc06c80259bd';
EXP.report='a08d005ee278a1111ff911d4f13966a815af5810a1fb9e7d5e467c2bdde280f8';

pkg=fileparts(mfilename('fullpath'));
payload=fullfile(pkg,'payload');
assert(isfolder(payload),'[R29 INSTALL] payload folder missing.');

P=struct( ...
 'class',fullfile(payload,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT.m'), ...
 'batch',fullfile(payload,'af7r80_fast_batch_append_R29.m'), ...
 'mexsrc',fullfile(payload,'af7r79_packet_retarget_mex.cpp'), ...
 'selftest',fullfile(payload,'AF7R58ZZY_R29_SELFTEST.m'), ...
 'runner',fullfile(payload,'run_flight_headless_R29_FINAL_SOURCE.m'), ...
 'launcher',fullfile(payload,'AF7R58ZZY_R29_RUN_CANONICAL.m'), ...
 'report',fullfile(payload,'AF7R58ZZY_R29_REPORT.m'));

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
show('R29 class',P.class,EXP.r29class);
show('R29 fast batch helper',P.batch,EXP.r29batch);
show('R29 packet MEX source',P.mexsrc,EXP.r29mexsrc);
show('R29 selftest',P.selftest,EXP.selftest);
show('R29 runner',P.runner,EXP.runner);
show('R29 launcher',P.launcher,EXP.launcher);
show('R29 report',P.report,EXP.report);

root=find_root(EXP);
ref=fullfile(root,'reference');
C=struct( ...
 'runner',fullfile(root,'run_flight_headless.m'), ...
 'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
 'r21',fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m'), ...
 'r28class',fullfile(root,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m'), ...
 'r28cpp',fullfile(ref,'af7r36_single_core_mex.cpp'), ...
 'hpp',fullfile(ref,'af7r36_core.hpp'), ...
 'r25helper',fullfile(root,'af7r77_compile_correction_packet_R25.m'), ...
 'r29class',fullfile(root,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT.m'), ...
 'batch',fullfile(root,'af7r80_fast_batch_append_R29.m'), ...
 'mexsrc',fullfile(root,'af7r79_packet_retarget_mex.cpp'), ...
 'mexbin',fullfile(root,['af7r79_packet_retarget_mex.' mexext]), ...
 'selftest',fullfile(root,'AF7R58ZZY_R29_SELFTEST.m'), ...
 'launcher',fullfile(root,'AF7R58ZZY_R29_RUN_CANONICAL.m'), ...
 'report',fullfile(root,'AF7R58ZZY_R29_REPORT.m'));

cd(root); addpath(ref,'-begin'); addpath(root,'-begin'); rehash;

fprintf('\n[B. EXACT CURRENT R28 CHECKPOINT]\n');
show('R28 runner',C.runner,EXP.r28runner);
show('R28 class',C.r28class,EXP.r28class);
show('production',C.prod,EXP.prod);
show('R21 class',C.r21,EXP.r21);
show('R28 generator cpp',C.r28cpp,EXP.r28cpp);
show('AF7R36 hpp',C.hpp,EXP.hpp);
show('R25 compile helper',C.r25helper,EXP.r25helper);
rev=af7r36_single_core_mex(5);
assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1'), ...
    '[R29 INSTALL] active AF7R36 MEX is not exact R28.');

OUT=struct();
OUT.schema='AF7R58ZZY_R29_INSTALL_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.status='INIT';

fprintf('\n[C. SHADOW BUILD + EXACT SELFTEST -- NO LIVE WRITE]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
builddir=fullfile(root,['AF7R58ZZY_R29_BUILD_' stamp]);
[ok,msg]=mkdir(builddir); assert(ok,'%s',msg);
testName='af7r79_packet_retarget_mex_R29_TEST';
build_mex(P.mexsrc,builddir,testName);
addpath(payload,'-begin'); addpath(builddir,'-begin'); rehash;
cleanup=onCleanup(@()cleanup_paths(builddir,payload)); %#ok<NASGU>
T=AF7R58ZZY_R29_SELFTEST(testName);
assert(T.closed,'[R29 INSTALL] shadow exact selftest did not close.');
OUT.shadow_selftest=T;
clear cleanup;

% Static source semantics before any live write.
K=fileread(P.class);
assert(contains(K,'r29_prepare_core_bundle_exact')&& ...
    contains(K,'af7r80_fast_batch_append_R29')&& ...
    contains(K,'af7r79_packet_retarget_mex'), ...
    '[R29 INSTALL] R29 lean-core anchors missing.');
q=strfind(K,'function [ok,reason,B] = prepare_staged_reference_only_bundle');
assert(numel(q)==1,'[R29 INSTALL] prepare override nonunique.');
block=K(q(1):min(numel(K),q(1)+7000));
a=strfind(block,'if obj.r28_core_only_prepare_mode');
b=strfind(block,'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED');
assert(~isempty(a)&&~isempty(b)&&a(1)<b(1), ...
    '[R29 INSTALL] Flight core still enters R21 full prepare before lean path.');
assert(contains(K,'pipeline_intervals=3')&&contains(K,'pipeline_delay_s=3*double(obj.prediction_dt)'), ...
    '[R29 INSTALL] measurement schedule changed unexpectedly.');
Rtxt=fileread(P.runner);
assert(count_occ(Rtxt,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT(agent,nominal_ref,replan_opts)')==1, ...
    '[R29 INSTALL] R29 runner binding missing/nonunique.');

fprintf('\n[D. TRANSACTIONAL LIVE INSTALL]\n');
backup=fullfile(root,['AF7R58ZZY_R29_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
backup_one(C.runner,backup,'run_flight_headless.m');
backup_optional(C.r29class,backup,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT.m');
backup_optional(C.batch,backup,'af7r80_fast_batch_append_R29.m');
backup_optional(C.mexsrc,backup,'af7r79_packet_retarget_mex.cpp');
backup_optional(C.mexbin,backup,['af7r79_packet_retarget_mex.' mexext]);
backup_optional(C.selftest,backup,'AF7R58ZZY_R29_SELFTEST.m');
backup_optional(C.launcher,backup,'AF7R58ZZY_R29_RUN_CANONICAL.m');
backup_optional(C.report,backup,'AF7R58ZZY_R29_REPORT.m');
OUT.backup=backup;

try
    copybin(P.class,C.r29class);
    copybin(P.batch,C.batch);
    copybin(P.mexsrc,C.mexsrc);
    copybin(P.selftest,C.selftest);
    copybin(P.launcher,C.launcher);
    copybin(P.report,C.report);
    copybin(P.runner,C.runner);

    assert(strcmp(sha_file(C.r29class),EXP.r29class));
    assert(strcmp(sha_file(C.batch),EXP.r29batch));
    assert(strcmp(sha_file(C.mexsrc),EXP.r29mexsrc));
    assert(strcmp(sha_file(C.selftest),EXP.selftest));
    assert(strcmp(sha_file(C.launcher),EXP.launcher));
    assert(strcmp(sha_file(C.report),EXP.report));
    assert(strcmp(sha_file(C.runner),EXP.runner));

    try, clear af7r79_packet_retarget_mex; catch, end
    build_mex(C.mexsrc,root,'af7r79_packet_retarget_mex');
    rehash; addpath(ref,'-begin'); addpath(root,'-begin');

    assert(paths_equal(which('af7r79_packet_retarget_mex'),C.mexbin), ...
        '[R29 INSTALL] live packet MEX path shadowed.');
    T2=AF7R58ZZY_R29_SELFTEST('af7r79_packet_retarget_mex');
    assert(T2.closed,'[R29 INSTALL] live exact selftest did not close.');
    OUT.live_selftest=T2;

    mc=meta.class.fromName('REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT');
    assert(~isempty(mc),'[R29 INSTALL] R29 class parse failed.');
    assert(strcmp(sha_file(C.prod),EXP.prod),'[R29 INSTALL] production source changed.');
    assert(strcmp(sha_file(C.r28cpp),EXP.r28cpp),'[R29 INSTALL] R28 generator source changed.');
    assert(strcmp(sha_file(C.hpp),EXP.hpp),'[R29 INSTALL] AF7R36 core header changed.');

    OUT.closed=true;
    OUT.status='R29_INSTALLED__RUN_CANONICAL_NEXT';
catch ME
    fprintf('\n[R29 INSTALL FAILURE] %s\n',ME.message);
    try
        try, clear af7r79_packet_retarget_mex; catch, end
        copybin(fullfile(backup,'run_flight_headless.m'),C.runner);
        restore_optional(backup,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT.m',C.r29class);
        restore_optional(backup,'af7r80_fast_batch_append_R29.m',C.batch);
        restore_optional(backup,'af7r79_packet_retarget_mex.cpp',C.mexsrc);
        restore_optional(backup,['af7r79_packet_retarget_mex.' mexext],C.mexbin);
        restore_optional(backup,'AF7R58ZZY_R29_SELFTEST.m',C.selftest);
        restore_optional(backup,'AF7R58ZZY_R29_RUN_CANONICAL.m',C.launcher);
        restore_optional(backup,'AF7R58ZZY_R29_REPORT.m',C.report);
        rehash; addpath(ref,'-begin'); addpath(root,'-begin');
        assert(strcmp(sha_file(C.runner),EXP.r28runner), ...
            '[R29 INSTALL] rollback runner mismatch.');
        assert(strcmp(sha_file(C.prod),EXP.prod), ...
            '[R29 INSTALL] rollback production mismatch.');
        fprintf(' rollback to R28 checkpoint              = PASS\n');
    catch MER
        fprintf(' rollback                                = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R29_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R29_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n');
fprintf(' R29 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf(' status                                  = %s\n',OUT.status);
fprintf(' production source modified              = 0\n');
fprintf(' controller source modified              = 0\n');
fprintf(' R28 generator/core math modified        = 0\n');
fprintf(' NEXT: AF7R58ZZY_R29_RUN_CANONICAL\n');
fprintf(' Then: AF7R58ZZY_R29_REPORT\n');
fprintf('====================================================================================================\n');
end

function root=find_root(EXP)
starts=[{pwd,fileparts(mfilename('fullpath'))},strsplit(path,pathsep)];
cand={};
for k=1:numel(starts)
    p=starts{k};
    if isempty(p)||~isfolder(p), continue; end
    for j=1:7
        cand{end+1}=p; %#ok<AGROW>
        q=fileparts(p); if isempty(q)||strcmp(q,p), break; end; p=q;
    end
end
cand=unique(cand); good={};
for k=1:numel(cand)
    r=cand{k};
    if isfile(fullfile(r,'run_flight_headless.m'))&& ...
       isfile(fullfile(r,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m'))&& ...
       isfile(fullfile(r,'reference','REPLANNING_SYSTEM.m'))&& ...
       strcmp(sha_file(fullfile(r,'run_flight_headless.m')),EXP.r28runner)&& ...
       strcmp(sha_file(fullfile(r,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m')),EXP.r28class)&& ...
       strcmp(sha_file(fullfile(r,'reference','REPLANNING_SYSTEM.m')),EXP.prod)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good),'[R29 INSTALL] exact R28 common_matlab root not found.');
score=zeros(size(good));
for k=1:numel(good)
    [~,n]=fileparts(good{k});
    score(k)=100*strcmpi(n,'common_matlab')+20*isfolder(fullfile(good{k},'.git'));
end
[~,ix]=max(score); root=good{ix};
end

function build_mex(src,outdir,name)
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output',name,'-outdir',outdir,src);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output',name,'-outdir',outdir,src);
end
rehash;
assert(isfile(fullfile(outdir,[name '.' mexext])), ...
    '[R29 INSTALL] MEX build output missing.');
end

function cleanup_paths(builddir,payload)
if path_has(builddir), rmpath(builddir); end
if path_has(payload), rmpath(payload); end
rehash;
end
function tf=path_has(p)
e=strsplit(path,pathsep); tf=false;
for k=1:numel(e), if paths_equal(e{k},p), tf=true; return; end; end
end
function backup_one(src,dst,name)
assert(isfile(src)); copyfile(src,fullfile(dst,name),'f');
end
function backup_optional(src,dst,name)
if isfile(src), copyfile(src,fullfile(dst,name),'f');
else
    f=fopen(fullfile(dst,[name '.ABSENT']),'wt'); assert(f>=0); fprintf(f,'ABSENT\n'); fclose(f);
end
end
function restore_optional(dirp,name,dst)
s=fullfile(dirp,name); a=fullfile(dirp,[name '.ABSENT']);
if isfile(s), copyfile(s,dst,'f');
elseif isfile(a)&&isfile(dst), delete(dst);
end
end
function show(n,p,h)
assert(isfile(p),'[R29 INSTALL] missing %s: %s',n,p);
q=sha_file(p); fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));
assert(strcmp(q,h),'[R29 INSTALL] %s SHA mismatch.',n);
end
function n=count_occ(s,p), n=numel(strfind(s,p)); end
function copybin(a,b)
f=fopen(a,'rb'); assert(f>=0); c=onCleanup(@()fclose(f)); x=fread(f,Inf,'*uint8'); clear c;
f=fopen(b,'wb'); assert(f>=0); c=onCleanup(@()fclose(f)); n=fwrite(f,x,'uint8'); assert(n==numel(x));
end
function tf=paths_equal(a,b)
if isempty(a)||isempty(b),tf=false;return;end
try,a=char(java.io.File(a).getCanonicalPath());b=char(java.io.File(b).getCanonicalPath());catch,end
if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end
function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0); c=onCleanup(@()fclose(f)); b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end
