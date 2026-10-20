function OUT=AF7R58ZZY_R33_INSTALL()
% AF7R58ZZY-R33 transactional install from exact flown R32 checkpoint.
% R33 runtime is exact R31 behavior plus removal of unused P.native_raw=RAW.
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R33 | R31 + NO UNUSED NATIVE_RAW INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] isolatedRuntimeChange=1 | productionModified=0 | controllerModified=0 | certificateMathChanged=0 | scheduleChanged=0 | mexChanged=0\n');

EXP=struct();
EXP.prod='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r32runner='9f0f42401ca2eb8674e986d3ed709d579e47c77ef51d703c31b0025398d3464e';
EXP.r32class='6a596f80168846d3f65f2f1dd0e8f501cbabc0455f64c51b0681bd32fa8153ba';
EXP.packetSource='794194666d68dead1a884467c82bb3dee484528a0f8a6f62f3cdab0570d07a60';
EXP.r33class='1e79242e5a03431afd285584b11f2446f972c9f109f1fc0847fd113a7af8e38c';
EXP.r33runner='41eb4a24a131382d22a44c21979545ec00f0a779fd3fed4ac1976c73416b6c8c';
EXP.selftest='ab55047bdff8c0f0d32b7b4f0df98b97c361445ef25cf5d1fb0cb2a78cf02df9';
EXP.launcher='aaddef643e575d89c6b0adfbf36e8d0e177ccd3c623c6ae7736af7fad3629b1e';
EXP.report='741902fa2509436d97b414018d3bfaa5cb3544c052463933163d57f7b9face3c';

pkg=fileparts(mfilename('fullpath'));
payload=fullfile(pkg,'payload');
assert(isfolder(payload),'[R33 INSTALL] payload folder missing.');

P=struct( ...
    'class',fullfile(payload,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW.m'), ...
    'runner',fullfile(payload,'run_flight_headless_R33_FINAL_SOURCE.m'), ...
    'selftest',fullfile(payload,'AF7R58ZZY_R33_SELFTEST.m'), ...
    'launcher',fullfile(payload,'AF7R58ZZY_R33_RUN_CANONICAL.m'), ...
    'report',fullfile(payload,'AF7R58ZZY_R33_REPORT.m'));

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
show('R33 class',P.class,EXP.r33class);
show('R33 runner',P.runner,EXP.r33runner);
show('R33 selftest',P.selftest,EXP.selftest);
show('R33 launcher',P.launcher,EXP.launcher);
show('R33 report',P.report,EXP.report);

root=find_root(EXP);
ref=fullfile(root,'reference');
C=struct( ...
    'runner',fullfile(root,'run_flight_headless.m'), ...
    'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
    'r32class',fullfile(root,'REPLANNING_SYSTEM_R32_DIRECT_CANDIDATE_STORE.m'), ...
    'packetSource',fullfile(root,'af7r79_packet_retarget_mex.cpp'), ...
    'packetMex',fullfile(root,['af7r79_packet_retarget_mex.' mexext]), ...
    'r33class',fullfile(root,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW.m'), ...
    'selftest',fullfile(root,'AF7R58ZZY_R33_SELFTEST.m'), ...
    'launcher',fullfile(root,'AF7R58ZZY_R33_RUN_CANONICAL.m'), ...
    'report',fullfile(root,'AF7R58ZZY_R33_REPORT.m'));

cd(root); addpath(ref,'-begin'); addpath(root,'-begin'); rehash;

fprintf('\n[B. EXACT FLOWN R32 CHECKPOINT]\n');
show('R32 runner',C.runner,EXP.r32runner);
show('R32 class',C.r32class,EXP.r32class);
show('production',C.prod,EXP.prod);
show('packet MEX source',C.packetSource,EXP.packetSource);
assert(isfile(C.packetMex),'[R33 INSTALL] packet MEX missing.');
rev=af7r79_packet_retarget_mex(2);
assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1'), ...
    '[R33 INSTALL] packet MEX revision mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R33_INSTALL_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.mex_modified=false;
OUT.status='INIT';

fprintf('\n[C. SHADOW PARSE + ISOLATED-CHANGE SELFTEST -- NO LIVE WRITE]\n');
addpath(payload,'-begin'); rehash;
cleanup=onCleanup(@()cleanup_payload(payload)); %#ok<NASGU>
assert(paths_equal(which('REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW'),P.class), ...
    '[R33 INSTALL] shadow class binding mismatch.');
mc=meta.class.fromName('REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW');
assert(~isempty(mc),'[R33 INSTALL] shadow class parse failed.');
T=AF7R58ZZY_R33_SELFTEST(P.class);
assert(T.closed,'[R33 INSTALL] shadow selftest did not close.');
OUT.shadow_selftest=T;
clear cleanup;

fprintf('\n[D. TRANSACTIONAL LIVE INSTALL]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,['AF7R58ZZY_R33_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
backup_one(C.runner,backup,'run_flight_headless.m');
backup_optional(C.r33class,backup,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW.m');
backup_optional(C.selftest,backup,'AF7R58ZZY_R33_SELFTEST.m');
backup_optional(C.launcher,backup,'AF7R58ZZY_R33_RUN_CANONICAL.m');
backup_optional(C.report,backup,'AF7R58ZZY_R33_REPORT.m');
OUT.backup=backup;

try
    copybin(P.class,C.r33class);
    copybin(P.runner,C.runner);
    copybin(P.selftest,C.selftest);
    copybin(P.launcher,C.launcher);
    copybin(P.report,C.report);

    assert(strcmp(sha_file(C.r33class),EXP.r33class));
    assert(strcmp(sha_file(C.runner),EXP.r33runner));
    assert(strcmp(sha_file(C.selftest),EXP.selftest));
    assert(strcmp(sha_file(C.launcher),EXP.launcher));
    assert(strcmp(sha_file(C.report),EXP.report));

    rehash; addpath(ref,'-begin'); addpath(root,'-begin');
    T2=AF7R58ZZY_R33_SELFTEST(C.r33class);
    assert(T2.closed,'[R33 INSTALL] live selftest did not close.');
    OUT.live_selftest=T2;

    mc2=meta.class.fromName('REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW');
    assert(~isempty(mc2),'[R33 INSTALL] live class parse failed.');
    Rtxt=fileread(C.runner);
    assert(count_occ(Rtxt,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW(agent,nominal_ref,replan_opts)')==1, ...
        '[R33 INSTALL] runner binding missing/nonunique.');
    assert(~contains(Rtxt,'REPLANNING_SYSTEM_R32_DIRECT_CANDIDATE_STORE(agent,nominal_ref,replan_opts)'), ...
        '[R33 INSTALL] stale R32 runner binding remains.');

    assert(strcmp(sha_file(C.prod),EXP.prod),'[R33 INSTALL] production changed.');
    assert(strcmp(sha_file(C.packetSource),EXP.packetSource), ...
        '[R33 INSTALL] packet MEX source changed.');

    OUT.closed=true;
    OUT.status='R33_INSTALLED__RUN_CANONICAL_NEXT';
catch ME
    fprintf('\n[R33 INSTALL FAILURE] %s\n',ME.message);
    try
        copybin(fullfile(backup,'run_flight_headless.m'),C.runner);
        restore_optional(backup,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW.m',C.r33class);
        restore_optional(backup,'AF7R58ZZY_R33_SELFTEST.m',C.selftest);
        restore_optional(backup,'AF7R58ZZY_R33_RUN_CANONICAL.m',C.launcher);
        restore_optional(backup,'AF7R58ZZY_R33_REPORT.m',C.report);
        rehash; addpath(ref,'-begin'); addpath(root,'-begin');
        assert(strcmp(sha_file(C.runner),EXP.r32runner), ...
            '[R33 INSTALL] rollback runner mismatch.');
        assert(strcmp(sha_file(C.prod),EXP.prod), ...
            '[R33 INSTALL] rollback production mismatch.');
        fprintf(' rollback to exact R32 checkpoint          = PASS\n');
    catch MER
        fprintf(' rollback                                = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R33_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R33_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n');
fprintf(' R33 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf(' status                                  = %s\n',OUT.status);
fprintf(' isolated runtime changes                = 1\n');
fprintf(' production source modified              = 0\n');
fprintf(' controller source modified              = 0\n');
fprintf(' MEX source/binary modified              = 0\n');
fprintf(' continuous certificate math modified    = 0\n');
fprintf(' measurement schedule changed            = 0\n');
fprintf(' NEXT: AF7R58ZZY_R33_RUN_CANONICAL\n');
fprintf(' Then: AF7R58ZZY_R33_REPORT\n');
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
    rr=fullfile(r,'run_flight_headless.m');
    cc=fullfile(r,'REPLANNING_SYSTEM_R32_DIRECT_CANDIDATE_STORE.m');
    pp=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    if isfile(rr)&&isfile(cc)&&isfile(pp)&& ...
            strcmp(sha_file(rr),EXP.r32runner)&& ...
            strcmp(sha_file(cc),EXP.r32class)&& ...
            strcmp(sha_file(pp),EXP.prod)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good),'[R33 INSTALL] exact flown R32 root not found.');
score=zeros(size(good));
for k=1:numel(good)
    [~,n]=fileparts(good{k});
    score(k)=100*strcmpi(n,'common_matlab')+20*isfolder(fullfile(good{k},'.git'));
end
[~,ix]=max(score); root=good{ix};
end

function cleanup_payload(payload)
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
    f=fopen(fullfile(dst,[name '.ABSENT']),'wt'); assert(f>=0);
    fprintf(f,'ABSENT\n'); fclose(f);
end
end
function restore_optional(dirp,name,dst)
s=fullfile(dirp,name); a=fullfile(dirp,[name '.ABSENT']);
if isfile(s), copyfile(s,dst,'f');
elseif isfile(a)&&isfile(dst), delete(dst);
end
end
function show(n,p,h)
assert(isfile(p),'[R33 INSTALL] missing %s: %s',n,p);
q=sha_file(p); fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));
assert(strcmp(q,h),'[R33 INSTALL] %s SHA mismatch.',n);
end
function n=count_occ(s,p), n=numel(strfind(s,p)); end
function copybin(a,b)
f=fopen(a,'rb'); assert(f>=0); c=onCleanup(@()fclose(f));
x=fread(f,Inf,'*uint8'); clear c;
f=fopen(b,'wb'); assert(f>=0); c=onCleanup(@()fclose(f));
n=fwrite(f,x,'uint8'); assert(n==numel(x));
end
function tf=paths_equal(a,b)
if isempty(a)||isempty(b),tf=false;return;end
try,a=char(java.io.File(a).getCanonicalPath());b=char(java.io.File(b).getCanonicalPath());catch,end
if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end
function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0); c=onCleanup(@()fclose(f));
b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
