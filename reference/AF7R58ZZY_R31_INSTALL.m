function OUT=AF7R58ZZY_R31_INSTALL()
% AF7R58ZZY-R31 direct-stage-store install from exact flown R30 checkpoint.
%
% Performance diagnosis encoded by this revision:
%   R30 inner prepare = 6.9180 ms
%   R30 outer core    = 13.0778 ms
%   gap               = 6.1598 ms
% The only operations in that outer-only interval are candidate/property/token
% handling, method output transfer and cleanup.  R31 removes the large staged
% bundle value return, removes the second external bundle assignment, and removes
% the onCleanup object from the Stage3 real-time path.
%
% No safety/certificate mathematics or four-cycle schedule is changed.
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R31 | DIRECT-STAGE-STORE INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | controllerModified=0 | certificateMathChanged=0 | scheduleChanged=0 | mexChanged=0\n');

EXP=struct();
EXP.prod='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
EXP.r30runner='d7f403cb7c25fab1d1bcc288abf69b5e2e4ae7790b44a6380fbfa58e1d457376';
EXP.r30class='ca1b8e2a5dbffecf57a777c023ff5c210b974252d0ff0e71a90709ebdeb70fec';
EXP.r28cpp='bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a';
EXP.hpp='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
EXP.packetSource='794194666d68dead1a884467c82bb3dee484528a0f8a6f62f3cdab0570d07a60';
EXP.batch='92f093168f1df95661c5e7f2e75005fe439406eb3ec22e53fd1f25d6bdfb8a15';
EXP.handles='1c9bf7407b9c2482c5f6d2676479966b808995c4d10ee2871a620faba55a82f8';
EXP.spacing='ba92a9bb05e5cea6916f904646ff416a8379ff5ed9ed4424baf362eacb43054e';

EXP.r31class='4137e87df85b92fad98b96cc01c42098fde527ca915868ee844939adc151e026';
EXP.r31runner='3fa4cf97d396d7e7891d94da3a986a061f59556f2743c81d979204188484b03b';
EXP.selftest='564040c31a071245365803a86cdf387fb39f6bdea24e34eaf349f0b70fe67cfe';
EXP.launcher='a8e460cddadcdc0957b8a6cba19752fc599f18df3653c7eb3d76881b4033a483';
EXP.report='dc4090142107ff77b39c92610a6354f6acee8b5e9a14cd00d542b39b195376da';

pkg=fileparts(mfilename('fullpath'));
payload=fullfile(pkg,'payload');
assert(isfolder(payload),'[R31 INSTALL] payload folder missing.');

P=struct( ...
    'class',fullfile(payload,'REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE.m'), ...
    'runner',fullfile(payload,'run_flight_headless_R31_FINAL_SOURCE.m'), ...
    'selftest',fullfile(payload,'AF7R58ZZY_R31_SELFTEST.m'), ...
    'launcher',fullfile(payload,'AF7R58ZZY_R31_RUN_CANONICAL.m'), ...
    'report',fullfile(payload,'AF7R58ZZY_R31_REPORT.m'));

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
show('R31 class',P.class,EXP.r31class);
show('R31 runner',P.runner,EXP.r31runner);
show('R31 selftest',P.selftest,EXP.selftest);
show('R31 launcher',P.launcher,EXP.launcher);
show('R31 report',P.report,EXP.report);

root=find_root(EXP);
ref=fullfile(root,'reference');
C=struct( ...
    'runner',fullfile(root,'run_flight_headless.m'), ...
    'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
    'r21',fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m'), ...
    'r30class',fullfile(root,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT.m'), ...
    'r28cpp',fullfile(ref,'af7r36_single_core_mex.cpp'), ...
    'hpp',fullfile(ref,'af7r36_core.hpp'), ...
    'packetSource',fullfile(root,'af7r79_packet_retarget_mex.cpp'), ...
    'packetMex',fullfile(root,['af7r79_packet_retarget_mex.' mexext]), ...
    'batch',resolve_exact_source('af7r58i_prepare_batch_model',EXP.batch,root,ref), ...
    'handles',resolve_exact_source('af7r58i_make_canonical_eval_handles',EXP.handles,root,ref), ...
    'spacing',resolve_exact_source('af7r58i_commit_spacing_guard',EXP.spacing,root,ref), ...
    'r31class',fullfile(root,'REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE.m'), ...
    'selftest',fullfile(root,'AF7R58ZZY_R31_SELFTEST.m'), ...
    'launcher',fullfile(root,'AF7R58ZZY_R31_RUN_CANONICAL.m'), ...
    'report',fullfile(root,'AF7R58ZZY_R31_REPORT.m'));

cd(root); addpath(ref,'-begin'); addpath(root,'-begin'); rehash;

fprintf('\n[B. EXACT FLOWN R30 CHECKPOINT]\n');
show('R30 runner',C.runner,EXP.r30runner);
show('R30 class',C.r30class,EXP.r30class);
show('production',C.prod,EXP.prod);
show('R21 class',C.r21,EXP.r21);
show('R28 generator cpp',C.r28cpp,EXP.r28cpp);
show('AF7R36 core hpp',C.hpp,EXP.hpp);
show('packet MEX source',C.packetSource,EXP.packetSource);
show('AF7R58I exact batch',C.batch,EXP.batch);
show('canonical handles',C.handles,EXP.handles);
show('spacing guard',C.spacing,EXP.spacing);
assert(isfile(C.packetMex),'[R31 INSTALL] packet MEX missing.');
rev=af7r79_packet_retarget_mex(2);
assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1'), ...
    '[R31 INSTALL] packet MEX revision mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R31_INSTALL_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.mex_modified=false;
OUT.status='INIT';

fprintf('\n[C. SHADOW PARSE + SOURCE/THEORY SELFTEST -- NO LIVE WRITE]\n');
addpath(payload,'-begin'); rehash;
cleanup=onCleanup(@()cleanup_payload(payload)); %#ok<NASGU>
assert(paths_equal(which('REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE'),P.class), ...
    '[R31 INSTALL] shadow R31 class binding mismatch.');
mc=meta.class.fromName('REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE');
assert(~isempty(mc),'[R31 INSTALL] shadow R31 class parse failed.');
T=AF7R58ZZY_R31_SELFTEST(P.class);
assert(T.closed,'[R31 INSTALL] shadow selftest did not close.');
OUT.shadow_selftest=T;
clear cleanup;

fprintf('\n[D. TRANSACTIONAL LIVE INSTALL]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,['AF7R58ZZY_R31_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
backup_one(C.runner,backup,'run_flight_headless.m');
backup_optional(C.r31class,backup,'REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE.m');
backup_optional(C.selftest,backup,'AF7R58ZZY_R31_SELFTEST.m');
backup_optional(C.launcher,backup,'AF7R58ZZY_R31_RUN_CANONICAL.m');
backup_optional(C.report,backup,'AF7R58ZZY_R31_REPORT.m');
OUT.backup=backup;

try
    copybin(P.class,C.r31class);
    copybin(P.runner,C.runner);
    copybin(P.selftest,C.selftest);
    copybin(P.launcher,C.launcher);
    copybin(P.report,C.report);

    assert(strcmp(sha_file(C.r31class),EXP.r31class));
    assert(strcmp(sha_file(C.runner),EXP.r31runner));
    assert(strcmp(sha_file(C.selftest),EXP.selftest));
    assert(strcmp(sha_file(C.launcher),EXP.launcher));
    assert(strcmp(sha_file(C.report),EXP.report));

    rehash; addpath(ref,'-begin'); addpath(root,'-begin');

    T2=AF7R58ZZY_R31_SELFTEST(C.r31class);
    assert(T2.closed,'[R31 INSTALL] live selftest did not close.');
    OUT.live_selftest=T2;

    mc2=meta.class.fromName('REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE');
    assert(~isempty(mc2),'[R31 INSTALL] live class parse failed.');
    Rtxt=fileread(C.runner);
    assert(count_occ(Rtxt,'REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE(agent,nominal_ref,replan_opts)')==1, ...
        '[R31 INSTALL] runner binding missing/nonunique.');
    assert(~contains(Rtxt,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT(agent,nominal_ref,replan_opts)'), ...
        '[R31 INSTALL] stale R30 runner binding remains.');

    % Protected authorities unchanged.
    assert(strcmp(sha_file(C.prod),EXP.prod),'[R31 INSTALL] production changed.');
    assert(strcmp(sha_file(C.r21),EXP.r21),'[R31 INSTALL] R21 changed.');
    assert(strcmp(sha_file(C.r28cpp),EXP.r28cpp),'[R31 INSTALL] generator cpp changed.');
    assert(strcmp(sha_file(C.hpp),EXP.hpp),'[R31 INSTALL] core hpp changed.');
    assert(strcmp(sha_file(C.packetSource),EXP.packetSource),'[R31 INSTALL] packet MEX source changed.');
    assert(strcmp(sha_file(C.batch),EXP.batch),'[R31 INSTALL] batch theorem helper changed.');
    assert(strcmp(sha_file(C.handles),EXP.handles),'[R31 INSTALL] canonical handle helper changed.');
    assert(strcmp(sha_file(C.spacing),EXP.spacing),'[R31 INSTALL] spacing guard changed.');

    OUT.closed=true;
    OUT.status='R31_INSTALLED__RUN_CANONICAL_NEXT';
catch ME
    fprintf('\n[R31 INSTALL FAILURE] %s\n',ME.message);
    try
        copybin(fullfile(backup,'run_flight_headless.m'),C.runner);
        restore_optional(backup,'REPLANNING_SYSTEM_R31_DIRECT_STAGE_STORE.m',C.r31class);
        restore_optional(backup,'AF7R58ZZY_R31_SELFTEST.m',C.selftest);
        restore_optional(backup,'AF7R58ZZY_R31_RUN_CANONICAL.m',C.launcher);
        restore_optional(backup,'AF7R58ZZY_R31_REPORT.m',C.report);
        rehash; addpath(ref,'-begin'); addpath(root,'-begin');
        assert(strcmp(sha_file(C.runner),EXP.r30runner),'[R31 INSTALL] rollback runner mismatch.');
        assert(strcmp(sha_file(C.prod),EXP.prod),'[R31 INSTALL] rollback production mismatch.');
        fprintf(' rollback to exact R30 checkpoint          = PASS\n');
    catch MER
        fprintf(' rollback                                = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R31_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R31_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n');
fprintf(' R31 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf(' status                                  = %s\n',OUT.status);
fprintf(' production source modified              = 0\n');
fprintf(' controller source modified              = 0\n');
fprintf(' MEX source/binary modified              = 0\n');
fprintf(' continuous certificate math modified    = 0\n');
fprintf(' measurement schedule changed            = 0\n');
fprintf(' NEXT: AF7R58ZZY_R31_RUN_CANONICAL\n');
fprintf(' Then: AF7R58ZZY_R31_REPORT\n');
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
    cc=fullfile(r,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT.m');
    pp=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    if isfile(rr)&&isfile(cc)&&isfile(pp)&& ...
            strcmp(sha_file(rr),EXP.r30runner)&& ...
            strcmp(sha_file(cc),EXP.r30class)&& ...
            strcmp(sha_file(pp),EXP.prod)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good),'[R31 INSTALL] exact flown R30 root not found.');
score=zeros(size(good));
for k=1:numel(good)
    [~,n]=fileparts(good{k});
    score(k)=100*strcmpi(n,'common_matlab')+20*isfolder(fullfile(good{k},'.git'));
end
[~,ix]=max(score); root=good{ix};
end

function p=resolve_exact_source(name,expectedSHA,root,ref)
candidates={};
w=which(name,'-all');
if ischar(w)
    if ~isempty(w), candidates={w}; end
elseif isstring(w)
    candidates=cellstr(w(:));
elseif iscell(w)
    candidates=w(:).';
end
candidates=[candidates,{fullfile(root,[name '.m']),fullfile(ref,[name '.m'])}];
good={};
for k=1:numel(candidates)
    q=candidates{k};
    if isempty(q)||~isfile(q), continue; end
    try
        if strcmp(sha_file(q),expectedSHA), good{end+1}=q; end %#ok<AGROW>
    catch
    end
end
assert(~isempty(good),'[R31 INSTALL] exact protected helper not found: %s',name);
active=which(name);
assert(~isempty(active)&&isfile(active),'[R31 INSTALL] active helper unresolved: %s',name);
assert(strcmp(sha_file(active),expectedSHA),'[R31 INSTALL] active helper SHA mismatch: %s',name);
p=active;
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
assert(isfile(p),'[R31 INSTALL] missing %s: %s',n,p);
q=sha_file(p); fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));
assert(strcmp(q,h),'[R31 INSTALL] %s SHA mismatch.',n);
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
