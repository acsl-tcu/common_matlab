function OUT=AF7R58ZZY_R30_R2_INSTALL()
% AF7R58ZZY-R30 transactional install from exact flown R29 checkpoint.
% R30-R2 installer correction:
%   protected AF7R58I helpers are resolved by active binding + exact SHA.
%   The previous installer incorrectly assumed they were in common_matlab root;
%   the installed authority is in the reference tree.
%
% Theory contract:
% - production REPLANNING_SYSTEM.m unchanged
% - controller/estimator/plant unchanged
% - continuous C7 / speed / specific-force / qdot / whole-body geometry
%   certificate mathematics unchanged
% - AF7R58I exact resource theorem restored
% - R29 packet-retarget MEX arithmetic retained bitwise exactly
% - four-cycle measurement schedule unchanged
% - only implementation change: construct final R/B once, with compiled packet
%   attached before sharing the nested reference struct
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R30-R2 | SINGLE-BUILD EXACT STAGE3 INSTALL\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | controllerModified=0 | certificateMathChanged=0 | scheduleChanged=0\n');

EXP=struct();
EXP.prod='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
EXP.r29runner='967f7972d142db1f52c8fadb5d6421bfbb67c8ab98e4309e2800702f64dbd32a';
EXP.r29class='0e5eb4a63a1fffc57776034531bc5b78203b3935cb235f24e9fdd16177c2c184';
EXP.r28cpp='bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a';
EXP.hpp='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
EXP.r25helper='8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44';
EXP.packetSource='794194666d68dead1a884467c82bb3dee484528a0f8a6f62f3cdab0570d07a60';
EXP.batch='92f093168f1df95661c5e7f2e75005fe439406eb3ec22e53fd1f25d6bdfb8a15';
EXP.handles='1c9bf7407b9c2482c5f6d2676479966b808995c4d10ee2871a620faba55a82f8';
EXP.spacing='ba92a9bb05e5cea6916f904646ff416a8379ff5ed9ed4424baf362eacb43054e';

EXP.r30class='ca1b8e2a5dbffecf57a777c023ff5c210b974252d0ff0e71a90709ebdeb70fec';
EXP.r30runner='d7f403cb7c25fab1d1bcc288abf69b5e2e4ae7790b44a6380fbfa58e1d457376';
EXP.selftest='4834d8c584c38af2514437112d6ee6fec7dd343774b9a869b00f16174675b293';
EXP.launcher='d1b9ecd1c2464fed67bf37086f8ffcbc8d78ee750c2d7f18fc04e687325429c2';
EXP.report='cc77dd507c529a2dd39aac4e7d7ed25037fe7688a41525fb2171c4fd21e2e69c';

pkg=fileparts(mfilename('fullpath'));
payload=fullfile(pkg,'payload');
assert(isfolder(payload),'[R30 INSTALL] payload folder missing.');

P=struct( ...
    'class',fullfile(payload,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT.m'), ...
    'runner',fullfile(payload,'run_flight_headless_R30_FINAL_SOURCE.m'), ...
    'packetSource',fullfile(payload,'af7r79_packet_retarget_mex.cpp'), ...
    'selftest',fullfile(payload,'AF7R58ZZY_R30_SELFTEST.m'), ...
    'launcher',fullfile(payload,'AF7R58ZZY_R30_RUN_CANONICAL.m'), ...
    'report',fullfile(payload,'AF7R58ZZY_R30_REPORT.m'));

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
show('R30 class',P.class,EXP.r30class);
show('R30 runner',P.runner,EXP.r30runner);
show('packet MEX source',P.packetSource,EXP.packetSource);
show('R30 selftest',P.selftest,EXP.selftest);
show('R30 launcher',P.launcher,EXP.launcher);
show('R30 report',P.report,EXP.report);

root=find_root(EXP);
ref=fullfile(root,'reference');
C=struct( ...
    'runner',fullfile(root,'run_flight_headless.m'), ...
    'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
    'r21',fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m'), ...
    'r29class',fullfile(root,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT.m'), ...
    'r28cpp',fullfile(ref,'af7r36_single_core_mex.cpp'), ...
    'hpp',fullfile(ref,'af7r36_core.hpp'), ...
    'r25helper',fullfile(root,'af7r77_compile_correction_packet_R25.m'), ...
    'batch',resolve_exact_source('af7r58i_prepare_batch_model',EXP.batch,root,ref), ...
    'handles',resolve_exact_source('af7r58i_make_canonical_eval_handles',EXP.handles,root,ref), ...
    'spacing',resolve_exact_source('af7r58i_commit_spacing_guard',EXP.spacing,root,ref), ...
    'packetSource',fullfile(root,'af7r79_packet_retarget_mex.cpp'), ...
    'packetMex',fullfile(root,['af7r79_packet_retarget_mex.' mexext]), ...
    'r30class',fullfile(root,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT.m'), ...
    'selftest',fullfile(root,'AF7R58ZZY_R30_SELFTEST.m'), ...
    'launcher',fullfile(root,'AF7R58ZZY_R30_RUN_CANONICAL.m'), ...
    'report',fullfile(root,'AF7R58ZZY_R30_REPORT.m'));

cd(root);
addpath(ref,'-begin');
addpath(root,'-begin');
rehash;

fprintf('\n[B. EXACT FLOWN R29 CHECKPOINT]\n');
show('R29 runner',C.runner,EXP.r29runner);
show('R29 class',C.r29class,EXP.r29class);
show('production',C.prod,EXP.prod);
show('R21 class',C.r21,EXP.r21);
show('R28 generator cpp',C.r28cpp,EXP.r28cpp);
show('AF7R36 core hpp',C.hpp,EXP.hpp);
show('R25 compile helper',C.r25helper,EXP.r25helper);
show('AF7R58I exact batch',C.batch,EXP.batch);
show('canonical handles',C.handles,EXP.handles);
show('spacing guard',C.spacing,EXP.spacing);
assert(paths_equal(which('af7r58i_prepare_batch_model'),C.batch), ...
    '[R30-R2 INSTALL] active AF7R58I batch helper binding mismatch.');
assert(paths_equal(which('af7r58i_make_canonical_eval_handles'),C.handles), ...
    '[R30-R2 INSTALL] active canonical handle helper binding mismatch.');
assert(paths_equal(which('af7r58i_commit_spacing_guard'),C.spacing), ...
    '[R30-R2 INSTALL] active spacing guard binding mismatch.');
show('packet MEX source',C.packetSource,EXP.packetSource);
assert(isfile(C.packetMex),'[R30 INSTALL] active packet MEX missing.');
rev=af7r79_packet_retarget_mex(2);
assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R29_PACKET_RETARGET_V1'), ...
    '[R30 INSTALL] active packet MEX revision mismatch.');

OUT=struct();
OUT.schema='AF7R58ZZY_R30_R2_INSTALL_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.status='INIT';

fprintf('\n[C. SHADOW BUILD + SOURCE/THEORY SELFTEST -- NO LIVE WRITE]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
builddir=fullfile(root,['AF7R58ZZY_R30_BUILD_' stamp]);
[ok,msg]=mkdir(builddir); assert(ok,'%s',msg);
testName='af7r79_packet_retarget_mex_R30_TEST';
build_mex(P.packetSource,builddir,testName);
addpath(payload,'-begin');
addpath(builddir,'-begin');
rehash;
cleanup=onCleanup(@()cleanup_paths(builddir,payload)); %#ok<NASGU>

assert(paths_equal(which('REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT'),P.class), ...
    '[R30 INSTALL] shadow R30 class binding mismatch.');
mc=meta.class.fromName('REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT');
assert(~isempty(mc),'[R30 INSTALL] shadow R30 class parse failed.');

T=AF7R58ZZY_R30_SELFTEST(testName,P.class);
assert(T.closed,'[R30 INSTALL] shadow selftest did not close.');
OUT.shadow_selftest=T;
clear cleanup;

fprintf('\n[D. TRANSACTIONAL LIVE INSTALL]\n');
backup=fullfile(root,['AF7R58ZZY_R30_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
backup_one(C.runner,backup,'run_flight_headless.m');
backup_one(C.packetSource,backup,'af7r79_packet_retarget_mex.cpp');
backup_one(C.packetMex,backup,['af7r79_packet_retarget_mex.' mexext]);
backup_optional(C.r30class,backup,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT.m');
backup_optional(C.selftest,backup,'AF7R58ZZY_R30_SELFTEST.m');
backup_optional(C.launcher,backup,'AF7R58ZZY_R30_RUN_CANONICAL.m');
backup_optional(C.report,backup,'AF7R58ZZY_R30_REPORT.m');
OUT.backup=backup;

try
    copybin(P.class,C.r30class);
    copybin(P.runner,C.runner);
    copybin(P.packetSource,C.packetSource);
    copybin(P.selftest,C.selftest);
    copybin(P.launcher,C.launcher);
    copybin(P.report,C.report);

    assert(strcmp(sha_file(C.r30class),EXP.r30class));
    assert(strcmp(sha_file(C.runner),EXP.r30runner));
    assert(strcmp(sha_file(C.packetSource),EXP.packetSource));
    assert(strcmp(sha_file(C.selftest),EXP.selftest));
    assert(strcmp(sha_file(C.launcher),EXP.launcher));
    assert(strcmp(sha_file(C.report),EXP.report));

    try, clear af7r79_packet_retarget_mex; catch, end
    build_mex(C.packetSource,root,'af7r79_packet_retarget_mex');
    rehash; addpath(ref,'-begin'); addpath(root,'-begin');

    assert(paths_equal(which('af7r79_packet_retarget_mex'),C.packetMex), ...
        '[R30 INSTALL] live packet MEX path shadowed.');
    rev2=af7r79_packet_retarget_mex(2);
    assert(ischar(rev2)&&strcmp(rev2,'AF7R58ZZY_R29_PACKET_RETARGET_V1'), ...
        '[R30 INSTALL] live packet MEX revision mismatch.');

    T2=AF7R58ZZY_R30_SELFTEST('af7r79_packet_retarget_mex',C.r30class);
    assert(T2.closed,'[R30 INSTALL] live selftest did not close.');
    OUT.live_selftest=T2;

    mc2=meta.class.fromName('REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT');
    assert(~isempty(mc2),'[R30 INSTALL] live R30 class parse failed.');
    Rtxt=fileread(C.runner);
    assert(count_occ(Rtxt,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT(agent,nominal_ref,replan_opts)')==1, ...
        '[R30 INSTALL] runner binding missing/nonunique.');
    assert(~contains(Rtxt,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT(agent,nominal_ref,replan_opts)'), ...
        '[R30 INSTALL] stale R29 runner binding remains.');

    % Recheck protected theory/runtime authorities after every live write.
    assert(strcmp(sha_file(C.prod),EXP.prod),'[R30 INSTALL] production changed.');
    assert(strcmp(sha_file(C.r21),EXP.r21),'[R30 INSTALL] R21 changed.');
    assert(strcmp(sha_file(C.r28cpp),EXP.r28cpp),'[R30 INSTALL] R28 generator changed.');
    assert(strcmp(sha_file(C.hpp),EXP.hpp),'[R30 INSTALL] AF7R36 core changed.');
    assert(strcmp(sha_file(C.batch),EXP.batch),'[R30 INSTALL] exact batch helper changed.');
    assert(strcmp(sha_file(C.handles),EXP.handles),'[R30 INSTALL] handle helper changed.');
    assert(strcmp(sha_file(C.spacing),EXP.spacing),'[R30 INSTALL] spacing guard changed.');

    OUT.closed=true;
    OUT.status='R30_INSTALLED__RUN_CANONICAL_NEXT';

catch ME
    fprintf('\n[R30 INSTALL FAILURE] %s\n',ME.message);
    try
        try, clear af7r79_packet_retarget_mex; catch, end
        copybin(fullfile(backup,'run_flight_headless.m'),C.runner);
        copybin(fullfile(backup,'af7r79_packet_retarget_mex.cpp'),C.packetSource);
        copybin(fullfile(backup,['af7r79_packet_retarget_mex.' mexext]),C.packetMex);
        restore_optional(backup,'REPLANNING_SYSTEM_R30_SINGLE_BUILD_EXACT.m',C.r30class);
        restore_optional(backup,'AF7R58ZZY_R30_SELFTEST.m',C.selftest);
        restore_optional(backup,'AF7R58ZZY_R30_RUN_CANONICAL.m',C.launcher);
        restore_optional(backup,'AF7R58ZZY_R30_REPORT.m',C.report);
        rehash; addpath(ref,'-begin'); addpath(root,'-begin');

        assert(strcmp(sha_file(C.runner),EXP.r29runner), ...
            '[R30 INSTALL] rollback runner mismatch.');
        assert(strcmp(sha_file(C.packetSource),EXP.packetSource), ...
            '[R30 INSTALL] rollback packet source mismatch.');
        assert(strcmp(sha_file(C.prod),EXP.prod), ...
            '[R30 INSTALL] rollback production mismatch.');
        fprintf(' rollback to exact R29 checkpoint          = PASS\n');
    catch MER
        fprintf(' rollback                                = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R30_R2_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R30_R2_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n');
fprintf(' R30-R2 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf(' status                                  = %s\n',OUT.status);
fprintf(' production source modified              = 0\n');
fprintf(' controller source modified              = 0\n');
fprintf(' continuous certificate math modified    = 0\n');
fprintf(' measurement schedule changed            = 0\n');
fprintf(' NEXT: AF7R58ZZY_R30_RUN_CANONICAL\n');
fprintf(' Then: AF7R58ZZY_R30_REPORT\n');
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
    cc=fullfile(r,'REPLANNING_SYSTEM_R29_STAGE3_LEAN_EXACT.m');
    pp=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    if isfile(rr)&&isfile(cc)&&isfile(pp)&& ...
            strcmp(sha_file(rr),EXP.r29runner)&& ...
            strcmp(sha_file(cc),EXP.r29class)&& ...
            strcmp(sha_file(pp),EXP.prod)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good),'[R30 INSTALL] exact flown R29 root not found.');
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
    '[R30 INSTALL] MEX build output missing.');
end

function cleanup_paths(builddir,payload)
if path_has(builddir), rmpath(builddir); end
if path_has(payload), rmpath(payload); end
rehash;
end

function tf=path_has(p)
e=strsplit(path,pathsep); tf=false;
for k=1:numel(e)
    if paths_equal(e{k},p), tf=true; return; end
end
end

function backup_one(src,dst,name)
assert(isfile(src),'[R30 INSTALL] backup source missing: %s',src);
copyfile(src,fullfile(dst,name),'f');
end

function backup_optional(src,dst,name)
if isfile(src)
    copyfile(src,fullfile(dst,name),'f');
else
    f=fopen(fullfile(dst,[name '.ABSENT']),'wt');
    assert(f>=0); fprintf(f,'ABSENT\n'); fclose(f);
end
end

function restore_optional(dirp,name,dst)
s=fullfile(dirp,name); a=fullfile(dirp,[name '.ABSENT']);
if isfile(s)
    copyfile(s,dst,'f');
elseif isfile(a)&&isfile(dst)
    delete(dst);
end
end

function p=resolve_exact_source(name,expectedSHA,root,ref)
% Resolve an already-installed protected helper by active MATLAB binding and
% exact SHA, without assuming it lives in common_matlab root vs reference.
candidates={};
w=which(name,'-all');
if ischar(w)
    if ~isempty(w), candidates={w}; end
elseif isstring(w)
    candidates=cellstr(w(:));
elseif iscell(w)
    candidates=w(:).';
end
candidates=[candidates, ...
    {fullfile(root,[name '.m']),fullfile(ref,[name '.m'])}];

% Keep only existing, exact-SHA candidates.
good={};
for k=1:numel(candidates)
    q=candidates{k};
    if isempty(q)||~isfile(q), continue; end
    try
        if strcmp(sha_file(q),expectedSHA)
            good{end+1}=q; %#ok<AGROW>
        end
    catch
    end
end
assert(~isempty(good),'[R30-R2 INSTALL] exact protected helper not found: %s',name);

% Canonicalize and deduplicate.
canon=cell(size(good));
for k=1:numel(good)
    try
        canon{k}=char(java.io.File(good{k}).getCanonicalPath());
    catch
        canon{k}=char(good{k});
    end
end
if ispc
    [~,ia]=unique(lower(string(canon)),'stable');
else
    [~,ia]=unique(string(canon),'stable');
end
good=good(sort(ia));

% The active binding must itself be one of the exact copies.  This prevents a
% shadow helper with different bytes from being silently accepted.
active=which(name);
assert(~isempty(active)&&isfile(active), ...
    '[R30-R2 INSTALL] active protected helper unresolved: %s',name);
assert(strcmp(sha_file(active),expectedSHA), ...
    '[R30-R2 INSTALL] active protected helper SHA mismatch: %s',name);

p=active;
end

function show(n,p,h)
assert(isfile(p),'[R30 INSTALL] missing %s: %s',n,p);
q=sha_file(p);
fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));
assert(strcmp(q,h),'[R30 INSTALL] %s SHA mismatch.',n);
end

function n=count_occ(s,p), n=numel(strfind(s,p)); end

function copybin(a,b)
f=fopen(a,'rb'); assert(f>=0,'cannot open %s',a);
c=onCleanup(@()fclose(f)); x=fread(f,Inf,'*uint8'); clear c;
f=fopen(b,'wb'); assert(f>=0,'cannot write %s',b);
c=onCleanup(@()fclose(f)); n=fwrite(f,x,'uint8');
assert(n==numel(x),'short write: %s',b);
end

function tf=paths_equal(a,b)
if isempty(a)||isempty(b),tf=false;return;end
try
    a=char(java.io.File(a).getCanonicalPath());
    b=char(java.io.File(b).getCanonicalPath());
catch
end
if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end

function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
