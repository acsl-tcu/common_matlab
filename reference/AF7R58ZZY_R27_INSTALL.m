function OUT = AF7R58ZZY_R27_INSTALL()
% AF7R58ZZY-R27 transactional exact work-elimination install.
% -------------------------------------------------------------------------
% R27 deliberately keeps the R26 4-cycle schedule unchanged.  It removes
% duplicate/non-authority work only:
%   - full final correction recompilation -> exact incremental append
%   - atomic-path 161-point postcommit cache -> demand-built normal cache
%
% Production REPLANNING_SYSTEM.m, controller, estimator and plant are not
% edited.  No continuous-time safety/certificate mathematics are changed.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R27 | TRANSACTIONAL EXACT WORK-ELIMINATION INSTALL\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] scheduleMoved=0 | workElimination=1 | incrementalExactCompile=1 | ' ...
    'lazyNonAuthorityCache=1 | productionModified=0 | controllerModified=0 | safetyMathChanged=0\n']);

EXP=struct();
EXP.production='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r26='2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6';
EXP.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
EXP.r19='928677701a06d2c8aabf96982276fb4ada5daf9c67c8ec1354182e823c85e482';
EXP.r26runner='cfa9f9d61b2762fdb899515b2a6d6ddf582d550c1eaf4046574e360b9d28fbe5';
EXP.r27runner='5f0e4ad0230d4e5cebcbf9de2ad1525d454a6fe178a9694c9323eebed5aa048c';
EXP.r27='be3bd20dd0c1acc71d1968727124918b9300b6542d97b197510e4b89442f06da';
EXP.helper='2a008e5ac8abf248416ffe0ecef9ab0ebb71dac28901aa6c9467b9a823772533';
EXP.selftest='b83ba68d6a4d5f5e3bdc3b2de5feae952cfaf30387d417498ade25c0feb9b32f';
EXP.report='3125a1f2f068c7b592af45fda5445870fca50c69e75c9347655141019d6585b0';
EXP.launch='02380d42dbb56313ede36e54aab686c9afbc71db97f4c0b3e2563ee91e77fcb0';
EXP.compile23='e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177';
EXP.compile25='8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44';
EXP.merge25='f7b5f4c46710c0bc4ee64c3a7e839717a14bc9821291bb8e15bddefeef976a36';

pkg=fileparts(mfilename('fullpath'));
payload=struct();
payload.runner=fullfile(pkg,'run_flight_headless_R27_FINAL_SOURCE.m');
payload.r27=fullfile(pkg,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
payload.helper=fullfile(pkg,'af7r78_incremental_compile_R27.m');
payload.selftest=fullfile(pkg,'AF7R58ZZY_R27_SELFTEST.m');
payload.report=fullfile(pkg,'AF7R58ZZY_R27_REPORT.m');
payload.launch=fullfile(pkg,'AF7R58ZZY_R27_RUN_CANONICAL.m');

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
check_file(payload.runner,EXP.r27runner,'runner payload');
check_file(payload.r27,EXP.r27,'R27 class payload');
check_file(payload.helper,EXP.helper,'incremental helper payload');
check_file(payload.selftest,EXP.selftest,'selftest payload');
check_file(payload.report,EXP.report,'report payload');
check_file(payload.launch,EXP.launch,'launcher payload');
[bcrlf,bcrcrlf,blf]=eol_counts(read_bytes(payload.runner));
fprintf(' runner CRLF / CRCRLF / LF              = %d / %d / %d\n',bcrlf,bcrcrlf,blf);
assert(bcrcrlf==0&&bcrlf==blf,'[R27 INSTALL] runner line-ending contract failed.');

fprintf('\n[B. CANONICAL ROOT DISCOVERY]\n');
root=find_root(EXP);
fprintf(' canonical root                          = %s\n',root);
C=struct();
C.runner=fullfile(root,'run_flight_headless.m');
C.production=fullfile(root,'reference','REPLANNING_SYSTEM.m');
C.r26=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
C.r27=fullfile(root,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');
C.r21=fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m');
C.r19=fullfile(root,'REPLANNING_SYSTEM_R19_FAST.m');
C.helper=fullfile(root,'af7r78_incremental_compile_R27.m');
C.selftest=fullfile(root,'AF7R58ZZY_R27_SELFTEST.m');
C.report=fullfile(root,'AF7R58ZZY_R27_REPORT.m');
C.launch=fullfile(root,'AF7R58ZZY_R27_RUN_CANONICAL.m');
C.compile23=fullfile(root,'af7r76_compile_correction_packet_R23.m');
C.compile25=fullfile(root,'af7r77_compile_correction_packet_R25.m');
C.merge25=fullfile(root,'af7r77_merge_result_reference_fast_R25.m');

check_file(C.production,EXP.production,'production REPLANNING_SYSTEM');
check_file(C.r26,EXP.r26,'R26 input class');
check_file(C.r21,EXP.r21,'R21 class');
check_file(C.r19,EXP.r19,'R19 class');
check_file(C.compile23,EXP.compile23,'R23 exact full compiler');
check_file(C.compile25,EXP.compile25,'R25 preflight compiler');
check_file(C.merge25,EXP.merge25,'R25 fast merge');
assert(isfile(C.runner),'[R27 INSTALL] canonical runner missing.');
hRunner=sha_file(C.runner);
fprintf(' current runner SHA                     = %s\n',hRunner);
assert(strcmp(hRunner,EXP.r26runner)||strcmp(hRunner,EXP.r27runner), ...
    '[R27 INSTALL] runner is neither exact R26 input nor exact R27 output.');

fprintf('\n[C. REQUIRED RUNTIME MEX AUTHORITY]\n');
reqMex={'af7r36_single_core_mex','af7r76_compiled_eval_mex','af7r77_compile_dcp_mex'};
for k=1:numel(reqMex)
    ex=exist(reqMex{k},'file');
    fprintf(' %-40s exist=%d | %s\n',reqMex{k},ex,which(reqMex{k}));
    assert(ex==3,'[R27 INSTALL] required MEX missing: %s',reqMex{k});
end

fprintf('\n[D. BACKUP CURRENT CANONICAL FILES]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,['AF7R58ZZY_R27_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
backup_one(C.runner,backup);
backup_one(C.r27,backup); backup_one(C.helper,backup); backup_one(C.selftest,backup);
backup_one(C.report,backup); backup_one(C.launch,backup);
fprintf(' backup                                  = %s\n',backup);

OUT=struct(); OUT.schema='AF7R58ZZY_R27_INSTALL_V1'; OUT.closed=false;
OUT.root=root; OUT.backup_directory=backup; OUT.production_modified=false;
OUT.controller_modified=false; OUT.runner_modified=false;

try
    fprintf('\n[E. INSTALL R27 SUPPORT FILES -- BINARY COPY]\n');
    copy_binary(payload.r27,C.r27);
    copy_binary(payload.helper,C.helper);
    copy_binary(payload.selftest,C.selftest);
    copy_binary(payload.report,C.report);
    copy_binary(payload.launch,C.launch);
    assert(strcmp(sha_file(C.r27),EXP.r27));
    assert(strcmp(sha_file(C.helper),EXP.helper));
    assert(strcmp(sha_file(C.selftest),EXP.selftest));
    assert(strcmp(sha_file(C.report),EXP.report));
    assert(strcmp(sha_file(C.launch),EXP.launch));

    cd(root);
    purge_shadow_paths(root);
    addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); rehash;

    fprintf('\n[F. EXACT INCREMENTAL-COMPILE SELFTEST]\n');
    ST=AF7R58ZZY_R27_SELFTEST();
    OUT.selftest=ST;
    assert(ST.closed,'[R27 INSTALL] selftest did not close.');

    fprintf('\n[G. R27 CLASS PARSE]\n');
    mc=meta.class.fromName('REPLANNING_SYSTEM_R27_LEAN_EXACT');
    assert(~isempty(mc),'[R27 INSTALL] R27 class parse failed.');
    fprintf(' class                                  = %s\n',mc.Name);
    cc=checkcode(C.r27,'-id');
    fprintf(' R27 checkcode entries                  = %d\n',numel(cc));

    fprintf('\n[H. INSTALL EXACT PREBUILT R27 RUNNER]\n');
    copy_binary(payload.runner,C.runner);
    rehash;
    assert(strcmp(sha_file(C.runner),EXP.r27runner),'[R27 INSTALL] runner copy mismatch.');
    R=fileread(C.runner);
    assert(numel(strfind(R,'REPLANNING_SYSTEM_R27_LEAN_EXACT(agent,nominal_ref,replan_opts)'))==1, ...
        '[R27 INSTALL] R27 constructor binding missing.');
    assert(isempty(strfind(R,'REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)')), ...
        '[R27 INSTALL] stale R26 constructor remains.');
    assert(numel(strfind(R,'replan_opts.ENABLE_PROFILING = false;'))==1&& ...
        numel(strfind(R,'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;'))==1, ...
        '[R27 INSTALL] clean timing flags changed.');
    assert(numel(strfind(R,'af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)'))==1, ...
        '[R27 INSTALL] exact fast merge binding changed.');
    OUT.runner_modified=true;

    fprintf('\n[I. ACTIVE BINDING / IMMUTABILITY]\n');
    addpath(fullfile(root,'reference'),'-begin'); addpath(root,'-begin'); cd(root); rehash;
    check_binding('run_flight_headless',C.runner,EXP.r27runner);
    check_binding('REPLANNING_SYSTEM_R27_LEAN_EXACT',C.r27,EXP.r27);
    check_binding('af7r78_incremental_compile_R27',C.helper,EXP.helper);
    check_binding('REPLANNING_SYSTEM',C.production,EXP.production);
    assert(strcmp(sha_file(C.production),EXP.production),'[R27 INSTALL] production changed.');
    fprintf(' production replanner modified           = 0\n');
    fprintf(' controller source modified             = 0\n');
    fprintf(' schedule moved                         = 0\n');

    OUT.closed=true;
    OUT.status='R27_INSTALLED__RUN_CANONICAL_NEXT';
catch ME
    fprintf('\n[R27 INSTALL FAILURE] %s\n',ME.message);
    % Roll runner back if we reached the runner write.  Support files may remain
    % inactive; the canonical runner is the execution authority.
    try
        rb=fullfile(backup,'run_flight_headless.m');
        if isfile(rb), copyfile(rb,C.runner); rehash; fprintf(' runner rollback                         = PASS\n'); end
    catch MER
        fprintf(' runner rollback                         = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_R27_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_R27_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n');
fprintf(' R27 INSTALL COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('status                                  = %s\n',OUT.status);
fprintf('runner SHA                              = %s\n',sha_file(C.runner));
fprintf('production replanner modified           = 0\n');
fprintf('controller modified                     = 0\n');
fprintf('schedule moved                          = 0\n');
fprintf('SELFTEST CLOSED                         = 1\n');
fprintf('\nNext:\n');
fprintf('  1) AF7R58ZZY_R27_RUN_CANONICAL\n');
fprintf('  2) AF7R58ZZY_R27_REPORT\n');
fprintf('Return AF7R58ZZY_R27_REPORT.txt and AF7R58ZZY_R27_OUT.mat.\n');
fprintf('====================================================================================================\n');
end

function root=find_root(EXP)
starts={pwd,fileparts(mfilename('fullpath'))}; pe=strsplit(path,pathsep); starts=[starts pe];
cand={};
for k=1:numel(starts)
    p=starts{k}; if isempty(p)||~isfolder(p), continue; end
    p=canonical_path(p);
    for j=1:7
        cand{end+1,1}=p; %#ok<AGROW>
        q=fileparts(p); if isempty(q)||strcmp(q,p), break; end; p=q;
    end
end
cand=unique(cand,'stable'); good={};
for k=1:numel(cand)
    r=cand{k}; pp=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    p21=fullfile(r,'REPLANNING_SYSTEM_R21_BALANCED.m'); p19=fullfile(r,'REPLANNING_SYSTEM_R19_FAST.m');
    try
        ok=isfile(pp)&&strcmp(sha_file(pp),EXP.production)&& ...
           isfile(p21)&&strcmp(sha_file(p21),EXP.r21)&& ...
           isfile(p19)&&strcmp(sha_file(p19),EXP.r19);
    catch, ok=false; end
    if ok, good{end+1,1}=r; end %#ok<AGROW>
end
assert(~isempty(good),'[R27 INSTALL] canonical common_matlab root not found.');
score=zeros(numel(good),1);
for k=1:numel(good)
    [~,nm]=fileparts(good{k});
    if strcmpi(nm,'common_matlab'), score(k)=score(k)+100; end
    if isfolder(fullfile(good{k},'.git')), score(k)=score(k)+20; end
end
[~,ix]=max(score); root=good{ix};
end

function purge_shadow_paths(root)
entries=strsplit(path,pathsep); rootN=normpath(root);
for k=1:numel(entries)
    e=entries{k}; if isempty(e)||~isfolder(e), continue; end
    en=normpath(e);
    if ~(strcmp(en,rootN)||startsWith(en,[rootN filesep]))||strcmp(en,rootN)|| ...
            paths_equal(e,fullfile(root,'reference')), continue; end
    rel=en(numel(rootN)+1:end); if startsWith(rel,filesep), rel=rel(2:end); end
    comps=regexp(rel,'[\\/]+','split'); bad=false;
    for j=1:numel(comps)
        z=lower(comps{j});
        if (startsWith(z,'tp')&&numel(z)>10)||contains(z,'backup_'), bad=true; end
    end
    if isfile(fullfile(e,'run_flight_headless.m'))|| ...
            isfile(fullfile(e,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m'))|| ...
            isfile(fullfile(e,'REPLANNING_SYSTEM_R26_BALANCED.m'))
        bad=true;
    end
    if bad
        try, rmpath(e); fprintf(' removed shadow path                     = %s\n',e); catch, end
    end
end
end

function check_file(p,h,label)
assert(isfile(p),'[R27 INSTALL] missing %s: %s',label,p);
got=sha_file(p); fprintf(' %-40s SHA=%s | expected=%d\n',label,got,strcmp(got,h));
assert(strcmp(got,h),'[R27 INSTALL] SHA mismatch: %s',label);
end
function check_binding(n,p,h)
g=which(n); assert(~isempty(g)&&paths_equal(g,p),'[R27 INSTALL] path binding mismatch: %s',n);
assert(strcmp(sha_file(g),h),'[R27 INSTALL] SHA binding mismatch: %s',n);
fprintf(' %-40s pathOK=1 shaOK=1\n',n);
end
function backup_one(p,b)
if isfile(p)
    [~,n,e]=fileparts(p);
    copyfile(p,fullfile(b,[n e]));
end
end

function copy_binary(a,b)
write_bytes(b,read_bytes(a));
end

function b=read_bytes(p)
f=fopen(p,'rb');
assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end

function write_bytes(p,b)
f=fopen(p,'wb');
assert(f>=0,'cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
n=fwrite(f,b,'uint8');
assert(n==numel(b),'short write: %s',p);
end

function [a,c,d]=eol_counts(b)
r=b(:).';
a=numel(strfind(r,uint8([13 10])));
c=numel(strfind(r,uint8([13 13 10])));
d=sum(r==10);
end

function p=canonical_path(p)
try
    p=char(java.io.File(p).getCanonicalPath());
catch
    p=char(p);
end
end

function p=normpath(p)
p=canonical_path(p);
if ispc, p=lower(p); end
end

function tf=paths_equal(a,b)
aa=canonical_path(a); bb=canonical_path(b);
if ispc, tf=strcmpi(aa,bb); else, tf=strcmp(aa,bb); end
end

function h=sha_file(p)
h=sha_bytes(read_bytes(p));
end

function h=sha_bytes(b)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(b(:)),'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
