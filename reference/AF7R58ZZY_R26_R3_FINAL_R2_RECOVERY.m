function OUT = AF7R58ZZY_R26_R3_FINAL_R2_RECOVERY()
% AF7R58ZZY-R26-R3 FINAL R2 RECOVERY
% =========================================================================
% Fixes the previous recovery bug caused by `clear classes` inside the
% recovery function, which cleared local function variables (including root)
% even though MATLAB could not clear STATE_CLASS.
%
% R2 deliberately performs NO `clear classes` and NO runtime runner synthesis.
% It installs already-reviewed exact bytes, purges path shadows, audits active
% bindings, and leaves class/object clearing to the canonical runner's own
% initial `clearvars` in BASE workspace.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | FINAL R2 SOURCE / PATH RECOVERY\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] prebuiltExactRunner=1 | noClearClasses=1 | binaryCopy=1 | shadowPurge=1 | ' ...
    'productionModified=0 | controllerModified=0 | safetyMathChanged=0\n']);

EXP=struct();
EXP.production='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.runner='cfa9f9d61b2762fdb899515b2a6d6ddf582d550c1eaf4046574e360b9d28fbe5';
EXP.r26='2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6';
EXP.r25='f2f13902bcf1589bacde9092527f3d0c8cb68f9d330d819b5282ad7b002f616c';
EXP.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
EXP.r19='928677701a06d2c8aabf96982276fb4ada5daf9c67c8ec1354182e823c85e482';
EXP.compile23='e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177';
EXP.merge25='f7b5f4c46710c0bc4ee64c3a7e839717a14bc9821291bb8e15bddefeef976a36';
EXP.report='e469bcb623f31867f5595dc7830fc065721c4bebf7854bc7233809ebcae90097';
EXP.selftest='d8f79887ad8a76280724146ee807363a2ad20f64e6ddc42e92451eb247353959';
EXP.launcher='b429678f00b906654e1bd51d759c0318c9644719a098174d9bf81e8cc6a12fe0';

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R3_FINAL_R2_RECOVERY_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.safety_math_changed=false;

pkg=fileparts(mfilename('fullpath'));
P=struct();
P.runner=fullfile(pkg,'run_flight_headless_R26_R3_FINAL_SOURCE.m');
P.r26=fullfile(pkg,'REPLANNING_SYSTEM_R26_BALANCED.m');
P.report=fullfile(pkg,'AF7R58ZZY_R26_R3_REPORT.m');
P.selftest=fullfile(pkg,'AF7R58ZZY_R26_R3_SELFTEST.m');
P.launcher=fullfile(pkg,'AF7R58ZZY_R26_R3_RUN_CANONICAL.m');

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
check_file_sha('runner payload',P.runner,EXP.runner);
check_file_sha('R26 class payload',P.r26,EXP.r26);
check_file_sha('report payload',P.report,EXP.report);
check_file_sha('selftest payload',P.selftest,EXP.selftest);
check_file_sha('launcher payload',P.launcher,EXP.launcher);
[crlf,crcrlf,lf]=eol_counts(read_bytes(P.runner));
fprintf(' runner CRLF / CRCRLF / LF              = %d / %d / %d\n',crlf,crcrlf,lf);
assert(crcrlf==0&&crlf==lf&&lf>0,'[R26-R3 FINAL R2] runner EOL contract failed.');

fprintf('\n[B. CANONICAL ROOT DISCOVERY -- RUNNER WHICH IS NOT TRUSTED]\n');
root=find_canonical_root(EXP);
OUT.root=root;
fprintf(' canonical root                          = %s\n',root);

C=struct();
C.runner=fullfile(root,'run_flight_headless.m');
C.production=fullfile(root,'reference','REPLANNING_SYSTEM.m');
C.r26=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
C.r25=fullfile(root,'REPLANNING_SYSTEM_R25_LEAN.m');
C.r21=fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m');
C.r19=fullfile(root,'REPLANNING_SYSTEM_R19_FAST.m');
C.compile23=fullfile(root,'af7r76_compile_correction_packet_R23.m');
C.merge25=fullfile(root,'af7r77_merge_result_reference_fast_R25.m');
C.report=fullfile(root,'AF7R58ZZY_R26_R3_REPORT.m');
C.selftest=fullfile(root,'AF7R58ZZY_R26_R3_SELFTEST.m');
C.launcher=fullfile(root,'AF7R58ZZY_R26_R3_RUN_CANONICAL.m');

check_file_sha('production',C.production,EXP.production);
check_file_sha('R25 class',C.r25,EXP.r25);
check_file_sha('R21 class',C.r21,EXP.r21);
check_file_sha('R19 class',C.r19,EXP.r19);
check_file_sha('R23 compile helper',C.compile23,EXP.compile23);
check_file_sha('R25 fast merge',C.merge25,EXP.merge25);

fprintf('\n[C. BACKUP CURRENT CANONICAL TARGETS]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,['AF7R58ZZY_R26_R3_FINAL_R2_BACKUP_' stamp]);
[ok,msg]=mkdir(backup); assert(ok,'%s',msg);
OUT.backup_directory=backup;
for nm={'run_flight_headless.m','REPLANNING_SYSTEM_R26_BALANCED.m', ...
        'AF7R58ZZY_R26_R3_REPORT.m','AF7R58ZZY_R26_R3_SELFTEST.m', ...
        'AF7R58ZZY_R26_R3_RUN_CANONICAL.m'}
    q=fullfile(root,nm{1});
    if isfile(q), copyfile(q,fullfile(backup,nm{1})); end
end
fprintf(' backup                                  = %s\n',backup);

fprintf('\n[D. INSTALL EXACT PREBUILT BYTES]\n');
copy_binary(P.runner,C.runner);
copy_binary(P.r26,C.r26);
copy_binary(P.report,C.report);
copy_binary(P.selftest,C.selftest);
copy_binary(P.launcher,C.launcher);
check_file_sha('canonical runner',C.runner,EXP.runner);
check_file_sha('canonical R26',C.r26,EXP.r26);
check_file_sha('canonical report',C.report,EXP.report);
check_file_sha('canonical selftest',C.selftest,EXP.selftest);
check_file_sha('canonical launcher',C.launcher,EXP.launcher);

fprintf('\n[E. PURGE SHADOW PATHS AND SET CANONICAL CURRENT FOLDER]\n');
% Current-folder precedence is stronger than path precedence, so move first.
cd(root);
entries=strsplit(path,pathsep);
removed={};
rootN=normpath(root);
for k=1:numel(entries)
    e=entries{k};
    if isempty(e)||~isfolder(e), continue; end
    en=normpath(e);
    if ~(strcmp(en,rootN)||startsWith(en,[rootN filesep])), continue; end
    if strcmp(en,rootN)||paths_equal(e,fullfile(root,'reference')), continue; end

    rel=en(numel(rootN)+1:end);
    if startsWith(rel,filesep), rel=rel(2:end); end
    comps=regexp(rel,'[\\/]+','split');
    bad=false;
    for j=1:numel(comps)
        z=lower(comps{j});
        if (startsWith(z,'tp')&&numel(z)>10) || contains(z,'backup_')
            bad=true;
        end
    end
    if isfile(fullfile(e,'run_flight_headless.m')) || ...
       isfile(fullfile(e,'REPLANNING_SYSTEM_R26_BALANCED.m')) || ...
       isfile(fullfile(e,'REPLANNING_SYSTEM_R25_LEAN.m')) || ...
       isfile(fullfile(e,'REPLANNING_SYSTEM_R21_BALANCED.m')) || ...
       isfile(fullfile(e,'REPLANNING_SYSTEM_R19_FAST.m'))
        bad=true;
    end
    if bad
        try
            rmpath(e);
            removed{end+1,1}=e; %#ok<AGROW>
            fprintf(' removed path                            = %s\n',e);
        catch ME
            fprintf(' remove path warning                     = %s | %s\n',e,ME.message);
        end
    end
end
OUT.removed_paths=removed;
addpath(fullfile(root,'reference'),'-begin');
addpath(root,'-begin');
cd(root);
rehash;

fprintf('\n[F. ACTIVE BINDING AUDIT]\n');
check_binding('run_flight_headless',C.runner,EXP.runner);
check_binding('REPLANNING_SYSTEM_R26_BALANCED',C.r26,EXP.r26);
check_binding('REPLANNING_SYSTEM_R25_LEAN',C.r25,EXP.r25);
check_binding('REPLANNING_SYSTEM_R21_BALANCED',C.r21,EXP.r21);
check_binding('REPLANNING_SYSTEM_R19_FAST',C.r19,EXP.r19);
check_binding('af7r76_compile_correction_packet_R23',C.compile23,EXP.compile23);
check_binding('af7r77_merge_result_reference_fast_R25',C.merge25,EXP.merge25);
check_binding('REPLANNING_SYSTEM',C.production,EXP.production);
check_binding('AF7R58ZZY_R26_R3_REPORT',C.report,EXP.report);
check_binding('AF7R58ZZY_R26_R3_SELFTEST',C.selftest,EXP.selftest);
check_binding('AF7R58ZZY_R26_R3_RUN_CANONICAL',C.launcher,EXP.launcher);

fprintf('\n[G. SOURCE SEMANTICS / PARSE]\n');
R=fileread(C.runner);
runnerChecks=[ ...
    count_occ(R,'REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)')==1, ...
    count_occ(R,'replan_opts.ENABLE_PROFILING = false;')==1, ...
    count_occ(R,'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;')==1, ...
    count_occ(R,'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 2.0;')==1, ...
    count_occ(R,'af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)')==1, ...
    count_occ(R,'r22_rt_observer_ms=NaN;')==1 ...
];
fprintf(' runner semantic checks                  = %s\n',mat2str(double(runnerChecks)));
assert(all(runnerChecks),'[R26-R3 FINAL R2] runner semantic audit failed.');
assert(~contains(R,'REPLANNING_SYSTEM_R25_LEAN(agent,nominal_ref,replan_opts)'), ...
    '[R26-R3 FINAL R2] stale R25 constructor remains.');

K=fileread(C.r26);
assert(contains(K,'AF7R58ZZY_R26_BALANCED_R3'),'[R26-R3 FINAL R2] R26 build id missing.');
assert(contains(K,'ta=t_now+3*dt;'),'[R26-R3 FINAL R2] 3-dt activation missing.');
assert(contains(K,'[ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);'), ...
    '[R26-R3 FINAL R2] normal-dispatch bridge missing.');
assert(contains(K,'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED'), ...
    '[R26-R3 FINAL R2] legal same-name superclass bridge missing.');
adv=extract_between(K,'function advance_staged_replan_pipeline', ...
    'function [ok,reason] = activate_staged_reference_only_bundle');
assert(~contains(adv,'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED'), ...
    '[R26-R3 FINAL R2] illegal direct superclass dispatch remains in advance().');
assert(contains(K,'CP=af7r76_compile_correction_packet_R23('), ...
    '[R26-R3 FINAL R2] exact R23 final compile missing.');

% checkcode is static and does not clear class state.
A=checkcode(C.runner,'-id');
B=checkcode(C.r26,'-id');
D=checkcode(C.report,'-id');
E=checkcode(C.launcher,'-id');
fprintf(' checkcode runner/class/report/launcher  = %d / %d / %d / %d entries\n', ...
    numel(A),numel(B),numel(D),numel(E));

fprintf('\n[H. STRUCTURAL SELFTEST -- NO CLEAR CLASSES]\n');
ST=AF7R58ZZY_R26_R3_SELFTEST();
assert(isstruct(ST)&&isfield(ST,'closed')&&ST.closed, ...
    '[R26-R3 FINAL R2] structural selftest failed.');
OUT.selftest=ST;

fprintf('\n[I. MEX PRESENCE -- ONLY REQUIRED EXECUTION HELPERS]\n');
for nm={'af7r76_compiled_eval_mex','af7r36_single_core_mex','af7r75_correction_posaccel_mex'}
    ex=exist(nm{1},'file');
    fprintf(' %-40s exist=%d | %s\n',nm{1},ex,which(nm{1}));
    assert(ex==3,'[R26-R3 FINAL R2] required MEX missing: %s',nm{1});
end
assert(exist('af7r75_posaccel_batch_R19','file')==2, ...
    '[R26-R3 FINAL R2] af7r75_posaccel_batch_R19 missing.');

fprintf('\n[J. IMMUTABILITY RECHECK]\n');
assert(strcmp(sha_file(C.production),EXP.production), ...
    '[R26-R3 FINAL R2] production changed unexpectedly.');
fprintf(' production SHA                          = %s\n',sha_file(C.production));
fprintf(' production replanner modified           = 0\n');
fprintf(' controller source modified             = 0\n');
fprintf(' clear classes executed                  = 0\n');

OUT.closed=true;
OUT.status='R26_R3_FINAL_R2_CLOSED__RUN_CANONICAL_NEXT';
assignin('base','AF7R58ZZY_R26_R3_FINAL_R2_RECOVERY_OUT',OUT);

fprintf('\n====================================================================================================\n');
fprintf(' R26-R3 FINAL R2 RECOVERY COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('canonical root                          = %s\n',root);
fprintf('pwd                                     = %s\n',pwd);
fprintf('active runner                           = %s\n',which('run_flight_headless'));
fprintf('active runner SHA                       = %s\n',sha_file(which('run_flight_headless')));
fprintf('expected runner SHA                     = %s\n',EXP.runner);
fprintf('production replanner modified           = 0\n');
fprintf('controller modified                     = 0\n');
fprintf('clear classes                            = NOT USED\n');
fprintf('SELFTEST CLOSED                         = 1\n');
fprintf('\nNEXT:\n');
fprintf('  AF7R58ZZY_R26_R3_RUN_CANONICAL\n');
fprintf('Then:\n');
fprintf('  AF7R58ZZY_R26_R3_REPORT\n');
fprintf('====================================================================================================\n');
end

function root=find_canonical_root(EXP)
starts={pwd,fileparts(mfilename('fullpath'))};
pe=strsplit(path,pathsep);
starts=[starts,pe];
cand={};
for k=1:numel(starts)
    p=starts{k};
    if isempty(p)||~isfolder(p), continue; end
    p=canonical_path(p);
    for j=1:7
        cand{end+1,1}=p; %#ok<AGROW>
        q=fileparts(p);
        if isempty(q)||strcmp(q,p), break; end
        p=q;
    end
end
cand=unique(cand,'stable');
good={};
for k=1:numel(cand)
    r=cand{k};
    pP=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    p21=fullfile(r,'REPLANNING_SYSTEM_R21_BALANCED.m');
    p19=fullfile(r,'REPLANNING_SYSTEM_R19_FAST.m');
    pC=fullfile(r,'af7r76_compile_correction_packet_R23.m');
    pM=fullfile(r,'af7r77_merge_result_reference_fast_R25.m');
    try
        ok=isfile(pP)&&strcmp(sha_file(pP),EXP.production)&& ...
           isfile(p21)&&strcmp(sha_file(p21),EXP.r21)&& ...
           isfile(p19)&&strcmp(sha_file(p19),EXP.r19)&& ...
           isfile(pC)&&strcmp(sha_file(pC),EXP.compile23)&& ...
           isfile(pM)&&strcmp(sha_file(pM),EXP.merge25);
    catch
        ok=false;
    end
    if ok, good{end+1,1}=r; end %#ok<AGROW>
end
good=unique(good,'stable');
assert(~isempty(good),'[R26-R3 FINAL R2] canonical common_matlab root not found.');
score=zeros(numel(good),1);
for k=1:numel(good)
    [~,nm]=fileparts(good{k});
    if strcmpi(nm,'common_matlab'), score(k)=score(k)+100; end
    if isfolder(fullfile(good{k},'.git')), score(k)=score(k)+20; end
    if isfile(fullfile(good{k},'run_flight_headless.m')), score(k)=score(k)+5; end
end
[~,ix]=max(score);
root=good{ix};
end

function check_binding(name,expectedPath,expectedSHA)
got=which(name);
assert(~isempty(got)&&isfile(got),'[R26-R3 FINAL R2] unresolved: %s',name);
h=sha_file(got);
pOK=paths_equal(got,expectedPath);
sOK=strcmp(h,expectedSHA);
fprintf(' %-40s pathOK=%d shaOK=%d\n',name,pOK,sOK);
fprintf('   %s\n',got);
assert(pOK&&sOK,'[R26-R3 FINAL R2] active binding mismatch: %s',name);
end

function check_file_sha(label,p,expected)
assert(isfile(p),'[R26-R3 FINAL R2] missing %s: %s',label,p);
h=sha_file(p);
fprintf(' %-40s SHA=%s | expected=%d\n',label,h,strcmp(h,expected));
assert(strcmp(h,expected),'[R26-R3 FINAL R2] SHA mismatch: %s',label);
end

function n=count_occ(s,p)
n=numel(strfind(s,p));
end

function s=extract_between(txt,a,b)
ia=strfind(txt,a); ib=strfind(txt,b);
if isempty(ia)||isempty(ib), s=''; return; end
i=ia(1); jj=find(ib>i,1,'first');
if isempty(jj), s=''; else, s=txt(i:ib(jj)-1); end
end

function tf=paths_equal(a,b)
if isempty(a)||isempty(b), tf=false; return; end
aa=canonical_path(a); bb=canonical_path(b);
if ispc, tf=strcmpi(aa,bb); else, tf=strcmp(aa,bb); end
end

function p=normpath(p)
p=canonical_path(p);
if ispc, p=lower(p); end
end

function p=canonical_path(p)
try
    p=char(java.io.File(p).getCanonicalPath());
catch
    p=char(p);
end
end

function copy_binary(src,dst)
write_bytes(dst,read_bytes(src));
end

function b=read_bytes(p)
f=fopen(p,'rb'); assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end

function write_bytes(p,b)
f=fopen(p,'wb'); assert(f>=0,'cannot write %s',p);
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

function h=sha_file(p)
h=sha_bytes(read_bytes(p));
end

function h=sha_bytes(b)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(b(:)),'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
