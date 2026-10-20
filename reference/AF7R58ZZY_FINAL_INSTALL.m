function OUT=AF7R58ZZY_FINAL_INSTALL()
% Flatten the actually-flown R33 path into reference/REPLANNING_SYSTEM.m.
% Transactional: only main production source, runner, and FINAL utility scripts
% are written. Existing exact runtime helpers/MEX authorities are verified, not
% rewritten.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY FINAL | INTEGRATE R33 INTO PRODUCTION REPLANNING_SYSTEM\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] flattenR19R21R33=1 | productionClass=REPLANNING_SYSTEM | controllerModified=0 | certificateMathChanged=0 | scheduleChanged=0\n');

EXP=struct();
EXP.currentProd='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r33class='1e79242e5a03431afd285584b11f2446f972c9f109f1fc0847fd113a7af8e38c';
EXP.r33runner='41eb4a24a131382d22a44c21979545ec00f0a779fd3fed4ac1976c73416b6c8c';
EXP.finalProd='643d35abe47f7c79f5ff796ea476988469a02909ebd053bb71c668527790ea9e';
EXP.finalRunner='f16fb1392f7817a8685e7a852a34d5687a463bc7d41046d7107a49c0e346cf13';
EXP.selftest='ce9a547537c11ea21d34d72d379c984f10a087563236444b292726c181079daa';
EXP.launcher='93c5983f8f47a5ff83a669668109d4013b42ea62ca947f838782e004d596a5b3';
EXP.report='07c4ed383d4a9e2e22ea5b039a916cb5aca9c953de5366ecf10e40b156eeda91';
EXP.archive='d72498348ef5e3ed02d3ddd02e9e339f90b6ea1ff8ced363314f7c05dcd79a37';

EXP.af7r75='69b0996e0a5dd2fa55a580214f9f8edbb517e0ebca7ed614a49814e612560f80';
EXP.af7r76='e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177';
EXP.af7r77='8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44';
EXP.af7r77merge='f7b5f4c46710c0bc4ee64c3a7e839717a14bc9821291bb8e15bddefeef976a36';
EXP.af7r36cpp='bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a';
EXP.af7r36hpp='ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
EXP.af7r79cpp='794194666d68dead1a884467c82bb3dee484528a0f8a6f62f3cdab0570d07a60';

pkg=fileparts(mfilename('fullpath'));
P=struct( ...
    'prod',fullfile(pkg,'payload','REPLANNING_SYSTEM.m'), ...
    'runner',fullfile(pkg,'payload','run_flight_headless_FINAL_SOURCE.m'), ...
    'selftest',fullfile(pkg,'AF7R58ZZY_FINAL_SELFTEST.m'), ...
    'launcher',fullfile(pkg,'AF7R58ZZY_FINAL_RUN_CANONICAL.m'), ...
    'report',fullfile(pkg,'AF7R58ZZY_FINAL_REPORT.m'), ...
    'archive',fullfile(pkg,'AF7R58ZZY_FINAL_ARCHIVE_LEGACY.m'));

fprintf('\n[A. PACKAGE BYTE AUTHORITY]\n');
show('integrated REPLANNING_SYSTEM',P.prod,EXP.finalProd);
show('final runner',P.runner,EXP.finalRunner);
show('final selftest',P.selftest,EXP.selftest);
show('final launcher',P.launcher,EXP.launcher);
show('final report',P.report,EXP.report);
show('legacy archive helper',P.archive,EXP.archive);

root=find_root(EXP);
ref=fullfile(root,'reference');
C=struct( ...
    'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
    'runner',fullfile(root,'run_flight_headless.m'), ...
    'r33class',fullfile(root,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW.m'), ...
    'selftest',fullfile(root,'AF7R58ZZY_FINAL_SELFTEST.m'), ...
    'launcher',fullfile(root,'AF7R58ZZY_FINAL_RUN_CANONICAL.m'), ...
    'report',fullfile(root,'AF7R58ZZY_FINAL_REPORT.m'), ...
    'archive',fullfile(root,'AF7R58ZZY_FINAL_ARCHIVE_LEGACY.m'));

cd(root); addpath(ref,'-begin'); addpath(root,'-begin'); rehash;

fprintf('\n[B. EXACT FLOWN R33 CHECKPOINT]\n');
show('production before integration',C.prod,EXP.currentProd);
show('R33 class',C.r33class,EXP.r33class);
show('R33 runner',C.runner,EXP.r33runner);

% Exact source authorities used by the flattened path.
s75=resolve_exact_source('af7r75_posaccel_batch_R19',EXP.af7r75,root,ref);
s76=resolve_exact_source('af7r76_compile_correction_packet_R23',EXP.af7r76,root,ref);
s77=resolve_exact_source('af7r77_compile_correction_packet_R25',EXP.af7r77,root,ref);
s77m=resolve_exact_source('af7r77_merge_result_reference_fast_R25',EXP.af7r77merge,root,ref);
show('af7r75_posaccel_batch_R19',s75,EXP.af7r75);
show('af7r76 compile helper',s76,EXP.af7r76);
show('af7r77 compile helper',s77,EXP.af7r77);
show('af7r77 merge helper',s77m,EXP.af7r77merge);

cpp36=fullfile(ref,'af7r36_single_core_mex.cpp');
hpp36=fullfile(ref,'af7r36_core.hpp');
cpp79=fullfile(root,'af7r79_packet_retarget_mex.cpp');
show('AF7R36 generator cpp',cpp36,EXP.af7r36cpp);
show('AF7R36 core hpp',hpp36,EXP.af7r36hpp);
show('packet-retarget cpp',cpp79,EXP.af7r79cpp);

reqMex={'af7r70_observer_mex','af7r71_decasteljau_batch_mex', ...
    'af7r75_correction_posaccel_mex','af7r76_compiled_eval_mex', ...
    'af7r77_compile_dcp_mex','af7r36_single_core_mex','af7r79_packet_retarget_mex'};
for i=1:numel(reqMex)
    assert(exist(reqMex{i},'file')==3,'[FINAL INSTALL] required MEX missing: %s',reqMex{i});
end
q=af7r36_single_core_mex(5);
assert(ischar(q)&&strcmp(q,'AF7R58ZZY_R28_NATIVE_ONCE_V1'), ...
    '[FINAL INSTALL] AF7R36 revision mismatch.');
q=af7r79_packet_retarget_mex(2);
assert(ischar(q)&&strcmp(q,'AF7R58ZZY_R29_PACKET_RETARGET_V1'), ...
    '[FINAL INSTALL] packet-retarget revision mismatch.');

fprintf('\n[C. STATIC FLATTENING AUDIT -- NO LIVE WRITE]\n');
static_integrated_audit(P.prod,P.runner);

OUT=struct();
OUT.schema='AF7R58ZZY_FINAL_INSTALL_V1';
OUT.closed=false;
OUT.status='INIT';

fprintf('\n[D. TRANSACTIONAL LIVE INSTALL]\n');
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
backup=fullfile(root,'_af7_final','backups',stamp);
if ~isfolder(backup), mkdir(backup); end
backup_one(C.prod,backup,'REPLANNING_SYSTEM.m');
backup_one(C.runner,backup,'run_flight_headless.m');
backup_optional(C.selftest,backup,'AF7R58ZZY_FINAL_SELFTEST.m');
backup_optional(C.launcher,backup,'AF7R58ZZY_FINAL_RUN_CANONICAL.m');
backup_optional(C.report,backup,'AF7R58ZZY_FINAL_REPORT.m');
backup_optional(C.archive,backup,'AF7R58ZZY_FINAL_ARCHIVE_LEGACY.m');
OUT.backup=backup;

try
    copybin(P.prod,C.prod);
    copybin(P.runner,C.runner);
    copybin(P.selftest,C.selftest);
    copybin(P.launcher,C.launcher);
    copybin(P.report,C.report);
    copybin(P.archive,C.archive);

    show('installed production',C.prod,EXP.finalProd);
    show('installed runner',C.runner,EXP.finalRunner);
    show('installed selftest',C.selftest,EXP.selftest);
    show('installed launcher',C.launcher,EXP.launcher);
    show('installed report',C.report,EXP.report);
    show('installed archive helper',C.archive,EXP.archive);

    % Protected runtime authorities must remain byte-identical.
    show('AF7R36 generator cpp',cpp36,EXP.af7r36cpp);
    show('AF7R36 core hpp',hpp36,EXP.af7r36hpp);
    show('packet-retarget cpp',cpp79,EXP.af7r79cpp);
    show('af7r75 helper',s75,EXP.af7r75);
    show('af7r76 helper',s76,EXP.af7r76);
    show('af7r77 helper',s77,EXP.af7r77);
    show('af7r77 merge helper',s77m,EXP.af7r77merge);

    static_integrated_audit(C.prod,C.runner);
    rehash; addpath(ref,'-begin'); addpath(root,'-begin');

    OUT.closed=true;
    OUT.status='FINAL_INTEGRATED__RUN_CANONICAL_NEXT';
catch ME
    fprintf('\n[FINAL INSTALL FAILURE] %s\n',ME.message);
    try
        copybin(fullfile(backup,'REPLANNING_SYSTEM.m'),C.prod);
        copybin(fullfile(backup,'run_flight_headless.m'),C.runner);
        restore_optional(backup,'AF7R58ZZY_FINAL_SELFTEST.m',C.selftest);
        restore_optional(backup,'AF7R58ZZY_FINAL_RUN_CANONICAL.m',C.launcher);
        restore_optional(backup,'AF7R58ZZY_FINAL_REPORT.m',C.report);
        restore_optional(backup,'AF7R58ZZY_FINAL_ARCHIVE_LEGACY.m',C.archive);
        rehash; addpath(ref,'-begin'); addpath(root,'-begin');
        show('rollback production',C.prod,EXP.currentProd);
        show('rollback runner',C.runner,EXP.r33runner);
        fprintf(' rollback to exact flown R33              = PASS\n');
    catch MER
        fprintf(' rollback                                = FAILED: %s\n',MER.message);
    end
    OUT.status=['FAILED__' ME.identifier];
    OUT.error=struct('identifier',ME.identifier,'message',ME.message);
    assignin('base','AF7R58ZZY_FINAL_INSTALL_OUT',OUT);
    rethrow(ME);
end

assignin('base','AF7R58ZZY_FINAL_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n');
fprintf(' FINAL INTEGRATION COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf(' status                                  = %s\n',OUT.status);
fprintf(' production main class                   = REPLANNING_SYSTEM\n');
fprintf(' historical runtime subclass required    = 0\n');
fprintf(' controller source modified              = 0\n');
fprintf(' certificate mathematics modified        = 0\n');
fprintf(' four-cycle schedule modified            = 0\n');
fprintf(' outputs consolidated under              = generated/AF7R58ZZY_FINAL\n');
fprintf(' NEXT: AF7R58ZZY_FINAL_RUN_CANONICAL\n');
fprintf(' Then: AF7R58ZZY_FINAL_REPORT\n');
fprintf(' After successful validation only: AF7R58ZZY_FINAL_ARCHIVE_LEGACY(true)\n');
fprintf('====================================================================================================\n');
end

function static_integrated_audit(source,runner)
K=fileread(source);
assert(contains(K,'classdef REPLANNING_SYSTEM < handle'),'[FINAL] main class mismatch.');
assert(~contains(K,'@REPLANNING_SYSTEM')&&~contains(K,'< REPLANNING_SYSTEM_R'), ...
    '[FINAL] historical superclass dependency remains.');
assert(~contains(K,'P.native_raw=RAW'),'[FINAL] native_raw regression.');
assert(contains(K,'r31_prepare_core_bundle_direct(P,ta)')&& ...
    contains(K,'''r31_direct_store_exact'',true),'[FINAL] R31 direct Stage3 store missing.');
assert(contains(K,'candidate=obj.af7r46_single_core_proposal_compact(CTX);')&& ...
    contains(K,'obj.staged_replan_candidate=candidate;')&& ...
    ~contains(K,'r32_single_core_proposal_direct'), ...
    '[FINAL] R31 Stage1 semantics not retained.');
assert(contains(K,'r21_prepare_staged_reference_only_bundle_integrated'), ...
    '[FINAL] exact R21 preflight fallback missing.');
assert(contains(K,'function [ok,reason,B] = r30_prepare_core_bundle_exact'), ...
    '[FINAL] latent exact core helper missing.');
assert(contains(K,'af7r58i_prepare_batch_model(')&& ...
    ~contains(K,'af7r80_fast_batch_append_R29('),'[FINAL] exact batch path mismatch.');
assert(contains(K,'pipeline_intervals=3')&& ...
    contains(K,'pipeline_delay_s=3*double(obj.prediction_dt)'), ...
    '[FINAL] measurement schedule changed.');

names=regexp(K,'(?m)^        function\s+(?:\[[^\]]+\]\s*=\s*|[^=\n]+?=\s*)?([A-Za-z]\w*)\s*\(','tokens');
names=cellfun(@(x)x{1},names,'UniformOutput',false);
[~,~,ic]=unique(names);
cnt=accumarray(ic(:),1);
assert(all(cnt==1),'[FINAL] duplicate method name after flattening.');

R=fileread(runner);
assert(count_occ(R,'REPLANNING_SYSTEM(agent,nominal_ref,replan_opts)')==1, ...
    '[FINAL] runner main-class binding missing/nonunique.');
assert(isempty(regexp(R,'REPLANNING_SYSTEM_R\d+_[A-Za-z0-9_]*\(agent,nominal_ref,replan_opts\)','once')), ...
    '[FINAL] runner still binds a historical subclass.');
fprintf(' static flattening/source-binding audit   = PASS\n');
end

function root=find_root(EXP)
starts=[{pwd,fileparts(mfilename('fullpath'))},strsplit(path,pathsep)];
cand={};
for k=1:numel(starts)
    p=starts{k};
    if isempty(p)||~isfolder(p),continue;end
    for j=1:7
        cand{end+1}=p; %#ok<AGROW>
        q=fileparts(p); if isempty(q)||strcmp(q,p),break;end; p=q;
    end
end
cand=unique(cand); good={};
for k=1:numel(cand)
    r=cand{k};
    rr=fullfile(r,'run_flight_headless.m');
    pp=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    cc=fullfile(r,'REPLANNING_SYSTEM_R33_R31_PLUS_NO_NATIVE_RAW.m');
    if isfile(rr)&&isfile(pp)&&isfile(cc)&& ...
            strcmp(sha_file(rr),EXP.r33runner)&& ...
            strcmp(sha_file(pp),EXP.currentProd)&& ...
            strcmp(sha_file(cc),EXP.r33class)
        good{end+1}=r; %#ok<AGROW>
    end
end
assert(~isempty(good),'[FINAL INSTALL] exact flown R33 common_matlab root not found.');
score=zeros(size(good));
for k=1:numel(good)
    [~,n]=fileparts(good{k});
    score(k)=100*strcmpi(n,'common_matlab')+20*isfolder(fullfile(good{k},'.git'));
end
[~,ix]=max(score); root=good{ix};
end

function p=resolve_exact_source(name,expectedSHA,root,ref)
cand={};
w=which(name,'-all');
if ischar(w)
    if ~isempty(w),cand={w};end
elseif isstring(w)
    cand=cellstr(w(:));
elseif iscell(w)
    cand=w(:).';
end
cand=[cand,{fullfile(root,[name '.m']),fullfile(ref,[name '.m'])}];
good={};
for k=1:numel(cand)
    q=cand{k};
    if ~isempty(q)&&isfile(q)
        try
            if strcmp(sha_file(q),expectedSHA),good{end+1}=q;end %#ok<AGROW>
        catch
        end
    end
end
assert(~isempty(good),'[FINAL INSTALL] exact helper not found: %s',name);
active=which(name);
assert(~isempty(active)&&isfile(active)&&strcmp(sha_file(active),expectedSHA), ...
    '[FINAL INSTALL] active helper is shadowed or wrong: %s',name);
p=active;
end

function backup_one(src,dst,name)
assert(isfile(src)); copyfile(src,fullfile(dst,name),'f');
end
function backup_optional(src,dst,name)
if isfile(src),copyfile(src,fullfile(dst,name),'f');
else
    f=fopen(fullfile(dst,[name '.ABSENT']),'wt');assert(f>=0);fprintf(f,'ABSENT\n');fclose(f);
end
end
function restore_optional(dirp,name,dst)
s=fullfile(dirp,name);a=fullfile(dirp,[name '.ABSENT']);
if isfile(s),copyfile(s,dst,'f');
elseif isfile(a)&&isfile(dst),delete(dst);
end
end
function show(n,p,h)
assert(isfile(p),'[FINAL INSTALL] missing %s: %s',n,p);
q=sha_file(p);fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));
assert(strcmp(q,h),'[FINAL INSTALL] %s SHA mismatch.',n);
end
function n=count_occ(s,p),n=numel(strfind(s,p));end
function copybin(a,b)
f=fopen(a,'rb');assert(f>=0);c=onCleanup(@()fclose(f));x=fread(f,Inf,'*uint8');clear c;
f=fopen(b,'wb');assert(f>=0);c=onCleanup(@()fclose(f));n=fwrite(f,x,'uint8');assert(n==numel(x));
end
function h=sha_file(p)
f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
