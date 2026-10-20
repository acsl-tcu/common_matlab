function OUT = AF7R58ZZY_R26_R3_FULL_AUTHORITY_RECOVERY()
% AF7R58ZZY-R26-R3 FULL AUTHORITY RECOVERY
% =========================================================================
% PURPOSE
%   Recover ONE authoritative R26-R3 execution tree after diagnostic
%   snapshot/backup directories shadowed common_matlab sources on MATLAB path.
%
% CONFIRMED FAILURE MODE
%   A captured diagnostic copy such as
%     ...\reference\tp...\sources\run_flight_headless.m
%   can precede the real repository runner on MATLAB path.  In that state,
%   "which" and a normal "run_flight_headless" command can execute old bytes
%   even when the canonical root runner was correctly patched.
%
% THIS RECOVERY DOES ALL OF THE FOLLOWING IN ONE TRANSACTION:
%   1) Finds the real common_matlab repo by immutable production/source SHAs,
%      NOT by which('run_flight_headless').
%   2) Removes diagnostic tp*/sources and AF7R...BACKUP path entries that can
%      shadow live code; changes pwd to the canonical repo root.
%   3) Restores exact reviewed R26-R3 class + report into the canonical root.
%   4) Rebuilds the canonical runner FROM AN EXACT R25 BACKUP SHA, applying
%      only the intended R26-R3 changes plus the clean-profile NaN fallback.
%   5) Writes the runner in binary mode (no CRLF translation).
%   6) Verifies SHA/path authority for the entire active chain.
%   7) Verifies R26 class parse, runner checkcode, clean profiling OFF,
%      fast merge retained, production/controller untouched.
%
% NO FLIGHT IS RUN HERE.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R26-R3 | FULL PATH / SOURCE AUTHORITY RECOVERY\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] canonicalRootDiscovery=1 | shadowPurge=1 | runnerRebuildFromExactR25=1 | ' ...
    'binaryWrite=1 | productionModified=0 | controllerModified=0 | safetyMathChanged=0\n']);

EXP=struct();
EXP.production='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXP.r25runner='0581116b243151e9b6855aacb676c704ea4b15991620213833f8ba914b9a3451';
EXP.runner='36a5adb8ff7846d175d30d5c59017617634aef24ffd06ad285f4eb355fdf4d98';
EXP.r26='2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6';
EXP.r25='f2f13902bcf1589bacde9092527f3d0c8cb68f9d330d819b5282ad7b002f616c';
EXP.r21='4b22dedd43eceb874e41abde44fbd87f93d88de0b81ea91a724c4cfd0073b90f';
EXP.r19='928677701a06d2c8aabf96982276fb4ada5daf9c67c8ec1354182e823c85e482';
EXP.compile='e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177';
EXP.merge='f7b5f4c46710c0bc4ee64c3a7e839717a14bc9821291bb8e15bddefeef976a36';
EXP.report='e469bcb623f31867f5595dc7830fc065721c4bebf7854bc7233809ebcae90097';

OUT=struct();
OUT.schema='AF7R58ZZY_R26_R3_FULL_AUTHORITY_RECOVERY_V1';
OUT.closed=false;
OUT.production_modified=false;
OUT.controller_modified=false;
OUT.safety_math_changed=false;
OUT.removed_path_entries={};

pkgdir=fileparts(mfilename('fullpath'));
pkgClass=fullfile(pkgdir,'REPLANNING_SYSTEM_R26_BALANCED.m');
pkgReport=fullfile(pkgdir,'AF7R58ZZY_R26_R3_REPORT.m');
assert(isfile(pkgClass)&&strcmp(sha_file(pkgClass),EXP.r26), ...
    '[R26-R3 RECOVERY] package R26 class SHA mismatch.');
assert(isfile(pkgReport)&&strcmp(sha_file(pkgReport),EXP.report), ...
    '[R26-R3 RECOVERY] package report SHA mismatch.');

%% A. Find canonical repository without trusting current which() authority.
fprintf('\n[A. CANONICAL REPOSITORY DISCOVERY]\n');
[root,candidates]=find_canonical_root(EXP);
OUT.root_candidates=candidates;
OUT.root=root;
fprintf(' canonical root                          = %s\n',root);
fprintf(' production                             = %s\n',fullfile(root,'reference','REPLANNING_SYSTEM.m'));
fprintf(' production SHA                         = %s\n',sha_file(fullfile(root,'reference','REPLANNING_SYSTEM.m')));

canonical=struct();
canonical.runner=fullfile(root,'run_flight_headless.m');
canonical.production=fullfile(root,'reference','REPLANNING_SYSTEM.m');
canonical.r26=fullfile(root,'REPLANNING_SYSTEM_R26_BALANCED.m');
canonical.r25=fullfile(root,'REPLANNING_SYSTEM_R25_LEAN.m');
canonical.r21=fullfile(root,'REPLANNING_SYSTEM_R21_BALANCED.m');
canonical.r19=fullfile(root,'REPLANNING_SYSTEM_R19_FAST.m');
canonical.compile=fullfile(root,'af7r76_compile_correction_packet_R23.m');
canonical.merge=fullfile(root,'af7r77_merge_result_reference_fast_R25.m');
canonical.report=fullfile(root,'AF7R58ZZY_R26_R3_REPORT.m');

assert(isfile(canonical.production)&&strcmp(sha_file(canonical.production),EXP.production), ...
    '[R26-R3 RECOVERY] canonical production mismatch.');
assert(isfile(canonical.r25)&&strcmp(sha_file(canonical.r25),EXP.r25), ...
    '[R26-R3 RECOVERY] canonical R25 mismatch.');
assert(isfile(canonical.r21)&&strcmp(sha_file(canonical.r21),EXP.r21), ...
    '[R26-R3 RECOVERY] canonical R21 mismatch.');
assert(isfile(canonical.r19)&&strcmp(sha_file(canonical.r19),EXP.r19), ...
    '[R26-R3 RECOVERY] canonical R19 mismatch.');
assert(isfile(canonical.compile)&&strcmp(sha_file(canonical.compile),EXP.compile), ...
    '[R26-R3 RECOVERY] canonical R23 compile helper mismatch.');
assert(isfile(canonical.merge)&&strcmp(sha_file(canonical.merge),EXP.merge), ...
    '[R26-R3 RECOVERY] canonical R25 merge helper mismatch.');

%% B. Diagnose and remove path shadows BEFORE using which() for authority.
fprintf('\n[B. PATH SHADOW AUDIT / PURGE]\n');
oldpwd=pwd;
fprintf(' pwd before                              = %s\n',oldpwd);

entries=strsplit(path,pathsep);
remove={};
rootN=normpath(root);
for k=1:numel(entries)
    e=entries{k};
    if isempty(e), continue; end
    en=normpath(e);
    if ~startsWith(en,[rootN filesep]) && ~strcmp(en,rootN)
        continue;
    end

    rel=en(numel(rootN)+1:end);
    if startsWith(rel,filesep), rel=rel(2:end); end
    comps=regexp(rel,'[\\/]+','split');

    isTmp=false;
    isBackup=false;
    for j=1:numel(comps)
        c=lower(comps{j});
        if startsWith(c,'tp') && numel(c)>10
            isTmp=true;
        end
        if contains(c,'backup_') || startsWith(c,'af7r58zzy_r26_') && contains(c,'backup')
            isBackup=true;
        end
    end

    % Also remove any noncanonical directory that directly contains a live
    % target basename. This catches source snapshots with unexpected names.
    targetShadow = ~strcmp(en,rootN) && ( ...
        isfile(fullfile(e,'run_flight_headless.m')) || ...
        isfile(fullfile(e,'REPLANNING_SYSTEM_R26_BALANCED.m')) || ...
        isfile(fullfile(e,'REPLANNING_SYSTEM_R25_LEAN.m')) || ...
        isfile(fullfile(e,'REPLANNING_SYSTEM_R21_BALANCED.m')) || ...
        isfile(fullfile(e,'REPLANNING_SYSTEM_R19_FAST.m')) || ...
        isfile(fullfile(e,'af7r76_compile_correction_packet_R23.m')) || ...
        isfile(fullfile(e,'af7r77_merge_result_reference_fast_R25.m')));

    if isTmp || isBackup || targetShadow
        remove{end+1,1}=e; %#ok<AGROW>
    end
end
remove=unique(remove,'stable');

% If current folder is within a shadow tree, leave it first.
pwdN=normpath(pwd);
mustCd=~strcmp(pwdN,rootN);
if mustCd
    % Always execute from canonical root after recovery. This also prevents
    % current-folder precedence from selecting a captured source.
    cd(root);
end

for k=1:numel(remove)
    try
        rmpath(remove{k});
        fprintf(' removed path                            = %s\n',remove{k});
    catch ME
        fprintf(' remove path WARNING                     = %s | %s\n',remove{k},ME.message);
    end
end
OUT.removed_path_entries=remove;

% Put live root first and canonical production reference second.
addpath(fullfile(root,'reference'),'-begin');
addpath(root,'-begin');
rehash;

fprintf(' pwd after                               = %s\n',pwd);

%% C. Restore reviewed R26 class/report to canonical root.
fprintf('\n[C. RESTORE REVIEWED R26-R3 CLASS / REPORT]\n');
copy_binary(pkgClass,canonical.r26);
copy_binary(pkgReport,canonical.report);
fprintf(' R26 class SHA                           = %s\n',sha_file(canonical.r26));
fprintf(' R26 report SHA                          = %s\n',sha_file(canonical.report));
assert(strcmp(sha_file(canonical.r26),EXP.r26),'[R26-R3 RECOVERY] R26 copy verification failed.');
assert(strcmp(sha_file(canonical.report),EXP.report),'[R26-R3 RECOVERY] report copy verification failed.');

%% D. Find exact R25 runner authority, independent of current runner state.
fprintf('\n[D. FIND EXACT R25 RUNNER AUTHORITY]\n');
r25source=find_exact_r25_runner(root,canonical.runner,EXP.r25runner);
assert(~isempty(r25source), ...
    '[R26-R3 RECOVERY] exact R25 runner SHA was not found in canonical runner/backups.');
fprintf(' selected R25 source                    = %s\n',r25source);
fprintf(' selected R25 SHA                       = %s\n',sha_file(r25source));

%% E. Deterministically rebuild the intended clean R26-R3 runner.
fprintf('\n[E. DETERMINISTIC RUNNER REBUILD]\n');
b=read_bytes(r25source);

[b,n1]=replace_ascii_once(b, ...
    'REPLANNING_SYSTEM_R25_LEAN(agent,nominal_ref,replan_opts)', ...
    'REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)');
[b,n2]=replace_ascii_once(b, ...
    'replan_opts.ENABLE_PROFILING = true;', ...
    'replan_opts.ENABLE_PROFILING = false;');
[b,n3]=replace_ascii_once(b, ...
    'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = true;', ...
    'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;');
[b,n4]=replace_ascii_once(b, ...
    'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 0.0;', ...
    'replan_opts.SPIKE_CAPTURE_THRESHOLD_MS = 2.0;');
assert(all([n1 n2 n3 n4]==1), ...
    '[R26-R3 RECOVERY] R26 option/constructor replacement count mismatch.');

% Clean-mode diagnostic snapshot patch: when light/internal profiling is OFF,
% internal STATE_CLASS timing fields may legitimately be empty. Preserve the
% outer timing authority and store NaN only for unavailable internal mirrors.
oldLines={ ...
'    % Snapshot scalar diagnostic channels; time this overhead explicitly.', ...
'    r22t=tic; r22st=replanner_instance.result.state;', ...
'    r22_rt_light_e2e_ms=double(r22st.rt_light_e2e_ms);', ...
'    r22_rt_observer_ms=double(r22st.rt_observer_ms);', ...
'    r22_rt_live_binding_ms=double(r22st.rt_live_binding_ms);', ...
'    r22_rt_dependency_ms=double(r22st.rt_dependency_ms);', ...
'    r22_rt_base_ref_ms=double(r22st.rt_base_ref_ms);', ...
'    r22_rt_cert_guard_ms=double(r22st.rt_cert_guard_ms);', ...
'    r22_rt_geometry_eval_ms=double(r22st.rt_geometry_eval_ms);', ...
'    r22_rt_reference_packet_ms=double(r22st.rt_reference_packet_ms);', ...
'    r22_rt_sensor_ms=double(r22st.rt_sensor_ms);', ...
'    r22_rt_kernel_ms=double(r22st.rt_kernel_ms);', ...
'    r22_rt_trigger_ms=double(r22st.rt_trigger_ms);', ...
'    r22_rt_replan_status_ms=double(r22st.rt_replan_status_ms);', ...
'    r22_rt_output_write_ms=double(r22st.rt_output_write_ms);', ...
'    r22_rt_state_trigger_write_ms=double(r22st.rt_state_trigger_write_ms);', ...
'    r22_rt_state_cert_write_ms=double(r22st.rt_state_cert_write_ms);', ...
'    r22_rt_state_meta_write_ms=double(r22st.rt_state_meta_write_ms);', ...
'    r22_rt_total_ms=double(r22st.rt_total_ms);', ...
'    r22_spike_front_ms=double(r22st.rt_spike_stage_front_ms);', ...
'    r22_spike_loadmass_ms=double(r22st.rt_spike_stage_loadmass_ms);', ...
'    r22_spike_dependency_ms=double(r22st.rt_spike_stage_dependency_ms);', ...
'    r22_spike_base_cert_geom_ms=double(r22st.rt_spike_stage_base_cert_geom_ms);', ...
'    r22_spike_sensor_trigger_ms=double(r22st.rt_spike_stage_sensor_trigger_ms);', ...
'    r22_spike_output_state_ms=double(r22st.rt_spike_stage_output_state_ms);', ...
'    r22_spike_write_active_output_ms=double(r22st.rt_spike_write_active_output_ms);', ...
'    r22_trigger_event_capture_ms=double(r22st.trigger_event_capture_ms);', ...
'    r22_diag_snapshot_s=toc(r22t);' ...
};

newLines={ ...
'    % Snapshot scalar diagnostic channels; time this overhead explicitly.', ...
'    % R26-R3 clean mode intentionally disables the internal light audit.', ...
'    % Empty internal timing mirrors are stored as NaN; outer timing remains authoritative.', ...
'    r22t=tic; r22st=replanner_instance.result.state;', ...
'    if replanner_instance.ENABLE_LIGHT_RUNTIME_AUDIT || replanner_instance.ENABLE_PROFILING', ...
'        r22_rt_light_e2e_ms=double(r22st.rt_light_e2e_ms);', ...
'        r22_rt_observer_ms=double(r22st.rt_observer_ms);', ...
'        r22_rt_live_binding_ms=double(r22st.rt_live_binding_ms);', ...
'        r22_rt_dependency_ms=double(r22st.rt_dependency_ms);', ...
'        r22_rt_base_ref_ms=double(r22st.rt_base_ref_ms);', ...
'        r22_rt_cert_guard_ms=double(r22st.rt_cert_guard_ms);', ...
'        r22_rt_geometry_eval_ms=double(r22st.rt_geometry_eval_ms);', ...
'        r22_rt_reference_packet_ms=double(r22st.rt_reference_packet_ms);', ...
'        r22_rt_sensor_ms=double(r22st.rt_sensor_ms);', ...
'        r22_rt_kernel_ms=double(r22st.rt_kernel_ms);', ...
'        r22_rt_trigger_ms=double(r22st.rt_trigger_ms);', ...
'        r22_rt_replan_status_ms=double(r22st.rt_replan_status_ms);', ...
'        r22_rt_output_write_ms=double(r22st.rt_output_write_ms);', ...
'        r22_rt_state_trigger_write_ms=double(r22st.rt_state_trigger_write_ms);', ...
'        r22_rt_state_cert_write_ms=double(r22st.rt_state_cert_write_ms);', ...
'        r22_rt_state_meta_write_ms=double(r22st.rt_state_meta_write_ms);', ...
'        r22_rt_total_ms=double(r22st.rt_total_ms);', ...
'        r22_spike_front_ms=double(r22st.rt_spike_stage_front_ms);', ...
'        r22_spike_loadmass_ms=double(r22st.rt_spike_stage_loadmass_ms);', ...
'        r22_spike_dependency_ms=double(r22st.rt_spike_stage_dependency_ms);', ...
'        r22_spike_base_cert_geom_ms=double(r22st.rt_spike_stage_base_cert_geom_ms);', ...
'        r22_spike_sensor_trigger_ms=double(r22st.rt_spike_stage_sensor_trigger_ms);', ...
'        r22_spike_output_state_ms=double(r22st.rt_spike_stage_output_state_ms);', ...
'        r22_spike_write_active_output_ms=double(r22st.rt_spike_write_active_output_ms);', ...
'        r22_trigger_event_capture_ms=double(r22st.trigger_event_capture_ms);', ...
'    else', ...
'        r22_rt_light_e2e_ms=NaN;', ...
'        r22_rt_observer_ms=NaN;', ...
'        r22_rt_live_binding_ms=NaN;', ...
'        r22_rt_dependency_ms=NaN;', ...
'        r22_rt_base_ref_ms=NaN;', ...
'        r22_rt_cert_guard_ms=NaN;', ...
'        r22_rt_geometry_eval_ms=NaN;', ...
'        r22_rt_reference_packet_ms=NaN;', ...
'        r22_rt_sensor_ms=NaN;', ...
'        r22_rt_kernel_ms=NaN;', ...
'        r22_rt_trigger_ms=NaN;', ...
'        r22_rt_replan_status_ms=NaN;', ...
'        r22_rt_output_write_ms=NaN;', ...
'        r22_rt_state_trigger_write_ms=NaN;', ...
'        r22_rt_state_cert_write_ms=NaN;', ...
'        r22_rt_state_meta_write_ms=NaN;', ...
'        r22_rt_total_ms=NaN;', ...
'        r22_spike_front_ms=NaN;', ...
'        r22_spike_loadmass_ms=NaN;', ...
'        r22_spike_dependency_ms=NaN;', ...
'        r22_spike_base_cert_geom_ms=NaN;', ...
'        r22_spike_sensor_trigger_ms=NaN;', ...
'        r22_spike_output_state_ms=NaN;', ...
'        r22_spike_write_active_output_ms=NaN;', ...
'        r22_trigger_event_capture_ms=NaN;', ...
'    end', ...
'    r22_diag_snapshot_s=toc(r22t);' ...
};

eol=sprintf('\r\n');
old=uint8(strjoin(oldLines,eol));
new=uint8(strjoin(newLines,eol));
ix=strfind(rowvec(b),rowvec(old));
fprintf(' diagnostic block matches               = %d\n',numel(ix));
assert(numel(ix)==1, ...
    '[R26-R3 RECOVERY] exact R25 diagnostic block not found uniquely.');
i=ix(1);
b=[b(1:i-1);new(:);b(i+numel(old):end)];

candidateSHA=sha_bytes(b);
[nCRLF,nCRCRLF,nLF]=eol_counts(b);
fprintf(' rebuilt runner SHA                     = %s\n',candidateSHA);
fprintf(' expected runner SHA                    = %s\n',EXP.runner);
fprintf(' CRLF / CRCRLF / LF                     = %d / %d / %d\n',nCRLF,nCRCRLF,nLF);
assert(strcmp(candidateSHA,EXP.runner), ...
    '[R26-R3 RECOVERY] deterministic runner SHA mismatch.');
assert(nCRCRLF==0,'[R26-R3 RECOVERY] CRCRLF detected.');

% Backup canonical current runner regardless of its state.
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
currentBackup=fullfile(root,['run_flight_headless.PRE_FULL_RECOVERY_' stamp '.m']);
if isfile(canonical.runner)
    copyfile(canonical.runner,currentBackup);
end
OUT.pre_recovery_runner_backup=currentBackup;

write_bytes(canonical.runner,b);
rehash;
assert(strcmp(sha_file(canonical.runner),EXP.runner), ...
    '[R26-R3 RECOVERY] canonical runner write verification failed.');

%% F. Clear cached target symbols and re-establish canonical path order.
fprintf('\n[F. CLEAR CACHE / RE-ESTABLISH AUTHORITY]\n');
try, evalin('base','clear replanner_instance runtime_profile'); catch, end
try, clear run_flight_headless; catch, end
try, clear REPLANNING_SYSTEM_R26_BALANCED; catch, end
rehash;
addpath(fullfile(root,'reference'),'-begin');
addpath(root,'-begin');
cd(root);
rehash;

%% G. Exact active-source binding audit.
fprintf('\n[G. EXACT ACTIVE SOURCE BINDING]\n');
targets={ ...
    'run_flight_headless',canonical.runner,EXP.runner; ...
    'REPLANNING_SYSTEM_R26_BALANCED',canonical.r26,EXP.r26; ...
    'REPLANNING_SYSTEM_R25_LEAN',canonical.r25,EXP.r25; ...
    'REPLANNING_SYSTEM_R21_BALANCED',canonical.r21,EXP.r21; ...
    'REPLANNING_SYSTEM_R19_FAST',canonical.r19,EXP.r19; ...
    'af7r76_compile_correction_packet_R23',canonical.compile,EXP.compile; ...
    'af7r77_merge_result_reference_fast_R25',canonical.merge,EXP.merge; ...
    'REPLANNING_SYSTEM',canonical.production,EXP.production; ...
    'AF7R58ZZY_R26_R3_REPORT',canonical.report,EXP.report ...
};

OUT.bindings=struct();
for k=1:size(targets,1)
    nm=targets{k,1}; expPath=targets{k,2}; expSHA=targets{k,3};
    got=which(nm);
    allgot=which(nm,'-all');
    if ischar(allgot), allgot={allgot}; end
    gotSHA='';
    if ~isempty(got)&&isfile(got), gotSHA=sha_file(got); end
    exactPath=paths_equal(got,expPath);
    exactSHA=strcmp(gotSHA,expSHA);
    fprintf(' %-40s\n',nm);
    fprintf('   resolved = %s\n',got);
    fprintf('   pathOK   = %d | shaOK=%d | SHA=%s\n',exactPath,exactSHA,gotSHA);
    if numel(allgot)>1
        for j=1:numel(allgot)
            fprintf('   all[%d]   = %s\n',j,allgot{j});
        end
    end
    assert(exactPath&&exactSHA, ...
        '[R26-R3 RECOVERY] active binding mismatch: %s',nm);
    key=matlab.lang.makeValidName(nm);
    OUT.bindings.(key)=struct('resolved',got,'sha',gotSHA,'all',{allgot});
end

%% H. Parse/source semantic checks.
fprintf('\n[H. PARSE / SEMANTIC GUARDS]\n');
runnerText=fileread(canonical.runner);
assert(contains(runnerText,'REPLANNING_SYSTEM_R26_BALANCED(agent,nominal_ref,replan_opts)'), ...
    '[R26-R3 RECOVERY] R26 constructor missing.');
assert(contains(runnerText,'replan_opts.ENABLE_PROFILING = false;'), ...
    '[R26-R3 RECOVERY] profiling is not OFF.');
assert(contains(runnerText,'replan_opts.ENABLE_LIGHT_RUNTIME_AUDIT = false;'), ...
    '[R26-R3 RECOVERY] light audit is not OFF.');
assert(contains(runnerText,'r22_rt_observer_ms=NaN;'), ...
    '[R26-R3 RECOVERY] clean diagnostic fallback missing.');
assert(contains(runnerText, ...
    'af7r77_merge_result_reference_fast_R25(agent.reference.result,r22_sload_result)'), ...
    '[R26-R3 RECOVERY] R25 fast merge not retained.');

C=checkcode(canonical.runner,'-id');
OUT.runner_checkcode=C;
fprintf(' runner checkcode entries               = %d\n',numel(C));

mc=meta.class.fromName('REPLANNING_SYSTEM_R26_BALANCED');
assert(~isempty(mc),'[R26-R3 RECOVERY] R26 class parse failed.');
fprintf(' R26 class parse                         = PASS\n');

classText=fileread(canonical.r26);
assert(contains(classText,'AF7R58ZZY_R26_BALANCED_R3'), ...
    '[R26-R3 RECOVERY] R26 build id mismatch.');
assert(contains(classText, ...
    '[ok,reason,B]=prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED'), ...
    '[R26-R3 RECOVERY] legal same-name R21 bridge missing.');
assert(contains(classText,'[ok,reason,B]=obj.prepare_staged_reference_only_bundle(P,ta);'), ...
    '[R26-R3 RECOVERY] normal dispatch bridge missing.');
assert(~contains(extract_between(classText, ...
    'function advance_staged_replan_pipeline', ...
    'function [ok,reason] = activate_staged_reference_only_bundle'), ...
    'prepare_staged_reference_only_bundle@REPLANNING_SYSTEM_R21_BALANCED'), ...
    '[R26-R3 RECOVERY] illegal direct superclass dispatch remains in advance().');
assert(contains(classText,'CP=af7r76_compile_correction_packet_R23('), ...
    '[R26-R3 RECOVERY] exact R23 final compile missing.');

%% I. Production/controller immutability re-check.
fprintf('\n[I. IMMUTABILITY]\n');
OUT.production_sha=sha_file(canonical.production);
OUT.production_modified=~strcmp(OUT.production_sha,EXP.production);
fprintf(' production SHA                          = %s\n',OUT.production_sha);
fprintf(' production modified                     = %d\n',OUT.production_modified);
assert(~OUT.production_modified,'[R26-R3 RECOVERY] production changed.');

% No controller file is written by this recovery. Record active controller path.
try
    OUT.controller_path=which('HLC_SUSPENDED_LOAD');
catch
    OUT.controller_path='';
end
fprintf(' controller source write performed       = 0\n');
fprintf(' controller resolved                     = %s\n',OUT.controller_path);

OUT.closed=true;
OUT.status='R26_R3_FULL_AUTHORITY_RECOVERED__RUN_CLEAN_FLIGHT_NEXT';
assignin('base','AF7R58ZZY_R26_R3_FULL_AUTHORITY_RECOVERY_OUT',OUT);

fprintf('\n====================================================================================================\n');
fprintf(' R26-R3 FULL AUTHORITY RECOVERY COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('canonical root                          = %s\n',root);
fprintf('active runner                           = %s\n',which('run_flight_headless'));
fprintf('active runner SHA                       = %s\n',sha_file(which('run_flight_headless')));
fprintf('active R26                              = %s\n',which('REPLANNING_SYSTEM_R26_BALANCED'));
fprintf('profiling/light audit                   = OFF / OFF\n');
fprintf('outer timing                            = retained\n');
fprintf('production replanner modified           = 0\n');
fprintf('controller modified                     = 0\n');
fprintf('SELFTEST CLOSED                         = 1\n');
fprintf('\nNEXT, IN THIS SAME MATLAB SESSION:\n');
fprintf('  1) which run_flight_headless -all\n');
fprintf('  2) run_flight_headless\n');
fprintf('  3) AF7R58ZZY_R26_R3_REPORT\n');
fprintf('====================================================================================================\n');
end

% =========================================================================
function [root,info]=find_canonical_root(EXP)
starts={pwd,fileparts(mfilename('fullpath'))};
pe=strsplit(path,pathsep);
starts=[starts,pe];
cand={};
for k=1:numel(starts)
    p=starts{k};
    if isempty(p)||~isfolder(p), continue; end
    p=canonical_path(p);
    for j=1:6
        cand{end+1,1}=p; %#ok<AGROW>
        q=fileparts(p);
        if strcmp(q,p)||isempty(q), break; end
        p=q;
    end
end
cand=unique(cand,'stable');

good={};
for k=1:numel(cand)
    r=cand{k};
    pProd=fullfile(r,'reference','REPLANNING_SYSTEM.m');
    p21=fullfile(r,'REPLANNING_SYSTEM_R21_BALANCED.m');
    p19=fullfile(r,'REPLANNING_SYSTEM_R19_FAST.m');
    pC=fullfile(r,'af7r76_compile_correction_packet_R23.m');
    pM=fullfile(r,'af7r77_merge_result_reference_fast_R25.m');
    try
        ok=isfile(pProd)&&strcmp(sha_file(pProd),EXP.production) && ...
            isfile(p21)&&strcmp(sha_file(p21),EXP.r21) && ...
            isfile(p19)&&strcmp(sha_file(p19),EXP.r19) && ...
            isfile(pC)&&strcmp(sha_file(pC),EXP.compile) && ...
            isfile(pM)&&strcmp(sha_file(pM),EXP.merge);
    catch
        ok=false;
    end
    if ok
        good{end+1,1}=r; %#ok<AGROW>
    end
end
good=unique(good,'stable');
assert(~isempty(good),'[R26-R3 RECOVERY] canonical common_matlab root not found.');

% Prefer a real Git working tree, then a path ending in common_matlab.
score=zeros(numel(good),1);
for k=1:numel(good)
    if isfolder(fullfile(good{k},'.git')), score(k)=score(k)+100; end
    [~,nm]=fileparts(good{k});
    if strcmpi(nm,'common_matlab'), score(k)=score(k)+10; end
end
[~,ix]=max(score);
root=good{ix};
info=good;
end

function p=find_exact_r25_runner(root,canonicalRunner,expectedSHA)
p='';
if isfile(canonicalRunner)
    try
        if strcmp(sha_file(canonicalRunner),expectedSHA)
            p=canonicalRunner; return;
        end
    catch
    end
end

patterns={ ...
    'AF7R58ZZY_R26_R3_BACKUP_*', ...
    'AF7R58ZZY_R26_R2_BACKUP_*', ...
    'AF7R58ZZY_R26_BACKUP_*', ...
    'AF7R58ZZY_R25_BACKUP_*' ...
};
Dall=[];
for k=1:numel(patterns)
    D=dir(fullfile(root,patterns{k}));
    Dall=[Dall;D(:)]; %#ok<AGROW>
end
if ~isempty(Dall)
    [~,ord]=sort([Dall.datenum],'descend');
    Dall=Dall(ord);
end
for k=1:numel(Dall)
    q=fullfile(Dall(k).folder,Dall(k).name,'run_flight_headless.m');
    if ~isfile(q), continue; end
    try
        if strcmp(sha_file(q),expectedSHA)
            p=q; return;
        end
    catch
    end
end

% Last resort: shallow scan only directories whose names explicitly say backup.
D=dir(root);
for k=1:numel(D)
    if ~D(k).isdir, continue; end
    nm=D(k).name;
    if strcmp(nm,'.')||strcmp(nm,'..')||~contains(lower(nm),'backup'), continue; end
    q=fullfile(root,nm,'run_flight_headless.m');
    if isfile(q)
        try
            if strcmp(sha_file(q),expectedSHA)
                p=q; return;
            end
        catch
        end
    end
end
end

function [b,n]=replace_ascii_once(b,old,new)
r=rowvec(b); o=rowvec(uint8(old)); z=rowvec(uint8(new));
ix=strfind(r,o); n=numel(ix);
if n~=1, return; end
i=ix(1);
r=[r(1:i-1) z r(i+numel(o):end)];
b=r(:);
end

function s=extract_between(txt,a,b)
ia=strfind(txt,a); ib=strfind(txt,b);
if isempty(ia)||isempty(ib), s=''; return; end
i=ia(1); j=ib(find(ib>i,1,'first'));
if isempty(j), s=''; else, s=txt(i:j-1); end
end

function tf=paths_equal(a,b)
if isempty(a)||isempty(b), tf=false; return; end
aa=normpath(a); bb=normpath(b);
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
b=read_bytes(src);
write_bytes(dst,b);
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
r=rowvec(b);
a=numel(strfind(r,uint8([13 10])));
c=numel(strfind(r,uint8([13 13 10])));
d=sum(r==10);
end

function x=rowvec(x)
x=x(:).';
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
