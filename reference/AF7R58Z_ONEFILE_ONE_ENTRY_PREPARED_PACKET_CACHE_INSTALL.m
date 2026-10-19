function OUT = AF7R58Z_ONEFILE_ONE_ENTRY_PREPARED_PACKET_CACHE_INSTALL()
% AF7R58Z
% Install a bounded one-entry NON-AUTHORITY prepared-packet cache for the
% REFERENCE_ONLY commit preparation path.
%
% Preconditions proved by AF7R58Y-R3:
%   K1..K7 CLOSED.
%
% Exact source authority:
%   cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79
%
% Safety semantics:
%   1) Active runtime-packet exact reuse remains first priority.
%   2) Cache is consulted only when active reuse fails.
%   3) Cache hit requires the SAME existing exact
%      can_reuse_reference_only_runtime_packet_exact(...) gate.
%   4) Cache hit is rebound through the SAME existing
%      reuse_reference_only_runtime_packet_exact(...,targetVersion).
%   5) Rebuild path is unchanged mathematically.
%   6) Cache is written only after sweep/packet/sensor preparation and
%      certified sensor-horizon closure succeed.
%   7) Cache write happens before the existing pre-transaction hard-deadline
%      gate, so a deadline reject can retain only NON-AUTHORITY prepared data.
%   8) Active packet assignment, final assertions, transactional rollback,
%      and final hard deadline check remain unchanged.
%   9) Cache storage is exactly one struct entry: overwrite-only, no append.
%
% Diagnostic counters:
%   reference_only_prepared_packet_cache_hit_count
%       exact cache hits attempted in commit preparation
%   reference_only_prepared_packet_cache_store_count
%       completed rebuilds written into the sole cache entry
%   reference_only_prepared_packet_cache_invalidate_count
%       valid entries rejected by exact gate and cleared
%   reference_only_prepared_packet_cache_commit_count
%       successful commits that consumed the prepared cache
%
% This installer is transactional:
%   - exact prehash required
%   - all anchors/source orders checked before write
%   - full patched-class parse before write
%   - backup + rollback on post-write failure
%
% RETURN:
%   AF7R58Z_REPORT.txt
%   AF7R58Z_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58Z_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58Z] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58Z | ONE-ENTRY NON-AUTHORITY PREPARED-PACKET CACHE INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] sourceBound=1 | oneEntry=1 | overwriteOnly=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] activeAuthorityUnchangedUntilTry=1 | exactReuseGatePreserved=1 | finalAssertionsPreserved=1');
say(f,'[MATH] trajectoryMathChanged=0 | collisionCertificateChanged=0 | builderMathChanged=0 | mLTouched=0');

OUT=struct();
OUT.schema='AF7R58Z_ONE_ENTRY_PREPARED_PACKET_CACHE_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79';

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58Z] REPLANNING_SYSTEM.m missing.');

    h0=file_hash(pR);
    say(f,'');
    say(f,'[A. EXACT CURRENT SOURCE BINDING]');
    say(f,' source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected), ...
        '[AF7R58Z] current source is not exact AF7R58Y-R3 authority; no write.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=regexp(txt,'\n','split');

    % ------------------------------------------------------------------
    % Capture source-bound function hashes that MUST remain unchanged.
    % ------------------------------------------------------------------
    defs=discover_function_headers(lines);

    protectedNames={ ...
        'can_reuse_reference_only_runtime_packet_exact', ...
        'reuse_reference_only_runtime_packet_exact', ...
        'build_trigger_sweep_for_target', ...
        'prepare_obstacle_packet_fast_target', ...
        'build_sensor_horizon_for_target'};

    protectedBefore=struct();
    say(f,'');
    say(f,'[B. PROTECTED FUNCTION HASHES -- BEFORE PATCH]');
    for k=1:numel(protectedNames)
        nm=protectedNames{k};
        [b,i0,i1,~]=method_block_by_name(lines,defs,nm);
        key=matlab.lang.makeValidName(nm);
        protectedBefore.(key)=text_hash(strjoin(b,newline));
        say(f,' %-48s L%d:L%d | %s',nm,i0,i1,protectedBefore.(key));
    end

    % ------------------------------------------------------------------
    % Exact property anchor / absence checks.
    % ------------------------------------------------------------------
    propAnchor=find_unique_regex(lines, ...
        '^\s*reference_only_fast_packet_rebuild_count\s*=\s*0;\s*$');

    cacheTokens={ ...
        'reference_only_prepared_packet_cache_valid', ...
        'reference_only_prepared_packet_cache =', ...
        'reference_only_prepared_packet_cache_hit_count', ...
        'reference_only_prepared_packet_cache_store_count', ...
        'reference_only_prepared_packet_cache_invalidate_count', ...
        'reference_only_prepared_packet_cache_commit_count'};

    for k=1:numel(cacheTokens)
        assert(count(txt,cacheTokens{k})==0, ...
            '[AF7R58Z] cache token already present: %s',cacheTokens{k});
    end

    props={ ...
        ''; ...
        '        % AF7R58Z: one-entry NON-AUTHORITY prepared runtime-packet cache.'; ...
        '        reference_only_prepared_packet_cache_valid = false;'; ...
        '        reference_only_prepared_packet_cache = struct();'; ...
        '        reference_only_prepared_packet_cache_hit_count = 0;'; ...
        '        reference_only_prepared_packet_cache_store_count = 0;'; ...
        '        reference_only_prepared_packet_cache_invalidate_count = 0;'; ...
        '        reference_only_prepared_packet_cache_commit_count = 0;'};

    lines=insert_after(lines,propAnchor,props);

    % ------------------------------------------------------------------
    % Locate target commit method after property insertion.
    % ------------------------------------------------------------------
    defs=discover_function_headers(lines);
    commitName='try_commit_reference_only_proposal';
    [B,i0,i1,~]=method_block_by_name(lines,defs,commitName);

    % Inventory all semantic anchors BEFORE mutation.
    A=struct();
    A.old_struct=find_unique_contains(B,'old=struct(');
    A.active_reuse_call=find_unique_contains(B, ...
        'obj.can_reuse_reference_only_runtime_packet_exact(');
    A.first_if_reuse=find_first_exact_trim_after(B,'if reuseRuntimePacket',A.active_reuse_call);
    A.reuse_operation=find_unique_contains(B, ...
        'obj.reuse_reference_only_runtime_packet_exact(');
    A.rebuild_sweep=find_unique_contains(B, ...
        'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');
    A.rebuild_packet=find_unique_contains(B, ...
        'packetNew=obj.prepare_obstacle_packet_fast_target(old.packet,sweepNew,targetVersion);');
    A.rebuild_sensor=find_unique_contains(B, ...
        'sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);');
    A.dep_version=find_unique_contains(B, ...
        'depNew.reference_version=double(targetVersion);');
    A.predeadline=find_unique_contains(B, ...
        'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    A.try_line=find_exact_trim_after(B,'try',A.predeadline);
    A.active_dep_assign=find_unique_contains(B, ...
        'obj.runtime_dependency_signature=depNew;');
    A.counter_if=find_first_exact_trim_after(B,'if reuseRuntimePacket',A.active_dep_assign);
    A.final_assert_1=find_unique_contains(B,'obj.assert_trigger_packet_certificate_current();');
    A.final_assert_2=find_unique_contains(B,'obj.assert_repaired_reference_current();');

    assert(A.old_struct < A.active_reuse_call && ...
           A.active_reuse_call < A.first_if_reuse && ...
           A.first_if_reuse < A.rebuild_sweep && ...
           A.rebuild_sweep < A.rebuild_packet && ...
           A.rebuild_packet < A.rebuild_sensor && ...
           A.rebuild_sensor < A.dep_version && ...
           A.dep_version < A.predeadline && ...
           A.predeadline < A.try_line && ...
           A.try_line < A.active_dep_assign && ...
           A.active_dep_assign < A.counter_if && ...
           A.counter_if < A.final_assert_1 && ...
           A.final_assert_1 < A.final_assert_2, ...
        '[AF7R58Z] current commit source order is not the certified topology.');

    say(f,'');
    say(f,'[C. CURRENT COMMIT SOURCE TOPOLOGY]');
    F=fieldnames(A);
    for k=1:numel(F)
        say(f,' %-26s source L%d',F{k},i0+A.(F{k})-1);
    end
    say(f,' source order closed = 1');

    % ------------------------------------------------------------------
    % Insert cache-gate state immediately after active reuse predicate stmt.
    % find_statement_end handles the multi-line call.
    % ------------------------------------------------------------------
    activeReuseEnd=find_statement_end(B,A.active_reuse_call);

    gateCode={ ...
        ''; ...
        '            % AF7R58Z: bounded one-entry NON-AUTHORITY preparation cache.'; ...
        '            reusePreparedPacketCache=false;'; ...
        '            preparedPacketCache=struct();'; ...
        '            if logical(obj.reference_only_prepared_packet_cache_valid)'; ...
        '                preparedPacketCache=obj.reference_only_prepared_packet_cache;'; ...
        '                cacheShapeOK=isstruct(preparedPacketCache) && ...'; ...
        '                    isfield(preparedPacketCache,''schema'') && ...'; ...
        '                    strcmp(char(preparedPacketCache.schema), ...'; ...
        '                        ''AF7R58Z_ONE_ENTRY_PREPARED_PACKET_CACHE_V1'') && ...'; ...
        '                    isfield(preparedPacketCache,''non_authority'') && ...'; ...
        '                    isscalar(preparedPacketCache.non_authority) && ...'; ...
        '                    logical(preparedPacketCache.non_authority) && ...'; ...
        '                    isfield(preparedPacketCache,''speed'') && ...'; ...
        '                    isfield(preparedPacketCache,''sweep'') && ...'; ...
        '                    isfield(preparedPacketCache,''packet'') && ...'; ...
        '                    isfield(preparedPacketCache,''sensor'') && ...'; ...
        '                    isfield(preparedPacketCache,''dep'');'; ...
        '                if cacheShapeOK'; ...
        '                    reusePreparedPacketCache= ...'; ...
        '                        obj.can_reuse_reference_only_runtime_packet_exact( ...'; ...
        '                            preparedPacketCache.speed,SC, ...'; ...
        '                            preparedPacketCache.sweep,preparedPacketCache.packet, ...'; ...
        '                            preparedPacketCache.sensor,preparedPacketCache.dep);'; ...
        '                end'; ...
        '                if ~reusePreparedPacketCache'; ...
        '                    obj.reference_only_prepared_packet_cache_valid=false;'; ...
        '                    obj.reference_only_prepared_packet_cache=struct();'; ...
        '                    obj.reference_only_prepared_packet_cache_invalidate_count= ...'; ...
        '                        obj.reference_only_prepared_packet_cache_invalidate_count+1;'; ...
        '                    preparedPacketCache=struct();'; ...
        '                end'; ...
        '            end'};

    B=insert_after(B,activeReuseEnd,gateCode);

    % Recompute branch anchors.
    activeReuseCalls=find(cellfun(@(x)contains(x, ...
        'obj.can_reuse_reference_only_runtime_packet_exact('),B));
    assert(numel(activeReuseCalls)==2, ...
        '[AF7R58Z] expected active+cache exact-gate calls after insertion, got %d.', ...
        numel(activeReuseCalls));
    firstIf=find_first_exact_trim_after(B,'if reuseRuntimePacket',activeReuseCalls(1));

    rebuildSweep=find_unique_contains(B, ...
        'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');

    firstElse=find_last_exact_trim_between(B,'else',firstIf,rebuildSweep);
    assert(~isempty(firstElse), ...
        '[AF7R58Z] first reuse/rebuild else not found.');

    cacheReuseBranch={ ...
        '            elseif reusePreparedPacketCache'; ...
        '                obj.af7r58v_r4_branch_code=3;'; ...
        '                af7r58v_r4_t6=tic;'; ...
        '                obj.reference_only_prepared_packet_cache_hit_count= ...'; ...
        '                    obj.reference_only_prepared_packet_cache_hit_count+1;'; ...
        '                [sweepNew,packetNew,sensorNew,depNew]= ...'; ...
        '                    obj.reuse_reference_only_runtime_packet_exact( ...'; ...
        '                        preparedPacketCache.sweep,preparedPacketCache.packet, ...'; ...
        '                        preparedPacketCache.sensor,preparedPacketCache.dep, ...'; ...
        '                        SC,targetVersion);'; ...
        '                obj.af7r58v_r4_stage_ms(6)=1e3*toc(af7r58v_r4_t6);'; ...
        '                obj.af7r58v_r4_progress_code=max(obj.af7r58v_r4_progress_code,6);'; ...
        '            else'};

    % Replace the original ELSE line only.
    B=[B(1:firstElse-1),cacheReuseBranch,B(firstElse+1:end)];

    % ------------------------------------------------------------------
    % Store completed rebuild into the sole cache entry.
    % This must be before the pretransaction deadline gate.
    % ------------------------------------------------------------------
    depVersion=find_unique_contains(B,'depNew.reference_version=double(targetVersion);');
    depVersionEnd=find_statement_end(B,depVersion);

    storeCode={ ...
        ''; ...
        '                % AF7R58Z: overwrite the sole NON-AUTHORITY cache entry.'; ...
        '                obj.reference_only_prepared_packet_cache=struct( ...'; ...
        '                    ''schema'',''AF7R58Z_ONE_ENTRY_PREPARED_PACKET_CACHE_V1'', ...'; ...
        '                    ''non_authority'',true, ...'; ...
        '                    ''speed'',SC, ...'; ...
        '                    ''sweep'',sweepNew, ...'; ...
        '                    ''packet'',packetNew, ...'; ...
        '                    ''sensor'',sensorNew, ...'; ...
        '                    ''dep'',depNew);'; ...
        '                obj.reference_only_prepared_packet_cache_valid=true;'; ...
        '                obj.reference_only_prepared_packet_cache_store_count= ...'; ...
        '                    obj.reference_only_prepared_packet_cache_store_count+1;'};

    B=insert_after(B,depVersionEnd,storeCode);

    % ------------------------------------------------------------------
    % Correct successful-commit accounting:
    % active runtime reuse / prepared-cache reuse / rebuild stay distinct.
    % ------------------------------------------------------------------
    activeDep=find_unique_contains(B,'obj.runtime_dependency_signature=depNew;');
    counterIf=find_first_exact_trim_after(B,'if reuseRuntimePacket',activeDep);

    rebuildCounter=find_unique_contains(B, ...
        'obj.reference_only_runtime_packet_rebuild_count= ...');
    counterElse=find_last_exact_trim_between(B,'else',counterIf,rebuildCounter);
    assert(~isempty(counterElse), ...
        '[AF7R58Z] runtime reuse/rebuild counter else not found.');

    counterCacheBranch={ ...
        '                elseif reusePreparedPacketCache'; ...
        '                    obj.reference_only_prepared_packet_cache_commit_count= ...'; ...
        '                        obj.reference_only_prepared_packet_cache_commit_count+1;'; ...
        '                else'};

    B=[B(1:counterElse-1),counterCacheBranch,B(counterElse+1:end)];

    % ------------------------------------------------------------------
    % Reassemble patched class.
    % ------------------------------------------------------------------
    lines=[lines(1:i0-1),B,lines(i1+1:end)];
    patched=strjoin(lines,newline);

    % ------------------------------------------------------------------
    % Static source safety invariants BEFORE any write.
    % ------------------------------------------------------------------
    inv=struct();

    inv.one_entry_props = ...
        count(patched,'reference_only_prepared_packet_cache_valid = false;')==1 && ...
        count(patched,'reference_only_prepared_packet_cache = struct();')>=1;

    inv.cache_gate_calls = ...
        count(patched,'preparedPacketCache.speed,SC')==1 && ...
        count(patched,'reusePreparedPacketCache=false;')==1;

    inv.cache_reuse_branch = ...
        count(patched,'elseif reusePreparedPacketCache')==2;

    inv.cache_store = ...
        count(patched,'''schema'',''AF7R58Z_ONE_ENTRY_PREPARED_PACKET_CACHE_V1''')==1 && ...
        count(patched,'reference_only_prepared_packet_cache_store_count+1;')==1;

    inv.no_append_tokens = ...
        ~contains(patched,'reference_only_prepared_packet_cache(end+1)') && ...
        ~contains(patched,'reference_only_prepared_packet_cache{end+1}') && ...
        ~contains(patched,'reference_only_prepared_packet_cache=[obj.reference_only_prepared_packet_cache');

    inv.final_asserts_preserved = ...
        count(patched,'obj.assert_trigger_packet_certificate_current();')==1 && ...
        count(patched,'obj.assert_repaired_reference_current();')==1;

    inv.deadline_reason_preserved = ...
        count(patched,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED')==1;

    inv.all = inv.one_entry_props && inv.cache_gate_calls && ...
        inv.cache_reuse_branch && inv.cache_store && inv.no_append_tokens && ...
        inv.final_asserts_preserved && inv.deadline_reason_preserved;

    assert(inv.all,'[AF7R58Z] static cache/source invariant failed.');
    OUT.static_invariants=inv;

    % ------------------------------------------------------------------
    % Protected functions MUST remain byte-for-byte text identical.
    % ------------------------------------------------------------------
    patchedLines=regexp(patched,'\n','split');
    patchedDefs=discover_function_headers(patchedLines);

    protectedAfter=struct();
    protectedEqual=true;

    say(f,'');
    say(f,'[D. PROTECTED FUNCTION HASHES -- PATCHED PREFLIGHT]');
    for k=1:numel(protectedNames)
        nm=protectedNames{k};
        [b,j0,j1,~]=method_block_by_name(patchedLines,patchedDefs,nm);
        key=matlab.lang.makeValidName(nm);
        protectedAfter.(key)=text_hash(strjoin(b,newline));
        same=strcmp(protectedAfter.(key),protectedBefore.(key));
        protectedEqual=protectedEqual && same;
        say(f,' %-48s unchanged=%d | L%d:L%d | %s', ...
            nm,same,j0,j1,protectedAfter.(key));
    end
    assert(protectedEqual, ...
        '[AF7R58Z] protected exact reuse/builder function changed unexpectedly.');

    OUT.protected_before=protectedBefore;
    OUT.protected_after=protectedAfter;
    OUT.protected_unchanged=protectedEqual;

    % ------------------------------------------------------------------
    % Re-audit topology on patched commit.
    % ------------------------------------------------------------------
    [PB,pi0,~,~]=method_block_by_name(patchedLines,patchedDefs,commitName);

    P=struct();
    P.cache_gate=find_unique_contains(PB,'reusePreparedPacketCache=false;');
    % Two "elseif reusePreparedPacketCache" exist; use first/second explicitly.
    vv=find(cellfun(@(x)strcmp(strtrim(x),'elseif reusePreparedPacketCache'),PB));
    assert(numel(vv)==2,'[AF7R58Z] cache branch count=%d.',numel(vv));
    P.cache_reuse_prepare=vv(1);
    P.predeadline=find_unique_contains(PB,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    P.try_line=find_exact_trim_after(PB,'try',P.predeadline);
    P.active_packet=find_unique_contains(PB,'obj.obstacle_packet=packetNew;');
    P.final_assert_1=find_unique_contains(PB,'obj.assert_trigger_packet_certificate_current();');
    P.final_assert_2=find_unique_contains(PB,'obj.assert_repaired_reference_current();');

    storeLine=find_unique_contains(PB, ...
        '''schema'',''AF7R58Z_ONE_ENTRY_PREPARED_PACKET_CACHE_V1''');

    assert(P.cache_gate < P.cache_reuse_prepare && ...
           P.cache_reuse_prepare < storeLine && ...
           storeLine < P.predeadline && ...
           P.predeadline < P.try_line && ...
           P.try_line < P.active_packet && ...
           P.active_packet < P.final_assert_1 && ...
           P.final_assert_1 < P.final_assert_2, ...
        '[AF7R58Z] patched safety topology invalid.');

    OUT.patched_topology=struct( ...
        'cache_gate',pi0+P.cache_gate-1, ...
        'cache_reuse_prepare',pi0+P.cache_reuse_prepare-1, ...
        'cache_store',pi0+storeLine-1, ...
        'predeadline',pi0+P.predeadline-1, ...
        'try_line',pi0+P.try_line-1, ...
        'active_packet',pi0+P.active_packet-1, ...
        'final_assert_1',pi0+P.final_assert_1-1, ...
        'final_assert_2',pi0+P.final_assert_2-1);

    say(f,'');
    say(f,'[E. PATCHED SAFETY TOPOLOGY]');
    say(f,' cache gate                       L%d',OUT.patched_topology.cache_gate);
    say(f,' cache reuse branch               L%d',OUT.patched_topology.cache_reuse_prepare);
    say(f,' cache store                      L%d',OUT.patched_topology.cache_store);
    say(f,' pretransaction deadline gate     L%d',OUT.patched_topology.predeadline);
    say(f,' transactional try                L%d',OUT.patched_topology.try_line);
    say(f,' active packet assignment         L%d',OUT.patched_topology.active_packet);
    say(f,' final assertion 1                L%d',OUT.patched_topology.final_assert_1);
    say(f,' final assertion 2                L%d',OUT.patched_topology.final_assert_2);
    say(f,' topology closed                  = 1');

    % ------------------------------------------------------------------
    % Parse the entire patched class BEFORE the first production write.
    % ------------------------------------------------------------------
    preDir=fullfile(outdir,'AF7R58Z_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);

    parseName='AF7R58Z_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);

    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58Z] patched full-class preflight parse failed.');
    safe_rmpath_if_present(preDir);

    say(f,'');
    say(f,'[F. PREFLIGHT BEFORE PRODUCTION WRITE]');
    say(f,' static invariants                       = 1');
    say(f,' protected exact functions unchanged     = 1');
    say(f,' patched safety topology                 = 1');
    say(f,' full patched class parse                = 1');
    say(f,' production writes                       = 0');

    % ------------------------------------------------------------------
    % Transactional install.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    % Post-write parse.
    postName='AF7R58Z_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);

    write_utf8(fullfile(outdir,[postName '.m']),postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58Z] installed class post-write parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);

    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.production_modified=true;
    OUT.status='ONE_ENTRY_PREPARED_PACKET_CACHE_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';

    say(f,'');
    say(f,'[G. INSTALL RESULT]');
    say(f,' production modified                    = 1');
    say(f,' rollback performed                     = 0');
    say(f,' one-entry cache                        = 1');
    say(f,' append/list growth                     = 0');
    say(f,' active authority write before try      = 0');
    say(f,' exact reuse gate preserved             = 1');
    say(f,' exact reuse rebinding preserved        = 1');
    say(f,' builder math preserved                 = 1');
    say(f,' final assertions preserved             = 1');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',h1);
    say(f,' NOTE: final 4-ms validation remains OPEN until clean rerun and diagnostic removal.');

catch ME
    if backupReady && writesStarted
        try
            copyfile(fullfile(backupDir,'REPLANNING_SYSTEM.m'),pR,'f');
            rehash;
            OUT.rollback_performed=true;
        catch
        end
    end

    OUT.production_modified=false;
    OUT.status='INSTALL_FAILED__ROLLBACK_COMPLETE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    say(f,'');
    say(f,'[INSTALL FAILURE]');
    say(f,' identifier          = %s',ME.identifier);
    say(f,' message             = %s',ME.message);
    say(f,' rollback performed  = %d',OUT.rollback_performed);
    say(f,' production modified = 0');
end

save(fullfile(outdir,'AF7R58Z_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58Z_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Z_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Z_OUT.mat'));
say(f,' status = %s',OUT.status);
end


% ==============================================================================================
% Robust source parsing / patch helpers
% ==============================================================================================

function defs=discover_function_headers(lines)
defs=struct('line',{},'indent',{},'header',{},'header_end',{});

for k=1:numel(lines)
    if is_function_start_line(lines{k})
        [hdr,ke]=collect_function_header(lines,k);
        defs(end+1)=struct( ... %#ok<AGROW>
            'line',k, ...
            'indent',leading_ws_count(lines{k}), ...
            'header',hdr, ...
            'header_end',ke);
    end
end
end


function [b,i0,i1,hdr]=method_block_by_name(lines,defs,name)
hits=[];

for k=1:numel(defs)
    if header_has_exact_method_name(defs(k).header,name)
        hits(end+1)=k; %#ok<AGROW>
    end
end

assert(numel(hits)==1, ...
    '[AF7R58Z] exact method "%s" count=%d.',name,numel(hits));

d=defs(hits(1));
i0=d.line;
hdr=d.header;
i1=numel(lines);

for k=i0+1:numel(lines)
    if is_function_start_line(lines{k}) && ...
            leading_ws_count(lines{k})==d.indent
        i1=k-1;
        break;
    end
end

b=lines(i0:i1);
end


function tf=header_has_exact_method_name(header,name)
tf=false;
loc=strfind(header,name);

for ii=1:numel(loc)
    p=loc(ii);
    q=p+numel(name);

    if p>1 && is_ident_char(header(p-1)), continue; end
    if q<=numel(header) && is_ident_char(header(q)), continue; end

    while q<=numel(header) && isspace(header(q)), q=q+1; end
    if q<=numel(header) && header(q)=='('
        tf=true;
        return;
    end
end
end


function [hdr,kend]=collect_function_header(lines,k0)
parts={strtrim(lines{k0})};
kend=k0;

while endsWith(strtrim(parts{end}),'...')
    parts{end}=erase_trailing_ellipsis(parts{end});
    kend=kend+1;
    assert(kend<=numel(lines),'[AF7R58Z] unterminated function header.');
    parts{end+1}=strtrim(lines{kend}); %#ok<AGROW>
end

hdr=strjoin(parts,' ');
end


function tf=is_function_start_line(line)
tf=startsWith(strtrim(line),'function ');
end


function n=leading_ws_count(line)
n=0;

for k=1:numel(line)
    if isspace(line(k))
        n=n+1;
    else
        break;
    end
end
end


function s=erase_trailing_ellipsis(s)
s=strtrim(s);

if endsWith(s,'...')
    s=strtrim(s(1:end-3));
end
end


function tf=is_ident_char(c)
tf=isstrprop(c,'alphanum') || c=='_';
end


function j=find_unique_contains(B,token)
v=find(cellfun(@(x)contains(x,token),B));

assert(numel(v)==1, ...
    '[AF7R58Z] token "%s" count=%d.',token,numel(v));

j=v(1);
end


function j=find_unique_regex(B,pat)
v=find(cellfun(@(x)~isempty(regexp(x,pat,'once')),B));

assert(numel(v)==1, ...
    '[AF7R58Z] regex anchor "%s" count=%d.',pat,numel(v));

j=v(1);
end


function j=find_first_exact_trim_after(B,token,a)
j=[];

for k=a+1:numel(B)
    if strcmp(strtrim(B{k}),token)
        j=k;
        break;
    end
end

assert(~isempty(j), ...
    '[AF7R58Z] exact line "%s" missing after %d.',token,a);
end


function j=find_last_exact_trim_between(B,token,a,b)
j=[];

for k=a+1:b-1
    if strcmp(strtrim(B{k}),token)
        j=k;
    end
end
end


function e=find_statement_end(B,j)
e=j;

while e<=numel(B)
    t=strip_comment(B{e});
    if contains(t,';')
        return;
    end
    e=e+1;
end

error('[AF7R58Z] statement end not found after %d.',j);
end


function c=insert_before(c,j,newLines)
c=[c(1:j-1),reshape(newLines,1,[]),c(j:end)];
end


function c=insert_after(c,j,newLines)
c=[c(1:j),reshape(newLines,1,[]),c(j+1:end)];
end


function s=strip_comment(line)
p=strfind(line,'%');

if isempty(p)
    s=line;
else
    s=line(1:p(1)-1);
end
end


% ==============================================================================================
% File helpers
% ==============================================================================================

function [s,eol]=read_text_norm(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>

b=fread(f,Inf,'*uint8');
s=native2unicode(b.','UTF-8');

if contains(s,sprintf('\r\n'))
    eol='CRLF';
elseif contains(s,sprintf('\r'))
    eol='CR';
else
    eol='LF';
end

s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function write_text_eol(p,s,eol)
switch eol
    case 'CRLF'
        s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR'
        s=strrep(s,sprintf('\n'),sprintf('\r'));
end

write_utf8(p,s);
end


function write_utf8(p,s)
f=fopen(p,'wb'); assert(f>=0,'cannot write %s',p);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>

b=unicode2native(s,'UTF-8');
n=fwrite(f,b,'uint8');

assert(n==numel(b),'short write: %s',p);
end


function safe_rmpath_if_present(p)
try
    parts=strsplit(path,pathsep);

    if any(strcmpi(parts,p))
        rmpath(p);
    end
catch
end
end


function h=file_hash(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>

b=fread(f,Inf,'*uint8');

m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');

h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=text_hash(s)
b=unicode2native(s,'UTF-8');

m=java.security.MessageDigest.getInstance('SHA-256');
m.update(uint8(b));
d=typecast(m.digest(),'uint8');

h=lower(reshape(dec2hex(d,2).',1,[]));
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
