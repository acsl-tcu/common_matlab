function OUT = AF7R58X_ONEFILE_REBUILD_REUSE_FORENSIC_AUDIT(replanner)
% AF7R58X
% Read-only source/live forensic audit after AF7R58W.
%
% Purpose:
%   1) Explain why every measured commit preparation took rebuild branch.
%   2) Prove whether failed pre-transaction preparation is discarded.
%   3) Determine whether a bounded NON-AUTHORITATIVE prepared-packet cache is
%      structurally possible without weakening any safety/certificate logic.
%   4) Separate this algorithmic issue from diagnostic timing overhead.
%
% This file NEVER edits production source or live state.
%
% Exact source authority:
%   cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79
%
% RETURN:
%   AF7R58X_REPORT.txt
%   AF7R58X_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58X_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58X] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58X | FAILED-PREPARATION REBUILD / REUSE FORENSIC AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');

OUT=struct();
OUT.schema='AF7R58X_REBUILD_REUSE_FORENSIC_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58X] REPLANNING_SYSTEM.m missing.');

    h=file_hash(pR);
    expected='cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79';

    say(f,'');
    say(f,'[A. EXACT CURRENT SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(h,expected),h);
    assert(strcmp(h,expected), ...
        '[AF7R58X] source is not AF7R58V-R4 production.');

    txt=normalize_newlines(fileread(pR));
    lines=regexp(txt,'\n','split');

    specs={ ...
        'commit','function [ok,reason] = try_commit_reference_only_proposal'; ...
        'canReuse','function tf = can_reuse_reference_only_runtime_packet_exact'; ...
        'reuse','function [sweepNew,packetNew,sensorNew,depNew] = reuse_reference_only_runtime_packet_exact'; ...
        'sweep','function sweep = build_trigger_sweep_for_target'; ...
        'packet','function packet = prepare_obstacle_packet_fast_target'; ...
        'sensor','function cert = build_sensor_horizon_for_target'};

    blocks=struct();
    ranges=struct();

    say(f,'');
    say(f,'[B. EXACT FUNCTION SOURCE INVENTORY]');
    for k=1:size(specs,1)
        key=specs{k,1};
        token=specs{k,2};
        [b,i0,i1]=method_block(lines,token);
        blocks.(key)=strjoin(b,newline);
        ranges.(key)=[i0 i1];
        say(f,' %-10s source L%d:L%d | SHA=%s', ...
            key,i0,i1,text_hash(blocks.(key)));
    end
    OUT.function_ranges=ranges;

    C=blocks.commit;
    R=blocks.canReuse;
    U=blocks.reuse;
    S=blocks.sweep;
    P=blocks.packet;
    H=blocks.sensor;

    % ------------------------------------------------------------------
    % Current commit ordering / failure semantics.
    % ------------------------------------------------------------------
    pos=struct();
    pos.reuse_test=must_pos(C,'can_reuse_reference_only_runtime_packet_exact');
    pos.reuse_op=must_pos(C,'reuse_reference_only_runtime_packet_exact');
    pos.rebuild_sweep=must_pos(C,'build_trigger_sweep_for_target');
    pos.rebuild_packet=must_pos(C,'prepare_obstacle_packet_fast_target');
    pos.rebuild_sensor=must_pos(C,'build_sensor_horizon_for_target');
    pos.predeadline=must_pos(C,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    pos.try=must_pos(C,newline + '            try');
    pos.active_sweep=must_pos(C,'obj.trigger_sweep_inflation=sweepNew;');
    pos.active_packet=must_pos(C,'obj.obstacle_packet=packetNew;');
    pos.active_sensor=must_pos(C,'obj.sensor_horizon_certificate=sensorNew;');

    Q=struct();
    Q.reuse_test_before_rebuild = pos.reuse_test < pos.rebuild_sweep;
    Q.rebuild_before_predeadline = ...
        pos.rebuild_sweep < pos.predeadline && ...
        pos.rebuild_packet < pos.predeadline && ...
        pos.rebuild_sensor < pos.predeadline;
    Q.predeadline_before_transaction = pos.predeadline < pos.try;
    Q.active_assignment_only_after_transaction = ...
        pos.try < pos.active_sweep && pos.try < pos.active_packet && pos.try < pos.active_sensor;
    Q.early_deadline_return = ...
        contains(C,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED') && ...
        contains(C,'return;');

    % Failed preparation is local if no prepared-new assignment reaches obj
    % before the transactional try.
    preTry=C(1:pos.try-1);
    Q.no_pretry_active_packet_write = ...
        ~contains(preTry,'obj.trigger_sweep_inflation=sweepNew;') && ...
        ~contains(preTry,'obj.obstacle_packet=packetNew;') && ...
        ~contains(preTry,'obj.sensor_horizon_certificate=sensorNew;');

    say(f,'');
    say(f,'[C. FAILED PREPARATION LIFECYCLE]');
    print_bool(f,'reuse predicate evaluated before rebuild',Q.reuse_test_before_rebuild);
    print_bool(f,'all rebuild work occurs before predeadline gate',Q.rebuild_before_predeadline);
    print_bool(f,'predeadline reject occurs before transactional try',Q.predeadline_before_transaction);
    print_bool(f,'active packet assignment occurs only after try',Q.active_assignment_only_after_transaction);
    print_bool(f,'predeadline branch has ordinary return',Q.early_deadline_return);
    print_bool(f,'no prepared-new packet is persisted before try',Q.no_pretry_active_packet_write);

    Q.failed_preparation_discarded = ...
        Q.rebuild_before_predeadline && ...
        Q.predeadline_before_transaction && ...
        Q.no_pretry_active_packet_write;

    print_bool(f,'failed predeadline preparation is discarded',Q.failed_preparation_discarded);

    % ------------------------------------------------------------------
    % Reuse predicate theorem surface.
    % ------------------------------------------------------------------
    K=struct();
    K.policy_reference_only = contains(R,'AVOIDANCE_COMMIT_POLICY') && contains(R,'REFERENCE_ONLY');
    K.requires_old_new_certified = ...
        contains(R,'oldSC') && contains(R,'newSC') && contains(R,'certified');
    K.speed_payload_checked = contains(R,'payload_speed_UB_m_s');
    K.speed_uav_checked = contains(R,'uav_speed_UB_m_s');
    K.exact_speed_equality = contains(R,'isequal') && ...
        K.speed_payload_checked && K.speed_uav_checked;
    K.old_sweep_checked = contains(R,'oldSweep');
    K.old_packet_checked = contains(R,'oldPacket');
    K.old_sensor_checked = contains(R,'oldSensor');
    K.old_dependency_checked = contains(R,'oldDep');
    K.target_version_not_input = ~contains(first_line(R),'targetVersion');
    K.predicate_is_readonly = isempty(regexp(R,'obj\.[A-Za-z]\w*\s*=', 'once'));

    say(f,'');
    say(f,'[D. EXACT REUSE-PREDICATE STRUCTURE]');
    print_bool(f,'REFERENCE_ONLY policy required',K.policy_reference_only);
    print_bool(f,'old/new speed certificates certified',K.requires_old_new_certified);
    print_bool(f,'payload speed bound checked',K.speed_payload_checked);
    print_bool(f,'UAV speed bound checked',K.speed_uav_checked);
    print_bool(f,'exact speed equality uses isequal',K.exact_speed_equality);
    print_bool(f,'old sweep checked',K.old_sweep_checked);
    print_bool(f,'old obstacle packet checked',K.old_packet_checked);
    print_bool(f,'old sensor certificate checked',K.old_sensor_checked);
    print_bool(f,'old dependency signature checked',K.old_dependency_checked);
    print_bool(f,'targetVersion absent from predicate inputs',K.target_version_not_input);
    print_bool(f,'predicate has no obj-property assignment',K.predicate_is_readonly);

    % ------------------------------------------------------------------
    % Exact reuse operation: verify version rebinding, not stale authority.
    % ------------------------------------------------------------------
    V=struct();
    V.accepts_target_version = contains(first_line(U),'targetVersion');
    V.rebinds_reference_version = contains(U,'reference_version');
    V.rebinds_reference_mode = contains(U,'reference_mode');
    V.uses_old_sweep = contains(U,'oldSweep');
    V.uses_old_packet = contains(U,'oldPacket');
    V.uses_old_sensor = contains(U,'oldSensor');
    V.uses_old_dep = contains(U,'oldDep');
    V.no_obj_assignment = isempty(regexp(U,'obj\.[A-Za-z]\w*\s*=', 'once'));

    say(f,'');
    say(f,'[E. REUSE OPERATION / VERSION REBINDING]');
    print_bool(f,'reuse operation accepts targetVersion',V.accepts_target_version);
    print_bool(f,'reference_version metadata is rebound',V.rebinds_reference_version);
    print_bool(f,'reference_mode metadata is handled',V.rebinds_reference_mode);
    print_bool(f,'old sweep consumed',V.uses_old_sweep);
    print_bool(f,'old packet consumed',V.uses_old_packet);
    print_bool(f,'old sensor consumed',V.uses_old_sensor);
    print_bool(f,'old dependency signature consumed',V.uses_old_dep);
    print_bool(f,'reuse operation has no obj-property assignment',V.no_obj_assignment);

    % ------------------------------------------------------------------
    % Builders should be side-effect-free w.r.t object state for safe
    % non-authoritative preparation caching.
    % ------------------------------------------------------------------
    BLD=struct();
    BLD.sweep_no_obj_assignment = isempty(regexp(S,'obj\.[A-Za-z]\w*\s*=', 'once'));
    BLD.packet_no_obj_assignment = isempty(regexp(P,'obj\.[A-Za-z]\w*\s*=', 'once'));
    BLD.sensor_no_obj_assignment = isempty(regexp(H,'obj\.[A-Za-z]\w*\s*=', 'once'));

    say(f,'');
    say(f,'[F. REBUILD BUILDER SIDE-EFFECT AUDIT]');
    print_bool(f,'trigger-sweep builder has no obj-property assignment',BLD.sweep_no_obj_assignment);
    print_bool(f,'obstacle-packet builder has no obj-property assignment',BLD.packet_no_obj_assignment);
    print_bool(f,'sensor-horizon builder has no obj-property assignment',BLD.sensor_no_obj_assignment);

    % ------------------------------------------------------------------
    % Search current source for an explicit non-authoritative prepared cache.
    % Avoid matching the diagnostic comments in this file; only production
    % source is searched.
    % ------------------------------------------------------------------
    cacheTokens={ ...
        'prepared_runtime_packet_cache', ...
        'reference_only_prepared_packet_cache', ...
        'precommit_runtime_packet_cache', ...
        'failed_preparation_cache'};
    cacheHits=zeros(size(cacheTokens));
    for k=1:numel(cacheTokens)
        cacheHits(k)=count(txt,cacheTokens{k});
    end

    CACHE=struct();
    CACHE.tokens=cacheTokens;
    CACHE.hits=cacheHits;
    CACHE.explicit_cache_present=any(cacheHits>0);

    say(f,'');
    say(f,'[G. EXISTING FAILED-PREPARATION CACHE]');
    for k=1:numel(cacheTokens)
        say(f,' %-44s hits=%d',cacheTokens{k},cacheHits(k));
    end
    print_bool(f,'explicit prepared-packet cache present',CACHE.explicit_cache_present);

    % ------------------------------------------------------------------
    % Optional live post-run evidence. Read only.
    % ------------------------------------------------------------------
    LIVE=struct();
    LIVE.available=false;
    if nargin<1 || isempty(replanner)
        [replanner,names]=find_live_replanner();
    else
        names={};
    end

    if ~isempty(replanner) && isa(replanner,'REPLANNING_SYSTEM')
        LIVE.available=true;
        LIVE.generator_request_count=double(replanner.generator_request_count);
        LIVE.deadline_fail_count=double(replanner.generator_deadline_fail_count);
        LIVE.reuse_count=read_num_prop(replanner,'reference_only_runtime_packet_reuse_count');
        LIVE.rebuild_count=read_num_prop(replanner,'reference_only_runtime_packet_rebuild_count');
        LIVE.fast_rebuild_count=read_num_prop(replanner,'reference_only_fast_packet_rebuild_count');
        LIVE.commit_count=double(replanner.replan_commit_count);
        LIVE.reference_version=double(replanner.active_reference_version);
        LIVE.reference_mode=char(replanner.active_reference_mode);

        say(f,'');
        say(f,'[H. LIVE POST-RUN COUNTERS]');
        say(f,' generator request count               = %.0f',LIVE.generator_request_count);
        say(f,' deadline fail count                   = %.0f',LIVE.deadline_fail_count);
        say(f,' runtime packet reuse count            = %.0f',LIVE.reuse_count);
        say(f,' runtime packet rebuild count          = %.0f',LIVE.rebuild_count);
        say(f,' fast packet rebuild count             = %.0f',LIVE.fast_rebuild_count);
        say(f,' successful replan commit count        = %.0f',LIVE.commit_count);
        say(f,' active reference mode/version         = %s / %.0f', ...
            LIVE.reference_mode,LIVE.reference_version);
    else
        LIVE.candidate_names=names;
        say(f,'');
        say(f,'[H. LIVE POST-RUN COUNTERS]');
        say(f,' live object unavailable; source theorem only.');
    end

    % ------------------------------------------------------------------
    % Conservative theorem conclusion.
    % We do NOT install a cache here. We only decide whether a bounded
    % one-entry, non-authoritative cache is a valid optimization candidate.
    % ------------------------------------------------------------------
    TH=struct();
    TH.failed_work_recomputed = ...
        Q.failed_preparation_discarded && ~CACHE.explicit_cache_present;

    TH.version_rebind_supported = ...
        K.target_version_not_input && ...
        V.accepts_target_version && ...
        V.rebinds_reference_version;

    TH.builders_side_effect_free = ...
        BLD.sweep_no_obj_assignment && ...
        BLD.packet_no_obj_assignment && ...
        BLD.sensor_no_obj_assignment;

    TH.exact_reuse_gate_available = ...
        K.requires_old_new_certified && ...
        K.exact_speed_equality && ...
        K.old_sweep_checked && ...
        K.old_packet_checked && ...
        K.old_sensor_checked && ...
        K.old_dependency_checked;

    TH.one_entry_cache_structurally_admissible = ...
        TH.failed_work_recomputed && ...
        TH.version_rebind_supported && ...
        TH.builders_side_effect_free && ...
        TH.exact_reuse_gate_available;

    % Full safety proof remains open until exact cache key / invalidation
    % is explicitly bound to every live dependency checked by production.
    TH.cache_safety_proof_closed=false;

    OUT.lifecycle=Q;
    OUT.reuse_predicate=K;
    OUT.reuse_operation=V;
    OUT.builders=BLD;
    OUT.cache_search=CACHE;
    OUT.live=LIVE;
    OUT.theorem=TH;

    say(f,'');
    say(f,'[I. CONSERVATIVE CONCLUSION]');
    print_bool(f,'failed preparation is recomputed after reject',TH.failed_work_recomputed);
    print_bool(f,'version metadata rebinding structurally supported',TH.version_rebind_supported);
    print_bool(f,'rebuild builders are side-effect-free',TH.builders_side_effect_free);
    print_bool(f,'exact production reuse gate is available',TH.exact_reuse_gate_available);
    print_bool(f,'bounded one-entry cache structurally admissible',TH.one_entry_cache_structurally_admissible);
    print_bool(f,'cache safety proof CLOSED now',TH.cache_safety_proof_closed);
    say(f,' NOTE: no cache is installed by AF7R58X.');
    say(f,' NOTE: next step, if structurally admissible, is an exact cache-key/invalidation proof.');
    say(f,' NOTE: diagnostic timing rings must later be removed/disabled before final 4-ms validation.');

    if TH.one_entry_cache_structurally_admissible
        OUT.status='FAILED_PREPARATION_RECOMPUTE_CLOSED__CACHE_KEY_PROOF_NEXT';
    elseif TH.failed_work_recomputed
        OUT.status='FAILED_PREPARATION_RECOMPUTE_CLOSED__CACHE_ADMISSIBILITY_OPEN';
    else
        OUT.status='FAILED_PREPARATION_LIFECYCLE_OPEN';
    end

    say(f,' status = %s',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    say(f,'');
    say(f,'[AUDIT ERROR]');
    say(f,' identifier = %s',ME.identifier);
    say(f,' message    = %s',ME.message);
end

save(fullfile(outdir,'AF7R58X_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58X_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58X_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58X_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function [b,i0,i1]=method_block(lines,token)
v=find(cellfun(@(x)contains(x,token),lines));
assert(numel(v)==1,'[AF7R58X] method token "%s" count=%d.',token,numel(v));

i0=v(1);
i1=numel(lines);
for k=i0+1:numel(lines)
    if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
        i1=k-1;
        break;
    end
end
b=lines(i0:i1);
end


function p=must_pos(s,token)
p=strfind(s,token);
assert(numel(p)==1,'[AF7R58X] token "%s" count=%d.',token,numel(p));
p=p(1);
end


function s=first_line(block)
v=regexp(block,'\n','split');
s=v{1};
end


function x=read_num_prop(obj,name)
if isprop(obj,name)
    x=double(obj.(name));
else
    x=NaN;
end
end


function [obj,names]=find_live_replanner()
obj=[];
names={};

W=evalin('base','whos');
for k=1:numel(W)
    try
        v=evalin('base',W(k).name);
    catch
        continue;
    end

    if isa(v,'REPLANNING_SYSTEM')
        names{end+1}=W(k).name; %#ok<AGROW>
    end
end

if numel(names)==1
    obj=evalin('base',names{1});
end
end


function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
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


function print_bool(f,label,v)
say(f,' %-58s = %d',label,logical(v));
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
