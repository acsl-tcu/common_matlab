function OUT = AF7R58X_R3_ONEFILE_REBUILD_REUSE_FORENSIC_AUDIT(replanner)
% AF7R58X-R3
% Robust read-only rebuild/reuse forensic audit.
%
% R3 intentionally avoids regexp-based method-name matching.
% Method definitions are discovered from actual class "function" header lines,
% continuation headers are joined, exact MATLAB identifier boundaries are
% checked manually, and method end is the next function at the SAME indent.
%
% No production source or live state is modified.
%
% Exact source authority:
%   cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79
%
% RETURN:
%   AF7R58X_R3_REPORT.txt
%   AF7R58X_R3_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58X_R3_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58X-R3] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58X-R3 | FAILED-PREPARATION REBUILD / REUSE FORENSIC AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1');
say(f,'[PARSER] regexpMethodNameMatching=0 | exactIdentifierBoundary=1 | sameIndentMethodBoundary=1');

OUT=struct();
OUT.schema='AF7R58X_R3_REBUILD_REUSE_FORENSIC_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58X-R3] REPLANNING_SYSTEM.m missing.');

    h=file_hash(pR);
    expected='cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79';

    say(f,'');
    say(f,'[A. EXACT CURRENT SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(h,expected),h);
    assert(strcmp(h,expected), ...
        '[AF7R58X-R3] source is not AF7R58V-R4 production.');

    txt=normalize_newlines(fileread(pR));
    lines=regexp(txt,'\n','split');

    % Discover all class method headers once, using string parsing only.
    defs=discover_function_headers(lines);
    OUT.discovered_function_count=numel(defs);

    say(f,'');
    say(f,'[B. FUNCTION DISCOVERY SANITY]');
    say(f,' discovered function headers           = %d',numel(defs));

    near={};
    for k=1:numel(defs)
        if contains(defs(k).header,'runtime_packet') || ...
                contains(defs(k).header,'try_commit_reference_only_proposal')
            near{end+1}=sprintf('L%d | %s',defs(k).line,strtrim(defs(k).header)); %#ok<AGROW>
        end
    end
    say(f,' relevant discovered headers           = %d',numel(near));
    for k=1:numel(near)
        say(f,'   %s',near{k});
    end
    OUT.relevant_discovered_headers=near;

    methodNames={ ...
        'commit','try_commit_reference_only_proposal'; ...
        'canReuse','can_reuse_reference_only_runtime_packet_exact'; ...
        'reuse','reuse_reference_only_runtime_packet_exact'; ...
        'sweep','build_trigger_sweep_for_target'; ...
        'packet','prepare_obstacle_packet_fast_target'; ...
        'sensor','build_sensor_horizon_for_target'};

    blocks=struct();
    blockLines=struct();
    ranges=struct();
    headers=struct();

    say(f,'');
    say(f,'[C. EXACT FUNCTION SOURCE INVENTORY]');
    for k=1:size(methodNames,1)
        key=methodNames{k,1};
        name=methodNames{k,2};

        [b,i0,i1,hdr]=method_block_by_name(lines,defs,name);
        blockLines.(key)=b;
        blocks.(key)=strjoin(b,newline);
        ranges.(key)=[i0 i1];
        headers.(key)=hdr;

        say(f,' %-10s source L%d:L%d | SHA=%s', ...
            key,i0,i1,text_hash(blocks.(key)));
        say(f,'   header = %s',strtrim(hdr));
    end

    OUT.function_ranges=ranges;
    OUT.function_headers=headers;

    C=blockLines.commit;
    R=blocks.canReuse;
    U=blocks.reuse;
    S=blockLines.sweep;
    P=blockLines.packet;
    H=blockLines.sensor;

    % ------------------------------------------------------------------
    % Commit ordering and failed-preparation lifecycle.
    % ------------------------------------------------------------------
    pos=struct();
    pos.reuse_test=find_unique_contains(C, ...
        'obj.can_reuse_reference_only_runtime_packet_exact(');
    pos.reuse_op=find_unique_contains(C, ...
        'obj.reuse_reference_only_runtime_packet_exact(');
    pos.rebuild_sweep=find_unique_contains(C, ...
        'obj.build_trigger_sweep_for_target(');
    pos.rebuild_packet=find_unique_contains(C, ...
        'obj.prepare_obstacle_packet_fast_target(');
    pos.rebuild_sensor=find_unique_contains(C, ...
        'obj.build_sensor_horizon_for_target(');
    pos.predeadline=find_unique_contains(C, ...
        'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    pos.try_line=find_exact_trim_after(C,'try',pos.predeadline);
    pos.active_sweep=find_unique_contains(C, ...
        'obj.trigger_sweep_inflation=sweepNew;');
    pos.active_packet=find_unique_contains(C, ...
        'obj.obstacle_packet=packetNew;');
    pos.active_sensor=find_unique_contains(C, ...
        'obj.sensor_horizon_certificate=sensorNew;');

    Q=struct();
    Q.reuse_test_before_rebuild=pos.reuse_test<pos.rebuild_sweep;
    Q.rebuild_before_predeadline= ...
        pos.rebuild_sweep<pos.predeadline && ...
        pos.rebuild_packet<pos.predeadline && ...
        pos.rebuild_sensor<pos.predeadline;
    Q.predeadline_before_transaction=pos.predeadline<pos.try_line;
    Q.active_assignment_only_after_transaction= ...
        pos.try_line<pos.active_sweep && ...
        pos.try_line<pos.active_packet && ...
        pos.try_line<pos.active_sensor;

    Q.early_deadline_return= ...
        has_exact_trim_between(C,'return;',pos.predeadline,pos.try_line);

    preTry=C(1:pos.try_line-1);
    Q.no_pretry_active_packet_write= ...
        ~any_contains(preTry,'obj.trigger_sweep_inflation=sweepNew;') && ...
        ~any_contains(preTry,'obj.obstacle_packet=packetNew;') && ...
        ~any_contains(preTry,'obj.sensor_horizon_certificate=sensorNew;');

    Q.failed_preparation_discarded= ...
        Q.rebuild_before_predeadline && ...
        Q.predeadline_before_transaction && ...
        Q.early_deadline_return && ...
        Q.no_pretry_active_packet_write;

    say(f,'');
    say(f,'[D. FAILED PREPARATION LIFECYCLE]');
    print_bool(f,'reuse predicate evaluated before rebuild',Q.reuse_test_before_rebuild);
    print_bool(f,'all rebuild work occurs before predeadline gate',Q.rebuild_before_predeadline);
    print_bool(f,'predeadline reject occurs before transactional try',Q.predeadline_before_transaction);
    print_bool(f,'active packet assignment occurs only after try',Q.active_assignment_only_after_transaction);
    print_bool(f,'predeadline reject branch has ordinary return',Q.early_deadline_return);
    print_bool(f,'no prepared-new packet persisted before try',Q.no_pretry_active_packet_write);
    print_bool(f,'failed predeadline preparation is discarded',Q.failed_preparation_discarded);

    % ------------------------------------------------------------------
    % Exact reuse predicate structure.
    % ------------------------------------------------------------------
    K=struct();
    K.policy_reference_only=contains(R,'AVOIDANCE_COMMIT_POLICY') && ...
        contains(R,'REFERENCE_ONLY');
    K.requires_old_new_certified=contains(R,'oldSC') && ...
        contains(R,'newSC') && contains(R,'certified');
    K.speed_payload_checked=contains(R,'payload_speed_UB_m_s');
    K.speed_uav_checked=contains(R,'uav_speed_UB_m_s');
    K.exact_speed_equality=contains(R,'isequal') && ...
        K.speed_payload_checked && K.speed_uav_checked;
    K.old_sweep_checked=contains(R,'oldSweep');
    K.old_packet_checked=contains(R,'oldPacket');
    K.old_sensor_checked=contains(R,'oldSensor');
    K.old_dependency_checked=contains(R,'oldDep');
    K.target_version_not_input=~contains(headers.canReuse,'targetVersion');
    K.predicate_is_readonly=~has_obj_property_assignment(blockLines.canReuse);

    say(f,'');
    say(f,'[E. EXACT REUSE-PREDICATE STRUCTURE]');
    print_bool(f,'REFERENCE_ONLY policy required',K.policy_reference_only);
    print_bool(f,'old/new speed certificates certified',K.requires_old_new_certified);
    print_bool(f,'payload speed bound checked',K.speed_payload_checked);
    print_bool(f,'UAV speed bound checked',K.speed_uav_checked);
    print_bool(f,'exact speed equality uses isequal',K.exact_speed_equality);
    print_bool(f,'old sweep checked',K.old_sweep_checked);
    print_bool(f,'old obstacle packet checked',K.old_packet_checked);
    print_bool(f,'old sensor certificate checked',K.old_sensor_checked);
    print_bool(f,'old dependency signature checked',K.old_dependency_checked);
    print_bool(f,'targetVersion absent from predicate header',K.target_version_not_input);
    print_bool(f,'predicate has no obj-property assignment',K.predicate_is_readonly);

    % ------------------------------------------------------------------
    % Exact reuse operation and version rebinding.
    % ------------------------------------------------------------------
    V=struct();
    V.accepts_target_version=contains(headers.reuse,'targetVersion') || ...
        contains(U,'targetVersion');
    V.rebinds_reference_version=contains(U,'reference_version');
    V.rebinds_reference_mode=contains(U,'reference_mode');
    V.uses_old_sweep=contains(U,'oldSweep');
    V.uses_old_packet=contains(U,'oldPacket');
    V.uses_old_sensor=contains(U,'oldSensor');
    V.uses_old_dep=contains(U,'oldDep');
    V.no_obj_assignment=~has_obj_property_assignment(blockLines.reuse);

    say(f,'');
    say(f,'[F. REUSE OPERATION / VERSION REBINDING]');
    print_bool(f,'reuse operation accepts targetVersion',V.accepts_target_version);
    print_bool(f,'reference_version metadata is rebound',V.rebinds_reference_version);
    print_bool(f,'reference_mode metadata is handled',V.rebinds_reference_mode);
    print_bool(f,'old sweep consumed',V.uses_old_sweep);
    print_bool(f,'old packet consumed',V.uses_old_packet);
    print_bool(f,'old sensor consumed',V.uses_old_sensor);
    print_bool(f,'old dependency signature consumed',V.uses_old_dep);
    print_bool(f,'reuse operation has no obj-property assignment',V.no_obj_assignment);

    % ------------------------------------------------------------------
    % Builder side effects. This parser only flags direct LHS assignment:
    % obj.<identifier> = ... ; it does NOT treat ==, <=, >=, ~= as writes.
    % ------------------------------------------------------------------
    BLD=struct();
    BLD.sweep_no_obj_assignment=~has_obj_property_assignment(S);
    BLD.packet_no_obj_assignment=~has_obj_property_assignment(P);
    BLD.sensor_no_obj_assignment=~has_obj_property_assignment(H);

    say(f,'');
    say(f,'[G. REBUILD BUILDER SIDE-EFFECT AUDIT]');
    print_bool(f,'trigger-sweep builder has no obj-property assignment',BLD.sweep_no_obj_assignment);
    print_bool(f,'obstacle-packet builder has no obj-property assignment',BLD.packet_no_obj_assignment);
    print_bool(f,'sensor-horizon builder has no obj-property assignment',BLD.sensor_no_obj_assignment);

    % ------------------------------------------------------------------
    % Existing failed-preparation cache search.
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
    say(f,'[H. EXISTING FAILED-PREPARATION CACHE]');
    for k=1:numel(cacheTokens)
        say(f,' %-44s hits=%d',cacheTokens{k},cacheHits(k));
    end
    print_bool(f,'explicit prepared-packet cache present',CACHE.explicit_cache_present);

    % ------------------------------------------------------------------
    % Optional live evidence, read-only.
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
        LIVE.reuse_count=read_num_prop(replanner, ...
            'reference_only_runtime_packet_reuse_count');
        LIVE.rebuild_count=read_num_prop(replanner, ...
            'reference_only_runtime_packet_rebuild_count');
        LIVE.fast_rebuild_count=read_num_prop(replanner, ...
            'reference_only_fast_packet_rebuild_count');
        LIVE.commit_count=double(replanner.replan_commit_count);
        LIVE.reference_version=double(replanner.active_reference_version);
        LIVE.reference_mode=char(replanner.active_reference_mode);

        say(f,'');
        say(f,'[I. LIVE POST-RUN COUNTERS]');
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
        say(f,'[I. LIVE POST-RUN COUNTERS]');
        say(f,' live object unavailable; source theorem only.');
    end

    % ------------------------------------------------------------------
    % Conservative theorem conclusion.
    % ------------------------------------------------------------------
    TH=struct();
    TH.failed_work_recomputed= ...
        Q.failed_preparation_discarded && ~CACHE.explicit_cache_present;

    TH.version_rebind_supported= ...
        K.target_version_not_input && ...
        V.accepts_target_version && ...
        V.rebinds_reference_version;

    TH.builders_side_effect_free= ...
        BLD.sweep_no_obj_assignment && ...
        BLD.packet_no_obj_assignment && ...
        BLD.sensor_no_obj_assignment;

    TH.exact_reuse_gate_available= ...
        K.requires_old_new_certified && ...
        K.exact_speed_equality && ...
        K.old_sweep_checked && ...
        K.old_packet_checked && ...
        K.old_sensor_checked && ...
        K.old_dependency_checked;

    TH.one_entry_cache_structurally_admissible= ...
        TH.failed_work_recomputed && ...
        TH.version_rebind_supported && ...
        TH.builders_side_effect_free && ...
        TH.exact_reuse_gate_available;

    % Not closed here by design: exact cache key/invalidation still needs
    % a separate proof against all live dependencies.
    TH.cache_safety_proof_closed=false;

    OUT.lifecycle=Q;
    OUT.reuse_predicate=K;
    OUT.reuse_operation=V;
    OUT.builders=BLD;
    OUT.cache_search=CACHE;
    OUT.live=LIVE;
    OUT.theorem=TH;

    say(f,'');
    say(f,'[J. CONSERVATIVE CONCLUSION]');
    print_bool(f,'failed preparation is recomputed after reject',TH.failed_work_recomputed);
    print_bool(f,'version metadata rebinding structurally supported',TH.version_rebind_supported);
    print_bool(f,'rebuild builders are side-effect-free',TH.builders_side_effect_free);
    print_bool(f,'exact production reuse gate is available',TH.exact_reuse_gate_available);
    print_bool(f,'bounded one-entry cache structurally admissible',TH.one_entry_cache_structurally_admissible);
    print_bool(f,'cache safety proof CLOSED now',TH.cache_safety_proof_closed);
    say(f,' NOTE: AF7R58X-R3 installs nothing.');
    say(f,' NOTE: diagnostic timing rings must be removed/disabled before final 4-ms validation.');

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

save(fullfile(outdir,'AF7R58X_R3_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58X_R3_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58X_R3_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58X_R3_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function defs=discover_function_headers(lines)
defs=struct('line',{},'indent',{},'header',{},'header_end',{});

for k=1:numel(lines)
    if is_function_start_line(lines{k})
        [hdr,ke]=collect_function_header(lines,k);
        d=struct();
        d.line=k;
        d.indent=leading_ws_count(lines{k});
        d.header=hdr;
        d.header_end=ke;
        defs(end+1)=d; %#ok<AGROW>
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

if numel(hits)~=1
    near={};
    for k=1:numel(defs)
        if contains(defs(k).header,name) || ...
                contains(defs(k).header,erase(name,'can_'))
            near{end+1}=sprintf('L%d | %s', ...
                defs(k).line,strtrim(defs(k).header)); %#ok<AGROW>
        end
    end

    nearText='';
    if ~isempty(near)
        nearText=strjoin(near,' || ');
    end

    error('[AF7R58X-R3] exact method "%s" count=%d. nearby=%s', ...
        name,numel(hits),nearText);
end

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
if isempty(loc), return; end

for ii=1:numel(loc)
    p=loc(ii);
    q=p+numel(name);

    if p>1
        c=header(p-1);
        if is_ident_char(c)
            continue;
        end
    end

    while q<=numel(header) && isspace(header(q))
        q=q+1;
    end

    if q<=numel(header) && header(q)=='('
        tf=true;
        return;
    end
end
end


function tf=is_ident_char(c)
tf=isstrprop(c,'alphanum') || c=='_';
end


function tf=is_function_start_line(line)
t=strtrim(line);
tf=startsWith(t,'function ');
end


function [hdr,kend]=collect_function_header(lines,k0)
parts={strtrim(lines{k0})};
kend=k0;

while endsWith(strtrim(parts{end}),'...')
    parts{end}=erase_trailing_ellipsis(parts{end});
    kend=kend+1;
    assert(kend<=numel(lines), ...
        '[AF7R58X-R3] unterminated function header continuation.');
    parts{end+1}=strtrim(lines{kend}); %#ok<AGROW>
end

hdr=strjoin(parts,' ');
end


function s=erase_trailing_ellipsis(s)
s=strtrim(s);
if endsWith(s,'...')
    s=strtrim(s(1:end-3));
end
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


function j=find_unique_contains(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1, ...
    '[AF7R58X-R3] token "%s" count=%d.',token,numel(v));
j=v(1);
end


function j=find_exact_trim_after(B,token,a)
j=[];
for k=a+1:numel(B)
    if strcmp(strtrim(B{k}),token)
        j=k;
        break;
    end
end
assert(~isempty(j), ...
    '[AF7R58X-R3] exact line "%s" missing after %d.',token,a);
end


function tf=has_exact_trim_between(B,token,a,b)
tf=false;
lo=max(1,a);
hi=min(numel(B),b);

for k=lo:hi
    if strcmp(strtrim(B{k}),token)
        tf=true;
        return;
    end
end
end


function tf=any_contains(B,token)
tf=any(cellfun(@(x)contains(x,token),B));
end


function tf=has_obj_property_assignment(B)
% Detect direct left-hand property assignment:
%   obj.foo = ...
% but not obj.foo == ..., <=, >=, ~=, or localVar = obj.foo.
tf=false;

for k=1:numel(B)
    line=B{k};

    % Ignore pure comment lines.
    t=strtrim(line);
    if isempty(t) || startsWith(t,'%')
        continue;
    end

    p=1;
    while p<=numel(line)-3
        rel=strfind(line(p:end),'obj.');
        if isempty(rel), break; end
        s=p+rel(1)-1;
        j=s+4;

        % Parse identifier after obj.
        if j>numel(line) || ...
                ~(isstrprop(line(j),'alpha') || line(j)=='_')
            p=s+4;
            continue;
        end

        j=j+1;
        while j<=numel(line) && is_ident_char(line(j))
            j=j+1;
        end

        while j<=numel(line) && isspace(line(j))
            j=j+1;
        end

        if j<=numel(line) && line(j)=='='
            nextIsEq=(j<numel(line) && line(j+1)=='=');
            prevIsComparator=(j>1 && any(line(j-1)=='=<>~'));
            if ~nextIsEq && ~prevIsComparator
                tf=true;
                return;
            end
        end

        p=s+4;
    end
end
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
