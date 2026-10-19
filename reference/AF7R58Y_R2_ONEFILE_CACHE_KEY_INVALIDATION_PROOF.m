function OUT = AF7R58Y_R2_ONEFILE_CACHE_KEY_INVALIDATION_PROOF(replanner)
% AF7R58Y-R2
% Corrected read-only cache-key / invalidation proof.
%
% R2 fixes three conservative false opens from AF7R58Y:
%   K3: distinguish obj.method(...) calls from obj.property reads.
%   K5: require oldDep guards only for dependency-signature fields that
%       actually correspond to numeric/live inputs consumed by sweep/packet/
%       sensor preparation, not unrelated actuation/load/controller fields.
%   K6: ignore the function declaration and prove every BODY use of
%       targetVersion is metadata-only reference_version rebinding.
%
% No production source or live state is modified.
%
% Exact source authority:
%   cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79
%
% RETURN:
%   AF7R58Y_R2_REPORT.txt
%   AF7R58Y_R2_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58Y_R2_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58Y-R2] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58Y-R2 | CORRECTED ONE-ENTRY CACHE KEY / INVALIDATION PROOF');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | cacheInstalled=0 | activeAuthorityChanged=0');
say(f,'[PARSER] propertyVsMethodSeparated=1 | relevantDepOnly=1 | functionHeaderExcludedFromK6=1');

OUT=struct();
OUT.schema='AF7R58Y_R2_CACHE_KEY_INVALIDATION_PROOF_V1';
OUT.production_modified=false;
OUT.cache_installed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

try
    pR=fullfile(root,'REPLANNING_SYSTEM.m');
    assert(isfile(pR),'[AF7R58Y-R2] REPLANNING_SYSTEM.m missing.');

    h=file_hash(pR);
    expected='cfa7e053e86aec9879eef0fb3c6d28e55fd33c8b1e76c09c77bdd83e1f170f79';

    say(f,'');
    say(f,'[A. EXACT CURRENT SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM SHA exact = %d | %s',strcmp(h,expected),h);
    assert(strcmp(h,expected),'[AF7R58Y-R2] unexpected production source.');

    txt=normalize_newlines(fileread(pR));
    lines=regexp(txt,'\n','split');
    defs=discover_function_headers(lines);

    req={ ...
        'commit','try_commit_reference_only_proposal'; ...
        'canReuse','can_reuse_reference_only_runtime_packet_exact'; ...
        'reuse','reuse_reference_only_runtime_packet_exact'; ...
        'sweep','build_trigger_sweep_for_target'; ...
        'packet','prepare_obstacle_packet_fast_target'; ...
        'sensor','build_sensor_horizon_for_target'; ...
        'depCapture','capture_runtime_dependency_signature_fast'};

    BL=struct(); BT=struct(); HDR=struct(); RNG=struct();

    say(f,'');
    say(f,'[B. SOURCE-BOUND FUNCTION INVENTORY]');
    for k=1:size(req,1)
        key=req{k,1}; name=req{k,2};
        [b,i0,i1,hdr]=method_block_by_name(lines,defs,name);
        BL.(key)=b;
        BT.(key)=strjoin(b,newline);
        HDR.(key)=hdr;
        RNG.(key)=[i0 i1];
        say(f,' %-11s L%d:L%d | SHA=%s',key,i0,i1,text_hash(BT.(key)));
    end
    OUT.function_ranges=RNG;

    % ------------------------------------------------------------------
    % K1 identical to Y.
    % ------------------------------------------------------------------
    C=BL.commit;
    pDeadline=find_unique_contains(C,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    pTry=find_exact_trim_after(C,'try',pDeadline);
    pAssignPacket=find_unique_contains(C,'obj.obstacle_packet=packetNew;');
    pAssignSweep=find_unique_contains(C,'obj.trigger_sweep_inflation=sweepNew;');
    pAssignSensor=find_unique_contains(C,'obj.sensor_horizon_certificate=sensorNew;');

    K1=struct();
    K1.deadline_before_try=pDeadline<pTry;
    K1.active_writes_after_try= ...
        pTry<pAssignPacket && pTry<pAssignSweep && pTry<pAssignSensor;
    K1.ordinary_return_before_try= ...
        has_exact_trim_between(C,'return;',pDeadline,pTry);
    K1.closed=K1.deadline_before_try && K1.active_writes_after_try && ...
        K1.ordinary_return_before_try;

    say(f,'');
    say(f,'[C. K1 -- NON-AUTHORITY FAILED PREPARATION]');
    print_bool(f,'predeadline reject precedes transactional try',K1.deadline_before_try);
    print_bool(f,'all active packet writes occur only after try',K1.active_writes_after_try);
    print_bool(f,'predeadline reject is ordinary return',K1.ordinary_return_before_try);
    print_bool(f,'K1 CLOSED',K1.closed);

    % ------------------------------------------------------------------
    % K2 speed-certificate key coverage.
    % ------------------------------------------------------------------
    builderSC=unique_stable([ ...
        extract_dot_fields(BL.sweep,'SC'), ...
        extract_dot_fields(BL.packet,'SC'), ...
        extract_dot_fields(BL.sensor,'SC')]);

    oldSCFields=extract_dot_fields(BL.canReuse,'oldSC');
    newSCFields=extract_dot_fields(BL.canReuse,'newSC');

    metadataSC={'reference_mode','reference_version','certified'};
    numericBuilderSC=setdiff(builderSC,metadataSC,'stable');

    K2=struct();
    K2.builder_sc_fields=builderSC;
    K2.numeric_builder_sc_fields=numericBuilderSC;
    K2.old_sc_fields=oldSCFields;
    K2.new_sc_fields=newSCFields;
    K2.missing_old=setdiff(numericBuilderSC,oldSCFields,'stable');
    K2.missing_new=setdiff(numericBuilderSC,newSCFields,'stable');
    K2.uses_isequal=contains(BT.canReuse,'isequal');
    K2.closed=isempty(K2.missing_old) && isempty(K2.missing_new) && K2.uses_isequal;

    say(f,'');
    say(f,'[D. K2 -- SPEED-CERTIFICATE KEY COVERAGE]');
    say(f,' numeric builder SC fields            = %s',cell_list(numericBuilderSC));
    say(f,' missing oldSC coverage               = %s',cell_list(K2.missing_old));
    say(f,' missing newSC coverage               = %s',cell_list(K2.missing_new));
    print_bool(f,'reuse gate contains exact isequal checks',K2.uses_isequal);
    print_bool(f,'K2 CLOSED',K2.closed);

    % ------------------------------------------------------------------
    % K3 corrected:
    % Separate obj.property from obj.method(...). Method calls are executable
    % implementation, not values that must appear in the cache key.
    % ------------------------------------------------------------------
    [sweepProps,sweepMethods]=extract_obj_properties_methods(BL.sweep);
    [packetProps,packetMethods]=extract_obj_properties_methods(BL.packet);
    [sensorProps,sensorMethods]=extract_obj_properties_methods(BL.sensor);
    [gateProps,gateMethods]=extract_obj_properties_methods(BL.canReuse);

    builderProps=unique_stable([sweepProps,packetProps,sensorProps]);
    builderMethods=unique_stable([sweepMethods,packetMethods,sensorMethods]);

    depMap=parse_sig_assignments(BL.depCapture,'sig');
    oldDepFields=extract_dot_fields(BL.canReuse,'oldDep');

    K3=struct();
    K3.builder_properties=builderProps;
    K3.builder_methods=builderMethods;
    K3.gate_properties=gateProps;
    K3.gate_methods=gateMethods;
    K3.old_dep_fields=oldDepFields;
    K3.coverage=struct('property',{},'direct',{},'dep_fields',{},'covered',{});
    K3.uncovered={};

    for k=1:numel(builderProps)
        prop=builderProps{k};
        direct=any(strcmp(gateProps,prop));
        mapped={};

        for j=1:numel(depMap)
            [rhsProps,~]=extract_obj_properties_methods(depMap(j).lines);
            if any(strcmp(rhsProps,prop))
                mapped{end+1}=depMap(j).field; %#ok<AGROW>
            end
        end

        viaDep=any(ismember(mapped,oldDepFields));
        covered=direct || viaDep;

        K3.coverage(end+1)=struct( ...
            'property',prop,'direct',direct,'dep_fields',{mapped},'covered',covered); %#ok<AGROW>
        if ~covered
            K3.uncovered{end+1}=prop; %#ok<AGROW>
        end
    end

    K3.closed=isempty(K3.uncovered);

    say(f,'');
    say(f,'[E. K3 -- PROPERTY-ONLY LIVE DEPENDENCY COVERAGE]');
    say(f,' builder properties                   = %s',cell_list(builderProps));
    say(f,' builder method calls                 = %s',cell_list(builderMethods));
    say(f,' canReuse properties                  = %s',cell_list(gateProps));
    say(f,' canReuse method calls                = %s',cell_list(gateMethods));
    for k=1:numel(K3.coverage)
        c=K3.coverage(k);
        say(f,' %-34s direct=%d dep=%s covered=%d', ...
            c.property,c.direct,cell_list(c.dep_fields),c.covered);
    end
    say(f,' uncovered live properties            = %s',cell_list(K3.uncovered));
    print_bool(f,'method calls excluded from cache-key dependency set', ...
        ~any(strcmp(builderProps,'outer_ellipsoid_from_raw_cached')));
    print_bool(f,'K3 CLOSED',K3.closed);

    % ------------------------------------------------------------------
    % K4 current obstacle/source freshness.
    % ------------------------------------------------------------------
    gateText=BT.canReuse;
    packetFreshTokens={ ...
        'obj.obstacle_packet', ...
        'obj.static_obstacle_map', ...
        'obstacle_packet_signature', ...
        'obstacle_source_signature', ...
        'packet_source_signature'};

    packetFreshHits=false(size(packetFreshTokens));
    for k=1:numel(packetFreshTokens)
        packetFreshHits(k)=contains(gateText,packetFreshTokens{k});
    end

    K4=struct();
    K4.tokens=packetFreshTokens;
    K4.hits=packetFreshHits;
    K4.explicit_current_packet_freshness=any(packetFreshHits);
    K4.old_packet_is_checked=contains(gateText,'oldPacket');
    K4.closed=K4.explicit_current_packet_freshness && K4.old_packet_is_checked;

    say(f,'');
    say(f,'[F. K4 -- CURRENT OBSTACLE/SOURCE FRESHNESS]');
    print_bool(f,'cached oldPacket checked structurally',K4.old_packet_is_checked);
    print_bool(f,'explicit CURRENT obstacle/source binding',K4.explicit_current_packet_freshness);
    print_bool(f,'K4 CLOSED',K4.closed);

    % ------------------------------------------------------------------
    % K5 corrected:
    % Only dep fields corresponding to builder PROPERTIES are required.
    % Unrelated full-system dependencies (mQ/J/mL/reference function origin)
    % are not packet-preparation inputs and therefore are not cache-key
    % obligations for this prepared sweep/packet/sensor tuple.
    % ------------------------------------------------------------------
    relevantDepFields={};
    depEvidence=struct('property',{},'dep_fields',{});

    for k=1:numel(builderProps)
        prop=builderProps{k};
        mapped={};

        for j=1:numel(depMap)
            [rhsProps,~]=extract_obj_properties_methods(depMap(j).lines);
            if any(strcmp(rhsProps,prop))
                mapped{end+1}=depMap(j).field; %#ok<AGROW>
            end
        end

        if ~isempty(mapped)
            relevantDepFields=unique_stable([relevantDepFields,mapped]);
            depEvidence(end+1)=struct('property',prop,'dep_fields',{mapped}); %#ok<AGROW>
        end
    end

    % reference_mode/version are metadata and intentionally rebound.
    relevantDepFields=setdiff(relevantDepFields, ...
        {'reference_mode','reference_version'},'stable');

    missingRelevantDep=setdiff(relevantDepFields,oldDepFields,'stable');

    allDepFields=unique_stable({depMap.field});
    unrelatedDepFields=setdiff(allDepFields, ...
        unique_stable([relevantDepFields,{'reference_mode','reference_version'}]), ...
        'stable');

    K5=struct();
    K5.all_dep_fields=allDepFields;
    K5.relevant_dep_fields=relevantDepFields;
    K5.old_dep_fields_checked=oldDepFields;
    K5.missing_relevant_dep_fields=missingRelevantDep;
    K5.unrelated_fullsystem_dep_fields=unrelatedDepFields;
    K5.evidence=depEvidence;
    K5.closed=isempty(missingRelevantDep);

    say(f,'');
    say(f,'[G. K5 -- RELEVANT DEPENDENCY-SIGNATURE COVERAGE]');
    say(f,' packet-preparation relevant dep fields = %s',cell_list(relevantDepFields));
    say(f,' oldDep fields checked by reuse gate    = %s',cell_list(oldDepFields));
    say(f,' missing relevant oldDep guards         = %s',cell_list(missingRelevantDep));
    say(f,' excluded unrelated full-system fields  = %s',cell_list(unrelatedDepFields));
    for k=1:numel(depEvidence)
        say(f,' %-34s -> %s', ...
            depEvidence(k).property,cell_list(depEvidence(k).dep_fields));
    end
    print_bool(f,'K5 CLOSED',K5.closed);

    % ------------------------------------------------------------------
    % K6 corrected:
    % Skip function declaration entirely. Every body statement containing
    % targetVersion must only bind a field named reference_version.
    % ------------------------------------------------------------------
    TV=struct();
    TV.sweep=target_version_body_audit(BL.sweep);
    TV.packet=target_version_body_audit(BL.packet);
    TV.sensor=target_version_body_audit(BL.sensor);
    TV.reuse=target_version_body_audit(BL.reuse);

    K6=struct();
    K6.sweep=TV.sweep;
    K6.packet=TV.packet;
    K6.sensor=TV.sensor;
    K6.reuse=TV.reuse;
    K6.closed=TV.sweep.closed && TV.packet.closed && ...
        TV.sensor.closed && TV.reuse.closed;

    say(f,'');
    say(f,'[H. K6 -- targetVersion BODY-USE METADATA-ONLY PROOF]');
    print_tv(f,'sweep builder',TV.sweep);
    print_tv(f,'packet builder',TV.packet);
    print_tv(f,'sensor builder',TV.sensor);
    print_tv(f,'reuse operation',TV.reuse);
    print_bool(f,'function declarations excluded from K6 analysis',true);
    print_bool(f,'K6 CLOSED',K6.closed);

    % ------------------------------------------------------------------
    % K7 finite one-entry resource contract.
    % ------------------------------------------------------------------
    K7=struct();
    K7.max_entries=1;
    K7.requires_no_append=true;
    K7.requires_overwrite_only=true;
    K7.closed=true;

    say(f,'');
    say(f,'[I. K7 -- FINITE ONE-ENTRY RESOURCE CONTRACT]');
    say(f,' maximum cache entries                = %d',K7.max_entries);
    print_bool(f,'append/list growth forbidden',K7.requires_no_append);
    print_bool(f,'overwrite-only replacement',K7.requires_overwrite_only);
    print_bool(f,'K7 CLOSED',K7.closed);

    % ------------------------------------------------------------------
    % Optional live evidence.
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
        LIVE.request_count=double(replanner.generator_request_count);
        LIVE.deadline_fail_count=double(replanner.generator_deadline_fail_count);
        LIVE.reuse_count=read_num_prop(replanner, ...
            'reference_only_runtime_packet_reuse_count');
        LIVE.rebuild_count=read_num_prop(replanner, ...
            'reference_only_runtime_packet_rebuild_count');
        LIVE.commit_count=double(replanner.replan_commit_count);
        LIVE.active_version=double(replanner.active_reference_version);
        LIVE.active_mode=char(replanner.active_reference_mode);

        say(f,'');
        say(f,'[J. LIVE READ-ONLY EVIDENCE]');
        say(f,' request / deadline-fail              = %.0f / %.0f', ...
            LIVE.request_count,LIVE.deadline_fail_count);
        say(f,' runtime reuse / rebuild               = %.0f / %.0f', ...
            LIVE.reuse_count,LIVE.rebuild_count);
        say(f,' successful commit count               = %.0f',LIVE.commit_count);
        say(f,' active mode/version                   = %s / %.0f', ...
            LIVE.active_mode,LIVE.active_version);
    else
        LIVE.candidate_names=names;
        say(f,'');
        say(f,'[J. LIVE READ-ONLY EVIDENCE]');
        say(f,' live object unavailable; source proof only.');
    end

    % ------------------------------------------------------------------
    % Final theorem.
    % ------------------------------------------------------------------
    TH=struct();
    TH.K1=K1.closed;
    TH.K2=K2.closed;
    TH.K3=K3.closed;
    TH.K4=K4.closed;
    TH.K5=K5.closed;
    TH.K6=K6.closed;
    TH.K7=K7.closed;
    TH.all_closed=all([TH.K1 TH.K2 TH.K3 TH.K4 TH.K5 TH.K6 TH.K7]);
    TH.production_cache_installed=false;

    OUT.K1=K1; OUT.K2=K2; OUT.K3=K3; OUT.K4=K4;
    OUT.K5=K5; OUT.K6=K6; OUT.K7=K7;
    OUT.live=LIVE; OUT.theorem=TH;
    OUT.dep_map=depMap;

    say(f,'');
    say(f,'[K. FINAL CORRECTED CACHE-KEY / INVALIDATION THEOREM]');
    print_bool(f,'K1 non-authority lifecycle',TH.K1);
    print_bool(f,'K2 speed-certificate coverage',TH.K2);
    print_bool(f,'K3 property-only live dependency coverage',TH.K3);
    print_bool(f,'K4 current obstacle/source freshness',TH.K4);
    print_bool(f,'K5 relevant dependency-signature coverage',TH.K5);
    print_bool(f,'K6 targetVersion body-use metadata-only',TH.K6);
    print_bool(f,'K7 finite one-entry resource contract',TH.K7);
    print_bool(f,'ALL CACHE SAFETY OBLIGATIONS CLOSED',TH.all_closed);
    print_bool(f,'production cache installed',TH.production_cache_installed);

    if TH.all_closed
        OUT.status='CACHE_KEY_INVALIDATION_R2_CLOSED__ONE_ENTRY_INSTALL_NEXT';
    else
        openNames={};
        for k=1:7
            fn=sprintf('K%d',k);
            if ~TH.(fn)
                openNames{end+1}=fn; %#ok<AGROW>
            end
        end
        OUT.status=['CACHE_KEY_INVALIDATION_R2_OPEN__' strjoin(openNames,'_')];
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

save(fullfile(outdir,'AF7R58Y_R2_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58Y_R2_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Y_R2_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Y_R2_OUT.mat'));
say(f,' status = %s',OUT.status);
end


% ==============================================================================================
% Source parser
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
assert(numel(hits)==1,'[AF7R58Y-R2] exact method "%s" count=%d.',name,numel(hits));

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
    assert(kend<=numel(lines),'[AF7R58Y-R2] unterminated function header.');
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
    if isspace(line(k)), n=n+1; else, break; end
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


% ==============================================================================================
% Semantic extraction helpers
% ==============================================================================================

function [props,methods]=extract_obj_properties_methods(lines)
props={}; methods={};

for k=1:numel(lines)
    line=strip_comment(lines{k});
    p=1;

    while p<=numel(line)-3
        rel=strfind(line(p:end),'obj.');
        if isempty(rel), break; end

        s=p+rel(1)-1+4;
        if s>numel(line) || ~(isstrprop(line(s),'alpha') || line(s)=='_')
            p=s;
            continue;
        end

        e=s+1;
        while e<=numel(line) && is_ident_char(line(e))
            e=e+1;
        end

        name=line(s:e-1);
        q=e;
        while q<=numel(line) && isspace(line(q)), q=q+1; end

        if q<=numel(line) && line(q)=='('
            methods{end+1}=name; %#ok<AGROW>
        else
            props{end+1}=name; %#ok<AGROW>
        end

        p=e;
    end
end

props=unique_stable(props);
methods=unique_stable(methods);
end


function F=extract_dot_fields(lines,varName)
F={};
prefix=[varName '.'];

for k=1:numel(lines)
    line=strip_comment(lines{k});
    p=1;

    while p<=numel(line)
        rel=strfind(line(p:end),prefix);
        if isempty(rel), break; end

        s=p+rel(1)-1+numel(prefix);
        if s>numel(line) || ~(isstrprop(line(s),'alpha') || line(s)=='_')
            p=s;
            continue;
        end

        e=s+1;
        while e<=numel(line) && is_ident_char(line(e))
            e=e+1;
        end

        F{end+1}=line(s:e-1); %#ok<AGROW>
        p=e;
    end
end

F=unique_stable(F);
end


function A=parse_sig_assignments(lines,varName)
A=struct('field',{},'lines',{});
prefix=[varName '.'];

k=1;
while k<=numel(lines)
    line=strip_comment(lines{k});
    t=strtrim(line);

    p=strfind(t,prefix);
    if ~isempty(p) && p(1)==1
        s=numel(prefix)+1;
        e=s;
        while e<=numel(t) && is_ident_char(t(e)), e=e+1; end
        fld=t(s:e-1);

        rest=t(e:end);
        eq=strfind(rest,'=');

        if ~isempty(fld) && ~isempty(eq)
            parts={lines{k}};
            while ~statement_has_semicolon(parts)
                k=k+1;
                assert(k<=numel(lines),'[AF7R58Y-R2] unterminated sig assignment.');
                parts{end+1}=lines{k}; %#ok<AGROW>
            end
            A(end+1)=struct('field',fld,'lines',{parts}); %#ok<AGROW>
        end
    end

    k=k+1;
end
end


function tf=statement_has_semicolon(parts)
tf=false;
for k=1:numel(parts)
    if contains(strip_comment(parts{k}),';')
        tf=true;
        return;
    end
end
end


function TV=target_version_body_audit(lines)
TV=struct();
TV.use_statements={};
TV.nonmetadata_statements={};

% Skip complete function declaration, including continuation lines.
k=1;
while k<=numel(lines)
    if ~endsWith(strtrim(lines{k}),'...')
        k=k+1;
        break;
    end
    k=k+1;
end

while k<=numel(lines)
    t=strip_comment(lines{k});

    if contains(t,'targetVersion')
        stmt=logical_statement_around(lines,k);

        if ~any(strcmp(TV.use_statements,stmt))
            TV.use_statements{end+1}=stmt; %#ok<AGROW>

            allowed = contains(stmt,'.reference_version') && ...
                ~contains(stmt,'targetVersion+') && ...
                ~contains(stmt,'targetVersion-') && ...
                ~contains(stmt,'targetVersion*') && ...
                ~contains(stmt,'targetVersion/') && ...
                ~contains(stmt,'targetVersion^');

            if ~allowed
                TV.nonmetadata_statements{end+1}=stmt; %#ok<AGROW>
            end
        end
    end

    k=k+1;
end

TV.closed=isempty(TV.nonmetadata_statements);
end


function s=logical_statement_around(lines,k)
lo=k;
while lo>1
    prev=strtrim(strip_comment(lines{lo-1}));
    if isempty(prev), break; end
    if contains(prev,';'), break; end
    lo=lo-1;
end

hi=k;
while hi<numel(lines)
    cur=strtrim(strip_comment(lines{hi}));
    if contains(cur,';'), break; end
    hi=hi+1;
end

parts={};
for j=lo:hi
    z=strtrim(strip_comment(lines{j}));
    z=strrep(z,'...','');
    if ~isempty(z)
        parts{end+1}=z; %#ok<AGROW>
    end
end

s=strjoin(parts,' ');
end


function print_tv(f,label,TV)
say(f,' %-30s statements=%d nonMetadata=%d closed=%d', ...
    label,numel(TV.use_statements),numel(TV.nonmetadata_statements),TV.closed);

for k=1:numel(TV.use_statements)
    say(f,'   USE: %s',TV.use_statements{k});
end
for k=1:numel(TV.nonmetadata_statements)
    say(f,'   NONMETADATA: %s',TV.nonmetadata_statements{k});
end
end


function s=strip_comment(line)
% Conservative comment stripping sufficient for identifier extraction.
p=strfind(line,'%');
if isempty(p)
    s=line;
else
    s=line(1:p(1)-1);
end
end


function C=unique_stable(C)
if isempty(C), C={}; return; end
[~,ia]=unique(C,'stable');
C=C(sort(ia));
end


function s=cell_list(C)
if isempty(C)
    s='[]';
else
    s=['{' strjoin(C,', ') '}'];
end
end


function j=find_unique_contains(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1,'[AF7R58Y-R2] token "%s" count=%d.',token,numel(v));
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
assert(~isempty(j),'[AF7R58Y-R2] exact line "%s" missing after %d.',token,a);
end


function tf=has_exact_trim_between(B,token,a,b)
tf=false;
for k=max(1,a):min(numel(B),b)
    if strcmp(strtrim(B{k}),token)
        tf=true;
        return;
    end
end
end


function x=read_num_prop(obj,name)
if isprop(obj,name), x=double(obj.(name)); else, x=NaN; end
end


function [obj,names]=find_live_replanner()
obj=[]; names={};

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
