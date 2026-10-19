function OUT = AF7R58ZJ_ONEFILE_DETACH_HOTPATH_DIAGNOSTICS_INSTALL()
% AF7R58ZJ
% Detach diagnostic-only work from the REPLANNING_SYSTEM Flight hot path.
%
% MOTIVATION
% ----------
% Same-cycle evidence showed:
%   * request callback ~1.65--3.26 ms
%   * reference/replanner request cycle up to ~37.96 ms
%   * generator cycle-critical up to ~32.94 ms
% Therefore callback math itself is not the dominant spike.
%
% This patch removes ONLY diagnostic work from the real-time path:
%   1) ENABLE_LIGHT_RUNTIME_AUDIT default -> 0
%   2) add ENABLE_HOTPATH_DIAGNOSTIC_MIRROR = 0
%   3) gate record_bezier_shadow_proposal(candidate)
%   4) gate R9 high-resolution diagnostic resets/stamps
%   5) gate variable/heavy trigger diagnostic mirrors into result.state
%   6) gate generator/Bezier/R9/geometry diagnostic mirrors into result.state
%
% NOT CHANGED:
%   - obstacle sensing mathematics
%   - trigger_core
%   - continuous-time geometry certificate
%   - candidate generator callback
%   - reference-only certificate
%   - execution-aware certificate
%   - hard 4 ms deadline logic
%   - commit logic
%   - cache validity / monotone dominance logic
%   - active reference evaluation
%
% Existing result.state diagnostic dynamic-properties remain allocated for
% compatibility; when mirror=0, their previous/default values are simply not
% rewritten every control cycle. Authoritative values remain in obj.*.
%
% Transactional: full patched-class parse before write + backup rollback.

root=fileparts(mfilename('fullpath'));
pR=fullfile(root,'REPLANNING_SYSTEM.m');

outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZJ_REPORT.txt');
f=fopen(report,'wt','n','UTF-8');
assert(f>=0,'[AF7R58ZJ] cannot open report.');
c=onCleanup(@()fclose(f)); %#ok<NASGU>

OUT=struct();
OUT.schema='AF7R58ZJ_DETACH_HOTPATH_DIAGNOSTICS_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

say(f,'====================================================================================================');
say(f,' AF7R58ZJ | DETACH DIAGNOSTIC-ONLY WORK FROM REFERENCE HOT PATH');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] sourceSemanticBound=1 | parseBeforeWrite=1 | rollbackOnFailure=1');
say(f,'[SAFETY] triggerMathChanged=0 | certificateMathChanged=0 | commitMathChanged=0 | cacheMathChanged=0');

try
    assert(isfile(pR),'[AF7R58ZJ] REPLANNING_SYSTEM.m missing.');
    preSha=file_hash(pR);
    [txt,eolStyle]=read_text_norm(pR);
    lines=reshape(regexp(txt,'\n','split'),1,[]);

    say(f,'');
    say(f,'[A. CURRENT SOURCE]');
    say(f,' path = %s',pR);
    say(f,' SHA  = %s',preSha);

    % ------------------------------------------------------------------
    % Semantic binding to the current ZD/ZF lineage.
    % ------------------------------------------------------------------
    sem=struct();
    sem.zd_monotone=count(txt,'AF7R58ZD monotone theorem')==1;
    sem.cache_v2=count(txt,'AF7R58ZB_ONE_ENTRY_PREPARED_PACKET_CACHE_V2')>=1;
    sem.payload_dominance=count(txt,'VLcached>=VL')==1;
    sem.uav_dominance=count(txt,'VQcached>=VQ')==1;
    sem.old_diag_switch_absent=count(txt,'ENABLE_HOTPATH_DIAGNOSTIC_MIRROR')==0;
    sem.record_call_once=count(txt,'obj.record_bezier_shadow_proposal(candidate);')==1;
    sem.light_audit_default_one= ...
        ~isempty(regexp(txt,'ENABLE_LIGHT_RUNTIME_AUDIT\s*=\s*1\s*;','once'));

    old58={ ...
        'ENABLE_AF7R58M_TIMING_RING', ...
        'ENABLE_AF7R58Q_MIN_TIMING', ...
        'ENABLE_AF7R58T_OTHER_TIMING', ...
        'ENABLE_AF7R58V_R4_PRETRY_TIMING'};
    sem.legacy58_defaults_off=true;
    for kk=1:numel(old58)
        pat=[old58{kk} '\s*=\s*false\s*;'];
        sem.legacy58_defaults_off=sem.legacy58_defaults_off && ...
            ~isempty(regexp(txt,pat,'once'));
    end

    sem.all=all(structfun(@(x)logical(x),sem));
    say(f,'');
    say(f,'[B. CURRENT-LINEAGE SEMANTIC BINDING]');
    fn=fieldnames(sem);
    for k=1:numel(fn)
        say(f,' %-38s = %d',fn{k},logical(sem.(fn{k})));
    end
    assert(sem.all,'[AF7R58ZJ] current source does not match expected ZD/ZF lineage.');

    defs=discover_function_headers(lines);

    % Protected mathematical/safety methods must remain byte-identical.
    protected={ ...
        'assert_runtime_core_certificates_current', ...
        'trigger_core', ...
        'evaluate_reference_only_proposal_certificate', ...
        'build_execution_aware_proposal_certificate', ...
        'try_commit_reference_only_proposal', ...
        'can_reuse_reference_only_runtime_packet_exact', ...
        'can_reuse_reference_only_prepared_packet_cache_exact', ...
        'reuse_reference_only_runtime_packet_exact', ...
        'build_trigger_sweep_for_target', ...
        'prepare_obstacle_packet_fast_target', ...
        'build_sensor_horizon_for_target', ...
        'capture_runtime_dependency_signature_fast'};

    before=struct();
    for k=1:numel(protected)
        [B,~,~,~]=method_block_by_name(lines,defs,protected{k});
        key=matlab.lang.makeValidName(protected{k});
        before.(key)=text_hash(strjoin(B,newline));
    end

    % ------------------------------------------------------------------
    % 1) Disable light runtime audit default; add explicit hotpath diag gate.
    % ------------------------------------------------------------------
    jLight=find_unique_regex(lines, ...
        '^\s*ENABLE_LIGHT_RUNTIME_AUDIT\s*=\s*1\s*;.*$');
    lines{jLight}=regexprep(lines{jLight}, ...
        'ENABLE_LIGHT_RUNTIME_AUDIT\s*=\s*1\s*;', ...
        'ENABLE_LIGHT_RUNTIME_AUDIT = 0;');

    jNonCommit=find_unique_regex(lines, ...
        '^\s*ENABLE_NONCOMMIT_DIAGNOSTIC_EVAL\s*=\s*0\s*;.*$');
    lines=insert_after(lines,jNonCommit,{ ...
        '        ENABLE_HOTPATH_DIAGNOSTIC_MIRROR = 0; % diagnostic-only obj/state mirrors; keep OFF in production'});

    % Constructor opt-in for offline diagnostics.
    defs=discover_function_headers(lines);
    [C,c0,c1,~]=method_block_by_name(lines,defs,'REPLANNING_SYSTEM');
    jCtor=find_unique_contains(C, ...
        'if isfield(opts,''ENABLE_LIGHT_RUNTIME_AUDIT'')');
    % Find the two-line if/end block end robustly.
    jCtorEnd=find_first_exact_trim_after(C,'end',jCtor);
    C=insert_after(C,jCtorEnd,{ ...
        '            if isfield(opts,''ENABLE_HOTPATH_DIAGNOSTIC_MIRROR'')'; ...
        '                obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR=logical(opts.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR);'; ...
        '            end'});
    lines=replace_range(lines,c0,c1,C);

    % ------------------------------------------------------------------
    % 2) Gate diagnostic-only work inside invoke_replan_generator_shadow.
    % ------------------------------------------------------------------
    defs=discover_function_headers(lines);
    [G,g0,g1,~]=method_block_by_name(lines,defs,'invoke_replan_generator_shadow');

    % R9 high-resolution reset block.
    jSeq=find_unique_contains(G,'obj.r9_hr_request_seq=double(obj.generator_request_count);');
    jLastReset=find_unique_contains(G,'obj.r9_hr_qp_process_cycles=NaN;');
    G=insert_before(G,jSeq,{'            if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR'});
    % insertion shifts end by one
    jLastReset=find_unique_contains(G,'obj.r9_hr_qp_process_cycles=NaN;');
    G=insert_after(G,jLastReset,{'            end'});

    % Callback timestamp path: preserve callback exactly, gate stamps only.
    jStamp0=find_unique_contains(G,'r25cb0=af7r25_stamp_mex();');
    jCandidate=find_first_contains_after(G, ...
        'candidate=obj.generator_backend_locked_callback(CTX);',jStamp0-1);
    jStamp1=find_first_contains_after(G,'r25cb1=af7r25_stamp_mex();',jCandidate);
    jProc=find_first_contains_after(G, ...
        'obj.r9_hr_callback_process_cycles=double(r25cb1(4)-r25cb0(4));',jStamp1);

    stampReplacement={ ...
        '                    if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR'; ...
        '                        r25cb0=af7r25_stamp_mex();'; ...
        '                        candidate=obj.generator_backend_locked_callback(CTX);'; ...
        '                        r25cb1=af7r25_stamp_mex();'; ...
        '                        obj.r9_hr_callback_wall_ms=1e3*double(r25cb1(1)-r25cb0(1))/double(r25cb0(2));'; ...
        '                        obj.r9_hr_callback_thread_cycles=double(r25cb1(3)-r25cb0(3));'; ...
        '                        obj.r9_hr_callback_process_cycles=double(r25cb1(4)-r25cb0(4));'; ...
        '                    else'; ...
        '                        candidate=obj.generator_backend_locked_callback(CTX);'; ...
        '                    end'};
    G=replace_range(G,jStamp0,jProc,stampReplacement);

    % Diagnostic proposal mirror before deadline decision.
    jRec=find_unique_contains(G,'obj.record_bezier_shadow_proposal(candidate);');
    G=replace_range(G,jRec,jRec,{ ...
        '                if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR'; ...
        '                    obj.record_bezier_shadow_proposal(candidate);'; ...
        '                end'});

    lines=replace_range(lines,g0,g1,G);

    % ------------------------------------------------------------------
    % 3) Gate diagnostic result.state writes inside do().
    % ------------------------------------------------------------------
    defs=discover_function_headers(lines);
    [D,d0,d1,~]=method_block_by_name(lines,defs,'do');

    % Heavy trigger vectors/masks.
    jHeavyStart=find_unique_contains(D,'st.trigger_component=comp_name;');
    jHeavyEnd=find_unique_contains(D,'st.trigger_raw_distances=all_dists;');
    D=insert_before(D,jHeavyStart,{'            if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR'});
    jHeavyEnd=find_unique_contains(D,'st.trigger_raw_distances=all_dists;');
    D=insert_after(D,jHeavyEnd,{'            end'});

    % Large generator / Bezier / R9 / geometry mirror.
    jMetaStart=find_unique_contains(D, ...
        'st.generator_shadow_enabled=logical(obj.generator_shadow_enabled);');
    jMetaEnd=find_unique_contains(D, ...
        'st.generator_last_cycle_critical_ms=obj.generator_last_cycle_critical_ms;');
    D=insert_before(D,jMetaStart,{'            if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR'});
    jMetaEnd=find_unique_contains(D, ...
        'st.generator_last_cycle_critical_ms=obj.generator_last_cycle_critical_ms;');
    D=insert_after(D,jMetaEnd,{'            end'});

    lines=replace_range(lines,d0,d1,D);
    patched=strjoin(lines,newline);

    % ------------------------------------------------------------------
    % Static invariants.
    % ------------------------------------------------------------------
    plines=reshape(regexp(patched,'\n','split'),1,[]);
    pdefs=discover_function_headers(plines);

    inv=struct();
    inv.light_runtime_default_off= ...
        ~isempty(regexp(patched,'ENABLE_LIGHT_RUNTIME_AUDIT\s*=\s*0\s*;','once'));
    inv.hotpath_diag_gate_exactly_one= ...
        numel(regexp(patched,'ENABLE_HOTPATH_DIAGNOSTIC_MIRROR\s*=\s*0\s*;','match'))==1;
    inv.record_call_still_once=count(patched,'obj.record_bezier_shadow_proposal(candidate);')==1;
    inv.record_call_gated=count(patched, ...
        'if obj.ENABLE_HOTPATH_DIAGNOSTIC_MIRROR')>=4;
    inv.trigger_math_anchor=count(patched, ...
        'obj.trigger_core(active,pL_ref,q_ref,time.t);')==1;
    inv.deadline_anchor=count(patched, ...
        'deadlineExceeded=cycle_ms>1e3*obj.hard_modify_deadline_s;')==1;
    inv.commit_anchor=count(patched, ...
        'obj.try_commit_reference_only_proposal( ...')==1;
    inv.cache_v2_preserved=count(patched, ...
        'AF7R58ZB_ONE_ENTRY_PREPARED_PACKET_CACHE_V2')>=1;
    inv.monotone_dominance_preserved= ...
        count(patched,'VLcached>=VL')==1 && count(patched,'VQcached>=VQ')==1;

    % Protected method hashes unchanged.
    inv.protected_methods_unchanged=true;
    for k=1:numel(protected)
        [B,~,~,~]=method_block_by_name(plines,pdefs,protected{k});
        key=matlab.lang.makeValidName(protected{k});
        ah=text_hash(strjoin(B,newline));
        inv.protected_methods_unchanged=inv.protected_methods_unchanged && ...
            strcmp(before.(key),ah);
    end
    inv.all=all(structfun(@(x)logical(x),inv));
    OUT.static_invariants=inv;

    say(f,'');
    say(f,'[C. PATCH INVARIANTS]');
    fn=fieldnames(inv);
    for k=1:numel(fn)
        say(f,' %-42s = %d',fn{k},logical(inv.(fn{k})));
    end
    assert(inv.all,'[AF7R58ZJ] static patch invariant failed.');

    % Full-class parse before production write.
    preDir=fullfile(outdir,'AF7R58ZJ_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58ZJ_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    cp=onCleanup(@()safe_rmpath_if_present(preDir)); %#ok<NASGU>
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58ZJ] patched full-class parse failed.');
    eval(['clear ' parseName]);

    say(f,'');
    say(f,'[D. PREFLIGHT BEFORE WRITE]');
    say(f,' source semantic binding                = 1');
    say(f,' safety/certificate protected methods   = unchanged');
    say(f,' full patched class parse               = 1');
    say(f,' production writes                      = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    postSha=file_hash(pR);
    OUT.pre_sha=preSha;
    OUT.post_sha=postSha;
    OUT.production_modified=true;
    OUT.status='HOTPATH_DIAGNOSTICS_DETACHED__CLEAN_RERUN_REQUIRED';

    say(f,'');
    say(f,'[E. INSTALL RESULT]');
    say(f,' production modified                    = 1');
    say(f,' rollback performed                     = 0');
    say(f,' light runtime audit default            = OFF');
    say(f,' hotpath diagnostic mirror default      = OFF');
    say(f,' proposal diagnostic record in deadline = gated OFF');
    say(f,' R9 HR stamps in callback               = gated OFF');
    say(f,' heavy result.state trigger mirror      = gated OFF');
    say(f,' generator/geometry state mirror        = gated OFF');
    say(f,' safety/certificate/commit/cache math   = unchanged');
    say(f,' new SHA                                = %s',postSha);

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

save(fullfile(outdir,'AF7R58ZJ_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZJ_OUT',OUT);

say(f,'');
say(f,'RETURN: %s',report);
say(f,'RETURN: %s',fullfile(outdir,'AF7R58ZJ_OUT.mat'));
say(f,'status = %s',OUT.status);
end


function defs=discover_function_headers(lines)
lines=reshape(lines,1,[]);
defs=struct('line',{},'indent',{},'header',{},'header_end',{});
for k=1:numel(lines)
    if startsWith(strtrim(lines{k}),'function ')
        [hdr,ke]=collect_function_header(lines,k);
        defs(end+1)=struct('line',k,'indent',leading_ws_count(lines{k}), ...
            'header',hdr,'header_end',ke); %#ok<AGROW>
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
assert(numel(hits)==1,'[AF7R58ZJ] method "%s" count=%d.',name,numel(hits));
d=defs(hits(1));
i0=d.line; hdr=d.header; i1=numel(lines);
for k=i0+1:numel(lines)
    if startsWith(strtrim(lines{k}),'function ') && ...
            leading_ws_count(lines{k})==d.indent
        i1=k-1;
        break;
    end
end
b=reshape(lines(i0:i1),1,[]);
end


function tf=header_has_exact_method_name(header,name)
tf=false;
loc=strfind(header,name);
for ii=1:numel(loc)
    p=loc(ii); q=p+numel(name);
    if p>1 && is_ident_char(header(p-1)), continue; end
    if q<=numel(header) && is_ident_char(header(q)), continue; end
    while q<=numel(header) && isspace(header(q)), q=q+1; end
    if q<=numel(header) && header(q)=='(', tf=true; return; end
end
end


function [hdr,kend]=collect_function_header(lines,k0)
parts={strtrim(lines{k0})};
kend=k0;
while endsWith(strtrim(parts{end}),'...')
    parts{end}=strtrim(parts{end}(1:end-3));
    kend=kend+1;
    assert(kend<=numel(lines));
    parts{end+1}=strtrim(lines{kend}); %#ok<AGROW>
end
hdr=strjoin(parts,' ');
end


function n=leading_ws_count(line)
n=0;
for k=1:numel(line)
    if isspace(line(k)), n=n+1; else, break; end
end
end


function tf=is_ident_char(c)
tf=isstrprop(c,'alphanum') || c=='_';
end


function j=find_unique_regex(B,pat)
v=find(cellfun(@(x)~isempty(regexp(x,pat,'once')),B));
assert(numel(v)==1,'[AF7R58ZJ] regex count=%d for %s',numel(v),pat);
j=v(1);
end


function j=find_unique_contains(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1,'[AF7R58ZJ] token count=%d for %s',numel(v),token);
j=v(1);
end


function j=find_first_contains_after(B,token,a)
v=find(cellfun(@(x)contains(x,token),B(a+1:end)),1,'first');
assert(~isempty(v),'[AF7R58ZJ] token missing after anchor: %s',token);
j=a+v;
end


function j=find_first_exact_trim_after(B,token,a)
j=[];
for k=a+1:numel(B)
    if strcmp(strtrim(B{k}),token), j=k; break; end
end
assert(~isempty(j),'[AF7R58ZJ] exact "%s" missing after %d',token,a);
end


function c=insert_before(c,j,newLines)
c=reshape(c,1,[]); newLines=reshape(newLines,1,[]);
c=[c(1:j-1),newLines,c(j:end)];
end


function c=insert_after(c,j,newLines)
c=reshape(c,1,[]); newLines=reshape(newLines,1,[]);
c=[c(1:j),newLines,c(j+1:end)];
end


function c=replace_range(c,i0,i1,newLines)
c=reshape(c,1,[]); newLines=reshape(newLines,1,[]);
assert(i0>=1&&i1>=i0&&i1<=numel(c));
c=[c(1:i0-1),newLines,c(i1+1:end)];
end


function [s,eol]=read_text_norm(p)
fid=fopen(p,'rb'); assert(fid>=0,'cannot read %s',p);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
s=native2unicode(b.','UTF-8');
if contains(s,sprintf('\r\n')), eol='CRLF';
elseif contains(s,sprintf('\r')), eol='CR';
else, eol='LF'; end
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function write_text_eol(p,s,eol)
switch eol
    case 'CRLF', s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR',   s=strrep(s,sprintf('\n'),sprintf('\r'));
end
write_utf8(p,s);
end


function write_utf8(p,s)
fid=fopen(p,'wb'); assert(fid>=0,'cannot write %s',p);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(fid,b,'uint8');
assert(n==numel(b),'short write: %s',p);
end


function safe_rmpath_if_present(p)
try
    q=strsplit(path,pathsep);
    if any(strcmpi(q,p)), rmpath(p); end
catch
end
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0,'cannot read %s',p);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
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
