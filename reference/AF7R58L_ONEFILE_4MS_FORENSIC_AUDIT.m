function OUT = AF7R58L_ONEFILE_4MS_FORENSIC_AUDIT(replanner)
% AF7R58L
% Read-only forensic audit for the remaining 4-ms deadline misses.
%
% PURPOSE
%   AF7R58K closed the live resource/canonical representation theorem but
%   observed 9 deadline misses in 10 real requests.  This audit captures:
%     - all timing/deadline/replan-related live object properties;
%     - last-replan status without mutating the object;
%     - exact current source excerpts around timing/deadline instrumentation;
%     - native AF7R36 per-stage timing fields and their propagation path;
%     - availability of historical per-request timing evidence.
%
% It DOES NOT modify production, run a synthetic replan, commit a plan,
% weaken any certificate, or guess a cause from the last cycle alone.
%
% Usage:
%   AF7R58L_ONEFILE_4MS_FORENSIC_AUDIT
%   AF7R58L_ONEFILE_4MS_FORENSIC_AUDIT(replanner)
%
% RETURN:
%   AF7R58L_REPORT.txt
%   AF7R58L_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58L_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58L] cannot open report.');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58L | 4-MS DEADLINE-MISS FORENSIC AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noCommit=1 | noSyntheticReplan=1 | sampledSafetyAcceptance=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58L_4MS_FORENSIC_AUDIT_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58L_REPORT.txt','AF7R58L_OUT.mat'};
OUT.error=struct();

try
    % ------------------------------------------------------------------
    % A. Source binding.
    % ------------------------------------------------------------------
    srcNames={ ...
        'REPLANNING_SYSTEM.m', ...
        'af7r36_core.hpp', ...
        'af7r36_single_core_mex.cpp', ...
        'af7r36_single_core_proposal.m', ...
        'af7r34_make_core_packet.m', ...
        'af7r58i_resource_contract.m', ...
        'af7r58i_commit_spacing_guard.m', ...
        'af7r58i_prepare_batch_model.m'};

    SRC=struct();
    say(f,'');
    say(f,'[A. CURRENT SOURCE BINDING]');
    for k=1:numel(srcNames)
        nm=srcNames{k};
        p=fullfile(root,nm);
        present=isfile(p);
        h='';
        if present, h=file_hash(p); end
        key=matlab.lang.makeValidName(nm);
        SRC.(key)=struct('name',nm,'path',p,'present',present,'sha256',h);
        say(f,' %-38s present=%d | %s',nm,present,h);
    end
    OUT.source_binding=SRC;

    assert(SRC.REPLANNING_SYSTEM_m.present,'[AF7R58L] REPLANNING_SYSTEM.m missing.');
    assert(SRC.af7r36_core_hpp.present,'[AF7R58L] af7r36_core.hpp missing.');

    % ------------------------------------------------------------------
    % B. Resolve live object.
    % ------------------------------------------------------------------
    names={};
    if nargin<1||isempty(replanner)
        [replanner,names]=find_live_replanner();
    end
    if isempty(replanner)
        OUT.live_object=struct('found',false,'candidate_names',{names});
        OUT.status='LIVE_REPLANNER_OBJECT_REQUIRED';
        say(f,'');
        say(f,'[B. LIVE OBJECT]');
        say(f,' unique REPLANNING_SYSTEM object not found.');
        if ~isempty(names), say(f,' candidates = %s',strjoin(names,', ')); end
        finish();
        return;
    end
    assert(isa(replanner,'REPLANNING_SYSTEM'),'[AF7R58L] object is not REPLANNING_SYSTEM.');

    % Essential ledger first.
    L=struct();
    L.active_mode=char(replanner.active_reference_mode);
    L.active_version=double(replanner.active_reference_version);
    L.dt=double(replanner.prediction_dt);
    L.request_count=double(replanner.generator_request_count);
    L.deadline_fail_count=double(replanner.generator_deadline_fail_count);
    L.last_cycle_ms=double(replanner.generator_last_cycle_critical_ms);
    L.deadline_ms=1e3*double(replanner.hard_modify_deadline_s);
    L.last_commit_time=double(replanner.last_replan_commit_time);
    OUT.live_ledger=L;

    say(f,'');
    say(f,'[B. LIVE 4-MS LEDGER]');
    say(f,' active mode/version             = %s / %.0f',L.active_mode,L.active_version);
    say(f,' prediction dt [s]              = %.17g',L.dt);
    say(f,' request count                  = %.0f',L.request_count);
    say(f,' deadline fail count            = %.0f',L.deadline_fail_count);
    say(f,' deadline fail fraction         = %.17g',safe_div(L.deadline_fail_count,L.request_count));
    say(f,' last critical cycle [ms]       = %.17g',L.last_cycle_ms);
    say(f,' hard deadline [ms]             = %.17g',L.deadline_ms);
    say(f,' last commit time [s]           = %.17g',L.last_commit_time);

    % ------------------------------------------------------------------
    % C. Capture all timing/replan/deadline live properties.
    % ------------------------------------------------------------------
    P=properties(replanner);
    keep=false(size(P));
    tokens={'time','tim','ms','cycle','deadline','replan','generator','runtime', ...
            'qpc','stamp','latency','duration','profile','count','warmup','cache'};
    for i=1:numel(P)
        q=lower(P{i});
        keep(i)=any(cellfun(@(t)contains(q,t),tokens));
    end
    selected=P(keep);

    PROP=struct();
    say(f,'');
    say(f,'[C. LIVE TIMING/REPLAN PROPERTY CAPTURE]');
    say(f,' selected properties = %d',numel(selected));
    for i=1:numel(selected)
        nm=selected{i};
        key=matlab.lang.makeValidName(nm);
        try
            v=replanner.(nm);
            PROP.(key)=pack_value(v);
            say_value(f,nm,v);
        catch MEp
            PROP.(key)=struct('accessible',false,'error',MEp.message);
            say(f,' %-46s = <inaccessible: %s>',nm,MEp.message);
        end
    end
    OUT.live_properties=PROP;

    % ------------------------------------------------------------------
    % D. Last status + method inventory.
    % ------------------------------------------------------------------
    ST=struct();
    if ismethod(replanner,'get_last_replan_status')
        try
            ST.last_replan_status=replanner.get_last_replan_status();
        catch MEs
            ST.last_replan_status_error=MEs.message;
        end
    end
    if ismethod(replanner,'get_avoidance_status')
        try
            ST.avoidance_status=replanner.get_avoidance_status();
        catch MEs
            ST.avoidance_status_error=MEs.message;
        end
    end
    M=methods(replanner);
    getMask=false(size(M));
    for i=1:numel(M)
        q=lower(M{i});
        getMask(i)=startsWith(q,'get_') && ...
            (contains(q,'replan')||contains(q,'status')||contains(q,'runtime')|| ...
             contains(q,'generator')||contains(q,'tim'));
    end
    ST.relevant_getters=M(getMask);
    OUT.status_capture=ST;

    say(f,'');
    say(f,'[D. LAST STATUS / GETTER INVENTORY]');
    if isfield(ST,'last_replan_status')
        say_struct_summary(f,' last_replan_status',ST.last_replan_status,0);
    elseif isfield(ST,'last_replan_status_error')
        say(f,' last_replan_status error = %s',ST.last_replan_status_error);
    else
        say(f,' get_last_replan_status method absent.');
    end
    say(f,' relevant getter methods = %d',numel(ST.relevant_getters));
    for i=1:numel(ST.relevant_getters)
        say(f,'   %s',ST.relevant_getters{i});
    end

    % ------------------------------------------------------------------
    % E. Exact source excerpts around timing/deadline machinery.
    % ------------------------------------------------------------------
    queries={ ...
        'generator_request_count', ...
        'generator_deadline_fail_count', ...
        'generator_last_cycle_critical_ms', ...
        'hard_modify_deadline_s', ...
        'cycle_ms', ...
        't_hard_cycle', ...
        'deadline', ...
        'af7r25_stamp_mex', ...
        'QueryPerformanceCounter', ...
        'ProcessCycleTime', ...
        'ThreadCycleTime', ...
        'candidate_build', ...
        'tim.total', ...
        'tim.episode', ...
        'tim.hessian', ...
        'tim.corridor', ...
        'tim.qp', ...
        'tim.synthesis', ...
        'tim.candidate'};

    targets={ ...
        fullfile(root,'REPLANNING_SYSTEM.m'), ...
        fullfile(root,'af7r36_core.hpp'), ...
        fullfile(root,'af7r36_single_core_mex.cpp'), ...
        fullfile(root,'af7r36_single_core_proposal.m')};

    EX=struct('file',{},'query',{},'line',{},'text',{},'context',{});
    say(f,'');
    say(f,'[E. SOURCE-BOUND TIMING/DEADLINE EXCERPTS]');
    for ti=1:numel(targets)
        p=targets{ti};
        if ~isfile(p), continue; end
        [lines,rel]=read_lines(p,root);
        for qi=1:numel(queries)
            hit=find(contains(lines,queries{qi},'IgnoreCase',true));
            % Keep every exact current-source hit but print at most 12/query/file.
            for hi=1:numel(hit)
                ln=hit(hi);
                a=max(1,ln-3); b=min(numel(lines),ln+3);
                ctx=lines(a:b);
                EX(end+1)=struct('file',rel,'query',queries{qi},'line',ln, ...
                    'text',lines{ln},'context',{ctx}); %#ok<AGROW>
            end
            nshow=min(numel(hit),12);
            for hi=1:nshow
                ln=hit(hi);
                say(f,' %s:%d | [%s] %s',rel,ln,queries{qi},strtrim(lines{ln}));
            end
            if numel(hit)>nshow
                say(f,'   ... %d more hits retained in MAT',numel(hit)-nshow);
            end
        end
    end
    OUT.source_excerpts=EX;

    % ------------------------------------------------------------------
    % F. Determine whether historical per-request timing is already retained.
    % ------------------------------------------------------------------
    H=struct();
    H.history_like_properties={};
    H.vector_timing_properties={};
    fn=fieldnames(PROP);
    for i=1:numel(fn)
        nm=fn{i};
        q=lower(nm);
        if contains(q,'hist')||contains(q,'log')||contains(q,'trace')||contains(q,'sample')
            H.history_like_properties{end+1}=nm; %#ok<AGROW>
        end
        v=PROP.(nm);
        if isstruct(v)&&isfield(v,'class')&&strcmp(v.class,'double')&& ...
                isfield(v,'numel')&&v.numel>1 && ...
                (contains(q,'ms')||contains(q,'time')||contains(q,'cycle')||contains(q,'tim'))
            H.vector_timing_properties{end+1}=nm; %#ok<AGROW>
        end
    end
    H.has_per_request_history=~isempty(H.history_like_properties)||~isempty(H.vector_timing_properties);

    % Native per-stage timing availability from source.
    core=fileread(fullfile(root,'af7r36_core.hpp'));
    cc=compact_ws(core);
    H.native_stage_fields=struct( ...
        'episode',contains(cc,'doubleepisode=0'), ...
        'hessian',contains(cc,'hessian=0'), ...
        'corridor',contains(cc,'corridor=0'), ...
        'qp',contains(cc,'qp=0'), ...
        'candidate_build',contains(cc,'candidate_build=0'), ...
        'synthesis',contains(cc,'synthesis=0'), ...
        'candidate',contains(cc,'candidate=0'), ...
        'total',contains(cc,'total=0'));
    H.native_stage_timing_exists=all(structfun(@logical,H.native_stage_fields));

    OUT.history_evidence=H;

    say(f,'');
    say(f,'[F. HISTORICAL TIMING EVIDENCE AVAILABILITY]');
    say(f,' history-like live properties              = %d',numel(H.history_like_properties));
    for i=1:numel(H.history_like_properties), say(f,'   %s',H.history_like_properties{i}); end
    say(f,' vector timing live properties             = %d',numel(H.vector_timing_properties));
    for i=1:numel(H.vector_timing_properties), say(f,'   %s',H.vector_timing_properties{i}); end
    say(f,' retained per-request history already found= %d',H.has_per_request_history);
    say(f,' native AF7R36 per-stage timing fields exist= %d',H.native_stage_timing_exists);
    say(f,' native fields: episode=%d hessian=%d corridor=%d qp=%d candidate_build=%d synthesis=%d candidate=%d total=%d', ...
        H.native_stage_fields.episode,H.native_stage_fields.hessian,H.native_stage_fields.corridor, ...
        H.native_stage_fields.qp,H.native_stage_fields.candidate_build,H.native_stage_fields.synthesis, ...
        H.native_stage_fields.candidate,H.native_stage_fields.total);

    % ------------------------------------------------------------------
    % G. Conservative interpretation.
    % ------------------------------------------------------------------
    I=struct();
    I.resource_or_canonical_issue=false;
    I.deadline_issue=(L.request_count>0)&&(L.deadline_fail_count>0);
    I.last_cycle_currently_passes=isfinite(L.last_cycle_ms)&&L.last_cycle_ms<=L.deadline_ms;
    I.can_attribute_existing_misses=H.has_per_request_history;
    I.need_low_overhead_ring_if_no_history=I.deadline_issue&&~I.can_attribute_existing_misses;
    OUT.interpretation=I;

    say(f,'');
    say(f,'[G. CONSERVATIVE INTERPRETATION]');
    say(f,' resource/canonical issue implicated        = 0');
    say(f,' real deadline issue exists                 = %d',I.deadline_issue);
    say(f,' last observed cycle currently <=4ms        = %d',I.last_cycle_currently_passes);
    say(f,' existing retained history can attribute 9 misses = %d',I.can_attribute_existing_misses);
    say(f,' low-overhead bounded timing ring needed if history absent = %d',I.need_low_overhead_ring_if_no_history);
    say(f,' NOTE: the last 3.5703-ms cycle is not evidence that the previous 9 misses were warmup.');
    say(f,'       Cause remains OPEN until request-by-request stage timing is available.');

    if I.deadline_issue && H.native_stage_timing_exists
        if H.has_per_request_history
            OUT.status='4MS_FORENSIC_CAPTURE_COMPLETE__ANALYZE_RETAINED_REQUEST_HISTORY_NEXT';
        else
            OUT.status='4MS_CAUSE_OPEN__INSTALL_BOUNDED_LOW_OVERHEAD_STAGE_TIMING_RING_NEXT';
        end
    elseif ~I.deadline_issue
        OUT.status='NO_RECORDED_DEADLINE_MISS_IN_CURRENT_OBJECT';
    else
        OUT.status='4MS_FORENSIC_OPEN__TIMING_AUTHORITY_INSUFFICIENT';
    end

    say(f,'');
    say(f,'[H. STATUS]');
    say(f,' status = %s',OUT.status);
    say(f,' production modified = 0');
    say(f,' next action must preserve all existing continuous-time safety/resource certificates.');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR]');
    say(f,' identifier = %s',ME.identifier);
    say(f,' message    = %s',ME.message);
    say(f,' production modified = 0');
end

finish();

    function finish()
        save(fullfile(outdir,'AF7R58L_OUT.mat'),'OUT','-v7');
        assignin('base','AF7R58L_OUT',OUT);
        say(f,'');
        say(f,'[RETURN THESE TWO FILES]');
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58L_REPORT.txt'));
        say(f,' RETURN: %s',fullfile(outdir,'AF7R58L_OUT.mat'));
        say(f,' status = %s',OUT.status);
    end
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


function S=pack_value(v)
S=struct();
S.accessible=true;
S.class=class(v);
S.size=size(v);
S.numel=numel(v);
if isnumeric(v)||islogical(v)
    if numel(v)<=5000
        S.value=v;
    else
        S.value=[];
        S.min=safe_min(v);
        S.max=safe_max(v);
    end
elseif ischar(v)||isstring(v)
    S.value=v;
elseif isstruct(v)
    S.value=v;
elseif iscell(v)
    if numel(v)<=200
        S.value=v;
    else
        S.value=[];
    end
else
    S.value=[];
end
end


function say_value(f,nm,v)
try
    if isnumeric(v)||islogical(v)
        if isscalar(v)
            say(f,' %-46s = %.17g',nm,double(v));
        elseif numel(v)<=20
            say(f,' %-46s = %s',nm,mat2str(v));
        else
            say(f,' %-46s = <%s %s, numel=%d>',nm,class(v),mat2str(size(v)),numel(v));
        end
    elseif ischar(v)
        say(f,' %-46s = %s',nm,v);
    elseif isstring(v)&&isscalar(v)
        say(f,' %-46s = %s',nm,char(v));
    elseif isstruct(v)
        say(f,' %-46s = <struct fields=%s>',nm,strjoin(fieldnames(v),','));
    elseif iscell(v)
        say(f,' %-46s = <cell %s>',nm,mat2str(size(v)));
    else
        say(f,' %-46s = <%s>',nm,class(v));
    end
catch
    say(f,' %-46s = <print unavailable>',nm);
end
end


function say_struct_summary(f,label,S,depth)
if depth>2
    say(f,'%s = <depth limit>',label);
    return;
end
if ~isstruct(S)
    say_value(f,label,S);
    return;
end
F=fieldnames(S);
say(f,'%s fields=%d',label,numel(F));
for i=1:numel(F)
    nm=F{i};
    v=S.(nm);
    if isstruct(v)&&isscalar(v)
        say_struct_summary(f,['   ' nm],v,depth+1);
    else
        say_value(f,['   ' nm],v);
    end
end
end


function [lines,rel]=read_lines(p,root)
txt=fileread(p);
lines=regexp(txt,'\r\n|\n|\r','split');
if startsWith(p,[root filesep])
    rel=p(numel(root)+2:end);
else
    rel=p;
end
end


function x=safe_min(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=min(v); end
end


function x=safe_max(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=max(v); end
end


function y=safe_div(a,b)
if b==0, y=NaN; else, y=a/b; end
end


function c=compact_ws(s)
c=regexprep(s,'\s+','');
end


function h=file_hash(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
