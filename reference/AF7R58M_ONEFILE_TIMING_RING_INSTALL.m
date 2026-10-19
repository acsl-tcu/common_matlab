function OUT = AF7R58M_ONEFILE_TIMING_RING_INSTALL()
% AF7R58M
% Transactional install of a bounded, no-throw, request-only timing ring.
%
% PURPOSE
%   Capture request-by-request timing for the unresolved 4-ms misses without
%   changing trajectory mathematics, collision certificates, resource gates,
%   actuator logic, controller logic, or commit decisions.
%
% IMPORTANT
%   - The ring runs only when invoke_replan_generator_shadow() is called.
%   - It is bounded (64 rows), numeric-only, and overwrites oldest rows.
%   - The capture helper is no-throw by construction.
%   - It is diagnostic instrumentation only.  It does not decide SAFE/UNSAFE.
%   - Production source is not written until patched source + helpers parse.
%   - Any post-write failure rolls production back.
%
% AFTER SUCCESS
%   Recreate REPLANNING_SYSTEM from a clean run and execute the same scenario
%   until the real replan requests occur.  Then the next audit reads the ring.
%
% RETURN
%   AF7R58M_REPORT.txt
%   AF7R58M_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58M_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58M] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58M | BOUNDED REQUEST-TIMING RING TRANSACTIONAL INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | bounded=1 | noThrowCapture=1 | rollbackOnFailure=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificatesChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58M_BOUNDED_TIMING_RING_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.error=struct();
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58M_REPORT.txt','AF7R58M_OUT.mat'};

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expectedR='503c4146f10892fdb45004c986bd6359f63e6ce5c59bf3f2ef61de971585badc';

helperNames={ ...
    'af7r58m_capture_timing_ring.m', ...
    'af7r58m_timing_ring_labels.m'};

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58M] REPLANNING_SYSTEM.m missing.');
    hR=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT PRE-INSTALL SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM reviewed SHA exact = %d | %s',strcmp(hR,expectedR),hR);
    assert(strcmp(hR,expectedR), ...
        '[AF7R58M] current REPLANNING_SYSTEM differs from AF7R58K/L authority; no install.');

    for k=1:numel(helperNames)
        assert(~isfile(fullfile(root,helperNames{k})), ...
            '[AF7R58M] helper already exists; refusing ambiguous overwrite: %s',helperNames{k});
    end

    [R,eolR]=read_text_norm(pR);

    % ------------------------------------------------------------------
    % Patch property block.  Exact source anchor is source-bound by SHA.
    % ------------------------------------------------------------------
    propAnchor='        generator_last_cycle_critical_ms   = NaN;';
    propInsert=blk({ ...
        propAnchor; ...
        ''; ...
        '        % AF7R58M bounded request-only timing ring (diagnostic only).'; ...
        '        ENABLE_AF7R58M_TIMING_RING          = true;'; ...
        '        af7r58m_timing_ring_capacity        = 64;'; ...
        '        af7r58m_timing_ring_width           = 32;'; ...
        '        af7r58m_timing_ring                  = nan(64,32);'; ...
        '        af7r58m_timing_ring_head             = 0;'; ...
        '        af7r58m_timing_ring_count            = 0;'; ...
        '        af7r58m_last_capture_overhead_us     = NaN;'; ...
        '        af7r58m_max_capture_overhead_us      = 0;'});
    R=patch_once(R,propAnchor,propInsert,'timing-ring properties');

    % ------------------------------------------------------------------
    % Every request creates one cleanup recorder.  It executes on every
    % ordinary/early return and therefore does not require branch edits.
    % ------------------------------------------------------------------
    reqAnchor='            obj.generator_request_count=obj.generator_request_count+1;';
    reqInsert=blk({ ...
        reqAnchor; ...
        '            af7r58m_ring_cleanup=onCleanup( ...'; ...
        '                @()af7r58m_capture_timing_ring(obj,t_hard_cycle)); %#ok<NASGU>'});
    R=patch_once(R,reqAnchor,reqInsert,'request cleanup recorder');

    helperSrc=helper_sources();

    % ------------------------------------------------------------------
    % Preflight parse in isolated directory before any production write.
    % ------------------------------------------------------------------
    preflightDir=fullfile(outdir,'AF7R58M_PREFLIGHT');
    [ok,msg]=mkdir(preflightDir); assert(ok,'%s',msg);

    for k=1:numel(helperNames)
        nm=helperNames{k};
        key=matlab.lang.makeValidName(nm);
        write_utf8(fullfile(preflightDir,nm),helperSrc.(key));
    end

    parseName='AF7R58M_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(R,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,R),'[AF7R58M] class rename anchor missing.');
    write_utf8(fullfile(preflightDir,[parseName '.m']),parseText);

    addpath(preflightDir,'-begin');
    cleanupPath=onCleanup(@()safe_rmpath(preflightDir)); %#ok<NASGU>
    rehash;

    say(f,'');
    say(f,'[B. PREFLIGHT PARSE -- PRODUCTION WRITES STILL ZERO]');
    for k=1:numel(helperNames)
        fn=erase(helperNames{k},'.m');
        eval(['clear ' fn]);
        pWhich=which(fn);
        assert(strcmpi(char(pWhich),char(fullfile(preflightDir,helperNames{k}))), ...
            '[AF7R58M] helper preflight path collision: %s',fn);
        nargin(fn);
        say(f,' helper %-36s PASS=1',fn);
    end

    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58M] patched REPLANNING_SYSTEM parse failed.');
    say(f,' patched REPLANNING_SYSTEM class              PASS=1');

    % ------------------------------------------------------------------
    % Backup and transactional production write.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,R,eolR);
    for k=1:numel(helperNames)
        nm=helperNames{k};
        copyfile(fullfile(preflightDir,nm),fullfile(root,nm),'f');
    end
    rehash;

    % Production resolution + parse.
    safe_rmpath(preflightDir);
    rehash;
    for k=1:numel(helperNames)
        fn=erase(helperNames{k},'.m');
        eval(['clear ' fn]);
        pWhich=which(fn);
        assert(strcmpi(char(pWhich),char(fullfile(root,helperNames{k}))), ...
            '[AF7R58M] production helper path mismatch: %s',fn);
        nargin(fn);
    end

    % Parse installed class through a renamed temporary copy as well.
    Rinstalled=fileread(pR);
    postParseName='AF7R58M_POST_PARSE_REPLANNING_SYSTEM';
    postParseText=strrep(Rinstalled,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postParseName ' < handle']);
    postParseFile=fullfile(outdir,[postParseName '.m']);
    write_utf8(postParseFile,postParseText);
    addpath(outdir,'-begin');
    cleanupPost=onCleanup(@()safe_rmpath(outdir)); %#ok<NASGU>
    eval(['clear ' postParseName]);
    mc2=meta.class.fromName(postParseName);
    assert(~isempty(mc2),'[AF7R58M] installed class post-parse failed.');

    % Source invariants after install.
    C=compact_ws(Rinstalled);
    inv=struct();
    inv.flag=contains(C,'ENABLE_AF7R58M_TIMING_RING=true;');
    inv.capacity=contains(C,'af7r58m_timing_ring_capacity=64;');
    inv.width=contains(C,'af7r58m_timing_ring_width=32;');
    inv.cleanup=contains(C,'af7r58m_capture_timing_ring(obj,t_hard_cycle)');
    inv.request_only=contains(C, ...
        'obj.generator_request_count=obj.generator_request_count+1;af7r58m_ring_cleanup=onCleanup(');
    inv.all=all(structfun(@logical,inv));
    assert(inv.all,'[AF7R58M] post-install source invariant failed.');

    hR2=file_hash(pR);
    OUT.source_hash_before=hR;
    OUT.source_hash_after=hR2;
    OUT.helper_hashes=struct();
    for k=1:numel(helperNames)
        fn=erase(helperNames{k},'.m');
        OUT.helper_hashes.(fn)=file_hash(fullfile(root,helperNames{k}));
    end
    OUT.invariants=inv;
    OUT.production_modified=true;
    OUT.status='TIMING_RING_INSTALL_COMPLETE__RERUN_REAL_REQUEST_SCENARIO_NEXT';

    say(f,'');
    say(f,'[C. PRODUCTION INSTALL]');
    say(f,' bounded ring capacity                        = 64 requests');
    say(f,' numeric columns                              = 32');
    say(f,' capture runs only on real generator requests = 1');
    say(f,' all return branches captured via onCleanup   = 1');
    say(f,' capture helper catches all internal errors   = 1');
    say(f,' control/safety decision modified             = 0');
    say(f,' trajectory/certificate mathematics modified = 0');
    say(f,' production modified                          = 1');

    labels=af7r58m_timing_ring_labels();
    OUT.column_labels=labels;
    say(f,'');
    say(f,'[D. RING COLUMNS]');
    for k=1:numel(labels)
        say(f,' %2d  %s',k,labels{k});
    end

    say(f,'');
    say(f,'[E. NEXT RUN]');
    say(f,' Recreate REPLANNING_SYSTEM from a clean run.');
    say(f,' Execute the same real scenario until >=10 generator requests if possible.');
    say(f,' Do NOT infer cause from one request; the next audit reads the whole bounded ring.');
    say(f,' This diagnostic ring is temporary instrumentation and is not a safety certificate.');

catch ME
    if backupReady&&writesStarted
        rollback_install(root,backupDir,pR,helperNames);
        OUT.rollback_performed=true;
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

save(fullfile(outdir,'AF7R58M_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58M_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58M_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58M_OUT.mat'));
say(f,' status = %s',OUT.status);

end


function S=helper_sources()
S=struct();

S.af7r58m_capture_timing_ring_m=blk({ ...
'function af7r58m_capture_timing_ring(obj,t_hard_cycle)'; ...
'% AF7R58M bounded diagnostic recorder.  This function must never throw.'; ...
'tcap=tic;'; ...
'try'; ...
'    if ~(isa(obj,''REPLANNING_SYSTEM'')&&logical(obj.ENABLE_AF7R58M_TIMING_RING))'; ...
'        return;'; ...
'    end'; ...
'    cap=double(obj.af7r58m_timing_ring_capacity);'; ...
'    wid=double(obj.af7r58m_timing_ring_width);'; ...
'    if ~(isscalar(cap)&&isscalar(wid)&&isfinite(cap)&&isfinite(wid)&&cap==64&&wid==32)'; ...
'        return;'; ...
'    end'; ...
'    if ~isequal(size(obj.af7r58m_timing_ring),[64 32])'; ...
'        return;'; ...
'    end'; ...
''; ...
'    r=nan(1,32);'; ...
'    r(1)=double(obj.generator_request_count);'; ...
'    r(2)=1e3*toc(t_hard_cycle);'; ...
'    r(3)=double(obj.generator_last_cycle_critical_ms);'; ...
'    r(4)=double(obj.generator_last_elapsed_before_ms);'; ...
'    r(5)=double(obj.generator_last_budget_remaining_ms);'; ...
'    r(6)=double(obj.generator_last_context_ms);'; ...
'    r(7)=double(obj.generator_last_callback_ms);'; ...
'    r(8)=double(obj.generator_last_validation_ms);'; ...
'    r(9)=double(obj.generator_last_critical_ms);'; ...
'    r(10)=double(obj.generator_last_runtime_ms);'; ...
'    r(11)=double(obj.generator_last_status_code);'; ...
'    r(12)=double(obj.generator_deadline_fail_count);'; ...
'    r(13)=double(obj.generator_candidate_count);'; ...
'    r(14)=double(obj.generator_validation_pass_count);'; ...
'    r(15)=double(obj.generator_validation_fail_count);'; ...
'    r(16)=double(obj.active_reference_version);'; ...
'    r(17)=double(obj.replan_commit_count);'; ...
'    r(18)=double(obj.reference_only_evaluation_count);'; ...
'    r(19)=double(obj.reference_only_commit_count);'; ...
'    r(20)=double(obj.reference_only_runtime_packet_reuse_count);'; ...
'    r(21)=double(obj.reference_only_runtime_packet_rebuild_count);'; ...
'    r(22)=double(obj.reference_only_fast_packet_rebuild_count);'; ...
'    r(23)=double(obj.compact_native_proposal_count);'; ...
'    r(24)=double(obj.r9_stage_episode_ms);'; ...
'    r(25)=double(obj.r9_synth_hessian_ms);'; ...
'    r(26)=double(obj.r9_synth_corridor_ms);'; ...
'    r(27)=double(obj.r9_synth_qp_ms);'; ...
'    r(28)=double(obj.r9_synth_candidate_build_ms);'; ...
'    r(29)=double(obj.r9_stage_synthesis_ms);'; ...
'    r(30)=double(obj.r9_stage_candidate_loop_ms);'; ...
'    r(31)=double(obj.r9_stage_total_internal_ms);'; ...
'    r(32)=double(obj.r9_hr_callback_wall_ms);'; ...
''; ...
'    head=double(obj.af7r58m_timing_ring_head);'; ...
'    idx=mod(head,cap)+1;'; ...
'    obj.af7r58m_timing_ring(idx,:)=r;'; ...
'    obj.af7r58m_timing_ring_head=idx;'; ...
'    obj.af7r58m_timing_ring_count=min(cap,double(obj.af7r58m_timing_ring_count)+1);'; ...
''; ...
'    us=1e6*toc(tcap);'; ...
'    obj.af7r58m_last_capture_overhead_us=us;'; ...
'    if isfinite(us)'; ...
'        obj.af7r58m_max_capture_overhead_us=max(double(obj.af7r58m_max_capture_overhead_us),us);'; ...
'    end'; ...
'catch'; ...
'    % Diagnostic instrumentation must never alter control flow.'; ...
'end'; ...
'end'; ...
''});

S.af7r58m_timing_ring_labels_m=blk({ ...
'function L = af7r58m_timing_ring_labels()'; ...
'L={ ...'; ...
' ''request_seq'', ...'; ...
' ''cleanup_entry_cycle_ms'', ...'; ...
' ''recorded_cycle_ms'', ...'; ...
' ''elapsed_before_ms'', ...'; ...
' ''budget_remaining_ms'', ...'; ...
' ''context_ms'', ...'; ...
' ''callback_ms'', ...'; ...
' ''validation_ms'', ...'; ...
' ''critical_ms'', ...'; ...
' ''runtime_ms'', ...'; ...
' ''status_code'', ...'; ...
' ''deadline_fail_count_cumulative'', ...'; ...
' ''candidate_count_cumulative'', ...'; ...
' ''validation_pass_count_cumulative'', ...'; ...
' ''validation_fail_count_cumulative'', ...'; ...
' ''active_reference_version'', ...'; ...
' ''replan_commit_count'', ...'; ...
' ''reference_only_evaluation_count'', ...'; ...
' ''reference_only_commit_count'', ...'; ...
' ''runtime_packet_reuse_count'', ...'; ...
' ''runtime_packet_rebuild_count'', ...'; ...
' ''fast_packet_rebuild_count'', ...'; ...
' ''compact_native_proposal_count'', ...'; ...
' ''stage_episode_ms'', ...'; ...
' ''stage_hessian_ms'', ...'; ...
' ''stage_corridor_ms'', ...'; ...
' ''stage_qp_ms'', ...'; ...
' ''stage_candidate_build_ms'', ...'; ...
' ''stage_synthesis_ms'', ...'; ...
' ''stage_candidate_loop_ms'', ...'; ...
' ''stage_total_internal_ms'', ...'; ...
' ''callback_wall_qpc_ms''};'; ...
'end'; ...
''});
end


function rollback_install(root,backupDir,pR,helperNames)
try
    copyfile(fullfile(backupDir,'REPLANNING_SYSTEM.m'),pR,'f');
catch
end
for k=1:numel(helperNames)
    p=fullfile(root,helperNames{k});
    if isfile(p)
        try delete(p); catch, end
    end
end
rehash;
end


function s=patch_once(s,old,new,label)
idx=strfind(s,old);
assert(numel(idx)==1, ...
    '[AF7R58M] patch anchor must occur exactly once (%s), found %d.',label,numel(idx));
k=idx(1);
s=[s(1:k-1) new s(k+numel(old):end)];
end


function [s,eol]=read_text_norm(p)
f=fopen(p,'rb'); assert(f>=0,'cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
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
f=fopen(p,'wb'); assert(f>=0,'[AF7R58M] cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(f,b,'uint8');
assert(n==numel(b),'[AF7R58M] short write: %s',p);
end


function safe_rmpath(p)
try rmpath(p); catch, end
end


function s=compact_ws(s)
s=regexprep(s,'\s+','');
end


function s=blk(c)
s=strjoin(c,newline);
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
