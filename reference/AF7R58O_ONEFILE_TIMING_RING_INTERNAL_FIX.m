function OUT = AF7R58O_ONEFILE_TIMING_RING_INTERNAL_FIX()
% AF7R58O
% Transactional fix for AF7R58M timing-ring capture.
%
% Root cause addressed:
%   AF7R58M used an external helper to write REPLANNING_SYSTEM diagnostic
%   properties.  If those properties have restricted SetAccess, reads from
%   external audits can succeed while writes from the helper fail.  The helper
%   intentionally swallowed errors, leaving the ring empty.
%
% Fix:
%   Move the bounded no-throw capture routine into REPLANNING_SYSTEM itself
%   and make onCleanup call that internal method.  No safety/trajectory logic,
%   deadline decision, controller, actuator, obstacle certificate, or mL
%   authority is changed.
%
% Production is not written until a renamed patched class parses successfully.
% Any post-write failure restores the pre-fix source.
%
% RETURN:
%   AF7R58O_REPORT.txt
%   AF7R58O_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58O_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58O] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58O | INTERNAL TIMING-RING WRITER TRANSACTIONAL FIX');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | productionModifiedInitially=0 | rollbackOnFailure=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58O_INTERNAL_TIMING_RING_FIX_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58O_REPORT.txt','AF7R58O_OUT.mat'};
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expectedR='96670bd3763f1902426ad8a1e82f995850f9b42670c9a3a9d2b5bee0375cf363';
backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58O] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT PRE-FIX SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM AF7R58M SHA exact = %d | %s',strcmp(h0,expectedR),h0);
    assert(strcmp(h0,expectedR), ...
        '[AF7R58O] current REPLANNING_SYSTEM differs from AF7R58M authority; no patch.');

    [R,eolR]=read_text_norm(pR);

    oldCall=blk({ ...
        '            af7r58m_ring_cleanup=onCleanup( ...'; ...
        '                @()af7r58m_capture_timing_ring(obj,t_hard_cycle)); %#ok<NASGU>'});
    newCall=blk({ ...
        '            af7r58m_ring_cleanup=onCleanup( ...'; ...
        '                @()obj.af7r58m_capture_timing_ring_internal(t_hard_cycle)); %#ok<NASGU>'});
    R=patch_once(R,oldCall,newCall,'cleanup callback');

    marker='        function invoke_replan_generator_shadow(obj,t_hard_cycle,t_now,';
    pos=strfind(R,marker);
    assert(numel(pos)==1,'[AF7R58O] invoke_replan_generator_shadow marker count=%d',numel(pos));

    methodText=blk({ ...
        '        function af7r58m_capture_timing_ring_internal(obj,t_hard_cycle)'; ...
        '            % AF7R58O bounded request-only diagnostic writer.'; ...
        '            % Must never throw and must never affect control decisions.'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58M_TIMING_RING)'; ...
        '                    return;'; ...
        '                end'; ...
        '                cap=double(obj.af7r58m_timing_ring_capacity);'; ...
        '                wid=double(obj.af7r58m_timing_ring_width);'; ...
        '                if ~(cap==64 && wid==32 && isequal(size(obj.af7r58m_timing_ring),[64 32]))'; ...
        '                    return;'; ...
        '                end'; ...
        ''; ...
        '                r=nan(1,32);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(t_hard_cycle);'; ...
        '                r(3)=double(obj.generator_last_cycle_critical_ms);'; ...
        '                r(4)=double(obj.generator_last_elapsed_before_ms);'; ...
        '                r(5)=double(obj.generator_last_budget_remaining_ms);'; ...
        '                r(6)=double(obj.generator_last_context_ms);'; ...
        '                r(7)=double(obj.generator_last_callback_ms);'; ...
        '                r(8)=double(obj.generator_last_validation_ms);'; ...
        '                r(9)=double(obj.generator_last_critical_ms);'; ...
        '                r(10)=double(obj.generator_last_runtime_ms);'; ...
        '                r(11)=double(obj.generator_last_status_code);'; ...
        '                r(12)=double(obj.generator_deadline_fail_count);'; ...
        '                r(13)=double(obj.generator_candidate_count);'; ...
        '                r(14)=double(obj.generator_validation_pass_count);'; ...
        '                r(15)=double(obj.generator_validation_fail_count);'; ...
        '                r(16)=double(obj.active_reference_version);'; ...
        '                r(17)=double(obj.replan_commit_count);'; ...
        '                r(18)=double(obj.reference_only_evaluation_count);'; ...
        '                r(19)=double(obj.reference_only_commit_count);'; ...
        '                r(20)=double(obj.reference_only_runtime_packet_reuse_count);'; ...
        '                r(21)=double(obj.reference_only_runtime_packet_rebuild_count);'; ...
        '                r(22)=double(obj.reference_only_fast_packet_rebuild_count);'; ...
        '                r(23)=double(obj.compact_native_proposal_count);'; ...
        '                r(24)=double(obj.r9_stage_episode_ms);'; ...
        '                r(25)=double(obj.r9_synth_hessian_ms);'; ...
        '                r(26)=double(obj.r9_synth_corridor_ms);'; ...
        '                r(27)=double(obj.r9_synth_qp_ms);'; ...
        '                r(28)=double(obj.r9_synth_candidate_build_ms);'; ...
        '                r(29)=double(obj.r9_stage_synthesis_ms);'; ...
        '                r(30)=double(obj.r9_stage_candidate_loop_ms);'; ...
        '                r(31)=double(obj.r9_stage_total_internal_ms);'; ...
        '                r(32)=double(obj.r9_hr_callback_wall_ms);'; ...
        ''; ...
        '                head=double(obj.af7r58m_timing_ring_head);'; ...
        '                idx=mod(head,cap)+1;'; ...
        '                obj.af7r58m_timing_ring(idx,:)=r;'; ...
        '                obj.af7r58m_timing_ring_head=idx;'; ...
        '                obj.af7r58m_timing_ring_count=min(cap,double(obj.af7r58m_timing_ring_count)+1);'; ...
        ''; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58m_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58m_max_capture_overhead_us= ...'; ...
        '                        max(double(obj.af7r58m_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostic capture must never alter the control path.'; ...
        '            end'; ...
        '        end'; ...
        ''; ...
        ''});

    R=[R(1:pos-1) methodText R(pos:end)];

    % Static source invariants before parse.
    C=compact_ws(R);
    pre=struct();
    pre.internal_method=contains(C,'functionaf7r58m_capture_timing_ring_internal(obj,t_hard_cycle)');
    pre.cleanup_internal=contains(C,'@()obj.af7r58m_capture_timing_ring_internal(t_hard_cycle)');
    pre.old_external_callback_absent=~contains(C,'@()af7r58m_capture_timing_ring(obj,t_hard_cycle)');
    pre.bounded_64x32=contains(C,'cap==64&&wid==32&&isequal(size(obj.af7r58m_timing_ring),[6432])');
    pre.no_throw=contains(C,'catch%Diagnosticcapturemustneveralterthecontrolpath.');
    pre.all=pre.internal_method&&pre.cleanup_internal&&pre.old_external_callback_absent&&pre.bounded_64x32;

    say(f,'');
    say(f,'[B. PREWRITE SOURCE INVARIANTS]');
    say(f,' internal capture method inserted       = %d',pre.internal_method);
    say(f,' cleanup calls internal method          = %d',pre.cleanup_internal);
    say(f,' external writer callback removed       = %d',pre.old_external_callback_absent);
    say(f,' fixed bounded schema retained          = %d',pre.bounded_64x32);
    say(f,' invariant conjunction                  = %d',pre.all);
    assert(pre.all,'[AF7R58O] prewrite source invariant failed.');

    % Preflight class parse under unique name before production write.
    preflightDir=fullfile(outdir,'AF7R58O_PREFLIGHT');
    [ok,msg]=mkdir(preflightDir); assert(ok,'%s',msg);
    parseName='AF7R58O_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(R,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,R),'[AF7R58O] class rename anchor missing.');
    parseFile=fullfile(preflightDir,[parseName '.m']);
    write_utf8(parseFile,parseText);

    addpath(preflightDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58O] patched class preflight parse failed.');
    safe_rmpath_if_present(preflightDir);

    say(f,'');
    say(f,'[C. PREFLIGHT PARSE -- PRODUCTION WRITES STILL ZERO]');
    say(f,' patched REPLANNING_SYSTEM class PASS=1');

    % Backup + write.
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,R,eolR);
    rehash;

    % Post-write parse under another unique name.
    postName='AF7R58O_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);
    postFile=fullfile(outdir,[postName '.m']);
    write_utf8(postFile,postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58O] installed class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.prewrite_invariants=pre;
    OUT.production_modified=true;
    OUT.status='INTERNAL_TIMING_RING_FIX_COMPLETE__CLEAN_RERUN_REQUIRED';

    say(f,'');
    say(f,'[D. INSTALL RESULT]');
    say(f,' internal class writer installed        = 1');
    say(f,' external helper no longer writes ring  = 1');
    say(f,' ring capacity/width unchanged           = 64 / 32');
    say(f,' safety/trajectory logic changed         = 0');
    say(f,' production modified                     = 1');
    say(f,' new REPLANNING_SYSTEM SHA               = %s',h1);

    say(f,'');
    say(f,'[E. NEXT STEP]');
    say(f,' Recreate REPLANNING_SYSTEM from a clean run.');
    say(f,' Run the same scenario until the real generator requests occur.');
    say(f,' Then run AF7R58N_ONEFILE_TIMING_RING_ANALYSIS on that SAME live object.');

catch ME
    if backupReady&&writesStarted
        try
            copyfile(fullfile(backupDir,'REPLANNING_SYSTEM.m'),pR,'f');
            rehash;
            OUT.rollback_performed=true;
        catch
        end
    end
    OUT.production_modified=false;
    OUT.status='FIX_FAILED__ROLLBACK_COMPLETE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    say(f,'');
    say(f,'[FIX FAILURE]');
    say(f,' identifier          = %s',ME.identifier);
    say(f,' message             = %s',ME.message);
    say(f,' rollback performed  = %d',OUT.rollback_performed);
    say(f,' production modified = 0');
end

save(fullfile(outdir,'AF7R58O_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58O_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58O_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58O_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function s=patch_once(s,old,new,label)
idx=strfind(s,old);
assert(numel(idx)==1, ...
    '[AF7R58O] patch anchor must occur exactly once (%s), found %d.',label,numel(idx));
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
f=fopen(p,'wb'); assert(f>=0,'[AF7R58O] cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(f,b,'uint8');
assert(n==numel(b),'[AF7R58O] short write: %s',p);
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
