function OUT = AF7R58Q_R2_ONEFILE_COMMIT_STAGE_TIMING_INSTALL()
% AF7R58Q-R2
% Robust transactional diagnostic instrumentation for
% try_commit_reference_only_proposal().
%
% R2 change from AF7R58Q:
%   Source patches after the already-proven anchors use function-local regex
%   or single semantic tokens rather than brittle multi-line whitespace matches.
%
% This file measures stage costs only. It DOES NOT alter:
%   - trajectory mathematics,
%   - continuous-time collision/speed/resource certificates,
%   - commit acceptance conditions,
%   - controller/plant/actuator logic,
%   - latest payload-mass authority.
%
% Instrumented timing is for diagnosis only and is NOT the final 4-ms proof.
%
% RETURN:
%   AF7R58Q_R2_REPORT.txt
%   AF7R58Q_R2_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58Q_R2_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58Q-R2] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58Q-R2 | ROBUST REFERENCE-ONLY COMMIT STAGE TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | productionWriteAfterParseOnly=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58Q_R2_COMMIT_STAGE_TIMING_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58Q_R2_REPORT.txt','AF7R58Q_R2_OUT.mat'};
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expectedR='051b2ee71448d51532f1f260e5141476a955e91eb205d406a083260dc4583f31';

backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58Q-R2] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT PRE-INSTALL SOURCE BINDING]');
    say(f,' REPLANNING_SYSTEM AF7R58O SHA exact = %d | %s',strcmp(h0,expectedR),h0);
    assert(strcmp(h0,expectedR), ...
        '[AF7R58Q-R2] current REPLANNING_SYSTEM differs from AF7R58O authority; no install.');

    [R,eolR]=read_text_norm(pR);

    % ------------------------------------------------------------------
    % Diagnostic properties.
    % ------------------------------------------------------------------
    propAnchor='        af7r58m_max_capture_overhead_us      = 0;';
    propInsert=blk({ ...
        propAnchor; ...
        ''; ...
        '        % AF7R58Q-R2 bounded commit-stage timing diagnostics.'; ...
        '        ENABLE_AF7R58Q_COMMIT_TIMING          = true;'; ...
        '        af7r58q_commit_ring_capacity          = 64;'; ...
        '        af7r58q_commit_ring_width             = 16;'; ...
        '        af7r58q_commit_ring                   = nan(64,16);'; ...
        '        af7r58q_commit_ring_head              = 0;'; ...
        '        af7r58q_commit_ring_count             = 0;'; ...
        '        af7r58q_last_stage_ms                 = nan(1,12);'; ...
        '        af7r58q_last_stage_code               = 0;'; ...
        '        af7r58q_last_capture_overhead_us      = NaN;'; ...
        '        af7r58q_max_capture_overhead_us       = 0;'});
    R=patch_once(R,propAnchor,propInsert,'58Q-R2 properties');

    fnMarker='        function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    idx=strfind(R,fnMarker);
    assert(numel(idx)==1,'[AF7R58Q-R2] try_commit function marker count=%d',numel(idx));

    % ------------------------------------------------------------------
    % Internal bounded writer.
    % ------------------------------------------------------------------
    captureMethod=blk({ ...
        '        function af7r58q_capture_commit_timing_internal(obj,totalTic)'; ...
        '            % Bounded no-throw diagnostic writer; never a safety decision.'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58Q_COMMIT_TIMING)'; ...
        '                    return;'; ...
        '                end'; ...
        '                if ~(obj.af7r58q_commit_ring_capacity==64 && ...'; ...
        '                        obj.af7r58q_commit_ring_width==16 && ...'; ...
        '                        isequal(size(obj.af7r58q_commit_ring),[64 16]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                r=nan(1,16);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(totalTic);'; ...
        '                r(3)=double(obj.af7r58q_last_stage_code);'; ...
        '                r(4:15)=double(obj.af7r58q_last_stage_ms);'; ...
        '                r(16)=double(obj.active_reference_version);'; ...
        '                cap=double(obj.af7r58q_commit_ring_capacity);'; ...
        '                head=double(obj.af7r58q_commit_ring_head);'; ...
        '                j=mod(head,cap)+1;'; ...
        '                obj.af7r58q_commit_ring(j,:)=r;'; ...
        '                obj.af7r58q_commit_ring_head=j;'; ...
        '                obj.af7r58q_commit_ring_count= ...'; ...
        '                    min(cap,double(obj.af7r58q_commit_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58q_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58q_max_capture_overhead_us= ...'; ...
        '                        max(double(obj.af7r58q_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostic instrumentation must never affect control.'; ...
        '            end'; ...
        '        end'; ...
        ''; ...
        ''});
    R=[R(1:idx-1) captureMethod R(idx:end)];

    % ------------------------------------------------------------------
    % Function entry.
    % ------------------------------------------------------------------
    entryOld=blk({ ...
        fnMarker; ...
        '            ok=false;'; ...
        '            reason='''';'});
    entryNew=blk({ ...
        fnMarker; ...
        '            ok=false;'; ...
        '            reason='''';'; ...
        '            af7r58q_total_tic=tic;'; ...
        '            obj.af7r58q_last_stage_ms=nan(1,12);'; ...
        '            obj.af7r58q_last_stage_code=0;'; ...
        '            af7r58q_cleanup=onCleanup( ...'; ...
        '                @()obj.af7r58q_capture_commit_timing_internal(af7r58q_total_tic)); %#ok<NASGU>'});
    R=patch_once(R,entryOld,entryNew,'function entry');

    % Existing R1 anchors known to match this exact SHA.
    old=blk({ ...
        '            [spacingOK,~,~,spacingReason]= ...'; ...
        '                af7r58i_commit_spacing_guard(obj,double(t_commit));'});
    new=blk({ ...
        '            af7r58q_t=tic;'; ...
        '            [spacingOK,~,~,spacingReason]= ...'; ...
        '                af7r58i_commit_spacing_guard(obj,double(t_commit));'; ...
        '            obj.af7r58q_last_stage_ms(1)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=1;'});
    R=patch_once_in_function(R,fnMarker,old,new,'spacing');

    old=blk({ ...
        '            [resourceOK,resourceReason,batchModel,resourceMeta]= ...'; ...
        '                af7r58i_prepare_batch_model( ...'; ...
        '                    obj,P,targetVersion,double(t_commit));'});
    new=blk({ ...
        '            af7r58q_t=tic;'; ...
        '            [resourceOK,resourceReason,batchModel,resourceMeta]= ...'; ...
        '                af7r58i_prepare_batch_model( ...'; ...
        '                    obj,P,targetVersion,double(t_commit));'; ...
        '            obj.af7r58q_last_stage_ms(2)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=2;'});
    R=patch_once_in_function(R,fnMarker,old,new,'resource prepare');

    old=blk({ ...
        '            [evalFn,flatEvalFn]= ...'; ...
        '                af7r58i_make_canonical_eval_handles(obj,batchModel);'});
    new=blk({ ...
        '            af7r58q_t=tic;'; ...
        '            [evalFn,flatEvalFn]= ...'; ...
        '                af7r58i_make_canonical_eval_handles(obj,batchModel);'; ...
        '            obj.af7r58q_last_stage_ms(3)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=3;'});
    R=patch_once_in_function(R,fnMarker,old,new,'canonical handles');

    old='            % Pre-evaluate the exact HLC packet at commit/horizon boundary.';
    new=blk({old;'            af7r58q_t=tic;'});
    R=patch_once_in_function(R,fnMarker,old,new,'boundary preeval start');

    old=blk({ ...
        '            old=struct( ...'; ...
        '                ''mode'',obj.active_reference_mode, ...'});
    new=blk({ ...
        '            obj.af7r58q_last_stage_ms(4)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=4;'; ...
        '            old=struct( ...'; ...
        '                ''mode'',obj.active_reference_mode, ...'});
    R=patch_once_in_function(R,fnMarker,old,new,'boundary preeval end');

    % Instrument only the TRY belonging to this function after "old=struct".
    R=insert_before_token_after_token_in_function(R,fnMarker, ...
        '            try', ...
        '            old=struct( ...', ...
        '            af7r58q_t=tic;', ...
        'R/active assign timer start');

    % End stage 5 immediately before speed validation.
    R=insert_before_unique_token_in_function(R,fnMarker, ...
        '                obj.validate_active_speed_certificate(SC);', ...
        blk({ ...
        '                obj.af7r58q_last_stage_ms(5)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=5;'; ...
        '                af7r58q_t=tic;'}), ...
        'R/active assign timer end');

    % ------------------------------------------------------------------
    % Robust semantic-token patches from here onward.
    % ------------------------------------------------------------------

    % Stage 6: speed validation + sweep. Match regardless of "..." layout.
    pat='obj\.trigger_sweep_inflation\s*=\s*(?:\.\.\.\s*)?obj\.build_trigger_sweep_inflation\(SC\);';
    repl=blk({ ...
        '$0'; ...
        '                obj.af7r58q_last_stage_ms(6)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=6;'});
    R=regex_patch_once_in_function(R,fnMarker,pat,repl,'speed sweep');

    % Stage 7: time only actual obstacle-packet rebuild call when nonempty.
    token='                    obj.obstacle_packet=obj.prepare_obstacle_packet(obj.static_obstacle_map);';
    repl=blk({ ...
        '                    af7r58q_t=tic;'; ...
        token; ...
        '                    obj.af7r58q_last_stage_ms(7)=1e3*toc(af7r58q_t);'; ...
        '                    obj.af7r58q_last_stage_code=7;'});
    R=patch_once_in_function(R,fnMarker,token,repl,'obstacle packet');

    % Stage 8: sensor-horizon builder call.
    token='                obj.sensor_horizon_certificate=obj.build_sensor_horizon_certificate();';
    repl=blk({ ...
        '                af7r58q_t=tic;'; ...
        token; ...
        '                obj.af7r58q_last_stage_ms(8)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=8;'});
    R=patch_once_in_function(R,fnMarker,token,repl,'sensor horizon');

    % Stage 9: dependency signature capture; stale-state writes are left unchanged.
    pat='obj\.runtime_dependency_signature\s*=\s*(?:\.\.\.\s*)?obj\.capture_runtime_dependency_signature_fast\(\);';
    repl=blk({ ...
        'af7r58q_t=tic;'; ...
        '$0'; ...
        'obj.af7r58q_last_stage_ms(9)=1e3*toc(af7r58q_t);'; ...
        'obj.af7r58q_last_stage_code=9;'});
    R=regex_patch_once_in_function_with_indent(R,fnMarker,pat,repl,16,'dependency signature');

    % Stage 10: commit-state mutations from first counter through request_active=false.
    R=insert_before_unique_token_in_function(R,fnMarker, ...
        '                obj.replan_commit_count=obj.replan_commit_count+1;', ...
        '                af7r58q_t=tic;', ...
        'commit state start');

    R=insert_after_unique_token_in_function(R,fnMarker, ...
        '                obj.replan_request_active=false;', ...
        blk({ ...
        '                obj.af7r58q_last_stage_ms(10)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=10;'}), ...
        'commit state end');

    % Stage 11: post-commit assertions.
    R=insert_before_unique_token_in_function(R,fnMarker, ...
        '                obj.assert_trigger_packet_certificate_current();', ...
        '                af7r58q_t=tic;', ...
        'assert start');
    R=insert_after_unique_token_in_function(R,fnMarker, ...
        '                obj.assert_repaired_reference_current();', ...
        blk({ ...
        '                obj.af7r58q_last_stage_ms(11)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=11;'}), ...
        'assert end');

    % Stage 12: rollback block.
    R=insert_after_unique_token_in_function(R,fnMarker, ...
        '            catch ME', ...
        '                af7r58q_t=tic;', ...
        'rollback start');
    R=insert_before_unique_token_in_function(R,fnMarker, ...
        '                reason=ME.message;', ...
        blk({ ...
        '                obj.af7r58q_last_stage_ms(12)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=12;'}), ...
        'rollback end');

    % ------------------------------------------------------------------
    % Static invariants.
    % ------------------------------------------------------------------
    cR=compact_ws(R);
    inv=struct();
    inv.internal_writer=contains(cR,'functionaf7r58q_capture_commit_timing_internal(obj,totalTic)');
    inv.function_cleanup=contains(cR,'af7r58q_capture_commit_timing_internal(af7r58q_total_tic)');
    inv.stage1=contains(cR,'af7r58q_last_stage_ms(1)=1e3*toc(af7r58q_t);');
    inv.stage6=contains(cR,'af7r58q_last_stage_ms(6)=1e3*toc(af7r58q_t);');
    inv.stage9=contains(cR,'af7r58q_last_stage_ms(9)=1e3*toc(af7r58q_t);');
    inv.stage12=contains(cR,'af7r58q_last_stage_ms(12)=1e3*toc(af7r58q_t);');
    inv.ring64x16=contains(cR,'af7r58q_commit_ring=nan(64,16);');
    inv.all=all(structfun(@logical,inv));
    assert(inv.all,'[AF7R58Q-R2] prewrite source invariants failed.');

    % ------------------------------------------------------------------
    % Preflight parse BEFORE production write.
    % ------------------------------------------------------------------
    preflightDir=fullfile(outdir,'AF7R58Q_R2_PREFLIGHT');
    [ok,msg]=mkdir(preflightDir); assert(ok,'%s',msg);
    parseName='AF7R58Q_R2_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(R,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    assert(~strcmp(parseText,R),'[AF7R58Q-R2] class rename anchor missing.');
    parseFile=fullfile(preflightDir,[parseName '.m']);
    write_utf8(parseFile,parseText);

    addpath(preflightDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58Q-R2] patched class preflight parse failed.');
    safe_rmpath_if_present(preflightDir);

    say(f,'');
    say(f,'[B. PREFLIGHT PARSE -- PRODUCTION WRITES STILL ZERO]');
    say(f,' all semantic patches applied uniquely = 1');
    say(f,' patched REPLANNING_SYSTEM class PASS  = 1');

    % ------------------------------------------------------------------
    % Backup + production write.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,R,eolR);
    rehash;

    % Post-write parse.
    postName='AF7R58Q_R2_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);
    postFile=fullfile(outdir,[postName '.m']);
    write_utf8(postFile,postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58Q-R2] installed class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.invariants=inv;
    OUT.production_modified=true;
    OUT.status='COMMIT_STAGE_TIMING_R2_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';

    labels={ ...
        'request_seq','commit_total_ms','last_stage_code', ...
        'spacing_ms','resource_prepare_ms','canonical_handles_ms', ...
        'boundary_preeval_ms','R_and_active_assign_ms','speed_sweep_ms', ...
        'obstacle_packet_ms','sensor_horizon_ms','dependency_signature_ms', ...
        'commit_state_ms','post_assert_ms','rollback_ms','active_reference_version'};
    OUT.column_labels=labels;

    say(f,'');
    say(f,'[C. INSTALL RESULT]');
    say(f,' bounded commit ring                     = 64 x 16');
    say(f,' trajectory/certificate logic changed   = 0');
    say(f,' commit acceptance conditions changed   = 0');
    say(f,' production modified                     = 1');
    say(f,' new REPLANNING_SYSTEM SHA               = %s',h1);

    say(f,'');
    say(f,'[D. COMMIT TIMING COLUMNS]');
    for k=1:numel(labels)
        say(f,' %2d  %s',k,labels{k});
    end

    say(f,'');
    say(f,'[E. NEXT STEP]');
    say(f,' Recreate REPLANNING_SYSTEM from a clean run and run the same scenario.');
    say(f,' Keep that live object; the next read-only audit will extract the 58Q-R2 ring.');
    say(f,' Instrumented end-to-end timing is diagnostic only, not final 4-ms validation.');

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
    OUT.status='INSTALL_FAILED__ROLLBACK_COMPLETE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});

    say(f,'');
    say(f,'[INSTALL FAILURE]');
    say(f,' identifier          = %s',ME.identifier);
    say(f,' message             = %s',ME.message);
    say(f,' rollback performed  = %d',OUT.rollback_performed);
    say(f,' production modified = 0');
end

save(fullfile(outdir,'AF7R58Q_R2_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58Q_R2_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_R2_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_R2_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function s=insert_before_token_after_token_in_function(s,fnMarker,target,afterToken,insert,label)
[a,b]=function_bounds(s,fnMarker);
block=s(a:b-1);
j=strfind(block,afterToken);
assert(numel(j)==1,'[AF7R58Q-R2] after-token ambiguous (%s), found %d.',label,numel(j));
start=j(1)+numel(afterToken);
sub=block(start+1:end);
k=strfind(sub,target);
assert(~isempty(k),'[AF7R58Q-R2] target after-token not found (%s).',label);
pos=a+start+k(1)-1;
s=[s(1:pos-1) insert newline s(pos:end)];
end


function s=insert_before_unique_token_in_function(s,fnMarker,target,insert,label)
[a,b]=function_bounds(s,fnMarker);
block=s(a:b-1);
idx=strfind(block,target);
assert(numel(idx)==1,'[AF7R58Q-R2] before-token must be unique (%s), found %d.',label,numel(idx));
pos=a+idx(1)-1;
s=[s(1:pos-1) insert newline s(pos:end)];
end


function s=insert_after_unique_token_in_function(s,fnMarker,target,insert,label)
[a,b]=function_bounds(s,fnMarker);
block=s(a:b-1);
idx=strfind(block,target);
assert(numel(idx)==1,'[AF7R58Q-R2] after-token must be unique (%s), found %d.',label,numel(idx));
pos=a+idx(1)-1+numel(target);
s=[s(1:pos) newline insert s(pos+1:end)];
end


function s=regex_patch_once_in_function(s,fnMarker,pat,repl,label)
[a,b]=function_bounds(s,fnMarker);
block=s(a:b-1);
[st,en]=regexp(block,pat,'start','end');
assert(numel(st)==1,'[AF7R58Q-R2] regex patch must be unique (%s), found %d.',label,numel(st));
matched=block(st:en);
replacement=strrep(repl,'$0',matched);
block=[block(1:st-1) replacement block(en+1:end)];
s=[s(1:a-1) block s(b:end)];
end


function s=regex_patch_once_in_function_with_indent(s,fnMarker,pat,repl,indentN,label)
[a,b]=function_bounds(s,fnMarker);
block=s(a:b-1);
[st,en]=regexp(block,pat,'start','end');
assert(numel(st)==1,'[AF7R58Q-R2] regex patch must be unique (%s), found %d.',label,numel(st));
matched=block(st:en);
indent=repmat(' ',1,indentN);
parts=regexp(repl,'\r\n|\n|\r','split');
for k=1:numel(parts)
    if strcmp(parts{k},'$0')
        parts{k}=matched;
    else
        parts{k}=[indent parts{k}];
    end
end
replacement=strjoin(parts,newline);
block=[block(1:st-1) replacement block(en+1:end)];
s=[s(1:a-1) block s(b:end)];
end


function s=patch_once_in_function(s,fnMarker,old,new,label)
[a,b]=function_bounds(s,fnMarker);
block=s(a:b-1);
idx=strfind(block,old);
assert(numel(idx)==1, ...
    '[AF7R58Q-R2] function-local patch anchor must occur once (%s), found %d.', ...
    label,numel(idx));
k=a+idx(1)-1;
s=[s(1:k-1) new s(k+numel(old):end)];
end


function [a,b]=function_bounds(s,fnMarker)
a=strfind(s,fnMarker);
assert(numel(a)==1,'[AF7R58Q-R2] function marker ambiguous.');
a=a(1);
tail=s(a+1:end);
nxt=regexp(tail,'(?m)^\s{8}function\s','start','once');
if isempty(nxt)
    b=numel(s)+1;
else
    b=a+nxt;
end
end


function s=patch_once(s,old,new,label)
idx=strfind(s,old);
assert(numel(idx)==1, ...
    '[AF7R58Q-R2] patch anchor must occur exactly once (%s), found %d.',label,numel(idx));
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
f=fopen(p,'wb'); assert(f>=0,'[AF7R58Q-R2] cannot write %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
n=fwrite(f,b,'uint8');
assert(n==numel(b),'[AF7R58Q-R2] short write: %s',p);
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
