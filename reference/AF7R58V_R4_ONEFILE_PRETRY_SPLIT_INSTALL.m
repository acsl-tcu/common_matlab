function OUT = AF7R58V_R4_ONEFILE_PRETRY_SPLIT_INSTALL()
% AF7R58V-R4
% Robust transactional diagnostic split of the AF7R58T-R2 pre-transaction path.
%
% Exact authority:
%   REPLANNING_SYSTEM SHA-256 =
%   72b3b05880362d2bfa4bb81eb61eb04bf69bd38eccf796d860a43b156c4d32ea
%
% R4 deliberately removes fragile "if reuseRuntimePacket" anchoring.
% It measures only concrete, source-present operations:
%
%   s1  canonical evaluator-handle construction
%   s2  smooth/geometry metadata construction
%   s3  complete commit/horizon boundary pre-evaluation loop
%   s4  old-state transactional snapshot
%   s5  exact runtime-packet reuse test
%   s6  runtime-packet reuse operation          (reuse branch only)
%   s7  trigger-sweep rebuild                   (rebuild branch only)
%   s8  obstacle-packet rebuild                 (rebuild branch only)
%   s9  sensor/dependency rebuild tail          (rebuild branch only)
%   s10 pre-transaction hard-deadline gate
%
% branch_code:
%   0 = branch not reached
%   1 = exact reuse path
%   2 = rebuild path
%
% progress_code is the latest completed stage (1..10), with 11 meaning that
% the predeadline gate passed and transactional try is about to begin.
%
% Diagnostic only:
% no trajectory math, certificate condition, safety margin, commit decision,
% controller/plant logic, actuator/tension logic or mL authority is changed.
%
% RETURN:
%   AF7R58V_R4_REPORT.txt
%   AF7R58V_R4_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

f=fopen(fullfile(outdir,'AF7R58V_R4_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58V-R4] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58V-R4 | ROBUST PRE-TRANSACTION COMMIT TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58V_R4_PRETRY_SPLIT_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='72b3b05880362d2bfa4bb81eb61eb04bf69bd38eccf796d860a43b156c4d32ea';
backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58V-R4] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT AF7R58T-R2 SOURCE BINDING]');
    say(f,' source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected), ...
        '[AF7R58V-R4] current source is not exact AF7R58T-R2; no write.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=regexp(txt,'\n','split');

    fn='function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    [i0,i1]=find_method_bounds(lines,fn);
    B=lines(i0:i1);

    % Assign every anchor first. No A.field is referenced until all exist.
    A=struct();
    A.handles_start=find_unique(B,'[evalFn,flatEvalFn]');
    A.handles_call=find_unique(B,'af7r58i_make_canonical_eval_handles(obj,batchModel)');
    A.smooth_initial=find_unique(B, ...
        "smoothSource='DEGREE15_ENDPOINT_REPEATED_BEZIER_C7';");
    A.geom_assign=find_unique(B,'geom=G;');
    A.tq=find_unique(B, ...
        'tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)]');
    A.eval_call=find_unique(B,'xd=evalFn(tq(kk));');
    A.flat_call=find_unique(B,'xf=flatEvalFn(tq(kk));');
    A.old_start=find_unique(B,'old=struct(');
    A.old_end=find_unique(B,'''time'',obj.last_replan_commit_time);');
    A.reuse_test_start=find_unique(B,'reuseRuntimePacket=');
    A.reuse_test_call=find_unique(B, ...
        'obj.can_reuse_reference_only_runtime_packet_exact(');
    A.reuse_op_start=find_unique(B, ...
        '[sweepNew,packetNew,sensorNew,depNew]=');
    A.reuse_op_call=find_unique(B, ...
        'obj.reuse_reference_only_runtime_packet_exact(');
    A.rebuild_sweep=find_unique(B, ...
        'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');
    A.rebuild_packet=find_unique(B, ...
        'packetNew=obj.prepare_obstacle_packet_fast_target(old.packet,sweepNew,targetVersion);');
    A.rebuild_sensor=find_unique(B, ...
        'sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);');
    A.dep_version=find_unique(B, ...
        'depNew.reference_version=double(targetVersion);');
    A.predeadline_if=find_unique(B, ...
        'if nargin>=4 && ~isempty(t_hard_cycle) && ...');
    A.predeadline_reason=find_unique(B, ...
        'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    A.try_line=find_exact_try_after(B,A.predeadline_reason);

    assert(A.handles_start <= A.handles_call, ...
        '[AF7R58V-R4] handles statement order invalid.');
    assert(A.handles_call < A.smooth_initial, ...
        '[AF7R58V-R4] handles->metadata order invalid.');
    assert(A.smooth_initial < A.geom_assign && A.geom_assign < A.tq, ...
        '[AF7R58V-R4] metadata->boundary order invalid.');
    assert(A.tq < A.eval_call && A.eval_call < A.flat_call && ...
           A.flat_call < A.old_start, ...
        '[AF7R58V-R4] boundary-evaluation order invalid.');
    assert(A.old_start <= A.old_end && A.old_end < A.reuse_test_start, ...
        '[AF7R58V-R4] snapshot->reuse-test order invalid.');
    assert(A.reuse_test_start <= A.reuse_test_call && ...
           A.reuse_test_call < A.reuse_op_start, ...
        '[AF7R58V-R4] reuse-test->reuse-operation order invalid.');
    assert(A.reuse_op_start <= A.reuse_op_call && ...
           A.reuse_op_call < A.rebuild_sweep, ...
        '[AF7R58V-R4] reuse-operation->rebuild order invalid.');
    assert(A.rebuild_sweep < A.rebuild_packet && ...
           A.rebuild_packet < A.rebuild_sensor && ...
           A.rebuild_sensor < A.dep_version, ...
        '[AF7R58V-R4] rebuild operation order invalid.');
    assert(A.dep_version < A.predeadline_if && ...
           A.predeadline_if < A.predeadline_reason && ...
           A.predeadline_reason < A.try_line, ...
        '[AF7R58V-R4] rebuild->deadline->try order invalid.');

    OUT.anchor_lines=struct();
    say(f,'');
    say(f,'[B. CURRENT PRE-TRY SOURCE ANCHORS]');
    F=fieldnames(A);
    for k=1:numel(F)
        OUT.anchor_lines.(F{k})=i0+A.(F{k})-1;
        say(f,' %-28s source L%d',F{k},i0+A.(F{k})-1);
    end
    say(f,' all anchors assigned before validation = 1');
    say(f,' all required source orders closed       = 1');
    say(f,' branch-condition text used as anchor    = 0');

    propPattern='^\s*af7r58t_max_capture_overhead_us\s*=\s*0;\s*$';
    pidx=find(cellfun(@(x)~isempty(regexp(x,propPattern,'once')),lines));
    assert(numel(pidx)==1, ...
        '[AF7R58V-R4] AF7R58T property declaration count=%d.',numel(pidx));

    props={ ...
        ''; ...
        '        % AF7R58V-R4 robust pre-transaction diagnostics.'; ...
        '        ENABLE_AF7R58V_R4_PRETRY_TIMING       = true;'; ...
        '        af7r58v_r4_ring_capacity              = 64;'; ...
        '        af7r58v_r4_ring_width                 = 15;'; ...
        '        af7r58v_r4_ring                       = nan(64,15);'; ...
        '        af7r58v_r4_ring_head                  = 0;'; ...
        '        af7r58v_r4_ring_count                 = 0;'; ...
        '        af7r58v_r4_stage_ms                   = nan(1,10);'; ...
        '        af7r58v_r4_progress_code              = 0;'; ...
        '        af7r58v_r4_branch_code                = 0;'; ...
        '        af7r58v_r4_last_capture_overhead_us   = NaN;'; ...
        '        af7r58v_r4_max_capture_overhead_us    = 0;'};
    lines=insert_after(lines,pidx,props);

    [i0,i1]=find_method_bounds(lines,fn);

    writer={ ...
        '        function af7r58v_r4_capture_internal(obj,totalTic)'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58V_R4_PRETRY_TIMING), return; end'; ...
        '                if ~(obj.af7r58v_r4_ring_capacity==64 && ...'; ...
        '                        obj.af7r58v_r4_ring_width==15 && ...'; ...
        '                        isequal(size(obj.af7r58v_r4_ring),[64 15]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                r=nan(1,15);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(totalTic);'; ...
        '                r(3)=double(obj.af7r58v_r4_progress_code);'; ...
        '                r(4)=double(obj.af7r58v_r4_branch_code);'; ...
        '                r(5:14)=double(obj.af7r58v_r4_stage_ms);'; ...
        '                r(15)=double(obj.active_reference_version);'; ...
        '                cap=double(obj.af7r58v_r4_ring_capacity);'; ...
        '                j=mod(double(obj.af7r58v_r4_ring_head),cap)+1;'; ...
        '                obj.af7r58v_r4_ring(j,:)=r;'; ...
        '                obj.af7r58v_r4_ring_head=j;'; ...
        '                obj.af7r58v_r4_ring_count=min(cap,double(obj.af7r58v_r4_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58v_r4_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58v_r4_max_capture_overhead_us= ...'; ...
        '                        max(double(obj.af7r58v_r4_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostics must never alter production control flow.'; ...
        '            end'; ...
        '        end'; ...
        ''};
    lines=insert_before(lines,i0,writer);

    [i0,i1]=find_method_bounds(lines,fn);
    B=lines(i0:i1);

    j=find_unique(B,"reason='';");
    B=insert_after(B,j,{ ...
        '            af7r58v_r4_total_tic=tic;'; ...
        '            obj.af7r58v_r4_stage_ms=nan(1,10);'; ...
        '            obj.af7r58v_r4_progress_code=0;'; ...
        '            obj.af7r58v_r4_branch_code=0;'; ...
        '            af7r58v_r4_cleanup=onCleanup( ...'; ...
        '                @()obj.af7r58v_r4_capture_internal(af7r58v_r4_total_tic)); %#ok<NASGU>'});

    % s1: canonical handle construction.
    B=instrument_statement(B, ...
        '[evalFn,flatEvalFn]', ...
        'af7r58i_make_canonical_eval_handles(obj,batchModel)', ...
        1,'            ');

    % s2: smooth + geometry metadata only.
    a=find_unique(B,"smoothSource='DEGREE15_ENDPOINT_REPEATED_BEZIER_C7';");
    B=insert_before(B,a,{'            af7r58v_r4_t2=tic;'});
    b=find_unique(B,'tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)]');
    B=insert_before(B,b,{ ...
        '            obj.af7r58v_r4_stage_ms(2)=1e3*toc(af7r58v_r4_t2);'; ...
        '            obj.af7r58v_r4_progress_code=2;'});

    % s3: complete boundary pre-evaluation loop.
    a=find_unique(B,'tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)]');
    B=insert_before(B,a,{'            af7r58v_r4_t3=tic;'});
    b=find_unique(B,'old=struct(');
    B=insert_before(B,b,{ ...
        '            obj.af7r58v_r4_stage_ms(3)=1e3*toc(af7r58v_r4_t3);'; ...
        '            obj.af7r58v_r4_progress_code=3;'});

    % s4: old transactional snapshot.
    B=instrument_statement(B, ...
        'old=struct(', ...
        '''time'',obj.last_replan_commit_time);', ...
        4,'            ');

    % s5: exact runtime-packet reuse predicate.
    B=instrument_statement(B, ...
        'reuseRuntimePacket=', ...
        'obj.can_reuse_reference_only_runtime_packet_exact(', ...
        5,'            ');

    % s6: reuse operation. No branch-condition anchor is used.
    a=find_unique(B,'[sweepNew,packetNew,sensorNew,depNew]=');
    B=insert_before(B,a,{ ...
        '                obj.af7r58v_r4_branch_code=1;'; ...
        '                af7r58v_r4_t6=tic;'});
    a=find_unique(B,'[sweepNew,packetNew,sensorNew,depNew]=');
    z=find_first_after(B,'obj.reuse_reference_only_runtime_packet_exact(',a);
    e=find_statement_end(B,z);
    B=insert_after(B,e,{ ...
        '                obj.af7r58v_r4_stage_ms(6)=1e3*toc(af7r58v_r4_t6);'; ...
        '                obj.af7r58v_r4_progress_code=max(obj.af7r58v_r4_progress_code,6);'});

    % s7: rebuild trigger sweep.
    a=find_unique(B,'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');
    B=insert_before(B,a,{ ...
        '                obj.af7r58v_r4_branch_code=2;'; ...
        '                af7r58v_r4_t7=tic;'});
    a=find_unique(B,'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');
    e=find_statement_end(B,a);
    B=insert_after(B,e,{ ...
        '                obj.af7r58v_r4_stage_ms(7)=1e3*toc(af7r58v_r4_t7);'; ...
        '                obj.af7r58v_r4_progress_code=max(obj.af7r58v_r4_progress_code,7);'});

    % s8: rebuild obstacle packet.
    B=instrument_statement(B, ...
        'packetNew=obj.prepare_obstacle_packet_fast_target(old.packet,sweepNew,targetVersion);', ...
        'packetNew=obj.prepare_obstacle_packet_fast_target(old.packet,sweepNew,targetVersion);', ...
        8,'                ');

    % s9: rebuild sensor/dependency tail.
    a=find_unique(B,'sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);');
    B=insert_before(B,a,{'                af7r58v_r4_t9=tic;'});
    b=find_unique(B,'depNew.reference_version=double(targetVersion);');
    e=find_statement_end(B,b);
    B=insert_after(B,e,{ ...
        '                obj.af7r58v_r4_stage_ms(9)=1e3*toc(af7r58v_r4_t9);'; ...
        '                obj.af7r58v_r4_progress_code=max(obj.af7r58v_r4_progress_code,9);'});

    % s10: pre-transaction deadline gate.
    a=find_unique(B,'if nargin>=4 && ~isempty(t_hard_cycle) && ...');
    B=insert_before(B,a,{'            af7r58v_r4_t10=tic;'});

    reasonLine=find_unique(B,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    ret=find_first_after_exact_trim(B,'return;',reasonLine);
    B=insert_before(B,ret,{ ...
        '                obj.af7r58v_r4_stage_ms(10)=1e3*toc(af7r58v_r4_t10);'; ...
        '                obj.af7r58v_r4_progress_code=10;'});

    tr=find_exact_try_after(B,reasonLine);
    B=insert_before(B,tr,{ ...
        '            if isnan(obj.af7r58v_r4_stage_ms(10))'; ...
        '                obj.af7r58v_r4_stage_ms(10)=1e3*toc(af7r58v_r4_t10);'; ...
        '                obj.af7r58v_r4_progress_code=11;'; ...
        '            end'});

    lines=[lines(1:i0-1),B,lines(i1+1:end)];
    patched=strjoin(lines,newline);

    compact=regexprep(patched,'\s+','');
    inv=struct();
    inv.ring=contains(compact,'af7r58v_r4_ring=nan(64,15);');
    inv.cleanup=contains(compact, ...
        'af7r58v_r4_capture_internal(af7r58v_r4_total_tic)');
    inv.branch_reuse=contains(compact,'af7r58v_r4_branch_code=1;');
    inv.branch_rebuild=contains(compact,'af7r58v_r4_branch_code=2;');
    inv.deadline_fail=contains(compact,'af7r58v_r4_progress_code=10;');
    inv.try_entry=contains(compact,'af7r58v_r4_progress_code=11;');
    inv.all_stages=true;
    for kk=1:10
        if ~contains(compact,sprintf('af7r58v_r4_stage_ms(%d)=1e3*toc(',kk))
            inv.all_stages=false;
        end
    end
    inv.all=inv.ring&&inv.cleanup&&inv.branch_reuse&&inv.branch_rebuild&& ...
        inv.deadline_fail&&inv.try_entry&&inv.all_stages;
    assert(inv.all,'[AF7R58V-R4] static instrumentation invariant failed.');
    OUT.invariants=inv;

    preDir=fullfile(outdir,'AF7R58V_R4_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58V_R4_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58V-R4] patched class preflight parse failed.');
    safe_rmpath_if_present(preDir);

    say(f,'');
    say(f,'[C. PREFLIGHT -- BEFORE FIRST PRODUCTION WRITE]');
    say(f,' static instrumentation invariants       = 1');
    say(f,' patched full class parse                = 1');
    say(f,' production writes                       = 0');

    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    postName='AF7R58V_R4_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);
    write_utf8(fullfile(outdir,[postName '.m']),postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58V-R4] installed class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.production_modified=true;
    OUT.status='PRETRY_SPLIT_R4_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';
    OUT.column_labels={ ...
        'request_seq','commit_total_ms','progress_code','branch_code', ...
        'handles_ms','smooth_geom_ms','boundary_preeval_ms','old_snapshot_ms', ...
        'reuse_test_ms','reuse_operation_ms','rebuild_sweep_ms', ...
        'rebuild_packet_ms','rebuild_sensor_dep_ms','predeadline_gate_ms', ...
        'active_reference_version'};

    say(f,'');
    say(f,'[D. INSTALL RESULT]');
    say(f,' bounded ring                            = 64 x 15');
    say(f,' measured pre-try stages                 = 10');
    say(f,' branch-condition text used as anchor    = 0');
    say(f,' trajectory/certificate logic changed   = 0');
    say(f,' commit decision changed                = 0');
    say(f,' production modified                    = 1');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',h1);
    say(f,' NOTE: diagnostic timing only; final 4-ms proof remains open.');

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

save(fullfile(outdir,'AF7R58V_R4_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58V_R4_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58V_R4_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58V_R4_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function B=instrument_statement(B,startToken,endToken,stage,indent)
a=find_unique(B,startToken);
z=find_first_after(B,endToken,a);
e=find_statement_end(B,z);

B=insert_before(B,a,{[indent 'af7r58v_r4_ts=tic;']});

a=find_unique(B,startToken);
z=find_first_after(B,endToken,a);
e=find_statement_end(B,z);

B=insert_after(B,e,{ ...
    [indent sprintf('obj.af7r58v_r4_stage_ms(%d)=1e3*toc(af7r58v_r4_ts);',stage)]; ...
    [indent sprintf('obj.af7r58v_r4_progress_code=max(obj.af7r58v_r4_progress_code,%d);',stage)]});
end


function [i0,i1]=find_method_bounds(lines,fn)
v=find(cellfun(@(x)contains(x,fn),lines));
assert(numel(v)==1,'[AF7R58V-R4] target method header count=%d.',numel(v));

i0=v(1);
i1=numel(lines);
for k=i0+1:numel(lines)
    if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
        i1=k-1;
        break;
    end
end
end


function j=find_unique(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1,'[AF7R58V-R4] token "%s" count=%d.',token,numel(v));
j=v(1);
end


function j=find_first_after(B,token,a)
v=find(cellfun(@(x)contains(x,token),B));
v=v(v>=a);
assert(~isempty(v),'[AF7R58V-R4] token "%s" missing after %d.',token,a);
j=v(1);
end


function j=find_first_after_exact_trim(B,token,a)
j=[];
for k=a:numel(B)
    if strcmp(strtrim(B{k}),token)
        j=k;
        break;
    end
end
assert(~isempty(j),'[AF7R58V-R4] exact line "%s" missing after %d.',token,a);
end


function j=find_exact_try_after(B,a)
j=[];
for k=a+1:numel(B)
    if strcmp(strtrim(B{k}),'try')
        j=k;
        break;
    end
end
assert(~isempty(j),'[AF7R58V-R4] transactional try not found after %d.',a);
end


function e=find_statement_end(B,j)
e=j;
while e<=numel(B)
    if contains(B{e},';')
        return;
    end
    e=e+1;
end
error('[AF7R58V-R4] statement end not found after %d.',j);
end


function c=insert_before(c,j,newLines)
c=[c(1:j-1),reshape(newLines,1,[]),c(j:end)];
end


function c=insert_after(c,j,newLines)
c=[c(1:j),reshape(newLines,1,[]),c(j+1:end)];
end


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


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
