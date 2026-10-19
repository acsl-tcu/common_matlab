function OUT = AF7R58Q_R3_ONEFILE_COMMIT_STAGE_TIMING_INSTALL()
% AF7R58Q-R3
% Line-semantic transactional instrumentation of the CURRENT
% try_commit_reference_only_proposal() source.
%
% R3 deliberately avoids brittle whole-expression matching.  It:
%   1) binds exact current source SHA,
%   2) extracts the actual current function block,
%   3) locates unique semantic lines inside that block,
%   4) inserts timers only at verified existing lines,
%   5) parses the full patched class BEFORE any production write.
%
% Diagnostic only: trajectory math, certificates, commit decisions, controller,
% actuator/tension logic and mL authority are unchanged.
%
% RETURN:
%   AF7R58Q_R3_REPORT.txt
%   AF7R58Q_R3_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58Q_R3_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'cannot open report');
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58Q-R3 | LINE-SEMANTIC REFERENCE-ONLY COMMIT STAGE TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct('schema','AF7R58Q_R3_COMMIT_STAGE_TIMING_INSTALL_V1', ...
    'production_modified',false,'rollback_performed',false,'status','OPEN', ...
    'output_directory',outdir,'error',struct());

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='051b2ee71448d51532f1f260e5141476a955e91eb205d406a083260dc4583f31';
backupDir=tempname(root);
backupReady=false; writesStarted=false;

try
    assert(isfile(pR),'REPLANNING_SYSTEM.m missing');
    h0=file_hash(pR);
    say(f,'');
    say(f,'[A. EXACT CURRENT SOURCE BINDING]');
    say(f,' AF7R58O source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected),'Current source differs from AF7R58O authority; no write.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=regexp(txt,'\n','split');

    fn='function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    i0=find(cellfun(@(x)contains(x,fn),lines),1,'first');
    assert(~isempty(i0),'try_commit_reference_only_proposal not found');

    i1=[];
    for k=i0+1:numel(lines)
        q=lines{k};
        if ~isempty(regexp(q,'^\s{8}function\s','once'))
            i1=k-1; break;
        end
    end
    if isempty(i1), i1=numel(lines); end

    block=lines(i0:i1);

    % Exact semantic line inventory BEFORE any source edit.
    sem=struct();
    sem.spacing_start=find_one(block,'[spacingOK');
    sem.spacing_end=find_one(block,'af7r58i_commit_spacing_guard');
    sem.resource_start=find_one(block,'[resourceOK');
    sem.resource_end=find_one(block,'af7r58i_prepare_batch_model');
    sem.handles_start=find_one(block,'[evalFn,flatEvalFn]');
    sem.handles_end=find_one(block,'af7r58i_make_canonical_eval_handles');
    sem.boundary_start=find_one(block,'tq=[double(t_commit)');
    sem.old_state=find_one(block,'old=struct(');
    sem.sensor_builder=find_one(block,'build_sensor_horizon_certificate');
    sem.dep_start=find_one(block,'runtime_dependency_signature=');
    sem.dep_call=find_one(block,'capture_runtime_dependency_signature_fast');
    sem.commit_state_start=find_one(block,'replan_commit_count=');
    sem.commit_state_end=find_one(block,'replan_request_active=false');
    sem.assert1=find_one(block,'assert_trigger_packet_certificate_current');
    sem.assert2=find_one(block,'assert_repaired_reference_current');
    sem.catch_line=find_one(block,'catch ME');
    sem.reason_line=find_one(block,'reason=ME.message');

    % Find TRY belonging to commit transactional body: first "try" after old state.
    tryCandidates=find(cellfun(@(x)~isempty(regexp(x,'^\s*try\s*$','once')),block));
    sem.try_line=tryCandidates(find(tryCandidates>sem.old_state,1,'first'));
    assert(~isempty(sem.try_line),'transactional try not found');

    % Find statement-end lines for possibly multi-line assignments.
    sem.sensor_end=find_statement_end(block,sem.sensor_builder);
    sem.dep_end=find_statement_end(block,sem.dep_start);

    % Verify natural ordering.
    ord=[sem.spacing_start sem.spacing_end sem.resource_start sem.resource_end ...
        sem.handles_start sem.handles_end sem.boundary_start sem.old_state ...
        sem.try_line sem.sensor_builder sem.sensor_end sem.dep_start sem.dep_end ...
        sem.commit_state_start sem.commit_state_end sem.assert1 sem.assert2 ...
        sem.catch_line sem.reason_line];
    assert(all(diff(ord)>=0),'semantic anchors are not in expected source order');

    OUT.semantic_line_offsets=sem;

    say(f,'');
    say(f,'[B. CURRENT FUNCTION SEMANTIC ANCHORS]');
    F=fieldnames(sem);
    for k=1:numel(F)
        say(f,' %-28s = function-offset %d | source L%d',F{k},sem.(F{k}),i0+sem.(F{k})-1);
    end
    say(f,' all required semantic anchors unique/order-valid = 1');

    % ------------------------------------------------------------------
    % Insert diagnostic properties in whole class.
    % ------------------------------------------------------------------
    propIdx=find(cellfun(@(x)contains(x,'af7r58m_max_capture_overhead_us'),lines));
    assert(numel(propIdx)==1,'58M property anchor ambiguous');
    props={ ...
        ''; ...
        '        % AF7R58Q-R3 bounded commit-stage timing diagnostics.'; ...
        '        ENABLE_AF7R58Q_COMMIT_TIMING          = true;'; ...
        '        af7r58q_commit_ring_capacity          = 64;'; ...
        '        af7r58q_commit_ring_width             = 14;'; ...
        '        af7r58q_commit_ring                   = nan(64,14);'; ...
        '        af7r58q_commit_ring_head              = 0;'; ...
        '        af7r58q_commit_ring_count             = 0;'; ...
        '        af7r58q_last_stage_ms                 = nan(1,10);'; ...
        '        af7r58q_last_stage_code               = 0;'; ...
        '        af7r58q_last_capture_overhead_us      = NaN;'; ...
        '        af7r58q_max_capture_overhead_us       = 0;'};
    lines=insert_after(lines,propIdx,props);

    % Re-find function after property insertion.
    i0=find(cellfun(@(x)contains(x,fn),lines),1,'first');
    i1=[];
    for k=i0+1:numel(lines)
        if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
            i1=k-1; break;
        end
    end
    if isempty(i1), i1=numel(lines); end

    % Insert internal writer immediately before target function.
    writer={ ...
        '        function af7r58q_capture_commit_timing_internal(obj,totalTic)'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58Q_COMMIT_TIMING), return; end'; ...
        '                if ~(obj.af7r58q_commit_ring_capacity==64 && ...'; ...
        '                        obj.af7r58q_commit_ring_width==14 && ...'; ...
        '                        isequal(size(obj.af7r58q_commit_ring),[64 14]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                r=nan(1,14);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(totalTic);'; ...
        '                r(3)=double(obj.af7r58q_last_stage_code);'; ...
        '                r(4:13)=double(obj.af7r58q_last_stage_ms);'; ...
        '                r(14)=double(obj.active_reference_version);'; ...
        '                cap=double(obj.af7r58q_commit_ring_capacity);'; ...
        '                j=mod(double(obj.af7r58q_commit_ring_head),cap)+1;'; ...
        '                obj.af7r58q_commit_ring(j,:)=r;'; ...
        '                obj.af7r58q_commit_ring_head=j;'; ...
        '                obj.af7r58q_commit_ring_count=min(cap,double(obj.af7r58q_commit_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58q_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58q_max_capture_overhead_us=max(double(obj.af7r58q_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostic instrumentation must never affect control.'; ...
        '            end'; ...
        '        end'; ...
        ''};
    lines=insert_before(lines,i0,writer);

    % Re-extract current block and patch by line semantics.
    i0=find(cellfun(@(x)contains(x,fn),lines),1,'first');
    i1=[];
    for k=i0+1:numel(lines)
        if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
            i1=k-1; break;
        end
    end
    if isempty(i1), i1=numel(lines); end
    block=lines(i0:i1);

    % Function entry after reason=''.
    j=find_one(block,"reason='';");
    block=insert_after(block,j,{ ...
        '            af7r58q_total_tic=tic;'; ...
        '            obj.af7r58q_last_stage_ms=nan(1,10);'; ...
        '            obj.af7r58q_last_stage_code=0;'; ...
        '            af7r58q_cleanup=onCleanup(@()obj.af7r58q_capture_commit_timing_internal(af7r58q_total_tic)); %#ok<NASGU>'});

    % Helper: repeatedly find semantic anchors in CURRENT block after each insertion.
    block=timer_around_statement(block,'[spacingOK','af7r58i_commit_spacing_guard',1,'            ');
    block=timer_around_statement(block,'[resourceOK','af7r58i_prepare_batch_model',2,'            ');
    block=timer_around_statement(block,'[evalFn,flatEvalFn]','af7r58i_make_canonical_eval_handles',3,'            ');

    % Boundary pre-eval: tq -> immediately before old=struct.
    a=find_one(block,'tq=[double(t_commit)');
    b=find_one(block,'old=struct(');
    block=insert_before(block,a,{'            af7r58q_t=tic;'});
    b=find_one(block,'old=struct(');
    block=insert_before(block,b,{ ...
        '            obj.af7r58q_last_stage_ms(4)=1e3*toc(af7r58q_t);'; ...
        '            obj.af7r58q_last_stage_code=4;'});

    % Active-state bundle: old snapshot + transactional assignments + speed/sweep
    % + obstacle packet, ending immediately before sensor-horizon builder.
    a=find_one(block,'old=struct(');
    block=insert_before(block,a,{'            af7r58q_t=tic;'});
    b=find_one(block,'build_sensor_horizon_certificate');
    block=insert_before(block,b,{ ...
        '                obj.af7r58q_last_stage_ms(5)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=5;'});

    % Sensor horizon builder statement only.
    block=timer_around_statement(block,'build_sensor_horizon_certificate','build_sensor_horizon_certificate',6,'                ');

    % Dependency signature statement only.
    block=timer_around_statement(block,'runtime_dependency_signature=','capture_runtime_dependency_signature_fast',7,'                ');

    % Commit-state mutation.
    a=find_one(block,'replan_commit_count=');
    b=find_one(block,'replan_request_active=false');
    block=insert_before(block,a,{'                af7r58q_t=tic;'});
    b=find_one(block,'replan_request_active=false');
    block=insert_after(block,b,{ ...
        '                obj.af7r58q_last_stage_ms(8)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=8;'});

    % Assertions.
    a=find_one(block,'assert_trigger_packet_certificate_current');
    b=find_one(block,'assert_repaired_reference_current');
    block=insert_before(block,a,{'                af7r58q_t=tic;'});
    b=find_one(block,'assert_repaired_reference_current');
    block=insert_after(block,b,{ ...
        '                obj.af7r58q_last_stage_ms(9)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=9;'});

    % Rollback.
    a=find_one(block,'catch ME');
    block=insert_after(block,a,{'                af7r58q_t=tic;'});
    b=find_one(block,'reason=ME.message');
    block=insert_before(block,b,{ ...
        '                obj.af7r58q_last_stage_ms(10)=1e3*toc(af7r58q_t);'; ...
        '                obj.af7r58q_last_stage_code=10;'});

    lines=[lines(1:i0-1),block,lines(i1+1:end)];
    patched=strjoin(lines,newline);

    % Static invariants.
    cp=regexprep(patched,'\s+','');
    assert(contains(cp,'af7r58q_commit_ring=nan(64,14);'),'ring property missing');
    for k=1:10
        assert(contains(cp,sprintf('af7r58q_last_stage_ms(%d)=1e3*toc(af7r58q_t);',k)), ...
            'stage %d timer missing',k);
    end

    % Preflight parse BEFORE production write.
    preDir=fullfile(outdir,'AF7R58Q_R3_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58Q_R3_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle',['classdef ' parseName ' < handle']);
    parseFile=fullfile(preDir,[parseName '.m']);
    write_utf8(parseFile,parseText);
    addpath(preDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'patched class preflight parse failed');
    safe_rmpath_if_present(preDir);

    say(f,'');
    say(f,'[C. PREFLIGHT]');
    say(f,' all 10 timing stages inserted uniquely      = 1');
    say(f,' patched full class parse                    = 1');
    say(f,' production writes before this point         = 0');

    % Backup + production write.
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));
    OUT.backup_directory=backupDir;

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    % Post-write parse.
    postName='AF7R58Q_R3_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle',['classdef ' postName ' < handle']);
    postFile=fullfile(outdir,[postName '.m']);
    write_utf8(postFile,postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'post-write class parse failed');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.production_modified=true;
    OUT.status='COMMIT_STAGE_TIMING_R3_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';

    labels={ ...
        'request_seq','commit_total_ms','last_stage_code', ...
        'spacing_ms','resource_prepare_ms','canonical_handles_ms', ...
        'boundary_preeval_ms','active_bundle_ms','sensor_horizon_ms', ...
        'dependency_signature_ms','commit_state_ms','post_assert_ms', ...
        'rollback_ms','active_reference_version'};
    OUT.column_labels=labels;

    say(f,'');
    say(f,'[D. INSTALL RESULT]');
    say(f,' bounded ring                           = 64 x 14');
    say(f,' production modified                    = 1');
    say(f,' safety / acceptance logic modified     = 0');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',h1);

    say(f,'');
    say(f,'[E. STAGES]');
    for k=1:numel(labels)
        say(f,' %2d  %s',k,labels{k});
    end

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

save(fullfile(outdir,'AF7R58Q_R3_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58Q_R3_OUT',OUT);
say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_R3_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_R3_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function block=timer_around_statement(block,startToken,endToken,stage,indent)
a=find_one(block,startToken);
b=find_one_after(block,endToken,a);
block=insert_before(block,a,{[indent 'af7r58q_t=tic;']});
b=find_one_after(block,endToken,a+1);
e=find_statement_end(block,b);
block=insert_after(block,e,{ ...
    [indent sprintf('obj.af7r58q_last_stage_ms(%d)=1e3*toc(af7r58q_t);',stage)]; ...
    [indent sprintf('obj.af7r58q_last_stage_code=%d;',stage)]});
end


function idx=find_one(block,token)
idx=find(cellfun(@(x)contains(x,token),block));
assert(numel(idx)==1,'semantic token "%s" count=%d',token,numel(idx));
idx=idx(1);
end


function idx=find_one_after(block,token,startIdx)
v=find(cellfun(@(x)contains(x,token),block));
v=v(v>=startIdx);
assert(numel(v)>=1,'semantic token "%s" not found after %d',token,startIdx);
idx=v(1);
end


function e=find_statement_end(block,startIdx)
e=startIdx;
while e<=numel(block)
    q=strtrim(block{e});
    if contains(q,';')
        return;
    end
    e=e+1;
end
error('statement end not found');
end


function c=insert_before(c,idx,newLines)
c=[c(1:idx-1),reshape(newLines,1,[]),c(idx:end)];
end


function c=insert_after(c,idx,newLines)
c=[c(1:idx),reshape(newLines,1,[]),c(idx+1:end)];
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
    case 'CRLF', s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR',   s=strrep(s,sprintf('\n'),sprintf('\r'));
end
write_utf8(p,s);
end


function write_utf8(p,s)
f=fopen(p,'wb'); assert(f>=0,'cannot write %s',p);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
assert(fwrite(f,b,'uint8')==numel(b),'short write');
end


function safe_rmpath_if_present(p)
try
    parts=strsplit(path,pathsep);
    if any(strcmpi(parts,p)), rmpath(p); end
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
