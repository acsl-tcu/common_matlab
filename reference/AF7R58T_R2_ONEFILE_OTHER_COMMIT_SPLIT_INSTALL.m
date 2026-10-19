function OUT = AF7R58T_R2_ONEFILE_OTHER_COMMIT_SPLIT_INSTALL()
% AF7R58T
% Transactional diagnostic split of AF7R58Q-R5 "OTHER_COMMIT".
%
% R2 change:
%   The commit-state anchor is the exact increment statement
%   obj.replan_commit_count=obj.replan_commit_count+1;
%   so rollback restoration obj.replan_commit_count=old.count; cannot match.
%
% This installer is bound to the exact AF7R58Q-R5 production SHA and uses only
% semantic anchors that must actually exist in the CURRENT
% try_commit_reference_only_proposal() function. No sensor/packet substage is
% assumed by name.
%
% Measured OTHER sub-stages:
%   1  entry_to_spacing
%   2  spacing_to_resource
%   3  resource_to_handles
%   4  post_handles_to_try
%   5  try_to_canonical_store
%   6  canonical_store_to_active_assign
%   7  active_assign_to_commit_state
%   8  commit_state
%   9  post_commit_assertions
%  10  final_deadline_success_path
%  11  rollback_path
%
% Diagnostic instrumentation only. No trajectory math, certificate condition,
% commit decision, controller/plant logic, actuator/tension logic, or mL
% authority is changed.
%
% Instrumented wall time is NOT final 4-ms validation.
%
% RETURN:
%   AF7R58T_R2_REPORT.txt
%   AF7R58T_R2_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58T_R2_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58T-R2] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58T-R2 | OTHER_COMMIT SOURCE-BOUND SUBSTAGE TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58T_R2_OTHER_COMMIT_SPLIT_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58T_R2_REPORT.txt','AF7R58T_R2_OUT.mat'};
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='1474c591fe2227f461eb43c2853d61909e63e9bef9bf6a40a851956c1c02e440';
backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58T-R2] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT AF7R58Q-R5 SOURCE BINDING]');
    say(f,' source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected), ...
        '[AF7R58T-R2] current REPLANNING_SYSTEM is not AF7R58Q-R5; no write.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=regexp(txt,'\n','split');

    fn='function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    [i0,i1]=find_method_bounds(lines,fn);
    B=lines(i0:i1);

    % ------------------------------------------------------------------
    % Current-source semantic inventory. Every required anchor must be
    % present uniquely before we even build patched source.
    % ------------------------------------------------------------------
    A=struct();
    A.first_gate=find_unique(B,'if ~obj.ENABLE_AUTO_REPLAN_COMMIT');
    A.spacing_start=find_unique(B,'[spacingOK');
    A.spacing_call=find_unique(B,'af7r58i_commit_spacing_guard(obj,double(t_commit))');
    A.resource_start=find_unique(B,'[resourceOK');
    A.resource_call=find_unique(B,'af7r58i_prepare_batch_model(');
    A.handles_start=find_unique(B,'[evalFn,flatEvalFn]');
    A.handles_call=find_unique(B,'af7r58i_make_canonical_eval_handles(obj,batchModel)');
    A.canonical_store=find_unique(B,'R.batch_flat_model=batchModel;');
    A.active_assign=find_unique(B,'obj.active_repaired_reference=R;');
    A.commit_state_start=find_unique(B,'obj.replan_commit_count=obj.replan_commit_count+1;');
    A.commit_state_end=find_unique(B,'obj.replan_request_active=false;');
    A.assert1=find_unique(B,'obj.assert_trigger_packet_certificate_current();');
    A.assert2=find_unique(B,'obj.assert_repaired_reference_current();');
    A.catch_line=find_unique(B,'catch ME');
    A.rollback_end=find_unique(B,'reason=ME.message;');

    % Transactional try = first exact "try" after handles and before store.
    tryIdx=find(cellfun(@(x)~isempty(regexp(x,'^\s*try\s*$','once')),B));
    tryIdx=tryIdx(tryIdx>A.handles_call & tryIdx<A.canonical_store);
    assert(numel(tryIdx)==1,'[AF7R58T-R2] transactional try count=%d.',numel(tryIdx));
    A.try_line=tryIdx(1);

    % Success-path endpoint before catch. Prefer ok=true if unique.
    okv=find(cellfun(@(x)contains(x,'ok=true;'),B));
    okv=okv(okv>A.assert2 & okv<A.catch_line);
    assert(numel(okv)==1,'[AF7R58T-R2] success ok=true anchor count=%d.',numel(okv));
    A.success_ok=okv(1);

    % Check strict source ordering.
    order=[A.first_gate,A.spacing_start,A.spacing_call,A.resource_start,A.resource_call, ...
        A.handles_start,A.handles_call,A.try_line,A.canonical_store,A.active_assign, ...
        A.commit_state_start,A.commit_state_end,A.assert1,A.assert2,A.success_ok, ...
        A.catch_line,A.rollback_end];
    assert(all(diff(order)>0),'[AF7R58T-R2] semantic source order is not strictly closed.');

    OUT.source_anchor_lines=struct();
    F=fieldnames(A);
    say(f,'');
    say(f,'[B. CURRENT COMMIT SOURCE ANCHORS]');
    for k=1:numel(F)
        OUT.source_anchor_lines.(F{k})=i0+A.(F{k})-1;
        say(f,' %-28s source L%d',F{k},i0+A.(F{k})-1);
    end
    say(f,' all required anchors unique/order-closed = 1');
    say(f,' optional sensor/packet-name assumption   = 0');

    % ------------------------------------------------------------------
    % Add AF7R58T diagnostic properties after exact R5 property declaration.
    % ------------------------------------------------------------------
    propPattern='^\s*af7r58q_min_max_capture_overhead_us\s*=\s*0;\s*$';
    pidx=find(cellfun(@(x)~isempty(regexp(x,propPattern,'once')),lines));
    assert(numel(pidx)==1,'[AF7R58T-R2] R5 property declaration anchor count=%d.',numel(pidx));

    props={ ...
        ''; ...
        '        % AF7R58T OTHER_COMMIT substage diagnostics.'; ...
        '        ENABLE_AF7R58T_OTHER_TIMING           = true;'; ...
        '        af7r58t_ring_capacity                 = 64;'; ...
        '        af7r58t_ring_width                    = 15;'; ...
        '        af7r58t_ring                          = nan(64,15);'; ...
        '        af7r58t_ring_head                     = 0;'; ...
        '        af7r58t_ring_count                    = 0;'; ...
        '        af7r58t_stage_ms                      = nan(1,11);'; ...
        '        af7r58t_last_capture_overhead_us      = NaN;'; ...
        '        af7r58t_max_capture_overhead_us       = 0;'};
    lines=insert_after(lines,pidx,props);

    % Re-find target after property insertion.
    [i0,i1]=find_method_bounds(lines,fn);

    % ------------------------------------------------------------------
    % Internal bounded no-throw writer.
    % ------------------------------------------------------------------
    writer={ ...
        '        function af7r58t_capture_internal(obj,totalTic)'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58T_OTHER_TIMING), return; end'; ...
        '                if ~(obj.af7r58t_ring_capacity==64 && ...'; ...
        '                        obj.af7r58t_ring_width==15 && ...'; ...
        '                        isequal(size(obj.af7r58t_ring),[64 15]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                s=double(obj.af7r58t_stage_ms);'; ...
        '                r=nan(1,15);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(totalTic);'; ...
        '                r(3:13)=s;'; ...
        '                r(14)=double(obj.active_reference_version);'; ...
        '                r(15)=double(obj.replan_commit_count);'; ...
        '                cap=double(obj.af7r58t_ring_capacity);'; ...
        '                j=mod(double(obj.af7r58t_ring_head),cap)+1;'; ...
        '                obj.af7r58t_ring(j,:)=r;'; ...
        '                obj.af7r58t_ring_head=j;'; ...
        '                obj.af7r58t_ring_count=min(cap,double(obj.af7r58t_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58t_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58t_max_capture_overhead_us= ...'; ...
        '                        max(double(obj.af7r58t_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostic capture must never alter control flow.'; ...
        '            end'; ...
        '        end'; ...
        ''};
    lines=insert_before(lines,i0,writer);

    % Re-extract target.
    [i0,i1]=find_method_bounds(lines,fn);
    B=lines(i0:i1);

    % Existing R5 function-wide timing initialization occurs after reason=''.
    % Add T cleanup there so ALL commit-path exits are captured.
    j=find_unique(B,"reason='';");
    B=insert_after(B,j,{ ...
        '            af7r58t_total_tic=tic;'; ...
        '            obj.af7r58t_stage_ms=nan(1,11);'; ...
        '            af7r58t_cleanup=onCleanup(@()obj.af7r58t_capture_internal(af7r58t_total_tic)); %#ok<NASGU>'});

    % Stage 1: entry gates/prechecks -> spacing statement.
    B=instrument_region(B, ...
        'if ~obj.ENABLE_AUTO_REPLAN_COMMIT', ...
        '[spacingOK',1,'            ');

    % Stage 2: after spacing statement -> resource statement.
    B=instrument_between_statements(B, ...
        '[spacingOK','af7r58i_commit_spacing_guard(obj,double(t_commit))', ...
        '[resourceOK',2,'            ');

    % Stage 3: after resource statement -> handles statement.
    B=instrument_between_statements(B, ...
        '[resourceOK','af7r58i_prepare_batch_model(', ...
        '[evalFn,flatEvalFn]',3,'            ');

    % Stage 4: after handles statement -> transactional try.
    B=instrument_after_statement_to_token(B, ...
        '[evalFn,flatEvalFn]','af7r58i_make_canonical_eval_handles(obj,batchModel)', ...
        'try',4,'            ',true);

    % Stage 5: inside try -> canonical model store (inclusive).
    B=instrument_try_to_statement(B, ...
        'af7r58i_make_canonical_eval_handles(obj,batchModel)', ...
        'R.batch_flat_model=batchModel;',5);

    % Stage 6: canonical store -> active assignment.
    B=instrument_after_statement_to_token(B, ...
        'R.batch_flat_model=batchModel;','R.batch_flat_model=batchModel;', ...
        'obj.active_repaired_reference=R;',6,'                ',false);

    % Stage 7: active assignment -> commit-state counter.
    B=instrument_region(B, ...
        'obj.active_repaired_reference=R;', ...
        'obj.replan_commit_count=obj.replan_commit_count+1;',7,'                ');

    % Stage 8: commit-state block.
    B=instrument_region_inclusive(B, ...
        'obj.replan_commit_count=obj.replan_commit_count+1;', ...
        'obj.replan_request_active=false;',8,'                ');

    % Stage 9: post-commit assertions.
    B=instrument_region_inclusive(B, ...
        'obj.assert_trigger_packet_certificate_current();', ...
        'obj.assert_repaired_reference_current();',9,'                ');

    % Stage 10: final deadline/success path.
    % Start after second assertion; normal path ends immediately before ok=true.
    a=find_unique(B,'obj.assert_repaired_reference_current();');
    B=insert_after(B,a,{'                af7r58t_t10=tic;'});
    b=find_unique(B,'ok=true;');
    B=insert_before(B,b,{ ...
        '                obj.af7r58t_stage_ms(10)=1e3*toc(af7r58t_t10);'});

    % If deadline error jumps to catch before normal stage-10 stop, capture
    % elapsed final-path time on catch entry before rollback timer starts.
    c=find_unique(B,'catch ME');
    B=insert_after(B,c,{ ...
        '                if isnan(obj.af7r58t_stage_ms(10)) && exist(''af7r58t_t10'',''var'')'; ...
        '                    obj.af7r58t_stage_ms(10)=1e3*toc(af7r58t_t10);'; ...
        '                end'; ...
        '                af7r58t_t11=tic;'});

    % Stage 11: rollback body until reason assignment.
    r=find_unique(B,'reason=ME.message;');
    B=insert_before(B,r,{ ...
        '                obj.af7r58t_stage_ms(11)=1e3*toc(af7r58t_t11);'});

    lines=[lines(1:i0-1),B,lines(i1+1:end)];
    patched=strjoin(lines,newline);

    % ------------------------------------------------------------------
    % Static diagnostic-source invariants.
    % ------------------------------------------------------------------
    compact=regexprep(patched,'\s+','');
    inv=struct();
    inv.writer=contains(compact,'functionaf7r58t_capture_internal(obj,totalTic)');
    inv.cleanup=contains(compact,'af7r58t_capture_internal(af7r58t_total_tic)');
    inv.ring=contains(compact,'af7r58t_ring=nan(64,15);');
    inv.all_stages=true;
    for k=1:11
        if ~contains(compact,sprintf('af7r58t_stage_ms(%d)=1e3*toc(',k))
            inv.all_stages=false;
        end
    end
    inv.all=inv.writer&&inv.cleanup&&inv.ring&&inv.all_stages;
    assert(inv.all,'[AF7R58T-R2] prewrite instrumentation invariant failed.');
    OUT.invariants=inv;

    % ------------------------------------------------------------------
    % Full class parse BEFORE first production write.
    % ------------------------------------------------------------------
    preDir=fullfile(outdir,'AF7R58T_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58T_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58T-R2] patched full class preflight parse failed.');
    safe_rmpath_if_present(preDir);

    say(f,'');
    say(f,'[C. PREFLIGHT -- BEFORE PRODUCTION WRITE]');
    say(f,' all 11 stage inserts present           = 1');
    say(f,' patched full class parse               = 1');
    say(f,' production writes                      = 0');

    % ------------------------------------------------------------------
    % Transactional production install.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    postName='AF7R58T_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);
    write_utf8(fullfile(outdir,[postName '.m']),postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58T-R2] installed full class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.production_modified=true;
    OUT.status='OTHER_COMMIT_SPLIT_R2_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';
    OUT.column_labels={ ...
        'request_seq','commit_total_ms', ...
        'entry_to_spacing_ms','spacing_to_resource_ms','resource_to_handles_ms', ...
        'post_handles_to_try_ms','try_to_canonical_store_ms', ...
        'canonical_store_to_active_assign_ms','active_assign_to_commit_state_ms', ...
        'commit_state_ms','post_commit_assertions_ms', ...
        'final_deadline_success_path_ms','rollback_path_ms', ...
        'active_reference_version','replan_commit_count'};

    say(f,'');
    say(f,'[D. INSTALL RESULT]');
    say(f,' bounded ring                           = 64 x 15');
    say(f,' OTHER substage count                   = 11');
    say(f,' trajectory/certificate logic changed  = 0');
    say(f,' commit acceptance logic changed        = 0');
    say(f,' production modified                    = 1');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',h1);
    say(f,' NOTE: timing is diagnostic only; final 4-ms proof requires instrumentation removal.');

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

save(fullfile(outdir,'AF7R58T_R2_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58T_R2_OUT',OUT);
say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58T_R2_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58T_R2_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function B=instrument_region(B,startToken,endToken,stage,indent)
a=find_unique(B,startToken);
b=find_unique(B,endToken);
assert(a<b,'[AF7R58T-R2] region order invalid.');
B=insert_before(B,a,{[indent sprintf('af7r58t_t%d=tic;',stage)]});
b=find_unique(B,endToken);
B=insert_before(B,b,{[indent sprintf('obj.af7r58t_stage_ms(%d)=1e3*toc(af7r58t_t%d);',stage,stage)]});
end


function B=instrument_region_inclusive(B,startToken,endToken,stage,indent)
a=find_unique(B,startToken);
b=find_unique(B,endToken);
assert(a<=b,'[AF7R58T-R2] inclusive region order invalid.');
B=insert_before(B,a,{[indent sprintf('af7r58t_t%d=tic;',stage)]});
b=find_unique(B,endToken);
e=find_statement_end(B,b);
B=insert_after(B,e,{[indent sprintf('obj.af7r58t_stage_ms(%d)=1e3*toc(af7r58t_t%d);',stage,stage)]});
end


function B=instrument_between_statements(B,startToken,startEndToken,nextToken,stage,indent)
a=find_unique(B,startToken);
z=find_unique(B,startEndToken);
assert(a<=z,'[AF7R58T-R2] statement anchor order invalid.');
e=find_statement_end(B,z);
B=insert_after(B,e,{[indent sprintf('af7r58t_t%d=tic;',stage)]});
b=find_unique(B,nextToken);
B=insert_before(B,b,{[indent sprintf('obj.af7r58t_stage_ms(%d)=1e3*toc(af7r58t_t%d);',stage,stage)]});
end


function B=instrument_after_statement_to_token(B,startToken,startEndToken,endToken,stage,indent,endIsExactTry)
a=find_unique(B,startToken);
z=find_unique(B,startEndToken);
assert(a<=z,'[AF7R58T-R2] after-statement anchor order invalid.');
e=find_statement_end(B,z);
B=insert_after(B,e,{[indent sprintf('af7r58t_t%d=tic;',stage)]});

if endIsExactTry
    cand=find(cellfun(@(x)~isempty(regexp(x,'^\s*try\s*$','once')),B));
    cand=cand(cand>e);
    assert(~isempty(cand),'[AF7R58T-R2] exact try not found.');
    b=cand(1);
else
    b=find_unique(B,endToken);
end
B=insert_before(B,b,{[indent sprintf('obj.af7r58t_stage_ms(%d)=1e3*toc(af7r58t_t%d);',stage,stage)]});
end


function B=instrument_try_to_statement(B,afterToken,endToken,stage)
h=find_unique(B,afterToken);
cand=find(cellfun(@(x)~isempty(regexp(x,'^\s*try\s*$','once')),B));
cand=cand(cand>h);
assert(~isempty(cand),'[AF7R58T-R2] transactional try not found.');
tr=cand(1);
B=insert_after(B,tr,{sprintf('                af7r58t_t%d=tic;',stage)});
z=find_unique(B,endToken);
e=find_statement_end(B,z);
B=insert_after(B,e,{sprintf('                obj.af7r58t_stage_ms(%d)=1e3*toc(af7r58t_t%d);',stage,stage)});
end


function [i0,i1]=find_method_bounds(lines,fn)
v=find(cellfun(@(x)contains(x,fn),lines));
assert(numel(v)==1,'[AF7R58T-R2] target method header count=%d.',numel(v));
i0=v(1); i1=numel(lines);
for k=i0+1:numel(lines)
    if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
        i1=k-1;
        break;
    end
end
end


function j=find_unique(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1,'[AF7R58T-R2] semantic token "%s" count=%d.',token,numel(v));
j=v(1);
end


function e=find_statement_end(B,j)
e=j;
while e<=numel(B)
    if contains(B{e},';')
        return;
    end
    e=e+1;
end
error('[AF7R58T-R2] statement end not found.');
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
    case 'CRLF', s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR',   s=strrep(s,sprintf('\n'),sprintf('\r'));
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
