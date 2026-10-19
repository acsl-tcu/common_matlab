function OUT = AF7R58V_ONEFILE_PRETRY_SPLIT_INSTALL()
% AF7R58V
% Transactional diagnostic split of the AF7R58T-R2 pre-transaction region.
%
% Exact production authority:
%   REPLANNING_SYSTEM SHA-256 =
%   72b3b05880362d2bfa4bb81eb61eb04bf69bd38eccf796d860a43b156c4d32ea
%
% Purpose:
%   AF7R58U showed that most failed commit calls never reach transactional
%   "try".  This installer measures ONLY the handles->try preparation path:
%
%   p1  canonical handle construction
%   p2  smooth/geometry metadata construction
%   p3  evalFn at commit boundary
%   p4  flatEvalFn at commit boundary
%   p5  evalFn at horizon boundary
%   p6  flatEvalFn at horizon boundary
%   p7  old-state snapshot
%   p8  runtime-packet exact-reuse test
%   p9  runtime-packet reuse path
%   p10 runtime-packet rebuild path
%   p11 pre-transaction deadline gate
%
% A progress code is retained even when the function returns before "try".
%
% Diagnostic only. No trajectory math, certificate logic, commit decision,
% safety margin, controller/plant behavior, actuator/tension logic, or mL
% authority is changed.
%
% RETURN:
%   AF7R58V_REPORT.txt
%   AF7R58V_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58V_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58V] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58V | PRE-TRANSACTION COMMIT PREPARATION TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58V_PRETRY_SPLIT_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='72b3b05880362d2bfa4bb81eb61eb04bf69bd38eccf796d860a43b156c4d32ea';
backupDir=tempname(root);
backupReady=false; writesStarted=false;

try
    assert(isfile(pR),'[AF7R58V] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT AF7R58T-R2 SOURCE BINDING]');
    say(f,' source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected),'[AF7R58V] current source is not AF7R58T-R2; no write.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=regexp(txt,'\n','split');
    fn='function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    [i0,i1]=find_method_bounds(lines,fn);
    B=lines(i0:i1);

    % ------------------------------------------------------------------
    % Verify actual current pre-try semantic anchors before patching.
    % ------------------------------------------------------------------
    A=struct();
    A.handles_start=find_unique(B,'[evalFn,flatEvalFn]');
    A.handles_call=find_unique(B,'af7r58i_make_canonical_eval_handles(obj,batchModel)');
    A.smooth_source=find_unique(B,'smoothSource=');
    A.geom_assign=find_unique(B,'geom=G;');
    A.tq=find_unique(B,'tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)]');
    A.eval_call=find_unique(B,'xd=evalFn(tq(kk));');
    A.flat_call=find_unique(B,'xf=flatEvalFn(tq(kk));');
    A.old_state=find_unique(B,'old=struct(');
    A.reuse_test=find_unique(B,'reuseRuntimePacket=');
    A.reuse_if=find_unique(B,'if reuseRuntimePacket');
    A.reuse_call=find_unique(B,'obj.reuse_reference_only_runtime_packet_exact(');
    A.rebuild_sweep=find_unique(B,'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');
    A.rebuild_packet=find_unique(B,'packetNew=obj.prepare_obstacle_packet_fast_target(old.packet,sweepNew,targetVersion);');
    A.rebuild_sensor=find_unique(B,'sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);');
    A.predeadline_reason=find_unique(B,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    A.try_line=find_exact_try_after(B,A.predeadline_reason);

    assert(A.handles_start < A.handles_call && ...
           A.handles_call < A.smooth_source && ...
           A.smooth_source < A.geom_assign && ...
           A.geom_assign < A.tq && ...
           A.tq < A.eval_call && A.eval_call < A.flat_call && ...
           A.flat_call < A.old_state && ...
           A.old_state < A.reuse_test && ...
           A.reuse_test < A.reuse_if && ...
           A.reuse_call < A.rebuild_sweep && ...
           A.rebuild_sweep < A.rebuild_packet && ...
           A.rebuild_packet < A.rebuild_sensor && ...
           A.rebuild_sensor < A.predeadline_reason && ...
           A.predeadline_reason < A.try_line, ...
           '[AF7R58V] current pre-try source order is not closed.');

    OUT.anchor_lines=struct();
    say(f,'');
    say(f,'[B. CURRENT PRE-TRY SOURCE ANCHORS]');
    F=fieldnames(A);
    for k=1:numel(F)
        OUT.anchor_lines.(F{k})=i0+A.(F{k})-1;
        say(f,' %-28s source L%d',F{k},i0+A.(F{k})-1);
    end
    say(f,' all required pre-try anchors unique/order-closed = 1');

    % ------------------------------------------------------------------
    % Diagnostic properties after exact AF7R58T property declaration.
    % ------------------------------------------------------------------
    propPattern='^\s*af7r58t_max_capture_overhead_us\s*=\s*0;\s*$';
    pidx=find(cellfun(@(x)~isempty(regexp(x,propPattern,'once')),lines));
    assert(numel(pidx)==1,'[AF7R58V] AF7R58T property declaration count=%d.',numel(pidx));

    props={ ...
        ''; ...
        '        % AF7R58V pre-transaction preparation diagnostics.'; ...
        '        ENABLE_AF7R58V_PRETRY_TIMING          = true;'; ...
        '        af7r58v_ring_capacity                 = 64;'; ...
        '        af7r58v_ring_width                    = 16;'; ...
        '        af7r58v_ring                          = nan(64,16);'; ...
        '        af7r58v_ring_head                     = 0;'; ...
        '        af7r58v_ring_count                    = 0;'; ...
        '        af7r58v_stage_ms                      = nan(1,11);'; ...
        '        af7r58v_progress_code                 = 0;'; ...
        '        af7r58v_last_capture_overhead_us      = NaN;'; ...
        '        af7r58v_max_capture_overhead_us       = 0;'};
    lines=insert_after(lines,pidx,props);

    [i0,i1]=find_method_bounds(lines,fn);

    writer={ ...
        '        function af7r58v_capture_internal(obj,totalTic)'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58V_PRETRY_TIMING), return; end'; ...
        '                if ~(obj.af7r58v_ring_capacity==64 && ...'; ...
        '                        obj.af7r58v_ring_width==16 && ...'; ...
        '                        isequal(size(obj.af7r58v_ring),[64 16]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                r=nan(1,16);'; ...
        '                r(1)=double(obj.generator_request_count);'; ...
        '                r(2)=1e3*toc(totalTic);'; ...
        '                r(3)=double(obj.af7r58v_progress_code);'; ...
        '                r(4:14)=double(obj.af7r58v_stage_ms);'; ...
        '                r(15)=double(obj.active_reference_version);'; ...
        '                r(16)=double(obj.generator_deadline_fail_count);'; ...
        '                cap=double(obj.af7r58v_ring_capacity);'; ...
        '                j=mod(double(obj.af7r58v_ring_head),cap)+1;'; ...
        '                obj.af7r58v_ring(j,:)=r;'; ...
        '                obj.af7r58v_ring_head=j;'; ...
        '                obj.af7r58v_ring_count=min(cap,double(obj.af7r58v_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58v_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58v_max_capture_overhead_us=max(double(obj.af7r58v_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '            end'; ...
        '        end'; ...
        ''};
    lines=insert_before(lines,i0,writer);

    [i0,i1]=find_method_bounds(lines,fn);
    B=lines(i0:i1);

    % Function-wide cleanup capture.
    j=find_unique(B,"reason='';");
    B=insert_after(B,j,{ ...
        '            af7r58v_total_tic=tic;'; ...
        '            obj.af7r58v_stage_ms=nan(1,11);'; ...
        '            obj.af7r58v_progress_code=0;'; ...
        '            af7r58v_cleanup=onCleanup(@()obj.af7r58v_capture_internal(af7r58v_total_tic)); %#ok<NASGU>'});

    % p1: canonical handles call.
    B=instrument_statement(B,'[evalFn,flatEvalFn]', ...
        'af7r58i_make_canonical_eval_handles(obj,batchModel)',1,'            ');

    % p2: smooth + geom metadata from smoothSource to tq.
    a=find_unique(B,'smoothSource=');
    B=insert_before(B,a,{'            af7r58v_t2=tic;'});
    b=find_unique(B,'tq=[double(t_commit),double(t_commit)+double(obj.horizon_s)]');
    B=insert_before(B,b,{ ...
        '            obj.af7r58v_stage_ms(2)=1e3*toc(af7r58v_t2);'; ...
        '            obj.af7r58v_progress_code=2;'});

    % p3..p6: individual boundary evaluator calls by kk branch.
    % Wrap the two calls and route stage index by kk.
    e=find_unique(B,'xd=evalFn(tq(kk));');
    B=insert_before(B,e,{ ...
        '                af7r58v_te=tic;'});
    e=find_unique(B,'xd=evalFn(tq(kk));');
    B=insert_after(B,e,{ ...
        '                if kk==1'; ...
        '                    obj.af7r58v_stage_ms(3)=1e3*toc(af7r58v_te);'; ...
        '                    obj.af7r58v_progress_code=max(obj.af7r58v_progress_code,3);'; ...
        '                else'; ...
        '                    obj.af7r58v_stage_ms(5)=1e3*toc(af7r58v_te);'; ...
        '                    obj.af7r58v_progress_code=max(obj.af7r58v_progress_code,5);'; ...
        '                end'});

    e=find_unique(B,'xf=flatEvalFn(tq(kk));');
    B=insert_before(B,e,{'                af7r58v_tf=tic;'});
    e=find_unique(B,'xf=flatEvalFn(tq(kk));');
    B=insert_after(B,e,{ ...
        '                if kk==1'; ...
        '                    obj.af7r58v_stage_ms(4)=1e3*toc(af7r58v_tf);'; ...
        '                    obj.af7r58v_progress_code=max(obj.af7r58v_progress_code,4);'; ...
        '                else'; ...
        '                    obj.af7r58v_stage_ms(6)=1e3*toc(af7r58v_tf);'; ...
        '                    obj.af7r58v_progress_code=max(obj.af7r58v_progress_code,6);'; ...
        '                end'});

    % p7 old-state snapshot.
    B=instrument_statement(B,'old=struct(', ...
        '''time'',obj.last_replan_commit_time);',7,'            ');

    % p8 reuse exact test.
    B=instrument_statement(B,'reuseRuntimePacket=', ...
        'obj.can_reuse_reference_only_runtime_packet_exact(',8,'            ');

    % p9 reuse branch call only.
    B=instrument_statement(B,'[sweepNew,packetNew,sensorNew,depNew]=', ...
        'obj.reuse_reference_only_runtime_packet_exact(',9,'                ');

    % p10 rebuild branch from sweep through dep version assignment.
    a=find_unique(B,'sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);');
    B=insert_before(B,a,{'                af7r58v_t10=tic;'});
    b=find_unique(B,'depNew.reference_version=double(targetVersion);');
    B=insert_after(B,b,{ ...
        '                obj.af7r58v_stage_ms(10)=1e3*toc(af7r58v_t10);'; ...
        '                obj.af7r58v_progress_code=max(obj.af7r58v_progress_code,10);'});

    % p11 pre-transaction deadline gate itself.
    % Time from the start of the gate until just before try.  If it returns,
    % cleanup preserves progress=11 and the elapsed gate timing.
    a=find_unique(B,'if nargin>=4 && ~isempty(t_hard_cycle) && ...');
    B=insert_before(B,a,{'            af7r58v_t11=tic;'});
    reasonLine=find_unique(B,'REFERENCE_ONLY_COMMIT_PREPARATION_DEADLINE_EXCEEDED');
    % Record before return in the fail branch.
    ret=find_first_after_exact_trim(B,'return;',reasonLine);
    B=insert_before(B,ret,{ ...
        '                obj.af7r58v_stage_ms(11)=1e3*toc(af7r58v_t11);'; ...
        '                obj.af7r58v_progress_code=11;'});
    % Record normal passage immediately before transactional try.
    tr=find_exact_try_after(B,reasonLine);
    B=insert_before(B,tr,{ ...
        '            if isnan(obj.af7r58v_stage_ms(11))'; ...
        '                obj.af7r58v_stage_ms(11)=1e3*toc(af7r58v_t11);'; ...
        '                obj.af7r58v_progress_code=12;'; ...
        '            end'});

    lines=[lines(1:i0-1),B,lines(i1+1:end)];
    patched=strjoin(lines,newline);

    compact=regexprep(patched,'\s+','');
    assert(contains(compact,'af7r58v_ring=nan(64,16);'),'[AF7R58V] ring invariant missing.');
    assert(contains(compact,'af7r58v_progress_code=11;'),'[AF7R58V] predeadline fail progress missing.');
    assert(contains(compact,'af7r58v_progress_code=12;'),'[AF7R58V] try-entry progress missing.');

    % Parse before write.
    preDir=fullfile(outdir,'AF7R58V_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58V_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle',['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);
    addpath(preDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58V] patched class preflight parse failed.');
    safe_rmpath_if_present(preDir);

    say(f,'');
    say(f,'[C. PREFLIGHT]');
    say(f,' pre-try instrumentation invariants      = 1');
    say(f,' patched full class parse                = 1');
    say(f,' production writes before parse          = 0');

    % Transactional install.
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    postName='AF7R58V_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle',['classdef ' postName ' < handle']);
    write_utf8(fullfile(outdir,[postName '.m']),postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58V] installed class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.production_modified=true;
    OUT.status='PRETRY_SPLIT_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';
    OUT.column_labels={ ...
        'request_seq','commit_total_ms','progress_code', ...
        'handles_ms','smooth_geom_ms','eval_commit_ms','flat_commit_ms', ...
        'eval_horizon_ms','flat_horizon_ms','old_snapshot_ms','reuse_test_ms', ...
        'reuse_path_ms','rebuild_path_ms','predeadline_gate_ms', ...
        'active_reference_version','deadline_fail_count_snapshot'};

    say(f,'');
    say(f,'[D. INSTALL RESULT]');
    say(f,' bounded ring                           = 64 x 16');
    say(f,' pre-try measured stages                = 11');
    say(f,' production modified                    = 1');
    say(f,' new REPLANNING_SYSTEM SHA              = %s',h1);
    say(f,' NOTE: diagnostic timing only; final 4-ms proof remains open.');

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

save(fullfile(outdir,'AF7R58V_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58V_OUT',OUT);
say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58V_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58V_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function B=instrument_statement(B,startToken,endToken,stage,indent)
a=find_unique(B,startToken);
z=find_first_after(B,endToken,a);
e=find_statement_end(B,z);
B=insert_before(B,a,{[indent sprintf('af7r58v_ts=tic;')]});
% refind after insertion
a=find_unique(B,startToken);
z=find_first_after(B,endToken,a);
e=find_statement_end(B,z);
B=insert_after(B,e,{ ...
    [indent sprintf('obj.af7r58v_stage_ms(%d)=1e3*toc(af7r58v_ts);',stage)]; ...
    [indent sprintf('obj.af7r58v_progress_code=max(obj.af7r58v_progress_code,%d);',stage)]});
end


function [i0,i1]=find_method_bounds(lines,fn)
v=find(cellfun(@(x)contains(x,fn),lines));
assert(numel(v)==1,'[AF7R58V] target method header count=%d.',numel(v));
i0=v(1); i1=numel(lines);
for k=i0+1:numel(lines)
    if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
        i1=k-1; break;
    end
end
end


function j=find_unique(B,token)
v=find(cellfun(@(x)contains(x,token),B));
assert(numel(v)==1,'[AF7R58V] token "%s" count=%d.',token,numel(v));
j=v(1);
end


function j=find_first_after(B,token,a)
v=find(cellfun(@(x)contains(x,token),B));
v=v(v>=a);
assert(~isempty(v),'[AF7R58V] token "%s" missing after %d.',token,a);
j=v(1);
end


function j=find_first_after_exact_trim(B,token,a)
v=[];
for k=a:numel(B)
    if strcmp(strtrim(B{k}),token)
        v=k; break;
    end
end
assert(~isempty(v),'[AF7R58V] exact trimmed token "%s" missing.',token);
j=v;
end


function j=find_exact_try_after(B,a)
v=[];
for k=a+1:numel(B)
    if strcmp(strtrim(B{k}),'try')
        v=k; break;
    end
end
assert(~isempty(v),'[AF7R58V] transactional try not found.');
j=v;
end


function e=find_statement_end(B,j)
e=j;
while e<=numel(B)
    if contains(B{e},';'), return; end
    e=e+1;
end
error('[AF7R58V] statement end not found.');
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
if contains(s,sprintf('\r\n')), eol='CRLF';
elseif contains(s,sprintf('\r')), eol='CR';
else, eol='LF'; end
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function write_text_eol(p,s,eol)
switch eol
    case 'CRLF', s=strrep(s,sprintf('\n'),sprintf('\r\n'));
    case 'CR', s=strrep(s,sprintf('\n'),sprintf('\r'));
end
write_utf8(p,s);
end


function write_utf8(p,s)
f=fopen(p,'wb'); assert(f>=0,'cannot write %s',p);
cl=onCleanup(@()fclose(f)); %#ok<NASGU>
b=unicode2native(s,'UTF-8');
assert(fwrite(f,b,'uint8')==numel(b),'short write: %s',p);
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
