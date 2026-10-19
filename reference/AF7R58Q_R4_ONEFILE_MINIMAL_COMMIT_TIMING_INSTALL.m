function OUT = AF7R58Q_R4_ONEFILE_MINIMAL_COMMIT_TIMING_INSTALL()
% AF7R58Q-R4
% Minimal source-proven instrumentation of REFERENCE_ONLY commit.
%
% Only four timing facts are collected:
%   total commit-function wall time
%   commit-spacing guard time
%   canonical resource-prepare time
%   canonical evaluator-handle construction time
%
% These anchors are already source-proven in the current AF7R58O + AF7R58I
% production chain.  No optional/assumed commit substage is patched.
%
% Diagnostic only. No trajectory/certificate/commit decision is changed.
%
% RETURN:
%   AF7R58Q_R4_REPORT.txt
%   AF7R58Q_R4_OUT.mat

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
f=fopen(fullfile(outdir,'AF7R58Q_R4_REPORT.txt'),'wt','n','UTF-8');
assert(f>=0,'[AF7R58Q-R4] cannot open report.');
cleanupF=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58Q-R4 | MINIMAL SOURCE-PROVEN REFERENCE-ONLY COMMIT TIMING INSTALL');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] diagnosticOnly=1 | boundedRing=1 | rollbackOnFailure=1 | parseBeforeWrite=1');
say(f,'[SAFETY] trajectoryMathChanged=0 | certificateLogicChanged=0 | commitDecisionChanged=0 | mLTouched=0');
say(f,' root = %s',root);

OUT=struct();
OUT.schema='AF7R58Q_R4_MINIMAL_COMMIT_TIMING_INSTALL_V1';
OUT.production_modified=false;
OUT.rollback_performed=false;
OUT.status='OPEN';
OUT.output_directory=outdir;
OUT.required_return_files={'AF7R58Q_R4_REPORT.txt','AF7R58Q_R4_OUT.mat'};
OUT.error=struct();

pR=fullfile(root,'REPLANNING_SYSTEM.m');
expected='051b2ee71448d51532f1f260e5141476a955e91eb205d406a083260dc4583f31';
backupDir=tempname(root);
backupReady=false;
writesStarted=false;

try
    assert(isfile(pR),'[AF7R58Q-R4] REPLANNING_SYSTEM.m missing.');
    h0=file_hash(pR);

    say(f,'');
    say(f,'[A. EXACT CURRENT SOURCE BINDING]');
    say(f,' AF7R58O source SHA exact = %d | %s',strcmp(h0,expected),h0);
    assert(strcmp(h0,expected), ...
        '[AF7R58Q-R4] current source differs from AF7R58O authority; no write.');

    [txt,eolStyle]=read_text_norm(pR);
    lines=regexp(txt,'\n','split');

    fnToken='function [ok,reason] = try_commit_reference_only_proposal(obj,P,t_commit,t_hard_cycle)';
    [i0,i1]=find_method_bounds(lines,fnToken);
    block=lines(i0:i1);

    % ------------------------------------------------------------------
    % Prove ONLY the four required anchors from the actual current block.
    % ------------------------------------------------------------------
    A=struct();
    A.function_start=1;
    A.reason=find_unique(block,"reason='';");
    A.spacing=find_unique(block,'af7r58i_commit_spacing_guard(obj,double(t_commit))');
    A.resource=find_unique(block,'af7r58i_prepare_batch_model(');
    A.handles=find_unique(block,'af7r58i_make_canonical_eval_handles(obj,batchModel)');
    A.canonical_store=find_unique(block,'R.batch_flat_model=batchModel;');

    assert(A.reason < A.spacing && A.spacing < A.resource && ...
        A.resource < A.handles && A.handles < A.canonical_store, ...
        '[AF7R58Q-R4] proven anchor order is not current.');

    say(f,'');
    say(f,'[B. CURRENT COMMIT FUNCTION -- PROVEN ANCHORS ONLY]');
    say(f,' reason initialization                 source L%d',i0+A.reason-1);
    say(f,' commit-spacing guard                 source L%d',i0+A.spacing-1);
    say(f,' canonical resource prepare           source L%d',i0+A.resource-1);
    say(f,' canonical evaluator handles          source L%d',i0+A.handles-1);
    say(f,' canonical model store                source L%d',i0+A.canonical_store-1);
    say(f,' optional sensor/packet stage assumed = 0');
    say(f,' anchor conjunction/order closed       = 1');

    OUT.source_anchor_lines=struct( ...
        'function',i0, ...
        'reason',i0+A.reason-1, ...
        'spacing',i0+A.spacing-1, ...
        'resource',i0+A.resource-1, ...
        'handles',i0+A.handles-1, ...
        'canonical_store',i0+A.canonical_store-1);

    % ------------------------------------------------------------------
    % Add bounded diagnostic properties at the already-installed AF7R58M
    % diagnostic property block.  Exact SHA makes this source-bound.
    % ------------------------------------------------------------------
    pidx=find(cellfun(@(x)contains(x,'af7r58m_max_capture_overhead_us'),lines));
    assert(numel(pidx)==1,'[AF7R58Q-R4] AF7R58M property anchor count=%d.',numel(pidx));

    props={ ...
        ''; ...
        '        % AF7R58Q-R4 minimal commit timing diagnostics.'; ...
        '        ENABLE_AF7R58Q_MIN_TIMING             = true;'; ...
        '        af7r58q_min_ring_capacity             = 64;'; ...
        '        af7r58q_min_ring_width                = 9;'; ...
        '        af7r58q_min_ring                      = nan(64,9);'; ...
        '        af7r58q_min_ring_head                 = 0;'; ...
        '        af7r58q_min_ring_count                = 0;'; ...
        '        af7r58q_min_stage_ms                  = nan(1,3);'; ...
        '        af7r58q_min_last_capture_overhead_us  = NaN;'; ...
        '        af7r58q_min_max_capture_overhead_us   = 0;'};
    lines=insert_after(lines,pidx,props);

    % Re-find method after property insertion.
    [i0,i1]=find_method_bounds(lines,fnToken);

    % ------------------------------------------------------------------
    % Internal no-throw ring writer.
    % ------------------------------------------------------------------
    writer={ ...
        '        function af7r58q_min_capture_internal(obj,totalTic)'; ...
        '            tcap=tic;'; ...
        '            try'; ...
        '                if ~logical(obj.ENABLE_AF7R58Q_MIN_TIMING), return; end'; ...
        '                if ~(obj.af7r58q_min_ring_capacity==64 && ...'; ...
        '                        obj.af7r58q_min_ring_width==9 && ...'; ...
        '                        isequal(size(obj.af7r58q_min_ring),[64 9]))'; ...
        '                    return;'; ...
        '                end'; ...
        '                st=double(obj.af7r58q_min_stage_ms);'; ...
        '                total_ms=1e3*toc(totalTic);'; ...
        '                known=sum(st(isfinite(st)));'; ...
        '                residual=total_ms-known;'; ...
        '                r=[double(obj.generator_request_count), total_ms, ...'; ...
        '                    st(1),st(2),st(3),residual, ...'; ...
        '                    double(obj.active_reference_version), ...'; ...
        '                    double(obj.replan_commit_count), ...'; ...
        '                    double(obj.generator_deadline_fail_count)];'; ...
        '                cap=double(obj.af7r58q_min_ring_capacity);'; ...
        '                j=mod(double(obj.af7r58q_min_ring_head),cap)+1;'; ...
        '                obj.af7r58q_min_ring(j,:)=r;'; ...
        '                obj.af7r58q_min_ring_head=j;'; ...
        '                obj.af7r58q_min_ring_count=min(cap,double(obj.af7r58q_min_ring_count)+1);'; ...
        '                us=1e6*toc(tcap);'; ...
        '                obj.af7r58q_min_last_capture_overhead_us=us;'; ...
        '                if isfinite(us)'; ...
        '                    obj.af7r58q_min_max_capture_overhead_us= ...'; ...
        '                        max(double(obj.af7r58q_min_max_capture_overhead_us),us);'; ...
        '                end'; ...
        '            catch'; ...
        '                % Diagnostic instrumentation must never change control flow.'; ...
        '            end'; ...
        '        end'; ...
        ''};
    lines=insert_before(lines,i0,writer);

    % Re-find and extract after writer insertion.
    [i0,i1]=find_method_bounds(lines,fnToken);
    block=lines(i0:i1);

    % ------------------------------------------------------------------
    % Total timer at function entry. This records ALL ordinary/early returns.
    % ------------------------------------------------------------------
    j=find_unique(block,"reason='';");
    block=insert_after(block,j,{ ...
        '            af7r58q_min_total_tic=tic;'; ...
        '            obj.af7r58q_min_stage_ms=nan(1,3);'; ...
        '            af7r58q_min_cleanup=onCleanup( ...'; ...
        '                @()obj.af7r58q_min_capture_internal(af7r58q_min_total_tic)); %#ok<NASGU>'});

    % ------------------------------------------------------------------
    % Stage 1: spacing guard statement.
    % ------------------------------------------------------------------
    block=instrument_statement(block, ...
        '[spacingOK', ...
        'af7r58i_commit_spacing_guard(obj,double(t_commit))', ...
        1,'            ');

    % Stage 2: resource prepare statement.
    block=instrument_statement(block, ...
        '[resourceOK', ...
        'af7r58i_prepare_batch_model(', ...
        2,'            ');

    % Stage 3: canonical handle statement.
    block=instrument_statement(block, ...
        '[evalFn,flatEvalFn]', ...
        'af7r58i_make_canonical_eval_handles(obj,batchModel)', ...
        3,'            ');

    % Replace only current target method.
    lines=[lines(1:i0-1),block,lines(i1+1:end)];
    patched=strjoin(lines,newline);

    % ------------------------------------------------------------------
    % Source invariants before parse/write.
    % ------------------------------------------------------------------
    c=regexprep(patched,'\s+','');
    inv=struct();
    inv.writer=contains(c,'functionaf7r58q_min_capture_internal(obj,totalTic)');
    inv.cleanup=contains(c,'af7r58q_min_capture_internal(af7r58q_min_total_tic)');
    inv.spacing=contains(c,'af7r58q_min_stage_ms(1)=1e3*toc(af7r58q_min_t);');
    inv.resource=contains(c,'af7r58q_min_stage_ms(2)=1e3*toc(af7r58q_min_t);');
    inv.handles=contains(c,'af7r58q_min_stage_ms(3)=1e3*toc(af7r58q_min_t);');
    inv.ring=contains(c,'af7r58q_min_ring=nan(64,9);');
    inv.all=all(structfun(@logical,inv));
    assert(inv.all,'[AF7R58Q-R4] source invariant failed before parse.');
    OUT.invariants=inv;

    % ------------------------------------------------------------------
    % Parse the complete patched class BEFORE writing production.
    % ------------------------------------------------------------------
    preDir=fullfile(outdir,'AF7R58Q_R4_PREFLIGHT');
    [ok,msg]=mkdir(preDir); assert(ok,'%s',msg);
    parseName='AF7R58Q_R4_PARSE_REPLANNING_SYSTEM';
    parseText=strrep(patched,'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' parseName ' < handle']);
    write_utf8(fullfile(preDir,[parseName '.m']),parseText);

    addpath(preDir,'-begin');
    eval(['clear ' parseName]);
    mc=meta.class.fromName(parseName);
    assert(~isempty(mc),'[AF7R58Q-R4] patched class preflight parse failed.');
    safe_rmpath_if_present(preDir);

    say(f,'');
    say(f,'[C. PREFLIGHT -- BEFORE FIRST PRODUCTION WRITE]');
    say(f,' minimal source invariants              = 1');
    say(f,' patched full class parse               = 1');
    say(f,' production writes                      = 0');

    % ------------------------------------------------------------------
    % Transactional production write.
    % ------------------------------------------------------------------
    [ok,msg]=mkdir(backupDir); assert(ok,'%s',msg);
    backupReady=true;
    OUT.backup_directory=backupDir;
    copyfile(pR,fullfile(backupDir,'REPLANNING_SYSTEM.m'));

    writesStarted=true;
    write_text_eol(pR,patched,eolStyle);
    rehash;

    % Post-write full class parse through unique rename.
    postName='AF7R58Q_R4_POST_PARSE_REPLANNING_SYSTEM';
    postText=strrep(fileread(pR),'classdef REPLANNING_SYSTEM < handle', ...
        ['classdef ' postName ' < handle']);
    write_utf8(fullfile(outdir,[postName '.m']),postText);
    addpath(outdir,'-begin');
    eval(['clear ' postName]);
    mc2=meta.class.fromName(postName);
    assert(~isempty(mc2),'[AF7R58Q-R4] installed class post-parse failed.');
    safe_rmpath_if_present(outdir);

    h1=file_hash(pR);
    OUT.source_hash_before=h0;
    OUT.source_hash_after=h1;
    OUT.production_modified=true;
    OUT.status='MINIMAL_COMMIT_TIMING_INSTALL_COMPLETE__CLEAN_RERUN_REQUIRED';
    OUT.column_labels={ ...
        'request_seq','commit_total_ms','spacing_ms','resource_prepare_ms', ...
        'canonical_handles_ms','other_commit_ms','active_reference_version', ...
        'replan_commit_count','deadline_fail_count_snapshot'};

    say(f,'');
    say(f,'[D. INSTALL RESULT]');
    say(f,' bounded timing ring                   = 64 x 9');
    say(f,' measured explicit stages              = 3');
    say(f,' total commit function measured        = 1');
    say(f,' residual other_commit measured        = total - known stages');
    say(f,' optional source stage assumptions     = 0');
    say(f,' trajectory/certificate logic changed = 0');
    say(f,' commit decision changed               = 0');
    say(f,' production modified                   = 1');
    say(f,' new REPLANNING_SYSTEM SHA             = %s',h1);

    say(f,'');
    say(f,'[E. NEXT STEP]');
    say(f,' NOW perform one clean run_flight_headless run.');
    say(f,' Keep the live REPLANNING_SYSTEM object after the run.');
    say(f,' Instrumented timing is diagnostic only; final 4-ms proof comes after removal.');

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

save(fullfile(outdir,'AF7R58Q_R4_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58Q_R4_OUT',OUT);
say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_R4_REPORT.txt'));
say(f,' RETURN: %s',fullfile(outdir,'AF7R58Q_R4_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function block=instrument_statement(block,startToken,endToken,stage,indent)
a=find_first(block,startToken);
b=find_first_after(block,endToken,a);
e=find_statement_end(block,b);

block=insert_before(block,a,{[indent 'af7r58q_min_t=tic;']});

% Re-find after insertion.
a=find_first(block,startToken);
b=find_first_after(block,endToken,a);
e=find_statement_end(block,b);

block=insert_after(block,e,{ ...
    [indent sprintf('obj.af7r58q_min_stage_ms(%d)=1e3*toc(af7r58q_min_t);',stage)]});
end


function [i0,i1]=find_method_bounds(lines,fnToken)
m=find(cellfun(@(x)contains(x,fnToken),lines));
assert(numel(m)==1,'[AF7R58Q-R4] target method header count=%d.',numel(m));
i0=m(1);
i1=numel(lines);
for k=i0+1:numel(lines)
    if ~isempty(regexp(lines{k},'^\s{8}function\s','once'))
        i1=k-1;
        break;
    end
end
end


function j=find_unique(block,token)
v=find(cellfun(@(x)contains(x,token),block));
assert(numel(v)==1,'[AF7R58Q-R4] token "%s" count=%d.',token,numel(v));
j=v(1);
end


function j=find_first(block,token)
v=find(cellfun(@(x)contains(x,token),block),1,'first');
assert(~isempty(v),'[AF7R58Q-R4] token "%s" missing.',token);
j=v;
end


function j=find_first_after(block,token,a)
v=find(cellfun(@(x)contains(x,token),block));
v=v(v>=a);
assert(~isempty(v),'[AF7R58Q-R4] end token "%s" missing.',token);
j=v(1);
end


function e=find_statement_end(block,j)
e=j;
while e<=numel(block)
    if contains(block{e},';')
        return;
    end
    e=e+1;
end
error('[AF7R58Q-R4] statement end not found.');
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
