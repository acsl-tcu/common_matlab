function OUT = AF7R58ZT_R3_ONEFILE_POST_SAFE_FAST_REJECTOR_TIMING_AUDIT()
% AF7R58ZT-R3
% Read-only audit after AF7R58ZS-R3.
% Controller is excluded.

wrapName='G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR';
srcName='G27V74R23_CONTINUOUS_WHOLE_BODY_CERT';
fastName='G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST';
gateName='G27V74R58ZS_R3_CONTINUOUS_WHOLE_BODY_CERT_GATED';

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZT-R3 | SAFE FAST-REJECTOR TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | controllerExcluded=1 | originalR23FinalAuthorityRequired=1\n');

OUT=struct();
OUT.schema='AF7R58ZT_R3_SAFE_FAST_REJECTOR_TIMING_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    [wrapFile,wrapHow]=resolve_wrapper_source(wrapName);
    assert(~isempty(wrapFile)&&isfile(wrapFile), ...
        '[AF7R58ZT-R3] live wrapper unresolved.');

    wrapDir=fileparts(wrapFile);
    W=normalize_newlines(fileread(wrapFile));

    Cgate=find_executable_direct_calls(W,gateName);
    Cold=find_executable_direct_calls(W,srcName);
    Cfast=find_executable_direct_calls(W,fastName);

    fastFile=fullfile(wrapDir,[fastName '.m']);
    gateFile=fullfile(wrapDir,[gateName '.m']);

    OUT.wrapper_resolution=wrapHow;
    OUT.wrapper_source=wrapFile;
    OUT.wrapper_sha=file_hash(wrapFile);
    OUT.wrapper_gate_call_count=numel(Cgate);
    OUT.wrapper_original_call_count=numel(Cold);
    OUT.wrapper_fast_call_count=numel(Cfast);
    OUT.fast_resolved=isfile(fastFile);
    OUT.gate_resolved=isfile(gateFile);

    if OUT.gate_resolved
        G=normalize_newlines(fileread(gateFile));
        OUT.gate_fast_call_count=numel(find_executable_direct_calls(G,fastName));
        OUT.gate_original_call_count=numel(find_executable_direct_calls(G,srcName));
        pFail=strfind(G,'if ~fastPass');
        pOrig=strfind(G,['C=' srcName '(varargin{:});']);
        OUT.gate_fail_before_original = ...
            numel(pFail)==1 && numel(pOrig)==1 && pFail<pOrig;
    else
        OUT.gate_fast_call_count=0;
        OUT.gate_original_call_count=0;
        OUT.gate_fail_before_original=false;
    end

    OUT.safe_architecture_live = ...
        OUT.wrapper_gate_call_count>=1 && ...
        OUT.wrapper_original_call_count==0 && ...
        OUT.wrapper_fast_call_count==0 && ...
        OUT.fast_resolved && OUT.gate_resolved && ...
        OUT.gate_fast_call_count==1 && ...
        OUT.gate_original_call_count==1 && ...
        OUT.gate_fail_before_original;

    fprintf('\n[A. LIVE SAFETY ARCHITECTURE]\n');
    fprintf(' wrapper resolution              = %s\n',wrapHow);
    fprintf(' wrapper gate/original/fast      = %d / %d / %d\n', ...
        OUT.wrapper_gate_call_count,OUT.wrapper_original_call_count,OUT.wrapper_fast_call_count);
    fprintf(' gate fast/original calls        = %d / %d\n', ...
        OUT.gate_fast_call_count,OUT.gate_original_call_count);
    fprintf(' fast FAIL precedes original     = %d\n',OUT.gate_fail_before_original);
    fprintf(' original R23 final-authority architecture = %d\n',OUT.safe_architecture_live);

    assert(evalin('base','exist(''replanner_instance'',''var'')')==1, ...
        '[AF7R58ZT-R3] replanner_instance missing.');
    R=evalin('base','replanner_instance');

    OUT.requests=double(R.generator_request_count);
    OUT.deadline_failures=double(R.generator_deadline_fail_count);
    OUT.commits=double(R.replan_commit_count);

    assert(evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZT-R3] run R4 diagnostic copy immediately before this audit.');
    P=evalin('base','AF7R58ZH_R4_RUNTIME_PROFILE');

    flight=(P.phase=='f') & isfinite(P.reference_s);
    request=flight & P.gen_request_delta>0;
    nonrequest=flight & ~request;
    ridx=find(request);

    OUT.reference_max_ms=1e3*max(P.reference_s(flight));
    OUT.reference_over_25ms=nnz(P.reference_s(flight)>0.025);
    OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonrequest));

    n=numel(ridx);
    OUT.request_t=zeros(1,n);
    OUT.request_reference_ms=zeros(1,n);
    OUT.request_hard_ms=zeros(1,n);
    OUT.request_fail=zeros(1,n);
    OUT.request_commit=zeros(1,n);

    for k=1:n
        j=ridx(k);
        OUT.request_t(k)=double(P.t(j));
        OUT.request_reference_ms(k)=1e3*double(P.reference_s(j));
        OUT.request_hard_ms(k)=double(P.gen_last_cycle_critical_ms(j));
        OUT.request_fail(k)=double(P.gen_deadline_delta(j));
        OUT.request_commit(k)=double(P.gen_commit_delta(j));
    end

    if n>0
        OUT.request_reference_max_ms=max(OUT.request_reference_ms);
        OUT.request_hard_max_ms=max(OUT.request_hard_ms);
        OUT.request_hard_over_4ms=nnz(OUT.request_hard_ms>4);
    else
        OUT.request_reference_max_ms=NaN;
        OUT.request_hard_max_ms=NaN;
        OUT.request_hard_over_4ms=0;
    end

    fprintf('\n[B. REQUEST-BY-REQUEST -- CONTROLLER EXCLUDED]\n');
    fprintf(' t[s]   reference[ms] hardCycle[ms] deadlineFail commit\n');
    for k=1:n
        fprintf('%6.3f %13.3f %13.3f %12.0f %6.0f\n', ...
            OUT.request_t(k),OUT.request_reference_ms(k),OUT.request_hard_ms(k), ...
            OUT.request_fail(k),OUT.request_commit(k));
    end

    fprintf('\n[C. SUMMARY]\n');
    fprintf(' requests / deadline failures / commits = %.0f / %.0f / %.0f\n', ...
        OUT.requests,OUT.deadline_failures,OUT.commits);
    fprintf(' max request hard-cycle [ms]            = %.6f\n',OUT.request_hard_max_ms);
    fprintf(' request hard-cycle >4 ms               = %d / %d\n',OUT.request_hard_over_4ms,n);
    fprintf(' max request reference [ms]             = %.6f\n',OUT.request_reference_max_ms);
    fprintf(' max nonrequest reference [ms]          = %.6f\n',OUT.nonrequest_reference_max_ms);
    fprintf(' all reference >25 ms                   = %d\n',OUT.reference_over_25ms);

    if ~OUT.safe_architecture_live
        OUT.status='SAFE_FAST_REJECTOR_ARCHITECTURE_NOT_CLOSED__DO_NOT_INTERPRET_TIMING';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0 && ...
            OUT.reference_over_25ms==0
        OUT.status='SAFE_FAST_REJECTOR__4MS_AND_25MS_EMPIRICALLY_CLOSED_THIS_RUN';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0
        OUT.status='SAFE_FAST_REJECTOR__4MS_CLOSED__REFERENCE_25MS_REMAINS';
    else
        OUT.status='SAFE_FAST_REJECTOR_ACTIVE__4MS_STILL_OPEN';
    end

    fprintf('\n[D. CLASSIFICATION]\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' final SAFE authority remains ORIGINAL R23.\n');
    fprintf(' empirical timing is not a formal OS/WCET theorem.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=pwd;
try
    if exist('wrapFile','var')&&~isempty(wrapFile),root=fileparts(wrapFile);end
catch
end
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZT_R3_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZT_R3_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'AF7R58ZT-R3 status = %s\n',OUT.status);
if isfield(OUT,'safe_architecture_live')
    fprintf(fid,'safe architecture live = %d\n',OUT.safe_architecture_live);
    fprintf(fid,'wrapper gate/original/fast = %d / %d / %d\n', ...
        OUT.wrapper_gate_call_count,OUT.wrapper_original_call_count,OUT.wrapper_fast_call_count);
    fprintf(fid,'gate fast/original = %d / %d\n', ...
        OUT.gate_fast_call_count,OUT.gate_original_call_count);
end
if isfield(OUT,'requests')
    fprintf(fid,'requests / deadline failures / commits = %.0f / %.0f / %.0f\n', ...
        OUT.requests,OUT.deadline_failures,OUT.commits);
end
if isfield(OUT,'request_hard_max_ms')
    fprintf(fid,'max request hard-cycle ms = %.9g\n',OUT.request_hard_max_ms);
    fprintf(fid,'request hard-cycle >4ms = %d\n',OUT.request_hard_over_4ms);
    fprintf(fid,'max request reference ms = %.9g\n',OUT.request_reference_max_ms);
    fprintf(fid,'max nonrequest reference ms = %.9g\n',OUT.nonrequest_reference_max_ms);
    fprintf(fid,'all reference >25ms = %d\n',OUT.reference_over_25ms);
    fprintf(fid,'\nrequest table:\n');
    fprintf(fid,'t_s,reference_ms,hard_cycle_ms,deadline_fail,commit\n');
    for k=1:numel(OUT.request_t)
        fprintf(fid,'%.9g,%.9g,%.9g,%.0f,%.0f\n', ...
            OUT.request_t(k),OUT.request_reference_ms(k),OUT.request_hard_ms(k), ...
            OUT.request_fail(k),OUT.request_commit(k));
    end
end
fprintf(fid,'final SAFE authority remains ORIGINAL R23.\n');
fprintf(fid,'empirical timing is not formal WCET.\n');
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZT_R3_OUT',OUT);

fprintf('\nRETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end


function [p,how]=resolve_wrapper_source(name)
p=which(name);
how='WHICH';
if ~isempty(p)&&isfile(p),return;end
p='';how='';
try
    if evalin('base','exist(''replanner_instance'',''var'')')==1
        R=evalin('base','replanner_instance');
        fh=R.generator_backend_locked_callback;
        if isa(fh,'function_handle')
            info=functions(fh);
            if isfield(info,'file')&&ischar(info.file)&&isfile(info.file)
                p=info.file;
                how='LIVE_LOCKED_CALLBACK_HANDLE';
                return;
            end
        end
    end
catch
end
end


function C=find_executable_direct_calls(txt,name)
L=regexp(normalize_newlines(txt),'\n','split');
C=struct('line',{},'text',{});
for k=1:numel(L)
    code=code_before_comment(L{k});
    if ~isempty(direct_call_starts(code,name))
        C(end+1)=struct('line',k,'text',strtrim(L{k})); %#ok<AGROW>
    end
end
end


function starts=direct_call_starts(code,name)
starts=[];
pos=strfind(code,name);
for k=1:numel(pos)
    p=pos(k);
    q=p+numel(name);
    if p>1 && is_ident_char(code(p-1)),continue;end
    if q<=numel(code)&&is_ident_char(code(q)),continue;end
    z=q;
    while z<=numel(code)&&isspace(code(z)),z=z+1;end
    if z<=numel(code)&&code(z)=='(',starts(end+1)=p;end %#ok<AGROW>
end
end


function tf=is_ident_char(c)
tf=isstrprop(c,'alphanum')||c=='_';
end


function code=code_before_comment(line)
p=strfind(line,'%');
if isempty(p),code=line;else,code=line(1:p(1)-1);end
end


function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function h=file_hash(p)
fid=fopen(p,'rb');assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
