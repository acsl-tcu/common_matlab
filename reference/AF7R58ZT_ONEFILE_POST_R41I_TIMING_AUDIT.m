function OUT = AF7R58ZT_ONEFILE_POST_R41I_TIMING_AUDIT()
% AF7R58ZT
% Read-only post-run timing audit after AF7R58ZS.
% Controller is intentionally excluded.

wrapName='G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR';
srcName='G27V74R23_CONTINUOUS_WHOLE_BODY_CERT';
fastName='G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST';

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZT | POST-R41I FAST-CERT REQUEST TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | controllerExcluded=1 | continuousCertificateStillRequired=1\n');

OUT=struct();
OUT.schema='AF7R58ZT_POST_R41I_TIMING_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    wrapFile=which(wrapName);
    srcFile=which(srcName);
    fastFile=which(fastName);
    assert(~isempty(wrapFile)&&isfile(wrapFile));
    assert(~isempty(srcFile)&&isfile(srcFile));
    assert(~isempty(fastFile)&&isfile(fastFile));

    W=normalize_newlines(fileread(wrapFile));
    OUT.wrapper_uses_fast=count(W,[fastName '('])==1 && ...
        count(W,[srcName '('])==0;
    OUT.original_r23_sha=file_hash(srcFile);
    OUT.fast_sha=file_hash(fastFile);
    OUT.wrapper_sha=file_hash(wrapFile);

    assert(evalin('base','exist(''replanner_instance'',''var'')')==1, ...
        '[AF7R58ZT] replanner_instance missing.');
    R=evalin('base','replanner_instance');

    OUT.requests=double(R.generator_request_count);
    OUT.deadline_failures=double(R.generator_deadline_fail_count);
    OUT.commits=double(R.replan_commit_count);
    OUT.last_hard_cycle_ms=double(R.generator_last_cycle_critical_ms);
    OUT.last_callback_ms=double(R.generator_last_callback_ms);

    fprintf('\n[A. LIVE SOURCE BINDING]\n');
    fprintf(' wrapper uses fast R41I cert = %d\n',OUT.wrapper_uses_fast);
    fprintf(' original R23 SHA            = %s\n',OUT.original_r23_sha);
    fprintf(' fast cert SHA               = %s\n',OUT.fast_sha);
    fprintf(' wrapper SHA                 = %s\n',OUT.wrapper_sha);

    assert(evalin('base','exist(''AF7R58ZH_R4_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZT] run R4 diagnostic copy immediately before this audit.');
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

    if ~OUT.wrapper_uses_fast
        OUT.status='FAST_CERT_NOT_LIVE__DO_NOT_INTERPRET';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0 && ...
            OUT.reference_over_25ms==0
        OUT.status='R41I_FAST_CERT__4MS_AND_25MS_EMPIRICALLY_CLOSED_THIS_RUN';
    elseif OUT.deadline_failures==0 && OUT.request_hard_over_4ms==0
        OUT.status='R41I_FAST_CERT__4MS_CLOSED__REFERENCE_25MS_REMAINS';
    else
        OUT.status='R41I_FAST_CERT_ACTIVE__4MS_STILL_OPEN';
    end

    fprintf('\n[D. CLASSIFICATION]\n');
    fprintf(' status = %s\n',OUT.status);
    fprintf(' empirical timing is not a formal OS/WCET theorem.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=fileparts(which(wrapName));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZT_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZT_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'====================================================================================================\n');
fprintf(fid,' AF7R58ZT | POST-R41I FAST-CERT REQUEST TIMING AUDIT\n');
fprintf(fid,'====================================================================================================\n');
fprintf(fid,'status = %s\n',OUT.status);
if isfield(OUT,'wrapper_uses_fast')
    fprintf(fid,'wrapper uses fast cert = %d\n',OUT.wrapper_uses_fast);
    fprintf(fid,'original R23 SHA = %s\n',OUT.original_r23_sha);
    fprintf(fid,'fast cert SHA = %s\n',OUT.fast_sha);
    fprintf(fid,'wrapper SHA = %s\n',OUT.wrapper_sha);
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
fprintf(fid,'empirical timing is not formal WCET.\n');
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZT_OUT',OUT);

fprintf('\nRETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end


function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
