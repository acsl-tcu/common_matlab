function OUT = AF7R58ZY_ONEFILE_PRECALL_STAGE_POSTRUN_AUDIT()
% AF7R58ZY
% Read-only decomposition of the <=4 ms sensor-to-target window.
%
% Requires:
%   run_flight_headless_AF7R58ZX_precall_stage_audit
%
% It separates:
%   sensor gate + sensor memory
%   future-trigger kernel / trigger wrapper
%   request-identity + pre-invoke residual
%   compact generator context
%   callback
%   native AF7R36 core
%
% No production writes. Controller excluded.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZY | PRE-CALLBACK 4-MS ROOT-CAUSE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | controllerExcluded=1 | noSafetyChange=1\n');

OUT=struct();
OUT.schema='AF7R58ZY_PRECALL_ROOT_CAUSE_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    assert(evalin('base','exist(''AF7R58ZX_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZY] AF7R58ZX_RUNTIME_PROFILE missing.');
    P=evalin('base','AF7R58ZX_RUNTIME_PROFILE');

    req={'t','phase','reference_s','gen_request_delta','gen_deadline_delta','gen_commit_delta', ...
        'gen_last_callback_ms','gen_last_cycle_critical_ms', ...
        'prof_sensor_ms','prof_kernel_ms','prof_trigger_ms', ...
        'gen_elapsed_before_ms','gen_context_ms','gen_budget_remaining_ms', ...
        'r9_total_ms'};
    for k=1:numel(req)
        assert(isfield(P,req{k}),'[AF7R58ZY] missing profile field %s.',req{k});
    end

    flight=(P.phase=='f') & isfinite(P.reference_s);
    request=flight & P.gen_request_delta>0;
    nonrequest=flight & ~request;
    idx=find(request);
    n=numel(idx);

    fields={'t_s','reference_ms','hard_ms','sensor_ms','kernel_ms','trigger_ms', ...
        'elapsed_before_ms','preinvoke_residual_ms','context_ms','callback_ms', ...
        'native_ms','callback_wrapper_ms','closure_ms','deadline_fail','commit'};
    for k=1:numel(fields), OUT.request.(fields{k})=nan(1,n); end

    fprintf('\n[A. REQUEST-BY-REQUEST]\n');
    fprintf([' t[s]   ref   hard sensor kernel trigger before preInvRes context callback native ' ...
             'cbWrap closure fail commit\n']);

    for k=1:n
        j=idx(k);
        t=double(P.t(j));
        ref=1e3*double(P.reference_s(j));
        hard=double(P.gen_last_cycle_critical_ms(j));
        sensor=double(P.prof_sensor_ms(j));
        kernel=double(P.prof_kernel_ms(j));
        trigger=double(P.prof_trigger_ms(j));
        before=double(P.gen_elapsed_before_ms(j));
        ctx=double(P.gen_context_ms(j));
        cb=double(P.gen_last_callback_ms(j));
        native=double(P.r9_total_ms(j));

        preRes=before-sensor-trigger;
        cbWrap=cb-native;
        closure=hard-before-ctx-cb;

        OUT.request.t_s(k)=t;
        OUT.request.reference_ms(k)=ref;
        OUT.request.hard_ms(k)=hard;
        OUT.request.sensor_ms(k)=sensor;
        OUT.request.kernel_ms(k)=kernel;
        OUT.request.trigger_ms(k)=trigger;
        OUT.request.elapsed_before_ms(k)=before;
        OUT.request.preinvoke_residual_ms(k)=preRes;
        OUT.request.context_ms(k)=ctx;
        OUT.request.callback_ms(k)=cb;
        OUT.request.native_ms(k)=native;
        OUT.request.callback_wrapper_ms(k)=cbWrap;
        OUT.request.closure_ms(k)=closure;
        OUT.request.deadline_fail(k)=double(P.gen_deadline_delta(j));
        OUT.request.commit(k)=double(P.gen_commit_delta(j));

        fprintf('%6.3f %6.2f %6.2f %6.2f %6.3f %7.2f %6.2f %9.2f %7.2f %8.2f %6.3f %6.2f %7.2f %4.0f %6.0f\n', ...
            t,ref,hard,sensor,kernel,trigger,before,preRes,ctx,cb,native,cbWrap,closure, ...
            OUT.request.deadline_fail(k),OUT.request.commit(k));
    end

    OUT.reference_max_ms=1e3*max(P.reference_s(flight));
    OUT.nonrequest_reference_max_ms=1e3*max(P.reference_s(nonrequest));
    OUT.reference_over_25ms=nnz(P.reference_s(flight)>0.025);
    OUT.hard_max_ms=maxfinite(OUT.request.hard_ms);
    OUT.hard_over_4ms=nnz(OUT.request.hard_ms>4);

    stageNames={'SENSOR','TRIGGER','PREINVOKE_RESIDUAL','CONTEXT','CALLBACK_WRAPPER','NATIVE','CLOSURE'};
    X=[OUT.request.sensor_ms;OUT.request.trigger_ms;OUT.request.preinvoke_residual_ms; ...
       OUT.request.context_ms;OUT.request.callback_wrapper_ms;OUT.request.native_ms; ...
       OUT.request.closure_ms];

    counts=zeros(1,size(X,1));
    maxvals=nan(1,size(X,1));
    for r=1:size(X,1)
        maxvals(r)=maxfinite(X(r,:));
    end
    for k=1:n
        v=X(:,k);
        v(~isfinite(v))=-inf;
        [~,ii]=max(v);
        if isfinite(v(ii)), counts(ii)=counts(ii)+1; end
    end

    OUT.stage_names=stageNames;
    OUT.stage_dominance_counts=counts;
    OUT.stage_max_ms=maxvals;

    fprintf('\n[B. STAGE MAXIMA / DOMINANCE]\n');
    for r=1:numel(stageNames)
        fprintf(' %-20s max=%9.4f ms | dominant requests=%d\n', ...
            stageNames{r},maxvals(r),counts(r));
    end

    fprintf('\n[C. SUMMARY]\n');
    fprintf(' request count                    = %d\n',n);
    fprintf(' request hard-cycle >4 ms        = %d / %d\n',OUT.hard_over_4ms,n);
    fprintf(' max hard-cycle [ms]             = %.6f\n',OUT.hard_max_ms);
    fprintf(' max reference [ms]              = %.6f\n',OUT.reference_max_ms);
    fprintf(' max nonrequest reference [ms]   = %.6f\n',OUT.nonrequest_reference_max_ms);
    fprintf(' reference >25 ms                = %d\n',OUT.reference_over_25ms);

    [~,im]=max(maxvals);
    OUT.dominant_stage_by_max=stageNames{im};

    if OUT.hard_over_4ms==0
        OUT.status='4MS_EMPIRICALLY_CLOSED_THIS_RUN';
    else
        OUT.status=['4MS_OPEN__DOMINANT_STAGE_' OUT.dominant_stage_by_max];
    end

    fprintf('\n[D. CLASSIFICATION]\n');
    fprintf(' dominant stage by maximum = %s\n',OUT.dominant_stage_by_max);
    fprintf(' status = %s\n',OUT.status);
    fprintf(' profiling is diagnostic-only; production math/authority unchanged.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=pwd;
try
    p=which('REPLANNING_SYSTEM');
    if ~isempty(p), root=fileparts(p); end
catch
end
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZY_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZY_OUT.mat');

fid=fopen(report,'wt','n','UTF-8'); assert(fid>=0);
fprintf(fid,'AF7R58ZY | PRE-CALLBACK 4-MS ROOT-CAUSE AUDIT\n');
fprintf(fid,'status=%s\n',OUT.status);
if isfield(OUT,'request')
    fprintf(fid,'t_s,reference_ms,hard_ms,sensor_ms,kernel_ms,trigger_ms,elapsed_before_ms,preinvoke_residual_ms,context_ms,callback_ms,native_ms,callback_wrapper_ms,closure_ms,deadline_fail,commit\n');
    for k=1:numel(OUT.request.t_s)
        fprintf(fid,'%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.0f,%.0f\n', ...
            OUT.request.t_s(k),OUT.request.reference_ms(k),OUT.request.hard_ms(k), ...
            OUT.request.sensor_ms(k),OUT.request.kernel_ms(k),OUT.request.trigger_ms(k), ...
            OUT.request.elapsed_before_ms(k),OUT.request.preinvoke_residual_ms(k), ...
            OUT.request.context_ms(k),OUT.request.callback_ms(k),OUT.request.native_ms(k), ...
            OUT.request.callback_wrapper_ms(k),OUT.request.closure_ms(k), ...
            OUT.request.deadline_fail(k),OUT.request.commit(k));
    end
    fprintf(fid,'\nSTAGE MAXIMA\n');
    for r=1:numel(OUT.stage_names)
        fprintf(fid,'%s,max_ms=%.17g,dominant_count=%d\n', ...
            OUT.stage_names{r},OUT.stage_max_ms(r),OUT.stage_dominance_counts(r));
    end
    fprintf(fid,'hardMaxMs=%.17g\n',OUT.hard_max_ms);
    fprintf(fid,'hardOver4ms=%d\n',OUT.hard_over_4ms);
    fprintf(fid,'referenceMaxMs=%.17g\n',OUT.reference_max_ms);
    fprintf(fid,'nonrequestReferenceMaxMs=%.17g\n',OUT.nonrequest_reference_max_ms);
    fprintf(fid,'referenceOver25ms=%d\n',OUT.reference_over_25ms);
end
fclose(fid);

OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZY_OUT',OUT);

fprintf('\nRETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end


function y=maxfinite(x)
x=x(isfinite(x));
if isempty(x), y=NaN; else, y=max(x); end
end
