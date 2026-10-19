function OUT = AF7R58ZZC_ONEFILE_REFERENCE_CYCLE_POSTRUN_AUDIT()
% AF7R58ZZC
% Read-only discrimination of main-thread compute vs wall-time waiting/descheduling.
% Requires a completed:
%   run_flight_headless_AF7R58ZZB_reference_cycle_audit
%
% af7r25_stamp_mex authority:
%   item 1 = QPC tick
%   item 2 = QPC frequency
%   item 3 = current-thread cycle counter
%   item 4 = process cycle counter
%
% No controller timing is used for classification. No production writes.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZC | REFERENCE WALL vs THREAD/PROCESS CYCLE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | controllerExcluded=1 | noSafetyChange=1\n');

OUT=struct();
OUT.schema='AF7R58ZZC_REFERENCE_CYCLE_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

try
    assert(evalin('base','exist(''AF7R58ZZB_RUNTIME_PROFILE'',''var'')')==1, ...
        '[AF7R58ZZC] AF7R58ZZB_RUNTIME_PROFILE missing.');
    P=evalin('base','AF7R58ZZB_RUNTIME_PROFILE');

    req={'t','phase','reference_s','reference_qpc_ms', ...
        'reference_thread_cycles','reference_process_cycles', ...
        'callback_qpc_ms','callback_thread_cycles','callback_process_cycles', ...
        'gen_request_delta','gen_deadline_delta','gen_commit_delta', ...
        'gen_last_cycle_critical_ms','gen_last_callback_ms'};
    for k=1:numel(req)
        assert(isfield(P,req{k}),'[AF7R58ZZC] profile missing %s.',req{k});
    end

    flight=(P.phase=='f') & isfinite(P.reference_s) & ...
        isfinite(P.reference_thread_cycles) & isfinite(P.reference_process_cycles);
    request=flight & P.gen_request_delta>0;
    ridx=find(request);
    fidx=find(flight);
    n=numel(ridx);
    assert(n>=1,'[AF7R58ZZC] no generator requests captured.');

    % Lowest-wall flight cycles approximate uninterrupted local execution.
    fw=1e3*double(P.reference_s(fidx));
    [~,ord]=sort(fw);
    nb=max(8,ceil(0.20*numel(fidx)));
    ib=fidx(ord(1:min(nb,numel(ord))));

    baseWall=median(1e3*double(P.reference_s(ib)),'omitnan');
    baseQpc=median(double(P.reference_qpc_ms(ib)),'omitnan');
    baseThread=median(double(P.reference_thread_cycles(ib)),'omitnan');
    baseProcess=median(double(P.reference_process_cycles(ib)),'omitnan');
    assert(isfinite(baseWall)&&baseWall>0&&isfinite(baseThread)&&baseThread>0);

    OUT.baseline=struct('wall_ms',baseWall,'qpc_ms',baseQpc, ...
        'thread_cycles',baseThread,'process_cycles',baseProcess, ...
        'sample_count',numel(ib));

    t=double(P.t(ridx));
    wall=1e3*double(P.reference_s(ridx));
    qpc=double(P.reference_qpc_ms(ridx));
    thr=double(P.reference_thread_cycles(ridx));
    proc=double(P.reference_process_cycles(ridx));
    hard=double(P.gen_last_cycle_critical_ms(ridx));
    cb=double(P.gen_last_callback_ms(ridx));
    cbq=double(P.callback_qpc_ms(ridx));
    cbthr=double(P.callback_thread_cycles(ridx));
    cbproc=double(P.callback_process_cycles(ridx));
    fail=double(P.gen_deadline_delta(ridx));
    commit=double(P.gen_commit_delta(ridx));

    wallR=wall/baseWall;
    thrR=thr/baseThread;
    procR=proc/baseProcess;

    % Deliberately conservative classifier. These labels diagnose the run;
    % they are not a timing theorem.
    waitLike=(wallR>=1.8) & (thrR<=1.35);
    computeLike=(wallR>=1.8) & (thrR>=1.60);
    otherThreadLike=(wallR>=1.8) & (thrR<=1.35) & (procR>=1.60);
    mixed=(wallR>=1.8) & ~(waitLike|computeLike);

    OUT.t_s=t;
    OUT.reference_wall_ms=wall;
    OUT.reference_qpc_ms=qpc;
    OUT.reference_thread_cycles=thr;
    OUT.reference_process_cycles=proc;
    OUT.hard_ms=hard;
    OUT.callback_ms=cb;
    OUT.callback_qpc_ms=cbq;
    OUT.callback_thread_cycles=cbthr;
    OUT.callback_process_cycles=cbproc;
    OUT.wall_ratio=wallR;
    OUT.thread_ratio=thrR;
    OUT.process_ratio=procR;
    OUT.wait_like=waitLike;
    OUT.compute_like=computeLike;
    OUT.other_thread_like=otherThreadLike;
    OUT.mixed=mixed;
    OUT.deadline_fail=fail;
    OUT.commit=commit;

    fprintf('\n[A. BASELINE -- LOWEST 20%% FLIGHT REFERENCE CYCLES]\n');
    fprintf(' sample count            = %d\n',numel(ib));
    fprintf(' reference wall median   = %.6f ms\n',baseWall);
    fprintf(' reference QPC median    = %.6f ms\n',baseQpc);
    fprintf(' thread cycles median    = %.17g\n',baseThread);
    fprintf(' process cycles median   = %.17g\n',baseProcess);

    fprintf('\n[B. REQUEST TABLE]\n');
    fprintf(' t[s] refWall QPC hard callback cbQPC wallR threadR processR wait compute otherThr mixed fail commit\n');
    for k=1:n
        fprintf('%6.3f %7.2f %7.2f %6.2f %8.2f %6.2f %5.2f %7.2f %8.2f %4d %7d %8d %5d %4.0f %6.0f\n', ...
            t(k),wall(k),qpc(k),hard(k),cb(k),cbq(k),wallR(k),thrR(k),procR(k), ...
            waitLike(k),computeLike(k),otherThreadLike(k),mixed(k),fail(k),commit(k));
    end

    OUT.count_wait_like=nnz(waitLike);
    OUT.count_compute_like=nnz(computeLike);
    OUT.count_other_thread_like=nnz(otherThreadLike);
    OUT.count_mixed=nnz(mixed);
    OUT.request_count=n;

    fprintf('\n[C. CLASSIFICATION]\n');
    fprintf(' wait/deschedule-like = %d / %d\n',OUT.count_wait_like,n);
    fprintf(' main-thread compute  = %d / %d\n',OUT.count_compute_like,n);
    fprintf(' other-thread/process = %d / %d\n',OUT.count_other_thread_like,n);
    fprintf(' mixed                = %d / %d\n',OUT.count_mixed,n);

    need=max(2,ceil(n/3));
    if OUT.count_wait_like>=need
        OUT.status='REFERENCE_SPIKES_WAIT_OR_DESCHEDULING_DOMINANT';
    elseif OUT.count_compute_like>=need
        OUT.status='REFERENCE_SPIKES_MAIN_THREAD_COMPUTE_DOMINANT';
    elseif OUT.count_other_thread_like>=need
        OUT.status='REFERENCE_SPIKES_OTHER_PROCESS_THREADS_DOMINANT';
    else
        OUT.status='REFERENCE_SPIKES_MIXED__USE_REQUEST_TABLE';
    end

    fprintf(' status = %s\n',OUT.status);
    fprintf(' empirical Windows/MATLAB timing is not formal WCET.\n');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    fprintf('\n[AUDIT ERROR] %s | %s\n',ME.identifier,ME.message);
end

root=pwd;
try
    p=which('REPLANNING_SYSTEM');
    if ~isempty(p),root=fileparts(p);end
catch
end
outdir=tempname(root);
[ok,msg]=mkdir(outdir);assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZZC_REPORT.txt');
matp=fullfile(outdir,'AF7R58ZZC_OUT.mat');
fid=fopen(report,'wt','n','UTF-8');assert(fid>=0);
fprintf(fid,'AF7R58ZZC status=%s\n',OUT.status);
if isfield(OUT,'baseline')
    fprintf(fid,'baselineWallMs=%.17g\n',OUT.baseline.wall_ms);
    fprintf(fid,'baselineQpcMs=%.17g\n',OUT.baseline.qpc_ms);
    fprintf(fid,'baselineThreadCycles=%.17g\n',OUT.baseline.thread_cycles);
    fprintf(fid,'baselineProcessCycles=%.17g\n',OUT.baseline.process_cycles);
    fprintf(fid,'waitLike=%d/%d\n',OUT.count_wait_like,OUT.request_count);
    fprintf(fid,'computeLike=%d/%d\n',OUT.count_compute_like,OUT.request_count);
    fprintf(fid,'otherThreadLike=%d/%d\n',OUT.count_other_thread_like,OUT.request_count);
    fprintf(fid,'mixed=%d/%d\n',OUT.count_mixed,OUT.request_count);
    fprintf(fid,'t_s,ref_wall_ms,qpc_ms,hard_ms,callback_ms,callback_qpc_ms,wall_ratio,thread_ratio,process_ratio,wait_like,compute_like,other_thread_like,mixed,fail,commit\n');
    for k=1:OUT.request_count
        fprintf(fid,'%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%d,%.0f,%.0f\n', ...
            OUT.t_s(k),OUT.reference_wall_ms(k),OUT.reference_qpc_ms(k),OUT.hard_ms(k), ...
            OUT.callback_ms(k),OUT.callback_qpc_ms(k),OUT.wall_ratio(k),OUT.thread_ratio(k), ...
            OUT.process_ratio(k),OUT.wait_like(k),OUT.compute_like(k),OUT.other_thread_like(k), ...
            OUT.mixed(k),OUT.deadline_fail(k),OUT.commit(k));
    end
end
fclose(fid);
OUT.report_path=report;
OUT.mat_path=matp;
save(matp,'OUT','-v7');
assignin('base','AF7R58ZZC_OUT',OUT);
fprintf('\nRETURN: %s\n',report);
fprintf('RETURN: %s\n',matp);
end
