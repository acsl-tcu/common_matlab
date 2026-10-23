function R = AF7R58R40_OFFLINE_SUMMARY(currentMat, baselineMat)
% R40R1: event-source-bound offline timing comparison.
% Read-only with respect to the replanner, plant, controller and safety gates.
% NO hard-coded event times: floating simulation clocks are compared by their
% original per-run event records. NO estimation of unmeasured internal time.
if nargin<1 || isempty(currentMat)
    currentMat='AF7R58R23_SPIKE_REPORT.mat';
end
assert(isfile(currentMat),'[R40R1] Current timing MAT missing: %s',currentMat);
A=AF7R58R39_TIMING_CLOSURE(currentMat);
R=struct();
R.schema='AF7R58R40R1_EVENT_BOUND_POSTRUN_V2';
R.current_timing_mat=char(currentMat);
R.current_p50_reference_ms=A.p50_reference_ms;
R.current_p95_reference_ms=A.p95_reference_ms;
R.current_max_reference_ms=A.max_reference_ms;
R.current_stage_table=A.stage_table;
R.current_top_table=A.top_table;
R.current_full_closure_residual_ms=A.max_full_residual_ms;
R.current_online_closure_residual_ms=A.max_online_residual_ms;
R.internal_timer_closure_verified=A.internal_closure_proved;
R.actual_trajectory_continuous_cert_rechecked=false;
R.estimator_origin_trigger_switched=false;
R.return_optimization_committed=false;
R.backup_timing_available=false;
R.baseline_comparison=table();
R.after_commit_followup=table();
R.event_match_verified=false;
R.flight_time_grid_match_verified=false;
R.baseline_p50_reference_ms=NaN;
R.baseline_max_reference_ms=NaN;
R.max_time_grid_difference_s=NaN;
R.baseline_timing_mat='';
if nargin>=2 && ~isempty(baselineMat)
    assert(isfile(baselineMat),'[R40R1] Baseline timing MAT missing: %s',baselineMat);
    assert(~strcmp(char(currentMat),char(baselineMat)), ...
        '[R40R1] Baseline path equals current path; no self comparison.');
    B=AF7R58R39_TIMING_CLOSURE(baselineMat);
    t=double(A.all_cycles.Time_s(:));
    tb=double(B.all_cycles.Time_s(:));
    assert(numel(t)==numel(tb),'[R40R1] Flight cycle counts differ.');
    dt=diff(t);
    assert(~isempty(dt) && all(isfinite(dt)) && all(dt>0), ...
        '[R40R1] Flight time grid invalid.');
    sampling_dt=min(dt);
    grid_error=max(abs(t-tb));
    eps_scale=max(1,max(abs([t;tb])));
    grid_tol=max(256*eps(eps_scale),sampling_dt*1e-7);
    assert(grid_error<=grid_tol, ...
        '[R40R1] Different Flight grids; comparison is unsafe.');
    EA=A.stage_table;
    EB=B.stage_table;
    assert(height(EA)==height(EB), ...
        '[R40R1] Stage event counts differ; no matched comparison.');
    % Event names must appear in the same chronology. The events in the
    % current and baseline MAT determine their own authoritative timestamps.
    na=string(EA.Event(:));
    nb=string(EB.Event(:));
    assert(isequal(na,nb), ...
        '[R40R1] Event sequences differ; cannot call trials equivalent.');
    te=double(EA.Time_s(:));
    teb=double(EB.Time_s(:));
    event_tol=max(256*eps(eps_scale),sampling_dt*1e-7);
    assert(all(isfinite(te)) && all(isfinite(teb)) && ...
           all(abs(te-teb)<=event_tol), ...
           '[R40R1] Event times differ between runs.');
    iA=zeros(height(EA),1);
    iB=zeros(height(EB),1);
    % Do NOT match to decimal literals such as 13.25: MATLAB accumulation
    % legitimately stores 13.25000000000012 in a Flight time grid.
    for k=1:height(EA)
        [da,iA(k)]=min(abs(t-te(k)));
        [db,iB(k)]=min(abs(tb-teb(k)));
        assert(da<=event_tol && db<=event_tol, ...
            '[R40R1] Stage event is missing from its own Flight grid.');
        assert(iA(k)==iB(k), ...
            '[R40R1] Same event landed in different Flight cycles.');
    end
    assert(numel(unique(iA))==numel(iA), ...
        '[R40R1] Multiple recorded stage events occupy the same Flight cycle.');
    RA=double(A.all_cycles.Reference_ms(:));
    RB=double(B.all_cycles.Reference_ms(:));
    FA=double(A.all_cycles.Full_ms(:));
    FB=double(B.all_cycles.Full_ms(:));
    R.baseline_comparison=table(t(iA),na,RB(iB),RA(iA), ...
        RA(iA)-RB(iB),FB(iB),FA(iA), ...
        'VariableNames',{'Time_s','Event','Before_reference_ms', ...
        'After_reference_ms','Delta_ms','Before_full_ms','After_full_ms'});
    R.baseline_p50_reference_ms=B.p50_reference_ms;
    R.baseline_max_reference_ms=B.max_reference_ms;
    R.baseline_timing_mat=char(baselineMat);
    R.backup_timing_available=true;
    R.event_match_verified=true;
    R.flight_time_grid_match_verified=true;
    R.max_time_grid_difference_s=grid_error;
    % Consecutive cycles AFTER commit, not just Stage3 wall time. This
    % reveals the secondary bumps without attributing them to a function.
    rows=[];
    for k=1:numel(iA)
        if ~contains(na(k),'ATOMIC_COMMIT_PASS'),continue;end
        ks=(iA(k)+1:min(numel(t),iA(k)+8)).';
        rows=[rows;[repmat(k,numel(ks),1),t(ks), ...
            RB(ks),RA(ks),RA(ks)-RB(ks),FB(ks),FA(ks)]]; %#ok<AGROW>
    end
    if ~isempty(rows)
        R.after_commit_followup=array2table(rows, ...
            'VariableNames',{'Event_index','Time_s', ...
            'Before_reference_ms','After_reference_ms','Delta_ms', ...
            'Before_full_ms','After_full_ms'});
    end
end
fprintf('[R40R1] ref p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    R.current_p50_reference_ms,R.current_p95_reference_ms,R.current_max_reference_ms);
fprintf('[R40R1] full/online closure residual %.9g / %.9g ms\n', ...
    R.current_full_closure_residual_ms,R.current_online_closure_residual_ms);
fprintf('[R40R1] INTERNAL timing closure %d (unmeasured means false)\n', ...
    R.internal_timer_closure_verified);
if R.backup_timing_available
    disp(R.baseline_comparison);
    disp(R.after_commit_followup);
end
save('AF7R58R40_OFFLINE_SUMMARY.mat','R');
end
