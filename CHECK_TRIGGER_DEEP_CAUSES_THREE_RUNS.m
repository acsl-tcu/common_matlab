function A=CHECK_TRIGGER_DEEP_CAUSES_THREE_RUNS()
% Compare exactly three previously captured, equally instrumented Flights.
% No assumption on phase timing, flight path, sensor shape, or obstacle count.
% Report elapsed-time ATTRIBUTION, not JIT/GC/OS causation or WCET proof.
files={'TRIGGER_DIAG_RUN_01.mat','TRIGGER_DIAG_RUN_02.mat', ...
       'TRIGGER_DIAG_RUN_03.mat'};
C=cell(1,3);
for i=1:3
    assert(isfile(files{i}),'CHECK_TRIGGER_DEEP_CAUSES:Missing', ...
        'Capture Flight %d with the updated CHECK_TRIGGER_CAPTURE_RUN.',i);
    s=load(files{i},'C');
    assert(isfield(s,'C') && strcmp(s.C.version,'TRIGGER_DIAG_V3_SENSOR') && ...
        s.C.run_id==i && s.C.n>0, ...
        'CHECK_TRIGGER_DEEP_CAUSES:Version', ...
        'All 3 captures must use version TRIGGER_DIAG_V2.');
    assert(isfield(s.C,'predicted') && nnz(s.C.predicted)>0 && ...
        isfield(s.C,'complete') && ...
        nnz(s.C.complete & s.C.predicted)==nnz(s.C.predicted) && ...
        ~any(strcmp(s.C.trigger_status, ...
                    'UNKNOWN_OBSERVED_GEOMETRY_BINDING')), ...
        'CHECK_TRIGGER_DEEP_CAUSES:NotRunning', ...
        ['Flight %d did not complete true 4-second predictions. ', ...
         'A fast UNKNOWN result is NOT a valid performance measurement.'],i);
    assert(isfield(s.C,'sensor_stage') && ...
        any(isfinite(s.C.sensor_stage(:))), ...
        'CHECK_TRIGGER_DEEP_CAUSES:SensorCounters', ...
        'Missing sensor internal stages in Flight %d. Run updated source.',i);
    C{i}=s.C;
end
A=struct('status','THREE_RUN_DEEP_ATTRIBUTION__NO_CAUSAL_OR_WCET_PROOF', ...
    'reason','', 'med_ms',nan(1,3),'p95_ms',nan(1,3), ...
    'max_ms',nan(1,3),'event_peak_count',zeros(1,3), ...
    'non_event_peak_count',zeros(1,3),'reference_cache_reset_count',zeros(1,3), ...
    'reference_fresh_max',nan(1,3), ...
    'sensor_max_ms',nan(1,3),'sensor_exact_max_ms',nan(1,3));
fid=fopen('TRIGGER_DIAG_DEEP_THREE_RUN_CAUSES.txt','w');
assert(fid>=0,'CHECK_TRIGGER_DEEP_CAUSES:Open');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'status=%s\n',A.status);
fprintf(fid,'IMPORTANT: Timers measure wall-clock time, not CPU-only work.\n');
fprintf(fid,'A long stage with unchanged workload is NOT proof of a GC/JIT/OS cause.\n');
fprintf(fid,'No Flight start time, obstacle count or specific path is assumed.\n');
fprintf(fid,'The same tracing options must be enabled in all three runs.\n');
fprintf(fid,'Reference columns are actually refreshed only when the cache reports it.\n');
fprintf(fid,'Geometry stage is summed over queried observed shapes.\n');
fprintf(fid,'\nRUN SUMMARY\n');
fprintf(fid,'run,n,med,p95,max,event_peak,non_event_peak,cache_resets,max_fresh\n');
for r=1:3
    d=C{r};
    A.med_ms(r)=percentile(d.replanner_ms,0.50);
    A.p95_ms(r)=percentile(d.replanner_ms,0.95);
    A.max_ms(r)=percentile(d.replanner_ms,1);
    A.sensor_max_ms(r)=percentile(d.sensor_ms,1);
    A.sensor_exact_max_ms(r)=percentile(d.sensor_stage(:,2),1);
    if any(isfinite(d.cache_reset))
        A.reference_cache_reset_count(r)=nnz(d.cache_reset==1);
    end
    A.reference_fresh_max(r)=percentile(d.cache_fresh,1);
    events=logical(d.acquired|d.sensor_transition|d.new_event);
    event_plus=events;
    event_plus(2:end)=events(2:end)|events(1:end-1);
    high=isfinite(d.replanner_ms)&d.replanner_ms>5;
    A.event_peak_count(r)=nnz(high&event_plus);
    A.non_event_peak_count(r)=nnz(high&~event_plus);
    fprintf(fid,'%d,%d,%.5f,%.5f,%.5f,%d,%d,%d,%.0f\n', ...
        r,d.n,A.med_ms(r),A.p95_ms(r),A.max_ms(r), ...
        A.event_peak_count(r),A.non_event_peak_count(r), ...
        A.reference_cache_reset_count(r),A.reference_fresh_max(r));
    fprintf(fid,'\nRUN %d: TOP 20 / EVENT BASELINES\n',r);
    fprintf(fid,['k,t,dt_first_pred,update,trigger,sensor,memory,ref,geom,', ...
        'mem_validate,mem_ids,mem_refresh,mem_book,geom_broad,geom_support,geom_exact,', ...
        'cache_reset,fresh,hits,func_calls,source_loop_ms,cache_wall_ms,', ...
        'rhs,rejected,acquired,det_changed,new_event,duplicate,tag\n']);
    [~,order]=sort(d.replanner_ms,'descend');
    order=order(1:min(20,numel(order)));
    for k=reshape(order,1,[])
        dt=NaN;
        if isfinite(d.first_prediction)
            dt=d.time_s(k)-d.time_s(d.first_prediction);
        end
        label='NO_RECORDED_EVENT';
        if d.acquired(k)
            label='ACQUISITION';
        elseif d.sensor_transition(k)
            label='DETECTION_TRANSITION';
        elseif d.new_event(k)
            label='NEW_CONTACT';
        elseif d.duplicate(k)
            label='DUPLICATE_EVENT';
        end
        fprintf(fid,['%d,%.9f,%.9f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,', ...
            '%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,', ...
            '%.0f,%.0f,%.0f,%.0f,%.5f,%.5f,%.0f,%.0f,%d,%d,%d,%d,%s\n'], ...
            k,d.time_s(k),dt,d.replanner_ms(k),d.trigger_ms(k), ...
            d.sensor_ms(k),d.memory_ms(k),d.reference_ms(k),d.geometry_ms(k), ...
            d.memory_stage(k,1:4),d.geometry_stage(k,1:3), ...
            d.cache_reset(k),d.cache_fresh(k),d.cache_hits(k),d.cache_calls(k), ...
            d.cache_source_loop_ms(k),d.cache_wall_ms(k), ...
            d.rhs_evals(k),d.rejected_steps(k),d.acquired(k), ...
            d.sensor_transition(k),d.new_event(k),d.duplicate(k),label);
    end
    fprintf(fid,'\nRUN %d: SENSOR OUTLIERS\n',r);
    fprintf(fid,['k,t,sensor_ms,scan_ms,exact_ms_nested,event_ids_ms,', ...
        'visible_ms,pack_ms,exact_calls,candidate_count,', ...
        'mask_changed,native_events\n']);
    sr=d.sensor_ms;sr(~isfinite(sr))=-Inf;
    [~,sidx]=sort(sr,'descend');sidx=sidx(1:min(20,numel(sidx)));
    for k=reshape(sidx,1,[])
        fprintf(fid,['%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,', ...
            '%.6f,%.0f,%.0f,%.0f,%.0f\n'], ...
            k,d.time_s(k),d.sensor_ms(k),d.sensor_stage(k,:), ...
            d.sensor_exact_calls(k),d.sensor_candidates(k), ...
            d.sensor_mask_changed(k),d.sensor_native_events(k));
    end
    fprintf(fid,'SENSOR exact_ms is nested within scan_ms, not additive.\n');
    fprintf(fid,'\nRUN %d: REFERENCE OUTLIERS\n',r);
    topstage(fid,d,d.reference_ms,'REFERENCE');
    fprintf(fid,'\nRUN %d: MEMORY OUTLIERS\n',r);
    topstage(fid,d,d.memory_ms,'MEMORY');
    fprintf(fid,'\nRUN %d: GEOMETRY OUTLIERS\n',r);
    topstage(fid,d,d.geometry_ms,'GEOMETRY');
end
fprintf(fid,'\nDECISION GUIDANCE\n');
fprintf(fid,'Reference spike: check cache_reset/fresh/function_calls versus source_loop_ms.\n');
fprintf(fid,'Memory spike: check four disjoint stages; unchanged IDs with long time needs further CPU/scheduler instrumentation.\n');
fprintf(fid,'Geometry spike: check broad/support/exact and body/cable query count.\n');
fprintf(fid,'RHS/rejected numbers diagnose native numerical workload, not MATLAB pauses.\n');
fprintf(fid,'An unsensed obstacle is never added by the trigger.\n');
fprintf(fid,'No sampled-only no-contact can certify continuous-time safety.\n');
save('TRIGGER_DIAG_DEEP_THREE_RUN_CAUSES.mat','A');
fprintf('Deep three-run attribution saved: TRIGGER_DIAG_DEEP_THREE_RUN_CAUSES.txt\n');
end
function topstage(fid,d,s,label)
ss=s;ss(~isfinite(ss))=-Inf;
[~,idx]=sort(ss,'descend');idx=idx(1:min(6,numel(idx)));
fprintf(fid,'k,wall_ms,rhs,rejected,acquired,changed,cache_reset,ref_fresh,ref_calls,mem_stage1,mem_stage2,mem_stage3,mem_stage4,geom_broad,geom_support,geom_exact\n');
for k=reshape(idx,1,[])
    fprintf(fid,'%d,%.6f,%.0f,%.0f,%d,%d,%.0f,%.0f,%.0f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
      k,s(k),d.rhs_evals(k),d.rejected_steps(k),d.acquired(k), ...
      d.sensor_transition(k),d.cache_reset(k),d.cache_fresh(k), ...
      d.cache_calls(k),d.memory_stage(k,1:4),d.geometry_stage(k,1:3));
end
fprintf(fid,'Stage=%s; all timings include possible external preemption.\n',label);
end
function v=percentile(x,p)
x=x(isfinite(x));
if isempty(x),v=NaN;return;end
x=sort(double(x(:)));i=max(1,min(numel(x),ceil(p*numel(x))));v=x(i);
end
