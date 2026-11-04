function A=CHECK_TRIGGER_COMPARE_THREE_RUNS()
% Read-only, 3-run event-conditioned reproducibility and timing audit.
% No 5-second Flight epoch or fixed obstacle count assumption.
F={'TRIGGER_DIAG_RUN_01.mat','TRIGGER_DIAG_RUN_02.mat','TRIGGER_DIAG_RUN_03.mat'};
C=cell(1,3);
for i=1:3
    assert(isfile(F{i}), 'CHECK_TRIGGER_COMPARE_THREE_RUNS:Missing', ...
        'Capture each of the 3 completed Flights first: %s.',F{i});
    z=load(F{i},'C');
    assert(isfield(z,'C') && strcmp(z.C.version,'TRIGGER_DIAG_V1') && ...
        z.C.run_id==i && z.C.n>0, ...
        'CHECK_TRIGGER_COMPARE_THREE_RUNS:Version');
    C{i}=z.C;
end
A=struct('status','THREE_RUNS_ANALYZED__NO_CAUSAL_OR_WCET_PROOF', ...
  'n',[C{1}.n C{2}.n C{3}.n],'reason','', ...
  'all_stage_recorded',all(cellfun(@(x)x.stage_recorded,C)), ...
  'replanner_med',nan(1,3),'replanner_p95',nan(1,3), ...
  'replanner_max',nan(1,3),'predictor_max',nan(1,3), ...
  'geometry_max',nan(1,3),'new_id_max',nan(1,3), ...
  'nonflight_max',nan(1,3),'unexpected_top_count',zeros(1,3), ...
  'incident_peak_count',zeros(1,3),'event_overlap_peak_count',zeros(1,3));
fid=fopen('TRIGGER_DIAG_THREE_RUN_COMPARISON.txt','w');
assert(fid>=0,'CHECK_TRIGGER_COMPARE_THREE_RUNS:File');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'status=%s\n',A.status);
fprintf(fid,'A spike EVENT correlation is not proof of root cause.\n');
fprintf(fid,'3 runs cannot establish a deterministic worst-case execution-time bound.\n');
fprintf(fid,'No fixed Flight entry time, obstacle count or motion is assumed.\n');
fprintf(fid,'Stage timing itself adds overhead; compare runs with the SAME instrumentation.\n');
fprintf(fid,'\n=== PER-RUN STATISTICS ===\n');
fprintf(fid,'run,frames,stage_on,first_predict,nonflight_max,update_med,update_p95,update_max,trigger_med,trigger_p95,trigger_max,geometry_max,rhs_max,reject_max,new_id_max,det_changes\n');
for i=1:3
    d=C{i};p=d.predicted;
    A.replanner_med(i)=stat(d.replanner_ms,0.50);
    A.replanner_p95(i)=stat(d.replanner_ms,0.95);
    A.replanner_max(i)=stat(d.replanner_ms,1);
    A.predictor_max(i)=stat(d.predictor_ms,1);
    A.geometry_max(i)=stat(d.geometry_ms,1);
    ix=find(p & d.acquired);
    if ~isempty(ix),A.new_id_max(i)=stat(d.replanner_ms(ix),1);end
    if isfinite(d.first_prediction)&&d.first_prediction>1
        A.nonflight_max(i)=stat(d.replanner_ms(1:d.first_prediction-1),1);
    end
    fprintf(fid,'%d,%d,%d,%.0f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.0f,%.0f,%.5f,%d\n', ...
        i,d.n,d.stage_recorded,d.first_prediction,A.nonflight_max(i), ...
        A.replanner_med(i),A.replanner_p95(i),A.replanner_max(i), ...
        stat(d.trigger_ms,0.50),stat(d.trigger_ms,0.95), ...
        stat(d.trigger_ms,1),A.geometry_max(i), ...
        stat(d.rhs_evals,1),stat(d.rejected_steps,1), ...
        A.new_id_max(i),nnz(d.sensor_transition));
end
fprintf(fid,'\n=== TOP 1 PERCENT, REASON-HINT CLASSIFIED BY MEASUREMENTS ===\n');
fprintf(fid,['run,k,dt_from_first_prediction_s,update_ms,trigger_ms,geom_ms,predictor_ms,' ...
  'blue_other,trigger_outer,rhs,rejected,acquired,det_changed,new_event,duplicate,category\n']);
for i=1:3
    d=C{i}; m=d.replanner_ms;
    finite=find(isfinite(m));[~,ord]=sort(m(finite),'descend');
    sel=finite(ord(1:max(1,ceil(.01*numel(finite)))));
    count_unattributed=0;
    A.incident_peak_count(i)=numel(sel);
    A.event_overlap_peak_count(i)=nnz(d.acquired(sel)|d.sensor_transition(sel)| ...
       d.new_event(sel)|d.duplicate(sel));
    for k=reshape(sel,1,[])
        category=label(d,k);
        if strcmp(category,'UNATTRIBUTED_TIMING_OR_ALLOCATION'),count_unattributed=count_unattributed+1;end
        dt=NaN;
        if isfinite(d.first_prediction)
            dt=d.time_s(k)-d.time_s(d.first_prediction);
        end
        fprintf(fid,['%d,%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,' ...
            '%.0f,%.0f,%d,%d,%d,%d,%s\n'], ...
            i,k,dt,d.replanner_ms(k),d.trigger_ms(k), ...
            d.geometry_ms(k),d.predictor_ms(k), ...
            d.blue_other_ms(k),d.trigger_outer_ms(k),d.rhs_evals(k),d.rejected_steps(k),d.acquired(k), ...
            d.sensor_transition(k),d.new_event(k),d.duplicate(k),category);
    end
    A.unexpected_top_count(i)=count_unattributed;
end
fprintf(fid,'\n=== EVENT CLASS SUMMARY (NOT CAUSATION) ===\n');
fprintf(fid,'Top-1percent event overlap counts=[%d %d %d] / [%d %d %d].\n', ...
    A.event_overlap_peak_count,A.incident_peak_count);
fprintf(fid,'run,event,count,update_median,update_max,trigger_max,sensor_max,geometry_max\n');
for i=1:3
    d=C{i};
    event={'INITIAL_FRAME','NEW_ID','SENSOR_SET_CHANGE', ...
      'NEW_CONTACT_EVENT','DUPLICATE_EVENT','SAMPLED_CONTACT', ...
      'NO_NEW_ID_AND_NO_SENSOR_TRANSITION'};
    ix={1,find(d.acquired),find(d.sensor_transition), ...
      find(d.new_event),find(d.duplicate),find(d.sampled_contact), ...
      find(d.predicted & ~d.acquired & ~d.sensor_transition)};
    for j=1:numel(event)
        ind=ix{j};
        fprintf(fid,'%d,%s,%d,%.5f,%.5f,%.5f,%.5f,%.5f\n', ...
            i,event{j},numel(ind), ...
            stat(d.replanner_ms(ind),.5),stat(d.replanner_ms(ind),1), ...
            stat(d.trigger_ms(ind),1),stat(d.sensor_ms(ind),1), ...
            stat(d.geometry_ms(ind),1));
    end
end
fprintf(fid,'\n=== TIME-ALIGNED PEAKS ACROSS THREE RUNS ===\n');
fprintf(fid,'Time coincidence is only meaningful when the actual scenarios/phase timing are the same.\n');
fprintf(fid,'We report top-1%% peaks within two local sample periods of the relative first-prediction time.\n');
idx=cell(1,3); tol=nan(1,3);
for i=1:3
    d=C{i};
    valid=find(d.predicted & isfinite(d.replanner_ms));
    if isempty(valid),idx{i}=[];continue;end
    th=stat(d.replanner_ms(valid),.99);
    idx{i}=valid(d.replanner_ms(valid)>=th);
    time_dist=diff(d.time_s(valid));time_dist=time_dist(time_dist>0);
    if ~isempty(time_dist),tol(i)=2*median(time_dist);end
end
if all(isfinite(tol)) && all(cellfun(@(x)~isempty(x),idx)) && ...
        all(cellfun(@(x)isfinite(x.first_prediction),C))
    a=C{1};b=C{2};c=C{3};
    t1=a.time_s(idx{1})-a.time_s(a.first_prediction);
    t2=b.time_s(idx{2})-b.time_s(b.first_prediction);
    t3=c.time_s(idx{3})-c.time_s(c.first_prediction);
    for j=1:numel(t1)
        [e2,j2]=min(abs(t2-t1(j)));[e3,j3]=min(abs(t3-t1(j)));
        if e2<=max(tol(1),tol(2)) && e3<=max(tol(1),tol(3))
            fprintf(fid,'run1_k=%d,run2_k=%d,run3_k=%d,dt=%.5f,update=[%.4f %.4f %.4f],labels=%s|%s|%s\n', ...
                idx{1}(j),idx{2}(j2),idx{3}(j3),t1(j), ...
                a.replanner_ms(idx{1}(j)),b.replanner_ms(idx{2}(j2)), ...
                c.replanner_ms(idx{3}(j3)), ...
                label(a,idx{1}(j)),label(b,idx{2}(j2)),label(c,idx{3}(j3)));
        end
    end
end
fprintf(fid,'\n=== MEASUREMENT-GUIDED NEXT DIAGNOSTIC ===\n');
fprintf(fid,'No event overlap + high blue_other => profile update attachment/stage; do not assert GC.\n');
fprintf(fid,'High geometry and exact-query counts => geometry workload branch.\n');
fprintf(fid,'High predictor and RHS/rejections => investigate native numerical workload.\n');
fprintf(fid,'High trigger_outer with low prediction and geometry => MATLAB trigger wrapper/event.\n');
fprintf(fid,'\nNEXT_ACTION: correlate high geometry_ms with exact query counts; high predictor_ms with RHS/rejected; high update-minus-trigger with stage detail and detection/memory transitions.\n');
fprintf(fid,'If all stage fields are absent, rerun with setenv(''REPLANNING_STAGE_TIMING'',''1'') before Flight construction.\n');
save('TRIGGER_DIAG_THREE_RUN_COMPARISON.mat','A','-v7.3');
fprintf('Three-run report saved: TRIGGER_DIAG_THREE_RUN_COMPARISON.txt\n');
end
function s=label(d,k)
if ~d.predicted(k)
    s='PRE_PREDICTION_OR_NONFLIGHT';
elseif d.acquired(k)
    s='NEW_OBSTACLE_ACQUISITION';
elseif d.sensor_transition(k)
    s='DETECTION_SET_TRANSITION';
elseif d.new_event(k)
    s='NEW_CONTACT_EVENT';
elseif d.duplicate(k)
    s='DUPLICATE_SUPPRESSION';
else
    gg=d.geometry_ms(k);gc=d.body_exact_queries(k)+d.cable_exact_queries(k);
    pp=d.predictor_ms(k);rr=d.rejected_steps(k);
    if isfinite(gg) && isfinite(gc) && gc>0 && ...
            gg>stat(d.geometry_ms(d.predicted),.95)
        s='GEOMETRY_WORK_CORRELATED';
    elseif isfinite(pp) && pp>stat(d.predictor_ms(d.predicted),.95)
        s='PREDICTOR_WALL_CORRELATED';
    elseif isfinite(rr) && rr>stat(d.rejected_steps(d.predicted),.95)
        s='NUMERIC_REJECTIONS_CORRELATED';
    else
        s='UNATTRIBUTED_TIMING_OR_ALLOCATION';
    end
end
end
function y=stat(x,q)
x=sort(x(isfinite(x)));
if isempty(x),y=NaN;return;end
if q==1,y=x(end);return;end
y=x(max(1,ceil(q*numel(x))));
end
