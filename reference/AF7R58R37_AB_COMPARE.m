function R=AF7R58R37_AB_COMPARE(beforePath,afterPath)
% Read-only comparison of two actual R23 full-loop timing MAT captures.
% Requires independently obtained simulation runs with equivalent stimuli.
% No internal timing fields are assumed present in the ablation run.
assert(isfile(beforePath)&&isfile(afterPath), 'Both R23 MAT files must exist.');
A=load(beforePath,'T'); B=load(afterPath,'T');
assert(isfield(A,'T')&&isfield(B,'T'),'Both MATs must contain T.');
a=A.T;b=B.T;
assert(isequal(cellstr(string(a.labels(:))),cellstr(string(b.labels(:)))), ...
    'R23 instrumentation labels differ.');
ta=double(a.time_s(:));tb=double(b.time_s(:));
assert(numel(ta)==numel(tb)&&all(abs(ta-tb)<1e-8), ...
    'Simulation Flight times do not match. Do not infer speedup.');
X=double(a.values_ms);Y=double(b.values_ms);n=numel(ta);
assert(size(X,1)==n&&size(Y,1)==n&&isequal(size(X),size(Y)), ...
    'Mismatch in recorded cycle count or components.');
labels=cellstr(string(a.labels(:))); 
get=@(z,key) z(:,find(strcmp(labels,key),1,'first'));
for nm={'reference_replanner','full_iteration','online_before_plant','controller', ...
        'logger','simulation_plant'}
    assert(nnz(strcmp(labels,nm{1}))==1,'Missing full-loop component.');
end
refA=get(X,'reference_replanner');refB=get(Y,'reference_replanner');
fullA=get(X,'full_iteration');fullB=get(Y,'full_iteration');
ctrlA=get(X,'controller');ctrlB=get(Y,'controller');
% R23 source records full iteration as the exact sum of nine components.
compIdx=1:9;
closureA=fullA-sum(X(:,compIdx),2);
closureB=fullB-sum(Y(:,compIdx),2);
assert(max(abs(closureA))<1e-7&&max(abs(closureB))<1e-7, ...
    'R23 full-iteration timing does not close.');
% Compare structural events without treating shorter runtime as safety proof.
ea=a.stage_events.events;eb=b.stage_events.events;
nameA=string({ea.name}).';nameB=string({eb.name}).';
timeA=[ea.time_s].';timeB=[eb.time_s].';
R=struct();R.schema='AF7R58R37_AB_COMPARISON_V1';
R.before_file=char(beforePath);R.after_file=char(afterPath);
R.events_match=isequal(nameA,nameB) && numel(timeA)==numel(timeB) && ...
  all((isnan(timeA)&isnan(timeB)) | abs(timeA-timeB)<1e-8);
R.stage_gate_summary_match=true;
for gate={'native_connected','native_success','native_speed_pass', ...
    'native_heading_pass','native_geometry_pass','commit_attempted', ...
    'activation_confirmed'}
    key=gate{1};
    if ~isfield(a.stage_events,key) || ~isfield(b.stage_events,key) || ...
       ~isequaln(a.stage_events.(key),b.stage_events.(key))
        R.stage_gate_summary_match=false;
    end
end
R.same_trajectory_not_proven_by_event_matching=true;
R.reference_p50_before_ms=median(refA);R.reference_p50_after_ms=median(refB);
R.reference_p95_before_ms=prctile(refA,95);R.reference_p95_after_ms=prctile(refB,95);
R.reference_max_before_ms=max(refA);R.reference_max_after_ms=max(refB);
R.full_p50_before_ms=median(fullA);R.full_p50_after_ms=median(fullB);
R.full_p95_before_ms=prctile(fullA,95);R.full_p95_after_ms=prctile(fullB,95);
R.full_over25_before=nnz(fullA>25);R.full_over25_after=nnz(fullB>25);
R.exclude_controller_gt100_before=nnz(fullA>25 & ctrlA<=100);
R.exclude_controller_gt100_after=nnz(fullB>25 & ctrlB<=100);
R.closure_before_max_ms=max(abs(closureA));
R.closure_after_max_ms=max(abs(closureB));
R.speedup_not_certified=true;
R.production_execution_safety_not_certified=true;
% Report fixed clock events and their same-cycle reference timing.
keep=strcmp(nameA,'STAGE1_NATIVE_RETURNED') | ...
     strcmp(nameA,'STAGE2_CERTIFIED_AND_PREPARED') | ...
     strcmp(nameA,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS');
keptName=nameA(keep);keptTime=timeA(keep);
ra=nan(size(keptTime));rb=ra;
for k=1:numel(keptTime)
    i=find(abs(ta-keptTime(k))<1e-8,1,'first');
    if ~isempty(i), ra(k)=refA(i);rb(k)=refB(i);end
end
R.stage_comparison=table(keptTime,keptName,ra,rb,rb-ra, ...
 'VariableNames',{'time_s','event','reference_before_ms','reference_after_ms','delta_ms'});
R.full_components_before_ms=X;R.full_components_after_ms=Y;
R.labels=labels;R.time_s=ta;
save('AF7R58R37_AB_COMPARE.mat','R');
writetable(R.stage_comparison,'AF7R58R37_AB_STAGE_COMPARISON.csv');
fid=fopen('AF7R58R37_AB_SUMMARY.txt','w');
assert(fid>0,'Cannot write AB summary.');
cl=onCleanup(@()fclose(fid));
fprintf(fid,'R37 EXPERIMENTAL LIGHT-LOGGING ABLATION: NO SAFETY CLAIM\n');
fprintf(fid,'events match: %d, gate summaries match: %d\n', ...
 R.events_match,R.stage_gate_summary_match);
fprintf(fid,'ref p50 %.6f -> %.6f ms\n',R.reference_p50_before_ms,R.reference_p50_after_ms);
fprintf(fid,'ref p95 %.6f -> %.6f ms\n',R.reference_p95_before_ms,R.reference_p95_after_ms);
fprintf(fid,'ref MAX %.6f -> %.6f ms\n',R.reference_max_before_ms,R.reference_max_after_ms);
fprintf(fid,'full p50 %.6f -> %.6f ms\n',R.full_p50_before_ms,R.full_p50_after_ms);
fprintf(fid,'full >25 (controller excluded): %d -> %d\n', ...
 R.exclude_controller_gt100_before,R.exclude_controller_gt100_after);
fprintf(fid,'full closure error maxima %.12g / %.12g ms\n', ...
 R.closure_before_max_ms,R.closure_after_max_ms);
if ~R.events_match || ~R.stage_gate_summary_match
    warning('R37:EventChange','Event history differs: timing-only equivalence NOT established.');
end
disp('[R37] read-only A/B comparison saved; one run per condition is NOT a WCET guarantee.');
disp(R.stage_comparison);
end
