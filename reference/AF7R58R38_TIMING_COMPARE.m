function R=AF7R58R38_TIMING_COMPARE(baselineMat,currentMat)
% Offline only. R23 packet exports 1601 Flight cycles with SAME time grid.
% No LOGGER dependency, no profiles fabricated when instrumentation is OFF.
B=load(baselineMat,'T'); C=load(currentMat,'T');
assert(isfield(B,'T')&&isfield(C,'T'),'Need the R23 Flight timing MAT.');
B=B.T;C=C.T;
assert(isequal(double(B.time_s(:)),double(C.time_s(:))), ...
    '[R38] baseline and current cycles differ.');
assert(isequal(cellstr(B.labels(:)),cellstr(C.labels(:))), ...
    '[R38] timing column labels differ.');
t=double(C.time_s(:)); X=double(C.values_ms); X0=double(B.values_ms);
labels=cellstr(C.labels(:));
idx=@(name)find(strcmp(labels,name),1,'first');
iRef=idx('reference_replanner');iFull=idx('full_iteration');
iOnline=idx('online_before_plant');iCtl=idx('controller');
assert(~isempty(iRef)&&~isempty(iFull)&&~isempty(iOnline)&&~isempty(iCtl));
iParts=cellfun(idx,{'sensor_acquisition','sensor','estimator', ...
    'reference_replanner','controller','input_transform', ...
    'simulation_plant','logger','other_overhead'});
iPre=cellfun(idx,{'sensor_acquisition','sensor','estimator', ...
    'reference_replanner','controller','input_transform'});
assert(isequal(size(X),size(X0))&&size(X,1)==numel(t));
fullResidual=X(:,iFull)-sum(X(:,iParts),2);
onlineResidual=X(:,iOnline)-sum(X(:,iPre),2);
assert(all(isfinite(fullResidual))&&all(isfinite(onlineResidual)));
[~,ord]=sort(X(:,iRef),'descend'); ord=ord(1:min(30,numel(ord)));
R=struct(); R.schema='AF7R58R38_AB_OUTER_CLOSURE_V1';
R.top_cycles=table(t(ord),X0(ord,iRef),X(ord,iRef), ...
    X0(ord,iFull),X(ord,iFull),X(ord,iCtl), ...
    'VariableNames',{'Time_s','BaseRef_ms','NowRef_ms', ...
        'BaseFull_ms','NowFull_ms','Controller_ms'});
R.stage_event_source='EVENTS_ARE_DIAGNOSTIC_NOT_A_COMPLETE_INNER_TIMER';
R.reference_p50_before_ms=median(X0(:,iRef));
R.reference_p50_after_ms=median(X(:,iRef));
R.reference_max_before_ms=max(X0(:,iRef));
R.reference_max_after_ms=max(X(:,iRef));
R.reference_p95_before_ms=prctile(X0(:,iRef),95);
R.reference_p95_after_ms=prctile(X(:,iRef),95);
R.full_cycle_max_abs_closure_residual_ms=max(abs(fullResidual));
R.online_max_abs_closure_residual_ms=max(abs(onlineResidual));
R.online_over25=sum(X(:,iOnline)>25);
R.full_over25_without_controller_gt100=sum(X(:,iFull)>25 & X(:,iCtl)<=100);
R.unknown_stage_inner_cost_is_not_zero=true;
R.controller_anomaly_is_retained_in_raw_data=true;
R.safety_candidate_and_commit_equivalence_not_checked=true;
R.hard_realtime_worstcase_proven=false;
end
