function R=AF7R58R43_TIMING_AUDIT(beforeFile,afterFile)
% Exact-cycle alignment, top spikes by causal region, no native-timer attribution.
B=load(beforeFile,'T'); A=load(afterFile,'T');
assert(isfield(B,'T')&&isfield(A,'T'),'Both files require T.');
b=B.T; a=A.T;
lb=string(b.labels(:));la=string(a.labels(:));
assert(isequal(lb,la),'Timing field labels differ.');
t=double(a.time_s(:)); tb=double(b.time_s(:));
assert(numel(tb)==numel(t) && all(diff(t)>0) && ...
    max(abs(tb-t))<1e-8*max(1,max(abs(t))), ...
    'Flight time grids differ.');
va=double(a.values_ms); vb=double(b.values_ms);
assert(isequal(size(va),size(vb))&&size(va,1)==numel(t), ...
    'Timing series incompatible.');
ref=col(la,'reference_replanner'); full=col(la,'full_iteration');
ctrl=col(la,'controller'); est=col(la,'estimator');
lg=col(la,'logger'); online=col(la,'online_before_plant');
allFinite=all(isfinite(va),2)&all(isfinite(vb),2);
assert(all(allFinite),'Timing records contain non-finite values.');
% Observational label, NOT automatic attribution: stage event only on recorded ticks.
E=a.stage_events.events;
evtT=[]; evtName=strings(0,1);
for i=1:numel(E)
   nm=string(E(i).name);
   if contains(nm,'STAGE1_NATIVE_RETURNED')|| ...
      contains(nm,'STAGE2_CERTIFIED_AND_PREPARED')|| ...
      contains(nm,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS')
       evtT(end+1,1)=double(E(i).time_s); %#ok<AGROW>
       evtName(end+1,1)=nm; %#ok<AGROW>
   end
end
stageIdx=zeros(numel(evtT),1);
for i=1:numel(evtT)
 [d,j]=min(abs(t-evtT(i))); 
 assert(d<=1e-8*max(1,abs(evtT(i))),'Stage event not on Flight grid.');
 stageIdx(i)=j;
end
idx=sort(unique(stageIdx));
% exclude previous controller-only abnormal episode only from causal ranking,
% never from measured full-cycle timing.
[~,descending]=sort(va(:,ref),'descend');
N=min(35,numel(descending));
k=descending(1:N);
R=struct();R.schema='R43_TIMING_TWO_RUN';
R.n_cycles=numel(t);R.stage_event_count=numel(stageIdx);
R.event_time_s=evtT;R.event_type=evtName;
R.event_values=table(t(stageIdx),vb(stageIdx,ref),va(stageIdx,ref), ...
    vb(stageIdx,full),va(stageIdx,full), ...
    'VariableNames',{'Time_s','Ref_before_ms','Ref_after_ms','Full_before_ms','Full_after_ms'});
R.top_reference_cycles=table(t(k),va(k,ref),va(k,full),va(k,est), ...
    va(k,ctrl),va(k,lg), ...
    'VariableNames',{'Time_s','Ref_ms','Full_ms','Estimator_ms','Controller_ms','Logger_ms'});
R.reference_p50_before_ms=median(vb(:,ref));
R.reference_p50_after_ms=median(va(:,ref));
R.reference_p95_before_ms=prctile(vb(:,ref),95);
R.reference_p95_after_ms=prctile(va(:,ref),95);
R.reference_max_before_ms=max(vb(:,ref));
R.reference_max_after_ms=max(va(:,ref));
R.full_exceed25_before_count=nnz(vb(:,full)>25);
R.full_exceed25_after_count=nnz(va(:,full)>25);
R.online_exceed25_after_count=nnz(va(:,online)>25);
R.native_stage_provenance_not_event_specific=true;
R.internal_closure_proved=false;
R.causes_not_proven='A/B single run confounded by host jitter and operating state';
save('AF7R58R43_TIMING_AUDIT.mat','R');
end
function j=col(names,needle)
 j=find(names==needle,1);
 assert(~isempty(j),'Missing timing label %s',needle);
end
