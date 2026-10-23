function R=AF7R58R44_RUNTIME_ATTRIBUTION(beforeMat,afterMat)
% Source-independent event/non-event attribution, read-only. No hard RT claim.
% Inputs must be from matched Flight time grids and obstacle/reference cases.
if nargin<1 || isempty(beforeMat)
    error('Provide the actual R41 baseline MAT, no invented fallback.');
end
if nargin<2 || isempty(afterMat)
    error('Provide the actual R42 MAT, no invented fallback.');
end
A=load(beforeMat,'T'); B=load(afterMat,'T');
assert(isfield(A,'T') && isfield(B,'T'),'Both files need the measured T.');
A=A.T;B=B.T;
ta=double(A.time_s(:));tb=double(B.time_s(:));
assert(numel(ta)==numel(tb),'Different Flight cycle counts.');
assert(all(abs(ta-tb)<=64*eps(max(1,max(abs([ta tb]),[],2)))), ...
    'Different Flight clocks; comparison refused.');
labelsA=cellstr(string(A.labels(:))); labelsB=cellstr(string(B.labels(:)));
assert(isequal(labelsA,labelsB),'Different timing component schema.');
X=double(A.values_ms);Y=double(B.values_ms);
assert(isequal(size(X),size(Y)) && size(X,1)==numel(ta), ...
    'Timing matrix dimensions inconsistent.');
idx=@(s)find(strcmp(labelsA,s),1);
iRef=idx('reference_replanner');iCtl=idx('controller');
iEst=idx('estimator');iLog=idx('logger');iFull=idx('full_iteration');
iOnline=idx('online_before_plant');
assert(all(~cellfun(@isempty,{iRef,iCtl,iEst,iLog,iFull,iOnline})), ...
    'Required timing components are missing.');
% Named Stage events: use their actual timestamps, not hard-coded doubles.
evA=A.stage_events.events;
if iscell(evA),evA=[evA{:}];end
names=string({evA.name});
mask=ismember(names,["STAGE1_NATIVE_RETURNED", ...
  "STAGE2_CERTIFIED_AND_PREPARED","STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS"]);
evtimes=double([evA(mask).time_s]);
assert(numel(evtimes)>=1 && all(isfinite(evtimes)), ...
    'Missing Stage events in baseline.');
eventCycle=false(numel(ta),1);
for j=1:numel(evtimes)
    [d,k]=min(abs(ta-evtimes(j)));
    assert(d<=max(1e-9,64*eps(max(1,abs(evtimes(j))))), ...
      'Stage event is not on this Flight grid.');
    eventCycle(k)=true;
end
% This is a DIAGNOSTIC grouping only, not an avoidance-duration restriction.
postCommit=false(numel(ta),1);
commitTimes=double([evA(names=="STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS").time_s]);
for j=1:numel(commitTimes)
    postCommit=postCommit | (ta>commitTimes(j) & ...
        ta<=commitTimes(j)+8*median(diff(ta)));
end
base=~eventCycle & ~postCommit;
rows=struct();
regimes={eventCycle,postCommit,base};regNames={'stage_events','postcommit_8_cycles','all_other_cycles'};
for j=1:numel(regimes)
    m=regimes{j};
    rows.(regNames{j})=struct( ...
       'count',nnz(m), ...
       'ref_before_p50_ms',median(X(m,iRef)), ...
       'ref_after_p50_ms',median(Y(m,iRef)), ...
       'ref_before_max_ms',max(X(m,iRef)), ...
       'ref_after_max_ms',max(Y(m,iRef)), ...
       'full_before_p95_ms',prctile(X(m,iFull),95), ...
       'full_after_p95_ms',prctile(Y(m,iFull),95));
end
R=struct('schema','R44_RUNTIME_ATTRIBUTION_V1','n',numel(ta), ...
   't',ta,'labels',{labelsA},'regimes',rows, ...
   'reference_before_ms',X(:,iRef),'reference_after_ms',Y(:,iRef), ...
   'controller_before_ms',X(:,iCtl),'controller_after_ms',Y(:,iCtl), ...
   'estimator_before_ms',X(:,iEst),'estimator_after_ms',Y(:,iEst), ...
   'logger_before_ms',X(:,iLog),'logger_after_ms',Y(:,iLog), ...
   'event_cycle',eventCycle,'postcommit_cycle',postCommit, ...
   'replanner_integration_caused_all_spikes_proved',false, ...
   'full_cycle_max_closure_before_ms',NaN, ...
   'full_cycle_max_closure_after_ms',NaN);
% Close exact P50 cycles from nonoverlapping component names.
parts={'sensor_acquisition','sensor','estimator','reference_replanner', ...
 'controller','input_transform','simulation_plant','logger','other_overhead'};
cols=cellfun(idx,parts);
R.full_cycle_max_closure_before_ms=max(abs(sum(X(:,cols),2)-X(:,iFull)));
R.full_cycle_max_closure_after_ms=max(abs(sum(Y(:,cols),2)-Y(:,iFull)));
R.output_report='No source-level causal attribution from a single timing pair.';
end
