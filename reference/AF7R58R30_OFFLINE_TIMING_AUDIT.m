function R = AF7R58R30_OFFLINE_TIMING_AUDIT(matFile)
% Independent, post-run-only timing audit: does NOT modify control/safety.
% Input is the R23 timing capture from the SAME run as R29.
if nargin~=1 || ~(ischar(matFile) || isstring(matFile))
    error('R30 requires the exact R23 spike MAT filepath.');
end
L=load(char(matFile),'T');
assert(isfield(L,'T'),'Timing MAT must contain T.');
T=L.T;
req={'time_s','labels','values_ms','stage_events'};
assert(all(isfield(T,req)),'Timing capture schema incomplete.');
labels=cellstr(string(T.labels));
needed={'reference_replanner','controller','online_before_plant', ...
    'full_iteration','estimator','logger','simulation_plant'};
idx=zeros(1,numel(needed));
for k=1:numel(needed)
    j=find(strcmp(labels,needed{k}));
    assert(isscalar(j),'Timing column missing or duplicated: %s',needed{k});
    idx(k)=j;
end
t=double(T.time_s(:)); Z=double(T.values_ms);
assert(size(Z,1)==numel(t) && size(Z,2)==numel(labels), ...
    'Timing rows/columns inconsistent.');
assert(all(isfinite(t)) && all(diff(t)>0),'Simulation timestamps invalid.');
for k=1:numel(idx)
    assert(all(isfinite(Z(:,idx(k)))),'Nonfinite timing column: %s',needed{k});
end
ref=Z(:,idx(1)); control=Z(:,idx(2)); online=Z(:,idx(3));
full=Z(:,idx(4));
% This exclusion is report classification ONLY; it never alters raw arrays.
controllerSystemSpike=(control>100);
R=struct();
R.schema='AF7R58R30_OFFLINE_TIMING_AUDIT_V1';
R.source=char(matFile);
R.flight_cycles=numel(t);
R.ref_p50_ms=median(ref);
R.ref_p95_ms=r30_percentile(ref,95);
R.ref_max_ms=max(ref);
R.online_over_25_count=sum(online>25);
R.full_over_25_count=sum(full>25);
R.full_over_25_without_controller_system_spike_count= ...
    sum((full>25)&~controllerSystemSpike);
R.controller_system_spike_count=sum(controllerSystemSpike);
R.reference_over_20_count=sum(ref>20);
% Stage times and reference-node totals are measured with different scopes.
% Subtraction is an UNATTRIBUTED elapsed-time difference, not confirmed waste.
E=T.stage_events.events;
if isempty(E)
    E=struct('name',{},'time_s',{},'stage_ms',{});
end
rows={};
for k=1:numel(E)
    ename=char(string(E(k).name));
    if ~(strcmp(ename,'STAGE1_NATIVE_RETURNED') || ...
            strcmp(ename,'STAGE2_CERTIFIED_AND_PREPARED') || ...
            strcmp(ename,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS'))
        continue;
    end
    et=double(E(k).time_s); st=double(E(k).stage_ms);
    if ~isfinite(et) || ~isfinite(st)
        continue;
    end
    [gap,j]=min(abs(t-et));
    assert(gap<2e-4,'Event not alignable to timing cycle: %s',ename);
    rows(end+1,:)={t(j),ename,ref(j),st,ref(j)-st,full(j)}; %#ok<AGROW>
end
if isempty(rows)
    R.stage_attribution=table(zeros(0,1),cell(0,1),zeros(0,1), ...
        zeros(0,1),zeros(0,1),zeros(0,1),'VariableNames', ...
        {'time_s','event','reference_ms','stage_ms','unattributed_ms','full_ms'});
else
    R.stage_attribution=cell2table(rows,'VariableNames', ...
        {'time_s','event','reference_ms','stage_ms','unattributed_ms','full_ms'});
end
% Startup and two replanning windows are only diagnostic grouping intervals.
% They are not scheduling caps or trajectory constraints.
mask=((t>=5-1e-4)&(t<=5.125+1e-4)) | ...
     ((t>=13.25-1e-4)&(t<=13.4+1e-4)) | ...
     ((t>=14.775-1e-4)&(t<=14.85+1e-4));
R.reference_over_20_outside_known_windows_count=sum((ref>20)&~mask);
R.full_over_25_outside_known_windows_no_controller_system_spike_count= ...
    sum((full>25)&~mask&~controllerSystemSpike);
[~,ord]=sort(ref,'descend');
ord=ord(1:min(30,numel(ord)));
R.highest_reference_cycles=table(t(ord),ref(ord),full(ord), ...
    control(ord),online(ord),'VariableNames', ...
    {'time_s','reference_ms','full_ms','controller_ms','online_ms'});
[~,worstOutside]=max(ref.*double(~mask));
R.worst_reference_nonstage_time_s=t(worstOutside);
R.worst_reference_nonstage_ms=ref(worstOutside);
% Same state trajectory is NOT held constant across nominal/repair regimes;
% before/after comparisons are association, never a causal proof.
periods=[5 13.2;13.3 14.75;14.825 23.5;23.6 45];
regimes=zeros(4,4);
for k=1:4
   sel=(t>=periods(k,1))&(t<=periods(k,2));
   regimes(k,:)=[sum(sel),median(ref(sel)),r30_percentile(ref(sel),95), ...
      r30_percentile(full(sel),95)];
end
R.regime_comparison=array2table(regimes,'VariableNames', ...
    {'cycles','ref_median_ms','ref_p95_ms','full_p95_ms'});
R.regime_comparison.start_s=periods(:,1);
R.regime_comparison.end_s=periods(:,2);
R.notes={ ...
    'Stage-to-reference differences are unattributed; do not call them redundant computation.', ...
    'Controller >100ms is excluded only from diagnostic ranking, not from raw timing.', ...
    'Reference category includes REPLANNING_SYSTEM, dispatch, and other reference adjustments.', ...
    'Fixed 25ms is a measurement threshold here; no runtime acceptance gate is modified.', ...
    'These MAT files contain no continuous trajectory/obstacle certificate for the return lobe.'};
fprintf('AF7R58R30 | READ-ONLY POST-RUN TIMING AUDIT\n');
fprintf('Flight cycles: %d | ref p50/p95/max %.4f / %.4f / %.4f ms\n', ...
    R.flight_cycles,R.ref_p50_ms,R.ref_p95_ms,R.ref_max_ms);
fprintf('Online >25ms: %d | full >25ms: %d | full >25 excluding controller >100ms: %d\n', ...
    R.online_over_25_count,R.full_over_25_count, ...
    R.full_over_25_without_controller_system_spike_count);
fprintf('Reference >20ms: %d; outside known event windows: %d\n', ...
    R.reference_over_20_count,R.reference_over_20_outside_known_windows_count);
fprintf('Reference spikes outside event windows: maximum %.4f ms at %.3f s\n', ...
    R.worst_reference_nonstage_ms,R.worst_reference_nonstage_time_s);
disp(R.stage_attribution);
disp(R.regime_comparison);
end

function y = r30_percentile(x,p)
% Base-MATLAB-only linear interpolation: avoids toolbox requirements.
x=sort(double(x(:)));
assert(~isempty(x)&&all(isfinite(x))&&p>=0&&p<=100);
a=1+(numel(x)-1)*p/100;
i=floor(a); j=ceil(a);
y=x(i)+(a-i)*(x(j)-x(i));
end
