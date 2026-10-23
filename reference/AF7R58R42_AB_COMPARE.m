function R=AF7R58R42_AB_COMPARE(beforeFile,afterFile)
% Compare exact source-bound per-cycle timing and event histories.
% Separate recorded measurements from any mathematical-safety claims.
B=load(beforeFile,'T'); A=load(afterFile,'T');
assert(isfield(B,'T')&&isfield(A,'T'), 'Both MAT files require T.');
b=B.T; a=A.T;
assert(isequal(string(b.labels(:)),string(a.labels(:))), ...
 'Timing labels mismatch; do not compare different schemas.');
tb=double(b.time_s(:)); ta=double(a.time_s(:));
assert(numel(tb)==numel(ta) && all(isfinite(tb)) && ...
       all(isfinite(ta)), 'Flight grids have invalid dimensions.');
% This is time-grid matching, not a trigger/activation acceptance tolerance.
delta=max(abs(tb-ta));
tol=1e-8*max(1,max(abs(tb)));
assert(delta<=tol, 'Runs have different Flight time grids.');
vb=double(b.values_ms);va=double(a.values_ms);
assert(size(vb,1)==numel(tb)&&isequal(size(vb),size(va)), ...
 'Timing matrices have different dimensions.');
labels=string(a.labels(:));
ref=find(labels=="reference_replanner",1);
full=find(labels=="full_iteration",1);
ctrl=find(labels=="controller",1);
assert(~isempty(ref)&&~isempty(full)&&~isempty(ctrl), ...
 'Required timing labels missing.');
E0=b.stage_events.events;E1=a.stage_events.events;
[n0,t0,ms0]=readEvents(E0);[n1,t1,ms1]=readEvents(E1);
R=struct();R.schema='R42_AB_SOURCE_BOUND_V1';
R.flight_cycles=numel(tb);
R.event_sequence_match=isequal(n0,n1)&&numel(t0)==numel(t1) && ...
    all( (isnan(t0)&isnan(t1)) | ...
    (isfinite(t0)&isfinite(t1)&abs(t0-t1)<=1e-8*max(1,max(abs(ta)))) );
R.stage_clock_match_not_required=true;
% Record but never assume equivalence from timing alone.
R.gate_summary_match=true;
gateNames={'native_connected','native_success','native_family', ...
 'native_speed_pass','native_heading_pass','native_geometry_pass', ...
 'r26_sim_commit_attempted','r26_sim_commit_succeeded'};
for j=1:numel(gateNames)
 nm=gateNames{j};
 if ~(isfield(b.stage_events,nm)&&isfield(a.stage_events,nm))
     R.gate_summary_match=false;
 elseif ~isequaln(b.stage_events.(nm),a.stage_events.(nm))
     R.gate_summary_match=false;
 end
end
R.reference_p50_before_ms=median(vb(:,ref));
R.reference_p50_after_ms=median(va(:,ref));
R.reference_p95_before_ms=prctile(vb(:,ref),95);
R.reference_p95_after_ms=prctile(va(:,ref),95);
R.reference_max_before_ms=max(vb(:,ref));
R.reference_max_after_ms=max(va(:,ref));
R.full_p50_before_ms=median(vb(:,full));
R.full_p50_after_ms=median(va(:,full));
R.controller_p50_before_ms=median(vb(:,ctrl));
R.controller_p50_after_ms=median(va(:,ctrl));
R.stage_events_before=table(n0,t0,ms0,'VariableNames', ...
 {'Event','Time_s','Stage_ms'});
R.stage_events_after=table(n1,t1,ms1,'VariableNames', ...
 {'Event','Time_s','Stage_ms'});
if R.event_sequence_match
    R.stage_events=table(n1,t1,ms0,ms1,'VariableNames', ...
      {'Event','Time_s','Stage_before_ms','Stage_after_ms'});
else
    R.stage_events=R.stage_events_after;
end
rows=contains(n1,'STAGE1_NATIVE_RETURNED') | ...
 contains(n1,'STAGE2_CERTIFIED_AND_PREPARED') | ...
 contains(n1,'STAGE3_SIM_ONLY_ATOMIC_COMMIT_PASS');
idx=zeros(nnz(rows),1); j=0;
for i=find(rows).'
 j=j+1;
 [~,idx(j)]=min(abs(ta-t1(i)));
 assert(abs(ta(idx(j))-t1(i))<=tol,'Event without matching Flight cycle.');
end
R.critical_times=table(ta(idx),vb(idx,ref),va(idx,ref), ...
 vb(idx,full),va(idx,full),'VariableNames', ...
 {'Time_s','Reference_before_ms','Reference_after_ms', ...
  'Full_before_ms','Full_after_ms'});
R.uncertainty='Single A/B run does not bound future worst-case runtime';
R.safety='Event history equality is not C7, yaw, or geometry equivalence';
save('AF7R58R42_AB_COMPARE.mat','R');
end
function [names,t,ms]=readEvents(events)
if isempty(events)
 names=strings(0,1);t=zeros(0,1);ms=zeros(0,1);return;
end
names=string({events.name}).';
t=double([events.time_s]).';
ms=double([events.stage_ms]).';
% the initial RUN_INITIALIZED event uses NaN time, compare consistently
end
