function OUT = AF7R45B_FLIGHT_ENTRY_AND_10S_AUDIT(logger,replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R45B | FLIGHT-ENTRY + 10 s FINE ATTRIBUTION\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | diagnosticOnly=1 | noAddedOnlineTimers=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
Cref=get_result_cells(A,'reference');
Cctl=get_result_cells(A,'controller');

if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    n=max(numel(Cref),numel(Cctl));
    t=(0:n-1)'*double(replanner.prediction_dt);
end
n=min([numel(t),numel(Cref),numel(Cctl)]);
t=t(1:n);

F=cell(n,1);
online=nan(n,1);
for k=1:n
    F{k}=get_member(Cref{k},'state');
    online(k)=1e3*scalar_or_nan(get_member(Cctl{k},'online_compute_total'));
end

fprintf('\n[A. FLIGHT ENTRY 4.95--5.20 s]\n');
fprintf([' #  t[s] online refE2E observer front load dep base sensorTrig out ' ...
         'bRef fGuard cert active geom [ms]\n']);
idx=find(t>=4.95 & t<=5.20);
for q=1:numel(idx)
    k=idx(q); st=F{k};
    fprintf('%2d %6.3f %6.2f %6.2f %7.3f %5.2f %5.2f %5.2f %5.2f %9.2f %5.2f %5.2f %6.2f %5.2f %6.2f %5.2f\n', ...
        q,t(k),online(k),v(st,'rt_light_e2e_ms'),v(st,'rt_observer_ms'), ...
        v(st,'rt_spike_stage_front_ms'),v(st,'rt_spike_stage_loadmass_ms'), ...
        v(st,'rt_spike_stage_dependency_ms'),v(st,'rt_spike_stage_base_cert_geom_ms'), ...
        v(st,'rt_spike_stage_sensor_trigger_ms'),v(st,'rt_spike_stage_output_state_ms'), ...
        v(st,'rt_spike_base_ref_ms'),v(st,'rt_spike_flight_guard_ms'), ...
        v(st,'rt_spike_cert_guard_ms'),v(st,'rt_spike_active_ref_ms'), ...
        v(st,'rt_spike_geometry_ms'));
end

fprintf('\n[B. 9.75--10.30 s]\n');
fprintf(' #  t[s] online refE2E observer dep sensorTrig out status hard curObs knownObs [ms]\n');
idx10=find(t>=9.75 & t<=10.30);
for q=1:numel(idx10)
    k=idx10(q); st=F{k};
    fprintf('%2d %6.3f %6.2f %6.2f %7.3f %5.2f %9.2f %5.2f %6.0f %5.2f %6.0f %8.0f\n', ...
        q,t(k),online(k),v(st,'rt_light_e2e_ms'),v(st,'rt_observer_ms'), ...
        v(st,'rt_spike_stage_dependency_ms'),v(st,'rt_spike_stage_sensor_trigger_ms'), ...
        v(st,'rt_spike_stage_output_state_ms'),v(st,'generator_last_status_code'), ...
        v(st,'generator_last_cycle_critical_ms'), ...
        v(st,'detected_obstacle_count_point'),v(st,'sensor_known_obstacle_count'));
end

S=replanner.get_avoidance_mode_status();
O=replanner.get_observer_hard_support_status();
M=replanner.get_simulated_sensor_memory_status();

fprintf('\n[C. CURRENT LAYER / OPTIMIZATION STATUS]\n');
fprintf(' commit policy                            = %s\n',char(S.commit_policy));
fprintf(' execution-aware switch                  = %d\n',logical(S.execution_aware_evaluation_enabled));
fprintf(' execution-aware evaluations             = %.0f\n',double(S.execution_aware_evaluation_count));
fprintf(' observer hard-support enabled           = %d\n',logical(O.enabled));
fprintf(' observer runtime mean/max [ms]          = %.6g / %.6g\n', ...
    double(O.runtime_mean_ms),double(O.runtime_max_ms));
fprintf(' ref-only exec-dep fast count            = %.0f\n', ...
    double(replanner.reference_only_exec_dependency_fast_count));
fprintf(' exact runtime-packet reuse count        = %.0f\n', ...
    double(replanner.reference_only_runtime_packet_reuse_count));
fprintf(' runtime-packet rebuild count            = %.0f\n', ...
    double(replanner.reference_only_runtime_packet_rebuild_count));
fprintf(' sensor known final / releases           = %.0f / %.0f\n', ...
    double(M.known_count),double(M.release_count));

% Identify the dominant Flight-entry internal component from existing logs.
names={'observer','dependency','base/cert/geom','sensor/trigger','output/state'};
vals=zeros(1,numel(names));
entry=find(t>=5.0 & t<=5.15);
mat=[series(F,entry,'rt_observer_ms'), ...
     series(F,entry,'rt_spike_stage_dependency_ms'), ...
     series(F,entry,'rt_spike_stage_base_cert_geom_ms'), ...
     series(F,entry,'rt_spike_stage_sensor_trigger_ms'), ...
     series(F,entry,'rt_spike_stage_output_state_ms')];
for j=1:size(mat,2)
    z=mat(:,j); z=z(isfinite(z));
    if isempty(z), vals(j)=NaN; else, vals(j)=max(z); end
end
vv=vals; vv(~isfinite(vv))=-Inf;
[domVal,j]=max(vv);
fprintf(' Flight-entry dominant measured stage     = %s (max %.6g ms)\n',names{j},domVal);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R45B_FLIGHT_ENTRY_10S_AUDIT_V1', ...
    'avoidance',S,'observer',O,'memory',M, ...
    'flight_entry_dominant_stage',names{j}, ...
    'flight_entry_dominant_ms',domVal);
assignin('base','AF7R45B_FLIGHT_ENTRY_10S_AUDIT_OUT',OUT);
end

function C=get_result_cells(A,name)
C={};
if ~isstruct(A)||~isfield(A,name), return; end
S=A.(name);
if ~isstruct(S)||~isfield(S,'result'), return; end
R=S.result;
if iscell(R), C=R(:); else, C=num2cell(R(:)); end
end

function x=get_member(a,n)
x=[];
try
    if isstruct(a)&&isfield(a,n), x=a.(n);
    elseif isobject(a)&&isprop(a,n), x=a.(n);
    end
catch
    x=[];
end
end

function y=scalar_or_nan(x)
y=NaN;
if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x))
    y=double(x);
end
end

function y=v(st,name)
y=scalar_or_nan(get_member(st,name));
end

function z=series(F,idx,name)
z=nan(numel(idx),1);
for i=1:numel(idx), z(i)=v(F{idx(i)},name); end
end
