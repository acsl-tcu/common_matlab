function R=AF7R58R36_TIMING_CLOSE(logger,runtime_profile,spike_mat_path)
% R36R1: read-only timing closure; all temporal series are Nx1 columns.
% Fixes R36 row/column implicit expansion (which could create NxN arrays).
% Supports offline recovery from an already saved R23 spike MAT:
%    R=AF7R58R36_TIMING_CLOSE([],[], 'AF7R58R23_SPIKE_REPORT.mat');
% No REPLANNING_SYSTEM, controller, safety, or commit path is modified.

if nargin<1, logger=[]; end
if nargin<2, runtime_profile=[]; end
if nargin<3, spike_mat_path=''; end

D=struct();
if ~isempty(spike_mat_path)
    assert(isfile(spike_mat_path), 'R36R1:MissingFile', ...
        'R23 timing MAT does not exist: %s', char(spike_mat_path));
    D=load(spike_mat_path, 'P', 'T');
end
if isempty(runtime_profile)
    assert(isfield(D,'P'), 'R36R1:MissingProfile', ...
        'No runtime_profile was supplied and the R23 MAT has no P variable.');
    P=D.P;
    profile_source='R23_TIMING_MAT';
else
    P=runtime_profile;
    profile_source='LIVE_RUNTIME_PROFILE';
end

req={'t','phase','r23_reference_s','full_iteration_s','r23_controller_s', ...
    'r23_estimator_s','simulation_plant_s','logger_s','online_before_plant_s', ...
    'sensor_acquisition_s','r23_sensor_s','r23_input_s','r23_other_s'};
assert(isstruct(P) && isscalar(P) && all(isfield(P,req)), ...
    'R36R1:MissingFields','Runtime profile is missing required R23 fields.');

all_t=double(P.t(:));
assert(isreal(all_t) && all(isfinite(all_t)), 'R36R1:InvalidTime', ...
    'Time vector must be real and finite.');
N_all=numel(all_t);
ph=P.phase;
if isstring(ph) && isscalar(ph), ph=char(ph); end
ph=ph(:);
assert(numel(ph)==N_all, 'R36R1:PhaseLength', ...
    'Profile phase and time have different numbers of samples.');
mask=ph=='f';
t=all_t(mask); N=numel(t);
assert(N>0 && all(diff(t)>0), 'R36R1:InvalidFlightTime', ...
    'Flight time vector must be nonempty and strictly increasing.');

R=struct();
R.schema='AF7R58R36_TIMING_CLOSURE_R1';
R.profile_source=profile_source;
R.time_s=t;
R.outer_reference_ms=local_flight_ms(P,'r23_reference_s',mask,N_all);
R.full_iteration_ms=local_flight_ms(P,'full_iteration_s',mask,N_all);
R.controller_ms=local_flight_ms(P,'r23_controller_s',mask,N_all);
R.estimator_ms=local_flight_ms(P,'r23_estimator_s',mask,N_all);
R.plant_ms=local_flight_ms(P,'simulation_plant_s',mask,N_all);
R.logger_ms=local_flight_ms(P,'logger_s',mask,N_all);
R.online_ms=local_flight_ms(P,'online_before_plant_s',mask,N_all);
R.sensor_acquisition_ms=local_flight_ms(P,'sensor_acquisition_s',mask,N_all);
R.sensor_ms=local_flight_ms(P,'r23_sensor_s',mask,N_all);
R.input_ms=local_flight_ms(P,'r23_input_s',mask,N_all);
R.other_ms=local_flight_ms(P,'r23_other_s',mask,N_all);

% Every term in both sums is Nx1: MATLAB implicit expansion is impossible.
R.full_component_sum_ms=R.sensor_acquisition_ms+R.sensor_ms+ ...
    R.estimator_ms+R.outer_reference_ms+R.controller_ms+ ...
    R.input_ms+R.plant_ms+R.logger_ms+R.other_ms;
R.full_cycle_closure_ms=R.full_iteration_ms-R.full_component_sum_ms;
R.online_component_sum_ms=R.sensor_acquisition_ms+R.sensor_ms+ ...
    R.estimator_ms+R.outer_reference_ms+R.controller_ms+R.input_ms;
R.online_cycle_closure_ms=R.online_ms-R.online_component_sum_ms;

% Do not treat unavailable LOGGER fields as zero-duration work.
names={'rtime_e_to_e_ms','rtime_observer_ms', ...
    'rtime_sensor_gate_ms','rtime_sensor_memory_ms', ...
    'rtime_trigger_kernel_ms','rtime_request_ms','rtime_advance_ms', ...
    'rtime_sensor_total_ms','rtime_sensor_closure_ms', ...
    'rtime_stage_start_ms','rtime_stage_advance_ms','rtime_capture_ms'};
R.missing=cell(0,2); R.timings=struct();
for j=1:numel(names)
    nm=names{j};
    if isempty(logger)
        R.missing(end+1,:)={nm,'LOGGER not provided; offline R23 outer timing only'}; %#ok<AGROW>
        continue;
    end
    try
        [v,~]=logger.data(1,"reference.result.state."+string(nm),"",'phase','f');
        assert(isnumeric(v) && isreal(v), 'R36R1:LoggerType', ...
            'LOGGER series %s is not real numeric.',nm);
        v=double(v(:));
        assert(numel(v)==N, 'R36R1:LoggerLength', ...
            'LOGGER series %s has %d samples, expected %d.',nm,numel(v),N);
        R.timings.(nm)=v;
    catch ME
        R.missing(end+1,:)={nm,sprintf('%s: %s',ME.identifier,ME.message)}; %#ok<AGROW>
    end
end

if isfield(R.timings,'rtime_e_to_e_ms')
    R.internal_do_ms=R.timings.rtime_e_to_e_ms;
    R.wrapper_and_adjacent_ms=R.outer_reference_ms-R.internal_do_ms;
else
    R.internal_do_ms=nan(N,1);
    R.wrapper_and_adjacent_ms=nan(N,1);
end

parts={'rtime_sensor_gate_ms','rtime_sensor_memory_ms', ...
    'rtime_trigger_kernel_ms','rtime_request_ms','rtime_advance_ms'};
R.sensor_closure_verified=false;
R.sensor_partition_residual_ms=nan(N,1);
if all(isfield(R.timings,[parts {'rtime_sensor_total_ms'}]))
    M=zeros(N,1);
    for j=1:numel(parts)
        M=M+R.timings.(parts{j});
    end
    measured=R.timings.rtime_sensor_total_ms;
    R.sensor_partition_residual_ms=measured-M;
    finite=isfinite(measured)&isfinite(M);
    if any(finite)
        R.sensor_closure_verified=all(abs(R.sensor_partition_residual_ms(finite)) <= ...
            1e-8*max(1,abs(measured(finite))));
    end
end

% Validate shape BEFORE ranking and table construction.
series={R.time_s,R.outer_reference_ms,R.internal_do_ms, ...
    R.wrapper_and_adjacent_ms,R.full_iteration_ms, ...
    R.full_cycle_closure_ms,R.online_cycle_closure_ms};
assert(all(cellfun(@(v) isequal(size(v),[N 1]),series)), ...
    'R36R1:Shape','All Flight timing series must be Nx1.');
[~,ind]=sort(R.outer_reference_ms,'descend');
ind=ind(1:min(30,N)); ind=ind(:);
R.top_cycles=table(t(ind),R.outer_reference_ms(ind), ...
    R.internal_do_ms(ind),R.wrapper_and_adjacent_ms(ind), ...
    R.full_iteration_ms(ind), ...
    'VariableNames',{'time_s','outer_ms','replanner_do_ms','unattributed_ms','full_ms'});
assert(height(R.top_cycles)==numel(ind), 'R36R1:Table', ...
    'Top-cycle table row count is not consistent.');

R.reference_p50_ms=median(R.outer_reference_ms);
R.reference_p95_ms=prctile(R.outer_reference_ms,95);
R.reference_max_ms=max(R.outer_reference_ms);
R.online_over_25_count=nnz(R.online_ms>25);
R.full_over_25_count=nnz(R.full_iteration_ms>25);
R.known_controller_spike_idx=find(R.controller_ms>100);
R.max_abs_full_cycle_closure_ms=max(abs(R.full_cycle_closure_ms));
R.max_abs_online_cycle_closure_ms=max(abs(R.online_cycle_closure_ms));
R.flight_cycle_count=N;

if isfield(D,'T')
    assert(isfield(D.T,'stage_events') && isfield(D.T,'time_s'), ...
        'R36R1:MalformedMAT','R23 timing MAT lacks T.time_s or T.stage_events.');
    tf=double(D.T.time_s(:));
    assert(numel(tf)==N && all(abs(tf-t)<=max(1e-8,4*eps(max(1,max(abs(t)))))), ...
        'R36R1:RunMismatch', ...
        'R23 MAT Flight time grid does not match supplied runtime_profile.');
    if ~isempty(runtime_profile) && isfield(D,'P')
        reference_from_mat=local_flight_ms(D.P,'r23_reference_s',mask,N_all);
        assert(all(abs(reference_from_mat-R.outer_reference_ms)<=1e-9), ...
            'R36R1:RunMismatch', ...
            'R23 MAT and runtime_profile differ; do not combine different runs.');
    end
    R.stage_events=D.T.stage_events;
end

save('AF7R58R36_TIMING_CLOSE.mat','R');
writetable(R.top_cycles,'AF7R58R36_TOP_REFERENCE.csv');
fprintf('[R36R1] Flight cycles=%d | reference p50/p95/max=%.4f / %.4f / %.4f ms\n', ...
    N,R.reference_p50_ms,R.reference_p95_ms,R.reference_max_ms);
fprintf('[R36R1] full/online closure max abs=%.9g / %.9g ms | full/online >25=%d/%d\n', ...
    R.max_abs_full_cycle_closure_ms,R.max_abs_online_cycle_closure_ms, ...
    R.full_over_25_count,R.online_over_25_count);
fprintf('[R36R1] LOGGER missing fields=%d | sensor closure verified=%d\n', ...
    size(R.missing,1),R.sensor_closure_verified);
end

function v=local_flight_ms(P,field,mask,N_all)
raw=P.(field);
assert(isnumeric(raw) && isreal(raw) && numel(raw)==N_all, ...
    'R36R1:ProfileLength', ...
    'Profile field %s must contain exactly %d real numeric samples.',field,N_all);
raw=double(raw(:));
v=1000*raw(mask);
assert(iscolumn(v) && numel(v)==nnz(mask), 'R36R1:ProfileShape', ...
    'Profile field %s could not be normalized to a Flight column.',field);
end
