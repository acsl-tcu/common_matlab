function A = AF7R58R47_TIMING_SOURCE_SPLIT(report_file)
% AF7R58R47_TIMING_SOURCE_SPLIT  Read-only R23 main-loop attribution.
% Input: original AF7R58R23_SPIKE_REPORT.txt from SAME simulation run.
% The TOP 30 lines are ranked events, not a complete per-cycle trace.
% Recorded P50/P95/MAX are parsed from the original report; no averaging
% of spikes, no arbitrary tail trimming and no production instrumentation.

assert(ischar(report_file) || (isstring(report_file)&&isscalar(report_file)), ...
    '[R47] Supply explicit R23 report filename.');
report_file=char(report_file);
assert(isfile(report_file),'[R47] Timing report not found: %s',report_file);
raw=fileread(report_file);
assert(contains(raw,'AF7R58R23 | FULL ITERATION SPIKE ATTRIBUTION'), ...
    '[R47] The input is not the expected original R23 cycle timing report.');
ls=splitlines(string(raw));
needle='time_s full online acquire sensor estimator reference controller input plant logger other';
i=find(contains(strtrim(ls),needle),1,'first');
assert(~isempty(i),'[R47] R23 TOP spikes column schema not found.');
X=zeros(0,12);
for k=i+1:numel(ls)
    s=strtrim(char(ls(k)));
    if isempty(s) || startsWith(s,'REPLANNING EVENTS'),break;end
    v=sscanf(s,'%f').';
    if isempty(v),break;end
    assert(numel(v)==12,'[R47] Malformed R23 spike row at line %d.',k);
    X(end+1,:)=v; %#ok<AGROW>
end
assert(~isempty(X) && all(isfinite(X(:))),'[R47] No finite cycle timing samples.');
components={'sensor_acquisition','sensor','estimator', ...
    'reference_replanner','controller','input_transform', ...
    'simulation_plant','logger','other_overhead'};
recordNames=[components,{'online_before_plant','full_iteration'}];
A=struct();
A.schema='AF7R58R47_TIMING_SOURCE_SPLIT_FROM_R23_V1';
A.report_file=report_file;
A.top_rows=X;
A.top_row_count=size(X,1);
A.component_names=components;
A.meaning='Top reported cycle rows only; summary from the complete recorded flight.';
A.top_row_max_budget_exceed_count=sum(X(:,2)>25); % 25 ms is an explicit system period.
A.component_sum_residual_ms=X(:,2)-sum(X(:,4:12),2);
A.max_abs_component_sum_residual_ms=max(abs(A.component_sum_residual_ms));
[~,idx]=max(X(:,4:12),[],2);
A.dominant_components=components(idx);
A.max_spike_time_s=X(1,1);
A.max_spike_full_ms=X(1,2);
A.max_spike_dominant_component=A.dominant_components{1};
A.max_spike_dominant_component_ms=X(1,idx(1)+3);
A.production_modified=false;
A.report_is_measurement_not_WCET_proof=true;
% Separate complete-flight summarized statistics from top-30-only rows.
summary=struct();
for j=1:numel(recordNames)
    key=recordNames{j};
    re=['(?m)^' regexptranslate('escape',key) ...
        '\s+p50=\s*([+\-0-9.eE]+)\s+p95=\s*([+\-0-9.eE]+)\s+max=\s*([+\-0-9.eE]+)'];
    m=regexp(raw,re,'tokens','once');
    assert(~isempty(m),'[R47] Summary bucket missing: %s',key);
    summary.(key)=str2double(m);
end
A.all_flight_summary_ms=summary;
m=regexp(raw,'Recorded Flight cycles\s*=\s*(\d+)','tokens','once');
assert(~isempty(m),'[R47] Recorded Flight cycle count missing.');
A.recorded_flight_cycles=str2double(m{1});
assert(A.recorded_flight_cycles>=A.top_row_count, ...
    '[R47] Timing report has inconsistent cycle count.');
A.stage1_native_returned_ms=[];
A.stage1_native_event_t_s=[];
for k=1:numel(ls)
    s=char(ls(k));
    if ~contains(s,'STAGE1_NATIVE_RETURNED'),continue;end
    m=regexp(s,'sim_t=([+\-0-9.eE]+)\s*\|\s*stage=([+\-0-9.eE]+)', ...
        'tokens','once');
    if ~isempty(m)
        A.stage1_native_event_t_s(end+1,1)=str2double(m{1}); %#ok<AGROW>
        A.stage1_native_returned_ms(end+1,1)=str2double(m{2}); %#ok<AGROW>
    end
end
A.status='READ_ONLY_FULL_CYCLE_SOURCE_ATTRIBUTION__NOT_A_HARD_RT_CERTIFICATE';
fprintf('\n[R47/TIMING] %d reported flight cycles; %d top spike rows\n', ...
    A.recorded_flight_cycles,A.top_row_count);
fprintf('[R47/TIMING] reference p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    summary.reference_replanner);
fprintf('[R47/TIMING] HLC p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    summary.controller);
fprintf('[R47/TIMING] full p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    summary.full_iteration);
fprintf('[R47/TIMING] largest cycle %.9f s = %.6f ms, dominated by %s (%.6f ms)\n', ...
    A.max_spike_time_s,A.max_spike_full_ms, ...
    A.max_spike_dominant_component,A.max_spike_dominant_component_ms);
fprintf('[R47/TIMING] sum closure worst residual %.6g ms\n', ...
    A.max_abs_component_sum_residual_ms);
fprintf('[R47/TIMING] %s\n',A.status);
end
