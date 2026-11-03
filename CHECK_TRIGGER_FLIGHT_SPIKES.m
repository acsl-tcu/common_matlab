function Q = CHECK_TRIGGER_FLIGHT_SPIKES(logger,replanner_instance)
% CHECK_TRIGGER_FLIGHT_SPIKES  Offline analysis of the EXISTING Flight log.
% No sensor scans, EKF updates, HLC/plant calls, predictions, or MEX builds.
% Usage: Q = CHECK_TRIGGER_FLIGHT_SPIKES(logger,replanner_instance);
assert(nargin==2 && isobject(logger) && isobject(replanner_instance) && ...
    ismethod(replanner_instance,'get_status'), ...
    'CHECK_TRIGGER_FLIGHT_SPIKES:Inputs', ...
    'Pass logger and replanner_instance from the completed Flight.');
D=logger.Data;
assert(isstruct(D) && isfield(D,'t') && isfield(D,'agent') && ...
    ~isempty(D.agent) && isfield(D.agent(1),'reference') && ...
    isfield(D.agent(1).reference,'result'), ...
    'CHECK_TRIGGER_FLIGHT_SPIKES:Logger', ...
    'Expected logged Data.t and Data.agent(1).reference.result.');
R=D.agent(1).reference.result;
if ~iscell(R), R=num2cell(R(:)); else, R=R(:); end
n=min([double(logger.k),numel(D.t),numel(R)]);
assert(n>0,'CHECK_TRIGGER_FLIGHT_SPIKES:NoFrames','No frames in the Flight log.');
t=double(D.t(1:n));t=t(:);
rp=nan(n,1);tr=nan(n,1);sn=nan(n,1);
pred=false(n,1);complete=false(n,1);native=false(n,1);
hor=nan(n,1);samples=nan(n,1);known=cell(n,1);new=cell(n,1);
trigger_status=cell(n,1);dispatch=cell(n,1);
for k=1:n
    x=R{k};
    rp(k)=local_num(x,'replanning_time_ms');
    tr(k)=local_num(x,'trigger_time_ms');
    s=local_field(x,'simulated_obstacle_sensor');
    sn(k)=local_num(s,'elapsed_ms');
    m=local_field(x,'observed_obstacle_memory');
    known{k}=local_id(m,'known_obstacle_ids');
    new{k}=local_id(m,'newly_acquired_ids');
    q=local_field(x,'future_collision_trigger');
    pred(k)=local_yes(q,'prediction_available');
    complete(k)=local_yes(q,'prediction_complete');
    hor(k)=local_num(q,'completed_prediction_horizon_s');
    samples(k)=local_num(q,'completed_prediction_samples');
    dispatch{k}=local_str(q,'predictor_dispatch');
    trigger_status{k}=local_str(q,'status');
    native(k)=strcmp(dispatch{k},'DIRECT_NATIVE_VERIFIED_BINDING');
end
S=replanner_instance.get_status();
Q=struct();
Q.status='REPORT_CREATED__NOMINAL_ONLY__NOT_FLIGHT_SAFETY';
Q.n=n; Q.time_s=t; Q.replanner_ms=rp; Q.trigger_ms=tr;
Q.sensor_ms=sn; Q.prediction_available=pred;
Q.prediction_complete=complete; Q.completed_horizon_s=hor;
Q.completed_samples=samples; Q.direct_native=native;
Q.known_ids={known}; Q.new_ids={new};
Q.trigger_status={trigger_status}; Q.predictor_dispatch={dispatch};
Q.preflight_status=S;
Q.replanner_summary=local_stats(rp);
Q.trigger_all_summary=local_stats(tr);
Q.trigger_predicted_summary=local_stats(tr(pred));
Q.trigger_complete_summary=local_stats(tr(complete));
Q.first_5sec_summary=local_stats(tr(t>=5 & t<5.15));
Q.steady_flight_summary=local_stats(tr(t>=6 & t<42));
Q.n_predicted=nnz(pred); Q.n_completed=nnz(complete);
Q.n_direct_native=nnz(native & pred);
Q.n_incomplete=nnz(pred & ~complete);
Q.n_full_four_sec=nnz(complete & abs(hor-4)<=1e-9);
Q.n_predicted_non_native=nnz(pred & ~native);
Q.n_missing_horizon=nnz(complete & abs(hor-4)>1e-9);
valid_idx=find(isfinite(tr));
[~,sort_idx]=sort(tr(valid_idx),'descend');
order=valid_idx(sort_idx);
Q.top_spike_indices=order(1:min(numel(order),15));
Q.acquisition_indices=find(~cellfun(@isempty,new));
Q.first_trace_enabled=local_field(S,'first_flight_trace_enabled');
Q.first_traces=local_field(S,'first_flight_traces');
fid=fopen('TRIGGER_FLIGHT_SPIKES_RESULT.txt','w');
assert(fid>0,'CHECK_TRIGGER_FLIGHT_SPIKES:File','Cannot write TXT in current folder.');
c=onCleanup(@()fclose(fid));
local_print(fid,'=== FLIGHT / TRIGGER SPIKE AUDIT ===\n');
local_print(fid,'status=%s\nframes=%d\n',Q.status,n);
local_print(fid,'prediction_available=%d\nprediction_complete=%d\n',Q.n_predicted,Q.n_completed);
local_print(fid,'full_4s_complete=%d\ndirect_native_predicted=%d\n',Q.n_full_four_sec,Q.n_direct_native);
local_print(fid,'predicted_non_native=%d\nincomplete_available=%d\ncompleted_not_4s=%d\n', ...
    Q.n_predicted_non_native,Q.n_incomplete,Q.n_missing_horizon);
local_print(fid,'replanner_all: %s\n',local_fmt(Q.replanner_summary));
local_print(fid,'trigger_all: %s\n',local_fmt(Q.trigger_all_summary));
local_print(fid,'trigger_predicted_only: %s\n',local_fmt(Q.trigger_predicted_summary));
local_print(fid,'trigger_5.000_5.150: %s\n',local_fmt(Q.first_5sec_summary));
local_print(fid,'trigger_steady_6_42s: %s\n',local_fmt(Q.steady_flight_summary));
for j={'native_mex_preflight_build_status','preflight_model_binding_status', ...
       'trigger_native_gate_status','isolated_first_use_prepare_status'}
    key=j{1}; v=local_field(S,key);
    if ischar(v) || (isstring(v)&&isscalar(v))
        local_print(fid,'%s=%s\n',key,char(v));
    end
end
local_print(fid,'\n--- top trigger elapsed spikes ---\n');
local_print(fid,'k  t[s]  trigger[ms]  replanner[ms]  sensor[ms]  pred  complete  native  full_h[s]  known_ids  new_ids  dispatch  status\n');
for i=reshape(Q.top_spike_indices,1,[])
    local_print(fid,'%d %.9f %.5f %.5f %.5f %d %d %d %.6f %s %s %s %s\n', ...
        i,t(i),tr(i),rp(i),sn(i),pred(i),complete(i),native(i), ...
        hor(i),mat2str(known{i}),mat2str(new{i}),dispatch{i},trigger_status{i});
end
local_print(fid,'\n--- 4.950..5.250s first-Flight window ---\n');
local_print(fid,'k  t[s]  trigger[ms] replanner[ms] sensor[ms] pred complete native horizon[s] known_ids new_ids\n');
for i=reshape(find(t>=4.95 & t<=5.25),1,[])
    local_print(fid,'%d %.9f %.5f %.5f %.5f %d %d %d %.6f %s %s\n', ...
        i,t(i),tr(i),rp(i),sn(i),pred(i),complete(i),native(i), ...
        hor(i),mat2str(known{i}),mat2str(new{i}));
end
local_print(fid,'\n--- newly acquired observed obstacle IDs ---\n');
for i=reshape(Q.acquisition_indices,1,[])
    local_print(fid,'k=%d t=%.9f new=%s known=%s trigger=%.5f ms\n', ...
        i,t(i),mat2str(new{i}),mat2str(known{i}),tr(i));
end
local_print(fid,'\n--- first-Flight embedded trace, if enabled BEFORE Flight ---\n');
if isstruct(Q.first_traces) && ~isempty(Q.first_traces)
    tt=Q.first_traces;
    for i=1:numel(tt)
        local_print(fid,'trace %d t=%.9f: %s\n',i,local_num(tt(i),'time_s'), ...
            local_str(tt(i),'trigger_status'));
        ff=fieldnames(tt(i));
        for j=1:numel(ff)
            v=tt(i).(ff{j});
            if isnumeric(v)&&isscalar(v)
                local_print(fid,'  %s: %.6f\n',ff{j},double(v));
            end
        end
        if isfield(tt(i),'core_trace') && isstruct(tt(i).core_trace)
            ff=fieldnames(tt(i).core_trace);
            for j=1:numel(ff)
                v=tt(i).core_trace.(ff{j});
                if isnumeric(v)&&isscalar(v)
                    local_print(fid,'  core_trace.%s: %.6f\n',ff{j},double(v));
                end
            end
        end
    end
else
    local_print(fid,'not captured; enable TRIGGER_FIRST_FLIGHT_TRACE=1 before a NEW Flight if needed\n');
end
save('TRIGGER_FLIGHT_SPIKES_RESULT.mat','Q','-v7.3');
fprintf('[CHECK_TRIGGER_FLIGHT_SPIKES] %s\n',Q.status);
fprintf('predicted=%d completed=%d full_4s=%d direct_native=%d\n', ...
    Q.n_predicted,Q.n_completed,Q.n_full_four_sec,Q.n_direct_native);
fprintf('replanner: %s\ntrigger(predicted only): %s\n', ...
    local_fmt(Q.replanner_summary),local_fmt(Q.trigger_predicted_summary));
fprintf('Saved TRIGGER_FLIGHT_SPIKES_RESULT.txt / .mat\n');
end
function v=local_field(s,k)
v=[];
if isstruct(s)&&isscalar(s)&&isfield(s,k),v=s.(k);end
end
function v=local_num(s,k)
v=NaN;x=local_field(s,k);
if isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x),v=double(x);end
end
function v=local_yes(s,k)
x=local_field(s,k);v=isscalar(x)&&... 
    (isnumeric(x)||islogical(x))&&isreal(x)&&isequal(double(x),1);
end
function v=local_id(s,k)
x=local_field(s,k);v=[];
if isnumeric(x)&&isreal(x)&&isvector(x)&&all(isfinite(x(:)))
    v=reshape(double(x),1,[]);
end
end
function v=local_str(s,k)
v='';x=local_field(s,k);
if ischar(x),v=x;elseif isstring(x)&&isscalar(x),v=char(x);end
end
function A=local_stats(x)
x=sort(x(isfinite(x)&x>=0));A=struct('n',numel(x), ...
    'median_ms',NaN,'p95_ms',NaN,'p99_ms',NaN,'max_ms',NaN);
if isempty(x),return;end
A.median_ms=median(x);A.p95_ms=x(max(1,ceil(.95*numel(x))));
A.p99_ms=x(max(1,ceil(.99*numel(x))));A.max_ms=x(end);
end
function s=local_fmt(a)
s=sprintf('N=%d median=%.4f ms P95=%.4f ms P99=%.4f ms max=%.4f ms', ...
    a.n,a.median_ms,a.p95_ms,a.p99_ms,a.max_ms);
end
function local_print(fid,varargin)
fprintf(fid,varargin{:});fprintf(1,varargin{:});
end
