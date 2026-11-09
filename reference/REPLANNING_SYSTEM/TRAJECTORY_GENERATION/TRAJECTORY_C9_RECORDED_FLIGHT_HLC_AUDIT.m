function R=TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT(logger_or_data,options)
%TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT Read the ORIGINAL Sim LOGGER.
% No runner/HLC/plant modification and NO extra work inside Flight cycles.
% LOGGER.logging already records Data.agent(1).controller.result{k},
% reference.result{k}, estimator.result{k}, Data.phase and Data.t.
% This function checks ACTUAL RECORDED post-HLC values, not fixtures.
% A plan cannot be inferred from a nominal reference: planned_ref28 must
% be explicitly provided when a generated C9 plan has truly been selected.
% Actual HLC loadmass was not archived by the legacy LOGGER; estimator
% mass is observable, but equality to HLC's consumed mass is NOT proven.
% All results are SAMPLED diagnostics, NEVER a continuous-time guarantee.
clock=tic;
if nargin<2, options=struct();end
assert(isstruct(options)&&isscalar(options), ...
    'C9_FLIGHT_TRACE:Options','Options must be a scalar struct.');
if isobject(logger_or_data)
    assert(isprop(logger_or_data,'Data'), ...
        'C9_FLIGHT_TRACE:Logger','Expected a Sim LOGGER with Data.');
    D=logger_or_data.Data;
elseif isstruct(logger_or_data)&&isscalar(logger_or_data)&& ...
        isfield(logger_or_data,'Data')
    D=logger_or_data.Data;
else
    D=logger_or_data;
end
assert(isstruct(D)&&isscalar(D)&& ...
    all(isfield(D,{'t','phase','agent'}))&& ...
    numel(D.agent)>=1&& ...
    isfield(D.agent(1),'controller')&& ...
    isfield(D.agent(1),'reference')&& ...
    isfield(D.agent(1),'estimator'), ...
    'C9_FLIGHT_TRACE:Data','The original Sim LOGGER fields are required.');
C=D.agent(1).controller.result;
F=D.agent(1).reference.result;
E=D.agent(1).estimator.result;
assert(iscell(C)&&iscell(F)&&iscell(E), ...
    'C9_FLIGHT_TRACE:Cells','LOGGER result histories must be cells.');
N=numel(C);
assert(N>=1&&numel(F)>=N&&numel(E)>=N&& ...
    numel(D.t)>=N&&numel(D.phase)>=N, ...
    'C9_FLIGHT_TRACE:History','Incomplete recording arrays.');
phase=D.phase(:);
if isnumeric(phase)
    is_flight=(phase(1:N)==double('f'));
elseif ischar(phase)
    is_flight=(phase(1:N)=='f');
elseif isstring(phase)||iscellstr(phase)
    is_flight=strcmp(string(phase(1:N)),"f");
else
    error('C9_FLIGHT_TRACE:Phase','Unknown Sim phase encoding.');
end
ids=find(is_flight(:)); M=numel(ids);
[lo,hi,source]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
R=struct('status','UNKNOWN_REAL_FLIGHT_LOG_NOT_CHECKED', ...
    'total_logged_records',N,'flight_records',M, ...
    'checked_flight_records',0,'failed_flight_records',0, ...
    'first_failed_record_index',NaN,'first_failed_time_s',NaN, ...
    'source_checked_at_audit_time',source, ...
    'source_preflight_identity_proven',false, ...
    'hlc_mass_consumed_each_tick_verified',false, ...
    'fixture_only',false,'sampled_HLC_trace_evaluated',false, ...
    'sampled_all_input_and_reference_checks_pass',false, ...
    'planned_reference_supplied',false,'plan_to_actual_reference_match',false, ...
    'physical_continuous_time_certified',false,'safe_to_commit',false, ...
    'time_s',reshape(double(D.t(ids)),1,[]), ...
    'logger_index',reshape(ids,1,[]), ...
    'sample_status',{{}},'actual_ref28',NaN(28,M), ...
    'hlc_internal_ref28',NaN(28,M), ...
    'unclamped_input4',NaN(4,M),'clamped_input4',NaN(4,M), ...
    'estimated_payload_mass_kg',NaN(1,M), ...
    'raw_input_headroom_min',NaN(1,M), ...
    'clamp_mismatch_count',0,'preclamp_saturation_count',0, ...
    'yaw_lift_mismatch_count',0,'reference_mismatch_count',0, ...
    'missing_or_invalid_record_count',0, ...
    'elapsed_offline_ms',NaN, ...
    'scope','ACTUAL_SAVED_SIM_LOGGER_SAMPLED_DIAGNOSTIC__NO_HLC_PROOF');
if M==0
    R.status='UNKNOWN_NO_RECORDED_FLIGHT_TICKS';
    R.elapsed_offline_ms=1000*toc(clock);return
end
assert(all(isfinite(R.time_s))&&all(diff(R.time_s)>0), ...
    'C9_FLIGHT_TRACE:Timeline','Flight timestamps must increase.');
planned=false; refs=[];tols=[];
if isfield(options,'planned_ref28')||isfield(options,'reference_tolerance28')
    assert(all(isfield(options,{'planned_ref28','reference_tolerance28'})), ...
        'C9_FLIGHT_TRACE:Plan','Provide BOTH planned 28 refs and tolerances.');
    refs=options.planned_ref28;
    tols=options.reference_tolerance28;
    assert(isnumeric(refs)&&isreal(refs)&&all(isfinite(refs(:)))&& ...
        size(refs,1)==28&&(size(refs,2)==M||size(refs,2)==N)&& ...
        isnumeric(tols)&&isreal(tols)&& ...
        isequal(size(tols),[28 1])&&all(isfinite(tols))&&all(tols>=0), ...
        'C9_FLIGHT_TRACE:Plan','The explicit true planned trace is malformed.');
    planned=true;
end
R.planned_reference_supplied=planned;
R.sample_status=cell(1,M);
for j=1:M
    k=ids(j);
    try
        cc=C{k};rf=F{k};est=E{k};
        assert(isstruct(cc)&&all(isfield(cc,{'tmp','input','xd'}))&& ...
            isstruct(rf)&&isfield(rf,'state')&& ...
            isstruct(est)&&isfield(est,'state'), ...
            'C9_FLIGHT_TRACE:Record','Missing Sim LOGGER record members.');
        ss=rf.state;
        if isstruct(ss)&&isfield(ss,'xd')
            sent=ss.xd;
        elseif isobject(ss)&&isprop(ss,'xd')
            sent=ss.xd;
        elseif isobject(ss)&&ismethod(ss,'get')
            sent=ss.get();
        else
            error('C9_FLIGHT_TRACE:Ref','No saved actual reference28.');
        end
        mass=est.state.mL;
        assert(isnumeric(mass)&&isreal(mass)&& ...
            isscalar(mass)&&isfinite(mass)&&mass>0, ...
            'C9_FLIGHT_TRACE:Mass','Missing/invalid recorded EKF mass.');
        if planned
            pindex=j;
            if size(refs,2)==N,pindex=k;end
            target=refs(:,pindex);tol=tols;
        else
            target=[];tol=[];
        end
        T=TRAJECTORY_C9_HLC_SAMPLE_VERIFY(sent,cc.xd,cc.tmp, ...
            cc.input,lo,hi,target,tol);
        R.actual_ref28(:,j)=double(sent);
        R.hlc_internal_ref28(:,j)=double(cc.xd);
        R.unclamped_input4(:,j)=double(cc.tmp);
        R.clamped_input4(:,j)=double(cc.input);
        R.estimated_payload_mass_kg(j)=double(mass);
        R.raw_input_headroom_min(j)=T.minimum_raw_actuator_headroom;
        R.sample_status{j}=T.status;
        R.clamp_mismatch_count=R.clamp_mismatch_count+double(~T.hlc_clamp_consistent);
        R.preclamp_saturation_count=R.preclamp_saturation_count+double(~T.raw_input_feasible);
        R.yaw_lift_mismatch_count=R.yaw_lift_mismatch_count+double(~T.internal_yaw_lift_valid);
        if planned
            R.reference_mismatch_count=R.reference_mismatch_count+double(~T.reference_match);
        end
        if T.sample_checked
            R.checked_flight_records=R.checked_flight_records+1;
        else
            R.failed_flight_records=R.failed_flight_records+1;
            if isnan(R.first_failed_record_index)
                R.first_failed_record_index=k;
                R.first_failed_time_s=R.time_s(j);
            end
        end
    catch ME
        R.sample_status{j}=['UNKNOWN_LOG_RECORD_' ME.identifier];
        R.failed_flight_records=R.failed_flight_records+1;
        R.missing_or_invalid_record_count=R.missing_or_invalid_record_count+1;
        if isnan(R.first_failed_record_index)
            R.first_failed_record_index=k;
            R.first_failed_time_s=R.time_s(j);
        end
    end
end
R.sampled_HLC_trace_evaluated=true;
R.sampled_all_input_and_reference_checks_pass= ...
    R.checked_flight_records==M && R.failed_flight_records==0;
R.plan_to_actual_reference_match=planned&&R.reference_mismatch_count==0&& ...
    R.missing_or_invalid_record_count==0;
if ~R.sampled_all_input_and_reference_checks_pass
    R.status='REJECT_OR_UNKNOWN_RECORDED_FLIGHT_HLC_SAMPLES__NO_COMMIT';
elseif planned
    R.status='CHECKED_ALL_RECORDED_FLIGHT_SAMPLES_AND_PLAN__NO_HLC_PROOF';
else
    R.status='CHECKED_RECORDED_FLIGHT_SAMPLES__PLAN_NOT_CONNECTED';
end
R.elapsed_offline_ms=1000*toc(clock);
end
