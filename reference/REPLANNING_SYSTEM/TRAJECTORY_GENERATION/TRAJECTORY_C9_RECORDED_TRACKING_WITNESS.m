function R = TRAJECTORY_C9_RECORDED_TRACKING_WITNESS(logger_or_data)
%TRAJECTORY_C9_RECORDED_TRACKING_WITNESS  Read-only offline HLC tracking.
% Uses the state ACTUALLY consumed by HLC, controller.result.x:
%   [quaternion(4);w(3);pL(3);vL(3);pT(3);wL(3)] (19 entries).
% pL=x(8:10), vL=x(11:13), decoded by a strict layout helper.
% Compares pL, vL against the ACTUAL 28-reference sent to HLC at that tick.
% Reference: xd(1:3) = pL ref; xd(5:7) = vL ref. This is not a UAV
% reference, not a C9 candidate verification, and not a continuous tube.
% A sampled maximum is a NECESSARY LOWER BOUND for a tube that would
% enclose the recorded samples. It cannot prove an all-time upper bound.
if isobject(logger_or_data)
    assert(isprop(logger_or_data,'Data'), ...
        'C9_TRACKING_WITNESS:Logger','Requires a Sim LOGGER Data property.');
    D=logger_or_data.Data;
elseif isstruct(logger_or_data)&&isscalar(logger_or_data)&& ...
        isfield(logger_or_data,'Data')
    D=logger_or_data.Data;
else
    D=logger_or_data;
end
assert(isstruct(D)&&isscalar(D)&&all(isfield(D,{'phase','t','agent'}))&& ...
    numel(D.agent)>=1, 'C9_TRACKING_WITNESS:Data', ...
    'Original Sim LOGGER is required (not a saved R summary).');
C=D.agent(1).controller.result;
F=D.agent(1).reference.result;
assert(iscell(C)&&iscell(F)&&numel(F)>=numel(C), ...
    'C9_TRACKING_WITNESS:Cell','Missing controller/reference result cells.');
N=numel(C);
P=D.phase(:);
if ischar(P)
    flight=(P(1:N)=='f');
elseif isnumeric(P)
    flight=(P(1:N)==double('f'));
elseif isstring(P)||iscellstr(P)
    flight=strcmp(string(P(1:N)),"f");
else
    error('C9_TRACKING_WITNESS:Phase','Unsupported Sim phase representation.');
end
ids=find(flight);
t=reshape(double(D.t(ids)),1,[]);
M=numel(ids);
R=struct('status','UNKNOWN_NOT_EVALUATED', ...
    'sample_count',M,'valid_sample_count',0,'invalid_sample_count',M, ...
    'time_s',t,'logger_index',reshape(ids,1,[]), ...
    'payload_position_error_norm_m',NaN(1,M), ...
    'payload_velocity_error_norm_mps',NaN(1,M), ...
    'max_observed_payload_position_error_m',NaN, ...
    'rms_observed_payload_position_error_m',NaN, ...
    'max_observed_payload_velocity_error_mps',NaN, ...
    'rms_observed_payload_velocity_error_mps',NaN, ...
    'sampled_payload_error_tube_necessary_radius_m',NaN, ...
    'last_reference_before_flight',NaN(28,1), ...
    'first_reference_in_flight',NaN(28,1), ...
    'adjacent_phase_reference_jump_observed',false, ...
    'adjacent_phase_yaw_difference_rad',NaN, ...
    'reference_samples_are_generated_C9',false, ...
    'continuous_time_tracking_certified',false, ...
    'safe_to_commit',false, ...
    'scope','HLC_CONSUMED_STATE_VS_ACTUAL_SENT_REFERENCE__SAMPLED_ONLY');
for j=1:M
    k=ids(j);
    try
        h=C{k};f=F{k};
        if ~(isstruct(h)&&isfield(h,'x')&& ...
                isnumeric(h.x)&&isreal(h.x)&&numel(h.x)==19&& ...
                all(isfinite(h.x(:)))&&isstruct(f)&&isfield(f,'state'))
            continue
        end
        if isstruct(f.state)&&isfield(f.state,'xd')
            xd=f.state.xd;
        elseif isobject(f.state)&&isprop(f.state,'xd')
            xd=f.state.xd;
        elseif isobject(f.state)&&ismethod(f.state,'get')
            xd=f.state.get();
        else
            continue
        end
        if ~(isnumeric(xd)&&isreal(xd)&&numel(xd)>=28&& ...
                all(isfinite(xd(:))))
            continue
        end
        [pL,vL,layout_ok]=TRAJECTORY_C9_HLC_PAYLOAD_STATE(h.x);
        if ~layout_ok,continue;end
        xd=double(xd(:));
        R.payload_position_error_norm_m(j)=norm(pL-xd(1:3));
        R.payload_velocity_error_norm_mps(j)=norm(vL-xd(5:7));
    catch
        % Invalid logger data is explicitly UNKNOWN; never silently pass.
    end
end
valid=isfinite(R.payload_position_error_norm_m)& ...
    isfinite(R.payload_velocity_error_norm_mps);
R.valid_sample_count=sum(valid);
R.invalid_sample_count=M-sum(valid);
if any(valid)
    pe=R.payload_position_error_norm_m(valid);
    ve=R.payload_velocity_error_norm_mps(valid);
    R.max_observed_payload_position_error_m=max(pe);
    R.rms_observed_payload_position_error_m=sqrt(mean(pe.^2));
    R.max_observed_payload_velocity_error_mps=max(ve);
    R.rms_observed_payload_velocity_error_mps=sqrt(mean(ve.^2));
    R.sampled_payload_error_tube_necessary_radius_m=max(pe);
end
% Report adjacent phase reference samples without claiming continuous
% junction discontinuity: the samples are at DIFFERENT instants.
if M>0 && ids(1)>1
    k=ids(1);q=last_ref(F{k-1});w=last_ref(F{k});
    if ~isempty(q)&&~isempty(w)
        R.last_reference_before_flight=q;
        R.first_reference_in_flight=w;
        d=atan2(sin(w(4)-q(4)),cos(w(4)-q(4)));
        R.adjacent_phase_yaw_difference_rad=d;
        R.adjacent_phase_reference_jump_observed=any(q~=w);
    end
end
if M==0
    R.status='UNKNOWN_NO_RECORDED_FLIGHT_TICKS';
elseif R.invalid_sample_count>0
    R.status='UNKNOWN_INCOMPLETE_HLC_CONSUMED_STATE_TRACE';
else
    R.status='CHECKED_SAMPLED_HLC_PAYLOAD_TRACKING__NO_TUBE_PROOF';
end
end
function y=last_ref(f)
y=[];
if ~(isstruct(f)&&isfield(f,'state')),return;end
try
    s=f.state;
    if isstruct(s)&&isfield(s,'xd')
        x=s.xd;
    elseif isobject(s)&&isprop(s,'xd')
        x=s.xd;
    elseif isobject(s)&&ismethod(s,'get')
        x=s.get();
    else
        return
    end
    if isnumeric(x)&&isreal(x)&&numel(x)>=28&&all(isfinite(x(:)))
        y=double(x(1:28));y=y(:);
    end
catch
end
end
