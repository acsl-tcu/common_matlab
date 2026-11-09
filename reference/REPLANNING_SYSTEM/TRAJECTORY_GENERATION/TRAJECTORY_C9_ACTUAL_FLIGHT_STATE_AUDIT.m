function R = TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT(logger_or_data)
%TRAJECTORY_C9_ACTUAL_FLIGHT_STATE_AUDIT  Read actual Sim LOGGER in memory.
% Read-only, post-flight diagnostic. Does NOT load MAT files, modify the
% runner, run any replanner, or declare a continuous-time tracking proof.
%
% HLC_SUSPENDED_LOAD.do constructs:
%   x=[quaternion(4); w(3); pL(3); vL(3); pT(3); wL(3)].
% The correct positions are x(8:10), velocities x(11:13). Layout is
% decoded by TRAJECTORY_C9_HLC_PAYLOAD_STATE and rejects non-19 vectors.
% The actual reference supplied to HLC has xd(1:3)=pLd, xd(5:7)=vLd.
%
% Logger.logging is AFTER plant.do in the Flight loop. As a separate
% diagnostic, the preceding LOGGER record's post-plant state is used as
% the nominal pre-plant truth alignment for the current controller tick.
% This is an alignment assumption to investigate, NOT a certified tube.
%
% Do not interpret this nominal-reference Flight as C9-generated Flight.
if isobject(logger_or_data)
    assert(isprop(logger_or_data,'Data'), ...
        'C9_ACTUAL_STATE:Logger','A Sim LOGGER with Data is required.');
    D=logger_or_data.Data;
elseif isstruct(logger_or_data) && isscalar(logger_or_data) && ...
        isfield(logger_or_data,'Data')
    D=logger_or_data.Data;
else
    D=logger_or_data;
end
assert(isstruct(D) && isscalar(D) && ...
    all(isfield(D,{'t','phase','agent'})) && ~isempty(D.agent) && ...
    isfield(D.agent(1),'controller') && ...
    isfield(D.agent(1),'reference') && ...
    isfield(D.agent(1),'estimator'), ...
    'C9_ACTUAL_STATE:Data','Missing original LOGGER.Data members.');
C=D.agent(1).controller.result;
F=D.agent(1).reference.result;
E=D.agent(1).estimator.result;
assert(iscell(C)&&iscell(F)&&iscell(E), ...
    'C9_ACTUAL_STATE:History','Recorded histories must be cell arrays.');
N=min([numel(C),numel(F),numel(E),numel(D.t),numel(D.phase)]);
phase=D.phase(:);
if isnumeric(phase)
    isflight=(phase(1:N)==double('f'));
elseif ischar(phase)
    isflight=(phase(1:N)=='f');
elseif isstring(phase)||iscellstr(phase)
    isflight=strcmp(string(phase(1:N)),"f");
else
    error('C9_ACTUAL_STATE:Phase','Unsupported Flight phase encoding.');
end
ids=find(isflight); M=numel(ids);
if isfield(D.agent(1),'plant') && isfield(D.agent(1).plant,'result') && ...
        iscell(D.agent(1).plant.result)
    P=D.agent(1).plant.result;
else
    P={};
end
R=struct();
R.status='UNKNOWN_NO_FLIGHT_SAMPLES';
R.time_s=reshape(double(D.t(ids)),1,[]);
R.logger_index=reshape(ids,1,[]);
R.sample_count=M;
R.valid_controller_reference_count=0;
R.invalid_controller_reference_count=M;
R.valid_EKF_count=0;
R.preceding_postplant_count=0;
R.actual_reference_payload_position_m=NaN(3,M);
R.actual_reference_payload_velocity_mps=NaN(3,M);
R.HLC_consumed_payload_position_m=NaN(3,M);
R.HLC_consumed_payload_velocity_mps=NaN(3,M);
R.signed_payload_position_error_m=NaN(3,M);
R.signed_payload_velocity_error_mps=NaN(3,M);
R.estimated_payload_position_m=NaN(3,M);
R.estimated_payload_velocity_mps=NaN(3,M);
R.preceding_postplant_payload_position_m=NaN(3,M);
R.preceding_postplant_payload_velocity_mps=NaN(3,M);
R.HLC_minus_EKF_payload_position_m=NaN(3,M);
R.HLC_minus_EKF_payload_velocity_mps=NaN(3,M);
R.EKF_minus_preceding_postplant_payload_position_m=NaN(3,M);
R.EKF_minus_preceding_postplant_payload_velocity_mps=NaN(3,M);
R.estimated_payload_mass_kg=NaN(1,M);
R.position_error_norm_m=NaN(1,M);
R.velocity_error_norm_mps=NaN(1,M);
R.max_position_error_m=NaN;
R.max_velocity_error_mps=NaN;
R.time_of_max_position_error_s=NaN;
R.time_of_max_velocity_error_s=NaN;
R.max_abs_signed_position_error_xyz_m=NaN(3,1);
R.max_abs_signed_velocity_error_xyz_mps=NaN(3,1);
R.max_HLC_EKF_position_disagreement_m=NaN;
R.max_EKF_preceding_postplant_position_disagreement_m=NaN;
R.actual_reference_start_end_displacement_m=NaN;
R.position_velocity_trapezoid_residual_m=NaN(3,max(M-1,0));
R.max_position_velocity_trapezoid_residual_m=NaN;
R.HLC_state_layout='QUAT4_W3_PL3_VL3_PT3_WL3__19';
R.generated_C9_reference_sent=false;
R.continuous_time_tracking_proven=false;
R.HLC_consumed_mass_each_tick_proven=false;
R.safe_to_commit=false;
R.plant_time_alignment='PREVIOUS_LOG_ENTRY_POST_PLANT__ASSUMED_PRE_CURRENT_TICK';
R.scope='CORRECT_19_STATE_LAYOUT__NOMINAL_SIM_SAMPLES_NOT_TUBE_PROOF';
if M==0,return;end
assert(all(isfinite(R.time_s)) && all(diff(R.time_s)>0), ...
    'C9_ACTUAL_STATE:Time','Flight timestamps must strictly increase.');
for j=1:M
    k=ids(j);
    try
        h=C{k};r=F{k};
        if ~isstruct(h) || ~isfield(h,'x') || ~isstruct(r) || ...
                ~isfield(r,'state')
            continue
        end
        [h_pL,h_vL,layout_ok]=TRAJECTORY_C9_HLC_PAYLOAD_STATE(h.x);
        xd=read_reference_28(r.state);
        if ~layout_ok || isempty(xd)
            continue
        end
        R.HLC_consumed_payload_position_m(:,j)=h_pL;
        R.HLC_consumed_payload_velocity_mps(:,j)=h_vL;
        R.actual_reference_payload_position_m(:,j)=xd(1:3);
        R.actual_reference_payload_velocity_mps(:,j)=xd(5:7);
        R.signed_payload_position_error_m(:,j)=h_pL-xd(1:3);
        R.signed_payload_velocity_error_mps(:,j)=h_vL-xd(5:7);
        R.position_error_norm_m(j)=norm(R.signed_payload_position_error_m(:,j));
        R.velocity_error_norm_mps(j)=norm(R.signed_payload_velocity_error_mps(:,j));
    catch
        % Invalid samples remain NaN and are counted, never silently PASS.
    end
    try
        e=E{k}.state;
        ep=read_state_three(e,'pL');
        ev=read_state_three(e,'vL');
        if ~isempty(ep) && ~isempty(ev)
            R.estimated_payload_position_m(:,j)=ep;
            R.estimated_payload_velocity_mps(:,j)=ev;
            if isfinite(R.position_error_norm_m(j))
                R.HLC_minus_EKF_payload_position_m(:,j)= ...
                    R.HLC_consumed_payload_position_m(:,j)-ep;
                R.HLC_minus_EKF_payload_velocity_mps(:,j)= ...
                    R.HLC_consumed_payload_velocity_mps(:,j)-ev;
            end
        end
        mass=read_state_scalar(e,'mL');
        if ~isempty(mass) && mass>0
            R.estimated_payload_mass_kg(j)=mass;
        end
    catch
    end
    % At index k the plant has ALREADY integrated. The previous recorded
    % post-plant state is the only plausible pre-current-tick comparison.
    if k>1 && k-1<=numel(P)
        try
            pp=P{k-1};
            if isstruct(pp) && isfield(pp,'state')
                p=read_state_three(pp.state,'pL');
                v=read_state_three(pp.state,'vL');
                if ~isempty(p) && ~isempty(v)
                    R.preceding_postplant_payload_position_m(:,j)=p;
                    R.preceding_postplant_payload_velocity_mps(:,j)=v;
                    if all(isfinite(R.estimated_payload_position_m(:,j)))
                        R.EKF_minus_preceding_postplant_payload_position_m(:,j)= ...
                            R.estimated_payload_position_m(:,j)-p;
                        R.EKF_minus_preceding_postplant_payload_velocity_mps(:,j)= ...
                            R.estimated_payload_velocity_mps(:,j)-v;
                    end
                end
            end
        catch
        end
    end
end
ok=isfinite(R.position_error_norm_m)&isfinite(R.velocity_error_norm_mps);
R.valid_controller_reference_count=sum(ok);
R.invalid_controller_reference_count=M-sum(ok);
R.valid_EKF_count=sum(all(isfinite(R.estimated_payload_position_m),1)& ...
    all(isfinite(R.estimated_payload_velocity_mps),1));
R.preceding_postplant_count=sum(all(isfinite(R.preceding_postplant_payload_position_m),1)& ...
    all(isfinite(R.preceding_postplant_payload_velocity_mps),1));
if any(ok)
    idx=find(ok);
    [R.max_position_error_m,ii]=max(R.position_error_norm_m(idx));
    [R.max_velocity_error_mps,jj]=max(R.velocity_error_norm_mps(idx));
    R.time_of_max_position_error_s=R.time_s(idx(ii));
    R.time_of_max_velocity_error_s=R.time_s(idx(jj));
    R.max_abs_signed_position_error_xyz_m= ...
        max(abs(R.signed_payload_position_error_m),[],2,'omitnan');
    R.max_abs_signed_velocity_error_xyz_mps= ...
        max(abs(R.signed_payload_velocity_error_mps),[],2,'omitnan');
    R.actual_reference_start_end_displacement_m=norm( ...
        R.actual_reference_payload_position_m(:,idx(end))- ...
        R.actual_reference_payload_position_m(:,idx(1)));
end
% Consistency diagnostic of the sampled velocity-error integral. It is
% neither a continuous-time bound nor a proof of controller tracking.
for j=1:M-1
    if ok(j)&&ok(j+1)
        dt=R.time_s(j+1)-R.time_s(j);
        R.position_velocity_trapezoid_residual_m(:,j)= ...
            R.signed_payload_position_error_m(:,j+1)- ...
            R.signed_payload_position_error_m(:,j)- ...
            (dt/2)*(R.signed_payload_velocity_error_mps(:,j+1)+ ...
                    R.signed_payload_velocity_error_mps(:,j));
    end
end
res=sqrt(sum(R.position_velocity_trapezoid_residual_m.^2,1));
res(~all(isfinite(R.position_velocity_trapezoid_residual_m),1))=NaN;
if any(isfinite(res))
    R.max_position_velocity_trapezoid_residual_m=max(res,[],'omitnan');
end
m=sqrt(sum(R.HLC_minus_EKF_payload_position_m.^2,1));
m(~all(isfinite(R.HLC_minus_EKF_payload_position_m),1))=NaN;
if any(isfinite(m)),R.max_HLC_EKF_position_disagreement_m=max(m,[],'omitnan');end
p=sqrt(sum(R.EKF_minus_preceding_postplant_payload_position_m.^2,1));
p(~all(isfinite(R.EKF_minus_preceding_postplant_payload_position_m),1))=NaN;
if any(isfinite(p))
    R.max_EKF_preceding_postplant_position_disagreement_m=max(p,[],'omitnan');
end
if R.invalid_controller_reference_count>0
    R.status='UNKNOWN_INCOMPLETE_ACTUAL_FLIGHT_REFERENCE_OR_HLC_STATE';
elseif R.valid_EKF_count<M
    R.status='CHECKED_HLC_REFERENCE_ONLY__EKF_HISTORY_INCOMPLETE';
elseif R.preceding_postplant_count<M
    R.status='CHECKED_HLC_AND_EKF_SAMPLES__PREVIOUS_PLANT_INCOMPLETE';
else
    R.status='CHECKED_NOMINAL_FLIGHT_HLC_EKF_AND_PREVIOUS_PLANT__NO_TUBE_PROOF';
end
end

function y=read_reference_28(s)
y=[];
try
    if isstruct(s)&&isfield(s,'xd')
        v=s.xd;
    elseif isobject(s)&&isprop(s,'xd')
        v=s.xd;
    elseif isobject(s)&&ismethod(s,'get')
        v=s.get();
    else
        return
    end
    if isnumeric(v)&&isreal(v)&&numel(v)>=28&&all(isfinite(v(:)))
        y=double(v(1:28));y=y(:);
    end
catch
end
end
function y=read_state_three(s,name)
y=[];
try
    if isstruct(s)&&isfield(s,name)
        v=s.(name);
    elseif isobject(s)&&isprop(s,name)
        v=s.(name);
    else
        return
    end
    if isnumeric(v)&&isreal(v)&&numel(v)==3&&all(isfinite(v(:)))
        y=double(v(:));
    end
catch
end
end
function y=read_state_scalar(s,name)
y=[];
try
    if isstruct(s)&&isfield(s,name)
        v=s.(name);
    elseif isobject(s)&&isprop(s,name)
        v=s.(name);
    else
        return
    end
    if isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)
        y=double(v);
    end
catch
end
end
