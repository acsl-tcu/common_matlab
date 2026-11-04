function R=TRIGGER_ENDPOINT_LIPSCHITZ_GATE(t_s,clearance_lower_m, ...
    uav_speed_upper_mps,payload_speed_upper_mps,observed_ids,assumptions)
% Conditional geometry theorem for the union of UAV sphere, ENTIRE taut
% straight cable capsule, and payload sphere, against static observed solids.
% No model, estimator, sensing, or obstacle information is manufactured here.
%
% At time t, any cable point is (1-s)*pQ(t)+s*pL(t), 0<=s<=1. Hence
% ||c(s,t)-c(s,t')|| <= max(VQ,VL)*|t-t'| if endpoint speeds are bounded.
% The whole-body minimum clearance is therefore Lipschitz with this V.
% A zero clearance anywhere in [ti,ti+1] requires
%     d(ti)+d(ti+1) <= V*(ti+1-ti).
% Thus strictly greater CERTIFIED LOWER CLEARANCES prove no collision
% between samples, without evaluating a separate angular cable cone.
%
% This module never certifies a REAL Flight: the caller must separately
% prove the sample clearance lower bounds, EKF/model errors, endpoint speed
% upper bounds, constant physical radii, taut straight cable and obstacle
% static provenance. The toy-test assumption flags are NOT such a proof.
R=struct('status','UNKNOWN_INPUT_OR_PROVENANCE', ...
    'safe_to_commit',false,'continuous_flight_safety_certified',false, ...
    'conditional_interval_clear',false,'reason','', ...
    'unresolved_obstacle_ids',zeros(1,0), ...
    'first_unresolved_left_s',NaN,'first_unresolved_right_s',NaN, ...
    'cleared',false(0,0),'strict_margin_m',nan(0,0));
required={'true_endpoint_speed_bounds','true_sample_clearance_lower_bounds', ...
    'fixed_body_radii','straight_taut_cable','static_observed_obstacles', ...
    'estimation_and_tracking_errors_included'};
if nargin~=6 || ~isstruct(assumptions) || ~isscalar(assumptions) || ...
        ~all(isfield(assumptions,required)) || ...
        ~all(cellfun(@(s)isequal(assumptions.(s),true),required))
    R.reason='Explicitly guaranteed physical/model provenance is missing.';
    return
end
if ~(isnumeric(t_s)&&isrow(t_s)&&isreal(t_s) && ...
        numel(t_s)>=2&&all(isfinite(t_s))&&all(diff(t_s)>0) && ...
        isnumeric(clearance_lower_m)&&isreal(clearance_lower_m)&& ...
        size(clearance_lower_m,2)==numel(t_s) && ...
        isnumeric(observed_ids)&&isreal(observed_ids)&& ...
        numel(observed_ids)==size(clearance_lower_m,1) && ...
        all(isfinite(observed_ids))&&all(observed_ids==fix(observed_ids)) && ...
        all(observed_ids>0)&&numel(unique(observed_ids))==numel(observed_ids) && ...
        isnumeric(uav_speed_upper_mps)&&isreal(uav_speed_upper_mps)&& ...
        isnumeric(payload_speed_upper_mps)&&isreal(payload_speed_upper_mps))
    R.reason='Malformed time, ID, distance or speed input.';
    return
end
m=numel(t_s)-1;
vq=reshape(double(uav_speed_upper_mps),1,[]);
vl=reshape(double(payload_speed_upper_mps),1,[]);
if ~(numel(vq)==m && numel(vl)==m && all(isfinite(vq)) && ...
        all(isfinite(vl)) && all(vq>=0) && all(vl>=0) && ...
        all(isfinite(clearance_lower_m(:))) && ...
        all(clearance_lower_m(:)>=0))
    R.reason='Some guaranteed clearance or speed bounds are unavailable.';
    return
end
ids=reshape(double(observed_ids),1,[]);
if isempty(ids)
    R.status='NO_OBSERVED_OBSTACLES__UNKNOWN_WORLD';
    R.reason='An empty observed set cannot certify the unknown environment.';
    return
end
h=diff(double(t_s));v=max(vq,vl);
% Strict '>' is intentional. Equality, tangencies, and interval-rounding
% ambiguity stay UNKNOWN. Arithmetic is ordinary MATLAB double and is NOT
% a directed-outward-rounding implementation for a deployable certificate.
margin=double(clearance_lower_m(:,1:end-1))+ ...
       double(clearance_lower_m(:,2:end))-repmat(v.*h,numel(ids),1);
clear=margin>0;
R.cleared=clear;R.strict_margin_m=margin;
R.conditional_interval_clear=all(clear(:));
if R.conditional_interval_clear
    R.status='CONDITIONAL_FULL_INTERVAL_CLEAR__NOT_A_FLIGHT_CERTIFICATE';
    R.reason='The endpoint Lipschitz inequality is sufficient IF all declared bounds are rigorously true.';
else
    [ii,jj]=find(~clear);
    R.unresolved_obstacle_ids=unique(ids(ii),'stable');
    j=min(jj);
    R.first_unresolved_left_s=double(t_s(j));
    R.first_unresolved_right_s=double(t_s(j+1));
    R.status='UNKNOWN_POSSIBLE_INTERSAMPLE_CONTACT';
    R.reason='Failure of a sufficient condition is NOT proof of collision.';
end
end
