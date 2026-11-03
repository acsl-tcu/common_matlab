function R=TRIGGER_TAUT_DIRECTION_CONE_SUPPORT(C)
% TRIGGER_TAUT_DIRECTION_CONE_SUPPORT
% Conditional all-time support of a UAV ball + full straight taut cable
% + payload ball using d_dot = omega_L x d and ||d|| = 1.
%
% Required upstream guarantees (NOT produced here):
%   true pQ(t) lies in C.uav_lower/upper for the WHOLE interval;
%   true d(t0) lies within C.initial_direction_angle_upper_rad of the
%   nominal UNIT axis C.initial_direction_axis;
%   ||omega_L(t)|| <= C.angular_speed_upper_rad_s for the WHOLE interval;
%   cable remains taut, length in [C.cable_length_lower_m,
%   C.cable_length_upper_m] and body radii are physical bounds.
% These guarantees must originate from a valid observer/controller/plant
% reachability proof. The interval length is NOT a Flight phase duration.
% Inputs are read only. No obstacle truth, prediction, trigger event, HLC
% action, time origin, or unverified estimator bound is manufactured.
%
% Every column of C.normals (3 x N) is a candidate normal. Not required
% to be unit length. The output is a conditional SUPPORT UPPER BOUND.
% Double rounding is not independently certified: NEVER safe_to_commit.
R=struct('status','UNKNOWN_DIRECTION_CONE','reason','', ...
    'conditional_enclosure',false,'safe_to_commit',false, ...
    'max_directional_projection',zeros(1,0), ...
    'uav_support_upper',zeros(1,0), ...
    'cable_support_upper',zeros(1,0), ...
    'payload_support_upper',zeros(1,0), ...
    'union_support_upper',zeros(1,0), ...
    'direction_cap_angle_rad',NaN, ...
    'required_evidence','Proof of initial angle, angular-rate bound, full-time UAV tube and taut mode');
try
    required={'uav_lower','uav_upper','initial_direction_axis', ...
        'initial_direction_angle_upper_rad','angular_speed_upper_rad_s', ...
        'interval_duration_s','cable_length_lower_m', ...
        'cable_length_upper_m','r_uav_m','r_cable_m','r_payload_m', ...
        'normals'};
    assert(isstruct(C)&&isscalar(C)&&all(isfield(C,required)), ...
        'TRIGGER_CONE:Required','All physical tube and cap parameters are required.');
    qlo=double(C.uav_lower(:));qhi=double(C.uav_upper(:));
    d0=double(C.initial_direction_axis(:));
    N=double(C.normals);
    assert(isequal(size(qlo),[3 1]) && isequal(size(qhi),[3 1]) && ...
        isequal(size(d0),[3 1]) && size(N,1)==3 && size(N,2)>=1 && ...
        all(isfinite([qlo;qhi;d0;N(:)])) && all(qlo<=qhi), ...
        'TRIGGER_CONE:Geometry','Finite UAV interval, direction axis and nonempty normals required.');
    d0norm=norm(d0);
    assert(isfinite(d0norm) && d0norm>0 && abs(d0norm-1)<=1e-10, ...
        'TRIGGER_CONE:UnitAxis','A genuine unit cap axis is required.');
    d0=d0/d0norm; % Axis DEFINITION, not a correction to true state.
    a0=double(C.initial_direction_angle_upper_rad);
    omega=double(C.angular_speed_upper_rad_s);
    h=double(C.interval_duration_s);
    Llo=double(C.cable_length_lower_m);
    Lhi=double(C.cable_length_upper_m);
    radii=double([C.r_uav_m C.r_cable_m C.r_payload_m]);
    assert(isscalar(a0)&&isscalar(omega)&&isscalar(h)&& ...
        isfinite(a0)&&isfinite(omega)&&isfinite(h) && ...
        a0>=0&&a0<=pi&&omega>=0&&h>=0 && ...
        all(isfinite([Llo Lhi radii])) && ...
        Llo>0 && Llo<=Lhi && all(radii>=0), ...
        'TRIGGER_CONE:Physical','Externally justified nonnegative angle/rate/time/radii and positive cable length required.');
    nn=sqrt(sum(N.^2,1));
    assert(all(isfinite(nn))&&all(nn>0), ...
        'TRIGGER_CONE:Normal','Every separation normal must be nonzero.');
    % The geodesic angular speed on S^2 is ||d_dot|| <= ||omega_L||.
    theta=min(pi,a0+omega*h);
    cospsi=max(-1,min(1,(d0.'*N)./nn));
    psi=acos(cospsi);
    % EXACT cap support in real arithmetic: max_{d:angle(d,d0)<=theta} n'*d
    % = ||n|| cos(max(0,acos(n'*d0/||n||)-theta)).
    projected=nn.*cos(max(0,psi-theta));
    % Directed rounding has not been machine-certified on this MATLAB.
    % Make the numeric fixture conservatively tolerant, but never claim
    % a formal floating-point inclusion on the basis of this padding.
    projected=projected+64*eps(max(1,max(nn)));
    pos=max(N,0);neg=min(N,0);
    qsupport=sum(pos.*qhi+neg.*qlo,1);
    % A taut cable is the WHOLE segment pQ+s*L*d, 0<=s<=1.
    cable_extension=max(0,Lhi.*projected);
    load_extension=max(Llo.*projected,Lhi.*projected);
    hu=qsupport+radii(1).*nn;
    hc=qsupport+cable_extension+radii(2).*nn;
    hl=qsupport+load_extension+radii(3).*nn;
    R.max_directional_projection=projected;
    R.uav_support_upper=hu;
    R.cable_support_upper=hc;
    R.payload_support_upper=hl;
    R.union_support_upper=max([hu;hc;hl],[],1);
    R.direction_cap_angle_rad=theta;
    R.conditional_enclosure=true;
    R.status='CONDITIONAL_TAUT_DIRECTION_CAP_SUPPORT__NOT_FLIGHT_SAFETY';
    R.reason=['Exact real-arithmetic cone formula and full taut-segment union; ' ...
      'requires external angular-rate/EKF/tube/tension proof and independent directed-rounding certification.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
