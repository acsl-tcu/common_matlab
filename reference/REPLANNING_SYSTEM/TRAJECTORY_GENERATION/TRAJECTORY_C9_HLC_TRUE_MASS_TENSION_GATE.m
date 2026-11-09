function R=TRAJECTORY_C9_HLC_TRUE_MASS_TENSION_GATE(estimated_mass_kg, ...
    certified_estimation_error_kg,nominal_w_lower_mps2, ...
    acceleration_error_mps2,required_tension_N)
%TRAJECTORY_C9_HLC_TRUE_MASS_TENSION_GATE Conditional true-mass tension.
% Exact physical identity of the positive-taut ideal point-load model:
%   tension(t)=mL_true(t)*||gravity-pL_actual''(t)||.
% If the caller CERTIFIES (not estimates from a covariance or a flight log)
%   |mL_true(t)-mL_EKF(now)| <= dm for EVERY t in the planned horizon,
%   ||g-pL_nom''(t)|| >= wmin for EVERY t,
%   ||pL_actual''(t)-pL_nom''(t)|| <= da for EVERY t,
% then tension >= (mL_EKF-dm)*(wmin-da), provided both factors >0.
% All source assertions remain EXTERNAL proof obligations; this function
% cannot validate their provenance and never certifies the real flight.
R=struct('status','UNKNOWN_TRUE_MASS_TENSION_PRECONDITIONS', ...
    'safe_to_commit',false,'real_tension_proven',false, ...
    'conditional_true_mass_lower_kg',NaN, ...
    'conditional_tension_lower_N',NaN, ...
    'conditional_tension_headroom_N',NaN, ...
    'scope','TRUE_MASS_OVER_HORIZON_AND_ACCELERATION_BOUNDS_REQUIRED');
v=[estimated_mass_kg,certified_estimation_error_kg, ...
    nominal_w_lower_mps2,acceleration_error_mps2,required_tension_N];
if ~(isnumeric(v)&&isreal(v)&&isequal(size(v),[1 5])&& ...
        all(isfinite(v))&&v(1)>0&&v(2)>=0&& ...
        v(3)>0&&v(4)>=0&&v(5)>=0)
    return
end
mmin=double(estimated_mass_kg)-double(certified_estimation_error_kg);
wmin=double(nominal_w_lower_mps2)-double(acceleration_error_mps2);
R.conditional_true_mass_lower_kg=mmin;
if ~(isfinite(mmin)&&mmin>0)
    R.status='UNKNOWN_TRUE_MASS_LOWER_NOT_STRICTLY_POSITIVE';return
end
if ~(isfinite(wmin)&&wmin>0)
    R.status='UNKNOWN_ACTUAL_CABLE_FORCE_DIRECTION_DEGENERATE';return
end
minT=mmin*wmin;
R.conditional_tension_lower_N=minT;
R.conditional_tension_headroom_N=minT-double(required_tension_N);
if ~(isfinite(minT)&&minT>double(required_tension_N))
    R.status='UNKNOWN_CONDITIONAL_TENSION_HEADROOM_NOT_STRICT';return
end
R.status='CONDITIONAL_TRUE_MASS_POSITIVE_TENSION__HORIZON_ERROR_PROOF_REQUIRED';
end
