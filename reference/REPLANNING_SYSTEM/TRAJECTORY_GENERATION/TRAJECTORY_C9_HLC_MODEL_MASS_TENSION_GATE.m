function R=TRAJECTORY_C9_HLC_MODEL_MASS_TENSION_GATE(model_mass_kg, ...
    certified_nominal_w_lower_mps2,model_acceleration_error_mps2, ...
    minimum_tension_N)
%TRAJECTORY_C9_HLC_MODEL_MASS_TENSION_GATE  Ideal-model tension lemma.
% Explicit RESEARCH CONVENTION only: at this Flight tick the freshest
% HLC/EKF loadmass is by definition the cable-load mass of the virtual
% simulation MODEL. No claim about physical true mass is made or needed.
% As model input data, mL is held constant for THIS certified horizon.
% The next tick must refresh the model and, if mL changes, revalidate.
%
% On the taut straight-cable point-load model
%  T_cable(t)=mL*norm(gravity-pL_model''(t)).
% If mL>0, norm(gravity-pL_nom''(t)) >= w_nom, and
% norm(pL_model''-pL_nom'') <= da throughout the time interval, then
% T_cable(t) >= mL*(w_nom-da) by the reverse triangle inequality.
% The two all-time bounds must be PROVEN for the defined model: arbitrary
% sampled values are not mathematical interval certificates.
% This helper does NOT prove model tracking equivalence, numeric rounding
% or real-plant tension. It never authorizes COMMIT.
R=struct('status','UNKNOWN_IDEAL_MODEL_TENSION_PRECONDITIONS', ...
    'safe_to_commit',false,'model_scope','IDEAL_VIRTUAL_SIMULATION', ...
    'model_mass_kg',NaN,'conditional_tension_lower_N',NaN, ...
    'conditional_tension_headroom_N',NaN, ...
    'actual_plant_tension_proven',false);
v=[model_mass_kg,certified_nominal_w_lower_mps2, ...
    model_acceleration_error_mps2,minimum_tension_N];
if ~(isnumeric(v)&&isreal(v)&&isequal(size(v),[1 4])&& ...
        all(isfinite(v))&&v(1)>0&&v(2)>0&&v(3)>=0&&v(4)>=0)
    return
end
R.model_mass_kg=double(v(1));
wl=double(v(2))-double(v(3));
if ~(isfinite(wl)&&wl>0)
    R.status='UNKNOWN_IDEAL_MODEL_TAUT_DIRECTION_DEGENERATE';return
end
low=double(v(1))*wl;
R.conditional_tension_lower_N=low;
R.conditional_tension_headroom_N=low-double(v(4));
if ~(isfinite(low)&&low>double(v(4)))
    R.status='UNKNOWN_IDEAL_MODEL_TENSION_HEADROOM_NOT_STRICT';return
end
R.status='CONDITIONAL_IDEAL_MODEL_POSITIVE_TAUT_TENSION';
end
