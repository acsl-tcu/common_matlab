function R=TRAJECTORY_C9_TAUT_TENSION_TIERS(w_lower_mps2,latest_EKF_mass_kg,tier,model_acceleration_error_mps2,physical_acceleration_error_mps2,mass_error_kg,required_tension_N)
%TRAJECTORY_C9_TAUT_TENSION_TIERS  Tension from the ideal taut-load ODE.
% Under the model m*pL'' = m*g + tension*q, ||q||=1:
%    tension(t)=m*||pL''(t)-g||.
% Input w_lower MUST have been certified for every time in the C9 interval,
% not inferred from point samples. For the MODEL_TUBE tier the all-time
% acceleration error must follow from the selected virtual error model.
% PHYSICAL_ROBUST additionally requires explicitly certified plant and mass
% uncertainty bounds. No empirical minimum speed, force, or mass is used.
%
% NOMINAL: m=latest_EKF_mass, e_a=0.
% MODEL_TUBE: same m; e_a=e_model.
% PHYSICAL_ROBUST: m >= latest_EKF_mass-dm; e_a=e_model+e_plant.
% Reverse triangle inequality proves
%    tension >= m_lower*(w_lower-e_a), if both factors are >0.
% A nonzero required_tension_N is only an EXTERNALLY AUTHORITATIVE force
% requirement; zero is mathematically sufficient for strict tautness.
% Real-arithmetic CONDITIONAL proof only, no MATLAB directed rounding.
R=struct('status','UNKNOWN_TENSION_TIER_INPUT', ...
    'safe_to_commit',false,'tier',tier,'conditional_only',true, ...
    'model_mass_kg',NaN,'mass_lower_kg',NaN, ...
    'acceleration_error_upper_mps2',NaN, ...
    'direction_norm_lower_mps2',NaN,'tension_lower_N',NaN, ...
    'tension_headroom_N',NaN,'strict_tautness_proven_under_assumptions',false, ...
    'required_tension_from_external_authority',true);
if isstring(tier)&&isscalar(tier),tier=char(tier);end
if ~ischar(tier)||~isrow(tier)|| ...
        ~any(strcmp(tier,{'NOMINAL','MODEL_TUBE','PHYSICAL_ROBUST'}))
    return
end
R.tier=tier;
if ~finite_scalar(w_lower_mps2)||w_lower_mps2<=0|| ...
        ~finite_scalar(latest_EKF_mass_kg)||latest_EKF_mass_kg<=0|| ...
        ~finite_scalar(required_tension_N)||required_tension_N<0
    return
end
m=double(latest_EKF_mass_kg);da=0;
if strcmp(tier,'MODEL_TUBE')||strcmp(tier,'PHYSICAL_ROBUST')
    if ~finite_scalar(model_acceleration_error_mps2)|| ...
            model_acceleration_error_mps2<0,return,end
    da=double(model_acceleration_error_mps2);
end
if strcmp(tier,'PHYSICAL_ROBUST')
    if ~finite_scalar(physical_acceleration_error_mps2)|| ...
            physical_acceleration_error_mps2<0|| ...
            ~finite_scalar(mass_error_kg)||mass_error_kg<0
        return
    end
    da=da+double(physical_acceleration_error_mps2);
    m=m-double(mass_error_kg);
end
R.model_mass_kg=double(latest_EKF_mass_kg);
R.mass_lower_kg=m;
R.acceleration_error_upper_mps2=da;
R.direction_norm_lower_mps2=double(w_lower_mps2)-da;
if ~(isfinite(m)&&m>0)
    R.status='UNKNOWN_TAUT_MODEL_NONPOSITIVE_MASS_LOWER';return
end
if ~(isfinite(da)&&isfinite(R.direction_norm_lower_mps2)&& ...
        R.direction_norm_lower_mps2>0)
    R.status='UNKNOWN_TAUT_DIRECTION_NOT_PROVEN';return
end
R.tension_lower_N=m*R.direction_norm_lower_mps2;
R.tension_headroom_N=R.tension_lower_N-double(required_tension_N);
if ~(isfinite(R.tension_lower_N)&&isfinite(R.tension_headroom_N)&& ...
        R.tension_lower_N>0&&R.tension_headroom_N>=0)
    R.status='UNKNOWN_EXTERNAL_TENSION_REQUIREMENT';return
end
R.strict_tautness_proven_under_assumptions=true;
R.status=['CONDITIONAL_' tier '_POSITIVE_TAUT_TENSION'];
end
function yes=finite_scalar(x)
yes=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x);
end
