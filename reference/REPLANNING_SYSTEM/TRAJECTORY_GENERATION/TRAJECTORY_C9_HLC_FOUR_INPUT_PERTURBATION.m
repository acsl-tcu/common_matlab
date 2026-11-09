function R=TRAJECTORY_C9_HLC_FOUR_INPUT_PERTURBATION(live_lower,live_upper, ...
    nominal_input_enclosure,thrust_variation_bound_N, ...
    inverse_beta_norm_bound,beta_variation_bound, ...
    rhs_variation_bound,nominal_torque_norm_bound)
%TRAJECTORY_C9_HLC_FOUR_INPUT_PERTURBATION  HLC beta2\\rhs robustness LEMMA.
% The ACTUAL HLC_SUSPENDED_LOAD.do computes
%    uf = Uf_SuspendedLoadxyDst(...),
%    beta2 = Beta2_SuspendedLoadxyDst(...),
%    rhs = V2_alpha2_SuspendedLoadxyDst(...),
%    us = beta2 \ rhs;   tmp=[uf(1);us];  [before saturation].
%
% Suppose a NOMINAL all-time raw input enclosure [low,high], and proven
% all-time uniform bounds (on the actual REAL closed-loop reach set):
%   ||beta0^{-1}||_2 <= K, ||beta-beta0||_2 <= db,
%   ||rhs-rhs0||_2 <= dr, ||us0||_2 <= U0,
%   |uf-uf0| <= df.  If K*db<1, Banach's perturbation lemma gives
%     ||us-us0||_2 <= K/(1-K*db)*(dr+db*U0).
% The 2-norm upper bound is conservatively applied to each torque axis.
% This is based on ACTUAL HLC algebra, not nominal inverse-dynamics torque.
% All inputs to this routine must themselves be certified for all time.
% This routine CANNOT establish those bounds; no sampled HLC result can.
% This is a CONDITIONAL mathematical implication and safe_to_commit=false.
R=struct('status','UNKNOWN_HLC_FOUR_INPUT_PRECONDITIONS', ...
    'safe_to_commit',false,'actual_HLC_4_input_certified',false, ...
    'conditional_headroom_all_positive',false, ...
    'conditional_raw_input_enclosure',NaN(4,2), ...
    'minimum_conditional_headroom',NaN, ...
    'torque_variation_bound_Nm',NaN, ...
    'beta_neumann_product',NaN, ...
    'scope','HLC_BETA2_BACKSLASH_SENSITIVITY_LEMMA_NOT_REAL_HLC_PROOF');
if ~isrealvec4(live_lower)||~isrealvec4(live_upper)|| ...
        ~isnumeric(nominal_input_enclosure)|| ...
        ~isreal(nominal_input_enclosure)|| ...
        ~isequal(size(nominal_input_enclosure),[4 2])|| ...
        any(~isfinite(nominal_input_enclosure(:)))|| ...
        ~nonnegative(thrust_variation_bound_N)|| ...
        ~nonnegative(inverse_beta_norm_bound)|| ...
        ~nonnegative(beta_variation_bound)|| ...
        ~nonnegative(rhs_variation_bound)|| ...
        ~nonnegative(nominal_torque_norm_bound)
    return
end
lo=double(live_lower(:));hi=double(live_upper(:));
nom=double(nominal_input_enclosure);
if any(lo>=hi)||lo(1)<0||any(nom(:,1)>nom(:,2))
    R.status='UNKNOWN_HLC_INPUT_LIMITS_OR_NOMINAL_INTERVALS';return
end
kd=double(inverse_beta_norm_bound)*double(beta_variation_bound);
R.beta_neumann_product=kd;
if ~(isfinite(kd)&&kd<1)
    R.status='UNKNOWN_HLC_BETA2_INVERTIBILITY_NOT_PROVEN';return
end
dus=double(inverse_beta_norm_bound)/(1-kd)* ...
    (double(rhs_variation_bound)+ ...
     double(beta_variation_bound)*double(nominal_torque_norm_bound));
if ~isfinite(dus)
    R.status='UNKNOWN_NONFINITE_HLC_TORQUE_PERTURBATION';return
end
v=[double(thrust_variation_bound_N);repmat(dus,3,1)];
raw=[nom(:,1)-v,nom(:,2)+v];
headroom=min([raw(:,1)-lo;hi-raw(:,2)]);
R.conditional_raw_input_enclosure=raw;
R.minimum_conditional_headroom=headroom;
R.torque_variation_bound_Nm=dus;
R.conditional_headroom_all_positive=all(raw(:,1)>=lo&raw(:,2)<=hi);
if R.conditional_headroom_all_positive
    R.status='CONDITIONAL_HLC_PRECLAMP_FOUR_INPUT_WITHIN_BOUNDS__NOT_EXECUTION_PROVEN';
else
    R.status='UNKNOWN_CONDITIONAL_HLC_HEADROOM_NOT_SUFFICIENT';
end
end
function ok=isrealvec4(x)
ok=isnumeric(x)&&isreal(x)&&isequal(size(x),[4 1])&&all(isfinite(x));
end
function ok=nonnegative(x)
ok=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&x>=0;
end
