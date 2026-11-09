function R=TRAJECTORY_C9_HLC_NOMINAL_FOUR_INPUT_GATE(live_lower,live_upper,nominal_input_interval_4x2)
%TRAJECTORY_C9_HLC_NOMINAL_FOUR_INPUT_GATE  Nominal pre-clamp HLC input lemma.
% If a separate all-time mathematical certificate encloses the four raw
% inputs of the IDEAL NOMINAL HLC trajectory in I(:,1:2), then checking
%    live_lower <= I(:,1) <= I(:,2) <= live_upper
% proves the pre-clamp input limits in real arithmetic. This helper DOES
% NOT generate those enclosures. Sampled extrema are not certificates.
% No virtual/plant tracking error bounds are demanded in nominal mode.
% Limits must originate from the active HLC source (not fixed here).
R=struct('status','UNKNOWN_NOMINAL_HLC_INPUT_ENCLOSURE', ...
    'conditional_headroom_all_positive',false, ...
    'minimum_conditional_headroom',NaN, ...
    'conditional_raw_input_enclosure',NaN(4,2), ...
    'safe_to_commit',false,'conditional_only',true);
if ~(isnumeric(live_lower)&&isreal(live_lower)&&isequal(size(live_lower),[4,1])&& ...
        isnumeric(live_upper)&&isreal(live_upper)&&isequal(size(live_upper),[4,1])&& ...
        isnumeric(nominal_input_interval_4x2)&&isreal(nominal_input_interval_4x2)&& ...
        isequal(size(nominal_input_interval_4x2),[4,2]))
    return
end
lo=double(live_lower);hi=double(live_upper);
I=double(nominal_input_interval_4x2);
if any(~isfinite([lo;hi;I(:)]))||any(lo>=hi)||any(I(:,1)>I(:,2))
    R.status='UNKNOWN_NOMINAL_HLC_INPUT_INVALID';return
end
R.conditional_raw_input_enclosure=I;
R.minimum_conditional_headroom=min([I(:,1)-lo;hi-I(:,2)]);
R.conditional_headroom_all_positive= ...
    all(I(:,1)>=lo & I(:,2)<=hi);
if R.conditional_headroom_all_positive
    R.status='CONDITIONAL_NOMINAL_HLC_FOUR_PRECLAMP_INPUTS';
else
    R.status='UNKNOWN_NOMINAL_HLC_INPUT_OUTSIDE_LIMITS';
end
end
