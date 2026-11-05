function R=TRIGGER_EXACT_BINARY64_TIME_SPAN(t_left_s,t_right_s)
% EXACT endpoints and conservative duration for interval-ODE propagation.
% The input binary64 timestamps denote exact real numbers. TwoSum applied
% to (right + (-left)) recovers an exact error term for the subtraction,
% under IEEE-754 round-to-nearest and gradual subnormal arithmetic.
% The duration of the physical time interval is nominal + roundoff_error.
% Every Picard tube MUST use an UPPER bound on duration; each propagated
% ENDPOINT MUST use the WHOLE [lower,upper] elapsed-time interval.
% Using just the upper time as an exact endpoint can exclude the true end
% state when the derivative has a known sign. Missing IEEE conditions =>
% UNKNOWN; this function is not a hardware-wide rounding certification.
R=struct('status','UNKNOWN_BINARY64_TIME_SPAN', ...
 'valid',false,'nominal_s',NaN,'roundoff_error_s',NaN, ...
 'lower_s',NaN,'upper_s',NaN,'exact_ge_4s',false, ...
 'rounding_environment_independently_verified',false,'reason','');
try
 assert(isa(t_left_s,'double')&&isa(t_right_s,'double') && ...
    isscalar(t_left_s)&&isscalar(t_right_s)&& ...
    isreal(t_left_s)&&isreal(t_right_s)&& ...
    all(isfinite([t_left_s t_right_s])) && t_right_s>t_left_s, ...
    'TRIGGER_EXACT_BINARY64_TIME_SPAN:Time', ...
    'Finite ordered binary64 time endpoints are required.');
 a=t_right_s; b=-t_left_s;
 % Knuth error-free TwoSum: a+b = s+e exactly, barring overflow or FTZ.
 s=a+b;
 z=s-a;
 e=(a-(s-z))+(b-z);
 assert(all(isfinite([s e]))&&s>0 && ...
    abs(e)<=eps(s), ...
    'TRIGGER_EXACT_BINARY64_TIME_SPAN:IEEE', ...
    'Time difference does not admit the assumed finite IEEE decomposition.');
 % One outward ULP is enough to enclose s+e, and safely handles ties.
 lo=TRIGGER_FLOAT_OUTWARD_ONE_ULP(s,-1);
 hi=TRIGGER_FLOAT_OUTWARD_ONE_ULP(s,+1);
 assert(isfinite(lo)&&isfinite(hi)&&lo>=0&&lo<=hi, ...
    'TRIGGER_EXACT_BINARY64_TIME_SPAN:Overflow', ...
    'No finite conservative elapsed-time interval exists.');
 R.nominal_s=s; R.roundoff_error_s=e;
 R.lower_s=lo;R.upper_s=hi;
 % For an EXACTLY four-second target, the sign of e distinguishes
 % [4 - tiny, 4] from [4, 4 + tiny]. A 1-ULP widening alone would
 % reject the true equality case; naive >= on rounded s can accept less.
 R.exact_ge_4s=(s>4) || (s==4 && e>=0);
 R.valid=true;
 R.status='CONDITIONAL_EXACT_BINARY64_TIME_ENCLOSURE';
 R.reason=['Exact binary64 timestamp difference is enclosed by the ' ...
    'reported directed bounds assuming IEEE nearest with gradual ' ...
    'subnormals; no true plant/clock/rounding provenance established.'];
catch ME
 R.reason=[ME.identifier ' | ' ME.message];
end
end
