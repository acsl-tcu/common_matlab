function R=AF7R58R40_RETURN_LOCAL_CAUSE_FROM_LOGGER(logger,runtime_profile)
% Signed-correction reverse crossing from REAL LOGGER samples, diagnostic only.
% No claims of continuous-time obstacle safety or optimality.
% No guessed Creturn, n, or fabricated estimator trajectory.
assert(isa(logger,'LOGGER'),'[R40] Live LOGGER required.');
assert(isstruct(runtime_profile) && isfield(runtime_profile,'phase') && ...
    isfield(runtime_profile,'t'),'[R40] runtime_profile is unavailable.');
[raw,~]=logger.data(1, ...
    'reference.result.state.rtime_correction_xyz','', 'phase','f');
mask=(runtime_profile.phase(:)=='f');
t=double(runtime_profile.t(mask)); t=t(:);
C=squeeze(double(raw));
if ismatrix(C) && size(C,1)==3 && size(C,2)==numel(t)
    C=C.';
end
assert(ismatrix(C) && isequal(size(C),[numel(t),3]), ...
    '[R40] Correction LOGGER samples are not aligned to Flight time.');
assert(all(isfinite([t;C(:)])) && all(diff(t)>0), ...
    '[R40] Samples must be finite with increasing timestamps.');
M=sqrt(sum(C.^2,2));
[peak,k0]=max(M);
assert(peak>0,'[R40] No nonzero correction to analyze.');
u=C(k0,:).'/peak;
s=C*u;
% Find the first positive-to-negative crossing AFTER maximal correction.
crossing=find(s(k0:end-1)>=0 & s(k0+1:end)<0,1,'first');
R=struct('schema','AF7R58R40_RETURN_LOCAL_CAUSE_V1', ...
    'peak_time_s',t(k0),'peak_correction_m',peak, ...
    'signed_axis',u,'negative_signed_witness_found',false, ...
    'crossing_time_interval_s',[NaN NaN], ...
    'secant_signed_velocity_m_s',NaN, ...
    'minimum_signed_m',min(s), ...
    'continuous_time_velocity_certified',false, ...
    'continuous_time_clearance_certified',false, ...
    'overshoot_proved_necessary',false);
if ~isempty(crossing)
    k=k0+crossing-1;
    R.crossing_time_interval_s=[t(k) t(k+1)];
    R.secant_signed_velocity_m_s=(s(k+1)-s(k))/(t(k+1)-t(k));
    R.negative_signed_witness_found=true;
end
% This sampled secant is NOT the actual tangent derivative at the crossing.
% It cannot prove an all-trajectory mathematical necessity for overshoot.
fprintf('[R40 RETURN] peak %.6f m at %.6f s; signed min %.6f m.\n', ...
    R.peak_correction_m,R.peak_time_s,R.minimum_signed_m);
if R.negative_signed_witness_found
    fprintf('[R40 RETURN] first reverse crossing %.6f--%.6f s; secant %.6f m/s.\n', ...
        R.crossing_time_interval_s,R.secant_signed_velocity_m_s);
end
end
