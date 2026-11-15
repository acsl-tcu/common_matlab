function tube=wp_six_trigger_error_tube_from_accel(timeOffsets,positionError0,velocityError0,relativeAccelUpper)
%WP_SIX_TRIGGER_ERROR_TUBE_FROM_ACCEL Deterministic position error bound.
%
% Given VALID upper bounds ||p(0)-pbar(0)||<=positionError0,
% ||v(0)-vbar(0)||<=velocityError0 and, for EVERY time in interval k,
% ||a(t)-abar(t)||<=relativeAccelUpper(k), then for all future times:
%   deltaV(t) <= vError0 + integral_0^t deltaA(s) ds,
%   deltaP(t) <= pError0 + t*vError0 +
%                       integral_0^t (t-s)*deltaA(s) ds.
% This routine integrates the piecewise-constant upper bound EXACTLY
% (as real arithmetic); no fitted margin, no empirical parameter.
%
% IMPORTANT: neither true initial errors nor relative acceleration bounds
% can be inferred from EKF covariance or test-flight residual MAX alone.
% A nominal model's absolute acceleration bound is NOT automatically a
% true-vs-predicted acceleration-difference bound.
t=reshape(timeOffsets,1,[]);
a=reshape(relativeAccelUpper,1,[]);
if numel(t)<2 || abs(t(1))>1e-12 || any(diff(t)<=0) || ...
        numel(a)~=numel(t)-1 || any(~isfinite([t,a])) || ...
        any(a<0) || ~isscalar(positionError0) || ~isscalar(velocityError0) || ...
        ~isfinite(positionError0) || ~isfinite(velocityError0) || ...
        positionError0<0 || velocityError0<0
    error('LightTrigger:ErrorTube','Required certified nonnegative difference bounds');
end
position=zeros(1,numel(t));velocity=position;
position(1)=positionError0;velocity(1)=velocityError0;
for k=1:numel(t)-1
    h=t(k+1)-t(k);
    position(k+1)=position(k)+h*velocity(k)+0.5*h*h*a(k);
    velocity(k+1)=velocity(k)+h*a(k);
end
tube=struct('timeOffsets',t,'rhoPosition',position, ...
    'rhoVelocity',velocity,'requiresCertifiedInputBounds',true, ...
    'physicalSafetyCertified',false);
end
