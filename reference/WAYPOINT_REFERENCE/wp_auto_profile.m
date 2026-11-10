function [Ta, v] = wp_auto_profile(D,vRequested,isLoop,amax,jmax,d6max,rampTime)
%WP_AUTO_PROFILE Determine C6 start/stop ramp duration from specified bounds.
% Limits apply to longitudinal PROGRESS s(t), not to the complete
% spatial acceleration/jerk on turns. Full physical limits and HLC actuator
% feasibility are not guaranteed. A manual positive rampTime is optional.
% AUTO: rise in ds/dt is v*S13(t/Ta) where S13 is C6 smoothstep.
% d2s/dt2 bound -> v*max|S13'|/Ta <= amax
% d3s/dt3 bound -> v*max|S13''|/Ta^2 <= jmax
% d6s/dt6 bound -> v*max|S13^(5)|/Ta^5 <= d6max
arguments
    D (1,1) double {mustBePositive}
    vRequested (1,1) double {mustBePositive}
    isLoop (1,1) logical
    amax (1,1) double {mustBePositive}
    jmax (1,1) double {mustBePositive}
    d6max (1,1) double {mustBePositive}
    rampTime (1,1) double = NaN
end
if ~isnan(rampTime) && (~isfinite(rampTime) || rampTime <= 0)
    error('WaypointRef:RampTime','Set rampTime to NaN (auto) or positive seconds.');
end
if isnan(rampTime)
    % Conservative maxima of the derivatives of the degree-13 smoothstep.
    C1=2.933; C2=13.177; C5=16892;
    computeTa = @(v) max([0.25, C1*v/amax, sqrt(C2*v/jmax), (C5*v/d6max)^(1/5)]);
    Ta=computeTa(vRequested);
    v=vRequested;
    if ~isLoop && v*Ta > D
        lo=0;hi=vRequested;
        for iter=1:60
            mid=(lo+hi)/2;
            if mid*computeTa(mid)>D
                hi=mid;
            else
                lo=mid;
            end
        end
        v=max(lo,eps);
        Ta=computeTa(v);
    end
else
    Ta=rampTime;
    v=vRequested;
    if ~isLoop
        v=min(v,D/Ta);
    end
end
end
