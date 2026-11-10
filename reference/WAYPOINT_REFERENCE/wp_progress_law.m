function [sd, duration, actualSpeed] = wp_progress_law(t, distance, speed, rampTime, isLoop)
%WP_PROGRESS_LAW  Distance-based progress and time derivatives (0..6).
% Open: smooth acceleration, cruise, smooth deceleration and then hold.
% Loop: smooth initial acceleration followed by perpetual cruise; no stop.
% Both are C6 in time; changing "speed" on each call is NOT supported.
arguments
    t (1,1) double {mustBeFinite}
    distance (1,1) double {mustBePositive}
    speed (1,1) double {mustBePositive}
    rampTime (1,1) double {mustBePositive}
    isLoop (1,1) logical
end
sd = zeros(7,1);
Ta = rampTime;
if isLoop
    v = speed;
    duration = Inf;
    actualSpeed = v;
    if t <= 0
        return
    elseif t < Ta
        u = t/Ta;
        sd(1) = v*Ta*wp_smoothstep13(u,-1);
        for k = 1:6
            sd(k+1) = v/Ta^(k-1)*wp_smoothstep13(u,k-1);
        end
    else
        sd(1) = v*(t-Ta/2);
        sd(2) = v;
    end
    return
end
v = min(speed, distance/Ta);
Tc = max(0, distance/v - Ta);
duration = 2*Ta + Tc;
actualSpeed = v;
if t <= 0
    return
elseif t < Ta
    u = t/Ta;
    sd(1) = v*Ta*wp_smoothstep13(u,-1);
    for k = 1:6
        sd(k+1) = v/Ta^(k-1)*wp_smoothstep13(u,k-1);
    end
elseif t < Ta+Tc
    sd(1) = v*(Ta/2 + t-Ta);
    sd(2) = v;
elseif t < duration
    u = (t-Ta-Tc)/Ta;
    sd(1) = v*(Ta/2+Tc+Ta*(u-wp_smoothstep13(u,-1)));
    sd(2) = v*(1-wp_smoothstep13(u,0));
    for k = 2:6
        sd(k+1) = -v/Ta^(k-1)*wp_smoothstep13(u,k-1);
    end
else
    sd(1) = distance;
end
sd(1) = max(0, min(distance, sd(1)));
end
