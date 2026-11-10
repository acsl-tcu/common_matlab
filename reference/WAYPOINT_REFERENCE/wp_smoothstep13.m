function y = wp_smoothstep13(u, derivativeOrder)
%WP_SMOOTHSTEP13  13th-degree C6 smoothstep, including antiderivative.
% derivativeOrder = -1: integral from 0 to u; 0..6: derivatives.
% u is clamped to [0,1]. The derivative branch is for internal use.
arguments
    u (1,1) double {mustBeFinite}
    derivativeOrder (1,1) double {mustBeInteger} = 0
end
if derivativeOrder < -1 || derivativeOrder > 6
    error('WaypointRef:SmoothstepOrder', 'Order must be -1..6.');
end
u = max(0, min(1, u));
if u > 0.5
    if derivativeOrder == -1
        y = u - 0.5 + wp_smoothstep13(1-u, -1);
    elseif derivativeOrder == 0
        y = 1 - wp_smoothstep13(1-u, 0);
    else
        y = (-1)^(derivativeOrder+1)*wp_smoothstep13(1-u, derivativeOrder);
    end
    return
end
coefficients = [1716 -9009 20020 -24024 16380 -6006 924];
y = 0;
for j = 1:7
    degree = j+6;  % 7..13
    a = coefficients(j);
    if derivativeOrder == -1
        y = y + a*u^(degree+1)/(degree+1);
    else
        k = derivativeOrder;
        y = y + a*prod((degree-k+1):degree)*u^(degree-k);
    end
end
end
