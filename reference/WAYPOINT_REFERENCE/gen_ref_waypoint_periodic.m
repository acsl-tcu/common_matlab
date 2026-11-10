function ref = gen_ref_waypoint_periodic(opts)
%GEN_REF_WAYPOINT_PERIODIC  Periodic Fourier interpolation of sampled points.
% pointsXYZ: N x 3 positions at equally spaced PHASES over one revolution.
% Do NOT repeat first point at the end. The interpolant passes through every
% point, is infinitely differentiable and joins smoothly after each lap.
% The periodic formula, not a polygonal path, supplies up to 6 derivatives.
% "speed" is progress rate in nominal metres/sec, not exact Cartesian
% speed unless the phase sampling follows arc length.
arguments
    opts.pointsXYZ (:,3) double
    opts.speed (1,1) double {mustBePositive} = 0.2
    opts.rampTime (1,1) double = NaN % AUTO unless positive
    opts.maxAccel (1,1) double {mustBePositive} = 0.30
    opts.maxJerk (1,1) double {mustBePositive} = 0.50
    opts.maxD6 (1,1) double {mustBePositive} = 1.00
    opts.yaw (1,1) double {mustBeFinite} = 0
end
P = opts.pointsXYZ;
N = size(P,1);
if N<15 || any(~isfinite(P(:)))
    error('WaypointRef:PeriodicPoints','Need >=15 finite XYZ samples in one lap.');
end
W = P.';
edge = diff([W,W(:,1)],1,2);
D = sum(sqrt(sum(edge.^2,1)));
if D<=1e-7
    error('WaypointRef:PeriodicLength','Closed route has zero path length.');
end
% D approximates a lap's arc length, using the sampled points.
% Fourier interpolation is exact at the sample phases (up to roundoff).
C = fft(W,[],2)/N;
if mod(N,2)==0
    freq = [0:N/2, -N/2+1:-1];
else
    freq = [0:(N-1)/2, -(N-1)/2:-1];
end
freq = freq(:);
[Ta,v] = wp_auto_profile(D,opts.speed,true, ...
    opts.maxAccel,opts.maxJerk,opts.maxD6,opts.rampTime);
ref.evaluate = @evaluate_time;
ref.evaluatePath = @evaluate_path;
ref.timeLaw = @time_law;
ref.waypoints = W;
ref.pointsXYZ = P;
ref.sampleS = D*(0:N-1)/N;
ref.pathLength = D;
ref.periodS = D;
ref.duration = Inf;
ref.speedPeak = v;
ref.rampTimeEffective = Ta;
ref.rampMode = 'auto';
if isfinite(opts.rampTime), ref.rampMode = 'manual'; end
ref.evaluateWarped = @evaluate_warped;
ref.order = 6;
ref.isLoop = true;
ref.curveType = 'periodic_fourier';
    function xd = evaluate_time(t)
        sd = time_law(t);
        xd = wp_pack_reference(evaluate_path(sd(1)), sd, opts.yaw);
    end
    function sd = time_law(t)
        sd = wp_progress_law(t,D,v,Ta,true);
    end
    function xd = evaluate_warped(tau,tauDer)
        base = time_law(tau);
        sd = wp_compose_scalar(base,tauDer);
        xd = wp_pack_reference(evaluate_path(sd(1)),sd,opts.yaw);
    end
    function pathDer = evaluate_path(s)
        phase = 2*pi*mod(s,D)/D;
        e = exp(1i*freq*phase);
        pathDer = zeros(3,7);
        omega = 2*pi/D;
        for k = 0:6
            pathDer(:,k+1) = real(C*(e.*(1i*freq*omega).^k));
        end
    end
end
