function ref = gen_ref_waypoint_smooth(opts)
%GEN_REF_WAYPOINT_SMOOTH  Open route through each waypoint without intermediate stops.
% pointsXYZ: Each ROW = [x, y, z] in metres (easy to edit).
% Output: 28x1 derivatives through sixth order, no Symbolic Toolbox.
% Only open routes here; circle/saddle use gen_ref_waypoint_periodic.m.
arguments
    opts.pointsXYZ (:,3) double = [0 0 3;0 1 3;0 2 3;0 3 3]
    opts.speed (1,1) double {mustBePositive} = 0.2
    opts.rampTime (1,1) double = NaN % AUTO unless positive
    opts.maxAccel (1,1) double {mustBePositive} = 0.30
    opts.maxJerk (1,1) double {mustBePositive} = 0.50
    opts.maxD6 (1,1) double {mustBePositive} = 1.00
    opts.blendRadius (1,1) double {mustBeNonnegative} = 0.4
    opts.yaw (1,1) double {mustBeFinite} = 0
end
P = opts.pointsXYZ;
if size(P,1)<2 || any(~isfinite(P(:)))
    error('WaypointRef:Points', 'Use at least two finite XYZ waypoint rows.');
end
W = P.'; % Internal 3xN representation only.
seg = diff(W,1,2);
len = sqrt(sum(seg.^2,1));
if any(len <= 1e-8)
    error('WaypointRef:Duplicate', 'Consecutive waypoints must be distinct.');
end
direction = seg ./ len;
if size(W,2)>2
    cosine = sum(direction(:,1:end-1).*direction(:,2:end),1);
    if any(cosine <= -0.985)
        error('WaypointRef:UTurn','Near 180-degree U-turn needs a separate route.');
    end
    if opts.blendRadius==0 && any(cosine < 1-1e-10)
        error('WaypointRef:Blend','Turns require blendRadius > 0.');
    end
end
S = [0,cumsum(len)];
D = S(end);
blend = zeros(1,size(W,2));
for i = 2:size(W,2)-1
    blend(i) = min([opts.blendRadius, 0.4*len(i-1), 0.4*len(i)]);
end
[Ta, vRequest] = wp_auto_profile(D,opts.speed,false, ...
    opts.maxAccel,opts.maxJerk,opts.maxD6,opts.rampTime);
[~,T,v] = wp_progress_law(0,D,vRequest,Ta,false);
ref.evaluate = @evaluate_time;
ref.evaluatePath = @evaluate_path;
ref.timeLaw = @time_law;
ref.waypoints = W;
ref.pointsXYZ = P;
ref.cumulativeS = S;
ref.pathLength = D;
ref.duration = T;
ref.speedPeak = v;
ref.rampTimeEffective = Ta;
ref.rampMode = 'auto';
if isfinite(opts.rampTime), ref.rampMode = 'manual'; end
ref.evaluateWarped = @evaluate_warped;
ref.blendRadiusEffective = blend;
ref.order = 6;
ref.isLoop = false;
ref.curveType = 'open_waypoints';
    function xd = evaluate_time(t)
        sd = time_law(t);
        xd = wp_pack_reference(evaluate_path(sd(1)), sd, opts.yaw);
    end
    function sd = time_law(t)
        sd = wp_progress_law(t,D,v,Ta,false);
    end
    function xd = evaluate_warped(tau,tauDer)
        base = time_law(tau); % s, ds/dtau, ..., d6s/dtau6
        sd = wp_compose_scalar(base,tauDer); % derivatives wrt real time
        xd = wp_pack_reference(evaluate_path(sd(1)), sd, opts.yaw);
    end
    function pathDer = evaluate_path(s)
        s = max(0,min(D,s));
        pathDer = zeros(3,7);
        for j = 2:size(W,2)-1
            b = blend(j);
            x = s-S(j);
            if b>0 && abs(x) <= b
                u = (x+b)/(2*b);
                dd = direction(:,j)-direction(:,j-1);
                a = direction(:,j-1);
                pathDer(:,1) = W(:,j) + x*(a + wp_smoothstep13(u,0)*dd);
                for k = 1:6
                    if k==1
                        pathDer(:,2) = a + (wp_smoothstep13(u,0) + ...
                            x*wp_smoothstep13(u,1)/(2*b))*dd;
                    else
                        c0 = wp_smoothstep13(u,k-1)/(2*b)^(k-1);
                        c1 = wp_smoothstep13(u,k)/(2*b)^k;
                        pathDer(:,k+1) = (k*c0+x*c1)*dd;
                    end
                end
                return
            end
        end
        j = find(S<=s,1,'last');
        j = max(1,min(j,size(direction,2)));
        pathDer(:,1) = W(:,j) + (s-S(j))*direction(:,j);
        pathDer(:,2) = direction(:,j);
    end
end
