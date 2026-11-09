function W=TRAJECTORY_C9_POINT_INTERSECTION_PREFILTER(piece,shapes,prior_proof,rad,g,L)
% TRAJECTORY_C9_POINT_INTERSECTION_PREFILTER
% A candidate-only sufficient INTERSECTION witness. Never certifies safety.
% For an actual supporting point s of ANY of the canonical 12 obstacle
% shapes, ||p_body-s|| < r_body implies the body's physical/tube-inflated
% occupancy intersects the obstacle at this exact instant. The same holds
% for the closest point on the entire straight cable. If no witness is
% found, the caller MUST run the original independent full-time proof.
% Numerical strict inequality includes a scale-dependent roundoff guard.
% This search prefilter is not a formal outward-rounded interval proof.
W=struct('definite_intersection',false,'status','NO_POINT_WITNESS', ...
    'obstacle_id',NaN,'body_index',NaN,'time_s',NaN, ...
    'witness_distance_m',NaN,'inflated_radius_m',NaN, ...
    'shape_support_calls',0,'elapsed_ms',0);
tStart=tic;
if ~isstruct(prior_proof)||~isfield(prior_proof,'unresolved_time_interval_s')|| ...
        ~isfield(prior_proof,'unresolved_obstacle_ids')
    W.status='NO_GEOMETRY_CERTIFICATE_LEAF';W.elapsed_ms=1000*toc(tStart);return
end
ids=double(prior_proof.unresolved_obstacle_ids(:).');
if isempty(ids)
    W.status='NO_FAILED_OBSTACLE_ID';W.elapsed_ms=1000*toc(tStart);return
end
if ~any(double(rad(:))>0)
    W.status='NO_POSITIVE_BODY_RADIUS';W.elapsed_ms=1000*toc(tStart);return
end
iv=double(prior_proof.unresolved_time_interval_s(:));
if numel(iv)~=2||~all(isfinite(iv))||iv(2)<=iv(1)
    W.status='NO_VALID_TIME_INTERVAL';W.elapsed_ms=1000*toc(tStart);return
end
T=double(piece.duration_s);t0=double(piece.t0_s);
t=mean(iv);u=(t-t0)/T;
if ~isfinite(u)||u<0||u>1
    W.status='TIME_OUTSIDE_C9_PIECE';W.elapsed_ms=1000*toc(tStart);return
end
P=double(piece.control_points_m);
p=P*bern(23,u);
a=(23*22/T^2)*diff(P,2,2)*bern(21,u);
w=double(g(:))-a;sw=norm(w);
if ~(isfinite(sw)&&sw>0),W.status='CABLE_DIRECTION_UNDEFINED';W.elapsed_ms=1000*toc(tStart);return;end
q=w/sw;
uav=p-double(L)*q;
points=[uav,p,p];
if isfield(prior_proof,'unresolved_body_indices')
    bodies=unique(double(prior_proof.unresolved_body_indices(:).'));
else
    bodies=1:3;
end
bodies=bodies(ismember(bodies,[1 2 3]));
if isempty(bodies),bodies=1:3;end
for j=1:numel(shapes)
    S=shapes(j);
    if ~any(ids==double(S.id)),continue;end
    for body=bodies
        r=double(rad(body));
        if ~(isfinite(r)&&r>0),continue;end
        if body==2
            lambda=max(0,min(1,(q.'*(p-S.center))/double(L)));
            pcheck=p-lambda*double(L)*q;
        else
            pcheck=points(:,body);
        end
        n=pcheck-S.center;
        sn=norm(n);
        if ~(isfinite(sn)&&sn>0),continue;end
        for side=[1 -1]
            [~,surface]=TRAJECTORY_OBSTACLE_SUPPORT.value_and_point( ...
                S,side*n/sn);
            W.shape_support_calls=W.shape_support_calls+1;
            if body==2
                % Closest cable point to the ACTUAL surface witness.
                lam=max(0,min(1,(q.'*(p-surface))/double(L)));
                closest=p-lam*double(L)*q;
            else
                closest=pcheck;
            end
            distance=norm(closest-surface);
            scale=max([1;abs(closest(:));abs(surface(:));r]);
            if isfinite(distance)&&distance<r-32*eps(scale)
                W.definite_intersection=true;
                W.status='UNKNOWN_ACTUAL_SHAPE_POINT_INSIDE_BODY_ENVELOPE';
                W.obstacle_id=double(S.id);
                W.body_index=body;
                W.time_s=t;
                W.witness_distance_m=distance;
                W.inflated_radius_m=r;
                W.elapsed_ms=1000*toc(tStart);return
            end
        end
    end
end
W.elapsed_ms=1000*toc(tStart);
end
function b=bern(n,u)
b=zeros(n+1,1);
if u<=0,b(1)=1;return;end
if u>=1,b(end)=1;return;end
if u<=0.5
    b(1)=(1-u)^n;
    for k=1:n,b(k+1)=b(k)*(n-k+1)/k*u/(1-u);end
else
    b(end)=u^n;
    for k=n:-1:1,b(k)=b(k+1)*k/(n-k+1)*(1-u)/u;end
end
end
