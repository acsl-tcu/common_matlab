function R=TRAJECTORY_C7_CORRIDOR_RAY(c_seed,c_nominal,geometry_rows,seed_free,nominal_free)
%TRAJECTORY_C7_CORRIDOR_RAY  Closed-form 1-D feasibility bound in FIXED planes.
%   For z(alpha)=(1-alpha)*seed+alpha*nominal, 0<=alpha<=1,
%   every fixed-normal geometry inequality c_i(alpha)<=0 is AFFINE.
%   The candidate derivative norm inequalities are CONVEX; if BOTH
%   endpoints satisfy them, every mixture satisfies them as well.
%   The function only determines the largest feasible ray fraction. It is
%   NOT a collision certificate, NOT a global optimizer and does not
%   authorize HLC commitment. The caller must verify endpoint derivative
%   bounds, continuous geometry, heading, and physical feasibility.
%
assert(isnumeric(c_seed)&&isreal(c_seed)&&isvector(c_seed)&& ...
    isnumeric(c_nominal)&&isreal(c_nominal)&&isvector(c_nominal)&& ...
    numel(c_seed)==numel(c_nominal)&& ...
    isnumeric(geometry_rows)&&isscalar(geometry_rows)&& ...
    geometry_rows==fix(geometry_rows)&&geometry_rows>=0&& ...
    geometry_rows<=numel(c_seed)&& ...
    isnumeric(seed_free)&&isreal(seed_free)&&isvector(seed_free)&& ...
    isnumeric(nominal_free)&&isreal(nominal_free)&& ...
    isequal(size(seed_free),size(nominal_free))&& ...
    all(isfinite([c_seed(:);c_nominal(:);seed_free(:);nominal_free(:)])), ...
    'TRAJECTORY_C7_CORRIDOR_RAY:Input','Finite, consistent rays required.');
cs=double(c_seed(:));cn=double(c_nominal(:));
R=struct('available',false,'fraction',0, ...
    'free_controls',double(seed_free(:)), ...
    'status','UNKNOWN_UNCERTIFIED_RAY_ENDPOINT', ...
    'geometry_rows',geometry_rows,'bounded_by_row',NaN, ...
    'integrated_deviation_reduction_proportion',0, ...
    'safe_to_commit',false);
% This is a strict algebraic prerequisite, not an optimization tolerance.
% Near-zero positive roundoff must be handled by the FINAL independent
% certificate, never silently converted to a proof of separation.
if any(cs>0) || any(cs(geometry_rows+1:end)>0) || ...
        any(cn(geometry_rows+1:end)>0)
    return
end
cg=cs(1:geometry_rows);ng=cn(1:geometry_rows);
steps=ng-cg;
idx=find(steps>0);
alpha=1;bound_row=NaN;
for k=reshape(idx,1,[])
    upper=-cg(k)/steps(k);
    if upper<alpha
        alpha=upper;bound_row=k;
    end
end
alpha=min(1,max(0,alpha));
% Stay strictly on the feasible side in binary64 whenever the ray reaches
% a separating plane. This offset is numerical, not a physical detour.
if isfinite(bound_row) && alpha>0
    alpha=max(0,alpha-32*eps(max(1,abs(alpha))));
end
R.fraction=alpha;
R.free_controls=double(seed_free(:))+alpha* ...
    (double(nominal_free(:))-double(seed_free(:)));
% Since J is a positive quadratic in (x-x_nominal), the ray J decreases
% by exactly [1-(1-alpha)^2] relative to J(seed) for ANY SPD Gram metric.
R.integrated_deviation_reduction_proportion=1-(1-alpha)^2;
R.bounded_by_row=bound_row;
R.available=true;
R.status='FIXED_CORRIDOR_RAY_ALGEBRAIC_BOUND__INDEPENDENT_PROOF_REQUIRED';
end
