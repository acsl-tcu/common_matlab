function R=AF7R58R39_RETURN_MATH_AUDIT(C,n)
% Evaluate directional sign of *entire* piecewise Bezier correction in continuous time.
% C is R33 two-segment candidate, each .control_points must be 3x16.
% This only checks a directional sign, NOT obstacle or dynamic safety.
validateattributes(n,{'double'},{'real','finite','numel',3}); n=n(:);assert(norm(n)>0);n=n/norm(n);
assert(isstruct(C)&&all(isfield(C,{'first','second'})),'Expected R33 piecewise Bezier candidate');
A=double(C.first.control_points); B=double(C.second.control_points);
assert(isequal(size(A),[3 16])&&isequal(size(B),[3 16]),'Require 15-degree, 3-D control points');
assert(all(isfinite([A(:);B(:)])),'Nonfinite control points');
a=n.'*A;b=n.'*B;
% Bernstein convex hull: if every coefficient >=0, all t are >=0.
R=struct('schema','AF7R58R39_DIRECTIONAL_BERNSTEIN_AUDIT_V1', ...
 'signed_control_points',[a b], ...
 'minimum_signed_control_point',min([a b]), ...
 'nonnegative_continuous_sufficient_certificate',all([a b]>=0), ...
 'negative_control_point_is_necessary_overshoot',false, ...
 'collision_safety_certified',false,'C7_certified_by_this_function',false, ...
 'admissible_for_commit',false);
% A negative control point is NOT a witness of a negative curve value.
% A tight exact sign test requires subdivision/root isolation.
end
