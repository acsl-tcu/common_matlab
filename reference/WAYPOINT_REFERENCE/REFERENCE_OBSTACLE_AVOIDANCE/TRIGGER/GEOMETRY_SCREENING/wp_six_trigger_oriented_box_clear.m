function separated=wp_six_trigger_oriented_box_clear(known,partIndices,bodyLow,bodyHigh)
%WP_SIX_TRIGGER_ORIENTED_BOX_CLEAR Proof-only oriented-ellipsoid box gate.
%   separated(j)=true only if the body AABB and the exact world-axis AABB
%   of observed obstacle part partIndices(j) are strictly disjoint.
%
%   Ellipsoid O=c+R*diag(a)*B(1) has axis-aligned support half widths:
%       extent(i)=sqrt(sum_k((R(i,k)*a(k))^2)).
%   bodyLow/bodyHigh are 3x1 and ALREADY include the selected mode's
%   drone/load/cable radii, sweep and independent forecast bounds.
%   Therefore a disjoint pair cannot intersect for ANY time in the interval.
%   This is NOT a guessed cone/radius, not a collision test, and never
%   restricts cable swing. Unresolved cases continue to the older checker.
%
%   Numerical eps guards are conservative in common double arithmetic;
%   they do NOT amount to mathematically verified outward rounding.
if isempty(partIndices)
    separated=false(1,0);return
end
if ~isfield(known,'orientedAabbHalfWidth')
    error('LightTrigger:StaleCache','Rebuild observed geometry cache with the V4 prepare function');
end
assert(isequal(size(bodyLow),[3 1]) && isequal(size(bodyHigh),[3 1]));
cent=known.center(:,partIndices);
extent=known.orientedAabbHalfWidth(:,partIndices);
% Use a scale tied to both geometry and body box, not an arbitrary margin.
scale=max(1,max(max(abs(cent)+extent,[],1),max(abs([bodyLow;bodyHigh]))));
tol=128*eps(scale);
upper=bsxfun(@plus,cent+extent,tol);
lower=bsxfun(@minus,cent-extent,tol);
separated=any(bsxfun(@gt,bodyLow,upper) | ...
              bsxfun(@lt,bodyHigh,lower),1);
end
