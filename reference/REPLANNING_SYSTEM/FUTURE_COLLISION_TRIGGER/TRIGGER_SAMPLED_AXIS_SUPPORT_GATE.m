function [excluded,axis_id]=TRIGGER_SAMPLED_AXIS_SUPPORT_GATE(shape,lo,hi,ru,rl,rc)
% TRIGGER_SAMPLED_AXIS_SUPPORT_GATE  Whole sampled-set static-solid exclusion.
%
% All nominal UAV and load centers lie in axis-aligned [lo,hi]. Every
% NOMINAL straight cable point is a convex combination of its endpoints,
% so it lies in the SAME box at every sampled time. For a coordinate
% normal n=+/-e_i, if min_{x in box} n'*(x-c) > h_{O-c}(n)+rmax, the
% original true solid O and ALL THREE finite physical bodies are strictly
% separated at EVERY SAMPLE. rmax=max(r_uav,r_payload,r_cable).
%
% The support is the sensor-defined original shape's convex-hull support,
% not a replacement ellipsoid. Enclosing balls are NOT used as a contact
% test. This is a SAMPLED-NOMINAL speed filter, NOT a continuous-time
% reachability certificate or mathematical floating-rounding proof.
% No unseen obstacle or future sensor state is accessed. Any invalid
% geometry/unsupported numeric query returns false (do exact fallback).
excluded=false; axis_id=0;
% A true sphere has no tighter coordinate support than its own enclosing
% sphere, which the caller has already checked by the exact AABB gap.
% Avoid adding support-function work to this common case.
if isstruct(shape) && isscalar(shape) && isfield(shape,'type') && ...
        strcmp(char(shape.type),'sphere')
    return
end
if ~(isstruct(shape)&&isscalar(shape)&&isfield(shape,'is_stationary') && ...
        isequal(shape.is_stationary,true) && ...
        isfield(shape,'center0') && isfield(shape,'bounding_radius_m') && ...
        isnumeric(lo)&&isreal(lo)&&isequal(size(lo),[3 1]) && ...
        isnumeric(hi)&&isreal(hi)&&isequal(size(hi),[3 1]) && ...
        all(isfinite([lo;hi])) && all(lo<=hi) && ...
        all(isfinite([ru rl rc])) && all([ru rl rc]>=0))
    return
end
c=double(shape.center0(:));
b=double(shape.bounding_radius_m);
if numel(c)~=3 || any(~isfinite(c)) || ~isscalar(b) || ~isfinite(b) || b<0
    return
end
try
    h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many( ...
        shape,[eye(3),-eye(3)]);
    if ~isequal(size(h),[1 6]) || any(~isfinite(h)) || any(h<0)
        return
    end
    r=max([double(ru),double(rl),double(rc)]);
    % Extra exclusion slack is conservative for engineering binary64.
    % No status from this function can grant continuous Flight safety.
    s=max([1;abs(double(lo));abs(double(hi));abs(c);abs(h(:));b;r]);
    guard=4096*eps(s);
    candidates=[double(lo)-c;c-double(hi)];
    supported=h+r+guard;
    ix=find(candidates(:).'>supported,1,'first');
    if ~isempty(ix)
        excluded=true;
        axis_id=ix;
    end
catch
    % A failed fast filter is NOT clearance. The exact existing shape
    % query remains responsible for the result, including its errors.
    excluded=false;axis_id=0;
end
end
