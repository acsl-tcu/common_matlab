function o=obs_approx_defaults(o)
% Approximation setup. One method for the ENTIRE scene; never per obstacle.
% Conservative geometry is evaluated/precomputed only at scene creation.
if nargin<1 || isempty(o),o=struct();end
if ~isfield(o,'maxPartsPerObstacle'),o.maxPartsPerObstacle=6;end
if ~isfield(o,'maxTotalParts'),o.maxTotalParts=36;end
if ~isfield(o,'targetPartSize'),o.targetPartSize=3.0;end
if ~isfield(o,'superExponent'),o.superExponent=8;end
if ~isfield(o,'hollowSides'),o.hollowSides=16;end
if ~isfield(o,'polySides'),o.polySides=8;end
% Retained only so old scene/test option structures remain readable.
if ~isfield(o,'subdivisions'),o.subdivisions=[2 2 1];end
if ~isfield(o,'superSubdivisions'),o.superSubdivisions=[1 1 1];end
assert(isscalar(o.maxPartsPerObstacle) && isfinite(o.maxPartsPerObstacle) && ...
    o.maxPartsPerObstacle>=1 && mod(o.maxPartsPerObstacle,1)==0, ...
    'maxPartsPerObstacle must be a positive integer');
assert(isscalar(o.maxTotalParts) && isfinite(o.maxTotalParts) && ...
    o.maxTotalParts>=1 && mod(o.maxTotalParts,1)==0, ...
    'maxTotalParts must be a positive integer');
assert(isscalar(o.targetPartSize) && isfinite(o.targetPartSize) && ...
    o.targetPartSize>0,'targetPartSize must be positive [m]');
assert(isscalar(o.superExponent) && o.superExponent>=2 && ...
    mod(o.superExponent,2)==0,'superExponent must be even and >=2');
assert(isscalar(o.hollowSides) && o.hollowSides>=8 && ...
    mod(o.hollowSides,1)==0,'hollowSides must be integer >=8');
assert(isscalar(o.polySides) && o.polySides>=8 && ...
    mod(o.polySides,1)==0,'polySides must be integer >=8');
if isfield(o,'perObstacle') && ~isempty(fieldnames(o.perObstacle))
    error('DRONE_OBSTACLES:MixedApproximation', ...
        'perObstacle approximation overrides are disabled. Select ONE scene-wide method.');
end
if isfield(o,'useSceneMethod') && logical(o.useSceneMethod)
    error('DRONE_OBSTACLES:MixedApproximation', ...
        'useSceneMethod is disabled. Select ONE scene-wide method.');
end
end
