function G = CBF_COMMON_GEOMETRY(P, options)
% CBF_COMMON_GEOMETRY
% Safety shapes used by REAL CBF (metres).
%
%   Drone       : sphere of physical radius 0.30 m
%   Payload     : sphere of physical radius 0.50 m
%   Cable       : thin enclosing ellipsoid for the CENTRAL cable segment
%   All shapes  : additional radial safety margin 0.05 m
%
% The drone/payload spheres already protect the cable near its endpoints.
% The cable ellipsoid therefore only needs to cover the middle segment.
% The UNION of these three shapes encloses the entire finite cable capsule.
% See the handover for the short mathematical inclusion proof.
%
% The scalar eta controls the ellipsoid aspect ratio. This is a strict
% outer approximation: it is not the actual physical cable surface.
%
% Shape matrix for cable direction pT (unit vector):
%   Qc = aShort^2 I + (aLong^2-aShort^2) pT*pT'.
%
% Existing arguments and field names are preserved for old callers.

if nargin < 2 || isempty(options)
    options = struct();
end

G = struct( ...
    'droneRadius',              0.30, ...
    'loadRadius',               0.50, ...
    'cableRadius',              0.015, ...
    'safetyMargin',             0.05, ...
    'cableEllipsoidEta',       1.00, ...
    'trimCableWithBodySpheres', true, ...
    'obstacleOuterEta',        1.00, ... % retained for older generated HOCBF functions
    'groundZ',                 0.0, ...
    'includeGround',           true);

% User overrides (same convention as the original function).
keys = fieldnames(options);
for i = 1:numel(keys)
    field = keys{i};
    if ~isfield(G,field)
        error('CBFCommon:GeometryOption', ...
            'Unknown geometry option: %s',field);
    end
    G.(field) = options.(field);
end

radii = {'droneRadius','loadRadius','cableRadius','safetyMargin'};
for i = 1:numel(radii)
    validateattributes(G.(radii{i}),{'numeric'}, ...
        {'scalar','real','finite','nonnegative'});
end
validateattributes(P.cableL,{'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(G.cableEllipsoidEta,{'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(G.obstacleOuterEta,{'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(G.trimCableWithBodySpheres,{'logical','numeric'}, ...
    {'scalar'});

G.droneSafetyRadius = G.droneRadius + G.safetyMargin;
G.loadSafetyRadius  = G.loadRadius + G.safetyMargin;
G.cableSafetyRadius = G.cableRadius + G.safetyMargin;

L = double(P.cableL);
r = double(G.cableSafetyRadius);

if G.trimCableWithBodySpheres
    % Triangle inequality: every point within r of a cable point s metres
    % from an endpoint is within that endpoint sphere whenever
    % s + r <= R_endpoint.  Trim ONLY those already protected portions.
    trimDrone = min(L/2, max(0,G.droneSafetyRadius-r));
    trimLoad  = min(L/2, max(0,G.loadSafetyRadius-r));
else
    trimDrone = 0;
    trimLoad  = 0;
end

coreLength = L - trimDrone - trimLoad;
coreHalf   = coreLength/2;

% Cable points are pLoad - c*L*pT, where pT points drone -> payload.
% Moving the ellipsoid centre is necessary when end-sphere radii differ.
coreCentreFromDrone = (trimDrone + L - trimLoad)/2;
G.cableCenterFromLoad = (L-coreCentreFromDrone)/L;

% Minkowski-sum outer ellipsoid of a segment and a radius-r sphere:
% E(segment) + E(r*I) is contained in E(Qc) with
%   aLong^2  = (1+eta)*coreHalf^2 + (1+1/eta)*r^2
%   aShort^2 = (1+1/eta)*r^2.
% Both longitudinal END CAPS of the true cable are covered by the endpoint
% spheres, so this central ellipsoid need not cover them twice.
eta = G.cableEllipsoidEta;
G.cableAxisLong  = sqrt((1+eta)*coreHalf^2 + ...
                        (1+1/eta)*r^2);
G.cableAxisShort = sqrt((1+1/eta)*r^2);

% Permanent geometry diagnostics (metres).
G.cableTrimDrone     = trimDrone;
G.cableTrimLoad      = trimLoad;
G.cableCoreLength    = coreLength;
G.cableEnclosureRule = 'UNION_OF_ENDPOINT_SPHERES_AND_CORE_ELLIPSOID';
end
