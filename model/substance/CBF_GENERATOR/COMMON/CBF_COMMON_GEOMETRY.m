function G = CBF_COMMON_GEOMETRY(P, options)
% CBF_COMMON_GEOMETRY  Physical safety geometry, independent of CBF method.
% Model D from WAYPOINT_REFERENCE: drone/load spheres + one cable ellipsoid
% outwardly enclosing the entire finite cable capsule.
% Does not impose any thrust, torque, relative-degree or discretization rule.
if nargin<2, options=struct(); end
G=struct('droneRadius',0.25,'loadRadius',0.16,'cableRadius',0.015,...
    'safetyMargin',0.05,'cableEllipsoidEta',0.25,...
    'obstacleOuterEta',1.0,'groundZ',0.0,'includeGround',true);
fields=fieldnames(options);
for i=1:numel(fields)
    if ~isfield(G,fields{i})
        error('CBFCommon:GeometryOption','Unknown geometry field: %s',fields{i});
    end
    G.(fields{i})=options.(fields{i});
end
numbers={'droneRadius','loadRadius','cableRadius','safetyMargin'};
for i=1:numel(numbers)
    v=G.(numbers{i});
    validateattributes(v,{'double'},{'scalar','finite','nonnegative'});
end
validateattributes(P.cableL,{'double'},{'scalar','positive','finite'});
validateattributes(G.cableEllipsoidEta,{'double'},{'scalar','positive','finite'});
validateattributes(G.obstacleOuterEta,{'double'},{'scalar','positive','finite'});
G.droneSafetyRadius=G.droneRadius+G.safetyMargin;
G.loadSafetyRadius=G.loadRadius+G.safetyMargin;
r=G.cableRadius+G.safetyMargin;
h=P.cableL/2;
eta=G.cableEllipsoidEta;
G.cableAxisLong=sqrt((1+eta)*h^2+(1+1/eta)*r^2);
G.cableAxisShort=sqrt((1+1/eta)*r^2);
% For normalized pT: Qc=aShort^2*I+(aLong^2-aShort^2)*pT*pT'.
% pDrone = pLoad - L*pT; cable center = pLoad - (L/2)*pT.
end
