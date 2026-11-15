function shape = wp_system_shape_at_pose(cfg,pDrone,pLoad)
% Construct system geometry from actual (or forecast) positions, 3x1.
% C/B/D each has its own generation function. No CBF derivatives here.
pDrone=pDrone(:);pLoad=pLoad(:);
assert(numel(pDrone)==3 && numel(pLoad)==3 && ...
    all(isfinite(pDrone)) && all(isfinite(pLoad)),'3D finite endpoints required');
L=norm(pLoad-pDrone);
if abs(L-cfg.cableLength)>cfg.cableLengthTol
    error('WPGeometry:CableLength', ...
        'Cable length %.4g disagrees with nominal %.4g (tol %.4g)', ...
        L,cfg.cableLength,cfg.cableLengthTol);
end
switch cfg.method
    case 'C', items=wp_shape_C(cfg,pDrone,pLoad);
    case 'B', items=wp_shape_B(cfg,pDrone,pLoad);
    case 'D', items=wp_shape_D(cfg,pDrone,pLoad);
    otherwise, error('WPGeometry:BadConfig','Unknown method');
end
shape=struct('method',cfg.method,'items',items,'pDrone',pDrone, ...
    'pLoad',pLoad,'cableLengthActual',L,'isProvisional',true);
end
