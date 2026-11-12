function cfg = wp_system_geometry_config(method, opts)
% WP_SYSTEM_GEOMETRY_CONFIG  PROVISIONAL geometry. Replace with future CBF geometry.
% method: 'C','B','D' or longer aliases. Dimensions in metres.
% Passing a complete cfg to collision/preview guarantees same geometry settings.
if nargin<2, opts=struct(); end
if nargin<1 || isempty(method), method='C'; end
switch lower(char(method))
    case {'c','multiple_spheres','c_multiple_spheres'}, method='C';
    case {'b','sphere_capsule','b_sphere_capsule'}, method='B';
    case {'d','sphere_cable_ellipsoid','d_sphere_cable_ellipsoid'}, method='D';
    otherwise, error('WPGeometry:Method','Unknown system geometry %s',char(method));
end
cfg=struct('method',method,'droneRadius',0.25,'loadRadius',0.16, ...
    'cableRadius',0.015,'safetyMargin',0.05,'cableLength',2.0, ...
    'cableSegments',8,'ellipsoidEta',0.25,'cableLengthTol',0.15, ...
    'nearDistance',1.0,'checkGround',true, ...
    'isProvisional',true,'cbfMatched',false);
if ~isstruct(opts) || ~isscalar(opts), error('WPGeometry:Options','opts must be scalar struct');end
fn=fieldnames(opts);
for i=1:numel(fn)
    if ~isfield(cfg,fn{i}),error('WPGeometry:UnknownOption','Unknown option %s',fn{i});end
    cfg.(fn{i})=opts.(fn{i});
end
numbers={'droneRadius','loadRadius','cableRadius','safetyMargin','cableLength','cableLengthTol','nearDistance'};
for i=1:numel(numbers)
    v=cfg.(numbers{i});
    assert(isscalar(v) && isfinite(v) && v>=0,'Invalid geometry parameter %s',numbers{i});
end
assert(cfg.cableLength>0 && cfg.droneRadius>0 && cfg.loadRadius>0);
assert(isscalar(cfg.cableSegments) && cfg.cableSegments>=1 && mod(cfg.cableSegments,1)==0);
assert(isscalar(cfg.ellipsoidEta) && isfinite(cfg.ellipsoidEta) && cfg.ellipsoidEta>0);
assert(isscalar(cfg.checkGround));
end
