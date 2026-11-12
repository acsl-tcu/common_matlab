function items=wp_shape_D(cfg,pD,pL)
% MODEL D: drone/load spheres and ONE outward ellipsoid enclosing the full
% finite cable capsule. The ellipsoid is a PROVISIONAL geometry, NOT CBF.
% For half segment length h, cable tube radius r and eta>0, the Minkowski
% support inequality gives A^2=(1+eta)*h^2+(1+1/eta)*r^2,
% B^2=(1+1/eta)*r^2. Thus capsule subset of this ellipsoid.
items=repmat(wp_item('sphere',pD,'drone',cfg.droneRadius+cfg.safetyMargin),0,1);
items(end+1)=wp_item('sphere',pD,'drone',cfg.droneRadius+cfg.safetyMargin);
h=norm(pL-pD)/2;
r=cfg.cableRadius+cfg.safetyMargin;
eta=cfg.ellipsoidEta;
aLong=sqrt((1+eta)*h^2+(1+1/eta)*r^2);
aShort=sqrt((1+1/eta)*r^2);
if aShort<=0, aShort=1e-9;end
R=wp_frame_from_axis(pL-pD);
items(end+1)=wp_item('ellipsoid',(pD+pL)/2,'cable',0,R, ...
    [aShort;aShort;aLong]);
items(end+1)=wp_item('sphere',pL,'load',cfg.loadRadius+cfg.safetyMargin);
end
