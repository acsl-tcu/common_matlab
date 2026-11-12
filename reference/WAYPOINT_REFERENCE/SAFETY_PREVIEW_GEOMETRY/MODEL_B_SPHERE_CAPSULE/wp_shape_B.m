function items=wp_shape_B(cfg,pD,pL)
% MODEL B: drone sphere, true capsule (or line if radius=0), load sphere.
items=repmat(wp_item('sphere',pD,'drone',cfg.droneRadius+cfg.safetyMargin),0,1);
items(end+1)=wp_item('sphere',pD,'drone',cfg.droneRadius+cfg.safetyMargin);
items(end+1)=wp_item('capsule',(pD+pL)/2,'cable', ...
    cfg.cableRadius+cfg.safetyMargin,eye(3),[],pD,pL);
items(end+1)=wp_item('sphere',pL,'load',cfg.loadRadius+cfg.safetyMargin);
end
