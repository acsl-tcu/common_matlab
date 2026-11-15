function items=wp_shape_C(cfg,pD,pL)
% MODEL C: drone sphere + overlapping cable spheres + load sphere.
% Radii satisfy exact geometric inclusion of the tensioned cable capsule:
% r_chain = sqrt((L/(2*N))^2 + (r_c + margin)^2).
N=cfg.cableSegments; L=norm(pL-pD);
rCable=cfg.cableRadius+cfg.safetyMargin;
rChain=hypot(L/(2*N),rCable);
items=repmat(wp_item('sphere',pD,'drone',cfg.droneRadius+cfg.safetyMargin),0,1);
items(end+1)=wp_item('sphere',pD,'drone',cfg.droneRadius+cfg.safetyMargin);
for k=0:N
    c=pD+(k/N)*(pL-pD);
    items(end+1)=wp_item('sphere',c,sprintf('cable_%02d',k),rChain); %#ok<AGROW>
end
items(end+1)=wp_item('sphere',pL,'load',cfg.loadRadius+cfg.safetyMargin);
end
