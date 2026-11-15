function inside=obs_original_contains(o,pts,opts)
% Geometric occupancy in the world frame. Hollow-cyl uses analytic ring.
if nargin<3,opts=struct();end
q=(o.R'*(pts-o.position)')';p=o.params;
if strcmp(o.type,'hollow_cylinder')
 r2=sum(q(:,1:2).^2,2);
 inside=r2>=p.innerRadius^2 & r2<=p.outerRadius^2 & abs(q(:,3))<=p.height/2;
 return;
end
C=obs_components(o,opts);inside=false(size(q,1),1);
for i=1:numel(C),inside=inside|obs_component_contains(C(i),q);end
end
