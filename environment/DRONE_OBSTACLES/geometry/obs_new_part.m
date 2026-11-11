function p=obs_new_part(kind,center,R,params,vertices)
if nargin<5, vertices=zeros(0,3);end
p=struct('kind',char(kind),'center',reshape(center,1,3),'R',R, ...
 'params',params,'vertices',vertices,'H',zeros(0,3),'b',zeros(0,1), ...
 'obstacleName','','method','','componentIndex',0);
if strcmp(kind,'convex_polyhedron')
 [p.H,p.b]=obs_poly_halfspaces(vertices);
end
end
