function V=obs_support_poly_vertices(c,options)
% Circumscribed convex polyhedron from finite supporting planes.
% Each support plane contains the entire source convex solid on one side.
if strcmp(c.kind,'poly'),V=c.vertices;return;end
N=options.polySides;
% Axis directions plus diagonals; enough bounded directions.
dirs=[eye(3);-eye(3)];
for i=1:N
 th=2*pi*(i-1)/N;
 for tilt=[0 0.65 -0.65]
  d=[cos(th)*cos(tilt),sin(th)*cos(tilt),sin(tilt)];
  dirs(end+1,:)=d; %#ok<AGROW>
 end
end
M=size(dirs,1); bb=zeros(M,1);
for i=1:M, bb(i)=obs_support(c,dirs(i,:));end
% Candidate vertices are intersections of three nonparallel support planes.
V=zeros(0,3);
for a=1:M-2
 for b=a+1:M-1
  for d=b+1:M
   H=dirs([a b d],:);
   if abs(det(H))<1e-9,continue;end
   pt=(H\bb([a b d]))';
   if all(dirs*pt'<=bb+1e-7),V(end+1,:)=pt;end %#ok<AGROW>
  end
 end
end
if size(V,1)<4,error('Supporting-plane polyhedron construction failed');end
V=unique(V,'rows'); % expansion is handled in obs_approx_convex_polyhedra
end
