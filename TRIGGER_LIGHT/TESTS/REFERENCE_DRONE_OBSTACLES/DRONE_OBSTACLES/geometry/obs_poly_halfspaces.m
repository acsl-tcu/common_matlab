function [H,b]=obs_poly_halfspaces(V)
% Outward supporting planes H*x<=b of the convex hull of Nx3 vertices.
assert(size(V,1)>=4 && size(V,2)==3,'polyhedron requires Nx3 vertices');
K=convhulln(V); cg=mean(V,1); H=zeros(size(K,1),3); b=zeros(size(K,1),1);
for k=1:size(K,1)
 f=V(K(k,:),:); n=cross(f(2,:)-f(1,:),f(3,:)-f(1,:));n=n/norm(n);
 if dot(n,cg-f(1,:))>0,n=-n;end
 H(k,:)=n;b(k)=dot(n,f(1,:));
end
end
