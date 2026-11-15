function [VL,VR,ok]=obs_split_outer_cover(V)
% Split a convex *outer-cover* polytope by a central coordinate plane.
% Both clipped convex hulls together cover the full input polytope.
% This splits the actual convex geometry (via edges), not its AABB.
V=unique(double(V),'rows');
VL=[];VR=[];ok=false;
if size(V,1)<4 || rank(V-V(1,:))<3,return;end
span=max(V,[],1)-min(V,[],1);
[largest,ax]=max(span);
if largest<=1e-8,return;end
cut=(min(V(:,ax))+max(V(:,ax)))/2;
z=V(:,ax)-cut;
tolerance=1e-10*max(1,largest);
if ~any(z < -tolerance) || ~any(z > tolerance),return;end

% Every edge of the full convex hull, including all section intersections.
try
    faces=convhulln(V,{'Qt','Qx'});
catch
    faces=convhulln(V);
end
edges=unique(sort([faces(:,[1 2]);faces(:,[2 3]);faces(:,[1 3])],2),'rows');
L=V(z<=tolerance,:);
R=V(z>=-tolerance,:);
for k=1:size(edges,1)
    a=edges(k,1);b=edges(k,2);
    za=z(a);zb=z(b);
    if (za < -tolerance && zb > tolerance) || ...
       (zb < -tolerance && za > tolerance)
        alpha=-za/(zb-za);
        p=V(a,:)+alpha*(V(b,:)-V(a,:));
        L(end+1,:)=p; %#ok<AGROW>
        R(end+1,:)=p; %#ok<AGROW>
    end
end
L=unique(L,'rows');R=unique(R,'rows');
if size(L,1)<4 || size(R,1)<4 || ...
        rank(L-L(1,:))<3 || rank(R-R(1,:))<3
    return;
end
VL=L;VR=R;ok=true;
end
