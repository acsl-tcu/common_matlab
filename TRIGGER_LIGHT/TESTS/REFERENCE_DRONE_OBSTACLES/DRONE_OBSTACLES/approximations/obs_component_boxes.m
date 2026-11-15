function blocks=obs_component_boxes(c,n)
% Partition a guaranteed enclosing AABB into sub-boxes.
[lo,hi]=obs_component_bounds(c);
n=reshape(n,1,3);assert(prod(n)<=2048,'Too many subcells per component');
edges={linspace(lo(1),hi(1),n(1)+1), ...
       linspace(lo(2),hi(2),n(2)+1), ...
       linspace(lo(3),hi(3),n(3)+1)};
blocks=struct('mid',{},'half',{});
for i=1:n(1),for j=1:n(2),for k=1:n(3)
 a=[edges{1}(i),edges{2}(j),edges{3}(k)];
 b=[edges{1}(i+1),edges{2}(j+1),edges{3}(k+1)];
 blocks(end+1)=struct('mid',(a+b)/2,'half',(b-a)/2); %#ok<AGROW>
end,end,end
end
