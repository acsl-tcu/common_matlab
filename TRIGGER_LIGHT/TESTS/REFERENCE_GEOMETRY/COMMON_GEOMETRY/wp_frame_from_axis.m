function R=wp_frame_from_axis(v)
% Right handed basis with local z-axis aligned with segment.
v=v(:);if norm(v)<1e-10,error('WPGeometry:ZeroCable','Cable endpoints coincide');end
z=v/norm(v);
if abs(z(3))<0.9, aux=[0;0;1];else,aux=[0;1;0];end
x=cross(aux,z);x=x/norm(x);y=cross(z,x);
R=[x y z];
end
