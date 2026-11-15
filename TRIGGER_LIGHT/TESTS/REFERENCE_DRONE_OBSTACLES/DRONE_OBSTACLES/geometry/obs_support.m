function s=obs_support(c,n)
% Exact support function max_{x in component} dot(n,x) in local frame.
n=n(:)';
switch c.kind
 case 'poly',s=max(c.vertices*n');
 case 'sphere',s=c.params.radius*norm(n);
 case 'ellipsoid',s=norm(c.params.axes(:)'.*n);
 case 'cylinder',s=c.params.radius*norm(n(1:2))+c.params.height/2*abs(n(3));
 case 'capsule',s=c.params.radius*norm(n)+c.params.straightLength/2*abs(n(3));
 case 'cone'
  h=c.params.height/2;
  s=max(h*n(3),-h*n(3)+c.params.radius*norm(n(1:2)));
 otherwise,error('unsupported analytic support');
end
end
