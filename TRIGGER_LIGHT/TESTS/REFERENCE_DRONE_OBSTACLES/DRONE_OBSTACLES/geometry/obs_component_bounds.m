function [lo,hi]=obs_component_bounds(c)
switch c.kind
 case 'poly'
  lo=min(c.vertices,[],1); hi=max(c.vertices,[],1);
 case 'sphere'
  r=c.params.radius;lo=[-r -r -r];hi=-lo;
 case 'ellipsoid'
  hi=reshape(c.params.axes,1,3);lo=-hi;
 case 'cylinder'
  hi=[c.params.radius c.params.radius c.params.height/2];lo=-hi;
 case 'capsule'
  hi=[c.params.radius c.params.radius c.params.straightLength/2+c.params.radius];lo=-hi;
 case 'cone'
  hi=[c.params.radius c.params.radius c.params.height/2];lo=-hi;
 otherwise,error('invalid component kind');
end
end
