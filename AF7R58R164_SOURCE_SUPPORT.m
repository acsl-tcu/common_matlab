function [h,valid]=AF7R58R164_SOURCE_SUPPORT(o,n)
valid=false;h=NaN;
if ~isfield(o,'type'),return;end
R=double(o.R_obs);c=double(o.p_center(:));
if ~isequal(size(R),[3 3])||numel(c)~=3|| ...
 any(~isfinite([R(:);c]))||norm(R'*R-eye(3),'fro')>1e-7||det(R)<0.9,return;end
u=R'*n;typ=lower(char(o.type));
switch typ
 case 'cylinder'
  if ~isfield(o,'raw_param'),return;end
  p=double(o.raw_param(:));if numel(p)~=2||any(p<=0),return;end
  excess=p(1)*norm(u(1:2))+(p(2)/2)*abs(u(3));
 case {'sphere','ball'}
  if ~isfield(o,'raw_param'),return;end
  p=double(o.raw_param(:));if numel(p)~=1||p<=0,return;end
  excess=p;
 case 'ellipsoid'
  if ~isfield(o,'ellipsoid_radii'),return;end
  p=double(o.ellipsoid_radii(:));if numel(p)~=3||any(p<=0),return;end
  excess=norm(p.*u);
 case {'box','cuboid'}
  if ~isfield(o,'raw_param'),return;end
  p=double(o.raw_param(:));if numel(p)~=3||any(p<=0),return;end
  excess=sum(abs(u).*p/2); % raw_param is complete edge lengths
 otherwise
  return;
end
h=n'*c+excess;valid=isfinite(h);
end
