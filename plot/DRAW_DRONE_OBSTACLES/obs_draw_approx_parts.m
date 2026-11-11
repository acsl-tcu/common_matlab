function obs_draw_approx_parts(parent,parts,faceColor,alpha,res)
% 近似形状は薄く重ねて表示する（元の形状を優先して読めるように）。
if nargin<5,res=25;end
[x,y,z]=sphere(res);
q=[x(:),y(:),z(:)];
for k=1:numel(parts)
 p=parts(k);
 switch p.kind
  case {'sphere','ellipsoid','superellipsoid'}
   if strcmp(p.kind,'sphere')
    pts=q*p.params.radius;
   elseif strcmp(p.kind,'ellipsoid')
    pts=q.*reshape(p.params.axes,1,3);
   else
    exponent=p.params.exponent;
    pts=(sign(q).*abs(q).^(2/exponent)).*reshape(p.params.axes,1,3);
   end
   xyz=(p.R*pts')'+p.center;
   surface('Parent',parent, ...
    'XData',reshape(xyz(:,1),size(x)), ...
    'YData',reshape(xyz(:,2),size(y)), ...
    'ZData',reshape(xyz(:,3),size(z)), ...
    'FaceColor',faceColor,'FaceAlpha',alpha,'EdgeColor','none');
  case 'convex_polyhedron'
   if size(p.vertices,1)<4,continue;end
   patch('Parent',parent,'Vertices',p.vertices, ...
    'Faces',convhulln(p.vertices), ...
    'FaceColor',faceColor,'FaceAlpha',alpha, ...
    'EdgeColor',faceColor,'EdgeAlpha',.13,'LineWidth',.25);
  otherwise
   error('Unsupported approximation kind for rendering: %s',p.kind);
 end
end
end
