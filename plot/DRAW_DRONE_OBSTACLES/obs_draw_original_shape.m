function obs_draw_original_shape(parent,o,options,faceColor,alpha,res)
% 元形状をできるだけ忠実に描画する。近似・衝突判定には使用しない。
% parent は hgtransform。全メッシュ頂点は時刻0のworld座標で登録する。
if nargin<6,res=32;end
if strcmp(o.type,'hollow_cylinder')
 draw_hollow_cylinder();return;
end
C=obs_components(o,options);
for ci=1:numel(C)
 c=C(ci);
 switch c.kind
  case 'poly'
   V=world_points(c.vertices);
   F=convhulln(V);
   patch('Parent',parent,'Vertices',V,'Faces',F, ...
       'FaceColor',faceColor,'FaceAlpha',alpha, ...
       'EdgeColor',faceColor*.72,'EdgeAlpha',.24,'LineWidth',.4);
  case {'sphere','ellipsoid'}
   [x,y,z]=sphere(res);
   if strcmp(c.kind,'sphere'),s=c.params.radius*[1 1 1];
   else,s=c.params.axes;end
   draw_surface(x*s(1),y*s(2),z*s(3));
  case 'cylinder'
   phi=linspace(0,2*pi,res+1);H=c.params.height;r=c.params.radius;
   xx=repmat(r*cos(phi),2,1);yy=repmat(r*sin(phi),2,1);
   zz=repmat([-H/2;H/2],1,numel(phi));draw_surface(xx,yy,zz);
   draw_cap(r, -H/2);draw_cap(r,H/2);
  case 'cone'
   phi=linspace(0,2*pi,res+1);r=c.params.radius;H=c.params.height;
   xx=[r*cos(phi);zeros(1,numel(phi))];
   yy=[r*sin(phi);zeros(1,numel(phi))];
   zz=repmat([-H/2;H/2],1,numel(phi));draw_surface(xx,yy,zz);
   draw_cap(r,-H/2);
  case 'capsule'
   phi=linspace(0,2*pi,res+1);r=c.params.radius;L=c.params.straightLength;
   xx=repmat(r*cos(phi),2,1); yy=repmat(r*sin(phi),2,1);
   zz=repmat([-L/2;L/2],1,numel(phi));draw_surface(xx,yy,zz);
   [sx,sy,sz]=sphere(res);
   iup=find(sz(:,1)>=-1e-10);
   ilo=find(sz(:,1)<=1e-10);
   draw_surface(r*sx(iup,:),r*sy(iup,:),r*sz(iup,:)+L/2);
   draw_surface(r*sx(ilo,:),r*sy(ilo,:),r*sz(ilo,:)-L/2);
 end
end

    function X=world_points(P)
        X=(o.R*P')'+o.position;
    end
    function draw_surface(X,Y,Z)
        shape=size(X);
        P=world_points([X(:),Y(:),Z(:)]);
        surface('Parent',parent,'XData',reshape(P(:,1),shape), ...
            'YData',reshape(P(:,2),shape),'ZData',reshape(P(:,3),shape), ...
            'FaceColor',faceColor,'FaceAlpha',alpha,'EdgeColor','none');
    end
    function draw_cap(r,zv)
        t=linspace(0,2*pi,res+1);
        draw_surface([zeros(1,numel(t));r*cos(t)], ...
            [zeros(1,numel(t));r*sin(t)],zv*ones(2,numel(t)));
    end
    function draw_hollow_cylinder()
        p=o.params;phi=linspace(0,2*pi,res+1);
        for r=[p.outerRadius,p.innerRadius]
            X=repmat(r*cos(phi),2,1);Y=repmat(r*sin(phi),2,1);
            Z=repmat([-p.height/2;p.height/2],1,numel(phi));
            draw_surface(X,Y,Z);
        end
        for z0=[-p.height/2,p.height/2]
            X=[p.innerRadius*cos(phi);p.outerRadius*cos(phi)];
            Y=[p.innerRadius*sin(phi);p.outerRadius*sin(phi)];
            draw_surface(X,Y,z0*ones(2,numel(phi)));
        end
    end
end
