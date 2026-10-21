function AF7R25_OBSTACLE_PLOTS()
% Overlay raw/planning obstacles on existing Figure 5/6 and clone Figure 6
% into a local equal-scale Figure 12. No local run_flight_headless helper is
% called, so this function is standalone.

assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
R=evalin('base','replanner_instance');
obs=R.static_obstacle_map;
if isempty(obs),return;end

for f=[5 6]
    hf=findobj(groot,'Type','figure','Number',f);
    if isempty(hf),continue;end
    ax=findobj(hf,'Type','axes');
    if isempty(ax),continue;end
    ax=ax(1);hold(ax,'on');draw_all(ax,obs);hold(ax,'off');
end

h6=findobj(groot,'Type','figure','Number',6);
if ~isempty(h6)
    a6=findobj(h6,'Type','axes');
    if ~isempty(a6)
        figure(12);clf;
        a12=axes('Parent',gcf);
        copyobj(allchild(a6(1)),a12);
        xlabel(a12,'x [m]');ylabel(a12,'y [m]');zlabel(a12,'z [m]');
        title(a12,'Obstacle-local 3-D: trajectory + raw/planning obstacle');
        hold(a12,'on');draw_all(a12,obs);hold(a12,'off');
        view(a12,3);grid(a12,'on');box(a12,'on');daspect(a12,[1 1 1]);
        local_limits(a12,obs);
    end
end
drawnow;
end

function draw_all(ax,obs)
for i=1:numel(obs)
    o=obs(i);if ~isfield(o,'p_center'),continue;end
    c=double(o.p_center(:));if numel(c)~=3||any(~isfinite(c)),continue;end
    R=eye(3);if isfield(o,'R_obs')&&isequal(size(o.R_obs),[3 3]),R=double(o.R_obs);end
    typ='';if isfield(o,'type'),typ=lower(char(o.type));end
    rp=[];if isfield(o,'raw_param'),rp=double(o.raw_param(:));end
    if strcmp(typ,'cylinder')&&numel(rp)>=2
        draw_cylinder(ax,c,R,rp(1),rp(2));
    elseif strcmp(typ,'sphere')&&~isempty(rp)
        draw_ellipsoid(ax,c,R,rp(1)*ones(3,1),0.20);
    elseif strcmp(typ,'ellipsoid')&&numel(rp)>=3
        draw_ellipsoid(ax,c,R,rp(1:3),0.20);
    end
    if isfield(o,'ellipsoid_radii')&&numel(o.ellipsoid_radii)>=3
        a=double(o.ellipsoid_radii(:));
        if all(isfinite(a(1:3)))&&all(a(1:3)>0),draw_wire(ax,c,R,a(1:3));end
    end
    id=i;if isfield(o,'id'),id=double(o.id);end
    text(ax,c(1),c(2),c(3),sprintf(' Obs %g',id),'FontWeight','bold');
end
end

function draw_cylinder(ax,c,R,r,h)
[X,Y,Z]=cylinder(r,48);Z=(Z-0.5)*h;[X,Y,Z]=xf(X,Y,Z,c,R);
surf(ax,X,Y,Z,'FaceAlpha',0.22,'EdgeAlpha',0.12,'HandleVisibility','off');
end

function draw_ellipsoid(ax,c,R,a,fa)
[X,Y,Z]=sphere(32);X=a(1)*X;Y=a(2)*Y;Z=a(3)*Z;[X,Y,Z]=xf(X,Y,Z,c,R);
surf(ax,X,Y,Z,'FaceAlpha',fa,'EdgeAlpha',0.10,'HandleVisibility','off');
end

function draw_wire(ax,c,R,a)
[X,Y,Z]=sphere(20);X=a(1)*X;Y=a(2)*Y;Z=a(3)*Z;[X,Y,Z]=xf(X,Y,Z,c,R);
mesh(ax,X,Y,Z,'FaceAlpha',0,'EdgeAlpha',0.32,'HandleVisibility','off');
end

function [X,Y,Z]=xf(X,Y,Z,c,R)
sz=size(X);V=R*[X(:)';Y(:)';Z(:)']+c;
X=reshape(V(1,:),sz);Y=reshape(V(2,:),sz);Z=reshape(V(3,:),sz);
end

function local_limits(ax,obs)
lo=[inf;inf;inf];hi=[-inf;-inf;-inf];
for i=1:numel(obs)
    o=obs(i);if ~isfield(o,'p_center'),continue;end;c=double(o.p_center(:));
    if numel(c)~=3||any(~isfinite(c)),continue;end
    R=eye(3);if isfield(o,'R_obs')&&isequal(size(o.R_obs),[3 3]),R=double(o.R_obs);end
    e=[1;1;1];
    if isfield(o,'ellipsoid_radii')&&numel(o.ellipsoid_radii)>=3
        e=abs(R)*double(o.ellipsoid_radii(1:3));
    elseif isfield(o,'raw_param')&&isfield(o,'type')
        rp=double(o.raw_param(:));typ=lower(char(o.type));
        if strcmp(typ,'cylinder')&&numel(rp)>=2,e=abs(R)*[rp(1);rp(1);rp(2)/2];end
    end
    lo=min(lo,c-e);hi=max(hi,c+e);
end
if any(~isfinite([lo;hi])),return;end
sp=max(hi-lo);pad=max(2,0.3*max(sp,1));
xlim(ax,[lo(1)-pad hi(1)+pad]);ylim(ax,[lo(2)-pad hi(2)+pad]);zlim(ax,[lo(3)-pad hi(3)+pad]);
end
