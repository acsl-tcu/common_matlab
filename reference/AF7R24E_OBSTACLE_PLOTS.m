function AF7R24E_OBSTACLE_PLOTS(logger,dt,replanner_instance)
% Raw obstacle + planner ellipsoid visualization.
% Diagnostic only; no planning/certification state is modified.

obs=replanner_instance.static_obstacle_map;
if isempty(obs),return;end

% Overlay on existing global 3-D plots.
for fig=[5 6]
    if ishghandle(fig)
        figure(fig);
        ax=gca;
        hold(ax,'on');
        draw_all(ax,obs,true);
        hold(ax,'off');
    end
end

% Dedicated local equal-scale view.
plot_suspended_load_trajectory(logger,dt, ...
    "figure_number",12, ...
    "phase","f", ...
    "title_text","Obstacle-local 3-D: raw obstacle + planning envelope");
figure(12);
ax=gca;
hold(ax,'on');
draw_all(ax,obs,true);
hold(ax,'off');
view(ax,3);
grid(ax,'on');
box(ax,'on');
daspect(ax,[1 1 1]);
set_local_limits(ax,obs);
end

function draw_all(ax,obs,drawEnvelope)
for i=1:numel(obs)
    o=obs(i);
    if ~isfield(o,'p_center'),continue;end
    c=double(o.p_center(:));
    if numel(c)~=3||any(~isfinite(c)),continue;end

    R=eye(3);
    if isfield(o,'R_obs')&&isequal(size(o.R_obs),[3 3])
        R=double(o.R_obs);
    end

    typ='';
    if isfield(o,'type'),typ=lower(char(o.type));end
    rp=[];
    if isfield(o,'raw_param'),rp=double(o.raw_param(:));end

    switch typ
        case 'cylinder'
            if numel(rp)>=2,draw_cylinder(ax,c,R,rp(1),rp(2));end
        case 'sphere'
            if ~isempty(rp),draw_ellipsoid(ax,c,R,rp(1)*ones(3,1),0.17);end
        case 'ellipsoid'
            if numel(rp)>=3,draw_ellipsoid(ax,c,R,rp(1:3),0.17);end
        otherwise
            if isfield(o,'ellipsoid_radii')&&numel(o.ellipsoid_radii)>=3
                draw_ellipsoid(ax,c,R,double(o.ellipsoid_radii(1:3)),0.17);
            end
    end

    % Planning/certification ellipsoid is shown separately when available.
    if drawEnvelope && isfield(o,'ellipsoid_radii')&&numel(o.ellipsoid_radii)>=3
        a=double(o.ellipsoid_radii(:));
        if all(isfinite(a(1:3)))&&all(a(1:3)>0)
            draw_wire_ellipsoid(ax,c,R,a(1:3));
        end
    end

    id=i;
    if isfield(o,'id'),id=double(o.id);end
    text(ax,c(1),c(2),c(3),sprintf('  Obs %g',id), ...
        'FontWeight','bold','Clipping','on');
end
end

function draw_cylinder(ax,c,R,r,h)
if ~(isfinite(r)&&r>0&&isfinite(h)&&h>0),return;end
[X,Y,Z]=cylinder(r,48);
Z=(Z-0.5)*h;
[X,Y,Z]=xform(X,Y,Z,c,R);
surf(ax,X,Y,Z,'FaceAlpha',0.22,'EdgeAlpha',0.12,'HandleVisibility','off');

th=linspace(0,2*pi,49);
for sg=[-1 1]
    V=R*[r*cos(th);r*sin(th);sg*(h/2)*ones(size(th))]+c;
    patch(ax,V(1,:),V(2,:),V(3,:),[0.5 0.5 0.5], ...
        'FaceAlpha',0.12,'EdgeAlpha',0.20,'HandleVisibility','off');
end
end

function draw_ellipsoid(ax,c,R,a,alpha)
a=double(a(:));
if numel(a)~=3||any(~isfinite(a))||any(a<=0),return;end
[X,Y,Z]=sphere(36);
X=a(1)*X;Y=a(2)*Y;Z=a(3)*Z;
[X,Y,Z]=xform(X,Y,Z,c,R);
surf(ax,X,Y,Z,'FaceAlpha',alpha,'EdgeAlpha',0.10,'HandleVisibility','off');
end

function draw_wire_ellipsoid(ax,c,R,a)
[X,Y,Z]=sphere(20);
X=a(1)*X;Y=a(2)*Y;Z=a(3)*Z;
[X,Y,Z]=xform(X,Y,Z,c,R);
mesh(ax,X,Y,Z,'FaceAlpha',0,'EdgeAlpha',0.32,'HandleVisibility','off');
end

function [X,Y,Z]=xform(X,Y,Z,c,R)
sz=size(X);
V=R*[X(:)';Y(:)';Z(:)']+c;
X=reshape(V(1,:),sz);
Y=reshape(V(2,:),sz);
Z=reshape(V(3,:),sz);
end

function set_local_limits(ax,obs)
lo=[inf;inf;inf];hi=[-inf;-inf;-inf];
for i=1:numel(obs)
    o=obs(i);
    if ~isfield(o,'p_center'),continue;end
    c=double(o.p_center(:));
    if numel(c)~=3||any(~isfinite(c)),continue;end
    R=eye(3);
    if isfield(o,'R_obs')&&isequal(size(o.R_obs),[3 3]),R=double(o.R_obs);end

    e=[1;1;1];
    if isfield(o,'ellipsoid_radii')&&numel(o.ellipsoid_radii)>=3
        a=double(o.ellipsoid_radii(1:3));
        e=abs(R)*a(:);
    elseif isfield(o,'raw_param')
        rp=double(o.raw_param(:));
        typ='';if isfield(o,'type'),typ=lower(char(o.type));end
        if strcmp(typ,'cylinder')&&numel(rp)>=2
            e=abs(R)*[rp(1);rp(1);rp(2)/2];
        elseif strcmp(typ,'sphere')&&~isempty(rp)
            e=rp(1)*ones(3,1);
        elseif strcmp(typ,'ellipsoid')&&numel(rp)>=3
            e=abs(R)*rp(1:3);
        end
    end
    lo=min(lo,c-e);
    hi=max(hi,c+e);
end
if any(~isfinite([lo;hi])),return;end
span=max(hi-lo);
pad=max(2,0.30*max(span,1));
xlim(ax,[lo(1)-pad hi(1)+pad]);
ylim(ax,[lo(2)-pad hi(2)+pad]);
zlim(ax,[lo(3)-pad hi(3)+pad]);
end
