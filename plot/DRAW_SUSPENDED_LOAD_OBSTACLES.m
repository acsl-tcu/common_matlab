function [fig, obstacleDrawer] = DRAW_SUSPENDED_LOAD_OBSTACLES(logger, self, world, varargin)
% DRAW_SUSPENDED_LOAD_OBSTACLES
% Self-contained plotting: drone, cable, load, obstacles, ground, sensor.
% Does NOT call another DRAW/animate/plotting .m file.
% HLC, EKF, sensor detection and the simulation are never modified.
%
% world: output of DRONE_OBSTACLES(...,'single_ellipsoid',...).
% Approximation geometry comes directly from world.parts.
% Original solids use obs_components (GEOMETRY computation only).
% Dynamic obstacle geometry uses obs_world_at_time (GEOMETRY only).
% Neither dependency draws or opens a figure.
%
% Example:
%   DRAW_SUSPENDED_LOAD_OBSTACLES(app.logger, app.agent(1), worldObstacles, ...
%       'cameraMode','combined','skip',4,'fps',10,'mp4','');
%
% Camera modes: 'combined' (default; flight + all obstacles), 'scene',
% 'wholeFlight', 'follow'. Explicit 'lims' overrides the selected mode.
% Display options: showOriginal, showApprox, showGround, showLabels,
% showDroneTrail, showLoadTrail, showReference, sensorRadius (default 5),
% showSensorSphere (default false), originalAlpha, approxAlpha.
%
% NOTE: This is a visualization, not sensor detection or a safety proof.

opts = struct(varargin{:});
opts = set_default(opts,'target',1);
opts = set_default(opts,'fps',30);
opts = set_default(opts,'skip',1);
opts = set_default(opts,'pause',0.01);
opts = set_default(opts,'mp4','');
opts = set_default(opts,'cameraMode','combined');
opts = set_default(opts,'cameraHalfSpan',[5 5 4]);
opts = set_default(opts,'cameraSmoothing',0.60);
opts = set_default(opts,'lims',[]);
opts = set_default(opts,'showOriginal',true);
opts = set_default(opts,'showApprox',true);
opts = set_default(opts,'showGround',true);
opts = set_default(opts,'showLabels',true);
opts = set_default(opts,'showDroneTrail',true);
opts = set_default(opts,'showLoadTrail',true);
opts = set_default(opts,'showReference',true);
opts = set_default(opts,'trailSeconds',Inf);
opts = set_default(opts,'showSensorSphere',false);
opts = set_default(opts,'sensorRadius',5.0);
opts = set_default(opts,'originalAlpha',0.35);
opts = set_default(opts,'approxAlpha',0.23);
opts = set_default(opts,'motionSamples',21);

assert(isequal(opts.target,1), 'Only target=1 is supported.');
validateattributes(opts.fps,{'numeric'},{'scalar','finite','positive'});
validateattributes(opts.skip,{'numeric'},{'scalar','integer','positive'});
validateattributes(opts.pause,{'numeric'},{'scalar','finite','nonnegative'});
validateattributes(opts.sensorRadius,{'numeric'},{'scalar','finite','positive'});
validateattributes(opts.cameraHalfSpan,{'numeric'},{'vector','numel',3,'finite','positive'});
validateattributes(opts.cameraSmoothing,{'numeric'},{'scalar','>=',0,'<',1});
validateattributes(opts.originalAlpha,{'numeric'},{'scalar','>=',0,'<=',1});
validateattributes(opts.approxAlpha,{'numeric'},{'scalar','>=',0,'<=',1});
validateattributes(opts.motionSamples,{'numeric'},{'scalar','integer','>=',2});
if ~isempty(opts.lims)
    validateattributes(opts.lims,{'numeric'},{'size',[3,2],'finite'});
    assert(all(opts.lims(:,2)>opts.lims(:,1)),'Invalid lims.');
end
cameraMode = lower(char(opts.cameraMode));
assert(ismember(cameraMode,{'combined','scene','wholeflight','follow'}), ...
    'cameraMode must be combined, scene, wholeFlight, or follow.');
assert(isstruct(world) && isfield(world,'parts') && isfield(world,'objects'), ...
    'world must be the structure produced by DRONE_OBSTACLES.');
if ~opts.showApprox && ~opts.showOriginal
    warning('DRAW_SUSPENDED_LOAD_OBSTACLES:InvisibleObstacles', ...
        'Both showApprox and showOriginal are disabled.');
end

t = logger.data(0,'t',''); t=t(:);
p = read_logged(logger,1,'p','p');
q = read_logged(logger,1,'q','p');
u = read_logged(logger,1,'input','');
assert(size(p,2)==3 && size(u,2)==4,'Expected logged p Nx3 and input Nx4.');
pL = get_load_position(logger,self,p);
Q = convert_attitude(q);
N = min([max(numel(t)-1,0),size(p,1),size(q,1),size(u,1),size(pL,1)]);
assert(N>=1,'No completed animation samples in logger.');

% The drone, rotors, suspended load and cable are constructed HERE.
% No DRAW_SUSPENDED_LOAD, DRAW_DRONE_OBSTACLES or other plotting files.
fig = figure('Color','w');
ax = axes('Parent',fig);
hold(ax,'on');
xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
droneDrawer = create_drone(ax,self,p(1,:),Q(1,:),u(1,:),pL(1,:));
if strcmpi(get(fig,'WindowStyle'),'normal')
    fig.Units='pixels';
    pos=fig.Position; pos(3:4)=[1280,720]; fig.Position=pos;
end

% Draw ALL approximation surfaces; do not depend on DRAW_DRONE_OBSTACLES.
% It is possible to use this on systems that only have DRONE_OBSTACLES.
[obstacleDrawer, obstacleBounds] = create_layer(ax,world,opts,t(1:N));

if opts.showReference
    try
        ref = read_logged(logger,1,'p','r');
        if size(ref,2)==3
            ref=ref(1:min(size(ref,1),N),:);
            plot3(ax,ref(:,1),ref(:,2),ref(:,3),'--', ...
                'Color',[0.9 0.38 0.15],'LineWidth',1.4,'DisplayName','Reference');
        end
    catch
        % Reference path may be absent in some logger configurations.
    end
end

droneTrail=[]; loadTrail=[];
if opts.showDroneTrail
    droneTrail=plot3(ax,nan,nan,nan,'-','LineWidth',1.7, ...
        'Color',[0.1 0.25 0.8],'DisplayName','Drone trajectory');
end
if opts.showLoadTrail
    loadTrail=plot3(ax,nan,nan,nan,'-','LineWidth',1.7, ...
        'Color',[0.0 0.55 0.24],'DisplayName','Load trajectory');
end

sensorHandle=[];
if opts.showSensorSphere
    [xs,ys,zs]=sphere(28);
    sensorHandle=surf(ax,opts.sensorRadius*xs+p(1,1), ...
        opts.sensorRadius*ys+p(1,2),opts.sensorRadius*zs+p(1,3), ...
        'FaceColor',[0.1 0.8 0.85],'FaceAlpha',0.06, ...
        'EdgeColor','none','HandleVisibility','off');
end

% A critical correction compared with the old adapter: in 'combined' mode,
% distant obstacles are INCLUDED in the viewing limits by default.
flightPts=[p(1:N,:);pL(1:N,:)];
flightBounds=[min(flightPts,[],1);max(flightPts,[],1)];
if opts.showSensorSphere
    flightBounds=[min(flightBounds(1,:),min(p(1:N,:),[],1)-opts.sensorRadius); ...
        max(flightBounds(2,:),max(p(1:N,:),[],1)+opts.sensorRadius)];
end
if isempty(opts.lims)
    switch cameraMode
        case 'combined'
            bounds=join_bounds(flightBounds,obstacleBounds);
            if opts.showGround && ~isempty(world.ground)
                bounds(1,3)=min(bounds(1,3),world.ground.point(3));
            end
            apply_bounds(ax,bounds);
        case 'scene'
            bounds=obstacleBounds;
            if isempty(bounds), bounds=flightBounds; end
            apply_bounds(ax,bounds);
        case 'wholeflight'
            apply_bounds(ax,flightBounds);
        case 'follow'
            center=(p(1,:)+pL(1,:))/2;
            half=max(opts.cameraHalfSpan(:)',abs(p(1,:)-pL(1,:))/2+1);
            set_window(ax,center,half);
    end
else
    set(ax,'XLim',opts.lims(1,:),'YLim',opts.lims(2,:), ...
        'ZLim',opts.lims(3,:));
end
axis(ax,'equal'); view(ax,3); grid(ax,'on');
axis(ax,'manual');
% Keep our intended limits after axis equal adjusts the plot box.
if ~isempty(opts.lims)
    set(ax,'XLim',opts.lims(1,:),'YLim',opts.lims(2,:), ...
        'ZLim',opts.lims(3,:));
elseif strcmp(cameraMode,'follow')
    set_window(ax,(p(1,:)+pL(1,:))/2, ...
        max(opts.cameraHalfSpan(:)',abs(p(1,:)-pL(1,:))/2+1));
else
    apply_bounds(ax,bounds);
end
update_ground_extent(obstacleDrawer,ax);

writer=[]; outputName='';
if ~(isempty(opts.mp4) || isequal(opts.mp4,false))
    if isequal(opts.mp4,true)
        outputName=fullfile('Data','SuspendedLoadWithObstacles.mp4');
    else
        outputName=char(opts.mp4);
    end
    [outdir,~,~]=fileparts(outputName);
    if ~isempty(outdir) && ~exist(outdir,'dir'),mkdir(outdir);end
    writer=VideoWriter(outputName,'MPEG-4');
    writer.FrameRate=opts.fps;
    open(writer);
end

% Camera follows only when explicitly requested. The combined/scene limits
% remain fixed, which also makes video frames easier to compare.
camCenter=(p(1,:)+pL(1,:))/2;
try
    for k=1:opts.skip:N
        update_drone(droneDrawer,p(k,:),Q(k,:),u(k,:),pL(k,:));
        obstacleDrawer.update(t(k));
        if ~isempty(droneTrail) && isgraphics(droneTrail)
            j=first_trail(t,k,opts.trailSeconds);
            set(droneTrail,'XData',p(j:k,1),'YData',p(j:k,2), ...
                'ZData',p(j:k,3));
        end
        if ~isempty(loadTrail) && isgraphics(loadTrail)
            j=first_trail(t,k,opts.trailSeconds);
            set(loadTrail,'XData',pL(j:k,1),'YData',pL(j:k,2), ...
                'ZData',pL(j:k,3));
        end
        if ~isempty(sensorHandle) && isgraphics(sensorHandle)
            set(sensorHandle,'XData',opts.sensorRadius*xs+p(k,1), ...
                'YData',opts.sensorRadius*ys+p(k,2), ...
                'ZData',opts.sensorRadius*zs+p(k,3));
        end
        if isempty(opts.lims) && strcmp(cameraMode,'follow')
            requested=(p(k,:)+pL(k,:))/2;
            camCenter=opts.cameraSmoothing*camCenter+(1-opts.cameraSmoothing)*requested;
            half=max(opts.cameraHalfSpan(:)',abs(p(k,:)-pL(k,:))/2+1);
            half=max(half,max(abs([p(k,:);pL(k,:)]-camCenter),[],1)+0.7);
            set_window(ax,camCenter,half);
            update_ground_extent(obstacleDrawer,ax);
        end
        title(ax,sprintf('Suspended load / obstacles    t = %.3f s',t(k)));
        drawnow;
        if ~isempty(writer),writeVideo(writer,getframe(fig));end
        if opts.pause>0,pause(opts.pause);end
    end
catch ME
    if ~isempty(writer),close(writer);end
    rethrow(ME);
end
if ~isempty(writer)
    close(writer);
    fprintf('[DRAW_SUSPENDED_LOAD_OBSTACLES] saved: %s\n',outputName);
end
end

function opts=set_default(opts,name,value)
if ~isfield(opts,name),opts.(name)=value;end
end

function [drawer,bounds]=create_layer(ax,world,opts,timeVector)
% The approximate single ellipsoids are the CBF geometry and MUST render.
parts=world.parts;
N=numel(parts);
approxHandles=gobjects(N,1);
partMeshes=cell(N,1);
colors=lines(max(N,1));
bounds=[];
for k=1:N
    part=parts(k);
    if ~strcmp(part.kind,'ellipsoid')
        error('DRAW_SUSPENDED_LOAD_OBSTACLES:PartKind', ...
            'This renderer expects single-ellipsoid parts, got %s.',part.kind);
    end
    [V,F]=mesh_part(part);
    partMeshes{k}=V;
    xyz=rigid(V,part.center,part.R);
    bounds=join_bounds(bounds,ellipsoid_bounds(part));
    if opts.showApprox
        approxHandles(k)=patch(ax,'Vertices',xyz,'Faces',F, ...
            'FaceColor',colors(mod(k-1,size(colors,1))+1,:), ...
            'FaceAlpha',opts.approxAlpha,'EdgeColor','none', ...
            'DisplayName',['CBF ellipsoid: ',part.obstacleName]);
    end
end

% Original obstacle geometry (computed, not drawn by other files).
originalHandles=gobjects(0,1);
originalCache=struct('handle',{},'objectIndex',{},'localVertices',{});
if opts.showOriginal
    if exist('obs_components','file')~=2
        error('DRAW_SUSPENDED_LOAD_OBSTACLES:NoOriginalGeometry', ...
            'Set showOriginal=false or add DRONE_OBSTACLES/geometry to path.');
    else
        for i=1:numel(world.objects)
            obj=world.objects(i);
            C=obs_components(obj,world.options);
            for j=1:numel(C)
                try
                    [V,F]=mesh_component(C(j));
                    xyz=rigid(V,obj.position,obj.R);
                    h=patch(ax,'Vertices',xyz,'Faces',F, ...
                        'FaceColor',[0.75 0.37 0.17], ...
                        'FaceAlpha',opts.originalAlpha,'EdgeColor','none', ...
                        'DisplayName',['Source obstacle: ',obj.name]);
                    originalHandles(end+1,1)=h; %#ok<AGROW>
                    originalCache(end+1)=struct('handle',h, ...
                        'objectIndex',i,'localVertices',V); %#ok<AGROW>
                catch err
                    warning('DRAW_SUSPENDED_LOAD_OBSTACLES:OriginalSkipped', ...
                        'Original geometry %s: %s',obj.name,err.message);
                end
            end
        end
    end
end

labelHandles=gobjects(numel(world.objects),1);
if opts.showLabels
    for i=1:numel(world.objects)
        o=world.objects(i);
        labelHandles(i)=text(ax,o.position(1),o.position(2),o.position(3), ...
            ['  ',o.name],'FontSize',8,'Color',[0.1 0.1 0.1], ...
            'Interpreter','none','HandleVisibility','off');
    end
end

groundHandle=[];
if opts.showGround && isfield(world,'ground') && ~isempty(world.ground)
    g=world.ground;
    if isfield(g,'kind') && strcmp(g.kind,'halfspace') && ...
            norm(g.normal(:)-[0;0;1])<1e-8
        z=g.point(3);
        groundHandle=surf(ax,[-1 1;-1 1],[-1 -1;1 1],z*ones(2), ...
            'FaceColor',[0.5 0.5 0.5],'FaceAlpha',0.12, ...
            'EdgeColor','none','DisplayName','Ground');
    else
        warning('DRAW_SUSPENDED_LOAD_OBSTACLES:Ground', ...
            'Ground display supports horizontal z=constant planes only.');
    end
end

hasDynamic = isfield(world,'dynamicIndices') && ~isempty(world.dynamicIndices);
if hasDynamic && exist('obs_world_at_time','file')~=2
    error('DRAW_SUSPENDED_LOAD_OBSTACLES:DynamicRequired', ...
        'Moving obstacles require obs_world_at_time.m on MATLAB path.');
end
% For moving obstacles, include sampled poses in camera bounds.
% This sampling is for rendering bounds only, never for safety prediction.
if hasDynamic
    times=linspace(timeVector(1),timeVector(end),opts.motionSamples);
    for tt=times
        snapshot=obs_world_at_time(world,tt);
        for kk=1:numel(snapshot.parts)
            bounds=join_bounds(bounds,ellipsoid_bounds(snapshot.parts(kk)));
        end
    end
end

drawer=struct('update',@(tt) update_layer(tt,world,hasDynamic, ...
    approxHandles,partMeshes,originalCache,labelHandles), ...
    'groundSurface',groundHandle,'approxHandles',approxHandles, ...
    'originalHandles',originalHandles,'labels',labelHandles);
end

function update_layer(t,world,hasDynamic,approxHandles,partMeshes,originalCache,labelHandles)
if ~hasDynamic,return;end
snapshot=obs_world_at_time(world,t);
for i=1:numel(approxHandles)
    if isgraphics(approxHandles(i))
        part=snapshot.parts(i);
        set(approxHandles(i),'Vertices', ...
            rigid(partMeshes{i},part.center,part.R));
    end
end
for i=1:numel(originalCache)
    C=originalCache(i);
    if isgraphics(C.handle)
        o=snapshot.objects(C.objectIndex);
        set(C.handle,'Vertices',rigid(C.localVertices,o.position,o.R));
    end
end
for i=1:numel(labelHandles)
    if isgraphics(labelHandles(i))
        o=snapshot.objects(i);
        set(labelHandles(i),'Position',o.position);
    end
end
end

function [V,F]=mesh_part(part)
[V,F]=sphere_mesh(part.params.axes,18);
end

function [V,F]=mesh_component(C)
switch C.kind
    case 'poly'
        V=C.vertices;
        F=convhulln(V);
    case 'sphere'
        [V,F]=sphere_mesh(C.params.radius*[1 1 1],16);
    case 'ellipsoid'
        [V,F]=sphere_mesh(C.params.axes,16);
    case 'cylinder'
        [V,F]=cylinder_mesh(C.params.radius,C.params.height,30);
    case 'cone'
        [V,F]=cone_mesh(C.params.radius,C.params.height,30);
    case 'capsule'
        [V,F]=capsule_mesh(C.params.radius,C.params.straightLength,16);
    otherwise
        error('Unsupported original component kind: %s',C.kind);
end
end

function [V,F]=sphere_mesh(axes_,n)
[x,y,z]=sphere(n);
a=axes_(:)';
V=[a(1)*x(:),a(2)*y(:),a(3)*z(:)];
F=grid_faces(n+1,n+1);
end

function F=grid_faces(nr,nc)
F=zeros(2*(nr-1)*(nc-1),3);
k=0;
for c=1:nc-1
    for r=1:nr-1
        a=r+(c-1)*nr;
        k=k+1;F(k,:)=[a a+1 a+nr];
        k=k+1;F(k,:)=[a+1 a+nr+1 a+nr];
    end
end
end

function [V,F]=cylinder_mesh(r,h,n)
th=(0:n-1)'*2*pi/n;
xy=r*[cos(th) sin(th)];
V=[xy,-h/2*ones(n,1);xy,h/2*ones(n,1);0 0 -h/2;0 0 h/2];
F=zeros(4*n,3);
for k=1:n
    j=mod(k,n)+1;
    F(4*k-3,:)=[k,j,j+n];
    F(4*k-2,:)=[k,j+n,k+n];
    F(4*k-1,:)=[2*n+1,j,k];
    F(4*k,:)=[2*n+2,k+n,j+n];
end
end

function [V,F]=cone_mesh(r,h,n)
th=(0:n-1)'*2*pi/n;
V=[r*cos(th),r*sin(th),-h/2*ones(n,1);0 0 h/2;0 0 -h/2];
F=zeros(2*n,3);
for k=1:n
    j=mod(k,n)+1;
    F(2*k-1,:)=[k,j,n+1];
    F(2*k,:)=[n+2,j,k];
end
end

function [V,F]=capsule_mesh(r,h,n)
% Triangulated capsule: two hemispheres joined by a cylindrical belt.
theta=linspace(0,2*pi,n+1);
phi=[linspace(0,pi/2,n/2+1),linspace(pi/2,pi,n/2+1)];
[th,ph]=meshgrid(theta,phi);
x=r*cos(th).*sin(ph);y=r*sin(th).*sin(ph);
z=r*cos(ph);
z(1:n/2+1,:)=z(1:n/2+1,:)+h/2;
z(n/2+2:end,:)=z(n/2+2:end,:)-h/2;
V=[x(:),y(:),z(:)];F=grid_faces(size(x,1),size(x,2));
end

function xyz=rigid(V,center,R)
xyz=(R*V')'+reshape(center,1,3);
end

function b=ellipsoid_bounds(part)
a=reshape(part.params.axes,3,1);
rad=sqrt(sum((part.R.*(a')).^2,2))';
b=[part.center-rad;part.center+rad];
end

function bounds=join_bounds(a,b)
if isempty(a),bounds=b;return;end
if isempty(b),bounds=a;return;end
bounds=[min(a(1,:),b(1,:));max(a(2,:),b(2,:))];
end

function apply_bounds(ax,bounds)
if isempty(bounds),return;end
center=mean(bounds,1);
half=max((bounds(2,:)-bounds(1,:))/2+1,[2 2 2]);
set_window(ax,center,half);
end

function set_window(ax,center,half)
set(ax,'XLim',center(1)+[-half(1),half(1)], ...
    'YLim',center(2)+[-half(2),half(2)], ...
    'ZLim',center(3)+[-half(3),half(3)]);
end

function update_ground_extent(drawer,ax)
if isempty(drawer.groundSurface) || ~isgraphics(drawer.groundSurface),return;end
xl=ax.XLim;yl=ax.YLim;
set(drawer.groundSurface,'XData',[xl(1) xl(2);xl(1) xl(2)], ...
    'YData',[yl(1) yl(1);yl(2) yl(2)]);
end

function k0=first_trail(t,k,seconds)
if isinf(seconds),k0=1;return;end
k0=find(t(1:k)>=t(k)-seconds,1,'first');
if isempty(k0),k0=1;end
end

function x=read_logged(logger,source,var,attribute)
raw=logger.data(source,var,attribute);
assert(~isempty(raw),'Missing logged %s.',var);
assert(mod(size(raw,2),numel(source))==0,'Logger shape mismatch.');
x=reshape(raw,size(raw,1),size(raw,2)/numel(source),numel(source));
x=x(:,:,1);
end

function pL=get_load_position(logger,self,p)
for name={'estimator.result.state.pL','pL'}
    try
        if strcmp(name{1},'pL'),a='p';else,a='e';end
        pL=read_logged(logger,1,name{1},a);
        if size(pL,2)==3,return;end
    catch
    end
end
for name={'estimator.result.state.pT','pT'}
    try
        if strcmp(name{1},'pT'),a='p';else,a='e';end
        pT=read_logged(logger,1,name{1},a);
        if size(pT,2)==3
            pL=p+pT*self.parameter.cableL;
            return
        end
    catch
    end
end
error('DRAW_SUSPENDED_LOAD_OBSTACLES:LoadPosition', ...
    'Cannot load the suspended-load position from logger.');
end

function Q=convert_attitude(q)
if size(q,2)==3
    R=quaternion(Eul2Quat(q')');
elseif size(q,2)==4
    R=quaternion(q);
elseif size(q,2)==9
    R=quaternion(q,'rotmat','frame');
else
    error('DRAW_SUSPENDED_LOAD_OBSTACLES:Attitude', ...
        'Expected attitude with 3, 4, or 9 columns.');
end
rv=rotvec(R);
angles=vecnorm(rv,2,2);
Q=zeros(size(q,1),4);Q(:,1)=1;
idx=angles>1e-12;
Q(idx,1:3)=rv(idx,:)./angles(idx);
Q(idx,4)=angles(idx);
end


function drawer=create_drone(ax,self,p0,q0,u0,pL0)
% Body / rotor / load construction ported from DRAW_SUSPENDED_LOAD.
% All MATLAB graphical primitives are created locally; no draw dependencies.
assert(isprop(self,'parameter'),'self.parameter is required.');
param=self.parameter;
assert(isprop(param,'Lx') && isprop(param,'Ly'), ...
    'self.parameter.Lx/Ly are required for the drone frame.');
assert(isprop(param,'rotor_r'), ...
    'self.parameter.rotor_r is required for rotor geometry.');
frameSize=[param.Lx,param.Ly];
rotorR=param.rotor_r;
if isprop(param,'Length')
    loadSize=repmat(param.Length,1,3);
else
    error('DRAW_SUSPENDED_LOAD_OBSTACLES:LoadSize', ...
        'self.parameter.Length is required (same convention as original DRAW).');
end

% Original four-rotor body: cylindrical rotors, crossed arms, ellipsoid hub.
[xr,yr,zr]=cylinder([0 rotorR]);
zr=0.001*zr;
[xb,yb,zb]=ellipsoid(0,0,0,1.2*frameSize(1)/4, ...
    0.8*frameSize(2)/4,0.02);
d=frameSize/2;
rotorCenters=[d(1),d(2),0.02; -d(1),d(2),0.02; ...
    d(1),-d(2),0.02; -d(1),-d(2),0.02];
rotorColors={[1,0,0],[0,0.7,0],[0,0,1],[0,0.7,0.7]};

[x,y,z]=cylinder(0.01);
z=z*norm(d)*2;
R1=rotmat(quaternion([frameSize(2),-frameSize(1),0]* ...
    pi/(norm(frameSize)*2),'rotvec'),'point');
R2=rotmat(quaternion([frameSize(2),frameSize(1),0]* ...
    pi/(norm(frameSize)*2),'rotvec'),'point');
F1=[R1*[x(1,:);y(1,:);z(1,:)];R1*[x(2,:);y(2,:);z(2,:)]] ...
    +[d(1);d(2);0;d(1);d(2);0];
F2=[R2*[x(1,:);y(1,:);z(1,:)];R2*[x(2,:);y(2,:);z(2,:)]] ...
    +[-d(1);d(2);0;-d(1);d(2);0];

frameParts=gobjects(1,6);
thrustTransforms=gobjects(1,4);
for ii=1:4
    frameParts(ii)=surface(ax,xr+rotorCenters(ii,1), ...
        yr+rotorCenters(ii,2),zr+rotorCenters(ii,3), ...
        'FaceColor',rotorColors{ii},'EdgeColor','none');
    arrow=quiver3(ax,rotorCenters(ii,1),rotorCenters(ii,2), ...
        rotorCenters(ii,3),0,0,1, ...
        'Color',rotorColors{ii},'LineWidth',1.2,'AutoScale','off');
    thrustTransforms(ii)=hgtransform('Parent',ax);
    set(arrow,'Parent',thrustTransforms(ii));
end
frameParts(5)=surface(ax, ...
    [F1(1,:);F1(4,:);NaN(1,size(F1,2));F2(1,:);F2(4,:)], ...
    [F1(2,:);F1(5,:);NaN(1,size(F1,2));F2(2,:);F2(5,:)], ...
    [F1(3,:);F1(6,:);NaN(1,size(F1,2));F2(3,:);F2(6,:)], ...
    'FaceColor',[0.30,0.75,0.9],'EdgeColor','none');
frameParts(6)=surface(ax,xb,yb,zb, ...
    'FaceColor',[0.30,0.75,0.9],'EdgeColor','none');
frameTransform=hgtransform('Parent',ax);
set(frameParts,'Parent',frameTransform);

% Suspended load and its cable, matching original DRAW's spherical load.
[xl,yl,zl]=sphere(12);
loadSurface=surf(ax,loadSize(1)*xl,loadSize(2)*yl,loadSize(3)*zl, ...
    'FaceColor','cyan','FaceAlpha',0.5,'EdgeColor','none', ...
    'DisplayName','Suspended load');
loadTransform=hgtransform('Parent',ax);
set(loadSurface,'Parent',loadTransform);
cable=plot3(ax,[p0(1),pL0(1)], ...
    [p0(2),pL0(2)],[p0(3),pL0(3)],'k-', ...
    'LineWidth',1.4,'DisplayName','Cable');
drawer=struct('frame',frameTransform,'thrust',thrustTransforms, ...
    'load',loadTransform,'cable',cable);
update_drone(drawer,p0,q0,u0,pL0);
end

function update_drone(drawer,p,q,u,pL)
% The axis-angle pose Q is precomputed from the original logger convention.
rotation=makehgtform('axisrotate',q(1:3),q(4));
translation=makehgtform('translate',p(:)');
set(drawer.frame,'Matrix',translation*rotation);
for ii=1:4
    if u(ii)>0
        scaling=makehgtform('scale',[1,1,u(ii)]);
    elseif u(ii)<0
        scaling=makehgtform('scale',[1,1,-u(ii)])* ...
            makehgtform('xrotate',pi);
    else
        scaling=eye(4);scaling(3,3)=1e-5;
    end
    set(drawer.thrust(ii),'Matrix',translation*rotation*scaling);
end
set(drawer.load,'Matrix',makehgtform('translate',pL(:)'));
set(drawer.cable,'XData',[p(1),pL(1)], ...
    'YData',[p(2),pL(2)],'ZData',[p(3),pL(3)]);
end
