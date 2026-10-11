function OUT = CURRENT_C6_SENSOR_DEPTH_CAMERA_V1(t,p_WC,R_WC,known_ids_prev)
% CURRENT_C6_SENSOR_DEPTH_CAMERA_V1
%
% Literature-aligned simulated forward depth camera for the current obstacle
% environment. This is a PERCEPTION FRONT-END ONLY. It does not modify the
% trajectory planner, safe-corridor construction, or obstacle certificate.
%
% INPUT
% -----
% t               : current physical time [s]
% p_WC            : 3x1 camera optical-center position in world frame
% R_WC            : 3x3 camera-to-world rotation
%                   optical frame convention:
%                     +z : forward
%                     +x : image right
%                     +y : image down
% known_ids_prev  : row/column vector of obstacle IDs already observed
%                   [] is allowed.
%
% OUTPUT
% ------
% OUT.visible_ids       : obstacle IDs visible in THIS sensor frame
% OUT.new_ids           : newly observed IDs
% OUT.known_ids         : persistent union of previous + current detections
% OUT.visible_obstacles : full obstacle structs visible in this frame
% OUT.new_obstacles     : full obstacle structs newly detected in this frame
% OUT.known_obstacles   : full obstacle structs whose IDs are known now
% OUT.depth_m           : sampled depth image after pixel skipping
% OUT.owner_id          : obstacle ID owning each sampled depth pixel
% OUT.sensor            : exact camera configuration used
% OUT.runtime_ms        : current call runtime
%
% LITERATURE-ALIGNED BASELINE
% ---------------------------
% The defaults follow the public Fast-Planner / EGO-Planner simulation
% configuration rather than the previous omnidirectional spherical sensor:
%
%   image size       : 640 x 480
%   fx,fy            : 387.229248046875
%   cx,cy            : 321.04638671875, 243.44969177246094
%   min depth        : 0.2 m
%   max depth        : 5.0 m
%   pixel skip       : 2
%   nominal rate     : 30 Hz
%
% The GPU local_sensing package in those systems renders a depth image with
% ray occlusion. Here the same geometric idea is reproduced analytically:
% each sampled camera ray intersects the physical obstacle shape and only
% the nearest positive hit is retained.
%
% CURRENT PROJECT ASSUMPTION AFTER DETECTION
% ------------------------------------------
% The sensor only decides whether an obstacle is visible. Once at least one
% sampled depth ray sees the obstacle, its existing exact planner model
% (including p_center, R_obs, ellipsoid_radii, Q_obs, v_center, id) becomes
% available to the planner. This intentionally separates perception
% visibility from the existing exact-support trajectory geometry.
%
% IMPORTANT
% ---------
% - No spherical sensing gate is used here.
% - No yaw policy is imposed here. R_WC is an input.
% - No replanning trigger is evaluated here.
% - No safety margin is added to the optical visibility geometry.
% - Occlusion is handled by nearest depth on each sampled pixel ray.
%
% For the current ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE, finite cylinders are
% raycast using their raw physical dimensions. Other supported obstacle
% types use exact/simple raw-shape intersections where implemented, with
% enclosing-ellipsoid fallback explicitly reported in OUT.geometry_mode.

if nargin < 1 || isempty(t)
    t = 0.0;
end
if nargin < 4 || isempty(known_ids_prev)
    known_ids_prev = [];
end

tStart=tic;

validateattributes(t,{'numeric'},{'real','finite','scalar'});
validateattributes(p_WC,{'numeric'},{'real','finite','size',[3 1]});
validateattributes(R_WC,{'numeric'},{'real','finite','size',[3 3]});

if norm(R_WC.'*R_WC-eye(3),'fro') > 1e3*eps(max(1,norm(R_WC,'fro')))
    error('CURRENT_C6_SENSOR_DEPTH_CAMERA_V1:Rotation',...
        'R_WC must be an orthonormal rotation matrix.');
end
if det(R_WC) <= 0
    error('CURRENT_C6_SENSOR_DEPTH_CAMERA_V1:Rotation',...
        'R_WC must have positive determinant.');
end

S=sensor_default_config();
obs=ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(t);
nObs=numel(obs);

if nObs==0
    OUT=sensor_empty_output(S,known_ids_prev,toc(tStart)*1e3);
    return;
end

% --------------------------------------------------------------------------------------
% Sampled pinhole rays. Pixel coordinates use the same 0-based convention as the
% published intrinsics.
% --------------------------------------------------------------------------------------
u=0:S.pixel_skip:(S.width-1);
v=0:S.pixel_skip:(S.height-1);
[U,V]=meshgrid(u,v);

x=(U(:)-S.cx)/S.fx;
y=(V(:)-S.cy)/S.fy;
D_C=[x.'; y.'; ones(1,numel(x))];
D_C=D_C./vecnorm(D_C,2,1);
D_W=R_WC*D_C;

nRay=size(D_W,2);
depth=inf(1,nRay);
owner=zeros(1,nRay);
geomMode=cell(1,nObs);

for io=1:nObs
    [lam,mode]=sensor_ray_obstacle_intersection(p_WC,D_W,obs(io));
    geomMode{io}=mode;

    valid=isfinite(lam) & lam>=S.min_depth_m & lam<=S.max_depth_m;
    nearer=valid & lam<depth;

    depth(nearer)=lam(nearer);
    owner(nearer)=obs(io).id;
end

visible_ids=unique(owner(owner~=0),'stable');
known_ids_prev=unique(known_ids_prev(:).','stable');
new_ids=setdiff(visible_ids,known_ids_prev,'stable');
known_ids=unique([known_ids_prev,visible_ids],'stable');

visible_mask=arrayfun(@(x)ismember(x.id,visible_ids),obs);
new_mask=arrayfun(@(x)ismember(x.id,new_ids),obs);
known_mask=arrayfun(@(x)ismember(x.id,known_ids),obs);

depthImage=reshape(depth,size(U));
ownerImage=reshape(owner,size(U));
depthImage(~isfinite(depthImage))=NaN;

OUT=struct();
OUT.time=t;
OUT.visible_ids=visible_ids;
OUT.new_ids=new_ids;
OUT.known_ids=known_ids;
OUT.visible_obstacles=obs(visible_mask);
OUT.new_obstacles=obs(new_mask);
OUT.known_obstacles=obs(known_mask);
OUT.depth_m=depthImage;
OUT.owner_id=ownerImage;
OUT.sampled_u=u;
OUT.sampled_v=v;
OUT.sensor=S;
OUT.camera_position_W=p_WC;
OUT.R_WC=R_WC;
OUT.geometry_mode=geomMode;
OUT.n_sampled_rays=nRay;
OUT.n_visible_pixels=sum(owner~=0);
OUT.runtime_ms=toc(tStart)*1e3;
end


function S=sensor_default_config()
S=struct();

% Fast-Planner published depth-camera intrinsics / image size.
S.width=640;
S.height=480;
S.fx=387.229248046875;
S.fy=387.229248046875;
S.cx=321.04638671875;
S.cy=243.44969177246094;

% EGO/Fast-Planner-style depth filtering baseline.
S.min_depth_m=0.2;
S.max_depth_m=5.0;
S.pixel_skip=2;

% Nominal simulator sensing rate. This function remains instantaneous;
% caller decides when it is invoked.
S.nominal_rate_hz=30.0;

S.optical_frame='+z forward, +x image-right, +y image-down';
S.visibility='sampled pinhole depth rays with nearest-hit occlusion';
S.post_detection_model='existing exact planner obstacle struct';
end


function OUT=sensor_empty_output(S,known_ids_prev,runtimeMs)
OUT=struct();
OUT.time=NaN;
OUT.visible_ids=[];
OUT.new_ids=[];
OUT.known_ids=unique(known_ids_prev(:).','stable');
OUT.visible_obstacles=[];
OUT.new_obstacles=[];
OUT.known_obstacles=[];
OUT.depth_m=nan(numel(0:S.pixel_skip:(S.height-1)),...
                    numel(0:S.pixel_skip:(S.width-1)));
OUT.owner_id=zeros(size(OUT.depth_m));
OUT.sampled_u=0:S.pixel_skip:(S.width-1);
OUT.sampled_v=0:S.pixel_skip:(S.height-1);
OUT.sensor=S;
OUT.camera_position_W=nan(3,1);
OUT.R_WC=nan(3,3);
OUT.geometry_mode={};
OUT.n_sampled_rays=numel(OUT.depth_m);
OUT.n_visible_pixels=0;
OUT.runtime_ms=runtimeMs;
end


function [lambda,mode]=sensor_ray_obstacle_intersection(oW,dW,item)
% Returns Euclidean ray distance because dW columns are normalized.
type=lower(item.type);

switch type
    case 'cylinder'
        lambda=sensor_ray_finite_cylinder(oW,dW,item);
        mode='raw finite cylinder';

    case 'sphere'
        lambda=sensor_ray_sphere(oW,dW,item);
        mode='raw sphere';

    case 'ellipsoid'
        lambda=sensor_ray_ellipsoid(oW,dW,item.p_center,item.R_obs,item.ellipsoid_radii);
        mode='raw ellipsoid';

    case 'box'
        lambda=sensor_ray_box(oW,dW,item);
        mode='raw oriented box';

    otherwise
        lambda=sensor_ray_ellipsoid(oW,dW,item.p_center,item.R_obs,item.ellipsoid_radii);
        mode=['enclosing ellipsoid fallback for ',type];
end
end


function lambda=sensor_ray_finite_cylinder(oW,dW,item)
% Finite cylinder local axis is z.
% raw_param = [radius, height].

r=item.raw_param(1);
h=item.raw_param(2);

R=item.R_obs;
c=item.p_center;

o=R.'*(oW-c);
d=R.'*dW;

N=size(d,2);
lambda=inf(1,N);

% Side wall: x^2+y^2=r^2, |z|<=h/2.
A=d(1,:).^2+d(2,:).^2;
B=2*(o(1)*d(1,:)+o(2)*d(2,:));
C=o(1)^2+o(2)^2-r^2;

disc=B.^2-4*A*C;
mask=A>0 & disc>=0;

sqrtD=zeros(1,N);
sqrtD(mask)=sqrt(disc(mask));

for signRoot=[-1,1]
    root=inf(1,N);
    root(mask)=(-B(mask)+signRoot*sqrtD(mask))./(2*A(mask));

    z=o(3)+root.*d(3,:);
    valid=root>=0 & abs(z)<=h/2;
    lambda(valid)=min(lambda(valid),root(valid));
end

% End caps z = +/- h/2, radial coordinate inside disc.
for zcap=[-h/2,h/2]
    maskCap=abs(d(3,:))>0;
    root=inf(1,N);
    root(maskCap)=(zcap-o(3))./d(3,maskCap);

    x=o(1)+root.*d(1,:);
    y=o(2)+root.*d(2,:);
    valid=root>=0 & (x.^2+y.^2)<=r^2;
    lambda(valid)=min(lambda(valid),root(valid));
end
end


function lambda=sensor_ray_sphere(oW,dW,item)
r=item.raw_param;
if ~isscalar(r)
    r=r(1);
end
c=item.p_center;

oc=oW-c;
B=2*(oc.'*dW);
C=dot(oc,oc)-r^2;
disc=B.^2-4*C;

lambda=inf(1,size(dW,2));
mask=disc>=0;
sq=zeros(size(disc));
sq(mask)=sqrt(disc(mask));

r1=(-B-sq)/2;
r2=(-B+sq)/2;

v1=mask & r1>=0;
v2=mask & r2>=0;

lambda(v1)=r1(v1);
lambda(v2)=min(lambda(v2),r2(v2));
end


function lambda=sensor_ray_ellipsoid(oW,dW,c,R,radii)
o=R.'*(oW-c);
d=R.'*dW;
invR2=1./(radii(:).^2);

A=invR2.'*(d.^2);
B=2*((invR2.*o).'*d);
C=sum(invR2.*(o.^2))-1;

disc=B.^2-4*A*C;
lambda=inf(1,size(dW,2));

mask=A>0 & disc>=0;
sq=zeros(size(disc));
sq(mask)=sqrt(disc(mask));

r1=inf(size(disc));
r2=inf(size(disc));
r1(mask)=(-B(mask)-sq(mask))./(2*A(mask));
r2(mask)=(-B(mask)+sq(mask))./(2*A(mask));

v1=mask & r1>=0;
v2=mask & r2>=0;
lambda(v1)=r1(v1);
lambda(v2)=min(lambda(v2),r2(v2));
end


function lambda=sensor_ray_box(oW,dW,item)
% Oriented box with raw_param = [dx dy dz].
half=item.raw_param(:)/2;
R=item.R_obs;
c=item.p_center;

o=R.'*(oW-c);
d=R.'*dW;

N=size(d,2);
tmin=-inf(1,N);
tmax= inf(1,N);
valid=true(1,N);

for ax=1:3
    da=d(ax,:);
    oa=o(ax);
    h=half(ax);

    parallel=(da==0);
    valid(parallel & (oa<-h | oa>h))=false;

    nonpar=~parallel;
    t1=(-h-oa)./da(nonpar);
    t2=( h-oa)./da(nonpar);

    lo=min(t1,t2);
    hi=max(t1,t2);

    idx=find(nonpar);
    tmin(idx)=max(tmin(idx),lo);
    tmax(idx)=min(tmax(idx),hi);
end

valid=valid & tmax>=max(tmin,0);
lambda=inf(1,N);

outside=valid & tmin>=0;
inside=valid & tmin<0 & tmax>=0;

lambda(outside)=tmin(outside);
lambda(inside)=tmax(inside);
end
