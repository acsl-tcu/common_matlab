function R=CHECK_TRAJECTORY_CORRIDOR_3D_DIRECTIONS()
%CHECK_TRAJECTORY_CORRIDOR_3D_DIRECTIONS
% Rotation-covariant full-body corridor regression, all 12 shapes.
% The certified inequalities are checked, not just the reported status.
% common_matlab root -> results also saved at common_matlab root.
root=fileparts(mfilename('fullpath'));
base=fullfile(root,'reference','REPLANNING_SYSTEM');
addpath(fullfile(base,'TRAJECTORY_GENERATION'), ...
        fullfile(base,'OBSTACLE_INFORMATION'));
assert(exist('TRAJECTORY_SAFE_CORRIDOR','file')==2 && ...
    exist('OBSTACLE_INFORMATION','class')==8, ...
    'CHECK_TRAJECTORY_DIR:Path','Production corridor/obstacle source not found.');
r=struct('uav_m',0.02,'payload_m',0.03,'cable_m',0.005);
opt=struct('robust_tube_enabled',false,'min_separation_m',0.01, ...
    'numerical_guard_m',1e-10,'max_subdivision_depth',4, ...
    'max_leaf_count',64,'max_normal_iterations',40);
U=repmat([0;0;0],1,24);
L=repmat([sqrt(2);0;-sqrt(2)],1,24);
S=mkpiece(U,L);
obs=all_shapes();
% Unlike a fixed up/down bypass, rotating the ENTIRE scene (including
% obstacle local frames) must preserve feasibility and constraint meaning.
angles=[0 0 0;22 31 47;-56 14 113;67 -38 -29;88 20 173];
checked=0;
for j=1:size(angles,1)
    Q=euler_rotation(angles(j,:));
    Uj=Q*U;Lj=Q*L;Sj=mkpiece(Uj,Lj);
    oj=obs;
    for k=1:numel(oj)
        oj(k).position=Q*oj(k).position;
        oj(k).initial_position=Q*oj(k).initial_position;
        oj(k).rotation=Q*oj(k).rotation;
    end
    C=TRAJECTORY_SAFE_CORRIDOR(Sj,oj,r,opt);
    assert(C.certified && ~C.safe_to_commit && ...
        C.leaf_count>=1 && numel(C.observed_obstacle_ids)==12, ...
        'CHECK_TRAJECTORY_DIR:Rotation','Safe multi-shape scene not certified.');
    K=TRAJECTORY_CORRIDOR_LINEAR_CONSTRAINTS(C,10000);
    x=[Uj(:);Lj(:)];
    assert(all(isfinite(nonzeros(K.Aineq))) && all(isfinite(K.bineq)) && ...
        all(K.Aineq*x-K.bineq<=1e-9), ...
        'CHECK_TRAJECTORY_DIR:Linear','Guide does not satisfy its corridor.');
    V=TRAJECTORY_CORRIDOR_VALIDATE_CANDIDATE(C,Sj,oj,r,opt,10000);
    assert(V.continuous_geometry_certified && ...
        V.corridor_constraints_satisfied && ~V.safe_to_commit, ...
        'CHECK_TRAJECTORY_DIR:Independent','Geometry validation failed.');
    checked=checked+1;
end
% An obstacle impaling the cable must always fail regardless of 3D
% rotation. No preferred lateral or vertical escape side is accepted.
mid=OBSTACLE_INFORMATION.define(100,'sphere',[sqrt(2)/2 0 -sqrt(2)/2],0.12);
for j=1:size(angles,1)
    Q=euler_rotation(angles(j,:));
    oi=mid;
    oi.position=Q*oi.position;
    oi.initial_position=Q*oi.initial_position;
    oi.rotation=Q*oi.rotation;
    C=TRAJECTORY_SAFE_CORRIDOR(mkpiece(Q*U,Q*L),oi,r,opt);
    assert(~C.certified&&~C.safe_to_commit, ...
        'CHECK_TRAJECTORY_DIR:CableCollision','A penetrating cable was certified.');
    checked=checked+1;
end
% Offset along a long cable: center-to-center direction does NOT yield
% a separating plane, but a world-axis-free Minkowski search does.
Ua=repmat([-1;0;0],1,24);
La=repmat([1;0;0],1,24);
side=OBSTACLE_INFORMATION.define(102,'sphere',[-0.8 0.25 0],0.05);
for j=1:size(angles,1)
    Q=euler_rotation(angles(j,:));
    os=side;os.position=Q*os.position;
    os.initial_position=Q*os.initial_position;
    os.rotation=Q*os.rotation;
    C=TRAJECTORY_SAFE_CORRIDOR(mkpiece(Q*Ua,Q*La),os,r,opt);
    assert(C.certified && C.leaf_count==1, ...
       'CHECK_TRAJECTORY_DIR:Offset','Failed to find a valid 3D oblique separator.');
    checked=checked+1;
end
% Curved safe guide: subdivision is allowed but never an arbitrary
% direction selector. A collision guide is NOT claimed to be solved.
curve=[-2 0 2;0 3 0;0 0 0];
load=curve-[zeros(2,3);2*ones(1,3)];
near=OBSTACLE_INFORMATION.define(101,'sphere',[0 0 -1],0.1);
for j=1:size(angles,1)
    Q=euler_rotation(angles(j,:));
    oj=near;oj.position=Q*oj.position;
    oj.initial_position=Q*oj.initial_position;
    oj.rotation=Q*oj.rotation;
    C=TRAJECTORY_SAFE_CORRIDOR(mkpiece(Q*curve,Q*load),oj,r,opt);
    assert(C.certified && C.leaf_count>1, ...
        'CHECK_TRAJECTORY_DIR:Curve','Time subdivision failed after rotation.');
    checked=checked+1;
end
R=struct('status','PASS_DIRECTION_INDEPENDENT_CORRIDOR_12_SHAPES__GUIDE_REQUIRED', ...
    'cases',checked,'shape_count',12,'world_rotations',size(angles,1), ...
    'safe_to_commit',false,'full_dynamics_certified',false, ...
    'can_find_detour_from_colliding_guide',false);
fid=fopen(fullfile(root,'TRAJECTORY_CORRIDOR_3D_DIRECTIONS_RESULT.txt'),'wt');
assert(fid>0,'CHECK_TRAJECTORY_DIR:Output','Cannot create audit output.');
cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',R.status);
fprintf(fid,'cases=%d shape_count=%d world_rotations=%d\n', ...
    R.cases,R.shape_count,R.world_rotations);
end

function S=mkpiece(U,L)
u=struct('control_points_m',U,'t0_s',0,'duration_s',2);
l=struct('control_points_m',L,'t0_s',0,'duration_s',2);
S=struct('uav',u,'payload',l);
end
function Q=euler_rotation(a)
r=a(1)*pi/180;p=a(2)*pi/180;y=a(3)*pi/180;
Rx=[1 0 0;0 cos(r) -sin(r);0 sin(r) cos(r)];
Ry=[cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
Rz=[cos(y) -sin(y) 0;sin(y) cos(y) 0;0 0 1];
Q=Rz*Ry*Rx;
end
function o=all_shapes()
vert=[0 0 0;1 0 0;0 1 0;0 0 1];
face=[1 3 2;1 2 4;1 4 3;2 3 4];
a={ ...
OBSTACLE_INFORMATION.define(1,'sphere',[0 6 0],0.3), ...
OBSTACLE_INFORMATION.define(2,'ellipsoid',[0 6 0],[0.5 1 2],'euler_deg',[20 40 50]), ...
OBSTACLE_INFORMATION.define(3,'box',[0 6 0],[1 0.6 2],'euler_deg',[5 25 40]), ...
OBSTACLE_INFORMATION.define(4,'cylinder',[0 6 0],[0.5 2],'euler_deg',[20 20 15]), ...
OBSTACLE_INFORMATION.define(5,'capsule',[0 6 0],[0.3 1]), ...
OBSTACLE_INFORMATION.define(6,'cone',[0 6 0],[0.5 2],'euler_deg',[10 20 30]), ...
OBSTACLE_INFORMATION.define(7,'frustum',[0 6 0],[0.8 0.3 1]), ...
OBSTACLE_INFORMATION.define(8,'pyramid',[0 6 0],[1 2]), ...
OBSTACLE_INFORMATION.define(9,'regular_prism',[0 6 0],[6 0.7 1]), ...
OBSTACLE_INFORMATION.define(10,'torus',[0 6 0],[1 0.2]), ...
OBSTACLE_INFORMATION.define(11,'polygon_prism',[0 6 0], ...
        struct('outline',[-1 -1;1 -1;1 1;-1 1],'height',2)), ...
OBSTACLE_INFORMATION.define(12,'polyhedron',[0 6 0], ...
        struct('vertices',vert,'faces',face)) ...
};
o=[a{:}];
end
