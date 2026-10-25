function S=AF7R58R79_SELFTEST()
% R79 algebra, adapter, whole-body convex support and fail-closed checks.
% TEST geometry and nominal constants are SYNTHETIC; not runtime fallbacks.
t0=0;t1=5;L=2;delay=1;v=1.5;
A=zeros(3,8);B=zeros(3,8);
A(2,2)=v;B(2,1)=v*(t1-delay);B(2,2)=v;
C=AF7R58R79_C7_NATURAL_CONNECTOR(t0,t1,A,B,delay,L);
assert(strcmp(C.trajectory_frame,'HLC_PAYLOAD_WORLD'));
assert(~C.native_committed&&~C.whole_body_certified);
assert(max(abs(C.raw_flight_cp(3,:)-C.payload_cp(3,:)-L))<1e-14);
err=0;
for r=0:7
    L0=AF7R58R79_C7_EVAL(C,t0,r);
    R0=AF7R58R79_C7_EVAL(C,t1,r);
    err=max([err;abs(L0-A(:,r+1));abs(R0-B(:,r+1))]);
end
assert(err<1e-6,'[R79] endpoint 0..7 position jets failed.');
% Subdivision-based strict forward-speed lower bound prevents backing up
% on this synthetic affine delayed-phase example.
V=15/C.duration_s*diff(C.payload_cp,1,2);
Y=AF7R58R79_HEADING_CERT(C,pi/2,128);
assert(Y.certified,'[R79] tangent yaw C6 constructive chart failed.');
obs=struct('center',[10;0;0], 'R',eye(3),'axes',[0.5;0.5;0.5]);
G=AF7R58R79_CONTINUOUS_BODY_CERT(C,obs,L,0.2,0.1,0.2,64);
assert(G.certified,'[R79] far ellipsoid should be certified.');
obs.center=[0;0;0];
H=AF7R58R79_CONTINUOUS_BODY_CERT(C,obs,L,0.2,0.1,0.2,64);
assert(~H.certified,'[R79] collision at left endpoint must not certify.');
% Rotated nonspherical ellipsoid: support-plane certificate invariant to R.
Rz=[cos(pi/3),-sin(pi/3),0;sin(pi/3),cos(pi/3),0;0,0,1];
obs.center=[10;0;0];obs.R=Rz;obs.axes=[0.6;0.3;0.8];
J=AF7R58R79_CONTINUOUS_BODY_CERT(C,obs,L,0.2,0.1,0.2,64);
assert(J.certified,'[R79] rotated obstacle certificate failed.');
% No evidence about controller input/tension is silently promoted.
K=AF7R58R79_CONTINUOUS_KINEMATIC_CERT(C,9.81,L,3.2);
assert(isfield(K,'certified')&&isfield(K,'status'));
S=struct('status','PASS_R79_C7_DELAYED_SEAM_AND_WHOLE_BODY_SUPPORT', ...
    'jet07_error',err,'min_forward_Bernstein_m_s',Y.horizontal_forward_speed_lower_m_s, ...
    'far_geometry_certified',G.certified, ...
    'endpoint_collision_rejected',~H.certified, ...
    'rotated_geometry_certified',J.certified, ...
    'tangent_yaw_c6_certified',Y.certified, ...
    'kinematic_certified',K.certified, ...
    'production_committed',false);
end
