function S=AF7R58R80_SELFTEST()
% Synthetic only. Separate body vs global ball, exact oriented support,
% cable collision, unknown case, non-single-half-plane tangent yaw.
t0=0;t1=5;L=2;g=9.81;delay=1;v=1.5;
A=zeros(3,8);B=zeros(3,8);
A(2,2)=v;B(2,1)=v*(t1-delay);B(2,2)=v;
C=AF7R58R79_C7_NATURAL_CONNECTOR(t0,t1,A,B,delay,L);
ob=struct('center',[1.6;1.0;1.0],'R',eye(3), ...
    'axes',[0.25;0.25;0.25]);
rQ=0.2;rC=0.1;rL=0.2;
G=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,ob,g,L,rQ,rC,rL,1024);
assert(G.certified,'[R80] separately separated cable corridor should certify.');
Ball=AF7R58R79_CONTINUOUS_BODY_CERT(C,ob,L,rQ,rC,rL,256);
assert(~Ball.certified, ...
    '[R80] synthetic example should expose whole-body ball conservatism.');
rot=[cos(pi/3),-sin(pi/3),0;sin(pi/3),cos(pi/3),0;0,0,1];
ob.R=rot;ob.axes=[0.2;0.3;0.18];
Grot=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,ob,g,L,rQ,rC,rL,1024);
assert(Grot.certified,'[R80] rotated anisotropic obstacle should certify.');
ob.R=eye(3);ob.axes=[0.2;0.2;0.2];ob.center=[0;0;1];
X=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,ob,g,L,rQ,rC,rL,128);
assert(~X.certified,'[R80] obstacle intersecting cable endpoint must fail closed.');
ob.center=[0;0;2];
Y=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,ob,g,L,rQ,rC,rL,128);
assert(~Y.certified,'[R80] obstacle intersecting UAV must fail closed.');
ob.center=[0;0;0];
Z=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,ob,g,L,rQ,rC,rL,128);
assert(~Z.certified,'[R80] obstacle intersecting payload must fail closed.');
E=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,struct([]),g,L,rQ,rC,rL,128);
assert(~E.certified,'[R80] unknown must not certify.');
% Smooth 180-degree turn, with nonzero xy velocity everywhere, which a
% fixed global forward n=[1;0] would reject on its second half.
D=[1,0,-1;0,2,0];P=zeros(3,4);
for j=1:3,P(1:2,j+1)=P(1:2,j)+D(:,j)/3;end
for deg=3:14
    Q=zeros(3,deg+2);Q(:,1)=P(:,1);Q(:,end)=P(:,end);
    for j=1:deg
        alpha=j/(deg+1);
        Q(:,j+1)=alpha*P(:,j)+(1-alpha)*P(:,j+1);
    end
    P=Q;
end
Turn=struct('payload_cp',P,'t0',0,'t1',1);
Heading=AF7R58R80_HEADING_C6_CERT(Turn,256);
assert(Heading.certified&&Heading.horizontal_speed_lower_m_s>0, ...
    '[R80] local tangent charts should certify a nonzero-speed 180-degree turn.');
Gate=AF7R58R80_ASSESS_NATURAL_CONNECTOR( ...
    C,ob,g,L,rQ,rC,rL,3.2,128);
assert(~Gate.ready_to_commit&&~Gate.production_committed, ...
    '[R80] reference safety cannot authorize a production commit.');
assert(~G.input_constraints_certified&&~G.production_committed, ...
    '[R80] must never promote geometric proof into actuator/production proof.');
S=struct('status','PASS_R80_SEPARATE_BODY_NO_GLOBAL_BALL_AND_LOCAL_YAW', ...
    'geometry_certified',G.certified,'legacy_ball_rejected',~Ball.certified, ...
    'rotated_certified',Grot.certified, ...
    'cable_collision_rejected',~X.certified, ...
    'uav_collision_rejected',~Y.certified, ...
    'payload_collision_rejected',~Z.certified, ...
    'unknown_rejected',~E.certified, ...
    'heading_180deg_certified',Heading.certified, ...
    'heading_180deg_min_speed_m_s',Heading.horizontal_speed_lower_m_s, ...
    'continuity_order',C.continuity_order, ...
    'kinematic_and_input_certified',false, ...
    'production_gate_closed',~Gate.ready_to_commit, ...
    'production_committed',false);
end
