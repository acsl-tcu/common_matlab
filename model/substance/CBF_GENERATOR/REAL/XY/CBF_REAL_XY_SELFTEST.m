function report = CBF_REAL_XY_SELFTEST()
% Test without modifying Sim/HLC/CBF01. Requires original CBF_COMMON_MODEL,
% CBF_COMMON_GEOMETRY and generated mode02 functions on MATLAB path.
q=[0.98;0.08;-0.12;0.10];q=q/norm(q);
pT=[0.08;-0.11;-0.99];pT=pT/norm(pT);
x=[q;0.1;-0.12;0.08;0;0;3;0.4;-0.2;0.1;pT;0.02;0.03;-0.04];
P=struct('m',0.85,'J',[0.012;0.013;0.022], ...
    'gravity',9.81,'mL',0.14,'cableL',2,'dstx',0,'dsty',0);
G=CBF_COMMON_GEOMETRY(P,struct('includeGround',false, ...
    'trimCableWithBodySpheres',false));
G.cableCenterFromLoad=0.5;
G.cableAxisShort=0.2;G.cableAxisLong=P.cableL/2+0.02;
T=9.5;yaw=0.04;u0=[T;0;0;yaw];
J0=CBF_REAL_XY_JET(x,u0,P);
Jx=CBF_REAL_XY_JET(x,u0+[0;1;0;0],P);
Jy=CBF_REAL_XY_JET(x,u0+[0;0;1;0],P);
Jq=CBF_REAL_XY_JET(x,u0+[0;0.35;-0.29;0],P);
X0=J0.stateTaylor;Xx=Jx.stateTaylor;Xy=Jy.stateTaylor;Xq=Jq.stateTaylor;
firstDiff=max(abs(X0(:,2)-CBF_COMMON_MODEL(x,u0,P)));
inds=[8:10 14:16];
L=[Xx(inds,1:4)-X0(inds,1:4), Xy(inds,1:4)-X0(inds,1:4)];
lowerDiff=max(abs(L(:)));
linPred=X0(inds,5)+.35*(Xx(inds,5)-X0(inds,5)) ...
    -.29*(Xy(inds,5)-X0(inds,5));
affineDiff=max(abs(Xq(inds,5)-linPred));
% Tilted, rotated 3-D ellipsoid; the z coordinate must not be discarded.
cz=cos(.24);sz=sin(.24);cy=cos(-.31);sy=sin(-.31);
Rz=[cz -sz 0;sz cz 0;0 0 1];Ry=[cy 0 sy;0 1 0;-sy 0 cy];
part=struct('kind','ellipsoid','center',[1.4;-0.9;2.6], ...
    'R',Rz*Ry,'params',struct('axes',[.5;.8;1.2]), ...
    'obstacleName','test3d');
lambda=[4;4;4;4];
E0=CBF_REAL_XY_EVAL(x,u0,P,G,part,lambda);
Ex=CBF_REAL_XY_EVAL(x,u0+[0;1;0;0],P,G,part,lambda);
Ey=CBF_REAL_XY_EVAL(x,u0+[0;0;1;0],P,G,part,lambda);
Eq=CBF_REAL_XY_EVAL(x,u0+[0;0.35;-0.29;0],P,G,part,lambda);
Ql=G.cableAxisShort^2*eye(3)+(G.cableAxisLong^2- ...
    G.cableAxisShort^2)*(pT*pT.');
Qo=part.R*diag(part.params.axes.^2)*part.R.';
d=x(8:10)-G.cableCenterFromLoad*P.cableL*pT-part.center;
n=d/norm(d);
staticH=n.'*d-sqrt(n.'*Ql*n)-sqrt(n.'*Qo*n);
shapeDiff=abs(staticH-E0.stages(1));
B=[Ex.stages(5)-E0.stages(5), Ey.stages(5)-E0.stages(5)];
stageLower=max(abs([Ex.stages(1:4)-E0.stages(1:4); ...
    Ey.stages(1:4)-E0.stages(1:4);Eq.stages(1:4)-E0.stages(1:4)]));
stageAffine=abs(Eq.stages(5)-E0.stages(5)-B*[.35;-.29]);
% Demonstrate pose-dependent 3-D whole-cable shape: change pT without changing x or
% obstacle centre (self-test only; not a physical time evolution).
p2=[.51;-.21;-.83];p2=p2/norm(p2);
x2=x;x2(14:16)=p2;
E2=CBF_REAL_XY_EVAL(x2,u0,P,G,part,lambda);
attitudeEffect=abs(E2.stages(1)-E0.stages(1));
partNoRotation=part;partNoRotation.R=eye(3);
E3=CBF_REAL_XY_EVAL(x,u0,P,G,partNoRotation,lambda);
rotationEffect=abs(E3.stages(1)-E0.stages(1));
assert(rotationEffect>1e-5,'CBFXY:Selftest','Obstacle rotation is ignored');

assert(firstDiff<1e-10,'CBFXY:Selftest','Physical RHS mismatch');
assert(lowerDiff<1e-10,'CBFXY:Selftest','Torque enters state early');
assert(affineDiff<1e-10,'CBFXY:Selftest','State order4 nonaffine');
assert(shapeDiff<1e-10,'CBFXY:Selftest','Direct support gap h0 mismatch');
assert(stageLower<1e-8,'CBFXY:Selftest','Torque enters lower HOCBF stages');
assert(stageAffine<1e-7,'CBFXY:Selftest','psi4 is not affine');
assert(attitudeEffect>1e-4,'CBFXY:Selftest','Cable attitude is ignored');
% Independent short-ZOH consistency: integrate the ORIGINAL physical RHS
% and compare the analytic h Taylor jet to the new direct support h.
step=1e-3;steps=40;h=step/steps;z=x;
for k=1:steps
    k1=CBF_COMMON_MODEL(z,u0,P);
    k2=CBF_COMMON_MODEL(z+0.5*h*k1,u0,P);
    k3=CBF_COMMON_MODEL(z+0.5*h*k2,u0,P);
    k4=CBF_COMMON_MODEL(z+h*k3,u0,P);
    z=z+h*(k1+2*k2+2*k3+k4)/6;
end
Eend=CBF_REAL_XY_EVAL(z,u0,P,G,part,lambda);
taylorH=E0.hTaylor(:,1).'*(step.^(0:4)).';
shortZohError=abs(taylorH-Eend.stages(1));
assert(shortZohError<1e-5,'CBFXY:Selftest','Physical ZOH vs Lie jet mismatch');

report=struct('passed',true,'physicalRhsDiff',firstDiff, ...
    'stateLowOrderTorqueDiff',lowerDiff,'stateOrder4AffineDiff',affineDiff, ...
    'threeDDirectSupportH0Diff',shapeDiff,'hocbfLowerStageTorqueDiff',stageLower, ...
    'psi4AffineDiff',stageAffine,'cableAttitudeChangesH',attitudeEffect, ...
    'psi4TorqueCoefficients',B,'obstacleRotationChangesH',rotationEffect, ...
    'shortZohTaylorErrorM',shortZohError);
fprintf('[CBF XY SELFTEST] PASS: RHS %.3g, low %.3g, affine %.3g, 3D h %.3g, pose %.3g\n', ...
    firstDiff,stageLower,stageAffine,shapeDiff,attitudeEffect);
end
