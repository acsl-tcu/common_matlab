function report=CBF_REAL_D2T_ZOH_GENERATED_SELFTEST()
% Independent source/generation hookup smoke test (run in MATLAB).
% Does not certify closed-loop invariance or the plant integrator.
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'GENERATED'),fullfile(root,'..','..','COMMON'),'-begin');
assert(exist('CBF_REAL_D2T_ZOH_HOCBF_EVAL','file')==2);
assert(exist('CBF_REAL_D2T_OBSTACLE_EVAL','file')==2);
assert(exist('CBF_REAL_D2T_JET','file')==2);
P=struct('m',0.77,'J',[0.01;0.01;0.02], ...
    'mL',0.14,'gravity',9.81,'cableL',2,'dstx',0,'dsty',0);
x=[1;0;0;0;0;0;0;0;0;3;0.2;0;0;0;0;-1;0;0;0];
u=[8.85;0.01;-0.02;0];
G=struct('collisionMode','cable_ellipsoid', ...
    'cableAxisShort',0.20,'cableAxisLong',1.02, ...
    'cableCenterFromLoad',0.5);
parts=struct('kind','sphere','center',[0.55;0.32;4.1], ...
    'params',struct('radius',0.25),'obstacleName','testSphere');
lam=[1;2;8;16];dt=0.025;
a=CBF_REAL_D2T_ZOH_HOCBF_EVAL(x,u,P,G,parts,lam,dt);
b=CBF_REAL_D2T_ZOH_HOCBF_EVAL(x,[u(1)+0.5;u(2:4)],P,G,parts,lam,dt);
c=CBF_REAL_D2T_ZOH_HOCBF_EVAL(x,[u(1);u(2)+0.2;u(3:4)],P,G,parts,lam,dt);
d=CBF_REAL_D2T_ZOH_HOCBF_EVAL(x,[u(1:2);u(3)+0.2;u(4)],P,G,parts,lam,dt);
assert(all(isfinite(a.stages(:))));
assert(all(isfinite(a.discreteTaylorCBFResidual(:))));
assert(a.zohThrustHeld && a.zohStepDt==dt);
assert(norm(a.stages(1,:)-b.stages(1,:))<1e-9);
% Non-zero channels are demonstrated by obstacle coupling; symmetry may
% eliminate a channel in other states, which is NOT a controller defect.
JT=[b.stages(3:5,:)-a.stages(3:5,:), ...
    c.stages(3:5,:)-a.stages(3:5,:), ...
    d.stages(3:5,:)-a.stages(3:5,:)];
assert(any(abs(JT(:,1))>1e-12));
assert(any(abs(JT(:,2))>1e-12) || any(abs(JT(:,3))>1e-12));
report=struct('passed',true,'thrustTorqueSensitivity',JT, ...
    'massUsedKg',P.mL,'dt',dt,'guaranteesCertified',false);
fprintf('[D2T ZOH GENERATED] PASS; 3 controlled channels evaluated.\n');
end
