function report=CBF_REAL_D2T_DISCRETE_SELFTEST()
% ID3 sampled-data four-stage smoke tests; MATLAB execution required.
% Not an intersample enclosure or closed-loop safety proof.
dt=0.025; old=[9;8.5];delta2=0.075;
T=2*old(1)-old(2)+delta2;
assert(abs((T-2*old(1)+old(2))/dt^2-120)<1e-10);
S=[10;1;1];w=[1;1;1];y=[delta2;0.02;-0.01];r=[delta2+0.2;0;0];
J=sum(w.*((y-r)./S).^2);
assert(abs(J-sum(w.*(([T;0.02;-0.01]-[T+0.2;0;0])./S).^2))<1e-12);
q=[cos(0.1);sin(0.1);0;0];pT=[0;0;-1];
x=[q;zeros(3,1);[0;0;1];[0;0.25;0];pT;zeros(3,1)];
P=struct('m',0.8,'mL',0.14,'cableL',2,'gravity',9.81, ...
    'J',[0.02;0.025;0.035],'dstx',0,'dsty',0);
part=struct('kind','sphere','center',[0.8;0.2;1.8], ...
    'params',struct('radius',0.25),'name','test_obstacle');
G=struct('a',1.02,'b',0.20,'centerFromLoad',0.5);
lam=[2;4;8;16];
a=CBF_REAL_D2T_SAMPLED_EVAL(x,old(1),old(2),y,0, ...
    P,part,G,dt,2,lam,0.005,[]);
assert(numel(a.residual)==2 && all(isfinite(a.residual)));
assert(isequal(size(a.currentStages),[4,1]));
assert(isequal(size(a.nextStages),[5,1]));
assert(a.gainCountUsed==4 && a.bootstrapUsed);
assert(abs(a.physicalInput(1)-T)<1e-12);
assert(abs(a.nextStages(5)-((a.nextStages(4)-a.currentStages(4)) ...
    +dt*lam(4)*a.nextStages(4)))<1e-8);
% Using exact previous-stage memory must NOT bootstrap a second time.
b=CBF_REAL_D2T_SAMPLED_EVAL(x,old(1),old(2),y,0, ...
    P,part,G,dt,2,lam,0.005,a.previousStagesUsed);
assert(~b.bootstrapUsed && norm(b.currentStages-a.currentStages,inf)<1e-8);
% Verify that each of the four user-selected gains influences stage4.
for j=1:4
    lamj=lam;lamj(j)=lam(j)*1.01;
    gainTest=CBF_REAL_D2T_SAMPLED_EVAL(x,old(1),old(2),y,0, ...
        P,part,G,dt,2,lamj,0.005,[]);
    assert(abs(gainTest.nextStages(5)-a.nextStages(5))>1e-12);
end
% Finite-difference probes for each physical input (not a rank proof).
for j=1:3
    yj=y;step=[0.1;0.02;0.02];yj(j)=yj(j)+step(j);
    c=CBF_REAL_D2T_SAMPLED_EVAL(x,old(1),old(2),yj,0, ...
        P,part,G,dt,2,lam,0.005,a.previousStagesUsed);
    assert(all(isfinite(c.residual)));
end
report=struct('passed',true,'d2tThrustN',delta2, ...
    'discreteAccelerationNps2',delta2/dt^2, ...
    'lambda',lam,'inputScales',S,'weightMatrix',diag(w./S.^2), ...
    'fourStagesUsed',true,'simValidated',false);
disp(report);
end
