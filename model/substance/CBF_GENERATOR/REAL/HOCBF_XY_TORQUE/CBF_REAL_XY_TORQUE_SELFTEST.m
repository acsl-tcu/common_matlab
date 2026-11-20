function report = CBF_REAL_XY_TORQUE_SELFTEST()
% Standalone, pre-Sim jet checks using original physical 19-state RHS.
% Run mainGUI once to populate paths, or addpath(genpath(projectRoot)).
q=[0.98;0.08;-0.12;0.10];q=q/norm(q);
pT=[0.08;-0.11;-0.99];pT=pT/norm(pT);
x=[q;0.1;-0.12;0.08;0;0;3;0.4;-0.2;0.1;pT;0.02;0.03;-0.04];
P=struct('m',0.85,'J',[0.012;0.013;0.022], ...
    'gravity',9.81,'mL',0.14,'cableL',2,'dstx',0,'dsty',0);
T=9.5;yaw=0.04;u0=[T;0;0;yaw];
J0=CBF_REAL_XY_TORQUE_JET(x,u0,P);
Jx=CBF_REAL_XY_TORQUE_JET(x,u0+[0;1;0;0],P);
Jy=CBF_REAL_XY_TORQUE_JET(x,u0+[0;0;1;0],P);
Jq=CBF_REAL_XY_TORQUE_JET(x,u0+[0;0.35;-0.29;0],P);
X0=J0.stateTaylor; Xx=Jx.stateTaylor; Xy=Jy.stateTaylor;Xq=Jq.stateTaylor;
dx=CBF_COMMON_MODEL(x,u0,P);
firstDiff=max(abs(X0(:,2)-dx));
% Positions and cable direction must NOT depend on tauX/tauY through h^(3).
inds=[8:10 14:16];
L=[Xx(inds,1:4)-X0(inds,1:4), ...
    Xy(inds,1:4)-X0(inds,1:4)];
lowerDiff=max(abs(L(:)));
linPred=X0(inds,5)+.35*(Xx(inds,5)-X0(inds,5)) ...
    -.29*(Xy(inds,5)-X0(inds,5));
affineDiff=max(abs(Xq(inds,5)-linPred));
assert(firstDiff<1e-10,'CBFXY:Selftest','Model derivative mismatch');
assert(lowerDiff<1e-10,'CBFXY:Selftest','Torque entered before order four');
assert(affineDiff<1e-10,'CBFXY:Selftest','Fourth derivative not affine');
report=struct('physicalRhsDifference',firstDiff, ...
    'lowerOrderTorqueDifference',lowerDiff, ...
    'fourthOrderAffineDifference',affineDiff, ...
    'modelParameters',P,'passed',true);
fprintf('[CBF XY SELFTEST] model=%.3g, lowerOrder=%.3g, affine=%.3g : PASS\n', ...
    firstDiff,lowerDiff,affineDiff);
end
