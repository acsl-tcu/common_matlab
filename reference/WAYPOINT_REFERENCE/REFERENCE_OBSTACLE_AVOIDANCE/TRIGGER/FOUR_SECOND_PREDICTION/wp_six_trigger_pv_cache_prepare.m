function K=wp_six_trigger_pv_cache_prepare(gains,pastRelativeTimes,filterTime,velocityWeight)
%WP_SIX_TRIGGER_PV_CACHE_PREPARE Precompute all history-fit matrices OFF tick.
% Original HLC F1/F2/F3 => 2nd/6th order EXACT ideal error modes.
% Current EKF pL and vL histories are inputs; weight has units of seconds
% and is the assumed POSITION/VELOCITY measurement std ratio (NOT a
% collision radius, NOT a physical error bound). Caller supplies it.
% Do not rebuild this for each 25-ms tick. Refresh on gain/T/history changes.
if nargin~=4 || ~isscalar(velocityWeight) || ~isfinite(velocityWeight) || velocityWeight<=0
    error('LightTrigger:PVWeight','Provide positive estimator position/velocity noise ratio (seconds)');
end
t=reshape(pastRelativeTimes,1,[]);
if numel(t)<8 || any(~isfinite(t)) || any(diff(t)<=0) || abs(t(end))>1e-9
    error('LightTrigger:PVHistory','Need ordered past times ending at 0');
end
A6=diag(ones(5,1),1);
Ax=A6;Ax(6,:)=-reshape(gains.F2,1,[]);
Ay=A6;Ay(6,:)=-reshape(gains.F3,1,[]);
Az=[0 1;-reshape(gains.F1,1,[])];
As={Ax,Ay,Az};
K=struct();K.W=cell(1,3);K.Phi=cell(1,3);K.A2=cell(1,3);
% Reduced least-squares maps with EKF e(0), e_dot(0) fixed exactly.
% Also precompute the x/y map with e_ddot(0) fixed by EKF cable
% direction, if the taut-cable condition is feasible at this tick.
K.BfixedPV=cell(1,3);K.WfreePV=cell(1,3);
K.BfixedPVA=cell(1,3);K.WfreePVA=cell(1,3);
K.Bfull=cell(1,3);
K.reducedConditionPV=nan(1,3);
K.reducedConditionPVA=nan(1,3);
K.condition=zeros(1,3);
for j=1:3
    A=As{j};n=size(A,1);m=numel(t);
    HM=zeros(m,n);HV=zeros(m,n);
    for k=1:m
        X=expm(A*t(k));
        HM(k,:)=X(1,:);HV(k,:)=X(2,:);
    end
    B=[HM;velocityWeight*HV];
    % This solve happens ONCE at setup, never inside the 25ms call.
    K.W{j}=B\eye(2*m); % retained for compatible offline diagnostics
    K.Bfull{j}=B;
    K.BfixedPV{j}=B(:,1:2);
    if n>2
        U=B(:,3:end);
        K.WfreePV{j}=U\eye(2*m);
        K.reducedConditionPV(j)=cond(U);
    else
        K.WfreePV{j}=zeros(0,2*m); % vertical 2-state: exact EKF p/v
        K.reducedConditionPV(j)=1;
    end
    if n>3
        K.BfixedPVA{j}=B(:,1:3);
        U=B(:,4:end);
        K.WfreePVA{j}=U\eye(2*m);
        K.reducedConditionPVA(j)=cond(U);
    else
        K.BfixedPVA{j}=zeros(2*m,0);
        K.WfreePVA{j}=zeros(0,2*m);
        K.reducedConditionPVA(j)=NaN;
    end
    K.condition(j)=cond(B);
    K.Phi{j}=expm(A*.025);
    K.A2{j}=A*A;
end
K.pastRelativeTimes=t;
K.velocityWeight=velocityWeight;
K.gainsF1=gains.F1;K.gainsF2=gains.F2;K.gainsF3=gains.F3;
K.governor=wp_six_trigger_governor_fast_coeff(.025,filterTime);
K.samplePeriod_s=.025;
K.precomputedOutsideTick=true;
K.initialPVExactlyAnchoredToCurrentEKF=true;
K.cableDirectionCanBeDynamicallyAnchored=true;
K.modelIsConditionalOnUnSaturatedOriginalHLC=true;
K.physicalPredictionErrorCertified=false;
end
