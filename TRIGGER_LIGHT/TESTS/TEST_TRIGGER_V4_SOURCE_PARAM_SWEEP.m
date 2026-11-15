function R=TEST_TRIGGER_V4_SOURCE_PARAM_SWEEP(n)
%TEST_TRIGGER_V4_SOURCE_PARAM_SWEEP Original 24-state RHS identities for
% diverse model parameters, not only one hardcoded test mass.
% Synthetic mL here models a CURRENT estimator snapshot, never plant truth.
if nargin<1,n=400;end
R=struct('status','NOT_RUN','cases',0,'maxResidual',NaN, ...
    'estimatedMassUsedInEquations',true,'physicalPlantTruthUsed',false);
if isempty(which('with_load_model_euler_for_HL')) || ...
        isempty(which('RodriguesQuaternion')) || isempty(which('Eul2Quat'))
    R.status='SKIPPED_ORIGINAL_SOURCE_NOT_ON_PATH';return
end
rng(112206);
maxError=0;
for k=1:n
    m=.3+1.4*rand; mL=.025+.8*rand; L=.5+3*rand;
    g=9.7+.2*rand;
    param=zeros(1,21);
    param(1)=m;param(6:8)=[.012+.04*rand,.014+.04*rand,.02+.05*rand];
    param(9)=g;param(20)=mL;param(21)=L;
    x=zeros(24,1);
    x(4:6)=[.45*(2*rand-1);.45*(2*rand-1);2*(rand-.5)];
    x(7:9)=.7*randn(3,1);
    x(10:12)=randn(3,1);
    x(1:3)=[0;0;3];
    q=randn(3,1);q=q/norm(q);x(19:21)=q;
    w=randn(3,1);w=w-q*(q.'*w);x(22:24)=w;
    x(13:15)=x(1:3)+L*q;
    x(16:18)=x(7:9)+L*cross(w,q);
    u=[20*rand; .3*randn(3,1)];
    rhs=with_load_model_euler_for_HL(x,u,param);
    rot=RodriguesQuaternion(Eul2Quat(x(4:6)));
    b=rot(:,3);
    qDot=cross(w,q);
    tension=(m*mL/(m+mL))*L*(qDot.'*qDot) ...
        -(mL/(m+mL))*u(1)*(q.'*b);
    aDrone=-g*[0;0;1]+u(1)/m*b+(tension/m)*q;
    aLoad=-g*[0;0;1]-(tension/mL)*q;
    wDot=-u(1)/(m*L)*cross(q,b);
    err=max([norm(rhs(7:9)-aDrone), norm(rhs(16:18)-aLoad), ...
        norm(rhs(19:21)-qDot),norm(rhs(22:24)-wDot)]);
    assert(err<1e-8,'Original source equation disagreement, case %d',k);
    maxError=max(maxError,err);
end
R.status='PASS_24STATE_SOURCE_IDENTITIES_ACROSS_MODEL_PARAMETER_VARIATION';
R.cases=n;R.maxResidual=maxError;
fprintf('V4 source identity: %d varied-parameter cases, max residual %.5g\n',n,maxError);
end
