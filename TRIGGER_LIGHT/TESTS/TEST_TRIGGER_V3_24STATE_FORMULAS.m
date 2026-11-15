function R=TEST_TRIGGER_V3_24STATE_FORMULAS(n)
%TEST_TRIGGER_V3_24STATE_FORMULAS Original 24-state RHS vs physical identities.
% Separate exact-source check, NOT proof of real experimental robustness.
% Compare the source with ideal taut-cable equations only on the source's
% defined kinematic manifold, with estimated mass as model parameter.
if nargin<1,n=300;end
R=struct('status','NOT_RUN','cases',0,'maxRhsResidual',NaN, ...
    'physicalTruthUsed',false);
need={'with_load_model_euler_for_HL','Eul2Quat','RodriguesQuaternion'};
for k=1:numel(need)
    if isempty(which(need{k}))
        R.status='SKIPPED_ORIGINAL_SOURCE_NOT_ON_PATH';
        return
    end
end
rng(33);
g=9.81;m=.762;mL=.14;L=2;
p=zeros(1,21);p([1,6,7,8,9,20,21])=[m,.022,.03,.048,g,mL,L];
maxResidual=0;
for k=1:n
    x=zeros(24,1);
    x(4:6)=[.8*(rand-.5);.8*(rand-.5);2*(rand-.5)];
    x(7:9)=randn(3,1)*.2;
    x(10:12)=randn(3,1)*.5;
    q=randn(3,1);q=q/norm(q);x(19:21)=q;
    w=randn(3,1)*.6;w=w-q*dot(w,q); % valid taut angular velocity
    x(22:24)=w;
    x(1:3)=[0;0;3];x(13:15)=x(1:3)+L*q;
    x(16:18)=x(7:9)+L*cross(x(22:24),q);
    u=[rand*20;randn(3,1)*.2];
    src=with_load_model_euler_for_HL(x,u,p);
    bRot=RodriguesQuaternion(Eul2Quat(x(4:6)));
    b=bRot(:,3);
    w=x(22:24);
    qdot=cross(w,q);
    T=(m*mL/(m+mL))*L*dot(qdot,qdot) - (mL/(m+mL))*u(1)*dot(q,b);
    aD=-g*[0;0;1]+u(1)/m*b+(T/m)*q;
    aL=-g*[0;0;1]-(T/mL)*q;
    wdot=-(u(1)/(m*L))*cross(q,b);
    residual=max([norm(src(7:9)-aD), norm(src(16:18)-aL), ...
        norm(src(19:21)-qdot),norm(src(22:24)-wdot)]);
    maxResidual=max(maxResidual,residual);
    assert(residual<1e-8, ...
        'Source RHS disagrees with nominal taut equations: residual %g',residual);
end
R.status='PASS_ORIGINAL_RHS_TAUT_IDENTITIES__ESTIMATED_MASS_MODEL';
R.cases=n;R.maxRhsResidual=maxResidual;
end
