function T=AF7R58R89_SELFTEST()
% Independent source-bound reference-only safety gate regressions.
g=9.81;L=2;lim=3.10559006211;
% Linear translation is already certified on the unsplit interval.
P=repmat([0;0;0],1,16);P(2,:)=linspace(0,1,16);
C=mk(P,1);
S=AF7R58R89_LOCAL_KINEMATIC_CERT(C,g,L,lim,512);
assert(S.certified && S.uav_speed_upper_m_s<lim, ...
    'R89:LinearCertified');
% Acceleration > gravity in horizontal direction makes old g-||a||
% negative, but the true specific force [a,0,g] is strictly positive.
A=11;h=0.1;P=zeros(3,16);
for j=0:15
    P(1,j+1)=0.5*A*h^2*(j*(j-1))/(15*14);
end
Q=AF7R58R89_LOCAL_KINEMATIC_CERT(mk(P,h),g,L,lim,512);
assert(Q.certified && Q.specific_force_lower_m_s2>0 && ...
    Q.end_vQ_m_s<lim,'R89:TiltedForceCertified');
Old=AF7R58R79_CONTINUOUS_KINEMATIC_CERT(mk(P,h),g,L,lim);
assert(~Old.certified,'R89:OldGlobalBoundExpectedToRejectTiltedCase');
P=zeros(3,16);P(1,:)=linspace(0,5,16);
F=AF7R58R89_LOCAL_KINEMATIC_CERT(mk(P,1),g,L,lim,512);
assert(~F.certified && ...
    strcmp(F.status,'NECESSARY_START_SPEED_LIMIT_VIOLATED'), ...
    'R89:RejectRealEndpointSpeed');
P=zeros(3,16);
for j=0:15
    P(3,j+1)=-0.5*g*h^2*(j*(j-1))/(15*14);
end
G=AF7R58R89_LOCAL_KINEMATIC_CERT(mk(P,h),g,L,lim,512);
assert(~G.certified,'R89:RejectZeroSpecificForce');
P=zeros(3,16);P(1,8)=4;P(1,9)=4;
Q2=AF7R58R89_LOCAL_KINEMATIC_CERT(mk(P,1),g,L,lim,1);
assert(~Q2.certified && ...
    strcmp(Q2.status,'INCONCLUSIVE_WORK_BUDGET'), ...
    'R89:NoWorkBudgetBypass');
T=struct('status','PASS_R89_LOCAL_KINEMATIC_AND_FAIL_CLOSED', ...
    'linear_certified',S.certified, ...
    'positive_tilted_force_certified',Q.certified, ...
    'old_global_g_minus_norm_a_fails_tilted_case',~Old.certified, ...
    'high_speed_endpoint_rejected',~F.certified, ...
    'zero_specific_force_rejected',~G.certified, ...
    'strict_work_budget_rejected',~Q2.certified, ...
    'continuous_closed_loop_safety_proved',false, ...
    'production_trigger_changed',false);
end
function C=mk(P,h)
C=struct('payload_cp',P,'t0',0,'t1',h,'duration_s',h);
end
