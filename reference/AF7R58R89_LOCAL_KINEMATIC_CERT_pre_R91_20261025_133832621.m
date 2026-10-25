function K=AF7R58R89_LOCAL_KINEMATIC_CERT(C,g,L,vLimit,maxNodes)
% R89: Conservative, piecewise Bernstein velocity certificate for a taut
% cable flatness reference. Work-limited; does NOT cap physical time.
% No sampled trajectory acceptance; every accepted leaf encloses ALL t.
% Unlike g-max||a||, supports positive specific force of arbitrary tilt.
% Reference-only. No HLC tracking/actuator/tension/unknown-space proof.
assert(isstruct(C)&&isfield(C,'payload_cp')&&...
    isequal(size(C.payload_cp),[3 16])&&...
    all(isfinite(C.payload_cp(:)))&&...
    isfield(C,'duration_s')&&isfinite(C.duration_s)&&C.duration_s>0, ...
    'R89:FiniteC7BezierRequired');
assert(all(isfinite([g L vLimit maxNodes]))&&...
    g>0&&L>0&&vLimit>0&&maxNodes>=1&&maxNodes==fix(maxNodes), ...
    'R89:LivePhysicsAndWorkBudgetRequired');
P=double(C.payload_cp);h=double(C.duration_s);
assert(isfield(C,'t0')&&isfinite(C.t0)&&...
    isfield(C,'t1')&&isfinite(C.t1)&&C.t1>C.t0 &&...
    abs((C.t1-C.t0)-h)<=64*eps(max([1 h abs(C.t1)])), ...
    'R89:InconsistentPhysicalTime');
K=struct('certified',false,'continuous_time',false, ...
    'status','INCONCLUSIVE_NOT_RUN','nodes',0,'safe_leaves',0, ...
    'start_vQ_m_s',NaN,'end_vQ_m_s',NaN, ...
    'uncertified_interval_s',[NaN NaN], ...
    'velocity_upper_m_s',NaN,'accel_upper_m_s2',NaN, ...
    'jerk_upper_m_s3',NaN,'specific_force_lower_m_s2',NaN, ...
    'cable_qdot_upper_rad_s',NaN,'uav_speed_upper_m_s',NaN, ...
    'finite_sensing_limit_m_s',vLimit, ...
    'reference_only',true,'production_commit_authorized',false, ...
    'limitations','No rigorous floating-point outward rounding; no actuator/tension/tube proof');
[K.start_vQ_m_s,ok0]=endpoint_speed(P(:,1), ...
    15/h*(P(:,2)-P(:,1)), ...
    15*14/h^2*(P(:,3)-2*P(:,2)+P(:,1)), ...
    15*14*13/h^3*(P(:,4)-3*P(:,3)+3*P(:,2)-P(:,1)),g,L);
[K.end_vQ_m_s,ok1]=endpoint_speed(P(:,16), ...
    15/h*(P(:,16)-P(:,15)), ...
    15*14/h^2*(P(:,16)-2*P(:,15)+P(:,14)), ...
    15*14*13/h^3*(P(:,16)-3*P(:,15)+3*P(:,14)-P(:,13)),g,L);
if ~ok0||~ok1
    K.status='INCONCLUSIVE_DEGENERATE_ENDPOINT_SPECIFIC_FORCE';return;
end
% Necessary endpoint condition, NOT a sampled sufficiency test.
slack=128*eps(max([1,vLimit,K.start_vQ_m_s,K.end_vQ_m_s]));
if K.start_vQ_m_s>=vLimit-slack
    K.status='NECESSARY_START_SPEED_LIMIT_VIOLATED';
    K.uncertified_interval_s=[C.t0 C.t0];return;
end
if K.end_vQ_m_s>=vLimit-slack
    K.status='NECESSARY_END_SPEED_LIMIT_VIOLATED';
    K.uncertified_interval_s=[C.t1 C.t1];return;
end
stack=struct('cp',P,'a',double(C.t0),'b',double(C.t1));
maxV=0;maxA=0;maxJ=0;maxQdot=0;maxVQ=0;minSigma=Inf;
while ~isempty(stack)
    if K.nodes>=maxNodes
        K.status='INCONCLUSIVE_WORK_BUDGET';
        K.uncertified_interval_s=[stack(end).a stack(end).b];return;
    end
    Z=stack(end);stack(end)=[];K.nodes=K.nodes+1;
    [pass,V,A,J,sigma,qdot,vQ]=local_bound(Z.cp,Z.b-Z.a,g,L,vLimit);
    if pass
        K.safe_leaves=K.safe_leaves+1;
        maxV=max(maxV,V);maxA=max(maxA,A);maxJ=max(maxJ,J);
        maxQdot=max(maxQdot,qdot);maxVQ=max(maxVQ,vQ);
        minSigma=min(minSigma,sigma);
        continue;
    end
    mid=Z.a+0.5*(Z.b-Z.a);
    if ~(isfinite(mid)&&mid>Z.a&&mid<Z.b)
        K.status='INCONCLUSIVE_FLOAT_TIME_RESOLUTION';
        K.uncertified_interval_s=[Z.a Z.b];return;
    end
    [left,right]=split_half(Z.cp);
    stack(end+1)=struct('cp',right,'a',mid,'b',Z.b); %#ok<AGROW>
    stack(end+1)=struct('cp',left,'a',Z.a,'b',mid); %#ok<AGROW>
end
K.certified=true;K.continuous_time=true;
K.status='CERTIFIED_LOCAL_BERNSTEIN_REFERENCE_KINEMATICS';
K.velocity_upper_m_s=maxV;K.accel_upper_m_s2=maxA;
K.jerk_upper_m_s3=maxJ;K.specific_force_lower_m_s2=minSigma;
K.cable_qdot_upper_rad_s=maxQdot;K.uav_speed_upper_m_s=maxVQ;
end

function [pass,V,A,J,sigma,qdot,vQ]=local_bound(P,h,g,L,vLimit)
pass=false;V=Inf;A=Inf;J=Inf;sigma=NaN;qdot=Inf;vQ=Inf;
if ~(isfinite(h)&&h>0),return;end
Vcp=15/h*diff(P,1,2);
Acp=14/h*diff(Vcp,1,2);
Jcp=13/h*diff(Acp,1,2);
if any(~isfinite([Vcp(:);Acp(:);Jcp(:)])),return;end
V=max(vecnorm(Vcp,2,1));A=max(vecnorm(Acp,2,1));
J=max(vecnorm(Jcp,2,1));
F=Acp+[zeros(2,size(Acp,2));g*ones(1,size(Acp,2))];
n=mean(F,2);nn=norm(n);
if ~(isfinite(nn)&&nn>0),return;end
n=n/nn;
% Convex hull: n'F(t)>=min_i n'F_i and ||F(t)||>=n'F(t).
sigma=min(n'*F);
sigmaSlack=128*eps(max([1,g,max(vecnorm(F,2,1))]));
if ~(isfinite(sigma)&&sigma>sigmaSlack),return;end
% qdot=(I-qq')*jerk/||specific_force||, hence ||qdot||<=J/sigma.
qdot=J/sigma;
vQ=V+L*qdot;
speedSlack=128*eps(max([1,vLimit,vQ]));
pass=isfinite(vQ)&&vQ<vLimit-speedSlack;
end

function [vQ,ok]=endpoint_speed(~,v,a,j,g,L)
f=a+[0;0;g];nf=norm(f);vQ=Inf;ok=false;
if ~(isfinite(nf)&&nf>128*eps(max([1,g,norm(a)]))),return;end
q=f/nf;
qd=(j-q*(q'*j))/nf;
vQ=norm(v+L*qd);ok=isfinite(vQ);
end

function [A,B]=split_half(P)
N=size(P,2);A=zeros(size(P));B=zeros(size(P));
A(:,1)=P(:,1);B(:,end)=P(:,end);
for k=2:N
    P=(P(:,1:end-1)+P(:,2:end))/2;
    A(:,k)=P(:,1);B(:,N-k+1)=P(:,end);
end
end
