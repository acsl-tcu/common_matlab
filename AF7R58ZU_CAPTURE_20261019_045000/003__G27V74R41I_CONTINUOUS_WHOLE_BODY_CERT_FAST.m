function C = G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST(CTX,P)
% G27V74R23_CONTINUOUS_WHOLE_BODY_CERT
% =========================================================================
% Continuous-time whole-body geometry SHADOW certificate.
%
% R23 improvement over R22
% ------------------------
% R22 enclosed q(t) with
%
%       ||q(t)-q_mid|| <= qdot_UB * leaf_half_width,
%
% which is rigorous but extremely conservative when the degree-15 return
% segment has large jerk.  In the observed case this produced qdot_UB~1008
% rad/s and an unusably large UAV/cable inflation.
%
% In the supported NOMINAL theorem class, nominal acceleration is exactly
% zero and the candidate correction is one fixed spatial direction w.
% Therefore on every leaf
%
%       A(t) = pLdd(t)+g e3 = g e3 + w*s2(t),
%
% where s2(t) is scalar.  The Bernstein derivative hull gives the exact
% interval enclosure s2(t) in [slo,shi].
%
% Because A(s) moves on a straight line that does not cross the origin when
% alpha_lb>0, q(s)=-A(s)/||A(s)|| moves monotonically on one great-circle
% arc between q(slo) and q(shi).  We therefore enclose the ENTIRE q arc
% directly by the ball
%
%       q(t) in B(q_c, r_q),
%
% where q_c is the normalized angular bisector of the endpoint directions
% and r_q is the endpoint chord distance to q_c.  No qdot*time enclosure is
% used for geometry.
%
% This gives:
%
%   payload union subset conv(P_L)
%   UAV union     subset conv(P_L - L*q_c) (+) B(L*r_q)
%   cable union   subset conv([P_L, P_L-L*q_c]) (+) B(L*r_q)
%
% These sets are separated from every DETECTED obstacle's raw enclosing
% ellipsoid using its exact support function plus live component radius and
% live d_margin.
%
% Acceptance is continuous-time sufficient. Failure is inconclusive and
% triggers adaptive de Casteljau subdivision.
%
% Authority
% ---------
% Directed/outward IEEE754 rounding is still OPEN:
%   mathematical_pass may be true
%   certified is ALWAYS false
%   ieee754_outward_closed is false
% =========================================================================

C=g27_empty_cert();

if ~isstruct(CTX) || ~isfield(CTX,'request') || ...
        ~isfield(CTX,'obstacles') || ~isfield(CTX,'live')
    C.reason_code=-100;
    return;
end
if ~isstruct(P) || ~isfield(P,'generated') || ~logical(P.generated)
    C.reason_code=-101;
    return;
end

R=CTX.request;
O=CTX.obstacles;
LIV=CTX.live;

% Fail closed outside the exact supported ACTIVE theorem.
if ~strcmpi(char(R.active_reference_mode),'NOMINAL')
    C.reason_code=1;
    return;
end
if ~logical(LIV.active_speed_certified) || ...
        ~logical(LIV.active_nominal_acceleration_zero)
    C.reason_code=2;
    return;
end

v=double(LIV.active_payload_velocity_vector_m_s(:));
if numel(v)~=3 || any(~isfinite(v))
    C.reason_code=3;
    return;
end
if abs(norm(v)-double(LIV.active_payload_speed_ub_m_s)) > ...
        256*eps(max(1,double(LIV.active_payload_speed_ub_m_s)))
    C.reason_code=4;
    return;
end
if ~isfield(O,'base_axes') || size(O.base_axes,2)~=double(O.count)
    C.reason_code=5;
    return;
end

t0=double(P.t0);
tm=double(P.tm);
t1=double(P.t1);
w=double(P.displacement(:));

if numel(w)~=3 || any(~isfinite(w)) || ...
        ~(isfinite(t0)&&isfinite(tm)&&isfinite(t1)&&t0<tm&&tm<t1)
    C.reason_code=6;
    return;
end

p0=double(R.xd_active(1:3));
if numel(p0)~=3 || any(~isfinite(p0))
    C.reason_code=7;
    return;
end

cup=[zeros(1,8),ones(1,8)];
cdn=[ones(1,8),zeros(1,8)];

Pup=g27_payload_parent_ctrl(p0,v,t0,t0,tm,w,cup);
Pdn=g27_payload_parent_ctrl(p0,v,t0,tm,t1,w,cdn);

MAX_DEPTH=14;
MAX_LEAVES=16384;

S=repmat(struct('P',zeros(3,16),'c',zeros(1,16), ...
    'ta',NaN,'tb',NaN,'depth',0),2*MAX_LEAVES,1);
top=2;
S(1)=struct('P',Pup,'c',cup,'ta',t0,'tb',tm,'depth',0);
S(2)=struct('P',Pdn,'c',cdn,'ta',tm,'tb',t1,'depth',0);

leafCount=0;
acceptedLeaves=0;
splitCount=0;
maxDepthUsed=0;
resourceExhausted=false;

% Certified accepted-leaf minima.
certPayload=inf;
certUav=inf;
certCable=inf;

% Attempted-leaf diagnostics (can be negative on coarse rejected leaves).
attemptPayload=inf;
attemptUav=inf;
attemptCable=inf;

minAlpha=inf;
maxQdot=0;              % diagnostic only, NOT used by geometry
maxQArc=0;              % accepted leaf q-ball radius on S^2
maxQArcInflation=0;     % L*maxQArc [m]
maxAttemptQArc=0;

g=double(LIV.gravity);
Lc=double(LIV.cable_length);
rhoL=double(LIV.r_load)+double(LIV.d_margin);
rhoQ0=double(LIV.r_drone)+double(LIV.d_margin);
rhoC0=double(LIV.r_cable)+double(LIV.d_margin);

if ~(isfinite(g)&&g>0&&isfinite(Lc)&&Lc>0&& ...
        rhoL>=0&&rhoQ0>=0&&rhoC0>=0)
    C.reason_code=9;
    return;
end

while top>0
    X=S(top);
    top=top-1;

    leafCount=leafCount+1;
    maxDepthUsed=max(maxDepthUsed,X.depth);

    T=X.tb-X.ta;
    if ~(T>0)
        C.reason_code=10;
        return;
    end

    c2=g27_derivative_ctrl_scalar(X.c,T,2);
    c3=g27_derivative_ctrl_scalar(X.c,T,3);

    slo=min(c2);
    shi=max(c2);

    [qcenter,qrad,alphaLB,qarcAngle,qok]= ...
        g27_q_arc_enclosure(w,g,slo,shi); %#ok<ASGLU>

    minAlpha=min(minAlpha,alphaLB);

    % Retain qdot only as a diagnostic for later dynamics work.
    if isfinite(alphaLB) && alphaLB>0
        jerkUB=norm(w)*max(abs(c3));
        qdotUB=jerkUB/alphaLB;
        maxQdot=max(maxQdot,qdotUB);
    end

    leafPass=false;

    if qok
        maxAttemptQArc=max(maxAttemptQArc,qrad);

        PQ=X.P-Lc*qcenter*ones(1,size(X.P,2));
        PC=[X.P,PQ];

        leafPayload=inf;
        leafUav=inf;
        leafCable=inf;

        leafPass=true;

        for io=1:double(O.count)
            co=double(O.centers(:,io));
            Ro=reshape(double(O.rotations_flat(:,io)),3,3);
            ao=double(O.base_axes(:,io));

            mL=g27_best_support_margin(X.P,co,Ro,ao,rhoL);
            mQ=g27_best_support_margin(PQ,co,Ro,ao,rhoQ0+Lc*qrad);
            mC=g27_best_support_margin(PC,co,Ro,ao,rhoC0+Lc*qrad);

            leafPayload=min(leafPayload,mL);
            leafUav=min(leafUav,mQ);
            leafCable=min(leafCable,mC);

            if ~(mL>0 && mQ>0 && mC>0)
                leafPass=false;
                break;
            end
        end

        attemptPayload=min(attemptPayload,leafPayload);
        attemptUav=min(attemptUav,leafUav);
        attemptCable=min(attemptCable,leafCable);

        if leafPass
            acceptedLeaves=acceptedLeaves+1;
            certPayload=min(certPayload,leafPayload);
            certUav=min(certUav,leafUav);
            certCable=min(certCable,leafCable);
            maxQArc=max(maxQArc,qrad);
            maxQArcInflation=max(maxQArcInflation,Lc*qrad);
        end
    end

    if leafPass
        continue;
    end

    if X.depth>=MAX_DEPTH || leafCount>=MAX_LEAVES || top+2>numel(S)
        resourceExhausted=true;
        C.reason_code=20;
        break;
    end

    [PL,PR]=g27_split_matrix(X.P,0.5);
    [cL,cR]=g27_split_scalar(X.c,0.5);
    mid=0.5*(X.ta+X.tb);

    splitCount=splitCount+1;

    top=top+1;
    S(top)=struct('P',PR,'c',cR,'ta',mid,'tb',X.tb,'depth',X.depth+1);
    top=top+1;
    S(top)=struct('P',PL,'c',cL,'ta',X.ta,'tb',mid,'depth',X.depth+1);
end

pass = ~resourceExhausted && acceptedLeaves>0 && ...
       isfinite(certPayload)&&isfinite(certUav)&&isfinite(certCable) && ...
       certPayload>0 && certUav>0 && certCable>0 && minAlpha>0;

C.mathematical_pass=logical(pass);
C.certified=false;
C.continuous_time=true;
C.whole_body=true;
C.detected_obstacles_only=true;
C.ieee754_outward_closed=false;
C.support_function_separation=true;
C.adaptive_decasteljau=true;
C.q_arc_enclosure=true;
C.qdot_used_for_geometry=false;
C.resource_exhausted=logical(resourceExhausted);

C.leaf_count=leafCount;
C.accepted_leaf_count=acceptedLeaves;
C.split_count=splitCount;
C.max_depth_used=maxDepthUsed;
C.max_depth_cap=MAX_DEPTH;
C.max_leaf_cap=MAX_LEAVES;

C.min_payload_margin_m=certPayload;
C.min_uav_margin_m=certUav;
C.min_cable_margin_m=certCable;
C.min_margin_m=min([certPayload,certUav,certCable]);

C.min_attempted_payload_margin_m=attemptPayload;
C.min_attempted_uav_margin_m=attemptUav;
C.min_attempted_cable_margin_m=attemptCable;
C.min_attempted_margin_m=min([attemptPayload,attemptUav,attemptCable]);

C.min_alpha_lb=minAlpha;
C.max_qdot_ub=maxQdot;
C.max_q_arc_radius=maxQArc;
C.max_q_arc_inflation_m=maxQArcInflation;
C.max_attempted_q_arc_radius=maxAttemptQArc;

C.nominal_velocity=v;
C.valid_from=t0;
C.valid_until=t1;

if pass
    C.reason_code=0;
elseif C.reason_code==0
    C.reason_code=21;
end

end


function C=g27_empty_cert()
C=struct( ...
    'mathematical_pass',false, ...
    'certified',false, ...
    'continuous_time',false, ...
    'whole_body',false, ...
    'detected_obstacles_only',true, ...
    'ieee754_outward_closed',false, ...
    'support_function_separation',false, ...
    'adaptive_decasteljau',false, ...
    'q_arc_enclosure',false, ...
    'qdot_used_for_geometry',false, ...
    'resource_exhausted',false, ...
    'reason_code',0, ...
    'leaf_count',0, ...
    'accepted_leaf_count',0, ...
    'split_count',0, ...
    'max_depth_used',0, ...
    'max_depth_cap',0, ...
    'max_leaf_cap',0, ...
    'min_payload_margin_m',NaN, ...
    'min_uav_margin_m',NaN, ...
    'min_cable_margin_m',NaN, ...
    'min_margin_m',NaN, ...
    'min_attempted_payload_margin_m',NaN, ...
    'min_attempted_uav_margin_m',NaN, ...
    'min_attempted_cable_margin_m',NaN, ...
    'min_attempted_margin_m',NaN, ...
    'min_alpha_lb',NaN, ...
    'max_qdot_ub',NaN, ...
    'max_q_arc_radius',NaN, ...
    'max_q_arc_inflation_m',NaN, ...
    'max_attempted_q_arc_radius',NaN, ...
    'nominal_velocity',nan(3,1), ...
    'valid_from',NaN, ...
    'valid_until',NaN);
end


function [qc,rad,alphaLB,angleSpan,ok]=g27_q_arc_enclosure(w,g,slo,shi)
% Entire q(s) arc for A(s)=g*e3+w*s, s in [slo,shi].
%
% A(s) lies on one affine line. If the line segment avoids the origin,
% normalized A(s) moves monotonically along the great-circle arc between
% endpoint directions. The angular-bisector ball therefore contains all q.

qc=[NaN;NaN;NaN];
rad=NaN;
angleSpan=NaN;
ok=false;

if slo>shi
    z=slo; slo=shi; shi=z;
end

alphaLB=g27_alpha_line_lower_bound(w,g,slo,shi);
if ~(isfinite(alphaLB)&&alphaLB>0)
    return;
end

A0=[0;0;g]+w*slo;
A1=[0;0;g]+w*shi;
n0=norm(A0);
n1=norm(A1);

if ~(isfinite(n0)&&isfinite(n1)&&n0>0&&n1>0)
    return;
end

q0=-A0/n0;
q1=-A1/n1;

dq=max(-1,min(1,dot(q0,q1)));
angleSpan=acos(dq);

if angleSpan<=64*eps
    qc=q0;
    rad=0;
    ok=true;
    return;
end

qs=q0+q1;
nqs=norm(qs);

% Near-antipodal endpoints: the current leaf is too wide; subdivide.
if ~(isfinite(nqs)&&nqs>1e-12)
    return;
end

qc=qs/nqs;
rad=max(norm(q0-qc),norm(q1-qc));

% Analytic equivalent = 2*sin(angleSpan/4). Keep a consistency guard.
radAnalytic=2*sin(0.25*angleSpan);
if abs(rad-radAnalytic)>1e-10*max(1,radAnalytic)
    return;
end

ok=isfinite(rad)&&rad>=0&&rad<2;
end


function P=g27_payload_parent_ctrl(p0,v,tref,ta,tb,w,c)
n=15;
T=tb-ta;
pa=p0+v*(ta-tref);
P=zeros(3,16);
for j=0:n
    P(:,j+1)=pa+v*(T*j/n)+w*c(j+1);
end
end


function D=g27_derivative_ctrl_scalar(c,T,r)
D=c;
for k=1:r
    D=(numel(D)-1)*diff(D)/T;
end
end


function lb=g27_alpha_line_lower_bound(w,g,slo,shi)
if slo>shi
    z=slo; slo=shi; shi=z;
end
a=dot(w,w);
if a<=0
    lb=abs(g);
    return;
end
b=g*w(3);
sstar=-b/a;
sstar=min(shi,max(slo,sstar));
A=[0;0;g]+w*sstar;
lb=norm(A);
end


function best=g27_best_support_margin(P,c,R,a,rho)
% R41I theorem-equivalent vectorized implementation.
% Same candidate-normal set and same support inequality as R23.

m=size(P,2);
N=zeros(3,m+7);
nN=0;

Rt=R.';
Pc=P-c;
Z=Rt*Pc;
NW=R*(Z./(a.^2));
nn=sqrt(sum(NW.^2,1));
good=isfinite(nn) & nn>0;
ig=find(good);
ng=numel(ig);
if ng>0
    N(:,1:ng)=NW(:,ig)./nn(ig);
    nN=ng;
end

d=mean(P,2)-c;
nd=norm(d);
if isfinite(nd) && nd>0
    nN=nN+1;
    N(:,nN)=d/nd;
end

for k=1:3
    nN=nN+1; N(:,nN)= R(:,k);
    nN=nN+1; N(:,nN)=-R(:,k);
end

N=N(:,1:nN);
RN=Rt*N;
h=(c.'*N) + sqrt(sum((a.*RN).^2,1)) + rho;
lower=min(N.'*P,[],2).';
best=max(lower-h);
end
function [L,R]=g27_split_matrix(P,u)
n=size(P,2)-1;
L=zeros(size(P));
R=zeros(size(P));
L(:,1)=P(:,1);
R(:,end)=P(:,end);
cur=P;
for r=1:n
    cur=(1-u)*cur(:,1:end-1)+u*cur(:,2:end);
    L(:,r+1)=cur(:,1);
    R(:,end-r)=cur(:,end);
end
end


function [L,R]=g27_split_scalar(c,u)
n=numel(c)-1;
L=zeros(size(c));
R=zeros(size(c));
L(1)=c(1);
R(end)=c(end);
cur=c;
for r=1:n
    cur=(1-u)*cur(1:end-1)+u*cur(2:end);
    L(r+1)=cur(1);
    R(end-r)=cur(end);
end
end
