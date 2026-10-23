function R = AF7R58R33_C7_TWOSEG_SEED(t0,T,J0,JT,L,g)
% AF7R58R33: two-segment, degree-15 C7 correction candidate.
% Exact endpoint-jet algebra in real arithmetic. Floating point has finite
% residuals; this function DOES NOT certify obstacle safety or actual flight.
%
% J0/JT: 3x8 correction jets, orders 0,...,7, physical-time derivatives.
% T: positive candidate duration; there is NO hard max avoidance time.
% L/g: current externally supplied cable length and gravity (no fallbacks).
%
% The shared 3x8 interior jet is free and chosen by an unconstrained,
% strictly convex quadratic integral cost with natural time tau=sqrt(L/g).
% The midpoint T/2 is a representation knot, not an obstacle-specific timing
% restriction; this is ONE family to be followed by feasibility checks.
validateattributes(t0,{'double'},{'real','scalar','finite'});
validateattributes(T,{'double'},{'real','scalar','finite','positive'});
validateattributes(J0,{'double'},{'real','finite','size',[3 8]});
validateattributes(JT,{'double'},{'real','finite','size',[3 8]});
validateattributes(L,{'double'},{'real','finite','scalar','positive'});
validateattributes(g,{'double'},{'real','finite','scalar','positive'});
assert(isfinite(t0+T) && t0+T>t0, 'Unrepresentable candidate duration.');
tau=sqrt(L/g); T1=T/2; T2=T/2;
Z=zeros(3,8);
C10=AF7R58R32_C7_JET_SEED(t0,T1,J0,Z);
C20=AF7R58R32_C7_JET_SEED(t0+T1,T2,Z,JT);
B0=[C10.control_points.'; C20.control_points.']; % 32x3
A=zeros(32,8); % linear map from dimensionless interior jets
for r=0:7
    E=zeros(3,8); E(1,r+1)=tau^(-r);
    C1=AF7R58R32_C7_JET_SEED(t0,T1,Z,E);
    C2=AF7R58R32_C7_JET_SEED(t0+T1,T2,E,Z);
    A(:,r+1)=[C1.control_points(1,:).';C2.control_points(1,:).'];
end
Q1=local_gram_cost(T1,tau);
Q2=local_gram_cost(T2,tau);
Q=blkdiag(Q1,Q2);
H=A.'*Q*A; H=(H+H.')/2;
B=A.'*Q*B0;
condition_est=cond(H);
% Guard against unresolvable linear algebra. A failed numerical solve is
% NOT a failed physical feasibility proof and must NOT be committed.
assert(all(isfinite(H(:))) && all(isfinite(B(:))) && ...
       isfinite(condition_est) && condition_est*eps(1)<1, ...
       'C7 middle-jet QP not numerically resolvable at machine precision.');
[U,p]=chol(H,'upper');
assert(p==0,'C7 middle-jet Hessian not strictly positive definite numerically.');
X=-(U\(U.'\B)); % 8x3 dimensionless interior jets
Jm=zeros(3,8);
for r=0:7
    Jm(:,r+1)=X(r+1,:).'/tau^r;
end
C1=AF7R58R32_C7_JET_SEED(t0,T1,J0,Jm);
C2=AF7R58R32_C7_JET_SEED(t0+T1,T2,Jm,JT);
Bs=[C1.control_points.';C2.control_points.'];
Jcost=sum(sum(Bs.*(Q*Bs)));
Jzero=sum(sum(B0.*(Q*B0)));
assert(all(isfinite(Bs(:))) && isfinite(Jcost) && Jcost>=0, ...
    'C7 two-segment optimizer numerical failure.');
res=local_residual(C1.control_points,C2.control_points,T1,T2,J0,Jm,JT);
% Use the same stored floating-point knots as the actual segment models.
% For nonzero t0, (t0+T/2)+T/2 and t0+T can differ by one ulp.
assert(C1.t_end==C2.t_start && C2.t_end>C1.t_end, ...
   'Two-segment candidate has inconsistent or nonrepresentable knots.');

% One-sided projection certificate relative to initial correction direction.
% Bernstein convex hull gives a continuous-time sufficient certificate.
% This does NOT certify collision avoidance, speed, yaw, or payload dynamics.
c0=J0(:,1);
if norm(c0)>0
    unit=c0/norm(c0);
    signed_controls=Bs*unit;
    margin=min(signed_controls);
    % Stored floating-point controls are treated as exact binary reals for
    % this shape-only Bernstein convex-hull implication.
    no_reverse_proven=(margin>=0);
else
    unit=[NaN;NaN;NaN]; margin=NaN; no_reverse_proven=false;
end
R=struct('schema','AF7R58R33_TWOSEG_C7_SEED_V1', ...
   't_start',C1.t_start,'t_mid',C1.t_end,'t_end',C2.t_end, ...
   'duration',T,'cable_natural_time',tau, ...
   'first',C1,'second',C2,'mid_jets',Jm, ...
   'integral_cost',Jcost,'zero_mid_jet_cost',Jzero, ...
   'hessian_condition',condition_est, ...
   'max_endpoint_jet_residual',res, ...
   'initial_direction',unit,'minimum_control_projection',margin, ...
   'one_sided_projection_certified',no_reverse_proven, ...
   'c7_algebraic_structure',true, ...
   'duration_automatically_optimized',false, ...
   'obstacle_wholebody_certified',false,'speed_certified',false, ...
   'yaw_certified',false,'execution_certified',false, ...
   'committable',false,'stage','NONCOMMITTABLE_SEED');
end

function Q = local_gram_cost(T,tau)
n=15;
weights=[tau^(-2),1,tau^2,tau^4];
Q=zeros(n+1);
for r=0:3
    if r==0, D=eye(n+1);
    else, D=diff(eye(n+1),r,1);
    end
    m=n-r; G=zeros(m+1);
    for i=0:m
        for j=0:m
            G(i+1,j+1)=nchoosek(m,i)*nchoosek(m,j) / ...
                ((2*m+1)*nchoosek(2*m,i+j));
        end
    end
    falling=prod(n-(0:r-1));
    Q=Q+weights(r+1)*T*(falling/T^r)^2*(D.'*G*D);
end
Q=(Q+Q.')/2;
end

function m=local_residual(B1,B2,T1,T2,J0,Jm,JT)
m=0;
for r=0:7
    if r==0, A=B1; B=B2;
    else, A=diff(B1,r,2); B=diff(B2,r,2);
    end
    f=prod(15-(0:r-1));
    Jleft0=f/T1^r*A(:,1);
    JleftEnd=f/T1^r*A(:,end);
    Jright0=f/T2^r*B(:,1);
    JrightEnd=f/T2^r*B(:,end);
    m=max(m,max(abs([Jleft0-J0(:,r+1); JleftEnd-Jm(:,r+1); ...
       Jright0-Jm(:,r+1); JrightEnd-JT(:,r+1)])));
end
end
