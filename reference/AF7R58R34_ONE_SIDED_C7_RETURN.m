function R = AF7R58R34_ONE_SIDED_C7_RETURN(t0,T,J0,JT,L,g,direction)
% AF7R58R34_ONE_SIDED_C7_RETURN
% Offline constrained candidate for a degree-15, two-segment C7 correction.
% Compare to unconstrained R33. The Bernstein control-point inequalities
% prove absence of reversal in ONE specified direction on BOTH segments,
% for the mathematical piecewise Bezier polynomial defined by the stored
% binary64 control points. This does NOT certify speed, yaw, obstacle or
% whole-body safety, flight feasibility, or a return to a nominal timeline.
%
% Inputs are external. No defaults: t0,T, J0,JT (3x8 physical jets),
% cable L, g, and explicit nonzero directional vector.
% A missing Optimization Toolbox is reported, never replaced by an
% unverified numerical approximation or a hidden fallback optimizer.
validateattributes(t0,{'double'},{'real','finite','scalar'});
validateattributes(T,{'double'},{'real','finite','scalar','positive'});
validateattributes(J0,{'double'},{'real','finite','size',[3 8]});
validateattributes(JT,{'double'},{'real','finite','size',[3 8]});
validateattributes(L,{'double'},{'real','finite','scalar','positive'});
validateattributes(g,{'double'},{'real','finite','scalar','positive'});
validateattributes(direction,{'double'},{'real','finite','numel',3});
n=direction(:);
assert(norm(n)>0,'Specify a nonzero physical direction.');
n=n/norm(n);
C0=AF7R58R33_C7_TWOSEG_SEED(t0,T,J0,JT,L,g);
B0=[C0.first.control_points.';C0.second.control_points.'];
min0=min(B0*n);
R=struct('schema','AF7R58R34_ONE_SIDED_C7_COMPARE_V1', ...
  'status','UNSOLVED','direction',n, ...
  'unconstrained_min_signed_control_m',min0, ...
  'unconstrained_algebraic_one_sided_proven',min0>=0, ...
  'constrained_algebraic_one_sided_proven',false, ...
  'c7_endpoint_jet_construction',true, ...
  'wholebody_geometry_certified',false,'speed_certified',false, ...
  'yaw_certified',false,'committable',false, ...
  'candidate',struct(),'integral_cost',NaN, ...
  'unconstrained_integral_cost',C0.integral_cost, ...
  'continuous_shape_certificate_is_not_flight_safety',true);
if exist('quadprog','file')~=2
    R.status='QUADPROG_NOT_AVAILABLE';
    return;
end
% This algebra is exact in real arithmetic. Segment durations and knots
% are copied from the actual R33 candidate, not recomputed timestamps.
h1=C0.first.duration;
h2=C0.second.duration;
tau=sqrt(L/g);
Z=zeros(3,8);
A=zeros(32,8);
for r=0:7
    E=zeros(3,8);
    E(:,r+1)=n/(tau^r);
    F=AF7R58R32_C7_JET_SEED(C0.first.t_start,h1,Z,E);
    G=AF7R58R32_C7_JET_SEED(C0.second.t_start,h2,E,Z);
    A(:,r+1)=[F.control_points.'*n;G.control_points.'*n];
end
x0=zeros(8,1);
for r=0:7
    x0(r+1)=tau^r * (n.'*C0.mid_jets(:,r+1));
end
signed0=B0*n;
b=signed0-A*x0;
Q=blkdiag(local_gram_cost(h1,tau),local_gram_cost(h2,tau));
H=A.'*Q*A;
H=(H+H.')/2;
f=A.'*Q*b;
if ~(all(isfinite(H),'all') && all(isfinite(f)) && ...
        all(isfinite(A),'all') && all(isfinite(b)))
    R.status='NONFINITE_QP';
    return;
end
options=optimoptions('quadprog','Display','off');
try
    [x,~,exitflag,output]=quadprog(2*H,2*f,-A,b,[],[],[],[],x0,options);
catch ME
    R.status='CONSTRAINED_QP_EXCEPTION';
    R.error_identifier=ME.identifier;
    R.error_message=ME.message;
    return;
end
R.qp_exitflag=exitflag;
R.qp_output=output;
if isempty(x) || exitflag<=0
    R.status='NO_CONSTRAINED_QP_SOLUTION';
    return;
end
Jm=C0.mid_jets;
for r=0:7
    Jm(:,r+1)=Jm(:,r+1)+n*((x(r+1)-x0(r+1))/tau^r);
end
C1=AF7R58R32_C7_JET_SEED(C0.first.t_start,h1,J0,Jm);
C2=AF7R58R32_C7_JET_SEED(C0.second.t_start,h2,Jm,JT);
B=[C1.control_points.';C2.control_points.'];
minimum=min(B*n);
R.minimum_signed_control_m=minimum;
R.qp_constraint_min_m=min(A*x+b);
R.constrained_algebraic_one_sided_proven=(minimum>=0);
R.joint_time_exact=(C1.t_end==C2.t_start);
R.status='C7_CANDIDATE_FAILED_STRICT_SHAPE_CERTIFICATE';
if ~R.joint_time_exact
    R.status='NONREPRESENTABLE_JOINT_KNOT';
    return;
end
if ~R.constrained_algebraic_one_sided_proven
    return;
end
R.integral_cost=sum(sum(B.*(Q*B)));
R.candidate=struct('first',C1,'second',C2,'mid_jets',Jm, ...
    'direction',n,'t_start',C1.t_start,'t_mid',C1.t_end, ...
    't_end',C2.t_end,'duration',C2.t_end-C1.t_start, ...
    'minimum_signed_control_m',minimum, ...
    'c7_algebraic_structure',true, ...
    'wholebody_geometry_certified',false,'yaw_certified',false, ...
    'speed_certified',false,'committable',false);
R.status='C7_ONE_SIDED_SHAPE_CANDIDATE_ONLY';
end

function Q = local_gram_cost(T,tau)
n=15;
w=[tau^(-2),1,tau^2,tau^4];
Q=zeros(n+1);
for r=0:3
    if r==0
        D=eye(n+1);
    else
        D=diff(eye(n+1),r,1);
    end
    m=n-r;
    G=zeros(m+1);
    for i=0:m
        for j=0:m
            G(i+1,j+1)=nchoosek(m,i)*nchoosek(m,j)/ ...
                ((2*m+1)*nchoosek(2*m,i+j));
        end
    end
    falling=prod(n-(0:r-1));
    Q=Q+w(r+1)*T*(falling/T^r)^2*(D.'*G*D);
end
Q=(Q+Q.')/2;
end
