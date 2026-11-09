function [A,b,R]=TRAJECTORY_C9_FORCE_WITNESS_PLANE(piece,authority,force_options,U,interval_s)
% TRAJECTORY_C9_FORCE_WITNESS_PLANE
% Local force/tension SEARCH planes for ONE joint 12-variable C9 QP.
% Given an unresolved continuous-time force-certificate leaf, derive the
% analytical (piecewise differentiable) gradient of THE SAME sufficient
% Bernstein thrust/tension bounds used by TRAJECTORY_C9_JOINT_FAST_CERTIFICATE:
%   s=||mean(W)|| - max_j||W_j-mean(W)||,
%   Q2=D2/s+3(D1/s)^2,
%   Fmax=(mU+mL)*(||mean(W)||+rho)+mU*L*Q2,
%   Fmin_raw=(mU+mL)*s-mU*L*Q2.
% W=g-pL'', D1=max||W'_j||, D2=max||W''_j||.
% The leaf is extracted EXACTLY by de Casteljau, not sampled in time.
% All local inequalities are converted to free-control DISPLACEMENT units
% by dividing both coefficients and deficit by the gradient norm. This
% needs no arbitrary geometric or force weighting. No obstacle-specific
% direction, duration, margin or physical bound is introduced.
%
% IMPORTANT: A'*x>=b is ONLY a local first-order search model. It is not a
% full-time certificate, and it NEVER authorizes HLC COMMIT. The caller
% must rerun the independent joint geometry/tension/thrust certificate.
% Ties/maxima use a deterministic active subgradient; their local linear
% models are not represented as conservative global inequalities.
A=zeros(12,0);b=zeros(0,1);
R=struct('status','UNKNOWN_NO_FORCE_PLANE','planes',0, ...
    'upper_bound_N',NaN,'lower_bound_N',NaN, ...
    'tension_lower_bound_N',NaN,'max_deficit_N',0, ...
    'immovable_violation',false, ...
    'zero_first_order_sensitivity',false, ...
    'force_gradient_upper_norm_N_per_m',NaN, ...
    'force_gradient_lower_norm_N_per_m',NaN, ...
    'force_gradient_upper_N_per_m',NaN(3,4), ...
    'force_gradient_lower_N_per_m',NaN(3,4), ...
    'interval_s',double(interval_s(:)).', ...
    'safe_to_commit',false,'scope','LOCAL_FORCE_SEARCH_ONLY__NOT_PROOF');
assert(isstruct(piece)&&isscalar(piece)&& ...
    isfield(piece,'control_points_m')&&isfield(piece,'duration_s')&& ...
    isfield(piece,'t0_s')&&isequal(size(piece.control_points_m),[3 24])&& ...
    all(isfinite(piece.control_points_m(:)))&& ...
    piece.duration_s>0&&isfinite(piece.duration_s)&& ...
    isstruct(authority)&&isscalar(authority)&& ...
    all(isfield(authority,{'cable_length_m','uav_mass_kg', ...
        'payload_mass_kg','gravity_world_mps2','thrust_limits_N', ...
        'minimum_tension_N','acceleration_guard_mps2'}))&& ...
    isstruct(force_options)&&isfield(force_options,'force_guard_N')&& ...
    isnumeric(U)&&isequal(size(U),[4 4])&&all(isfinite(U(:)))&& ...
    isreal(interval_s)&&numel(interval_s)==2&& ...
    all(isfinite(interval_s(:)))&& ...
    interval_s(2)>interval_s(1), ...
    'TRAJECTORY_C9_FORCE_PLANE:Input','Live authority and proof leaf required.');
t0=double(piece.t0_s);T=double(piece.duration_s);
u=(double(interval_s(:)).'-t0)/T;
% A numerical time comparison is merely protection against rounding on
% a certificate-returned dyadic leaf, NOT a reduction of proof scope.
assert(u(1)>=-64*eps(max(1,abs(t0/T)))&& ...
    u(2)<=1+64*eps(max(1,abs((t0+T)/T)))&& ...
    u(2)>u(1), ...
    'TRAJECTORY_C9_FORCE_PLANE:Leaf','Witness must lie inside piece.');
u=max(0,min(1,u));
dt=T*(u(2)-u(1));
P=extract_leaf(double(piece.control_points_m),u(1),u(2));
% Four scalar free-control indicator polynomials, restricted to THIS leaf.
E=zeros(4,24);E(:,11:14)=eye(4);
E=extract_leaf(E,u(1),u(2));
k2=23*22/dt^2;
k3=k2*21/dt;
k4=k3*20/dt;
g=double(authority.gravity_world_mps2(:));
W=repmat(g,1,22)-k2*diff(P,2,2);
W1=-k3*diff(P,3,2);
W2=-k4*diff(P,4,2);
DW=-k2*diff(E,2,2);
DW1=-k3*diff(E,3,2);
DW2=-k4*diff(E,4,2);
if ~all(isfinite([W(:);W1(:);W2(:);DW(:);DW1(:);DW2(:)]))
    R.status='UNKNOWN_NONFINITE_FORCE_LEAF';return
end
c=mean(W,2);nc=norm(c); dc=mean(DW,2);
[vR,jR]=max(sqrt(sum((W-c).^2,1)));
[v1,j1]=max(sqrt(sum(W1.^2,1)));
[v2,j2]=max(sqrt(sum(W2.^2,1)));
low=nc-vR;
mU=double(authority.uav_mass_kg);
mL=double(authority.payload_mass_kg);
L=double(authority.cable_length_m);
M=mU+mL;
minw=double(authority.minimum_tension_N)/mL+ ...
    double(authority.acceleration_guard_mps2);
R.tension_lower_bound_N=mL*low;
if ~(isfinite(low)&&nc>0)
    R.status='UNKNOWN_SINGULAR_FORCE_WITNESS';return
end
Gnc=grad_norm(c,nc,dc);
Grho=grad_norm(W(:,jR)-c,vR,DW(:,jR)-dc);
Glow=Gnc-Grho;
if low<=minw
    [A,b,R]=append_plane(A,b,R,Glow,minw-low,U, ...
        'UNKNOWN_TENSION_LINEARIZATION_IMMOVABLE');
    if R.immovable_violation,return;end
    if low<=0
        R.status='LOCAL_TENSION_WITNESS_ONLY__NOT_PROOF';return
    end
end
Gd1=grad_norm(W1(:,j1),v1,DW1(:,j1));
Gd2=grad_norm(W2(:,j2),v2,DW2(:,j2));
Q2=v2/low+3*(v1/low)^2;
GQ2=Gd2/low+6*v1*Gd1/(low^2)- ...
    (v2/(low^2)+6*v1^2/(low^3))*Glow;
fdelta=mU*L*Q2;
fu=M*(nc+vR)+fdelta;
flraw=M*low-fdelta;
Gu=M*(Gnc+Grho)+mU*L*GQ2;
Gl=M*Glow-mU*L*GQ2;
R.upper_bound_N=fu;
R.lower_bound_N=max(0,flraw);
R.force_gradient_upper_N_per_m=Gu;
R.force_gradient_lower_N_per_m=Gl;
R.force_gradient_upper_norm_N_per_m=norm(Gu,'fro');
R.force_gradient_lower_norm_N_per_m=norm(Gl,'fro');
flimits=double(authority.thrust_limits_N(:));
guard=double(force_options.force_guard_N);
if fu>flimits(2)-guard
    [A,b,R]=append_plane(A,b,R,-Gu,fu-(flimits(2)-guard),U, ...
        'UNKNOWN_UPPER_THRUST_LINEARIZATION_IMMOVABLE');
    if R.immovable_violation,return;end
end
% The actual certificate uses max(0,raw_lower), not raw_lower alone.
% If minimum permitted force is zero, max(0,.) ALREADY meets the lower
% constraint and creating a fictitious lower plane would be incorrect.
if max(0,flraw)<flimits(1)+guard
    [A,b,R]=append_plane(A,b,R,Gl, ...
        (flimits(1)+guard)-flraw,U, ...
        'UNKNOWN_LOWER_THRUST_LINEARIZATION_IMMOVABLE');
    if R.immovable_violation,return;end
end
if R.planes>0
    R.status='FORCE_WITNESS_PLANES_AVAILABLE__INDEPENDENT_PROOF_REQUIRED';
else
    R.status='UNKNOWN_FORCE_WITNESS_NO_VIOLATED_BOUND';
end
end

function G=grad_norm(v,n,dv)
% Each scalar free Bernstein control on each coordinate acts independently.
% Jacobian for ||v|| wrt the 3x4 free-control matrix.
if n>0
    G=(v/n)*dv.';
else
    G=zeros(3,4); % A zero norm has no unique gradient: fail closed below.
end
end

function [A,b,R]=append_plane(A,b,R,G,deficit,U,why)
if ~(isfinite(deficit)&&deficit>=0&&all(isfinite(G(:))))
    R.status='UNKNOWN_NONFINITE_FORCE_GRADIENT';
    R.immovable_violation=true;return
end
v=reshape(G/U,12,1);
n=norm(v);
if ~(isfinite(n)&&n>0)
    R.immovable_violation=true;
    R.zero_first_order_sensitivity=true;
    R.status=why;return
end
A(:,end+1)=v/n;
b(end+1,1)=deficit/n;
R.planes=R.planes+1;
R.max_deficit_N=max(R.max_deficit_N,deficit);
end

function Q=extract_leaf(P,a,b)
% De Casteljau restriction of one polynomial to [a,b], 0<=a<b<=1.
Q=P;
if a>0
    [~,Q]=split_at(Q,a);
end
if b<1
    [Q,~]=split_at(Q,(b-a)/(1-a));
end
end

function [L,R]=split_at(P,u)
n=size(P,2);Q=P;L=zeros(size(P));R=L;
L(:,1)=P(:,1);R(:,n)=P(:,n);
for k=2:n
    Q=(1-u)*Q(:,1:end-1)+u*Q(:,2:end);
    L(:,k)=Q(:,1);R(:,n-k+1)=Q(:,end);
end
end
