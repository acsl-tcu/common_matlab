function S=TRAJECTORY_C9_DURATION_FULLBODY_JACOBIAN(t0_s,T_s,left_jets,right_jets,free,phases,gravity_world_mps2,cable_length_m,precomputed_duration)
% TRAJECTORY_C9_DURATION_FULLBODY_JACOBIAN
% Exact 13-column FIRST-ORDER tangent of the *ideal taut* load/UAV/cable
% CENTERLINES of ONE C9 polynomial: 12 free Bernstein coordinates plus
% physical duration T. No numerical finite differences or fixed direction.
%
% Coordinates are x=[free(:);T], with free(:) in MATLAB COLUMN order.
% For a fixed cable fraction lambda in [0,1]:
%    p(lambda)=pL-lambda*L*q, q=(g-pL'')/||g-pL''||.
%    d p(lambda) = d pL + lambda*L*(I-qq')/||g-pL''|| * d pL''.
% This formula is EXACT at first order for FIXED lambda. A closest
% cable-point lambda must be recalculated by the caller after each update.
% The caller must use an exact shape support and independent ALL-TIME
% certificate to decide safety; no tangent, sample or QP is a certificate.
%
% Outputs are 3x13xN for each center or cable direction. Both
% fixed-normalized-phase and fixed-physical-time duration derivatives are
% returned. For a fixed physical time t, du/dT=-u/T and the extra
% -u*p^(r+1) contribution MUST be included. The two derivative modes
% coincide for the 12 free-control columns but not for the 13th.
%
% Assumes constant externally supplied gravity and cable length over this
% candidate. Future mass changes, tracking tube and HLC input constraints
% must be certified independently. Nonzero ||g-pL''|| is compulsory.
if ~(nargin==8||nargin==9)
    error('C9_DURATION_FULLBODY:Arguments','Eight inputs plus optional validated duration sensitivity.');
end
assert(isnumeric(phases)&&isreal(phases)&&isvector(phases) && ...
    all(isfinite(phases(:)))&&all(phases(:)>=0)&&all(phases(:)<=1), ...
    'C9_DURATION_FULLBODY:Phases','Need finite normalized phases in [0,1].');
g=double(gravity_world_mps2(:));L=double(cable_length_m);
assert(isequal(size(g),[3 1])&&all(isfinite(g))&& ...
    isscalar(L)&&isreal(L)&&isfinite(L)&&L>0, ...
    'C9_DURATION_FULLBODY:Authority','Need authoritative gravity and cable length.');
u=double(phases(:).');N=numel(u);
S=struct('status','UNKNOWN_TAUT_DIRECTION_NOT_EVALUATED', ...
    'safe_to_commit',false,'scope', ...
    'ONE_C9_13D_LOCAL_LINEARIZATION_ONLY__NO_CONTINUOUS_TIME_OR_HLC_PROOF', ...
    'phase',u,'time_s',t0_s+u*T_s, ...
    'parameter_order','[free(:);duration_s]', ...
    'payload_m',NaN(3,N),'uav_m',NaN(3,N), ...
    'cable_direction',NaN(3,N),'taut_denominator_mps2',NaN(1,N), ...
    'payload_J_fixed_phase',NaN(3,13,N), ...
    'uav_J_fixed_phase',NaN(3,13,N), ...
    'cable_q_J_fixed_phase',NaN(3,13,N), ...
    'payload_J_fixed_time',NaN(3,13,N), ...
    'uav_J_fixed_time',NaN(3,13,N), ...
    'cable_q_J_fixed_time',NaN(3,13,N));
if nargin==9
    assert(TRAJECTORY_C9_DURATION_CACHE_MATCH(precomputed_duration, ...
        t0_s,T_s,left_jets,right_jets,free,u), ...
        'C9_DURATION_FULLBODY:StaleDurationCache', ...
        'Duration tangent cache does not describe this exact C9 candidate and phases.');
    D=precomputed_duration;
else
    D=TRAJECTORY_C9_DURATION_SENSITIVITY( ...
        t0_s,T_s,left_jets,right_jets,free,u);
end
E=eye(3);
for i=1:N
    x=u(i);
    p=D.physical_jets(:,1,i);
    accel=D.physical_jets(:,3,i);
    w=g-accel;
    sw=norm(w);
    if ~(isfinite(sw)&&sw>0)
        S.status='UNKNOWN_TAUT_DIRECTION_ZERO_OR_NONFINITE';
        return
    end
    q=w/sw;
    % General 13-variable derivative of payload position/acceleration.
    b23=local_basis(23,x);b21=local_basis(21,x);
    pFree=zeros(3,12);aFree=zeros(3,12);
    for k=1:4
        j=(k-1)*3+(1:3);
        pFree(:,j)=b23(10+k)*E;
        % Contribution of P_(9+k) (zero-based k=1..4) to pL''.
        d2=(23*22/T_s^2)*(b21(8+k)-2*b21(9+k)+b21(10+k));
        aFree(:,j)=d2*E;
    end
    jpPhase=[pFree,D.dphysical_jets_dT_fixed_phase(:,1,i)];
    jaPhase=[aFree,D.dphysical_jets_dT_fixed_phase(:,3,i)];
    jpTime=[pFree,D.dphysical_jets_dT_fixed_time(:,1,i)];
    jaTime=[aFree,D.dphysical_jets_dT_fixed_time(:,3,i)];
    project=(E-q*q.')/sw;
    jqPhase=-project*jaPhase;
    jqTime=-project*jaTime;
    S.payload_m(:,i)=p;
    S.uav_m(:,i)=p-L*q;
    S.cable_direction(:,i)=q;
    S.taut_denominator_mps2(i)=sw;
    S.payload_J_fixed_phase(:,:,i)=jpPhase;
    S.uav_J_fixed_phase(:,:,i)=jpPhase-L*jqPhase;
    S.cable_q_J_fixed_phase(:,:,i)=jqPhase;
    S.payload_J_fixed_time(:,:,i)=jpTime;
    S.uav_J_fixed_time(:,:,i)=jpTime-L*jqTime;
    S.cable_q_J_fixed_time(:,:,i)=jqTime;
end
S.status='VALID_LOCAL_13D_TANGENT__INDEPENDENT_FULL_TIME_PROOF_REQUIRED';
end

function b=local_basis(n,u)
% Stable exact-in-degree Bernstein basis, degrees n<=23 only.
assert(n>=0&&n<=23&&n==fix(n)&&u>=0&&u<=1, ...
    'C9_DURATION_FULLBODY:Basis','Invalid polynomial degree or phase.');
b=zeros(1,n+1);
if u==0,b(1)=1;return;end
if u==1,b(end)=1;return;end
c=1;
for j=0:n
    if j>0,c=c*(n-j+1)/j;end
    b(j+1)=c*u^j*(1-u)^(n-j);
end
end
