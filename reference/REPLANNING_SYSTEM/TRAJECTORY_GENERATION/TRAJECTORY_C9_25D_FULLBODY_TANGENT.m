function S=TRAJECTORY_C9_25D_FULLBODY_TANGENT(t0,T,J0,J1,X,phase,g,L)
%TRAJECTORY_C9_25D_FULLBODY_TANGENT Exact local derivative, one degree-27 C9.
% The 25 physical decision variables are [vec(3x8 interior controls); T].
% Physical endpoint jets (0..9) remain fixed as T changes.
% At fixed normalized u, for q=(g-pL'')/||g-pL''||:
%   dpL/dx = Jp, daL/dx = Ja,
%   dq/dx = -(I-q*q')/||g-aL|| * Ja,
%   dp(lambda)/dx = Jp + lambda*L*(I-q*q')/||g-aL||*Ja.
% The duration column includes derivative of BOTH endpoint control sets,
% the T^-2 acceleration factor, and (for fixed physical t) du/dT=-u/T.
% This is a SEARCH derivative only; it is never an interval/safety proof.
assert(isnumeric(phase)&&isreal(phase)&&isvector(phase)&& ...
    all(isfinite(phase(:)))&&all(phase(:)>=0)&&all(phase(:)<=1), ...
    'C9_25D_TANGENT:Phase','Finite normalized phases are necessary.');
assert(isnumeric(g)&&isreal(g)&&isequal(size(g),[3 1])&& ...
    all(isfinite(g))&&isscalar(L)&&isreal(L)&&isfinite(L)&&L>0, ...
    'C9_25D_TANGENT:Physics','Real current gravity and cable length required.');
[P,dP]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(J0,J1,X,T);
n=27;
accel_scale=n*(n-1)/T^2;
A=accel_scale*diff(P,2,2);
dA=accel_scale*diff(dP,2,2)-2/T*A;
u=double(phase(:).');N=numel(u);
S=struct('status','UNKNOWN_C9_25D_DEGENERATE', ...
    'safe_to_commit',false,'piece',TRAJECTORY_C9_FLAT_BERNSTEIN.build27( ...
        t0,T,J0,J1,X),'phase',u,'time_s',t0+T*u, ...
    'payload_m',NaN(3,N),'uav_m',NaN(3,N), ...
    'cable_q',NaN(3,N),'payload_velocity_mps',NaN(3,N), ...
    'payload_acceleration_mps2',NaN(3,N), ...
    'payload_J_fixed_phase',NaN(3,25,N), ...
    'uav_J_fixed_phase',NaN(3,25,N), ...
    'cable_q_J_fixed_phase',NaN(3,25,N), ...
    'payload_J_fixed_time',NaN(3,25,N), ...
    'uav_J_fixed_time',NaN(3,25,N), ...
    'acceleration_J_fixed_phase',NaN(3,25,N), ...
    'acceleration_control_J',zeros(3,26,25));
% Acceleration-control derivative tensor is also the Hessian primitive.
% Its cross-axis derivatives are exactly ZERO, not uninitialized NaNs.
% For each free coordinate, only the matching physical axis has nonzero
% acceleration sensitivity; the time column can couple all three axes.
for j=1:8
    jj=j+10;
    column=zeros(1,28);column(jj)=1;
    second=accel_scale*diff(column,2,2);
    for axis=1:3
        ix=3*(j-1)+axis;
        S.acceleration_control_J(axis,:,ix)=second;
    end
end
S.acceleration_control_J(:,:,25)=dA;
V=(n/T)*diff(P,1,2);
for k=1:N
    b=bernstein_weights(n,u(k));
    b1=bernstein_weights(n-1,u(k));
    b2=bernstein_weights(n-2,u(k));
    p=P*b; v=V*b1; a=A*b2;
    w=g-a;normw=norm(w);
    if ~(isfinite(normw)&&normw>0)
        S.status='UNKNOWN_C9_25D_NONTAUT_DIRECTION';return
    end
    q=w/normw;
    Xfree=zeros(3,25);Xacc=zeros(3,25);
    for j=1:8
        jj=j+10;
        cf=b(jj);
        ca=accel_scale*(b2(jj-2)-2*b2(jj-1)+b2(jj));
        ix=3*(j-1)+(1:3);
        Xfree(:,ix)=cf*eye(3);
        Xacc(:,ix)=ca*eye(3);
    end
    Xfree(:,25)=dP*b;
    Xacc(:,25)=dA*b2;
    K=(eye(3)-q*q.')/normw;
    Jq=-K*Xacc;
    Ju=Xfree-L*Jq;
    % At fixed physical t, differentiate u=(t-t0)/T as well.
    % p'_u=T*v and a'_u=T*p''' (using degree-24 control points).
    j=(n-2)/T*diff(A,1,2)*bernstein_weights(n-3,u(k));
    qp=-K*j;
    du_dT=-u(k)/T;
    Xp_time=Xfree;
    Xp_time(:,25)=Xfree(:,25)+T*v*du_dT;
    Xq_time=Jq;
    Xq_time(:,25)=Jq(:,25)+T*qp*du_dT;
    S.payload_m(:,k)=p;
    S.uav_m(:,k)=p-L*q;
    S.cable_q(:,k)=q;
    S.payload_velocity_mps(:,k)=v;
    S.payload_acceleration_mps2(:,k)=a;
    S.payload_J_fixed_phase(:,:,k)=Xfree;
    S.uav_J_fixed_phase(:,:,k)=Ju;
    S.cable_q_J_fixed_phase(:,:,k)=Jq;
    S.payload_J_fixed_time(:,:,k)=Xp_time;
    S.uav_J_fixed_time(:,:,k)=Xp_time-L*Xq_time;
    S.acceleration_J_fixed_phase(:,:,k)=Xacc;
end
% Do not label a tangent VALID unless every component of the matrix
% used by the 25-variable QP is finite. Check the complete 3x26x25 tensor,
% not only the 25 selected axis slices written by the loop.
if ~all(isfinite(S.acceleration_control_J(:)))
    S.status='UNKNOWN_C9_25D_NONFINITE_ACCELERATION_CONTROL_J';return
end
S.status='VALID_LOCAL_C9_25D_TANGENT__NOT_A_CONTINUOUS_TIME_CERTIFICATE';
end

function b=bernstein_weights(n,u)
% Numerically stable degree <=27; endpoints are EXACT.
b=zeros(n+1,1);
if u==0,b(1)=1;return;end
if u==1,b(end)=1;return;end
if u<=0.5
    b(1)=(1-u)^n;
    for k=1:n,b(k+1)=b(k)*(n-k+1)*u/(k*(1-u));end
else
    b(end)=u^n;
    for k=n:-1:1,b(k)=b(k+1)*k*(1-u)/((n-k+1)*u);end
end
end
