function R=TRAJECTORY_C9_ACTIVE_REFERENCE_WINDOW(piece,times_s,cable_length_m)
%TRAJECTORY_C9_ACTIVE_REFERENCE_WINDOW One active C9 reference, many times.
% The SAME 28xN load/jets are intended for future active-trajectory HLC
% evaluation and 4-second trigger prediction. No original nominal-clock
% recovery, interpolation of a sampled path, or extrapolation past piece.
% This function does NOT install the reference in Flight. Atomic switching
% of trigger source and HLC source remains subject to execution proofs.
R=struct('status','UNKNOWN_ACTIVE_C9_WINDOW_INPUT', ...
 'valid',false,'HLC_28xN',[],'pre_SLOAD_28xN',[], ...
 'yaw_principal_1xN',[],'yaw_branch_continuity_proven',false, ...
 'safe_to_commit',false);
if ~(isstruct(piece)&&isscalar(piece)&&isfield(piece,'control_points_m')&& ...
    isfield(piece,'duration_s')&&isfield(piece,'t0_s')&& ...
    isnumeric(times_s)&&isreal(times_s)&&isvector(times_s)&& ...
    ~isempty(times_s)&&all(isfinite(times_s(:)))&& ...
    all(diff(times_s(:))>0)&& ...
    isnumeric(cable_length_m)&&isscalar(cable_length_m)&& ...
    isfinite(cable_length_m)&&cable_length_m>0)
    return
end
try
    t=double(times_s(:).');
    T=double(piece.duration_s);t0=double(piece.t0_s);
    if ~(isfinite(T)&&T>0&&all(t>=t0)&&all(t<=t0+T))
        R.status='UNKNOWN_ACTIVE_C9_WINDOW_OUTSIDE_PIECE';return
    end
    P=double(piece.control_points_m);
    if ~(size(P,1)==3&&any(size(P,2)==[24 28])&& ...
         all(isfinite(P(:))))
        R.status='UNKNOWN_ACTIVE_C9_WINDOW_CONTROLS';return
    end
    n=size(P,2)-1;N=numel(t);u=(t-t0)/T;
    D=TRAJECTORY_C9_FLAT_BERNSTEIN.derivative_cache(P,T);
    J=zeros(3,N,8);
    for j=0:7
        B=bernstein_eval_matrix(n-j,u);
        J(:,:,j+1)=D{j+1}*B;
    end
    z=complex(J(1,:,2),J(2,:,2));
    if any(~isfinite(abs(z))|abs(z)==0)
        R.status='UNKNOWN_ACTIVE_C9_WINDOW_ZERO_HORIZONTAL_TANGENT';return
    end
    w=complex(zeros(7,N));w(1,:)=1;
    for k=1:6
        w(k+1,:)=complex(J(1,:,k+2),J(2,:,k+2))./(factorial(k)*z);
    end
    b=complex(zeros(7,N));
    for ord=1:6
        s=zeros(1,N);
        for k=1:ord-1
            s=s+k*b(k+1,:).*w(ord-k+1,:);
        end
        b(ord+1,:)=w(ord+1,:)-s/ord;
    end
    yaw=zeros(7,N);
    yaw(1,:)=atan2(imag(z),real(z));
    for k=1:6
        yaw(k+1,:)=imag(b(k+1,:))*factorial(k);
    end
    H=zeros(28,N);
    for k=0:6
        H(4*k+(1:3),:)=J(:,:,k+1);
        H(4*k+4,:)=yaw(k+1,:);
    end
    if any(~isfinite(H(:)))
        R.status='UNKNOWN_ACTIVE_C9_WINDOW_NONFINITE';return
    end
    W=H;W(3,:)=W(3,:)+cable_length_m;
    R.HLC_28xN=H;
    R.pre_SLOAD_28xN=W;
    R.yaw_principal_1xN=yaw(1,:);
    % Exact yaw branch certification still requires checking that no
    % horizontal complex velocity zero or +/-pi wrap occurs BETWEEN times.
    % Sampling alone cannot establish that for all t.
    R.yaw_branch_continuity_proven=false;
    R.valid=true;
    R.status='PASS_ACTIVE_C9_28D_SAME_SOURCE_WINDOW__HEADING_BRANCH_UNCERTIFIED';
catch ME
    R.status=['UNKNOWN_ACTIVE_C9_WINDOW_EXCEPTION_' ME.identifier];
end
end
function B=bernstein_eval_matrix(n,u)
% Stable 0..1 Bernstein basis, including mathematically exact endpoints.
N=numel(u);B=zeros(n+1,N);
for k=1:N
    x=u(k);
    if x==0,B(1,k)=1;continue;end
    if x==1,B(end,k)=1;continue;end
    if x<=.5
        B(1,k)=(1-x)^n;
        for j=1:n
            B(j+1,k)=B(j,k)*(n-j+1)*x/(j*(1-x));
        end
    else
        B(end,k)=x^n;
        for j=n:-1:1
            B(j,k)=B(j+1,k)*j*(1-x)/((n-j+1)*x);
        end
    end
end
end
