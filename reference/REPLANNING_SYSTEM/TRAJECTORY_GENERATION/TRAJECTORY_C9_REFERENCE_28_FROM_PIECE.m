function R=TRAJECTORY_C9_REFERENCE_28_FROM_PIECE(piece,t_s,cable_length_m,desired_yaw_branch)
%TRAJECTORY_C9_REFERENCE_28_FROM_PIECE Exact heading-aligned HLC xd(1:28).
% A source-level adapter primitive, NOT a license to switch Flight refs.
% The existing SUSPENDED_LOAD_REF_ADJUST subtracts cable length from
% xd(3) in f phase. Therefore the PRE-SLOAD reference must have +L in
% xd(3) (position ONLY) to deliver the intended payload position to HLC.
% Yaw derivatives 0..6 are computed analytically from log(vx+i*vy),
% not sampled differences, and require horizontal velocity nonzero.
R=struct('valid',false,'status','UNKNOWN_C9_REFERENCE28_INPUT', ...
 'xd_pre_sload_28',NaN(28,1),'xd_at_HLC_28',NaN(28,1), ...
 'yaw_derivatives_0_to_6',NaN(7,1),'heading_exact',false, ...
 'safe_to_commit',false);
if nargin<4,desired_yaw_branch=[];end
if ~(isstruct(piece)&&isscalar(piece)&& ...
        isnumeric(t_s)&&isscalar(t_s)&&isfinite(t_s)&& ...
        isnumeric(cable_length_m)&&isscalar(cable_length_m)&& ...
        isfinite(cable_length_m)&&cable_length_m>0 && ...
        (isempty(desired_yaw_branch)||(isnumeric(desired_yaw_branch)&& ...
          isscalar(desired_yaw_branch)&&isfinite(desired_yaw_branch))))
    return
end
try
    J=TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(piece,t_s);
    if ~isequal(size(J),[3 10])||any(~isfinite(J(:)))
        R.status='UNKNOWN_C9_REFERENCE28_JETS_INVALID';return
    end
    z=complex(J(1,2),J(2,2));
    if abs(z)==0||~isfinite(abs(z))
        R.status='UNKNOWN_C9_REFERENCE28_HORIZONTAL_TANGENT_ZERO';return
    end
    % Taylor coefficients z_j = (d/dt)^j(vx+i vy)/j!, j=0..6.
    % ln(z(t)/z0) = sum b_j (t-t0)^j; derivative recursion by z'/z.
    w=zeros(7,1);w(1)=1;
    for k=1:6
        w(k+1)=complex(J(1,k+2),J(2,k+2))/(factorial(k)*z);
    end
    b=zeros(7,1);
    for n=1:6
        s=0;
        for k=1:n-1
            s=s+k*b(k+1)*w(n-k+1);
        end
        b(n+1)=w(n+1)-s/n;
    end
    yaw=zeros(7,1);yaw(1)=atan2(imag(z),real(z));
    if ~isempty(desired_yaw_branch)
        yaw(1)=yaw(1)+2*pi*round((desired_yaw_branch-yaw(1))/(2*pi));
    end
    for k=1:6
        yaw(k+1)=imag(b(k+1))*factorial(k);
    end
    xd=zeros(28,1);
    for k=0:6
        xd(4*k+(1:3))=J(:,k+1);
        xd(4*k+4)=yaw(k+1);
    end
    if any(~isfinite([xd;yaw]))
        R.status='UNKNOWN_C9_REFERENCE28_NONFINITE';return
    end
    R.xd_at_HLC_28=xd;
    xd(3)=xd(3)+cable_length_m;
    R.xd_pre_sload_28=xd;
    R.yaw_derivatives_0_to_6=yaw;
    R.heading_exact=true;
    R.valid=true;
    R.status='PASS_SOURCE_REFERENCE_FORMAT_AND_ANALYTIC_HEADING__SWITCH_NOT_AUTHORIZED';
catch ME
    R.status=['UNKNOWN_C9_REFERENCE28_EXCEPTION_' ME.identifier];
end
end
