function Q=TRAJECTORY_C9_13D_NATURAL_QP(req,shapes,phases,rad,ground_active)
% TRAJECTORY_C9_13D_NATURAL_QP
% One linearized 13-variable QP in the REAL physical coordinates
%        dx=[d free(:); d duration_s].
% The physical objective is precisely the first-order squared
% change in load acceleration integrated over the current time piece:
%    min (1/2) * integral_0^T || J_a(u)*dx ||^2 dt.
% No arbitrary relative time-vs-space weight, velocity floor, or
% obstacle-specific path exists.  The resulting H is an EXACT polynomial
% Gram integral of the 3x13 physical acceleration tangent, not quadrature.
% If H is singular, the natural metric cannot uniquely determine duration;
% return UNKNOWN instead of inserting an unmotivated regularizer.
% All static shapes, cable points, payload and UAV share ONE QP; only the
% certified solver (called elsewhere) may assert all-time safety.
clock=tic;
Q=struct('status','UNKNOWN_NOT_STARTED','solved',false, ...
    'safe_to_commit',false,'H',NaN(13),'raw_A',zeros(13,0), ...
    'raw_b',zeros(0,1),'A',zeros(13,0),'b',zeros(0,1), ...
    'chol_upper',NaN(13),'obstacle_plane_count',0, ...
    'ground_plane_count',0,'unmovable_count',0, ...
    'radius_inflated_m',rad,'duration_bound_rows',0,'torque_search_plane_count',0, ...
    'torque_point_status','NOT_RUN','torque_point_violation_count',0, ...
    'duration_s',req.duration_s,'elapsed_ms',NaN, ...
    'shared_duration_cache_enabled',false,'batched_jet_enabled',false, ...
    'duration_sensitivity_ms',NaN,'fullbody_tangent_ms',NaN, ...
    'torque_gradient_ms',NaN);
T=double(req.duration_s);
free=double(req.nominal_free);
% Default enabled after 7-repeat MATLAB A/B parity + timing audit.
% Caller can explicitly turn it OFF; stale cache still fails closed.
share=true;
if isfield(req.search_options,'use_shared_13d_duration_cache')
    v=req.search_options.use_shared_13d_duration_cache;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D_QP:SharedCacheFlag','Cache flag must be 0/1.');
    share=logical(v);
end
Q.shared_duration_cache_enabled=share;
t_s=tic;
S=TRAJECTORY_C9_DURATION_SENSITIVITY(req.t0_s,T, ...
    req.start_jets,req.end_jets,free,phases);
Q.duration_sensitivity_ms=1000*toc(t_s);
% Physical derivative of pL'' with respect to each free P_j and to T.
% Coefficient arrays in the Bernstein basis B^21_k (k=0..21).
n=21; C=zeros(3,n+1,13);
factor=23*22/T^2;
for j=1:4
    col=j+10; % Control P_(j+10), MATLAB one-based 11..14.
    for axis=1:3
        ix=3*(j-1)+axis;
        C(axis,col-2,ix)=factor;
        C(axis,col-1,ix)=-2*factor;
        C(axis,col,ix)=factor;
    end
end
C(:,:,13)=S.dphysical_derivative_controls_dT{3};
% Exact Bernstein Gram. i,j are ZERO based; integral B_i^n B_j^n =
% C(n,i)C(n,j) / ((2*n+1) C(2*n,i+j)).
B=zeros(n+1);
for i=0:n
    for j=0:i
        logv=gammaln(n+1)-gammaln(i+1)-gammaln(n-i+1) + ...
             gammaln(n+1)-gammaln(j+1)-gammaln(n-j+1) - ...
             log(2*n+1) - gammaln(2*n+1) + ...
             gammaln(i+j+1)+gammaln(2*n-i-j+1);
        B(i+1,j+1)=exp(logv);
        B(j+1,i+1)=B(i+1,j+1);
    end
end
H=zeros(13);
for axis=1:3
    X=reshape(C(axis,:,:),n+1,13);
    H=H+T*(X.'*B*X);
end
H=0.5*(H+H.');Q.H=H;
% There is no arbitrary minimum-duration parameter or regularizer.
[R,p]=chol(H,'upper');
if p~=0 || ~all(isfinite(R(:)))
    Q.status='UNKNOWN_13D_ACCELERATION_METRIC_NOT_POSITIVE_DEFINITE';
    Q.elapsed_ms=1000*toc(clock);return
end
Q.chol_upper=R;
t_full=tic;
if share
    F=TRAJECTORY_C9_DURATION_FULLBODY_JACOBIAN( ...
        req.t0_s,T,req.start_jets,req.end_jets,free,phases, ...
        req.authority.gravity_world_mps2,req.authority.cable_length_m,S);
else
    F=TRAJECTORY_C9_DURATION_FULLBODY_JACOBIAN( ...
        req.t0_s,T,req.start_jets,req.end_jets,free,phases, ...
        req.authority.gravity_world_mps2,req.authority.cable_length_m);
end
Q.fullbody_tangent_ms=1000*toc(t_full);
if ~strcmp(F.status,'VALID_LOCAL_13D_TANGENT__INDEPENDENT_FULL_TIME_PROOF_REQUIRED')
    Q.status=F.status;Q.elapsed_ms=1000*toc(clock);return
end
N=numel(phases);Ls=double(req.authority.cable_length_m);
delta=double(req.search_options.min_separation_m)+ ...
    double(req.geometry_options.numerical_guard_m);
% Bounds per physical body ordered UAV / cable / payload.
A=zeros(13,3*N*(numel(shapes)+double(ground_active))+2);
b=zeros(size(A,2),1);m=0;
for k=1:N
    pos=F.payload_m(:,k);q=F.cable_direction(:,k);
    jp=F.payload_J_fixed_phase(:,:,k);
    jq=F.cable_q_J_fixed_phase(:,:,k);
    for j=1:numel(shapes)
        shape=shapes(j);
        lambda=max(0,min(1,((pos-shape.center).'*q)/Ls));
        frac=[1,lambda,0];
        for body=1:3
            lam=frac(body);
            center=pos-lam*Ls*q;
            jac=jp-lam*Ls*jq;
            nvec=center-shape.center;
            if ~all(isfinite(nvec)) || norm(nvec)==0
                Q.status='UNKNOWN_13D_PLANE_DIRECTION_UNDEFINED';
                Q.elapsed_ms=1000*toc(clock);return
            end
            nvec=nvec/norm(nvec);
            h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,nvec);
            rhs=h+rad(body)+delta-nvec.'*center;
            grad=jac.'*nvec;
            [A,b,m,Q]=append_plane(A,b,m,Q,grad,rhs,'shape');
            if Q.unmovable_count>0
                Q.status='UNKNOWN_13D_UNMOVABLE_SEPARATION_PLANE';
                Q.elapsed_ms=1000*toc(clock);return
            end
        end
    end
    if ground_active
        % Ground z<=0: separately inflated actual UAV/cable/payload.
        % Cable z minimum is at one of the endpoints for fixed q.
        cable_lam=double(q(3)>0);
        frac=[1,cable_lam,0];
        for body=1:3
            lam=frac(body);
            center=pos-lam*Ls*q;
            jac=jp-lam*Ls*jq;
            rhs=rad(body)+delta-center(3);
            grad=jac(3,:).';
            [A,b,m,Q]=append_plane(A,b,m,Q,grad,rhs,'ground');
            if Q.unmovable_count>0
                Q.status='UNKNOWN_13D_UNMOVABLE_GROUND_PLANE';
                Q.elapsed_ms=1000*toc(clock);return
            end
        end
    end
end
% The SAME local QP also carries pointwise linearizations of the
% *actual 3-axis nominal torque map* in all 13 physical coordinates.
% These are SEARCH inequalities only. They must never certify all time.
% A missing/singular heading or unavailable live physical authority is
% UNKNOWN, not a reason to silently discard the torque constraints.
if ~isfield(req,'torque_options') || ~isstruct(req.torque_options)
    Q.status='UNKNOWN_13D_TORQUE_OPTIONS_REQUIRED';
    Q.elapsed_ms=1000*toc(clock);return
end
if ~(isfield(req.authority,'torque_limits_Nm')&& ...
        isequal(size(req.authority.torque_limits_Nm),[3 2])&& ...
        isfield(req.torque_options,'torque_guard_Nm'))
    Q.status='UNKNOWN_13D_ACTUAL_TORQUE_LIMITS_REQUIRED';
    Q.elapsed_ms=1000*toc(clock);return
end
t_grad=tic;
if share
    Z=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_13D( ...
        S.piece,req.authority,req.torque_options,phases,S);
else
    Z=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_13D( ...
        S.piece,req.authority,req.torque_options,phases);
end
Q.torque_gradient_ms=1000*toc(t_grad);
Q.batched_jet_enabled=Z.jet_batch_enabled;
Q.torque_point_status=Z.status;
if ~all(Z.valid)
    Q.status='UNKNOWN_13D_HEADING_OR_TORQUE_POINT_DEGENERACY';
    Q.elapsed_ms=1000*toc(clock);return
end
lims=double(req.authority.torque_limits_Nm);
guard=double(req.torque_options.torque_guard_Nm(:));
low=lims(:,1)+guard;high=lims(:,2)-guard;
if ~(numel(guard)==3&&all(isfinite([lims(:);guard]))&&all(high>low))
    Q.status='UNKNOWN_13D_NO_NONEMPTY_TORQUE_HEADROOM';
    Q.elapsed_ms=1000*toc(clock);return
end
% Grow rows to hold every initially satisfied inequality as well. A
% simultaneous update may violate an inequality initially slack at dx=0.
A=[A zeros(13,6*N)];b=[b;zeros(6*N,1)];
for k=1:N
    M=Z.torque_Nm(:,k);
    D=Z.gradient_Nm_per_physical(:,:,k);
    Q.torque_point_violation_count=Q.torque_point_violation_count+ ...
        sum(M<low|M>high);
    for axis=1:3
        % Upper M + dM*dx <= high; lower M+dM*dx >= low.
        [A,b,m,Q]=append_plane(A,b,m,Q,-D(axis,:).',M(axis)-high(axis),'torque');
        [A,b,m,Q]=append_plane(A,b,m,Q,+D(axis,:).',low(axis)-M(axis),'torque');
        Q.torque_search_plane_count=Q.torque_search_plane_count+2;
        if Q.unmovable_count>0
            Q.status='UNKNOWN_13D_TORQUE_POINT_IMMOVABLE';
            Q.elapsed_ms=1000*toc(clock);return
        end
    end
end
% Physical time interval; valid for the entire Flight piece. T>0 is a
% strict physical domain, and candidate T=0 must be rejected afterward.
% Linear QP rows only enforce T>=0 at this stage, no epsilon fudge.
Tmax=double(req.flight_interval_s(2))-double(req.t0_s);
e=zeros(13,1);e(13)=1;
[A,b,m,Q]=append_plane(A,b,m,Q,e,-T,'duration');
[A,b,m,Q]=append_plane(A,b,m,Q,-e,T-Tmax,'duration');
Q.duration_bound_rows=2;
Q.raw_A=A(:,1:m);Q.raw_b=b(1:m);
Q.A=R.'\Q.raw_A; Q.b=Q.raw_b;
Q.status='LOCAL_13D_QP_READY__NOT_SAFETY_CERTIFIED';
Q.solved=true;
Q.elapsed_ms=1000*toc(clock);
end

function [A,b,m,Q]=append_plane(A,b,m,Q,grad,rhs,kind)
if ~(all(isfinite(grad))&&isfinite(rhs))
    Q.unmovable_count=Q.unmovable_count+1;return
end
if norm(grad)==0
    if rhs>0,Q.unmovable_count=Q.unmovable_count+1;end
    return
end
m=m+1;A(:,m)=grad;b(m)=rhs;
if strcmp(kind,'ground')
    Q.ground_plane_count=Q.ground_plane_count+1;
elseif strcmp(kind,'shape')
    Q.obstacle_plane_count=Q.obstacle_plane_count+1;
end
end
