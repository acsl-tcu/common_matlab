function Q=TRAJECTORY_C9_25D_NATURAL_QP(req,shapes,phase,rad,ground_active,prior_proof)
%TRAJECTORY_C9_25D_NATURAL_QP  ONE 25-variable, degree-27 C9 local QP.
% x=[vec(eight 3D interior controls); T] (24 + 1 physical variables).
% Construct exact first-order 3D UAV/cable/payload tangent rows using
% actual canonical shape support. Every candidate subsequently passes the
% independent continuous-time oracle; no sample here certifies safety.
% No obstacle-specific direction, dimensionalized fallback, or hard time cap.
clock=tic;
Q=struct('solved',false,'status','UNKNOWN_C9_25D_QP_NOT_STARTED', ...
    'safe_to_commit',false,'H',NaN(25),'chol_upper',NaN(25), ...
    'A',zeros(25,0),'b',zeros(0,1), ...
    'geometry_plane_count',0,'ground_plane_count',0, ...
    'duration_bound_rows',0,'elapsed_ms',NaN,'candidate_tangent',struct(), ...
    'torque_point_rows',0,'torque_tangent_ms',0, ...
    'physical_leaf_rows',0,'physical_leaf_status','NOT_RUN', ...
    'optional_leaf_skipped',false,'torque_tangent_status','NOT_RUN', ...
    'optional_torque_tangent_skipped',false, ...
    'metric_status','NOT_RUN','metric_ms',NaN, ...
    'native_25d_torque_executed',false,'native_25d_torque_ms',0, ...
    'linear_constraint_count',0);
T=double(req.duration_s);
F=double(req.nominal_free27);
S=TRAJECTORY_C9_25D_FULLBODY_TANGENT( ...
    req.t0_s,T,req.start_jets,req.end_jets,F,phase, ...
    double(req.authority.gravity_world_mps2(:)), ...
    double(req.authority.cable_length_m));
Q.candidate_tangent=S;
if ~strcmp(S.status, ...
        'VALID_LOCAL_C9_25D_TANGENT__NOT_A_CONTINUOUS_TIME_CERTIFICATE')
    Q.status=S.status;Q.elapsed_ms=1000*toc(clock);return
end
% 26-node Gauss-Legendre is EXACT for all degree-50 products of
% degree-25 acceleration derivatives. QR builds the acceleration metric
% without ever forming the badly conditioned Bernstein 26x26 Gram.
% A positive numerical regularization is NOT silently introduced.
M=TRAJECTORY_C9_25D_EXACT_ACCELERATION_QR( ...
    S.acceleration_control_J,T);
Q.metric_status=M.status;
Q.metric_ms=M.elapsed_ms;
if ~M.valid
    Q.status=M.status;Q.elapsed_ms=1000*toc(clock);return
end
Q.H=M.H;
U=M.U;
Q.chol_upper=U;
L=double(req.authority.cable_length_m);
margin=double(req.search_options.min_separation_m)+ ...
    double(req.geometry_options.numerical_guard_m);
if ~(isfinite(margin)&&margin>=0)
    Q.status='UNKNOWN_C9_25D_NONFINITE_MARGIN';
    Q.elapsed_ms=1000*toc(clock);return
end
N=numel(phase);
A=zeros(25,3*N*(numel(shapes)+double(ground_active))+ ...
    28*numel(shapes)+6*N+2);
b=zeros(size(A,2),1);m=0;
for k=1:N
    p=S.payload_m(:,k);q=S.cable_q(:,k);
    Jp=S.payload_J_fixed_phase(:,:,k);
    Jq=S.cable_q_J_fixed_phase(:,:,k);
    for j=1:numel(shapes)
        shape=shapes(j);
        lamNear=max(0,min(1,(q.'*(p-shape.center))/L));
        for body=1:3
            switch body
                case 1,lam=1;
                case 2,lam=lamNear;
                case 3,lam=0;
            end
            center=p-lam*L*q;
            J=Jp-lam*L*Jq;
            v=center-shape.center;
            nv=norm(v);
            if ~(isfinite(nv)&&nv>0)
                Q.status='UNKNOWN_C9_25D_SEARCH_NORMAL_UNDEFINED';
                Q.elapsed_ms=1000*toc(clock);return
            end
            n=v/nv;h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,n);
            rhs=h+rad(body)+margin-n.'*center;
            grad=J.'*n;
            [A,b,m,possible]=add_row(A,b,m,grad,rhs);
            if ~possible
                Q.status='UNKNOWN_C9_25D_IMMOVABLE_OBSTACLE_PLANE';
                Q.elapsed_ms=1000*toc(clock);return
            end
            Q.geometry_plane_count=Q.geometry_plane_count+1;
        end
    end
    if ground_active
        % Infinite z<=0 halfspace applies only to the Flight phase.
        % The straight cable reaches minimum z at one endpoint.
        lam=[1,double(q(3)>0),0];
        for body=1:3
            center=p-lam(body)*L*q;
            J=Jp-lam(body)*L*Jq;
            rhs=rad(body)+margin-center(3);
            [A,b,m,possible]=add_row(A,b,m,J(3,:).',rhs);
            if ~possible
                Q.status='UNKNOWN_C9_25D_IMMOVABLE_GROUND_PLANE';
                Q.elapsed_ms=1000*toc(clock);return
            end
            Q.ground_plane_count=Q.ground_plane_count+1;
        end
    end
end
% The failed full-time oracle can supply an entire unresolved physical
% payload leaf, not merely sampled points. Those exact Bernstein control
% inequalities are included in THIS SAME local 25D QP. The final proof
% still checks EVERY shape/body/leaf independently.
if nargin>=6 && isstruct(prior_proof)
    leaf_on=true;
    if isfield(req.search_options,'enable_25d_physical_leaf_hull_rows')
        v=req.search_options.enable_25d_physical_leaf_hull_rows;
        assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
            isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_25D_QP:LeafSwitch','Payload leaf switch must be 0/1.');
        leaf_on=logical(v);
    end
    if leaf_on
        W=TRAJECTORY_C9_25D_LEAF_HULL_ROWS( ...
            req,shapes,prior_proof,rad);
        Q.physical_leaf_status=W.status;
        if startsWith(W.status,'UNKNOWN_')
            % This is a SEARCH-ONLY optional tangent. An unavailable
            % witness cut does not imply the 25D base-QP is undefined.
            % The candidate still requires a complete independent oracle.
            Q.optional_leaf_skipped=true;
        else
        m_before_optional_leaf=m;
        for ii=1:W.rows
            [A,b,m,ok]=add_row(A,b,m,W.A(:,ii),W.b(ii));
            if ~ok
                % A witness facet can be stationary under the current
                % linearization, even when nonlinear future motions can
                % improve it. Do not call that a global impossibility.
                Q.optional_leaf_skipped=true;
                break
            end
        end
        if Q.optional_leaf_skipped
            m=m_before_optional_leaf;
        else
            Q.physical_leaf_rows=W.rows;
        end
        end
    end
end
% The same 25-variable QP may include the actual *nominal flatness*
% torque's 3-axis first-order rows, with no 13D-only Jacobian reuse.
% These point rows are SEARCH constraints, never a whole-time torque proof.
torque_rows=true;
if isfield(req.search_options,'enable_25d_nominal_torque_local_rows')
    sw=req.search_options.enable_25d_nominal_torque_local_rows;
    assert((isnumeric(sw)||islogical(sw))&&isscalar(sw)&& ...
        isfinite(double(sw))&&any(double(sw)==[0 1]), ...
        'C9_25D_QP:TorqueMode','25D torque-local option must be 0/1');
    torque_rows=logical(sw);
end
if torque_rows
    assert(isfield(req,'torque_options')&& ...
        isstruct(req.torque_options)&& ...
        isfield(req.authority,'torque_limits_Nm'), ...
        'C9_25D_QP:TorqueAuthority', ...
        'Current nominal torque options and HLC limits are necessary.');
    tt=tic;
    Z=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_25D( ...
        S.piece,req.authority,req.torque_options,phase);
    Q.torque_tangent_ms=1000*toc(tt);
    Q.torque_tangent_status=Z.status;
    Q.native_25d_torque_executed=Z.native_executed;
    Q.native_25d_torque_ms=Z.native_kernel_ms;
    if startsWith(Z.status,'UNKNOWN_NATIVE_25D')
        % Explicitly requested native code was not pinned/available;
        % this is an integration error, not permission to bypass it.
        Q.status=Z.status;Q.elapsed_ms=1000*toc(clock);return
    end
    if ~all(Z.valid)
        % A singular sampled torque tangent is not a certificate of
        % impossibility; omit ONLY the optional search linearization.
        % All-time torque certification remains mandatory on the candidate.
        Q.optional_torque_tangent_skipped=true;
    else
    limits=double(req.authority.torque_limits_Nm);
    guard=double(req.torque_options.torque_guard_Nm(:));
    assert(isequal(size(limits),[3 2])&&numel(guard)==3&& ...
        all(isfinite([limits(:);guard]))&&all(guard>=0), ...
        'C9_25D_QP:TorqueLimits','Live three-axis bounds required.');
    lo=limits(:,1)+guard;
    hi=limits(:,2)-guard;
    if any(hi<=lo)
        Q.status='UNKNOWN_C9_25D_NOMINAL_TORQUE_NO_HEADROOM';
        Q.elapsed_ms=1000*toc(clock);return
    end
    m_before_torque=m;
    for k=1:N
        for axis=1:3
            tau=Z.torque_Nm(axis,k);
            D=reshape(Z.gradient_Nm_per_physical(axis,:,k),25,1);
            [A,b,m,ok1]=add_row(A,b,m,+D,lo(axis)-tau);
            [A,b,m,ok2]=add_row(A,b,m,-D,tau-hi(axis));
            if ~(ok1&&ok2)
                % A zero first derivative is only failure of THIS local
                % linearization; the nonlinear candidate could still
                % improve. Optional search rows may be dropped, never
                % the subsequent all-time independent torque proof.
                Q.optional_torque_tangent_skipped=true;
                break
            end
            Q.torque_point_rows=Q.torque_point_rows+2;
        end
        if Q.optional_torque_tangent_skipped, break; end
    end
    if Q.optional_torque_tangent_skipped
        m=m_before_torque;
        Q.torque_point_rows=0;
    end
    end
end
% Physical duration is a free variable, constrained by current Flight.
Tmax=double(req.flight_interval_s(2))-double(req.t0_s);
e=zeros(25,1);e(25)=1;
[A,b,m,ok1]=add_row(A,b,m,e,-T);
[A,b,m,ok2]=add_row(A,b,m,-e,T-Tmax);
if ~(ok1&&ok2)
    Q.status='UNKNOWN_C9_25D_DURATION_QP_INVALID';
    Q.elapsed_ms=1000*toc(clock);return
end
Q.duration_bound_rows=2;
% Local linearized constraints A_raw'*dx >= b. In whitened coordinates
% z=U*dx, the normals are U'\A_raw; verified by U'*U=H.
Q.A=U.'\A(:,1:m);
Q.b=b(1:m);
Q.linear_constraint_count=m;
Q.status='LOCAL_C9_25D_QP_READY__FULL_TIME_PROOF_REQUIRED';
Q.solved=true;
Q.elapsed_ms=1000*toc(clock);
end

function [A,b,m,possible]=add_row(A,b,m,grad,rhs)
possible=all(isfinite(grad))&&isfinite(rhs);
if ~possible,return;end
if norm(grad)==0
    possible=rhs<=0;
    return
end
m=m+1;
A(:,m)=grad;b(m)=rhs;
end
