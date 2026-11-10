function Q=TRAJECTORY_C9_13D_NATURAL_QP(req,shapes,phases,rad,ground_active,prior_proof,outer_index)
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
if nargin<6,prior_proof=struct();end
if nargin<7,outer_index=0;end
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
    'torque_gradient_ms',NaN,'cached_gram_enabled',false, ...
    'gram_build_ms',NaN, ...
    'witness_transverse_enabled',false, ...
    'witness_plane_replacements',0, ...
    'witness_support_calls',0, ...
    'witness_plane_build_ms',0, ...
    'witness_last_status','NOT_REQUESTED', ...
    'coherent_cable_corridor_enabled',false, ...
    'coherent_plane_groups',0,'coherent_fallback_groups',0, ...
    'coherent_support_evaluations',0, ...
    'qp_outer_ball_enabled',false, ...
    'qp_outer_ball_groups_tested',0, ...
    'qp_outer_ball_groups_skipped',0, ...
    'qp_outer_ball_shape_radius_max_m',0, ...
    'geometry_plane_normals_3xm',zeros(3,0), ...
    'geometry_plane_thresholds_m',zeros(0,1), ...
    'geometry_plane_lambdas',zeros(0,1), ...
    'geometry_plane_phases',zeros(0,1), ...
    'geometry_plane_linearized_rhs_m',zeros(0,1));
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
% Degree 21 is representation-intrinsic. The exact Gram matrix does
% not depend on the Flight, shape, current mass, or duration; reuse it.
% Caller can explicitly switch OFF for A/B regression.
use_cached_gram=true;
if isfield(req.search_options,'use_cached_13d_bernstein_gram')
    v=req.search_options.use_cached_13d_bernstein_gram;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D_QP:GramSwitch','Gram cache must be 0/1.');
    use_cached_gram=logical(v);
end
Q.cached_gram_enabled=use_cached_gram;
tGram=tic;
B=c9_13d_gram_matrix(n,use_cached_gram);
Q.gram_build_ms=1000*toc(tGram);
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
% Optional full-time-certificate-guided normal selection. This changes
% only local search rows (not the nonlinear all-time proof). It uses an
% unresolved obstacle/body/time leaf instead of a hard-coded bypass side.
% A/B flag remains OFF until a 15-case MATLAB comparison is available.
witness_on=false;
if isfield(req.search_options,'enable_13d_witness_transverse_plane')
    v=req.search_options.enable_13d_witness_transverse_plane;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D_QP:WitnessFlag','Witness selection must be 0/1.');
    witness_on=logical(v);
end
Q.witness_transverse_enabled=witness_on;
% Acetrans-like COHERENT HALF-SPACE search: one common supporting plane
% for both cable endpoints, rather than independent potentially opposing
% normals for UAV/cable/payload.  In the straight, taut cable model,
% convexity then covers the ENTIRE cable segment.  This only makes a
% LOCAL 13D QP; all-time independent certification still decides safety.
% Explicit switch defaults OFF until the 15-scene MATLAB A/B evidence.
coherent=false;
if isfield(req.search_options,'enable_13d_coherent_cable_corridor')
    v=req.search_options.enable_13d_coherent_cable_corridor;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&&isreal(v) ...
        &&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D_QP:CoherentFlag','Coherent corridor flag must be 0/1.');
    coherent=logical(v);
end
Q.coherent_cable_corridor_enabled=coherent;
witness_u=NaN;focus_ids=[];focus_bodies=[];
if witness_on && isstruct(prior_proof)&& ...
        all(isfield(prior_proof,{'unresolved_time_interval_s', ...
            'unresolved_obstacle_ids','unresolved_body_indices'}))
    iv=double(prior_proof.unresolved_time_interval_s(:));
    if numel(iv)==2&&all(isfinite(iv))&&iv(2)>=iv(1) && ...
            iv(1)>=req.t0_s&&iv(2)<=req.t0_s+T
        witness_u=(mean(iv)-req.t0_s)/T;
        focus_ids=double(prior_proof.unresolved_obstacle_ids(:)).';
        focus_bodies=double(prior_proof.unresolved_body_indices(:)).';
    end
end
N=numel(phases);Ls=double(req.authority.cable_length_m);
delta=double(req.search_options.min_separation_m)+ ...
    double(req.geometry_options.numerical_guard_m);
% SEARCH-ONLY broadphase: if the CURRENT candidate's payload centre
% is farther than (L + largest body radius + shape outer radius + delta)
% from a shape origin at THIS probe phase, no local separation-plane row
% is needed there. This NEVER proves the UPDATED candidate safe; ALL
% observed shapes are still checked by the independent full-time oracle.
% Shape outer radii come from the canonical prepared 12-shape geometry,
% not from arbitrary obstacle-specific heuristics or precomputed masks.
qp_outer_ball=false;
if isfield(req.search_options,'enable_13d_qp_geometry_outer_ball_screen')
    v=req.search_options.enable_13d_qp_geometry_outer_ball_screen;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D_QP:OuterBallFlag','Expected boolean 0/1.');
    qp_outer_ball=logical(v);
end
Q.qp_outer_ball_enabled=qp_outer_ball;
shape_outer=zeros(1,numel(shapes));
if qp_outer_ball
    for si=1:numel(shapes)
        shape_outer(si)=c9_qp_prepared_shape_outer_radius(shapes(si));
    end
    if ~isempty(shape_outer)
        Q.qp_outer_ball_shape_radius_max_m=max(shape_outer);
    end
end
% Bounds per physical body ordered UAV / cable / payload.
A=zeros(13,3*N*(numel(shapes)+double(ground_active))+2);
b=zeros(size(A,2),1);m=0;
% Keep only the SAME fixed planes used by this local QP, so the search
% can evaluate the exact nonlinear body position before accepting a step.
% Their support values are calculated once, never again in the line search.
Ng=3*N*(numel(shapes)+double(ground_active));
gnormal=zeros(3,Ng);gthreshold=zeros(Ng,1);
glambda=zeros(Ng,1);gphase=zeros(Ng,1);grhs=zeros(Ng,1);ng=0;
for k=1:N
    pos=F.payload_m(:,k);q=F.cable_direction(:,k);
    jp=F.payload_J_fixed_phase(:,:,k);
    jq=F.cable_q_J_fixed_phase(:,:,k);
    for j=1:numel(shapes)
        shape=shapes(j);
        if qp_outer_ball
            Q.qp_outer_ball_groups_tested=Q.qp_outer_ball_groups_tested+1;
            separation=norm(pos-shape.center);
            threshold=Ls+max(rad)+delta+shape_outer(j);
            % Pure roundoff allowance, not an invented physical margin.
            scale=max([1,abs(pos(:).'),abs(shape.center(:).'), ...
                Ls,max(rad),delta,shape_outer(j)]);
            rounding_guard=64*eps(scale);
            if isfinite(separation)&&isfinite(threshold)&& ...
                    separation>threshold+rounding_guard
                Q.qp_outer_ball_groups_skipped= ...
                    Q.qp_outer_ball_groups_skipped+1;
                continue
            end
        end
        lambda=max(0,min(1,((pos-shape.center).'*q)/Ls));
        if coherent
            % Segment closest point derives the seed direction from the
            % ACTUAL nominal geometry; it is not a fixed bypass side.
            seed=pos-lambda*Ls*q;
            v=seed-shape.center;
            nv=norm(v);
            valid_normal=isfinite(nv)&&nv>0;
            if valid_normal
                shared_n=v/nv;
            else
                shared_n=zeros(3,1);
            end
            % A failed full-time proof can supply a 3D transverse guide;
            % use ONE direction for BOTH endpoints, never opposite sides.
            focused=isfinite(witness_u) && ...
                abs(phases(k)-witness_u)<=32*eps(max(1,witness_u)) ...
                &&ismember(double(shape.id),focus_ids);
            if focused || ~valid_normal
                wt=tic;
                W=TRAJECTORY_C9_13D_WITNESS_TRANSVERSE_NORMAL( ...
                    S.piece,phases(k),Ls,shape,seed,lambda, ...
                    req.authority.gravity_world_mps2,max(rad),delta,outer_index);
                Q.witness_plane_build_ms=Q.witness_plane_build_ms+1000*toc(wt);
                Q.witness_support_calls=Q.witness_support_calls+W.support_calls;
                Q.witness_last_status=W.status;
                if W.active
                    shared_n=W.normal;
                    valid_normal=true;
                    Q.witness_plane_replacements= ...
                        Q.witness_plane_replacements+1;
                end
            end
            if valid_normal && all(isfinite(shared_n))
                h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,shared_n);
                Q.coherent_support_evaluations= ...
                    Q.coherent_support_evaluations+1;
                % Effective endpoint radii also dominate the cable
                % radius. If both endpoints are safe in the SAME
                % half-space, EVERY straight cable point is safe there.
                lambda_end=[1,0];
                effective_radius=[max(rad(1),rad(2)), ...
                                  max(rad(3),rad(2))];
                for endid=1:2
                    lam=lambda_end(endid);
                    center=pos-lam*Ls*q;
                    jac=jp-lam*Ls*jq;
                    rhs=h+effective_radius(endid)+delta-shared_n.'*center;
                    grad=jac.'*shared_n;
                    ng=ng+1;
                    gnormal(:,ng)=shared_n;
                    gthreshold(ng)=h+effective_radius(endid)+delta;
                    glambda(ng)=lam;
                    gphase(ng)=phases(k);
                    grhs(ng)=rhs;
                    [A,b,m,Q]=append_plane(A,b,m,Q,grad,rhs,'shape');
                    if Q.unmovable_count>0
                        Q.status='UNKNOWN_13D_UNMOVABLE_COHERENT_PLANE';
                        Q.elapsed_ms=1000*toc(clock);return
                    end
                end
                Q.coherent_plane_groups=Q.coherent_plane_groups+1;
                continue
            end
            % SO(3)-degenerate cases are not assigned an arbitrary side:
            % the legacy three-body planes remain a search fallback.
            Q.coherent_fallback_groups=Q.coherent_fallback_groups+1;
        end
        frac=[1,lambda,0];
        for body=1:3
            lam=frac(body);
            center=pos-lam*Ls*q;
            jac=jp-lam*Ls*jq;
            nvec=center-shape.center;
            if isfinite(witness_u)&& ...
                    abs(phases(k)-witness_u)<=32*eps(max(1,witness_u)) && ...
                    ismember(double(shape.id),focus_ids)&&ismember(body,focus_bodies)
                wt=tic;
                W=TRAJECTORY_C9_13D_WITNESS_TRANSVERSE_NORMAL( ...
                    S.piece,phases(k),Ls,shape,center,lam, ...
                    req.authority.gravity_world_mps2,rad(body),delta,outer_index);
                Q.witness_plane_build_ms=Q.witness_plane_build_ms+1000*toc(wt);
                Q.witness_support_calls=Q.witness_support_calls+W.support_calls;
                Q.witness_last_status=W.status;
                if W.active
                    nvec=W.normal;
                    Q.witness_plane_replacements=Q.witness_plane_replacements+1;
                end
            end
            if ~all(isfinite(nvec)) || norm(nvec)==0
                Q.status='UNKNOWN_13D_PLANE_DIRECTION_UNDEFINED';
                Q.elapsed_ms=1000*toc(clock);return
            end
            nvec=nvec/norm(nvec);
            h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,nvec);
            rhs=h+rad(body)+delta-nvec.'*center;
            grad=jac.'*nvec;
            ng=ng+1;
            gnormal(:,ng)=nvec;gthreshold(ng)=h+rad(body)+delta;
            glambda(ng)=lam;gphase(ng)=phases(k);grhs(ng)=rhs;
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
            ng=ng+1;
            gnormal(:,ng)=[0;0;1];gthreshold(ng)=rad(body)+delta;
            glambda(ng)=lam;gphase(ng)=phases(k);grhs(ng)=rhs;
            [A,b,m,Q]=append_plane(A,b,m,Q,grad,rhs,'ground');
            if Q.unmovable_count>0
                Q.status='UNKNOWN_13D_UNMOVABLE_GROUND_PLANE';
                Q.elapsed_ms=1000*toc(clock);return
            end
        end
    end
end
Q.geometry_plane_normals_3xm=gnormal(:,1:ng);
Q.geometry_plane_thresholds_m=gthreshold(1:ng);
Q.geometry_plane_lambdas=glambda(1:ng);
Q.geometry_plane_phases=gphase(1:ng);
Q.geometry_plane_linearized_rhs_m=grhs(1:ng);
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

function B=c9_13d_gram_matrix(n,use_cache)
% Backwards-compatible local wrapper. The identical degree-only matrix now
% lives in a separate persistent helper that preflight can prepare WITHOUT
% building a Flight candidate or reading its observations/estimated mass.
B=TRAJECTORY_C9_13D_STATIC_GRAM(n,use_cache);
end

function r=c9_qp_prepared_shape_outer_radius(S)
% Bound of the true canonical source shape about its pose origin. The
% oracle's broadphase uses the SAME formulas; only LOCAL QP rows are
% suppressed here, never full-time certification. No runtime shape slot.
if ~isempty(S.local_vertices)
    r=max(sqrt(sum(double(S.local_vertices).^2,2)));
else
    d=S.dim;
    switch S.kind
        case 'sphere',r=double(d(1));
        case 'ellipsoid',r=max(double(d(:)));
        case 'box',r=norm(double(d(:)))/2;
        case 'cylinder',r=hypot(double(d(1)),double(d(2))/2);
        case 'capsule',r=double(d(1))+double(d(2))/2;
        case 'cone',r=hypot(double(d(1)),double(d(2))/2);
        case 'frustum'
            r=max(hypot(double(d(1)),double(d(3))/2), ...
                hypot(double(d(2)),double(d(3))/2));
        case 'torus',r=double(d(1))+double(d(2));
        otherwise
            error('C9_13D_QP:OuterShape', ...
                'Unvalidated shape kind must never be screened');
    end
end
assert(isfinite(r)&&r>=0,'C9_13D_QP:OuterShape', ...
    'Shape outer radius must be finite nonnegative');
end
