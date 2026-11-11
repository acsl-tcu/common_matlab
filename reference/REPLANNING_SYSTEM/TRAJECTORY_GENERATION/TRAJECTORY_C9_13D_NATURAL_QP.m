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
    'torque_gradient_ms',NaN,'cached_gram_enabled',false,'shared_jet_enabled',false, ...
    'shared_jet_executed',false, ...
    'native_torque_batch_executed',false,'native_torque_batch_ms',0, ...
    'batched_geometry_requested',false,'batched_geometry_executed',false, ...
    'batched_geometry_ms',0,'batched_geometry_row_count',0, ...
    'certificate_leaf_cut_enabled',false,'certificate_leaf_cut_count',0, ...
    'certificate_leaf_cut_support_calls',0,'certificate_leaf_cut_status','NOT_REQUESTED', ...
    'certificate_leaf_physical_duration_tangent',false, ...
    'certificate_leaf_payload_witness_gate',false, ...
    'oracle_body_time_focus_requested',false, ...
    'oracle_body_time_focus_active',false, ...
    'oracle_body_time_focus_skipped_shape_phase_groups',0, ...
    'oracle_body_time_focus_skipped_body_planes',0, ...
    'oracle_body_time_focus_eligible_shape_phase_groups',0, ...
    'oracle_body_time_outer_guard_requested',false, ...
    'oracle_body_time_outer_guard_checked',false, ...
    'oracle_body_time_outer_guard_rejected',false, ...
    'oracle_body_time_outer_guard_far_body_checks',0, ...
    'oracle_body_time_outer_guard_near_body_checks',0, ...
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
% The A/B option is read BEFORE the sensitivity kernel so that the
% legacy OFF path allocates no extra cached basis matrices at all.
share_jets=false;
if isfield(req.search_options,'enable_13d_shared_jet_kernel')
    flag=req.search_options.enable_13d_shared_jet_kernel;
    assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
        isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
        'C9_13D_QP:SharedJetFlag','Shared-jet switch must be 0/1.');
    share_jets=logical(flag);
end
share_jets=share_jets&&share;
Q.shared_jet_enabled=share_jets;
t_s=tic;
S=TRAJECTORY_C9_DURATION_SENSITIVITY(req.t0_s,T, ...
    req.start_jets,req.end_jets,free,phases,share_jets);
Q.duration_sensitivity_ms=1000*toc(t_s);
S.enable_13d_shared_basis=share_jets;
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
% Search-only proof-witness focus. If the last UNFILTERED oracle reports
% an exact unresolved obstacle/body/physical-time leaf, build shape
% separation rows only for that witness. This removes stale safe-body
% planes from a single simultaneous QP, NOT from the safety certificate.
% Ground and torque rows remain unconditional. Never apply it to a
% geometry-PASS/torque-failure oracle or a coherent shared-endpoint plane.
focus_requested=false;
if isfield(req.search_options,'enable_13d_oracle_body_time_qp_rows')
    v=req.search_options.enable_13d_oracle_body_time_qp_rows;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'C9_13D_QP:BodyTimeFocusFlag','Body/time focus must be 0/1.');
    focus_requested=logical(v);
end
Q.oracle_body_time_focus_requested=focus_requested;
focus_active=false;focus_shape_ids=[];focus_body_ids=[];
focus_leaf=[NaN NaN];focus_phase_mask=false(1,N);
if focus_requested && ~coherent && ~isempty(shapes) && ...
        isstruct(prior_proof)&&isscalar(prior_proof)&& ...
        isfield(prior_proof,'status')&& ...
        strcmp(prior_proof.status,'UNKNOWN_JOINT_GEOMETRY_NOT_SEPARATED')&& ...
        all(isfield(prior_proof,{'unresolved_obstacle_ids', ...
            'unresolved_body_indices','unresolved_time_interval_s'}))
    ids=double(prior_proof.unresolved_obstacle_ids(:)).';
    bodies=double(prior_proof.unresolved_body_indices(:)).';
    leaf=double(prior_proof.unresolved_time_interval_s(:)).';
    available=double([shapes.id]);
    valid=~isempty(ids)&&all(isfinite(ids))&& ...
        all(ismember(ids,available)) && ...
        ~isempty(bodies)&&all(ismember(bodies,[1 2 3])) && ...
        numel(leaf)==2&&all(isfinite(leaf))&&leaf(1)<=leaf(2)&& ...
        leaf(1)>=req.t0_s&&leaf(2)<=req.t0_s+T;
    if valid
        times=req.t0_s+T*double(phases(:).');
        scale=max([1,abs(times),abs(leaf)]);
        guard=128*eps(max(scale)); % roundoff-only overlap tolerance
        probe=(times>=leaf(1)-guard)&(times<=leaf(2)+guard);
        if any(probe)
            focus_shape_ids=unique(ids,'stable');
            focus_body_ids=unique(bodies,'stable');
            focus_leaf=leaf;
            focus_phase_mask=probe;
            focus_active=true;
        end
    end
end
% The independent oracle only identifies one UNRESOLVED leaf. It does
% NOT certify that the other bodies/times may be discarded from a local QP.
% If enabled, keep the aggressive focus ONLY when every omitted CURRENT
% body/shape/phase pair is strictly outside a mathematically enclosing
% obstacle sphere. This is an A/B SEARCH guard, NEVER an all-time proof.
% If ANY omitted body cannot be screened, immediately restore ALL local
% body/time rows BEFORE the QP is built (no extra search stage, no second
% oracle). It uses no fixed detour side or empirical geometric threshold.
outer_guard=false;
if isfield(req.search_options,'enable_13d_oracle_body_time_outer_guard')
    sw=req.search_options.enable_13d_oracle_body_time_outer_guard;
    assert((isnumeric(sw)||islogical(sw))&&isscalar(sw)&& ...
        isreal(sw)&&isfinite(double(sw))&&any(double(sw)==[0 1]), ...
        'C9_13D_QP:BodyTimeOuterGuard','Expected 0/1 outer guard.');
    outer_guard=logical(sw);
end
Q.oracle_body_time_outer_guard_requested=outer_guard;
if outer_guard && focus_active
    Q.oracle_body_time_outer_guard_checked=true;
    % The existing canonical shape-radius formulas encompass the true 3D
    % geometry (including arbitrary SO(3) orientation). No whole-system
    % enclosing sphere is used as a final collision certificate.
    if ~qp_outer_ball
        for sj=1:numel(shapes)
            shape_outer(sj)=c9_qp_prepared_shape_outer_radius(shapes(sj));
        end
    end
    reject=false;
    for k_guard=1:N
        p_guard=F.payload_m(:,k_guard);
        q_guard=F.cable_direction(:,k_guard);
        for sj=1:numel(shapes)
            shape_guard=shapes(sj);
            in_leaf=focus_phase_mask(k_guard)&& ...
                ismember(double(shape_guard.id),focus_shape_ids);
            if in_leaf
                omitted_body=find(~ismember(1:3,focus_body_ids));
            else
                omitted_body=1:3;
            end
            if isempty(omitted_body),continue;end
            v_guard=p_guard-double(shape_guard.center(:));
            lambda_guard=max(0,min(1,(v_guard.'*q_guard)/Ls));
            cable_lambdas=[1,lambda_guard,0];
            for b_guard=omitted_body
                center_guard=p_guard-Ls*cable_lambdas(b_guard)*q_guard;
                distance_guard=norm(center_guard-shape_guard.center(:));
                threshold_guard=shape_outer(sj)+rad(b_guard)+delta;
                % Roundoff allowance for a SEARCH decision only; the
                % final full-time oracle independently rechecks everything.
                magnitude_guard=max([1;abs(center_guard(:)); ...
                    abs(shape_guard.center(:));abs(shape_outer(sj)); ...
                    abs(rad(b_guard));abs(delta)]);
                rounding_guard=128*eps(magnitude_guard);
                if ~(isfinite(distance_guard)&&isfinite(threshold_guard)&& ...
                        distance_guard>threshold_guard+rounding_guard)
                    Q.oracle_body_time_outer_guard_near_body_checks= ...
                        Q.oracle_body_time_outer_guard_near_body_checks+1;
                    reject=true;break
                end
                Q.oracle_body_time_outer_guard_far_body_checks= ...
                    Q.oracle_body_time_outer_guard_far_body_checks+1;
            end
            if reject,break;end
        end
        if reject,break;end
    end
    if reject
        focus_active=false;
        Q.oracle_body_time_outer_guard_rejected=true;
    end
end
Q.oracle_body_time_focus_active=focus_active;
% Structural performance path: replace O(probes * shapes * bodies) MATLAB
% calls by one batched all-body geometry/13D-Jacobian assembly. This is an
% algebraic rearrangement of the SAME search planes, not a collision or
% input certificate. Other explicit witness/coherent/focused strategies
% retain their original code and do not silently change behaviour.
batch_planes=false;
if isfield(req.search_options,'enable_13d_batched_geometry_planes')
    flag=req.search_options.enable_13d_batched_geometry_planes;
    assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
        isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
        'C9_13D_QP:BatchFlag','Batch switch must be 0/1.');
    batch_planes=logical(flag);
end
Q.batched_geometry_requested=batch_planes;
if batch_planes && ~witness_on && ~coherent && ~focus_active
    BB=TRAJECTORY_C9_13D_BATCH_GEOMETRY_PLANES( ...
        F,shapes,phases,rad,Ls,delta,ground_active, ...
        qp_outer_ball,shape_outer);
    Q.batched_geometry_ms=BB.elapsed_ms;
    if ~strcmp(BB.status,'LOCAL_BATCH_GEOMETRY_PLANES_READY__NOT_CERTIFIED')
        Q.status=BB.status;Q.elapsed_ms=1000*toc(clock);return
    end
    Q.batched_geometry_executed=true;
    Q.batched_geometry_row_count=size(BB.A,2);
    Q.qp_outer_ball_groups_tested=BB.outer_groups_tested;
    Q.qp_outer_ball_groups_skipped=BB.outer_groups_skipped;
    Q.obstacle_plane_count=BB.obstacle_count;
    Q.ground_plane_count=BB.ground_count;
    m=size(BB.A,2);
    A(:,1:m)=BB.A;b(1:m)=BB.b;
    ng=numel(BB.lambda);
    gnormal(:,1:ng)=BB.normal;
    gthreshold(1:ng)=BB.threshold;
    glambda(1:ng)=BB.lambda;
    gphase(1:ng)=BB.phase;
    grhs(1:ng)=BB.rhs;
else
for k=1:N
    pos=F.payload_m(:,k);q=F.cable_direction(:,k);
    jp=F.payload_J_fixed_phase(:,:,k);
    jq=F.cable_q_J_fixed_phase(:,:,k);
    for j=1:numel(shapes)
        shape=shapes(j);
        if focus_active && (~focus_phase_mask(k) || ...
                ~ismember(double(shape.id),focus_shape_ids))
            Q.oracle_body_time_focus_skipped_shape_phase_groups= ...
                Q.oracle_body_time_focus_skipped_shape_phase_groups+1;
            continue
        end
        if focus_active
            Q.oracle_body_time_focus_eligible_shape_phase_groups= ...
                Q.oracle_body_time_focus_eligible_shape_phase_groups+1;
        end
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
            if focus_active && ~ismember(body,focus_body_ids)
                Q.oracle_body_time_focus_skipped_body_planes= ...
                    Q.oracle_body_time_focus_skipped_body_planes+1;
                continue
            end
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
end % batched QP search planes versus legacy per-shape/per-phase loops
Q.geometry_plane_normals_3xm=gnormal(:,1:ng);
Q.geometry_plane_thresholds_m=gthreshold(1:ng);
Q.geometry_plane_lambdas=glambda(1:ng);
Q.geometry_plane_phases=gphase(1:ng);
Q.geometry_plane_linearized_rhs_m=grhs(1:ng);
% OPTIONAL: use a failed ALL-TIME proof's actual Bernstein leaf to form
% additional 13D payload-hull SEARCH rows in THIS SAME simultaneous QP.
% This targets all-time hull failure instead of a point-only witness.
% Final candidate safety remains the independent unfiltered oracle.
add_leaf=false;
if isfield(req.search_options,'enable_13d_certificate_leaf_hull_cuts')
    flag=req.search_options.enable_13d_certificate_leaf_hull_cuts;
    assert((isnumeric(flag)||islogical(flag))&&isscalar(flag)&& ...
        isreal(flag)&&isfinite(double(flag))&&any(double(flag)==[0 1]), ...
        'C9_13D_QP:LeafCutFlag','Leaf cut mode must be 0/1.');
    add_leaf=logical(flag);
end
Q.certificate_leaf_cut_enabled=add_leaf;
if add_leaf && share && ~isempty(shapes)
    % Fixed physical-time tangent corrects the moving leaf endpoints as
    % duration changes. Explicit OFF is retained for previous A/B tests.
    use_physical_leaf=false; % Preserve previous explicit leaf-ON A/B unless opted in.
    if isfield(req.search_options,'enable_13d_physical_leaf_duration_tangent')
        v=req.search_options.enable_13d_physical_leaf_duration_tangent;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_13D_QP:PhysicalLeafFlag','Physical leaf derivative must be 0/1.');
        use_physical_leaf=logical(v);
    end
    Q.certificate_leaf_physical_duration_tangent=use_physical_leaf;
    payload_witness_gate=false; % Backward-compatible; focused A/B explicitly ON.
    if isfield(req.search_options,'enable_13d_payload_witness_leaf_gate')
        v=req.search_options.enable_13d_payload_witness_leaf_gate;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_13D_QP:PayloadWitnessFlag','Payload witness gate must be 0/1.');
        payload_witness_gate=logical(v);
    end
    Q.certificate_leaf_payload_witness_gate=payload_witness_gate;
    [LA,Lb,LD]=TRAJECTORY_C9_13D_CERTIFICATE_LEAF_CUTS( ...
        S.piece,S,shapes,prior_proof,rad(3),delta, ...
        use_physical_leaf,payload_witness_gate);
    Q.certificate_leaf_cut_status=LD.status;
    Q.certificate_leaf_cut_support_calls=LD.support_calls;
    for rowidx=1:numel(Lb)
        [A,b,m,Q]=append_plane(A,b,m,Q,LA(:,rowidx),Lb(rowidx),'shape');
        if Q.unmovable_count>0
            Q.status='UNKNOWN_13D_IMMOVABLE_CERTIFICATE_LEAF_CUT';
            Q.elapsed_ms=1000*toc(clock);return
        end
        Q.certificate_leaf_cut_count=Q.certificate_leaf_cut_count+1;
    end
elseif add_leaf
    Q.certificate_leaf_cut_status='NOT_APPLICABLE_NO_SHARED_CACHE_OR_SHAPES';
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
torque_options=req.torque_options;
if share
    torque_options.use_13d_shared_jet_kernel=share_jets;
    Z=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_13D( ...
        S.piece,req.authority,torque_options,phases,S);
else
    Z=TRAJECTORY_C9_TORQUE_FORWARD_JACOBIAN_13D( ...
        S.piece,req.authority,req.torque_options,phases);
end
Q.torque_gradient_ms=1000*toc(t_grad);
Q.native_torque_batch_executed=isfield(Z,'native_torque_batch_executed')&& ...
    isequal(Z.native_torque_batch_executed,true);
if isfield(Z,'native_torque_batch_ms')
    Q.native_torque_batch_ms=Z.native_torque_batch_ms;
end
Q.shared_jet_executed=isfield(Z,'shared_jet_enabled')&& ...
    isequal(Z.shared_jet_enabled,true);
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
