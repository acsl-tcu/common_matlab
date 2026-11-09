function R=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P)
%TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE  Tube-inflated existing C9 corridor.
% C is an ALREADY nominally certified FAST_SHARED_CORRIDOR witness.  On
% each certified dyadic leaf we reconstruct Bernstein payload controls,
% translate the UAV/cable hull by the saved taut direction, and require
% strict clearance beyond the extra physical execution error of P.
% All known obstacles are tested independently for each body. Ground
% z<=0 is separately required in Flight. No new avoidance direction or
% artificially inflated system-sized sphere is introduced.
%
% This is an exact REAL-ARITHMETIC conditional implication. The corridor
% is evaluated in ordinary doubles, not directed-rounded interval math;
% EKF/plant error, mass, tautness, input and numerical rounding remain
% external obligations. Therefore no real-Flight COMMIT is ever granted.
R=struct('status','UNKNOWN_HLC_CORRIDOR_TUBE_NOT_EVALUATED', ...
    'safe_to_commit',false,'real_HLC_geometry_proven',false, ...
    'nominal_corridor_witness_checked',false, ...
    'conditional_all_body_separated',false, ...
    'min_obstacle_residual_m',NaN,'min_ground_residual_m',NaN, ...
    'min_total_residual_m',NaN, ...
    'recomputed_nominal_w_lower_mps2',NaN, ...
    'w_lower_method','BERNSTEIN_BALL_AND_SUPPORTING_PROJECTION', ...
    'checked_leaves',0, ...
    'checked_obstacle_body_pairs',0, ...
    'affine_cable_execution_tube_used',false, ...
    'failure_leaf_index',NaN,'failure_body_index',NaN, ...
    'failure_obstacle_index',NaN, ...
    'scope','CONDITIONAL_ALL_TIME_BODY_TUBE_IN_EXISTING_CORRIDOR__NOT_REAL_HLC');
requiredC={'certified','leaves','guide_piece','radii','authority', ...
    'obstacle_ids','ground_applicable','proof_options'};
requiredP={'status','uav_m','payload_m','cable_m'};
if ~(isstruct(C)&&isscalar(C)&&all(isfield(C,requiredC))&& ...
        isequal(C.certified,true)&& ...
        isstruct(P)&&isscalar(P)&&all(isfield(P,requiredP))&& ...
        strcmp(P.status, ...
        'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN'))
    R.status='UNKNOWN_NOMINAL_CORRIDOR_OR_EXECUTION_TUBE_MISSING';return
end
if ~isstruct(C.guide_piece)|| ...
        ~all(isfield(C.guide_piece,{'control_points_m','t0_s','duration_s'}))|| ...
        ~isequal(size(C.guide_piece.control_points_m),[3 24])|| ...
        ~isstruct(C.radii)||~isstruct(C.authority)|| ...
        ~all(isfield(C.radii,{'uav_m','payload_m','cable_m'}))|| ...
        ~isfield(C.authority,'cable_length_m')|| ...
        ~isfield(C.proof_options,'robust_tube_enabled')
    R.status='UNKNOWN_CORRIDOR_WITNESS_DIMENSIONS';return
end
B=double(C.guide_piece.control_points_m);
t0=double(C.guide_piece.t0_s);
T=double(C.guide_piece.duration_s);
L=double(C.authority.cable_length_m);
tube=double([P.uav_m,P.payload_m,P.cable_m]);
rad=double([C.radii.uav_m,C.radii.payload_m,C.radii.cable_m]);
if ~(isscalar(t0)&&isscalar(T)&&isscalar(L)&& ...
        isfinite(t0)&&isfinite(T)&&T>0&&isfinite(L)&&L>0&& ...
        all(isfinite(B(:)))&&all(isfinite(tube))&&all(tube>=0)&& ...
        all(isfinite(rad))&&all(rad>=0))
    R.status='UNKNOWN_NONFINITE_CORRIDOR_OR_TUBE';return
end
if isequal(double(C.proof_options.robust_tube_enabled),1)
    if ~isfield(C.proof_options,'uncertainty_bounds_m')
        R.status='UNKNOWN_CORRIDOR_PRIOR_TUBE_MISSING';return
    end
    U=C.proof_options.uncertainty_bounds_m;
    if ~isstruct(U)||~all(isfield(U,{'uav_m','payload_m','cable_m'}))
        R.status='UNKNOWN_CORRIDOR_PRIOR_TUBE_INVALID';return
    end
    u=double([U.uav_m,U.payload_m,U.cable_m]);
    if ~all(isfinite(u))||any(u<0)
        R.status='UNKNOWN_CORRIDOR_PRIOR_TUBE_INVALID';return
    end
    rad=rad+u;
elseif ~isequal(double(C.proof_options.robust_tube_enabled),0)
    R.status='UNKNOWN_CORRIDOR_TUBE_SWITCH_INVALID';return
end
% With taut straight cable, the execution error at fraction lam
% is a convex combination of the PAYLOAD and UAV endpoint errors.
% The minimum of the normal clearance minus that AFFINE error radius
% over lam in [0,1] occurs at an endpoint; no cable discretization.
% Optional A/B flag: legacy maximally inflated whole-cable ball remains
% the default until the current MATLAB implementation is audited.
use_affine=false;
if isfield(C.proof_options,'use_affine_cable_execution_tube')
    v=C.proof_options.use_affine_cable_execution_tube;
    if ~((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]))
        R.status='UNKNOWN_AFFINE_CABLE_FLAG_INVALID';return
    end
    use_affine=logical(v);
end
R.affine_cable_execution_tube_used=use_affine;
% A separately certified additional cable error must not disappear.
% P.cable_m is an all-cable bound; if larger than the endpoint maximum,
% keep the excess at BOTH endpoints.
extra_cable=max(0,tube(3)-max(tube(1:2)));
nObs=numel(C.obstacle_ids);
if ~(isnumeric(C.obstacle_ids)&&all(isfinite(C.obstacle_ids(:)))&& ...
        numel(unique(C.obstacle_ids))==nObs&& ...
        isstruct(C.leaves)&&~isempty(C.leaves))
    R.status='UNKNOWN_CORRIDOR_LEAVES_OR_OBSTACLE_IDS';return
end
marginObs=Inf;marginGround=Inf;wLower=Inf;
lastEnd=t0;
for k=1:numel(C.leaves)
    leaf=C.leaves(k);
    fields={'time_interval_s','map','q0','guide_cone_padding_m', ...
        'uav_normals','payload_normals','cable_normals', ...
        'uav_offsets','payload_offsets','cable_offsets'};
    if ~all(isfield(leaf,fields))|| ...
            ~isequal(size(leaf.map),[24 24])|| ...
            ~isequal(size(leaf.time_interval_s),[1 2])|| ...
            ~isequal(size(leaf.q0),[3 1])
        R.status='UNKNOWN_LEAF_WITNESS_INCOMPLETE';return
    end
    inter=double(leaf.time_interval_s);
    if any(~isfinite(inter))||inter(2)<=inter(1)|| ...
            inter(1)>lastEnd||inter(2)<=lastEnd|| ...
            (k==1&&inter(1)>t0)|| ...
            (k==numel(C.leaves)&&inter(2)<t0+T)
        R.status='UNKNOWN_CORRIDOR_LEAF_TIME_COVERAGE';return
    end
    lastEnd=inter(2);
    q=double(leaf.q0(:));pad=double(leaf.guide_cone_padding_m);
    if ~all(isfinite(leaf.map(:)))||~all(isfinite(q))|| ...
            ~isfinite(pad)||pad<0||abs(norm(q)-1)>32*eps(1)
        R.status='UNKNOWN_CORRIDOR_LEAF_DIRECTION';return
    end
    H=B*double(leaf.map);
    if ~isfield(C.authority,'gravity_world_mps2')|| ...
            ~isequal(size(C.authority.gravity_world_mps2),[3 1])
        R.status='UNKNOWN_GRAVITY_SOURCE_FOR_CABLE_FORCE';return
    end
    g=double(C.authority.gravity_world_mps2(:));
    dtime=inter(2)-inter(1);
    W=repmat(g,1,22)-(23*22/dtime^2)*diff(H,2,2);
    % Bernstein convex hull: W(t) belongs to conv(W(:,i)) for ALL t.
    % For every unit vector n, ||W(t)|| >= n'*W(t) >= min_i(n'*W_i).
    % Selecting n=mean(W_i)/||mean(W_i)|| is an algebraic certificate
    % direction, NOT a fixed bypass direction or an empirical threshold.
    % The previous enclosing-ball bound is retained; taking max of two
    % PROVEN lower bounds can never weaken the certificate.
    wcenter=mean(W,2);
    wnorm=norm(wcenter);
    wrad=max(sqrt(sum((W-wcenter).^2,1)));
    ball_lower=wnorm-wrad;
    projection_lower=-Inf;
    if wnorm>0
        projection_lower=min((wcenter.'*W)/wnorm);
    end
    wl=max(ball_lower,projection_lower);
    if ~(all(isfinite(W(:)))&&isfinite(wl)&&wl>0)
        R.status='UNKNOWN_CORRIDOR_NOMINAL_CABLE_FORCE';return
    end
    wLower=min(wLower,wl);
    hull={H-L*q,H,[H,H-L*q]};
    normals={leaf.uav_normals,leaf.payload_normals,leaf.cable_normals};
    offsets={leaf.uav_offsets,leaf.payload_offsets,leaf.cable_offsets};
    for body=1:3
        N=double(normals{body});b=double(offsets{body}(:));
        if ~isequal(size(N),[3 nObs])||numel(b)~=nObs|| ...
                any(~isfinite(N(:)))||any(~isfinite(b))
            R.status='UNKNOWN_CORRIDOR_PLANE_DIMENSIONS';return
        end
        for j=1:nObs
            n=N(:,j);nn=norm(n);
            if ~(isfinite(nn)&&nn>0)
                R.status='UNKNOWN_NONUNIT_CORRIDOR_NORMAL';return
            end
            % For normalized n, the support of an execution error ball
            % is ||n||*tube(body). The original corridor offset already
            % includes nominal geometry radii, cone padding and guards.
            if body==3 && use_affine
                % H=payload nominal, H-L*q=UAV nominal on this leaf.
                % Every cable fraction is an affine interpolation of its
                % physical endpoints, so the minimum of the linear
                % certificate is EXACTLY the smaller endpoint margin.
                ep=tube(2)+extra_cable;
                eu=tube(1)+extra_cable;
                slack=min(min(n.'*H)-nn*ep, ...
                          min(n.'*(H-L*q))-nn*eu)-b(j);
            else
                slack=min(n.'*hull{body})-b(j)-nn*tube(body);
            end
            if ~isfinite(slack)||slack<=0
                R.status='UNKNOWN_TUBE_OUTSIDE_OBSTACLE_HALFSPACE';
                R.failure_leaf_index=k;R.failure_body_index=body;
                R.failure_obstacle_index=j;return
            end
            marginObs=min(marginObs,slack/nn);
            R.checked_obstacle_body_pairs=R.checked_obstacle_body_pairs+1;
        end
    end
    if isequal(C.ground_applicable,true)
        for body=1:3
            extra=double(body==1||body==3)*pad;
            % Entire actual spherical/cylindrical body stays strictly z>0.
            % Here the original geometric radius and any prior corridor
            % tube are included; execution tube is added separately.
            if body==3 && use_affine
                ep=tube(2)+extra_cable;
                eu=tube(1)+extra_cable;
                slack=min(min(H(3,:))-ep, ...
                          min(H(3,:)-L*q(3))-eu)-extra-rad(body);
            else
                slack=min(hull{body}(3,:))-extra-rad(body)-tube(body);
            end
            if ~isfinite(slack)||slack<=0
                R.status='UNKNOWN_TUBE_INTERSECTS_GROUND';
                R.failure_leaf_index=k;R.failure_body_index=body;return
            end
            marginGround=min(marginGround,slack);
        end
    elseif ~isequal(C.ground_applicable,false)
        R.status='UNKNOWN_CORRIDOR_GROUND_SCOPE';return
    end
    R.checked_leaves=k;
end
% The bridge must not allow a caller to assert a larger lower bound on
% ||g-pL''|| than this reconstructed Bernstein lower bound.  That would
% shrink the claimed physical direction-error tube without evidence.
if ~isfield(P,'nominal_w_lower_mps2')|| ...
        ~isnumeric(P.nominal_w_lower_mps2)|| ...
        ~isscalar(P.nominal_w_lower_mps2)|| ...
        ~isfinite(P.nominal_w_lower_mps2)|| ...
        P.nominal_w_lower_mps2>wLower
    R.status='UNKNOWN_HLC_NOMINAL_W_BOUND_EXCEEDS_CORRIDOR_PROOF';return
end
R.recomputed_nominal_w_lower_mps2=wLower;
R.nominal_corridor_witness_checked=true;
R.min_obstacle_residual_m=marginObs;
R.min_ground_residual_m=marginGround;
R.min_total_residual_m=min(marginObs,marginGround);
R.conditional_all_body_separated=true;
R.status='CONDITIONAL_ALL_BODY_EXECUTION_TUBE_INSIDE_C9_CORRIDOR__NOT_REAL_HLC_PROOF';
end
