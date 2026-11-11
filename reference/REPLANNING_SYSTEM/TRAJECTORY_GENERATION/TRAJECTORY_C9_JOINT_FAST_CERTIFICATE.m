function [R,prepared_shapes]=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE(piece,observed,radii,authority,geometry_options,force_options)
% TRAJECTORY_C9_JOINT_FAST_CERTIFICATE
% One-pass SUFFICIENT continuous-time certificate, ideal taut-load model.
% Tension, thrust-magnitude, and UAV/cable/payload geometry are proved
% from ONE set of payload Bernstein controls per time leaf (degree 23 or 27).
%
%   w=g-pL'', q=w/||w||, pU=pL-L*q, cable=pL-lambda*L*q.
% If the Bernstein hull of w lies within B(c,rho), ||c||>rho, then
% ||q-c/||c|||| <= chord(rho/||c||).  On the leaf, the complete
% cable/UAV/payload occupancy is enclosed by separate shifted payload
% control-point convex hulls with L*chord direction padding (only for
% cable and UAV). Shape exclusion is certified by exact convex-hull
% support, for each of the 12 OBSERVED shape families and arbitrary SO(3).
% The known infinite ground z=0 is checked on EVERY Flight leaf only
% (explicit non-f phases exclude it), with independent UAV/payload/cable
% radii. The internal rr indexing is UAV,CABLE,PAYLOAD; the ground-leaf
% API uses UAV,PAYLOAD,CABLE. It is NOT a sensor obstacle.
% The first stage is an obstacle-centred outer-ball REJECTION ONLY. It
% cannot declare a collision, produce a corridor, or enlarge a detour.
% The second stage uses actual directed separating planes of the 3 hulls.
% A failure to find planes returns UNKNOWN; time subdivision may sharpen
% the hull but a safety claim is NEVER made by sampling.
%
% Bounds for nominal force:  F=-(mU+mL)*w - mU*L*q''.
%    ||q''|| <= ||w''||/s + 3*(||w'||/s)^2, s=||c||-rho.
% Hence triangle/reverse-triangle bounds certify all-time thrust magnitude.
% This is a sufficient, not necessary, condition. Floating-point rounding
% is NOT interval-arithmetic certified. Caller supplies numerical guard.
% NO HLC commit, torque, tracking or real ODE execution certificate.
start=tic;
assert(isstruct(piece)&&isscalar(piece)&& ...
    all(isfield(piece,{'control_points_m','t0_s','duration_s'})) && ...
    size(piece.control_points_m,1)==3&& ...
    any(size(piece.control_points_m,2)==[24 28])&& ...
    isnumeric(piece.control_points_m)&&isreal(piece.control_points_m)&& ...
    all(isfinite(piece.control_points_m(:)))&& ...
    isnumeric(piece.duration_s)&&isreal(piece.duration_s)&& ...
    isscalar(piece.duration_s)&&isfinite(piece.duration_s)&& ...
    piece.duration_s>0&&isnumeric(piece.t0_s)&& ...
    isscalar(piece.t0_s)&&isreal(piece.t0_s)&& ...
    isfinite(piece.t0_s)&&isfinite(piece.t0_s+piece.duration_s), ...
    'TRAJECTORY_C9_JOINT:Piece','Require finite 3D degree-23 or degree-27 load piece.');
assert(isstruct(observed), 'TRAJECTORY_C9_JOINT:Observed', ...
    'Only explicitly observed static obstacles may be checked.');
assert(isstruct(authority)&&isscalar(authority)&& ...
    all(isfield(authority,{'cable_length_m','payload_mass_kg', ...
        'uav_mass_kg','gravity_world_mps2','thrust_limits_N', ...
        'minimum_tension_N','acceleration_guard_mps2'})), ...
    'TRAJECTORY_C9_JOINT:Authority','All physical authority is external.');
for key={'cable_length_m','payload_mass_kg','uav_mass_kg'}
    x=authority.(key{1});
    assert(isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&x>0, ...
        'TRAJECTORY_C9_JOINT:Authority','Positive live physical authority required.');
end
for key={'minimum_tension_N','acceleration_guard_mps2'}
    x=authority.(key{1});
    assert(isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&x>=0, ...
        'TRAJECTORY_C9_JOINT:Authority','Nonnegative live authority required.');
end
g=double(authority.gravity_world_mps2(:));
f=double(authority.thrust_limits_N(:));
assert(isequal(size(authority.gravity_world_mps2),[3 1])&& ...
    isnumeric(authority.gravity_world_mps2)&& ...
    isreal(authority.gravity_world_mps2)&&all(isfinite(g))&& ...
    isnumeric(authority.thrust_limits_N)&& ...
    isreal(authority.thrust_limits_N)&&numel(f)==2&& ...
    all(isfinite(f))&&f(1)>=0&&f(2)>=f(1), ...
    'TRAJECTORY_C9_JOINT:Authority','Invalid gravity/thrust authority.');
assert(isstruct(geometry_options)&&isscalar(geometry_options)&& ...
    all(isfield(geometry_options,{'max_subdivision_depth', ...
        'max_node_count','numerical_guard_m','robust_tube_enabled'})), ...
    'TRAJECTORY_C9_JOINT:Options','Explicit geometry proof options required.');
assert(isstruct(force_options)&&isscalar(force_options)&& ...
    all(isfield(force_options,{'max_subdivision_depth', ...
        'max_node_count','force_guard_N'})), ...
    'TRAJECTORY_C9_JOINT:Options','Explicit force proof options required.');
for v=[geometry_options.max_subdivision_depth, ...
        geometry_options.max_node_count,force_options.max_subdivision_depth, ...
        force_options.max_node_count]
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&& ...
        isfinite(v)&&v>=0&&fix(v)==v, ...
        'TRAJECTORY_C9_JOINT:Budget','Proof budgets must be finite integers.');
end
assert(geometry_options.max_node_count>=1&&force_options.max_node_count>=1, ...
    'TRAJECTORY_C9_JOINT:Budget','Positive proof node budgets required.');
assert(isnumeric(geometry_options.numerical_guard_m)&& ...
    isreal(geometry_options.numerical_guard_m)&& ...
    isscalar(geometry_options.numerical_guard_m)&& ...
    isfinite(geometry_options.numerical_guard_m)&& ...
    geometry_options.numerical_guard_m>=0&& ...
    isnumeric(force_options.force_guard_N)&& ...
    isreal(force_options.force_guard_N)&& ...
    isscalar(force_options.force_guard_N)&& ...
    isfinite(force_options.force_guard_N)&&force_options.force_guard_N>=0, ...
    'TRAJECTORY_C9_JOINT:Guard','Caller-owned guards required.');
switch_on=geometry_options.robust_tube_enabled;
assert((isnumeric(switch_on)||islogical(switch_on))&& ...
    isscalar(switch_on)&&(isequal(switch_on,0)||isequal(switch_on,1)), ...
    'TRAJECTORY_C9_JOINT:Tube','Robust switch must be 0/1.');
keys={'uav_m','cable_m','payload_m'};
assert(isstruct(radii)&&isscalar(radii)&&all(isfield(radii,keys)), ...
    'TRAJECTORY_C9_JOINT:Radii','Live separate radii required.');
rr=zeros(1,3);
for k=1:3
    x=radii.(keys{k});
    assert(isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&x>=0, ...
        'TRAJECTORY_C9_JOINT:Radii','Invalid physical radius.');
    rr(k)=double(x);
end
if switch_on
    assert(isfield(geometry_options,'uncertainty_bounds_m')&& ...
        isstruct(geometry_options.uncertainty_bounds_m)&& ...
        all(isfield(geometry_options.uncertainty_bounds_m,keys)), ...
        'TRAJECTORY_C9_JOINT:Tube','External per-body tube required when ON.');
    for k=1:3
        x=geometry_options.uncertainty_bounds_m.(keys{k});
        assert(isnumeric(x)&&isreal(x)&&isscalar(x)&& ...
            isfinite(x)&&x>=0, ...
            'TRAJECTORY_C9_JOINT:Tube','Invalid external tube bound.');
        rr(k)=rr(k)+double(x);
    end
end
assert(all(isfinite(rr)), 'TRAJECTORY_C9_JOINT:Tube', ...
    'Inflated radii must be finite.');
% Shape/pose preparation is invariant across candidate free-controls and
% across proof leaves. Cache ONLY the last exact observed snapshot; every
% change of count, ID, dimensions, position, rotation, order, or metadata
% invalidates it. No detection-mask or obstacle-specific precomputation.
% Cache entries are installed only AFTER the canonical 12-shape validation
% succeeds. Physical authority, inflated radii, phase, and all guards are
% intentionally recomputed per call, even for a cache hit.
snapshot_clock=tic;
[shapes,ids,outer,centers,geometry_snapshot_cache_hit]= ...
    validated_geometry_snapshot(observed);
% Optional second output: the SAME canonical, fully validated observed
% snapshot used by THIS proof. A single caller can reuse it to form
% search planes, without a second shape validation. This is NOT proof
% reuse and it cannot authorize HLC execution. The subsequent candidate
% always goes through the normal independent joint certificate.
prepared_shapes=shapes;
geometry_snapshot_prepare_ms=1000*toc(snapshot_clock);
[ground_active,trajectory_phase]=TRAJECTORY_GROUND_SCOPE(geometry_options);
if ground_active
    ground=GROUND_INFORMATION.definition();
end
L=double(authority.cable_length_m);
mU=double(authority.uav_mass_kg);
mL=double(authority.payload_mass_kg);
maxdepth=min(double(geometry_options.max_subdivision_depth), ...
    double(force_options.max_subdivision_depth));
maxnodes=min(double(geometry_options.max_node_count), ...
    double(force_options.max_node_count));
minw=double(authority.minimum_tension_N)/mL+ ...
    double(authority.acceleration_guard_mps2);
tol=double(geometry_options.numerical_guard_m);
% Floating-point guard for vectorised outer-distance screening. This is
% numerical rounding protection, NOT a physical / tracking-error margin.
scale=max([1;abs(double(piece.control_points_m(:))); ...
    abs(centers(:));abs(outer(:));L;rr(:)]);
outer_roundoff_guard=16*eps(scale);
fguard=double(force_options.force_guard_N);
% Optional strictly tighter support of the entire taut cable: the
% interpolation parameter lambda multiplies BOTH the cable displacement
% and its direction-cone uncertainty. This is a sufficient ALL-TIME bound.
% The old looser bound remains the default for actual-scene A/B parity.
use_lambda_cable_envelope=false;
if isfield(geometry_options,'use_lambda_exact_cable_cone_envelope')
    v=geometry_options.use_lambda_exact_cable_cone_envelope;
    assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_JOINT:CableEnvelopeFlag','Expected boolean 0/1');
    use_lambda_cable_envelope=logical(v);
end
% Optional whole-separator native NORMAL generator. The proposed
% direction is always rechecked using the ORIGINAL MATLAB support value;
% a native proposal can never act as a complete geometry certificate.
use_native_support_separator=false;
if isfield(geometry_options,'use_native_support_separator')
    v=geometry_options.use_native_support_separator;
    assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_JOINT:NativeSeparatorFlag', ...
        'Native support separator flag must be 0/1.');
    use_native_support_separator=logical(v);
end
% Optional exact spherical-cap plane support for taut q=w/||w||.
% A single geometry halfspace then treats payload, cable and UAV with
% their respective physical radii. The previous chord-envelope and the
% canonical full-time oracle remain intact as a fallback. This changes
% only the mathematical SUFFICIENT full-time test, never returns a false
% positive under its stated real-arithmetic premises.
use_direction_cone_support=false;
if isfield(geometry_options,'use_exact_direction_cone_support')
    v=geometry_options.use_exact_direction_cone_support;
    assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_JOINT:ConeFlag','Cone support flag must be 0/1.');
    use_direction_cone_support=logical(v);
end
% When a parent time cell has ALREADY certified one obstacle to be
% separated for every time, descendants need not repeat that particular
% obstacle. This is a proof-tree memo, NOT a sampled-point shortcut.
% No obstacle IDs/poses are fixed and each newly proposed C9 restarts it.
use_ancestral_shape_certificates=false;
if isfield(geometry_options,'use_ancestral_obstacle_certificate_reuse')
    v=geometry_options.use_ancestral_obstacle_certificate_reuse;
    assert((isnumeric(v)||islogical(v))&&isscalar(v)&& ...
        isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
        'TRAJECTORY_C9_JOINT:AncestralSwitch', ...
        'Ancestral shape-certificate reuse flag must be 0/1.');
    use_ancestral_shape_certificates=logical(v);
end
R=struct('exact_direction_cone_enabled',use_direction_cone_support, ...
    'exact_direction_cone_axes',0, ...
    'exact_direction_cone_positive_part_gates',0, ...
    'exact_direction_cone_completed_obstacles',0, ...
    'exact_direction_cone_ground_rescues',0, ...
    'native_separator_requested',use_native_support_separator, ...
    'native_separator_proposals',0, ...
    'ancestral_shape_certificates_enabled',use_ancestral_shape_certificates, ...
    'ancestral_shape_skipped_node_checks',0, ...
    'ancestral_shape_certified_parent_clears',0, ...
    'ancestral_shape_inherited_outer_clears',0, ...
    'cable_lambda_envelope_enabled',use_lambda_cable_envelope, ...
    'cable_lambda_envelope_axis_evaluations',0, ...
    'cable_lambda_envelope_strictly_improved',0, ...
    'status','UNKNOWN_NOT_CERTIFIED','certified',false, ...
    'certified_continuous_geometry',false, ...
    'certified_continuous_tension',false, ...
    'certified_continuous_thrust',false,'safe_to_commit',false, ...
    'considered_obstacle_ids',double(ids),'unresolved_obstacle_ids',[], ...
    'unresolved_body_indices',zeros(1,0), ...
    'checked_node_count',0,'checked_leaf_count',0, ...
    'shape_support_calls',0,'outer_rejections',0, ...
    'near_obstacle_leaf_checks',0, ...
    'certified_leaf_intervals_s',zeros(0,2), ...
    'minimum_tension_lower_bound_N',Inf, ...
    'thrust_lower_bound_N',Inf,'thrust_upper_bound_N',0, ...
    'maximum_cable_cone_padding_m',0, ...
    'ground_checked',ground_active,'trajectory_phase',trajectory_phase, ...
    'ground_status','UNKNOWN_NOT_CHECKED', ...
    'ground_minimum_clearance_lower_bound_m',Inf, ...
    'unresolved_time_interval_s',[NaN NaN], ...
    'geometry_snapshot_cache_hit',geometry_snapshot_cache_hit, ...
    'geometry_snapshot_prepare_ms',geometry_snapshot_prepare_ms, ...
    'elapsed_ms',NaN,'authority_snapshot',authority, ...
    'scope','IDEAL_TAUT_C9_ALL_TIME_GEOMETRY_THRUST_TENSION__NO_TORQUE_HLC');
if ~ground_active
    R.ground_status='NOT_APPLICABLE_OUTSIDE_FLIGHT';
    R.ground_minimum_clearance_lower_bound_m=NaN;
end
P=double(piece.control_points_m);T=double(piece.duration_s);
% Degree-general polynomial proof: allows the exact SAME continuous-time
% geometric sufficient certificate to verify degree-27 C9 candidates.
% The TORQUE, HLC and optimizer modules are not generalized by this edit.
degree=size(P,2)-1;
W=repmat(g,1,degree-1)-(degree*(degree-1)/T^2)*diff(P,2,2);
W1=-(degree*(degree-1)*(degree-2)/T^3)*diff(P,3,2);
W2=-(degree*(degree-1)*(degree-2)*(degree-3)/T^4)*diff(P,4,2);
[passed,R]=test_leaf(P,W,W1,W2,double(piece.t0_s),T,0,R,1:numel(shapes));
R.certified=passed;
R.certified_continuous_geometry=passed;
R.certified_continuous_tension=passed;
R.certified_continuous_thrust=passed;
if passed
    R.status='CERTIFIED_C9_JOINT_IDEAL_TAUT_ALL_TIME__NO_TORQUE_HLC';
    R.unresolved_time_interval_s=[NaN NaN];
    R.unresolved_obstacle_ids=zeros(1,0);
    R.unresolved_body_indices=zeros(1,0);
end
R.elapsed_ms=toc(start)*1000;

    function [ok,S]=test_leaf(Q,V,V1,V2,t,dt,depth,S,pending_shapes)
        ok=false;
        if S.checked_node_count>=maxnodes
            S.status='UNKNOWN_JOINT_PROOF_NODE_BUDGET';
            S.unresolved_time_interval_s=[t,t+dt];return
        end
        S.checked_node_count=S.checked_node_count+1;
        % A certified obstacle on an ancestor is certified throughout
        % this descendant by interval containment.  It cannot return
        % to the local geometry test even when q varies on this leaf.
        if use_ancestral_shape_certificates
            S.ancestral_shape_skipped_node_checks= ...
                S.ancestral_shape_skipped_node_checks+ ...
                numel(shapes)-numel(pending_shapes);
        else
            pending_shapes=1:numel(shapes);
        end
        child_pending=pending_shapes;
        c=mean(V,2);nc=norm(c);
        rho=max(sqrt(sum((V-c).^2,1)));
        low=nc-rho;
        D1=max(sqrt(sum(V1.^2,1)));
        D2=max(sqrt(sum(V2.^2,1)));
        taut=isfinite(low)&&isfinite(D1)&&isfinite(D2)&& ...
            nc>rho&&low>minw;
        forceok=false;geomok=false;
        if taut
            qdd=D2/low+3*(D1/low)^2;
            fdelta=mU*L*qdd;
            flo=max(0,(mU+mL)*low-fdelta);
            fhi=(mU+mL)*(nc+rho)+fdelta;
            forceok=isfinite(flo)&&isfinite(fhi)&& ...
                flo>=f(1)+fguard&&fhi<=f(2)-fguard;
            if forceok
                ratio=rho/nc;
                chord=ratio*sqrt(2/(1+sqrt(max(0,1-ratio^2))));
                pad=L*chord;
                direction=c/nc;
                groundok=true;
                clearance=[Inf Inf Inf];
                if ground_active
                    [groundok,clearance]=TRAJECTORY_C9_GROUND_LEAF( ...
                        Q,direction,pad,L,rr([1 3 2]),ground,tol);
                    if ground_active && use_direction_cone_support
                        % Exact cap projection in upward world ground normal.
                        % The original ground routine is still invoked and
                        % validates the SOURCE ground definition. Both are
                        % legitimate lower bounds; take their maximum.
                        legacyGroundOK=groundok;
                        [capGround,~,capGroundValid]= ...
                            TRAJECTORY_C9_DIRECTION_CONE_SUPPORT( ...
                                c,rho,ground.normal_world);
                        if capGroundValid
                            minz=min(Q(3,:));
                            capClear=[minz-L*capGround-rr(1), ...
                                minz-rr(3), ...
                                minz-L*max(0,capGround)-rr(2)] ...
                                -ground.height_m-tol;
                            clearance=max(clearance,capClear);
                            groundok=all(isfinite(clearance))&&all(clearance>0);
                            if groundok && ~legacyGroundOK
                                S.exact_direction_cone_ground_rescues= ...
                                    S.exact_direction_cone_ground_rescues+1;
                            end
                        end
                    end
                    if ~groundok
                        S.ground_status='UNKNOWN_GROUND_CLEARANCE_NOT_PROVEN';
                    end
                end
                geomok=groundok;
                m=mean(Q,2);
                Rload=max(sqrt(sum((Q-m).^2,1)));
                % Outer-ball gate proves ABSENCE only, never collision.
                % All actual taut UAV/cable/payload points are at most
                % L from the load trajectory, independent of q direction.
                broad=Rload+max([rr(1)+L,rr(2)+L,rr(3)])+tol;
                % Filter every observed obstacle at once. This predicate
                % can ONLY discard objects proven beyond the leaf's
                % full-body outer bound. NaN/Inf arithmetic fails CLOSED
                % and leaves an obstacle in the exact geometry stage.
                delta=centers(:,pending_shapes)-m;
                distance=sqrt(sum(delta.*delta,1));
                definitely_far=isfinite(distance) & ...
                    (distance>broad+outer(pending_shapes)+outer_roundoff_guard);
                S.outer_rejections=S.outer_rejections+nnz(definitely_far);
                near_ids=pending_shapes(~definitely_far);
                S.near_obstacle_leaf_checks= ...
                    S.near_obstacle_leaf_checks+numel(near_ids);
                if use_ancestral_shape_certificates
                    % The broadphase envelope is valid on THIS WHOLE
                    % parent cell: all descendants inherit the exclusion.
                    child_pending=near_ids;
                    S.ancestral_shape_inherited_outer_clears= ...
                        S.ancestral_shape_inherited_outer_clears+ ...
                        nnz(definitely_far);
                end
                for j=near_ids
                    % Short-circuit at the FIRST unresolved shape, exactly
                    % like the baseline.  Inheritance must NOT introduce
                    % extra support calls at a difficult ancestor node.
                    if ~groundok,break;end
                    O=shapes(j);
                    % Rotation-equivariant NORMALS from actual load/UAV/
                    % cable centre geometry. No fixed world-axis direction.
                    normals=[m-O.center, m-L*direction-O.center, ...
                        m-(L/2)*direction-O.center];
                    separated_parts=false(1,3);
                    % A positive certificate is obstacle-wide only if
                    % all three physical bodies have been separated.
                    for a=1:3
                        n=normals(:,a);len=norm(n);
                        if ~(isfinite(len)&&len>0),continue;end
                        n=n/len;
                        h=TRAJECTORY_OBSTACLE_SUPPORT.value(O,n);
                        S.shape_support_calls=S.shape_support_calls+1;
                        mn=min(n.'*Q);
                        sh=L*(n.'*direction);
                        % NEW: exact support of the spherical direction
                        % cap; if this direct plane works, expensive
                        % repeated Gilbert searches can be bypassed.
                        if use_direction_cone_support
                            S.exact_direction_cone_axes= ...
                                S.exact_direction_cone_axes+1;
                            [capMax,~,capValid]= ...
                                TRAJECTORY_C9_DIRECTION_CONE_SUPPORT(c,rho,n);
                            if capValid
                                prev=separated_parts;
                                % UAV pU=pL-L*q; all-time control hull.
                                separated_parts(1)=separated_parts(1)|| ...
                                    (mn-L*capMax>h+rr(1)+tol);
                                % Entire cable p(lambda)=pL-lambda L*q.
                                separated_parts(2)=separated_parts(2)|| ...
                                    (mn-L*max(0,capMax)>h+rr(2)+tol);
                                % The payload position has no q term.
                                separated_parts(3)=separated_parts(3)|| ...
                                    (mn>h+rr(3)+tol);
                                S.exact_direction_cone_positive_part_gates= ...
                                    S.exact_direction_cone_positive_part_gates+ ...
                                    nnz(separated_parts&~prev);
                            end
                        end
                        if mn-sh>h+rr(1)+pad+tol
                            separated_parts(1)=true;
                        end
                        % For all u in this leaf and all lambda in [0,1]:
                        % n·(pL-lambda*L*q(u)) >=
                        %    min_i n·Q_i - max(0,L*n·direction+L*chord).
                        % The original min(mn,mn-sh)-pad subtracts the
                        % FULL cable-direction error even at lambda=0.
                        % Both inequalities are sufficient, but the new
                        % one is never weaker in exact real arithmetic.
                        if use_lambda_cable_envelope
                            old_lhs=min(mn,mn-sh)-pad;
                            cable_lhs=mn-max(0,sh+pad);
                            S.cable_lambda_envelope_axis_evaluations= ...
                                S.cable_lambda_envelope_axis_evaluations+1;
                            if cable_lhs>old_lhs
                                S.cable_lambda_envelope_strictly_improved= ...
                                    S.cable_lambda_envelope_strictly_improved+1;
                            end
                            if cable_lhs>h+rr(2)+tol
                                separated_parts(2)=true;
                            end
                        elseif min(mn,mn-sh)>h+rr(2)+pad+tol
                            separated_parts(2)=true;
                        end
                        if mn>h+rr(3)+tol
                            separated_parts(3)=true;
                        end
                        if all(separated_parts),break;end
                    end
                    if use_direction_cone_support &&all(separated_parts)
                        S.exact_direction_cone_completed_obstacles= ...
                            S.exact_direction_cone_completed_obstacles+1;
                    end
                    if ~all(separated_parts)
                        % No world-fixed bypass direction. Use the genuine
                        % supporting shape/witness for a data-derived
                        % minimum-distance normal for each unresolved body.
                        hulls={Q-L*direction,[Q,Q-L*direction],Q};
                        margins=[rr(1)+pad+tol,rr(2)+pad+tol,rr(3)+tol];
                        for body=find(~separated_parts)
                            [proof,~,~,ncalls,~,nativeUsed]= ...
                                TRAJECTORY_C9_SUPPORT_SEPARATOR( ...
                                    hulls{body},O,margins(body),m, ...
                                    geometry_options.max_node_count, ...
                                    use_native_support_separator);
                            S.native_separator_proposals= ...
                                S.native_separator_proposals+double(nativeUsed);
                            S.shape_support_calls=S.shape_support_calls+ncalls;
                            separated_parts(body)=proof;
                        end
                    end
                    if ~all(separated_parts)
                        geomok=false;
                        S.unresolved_obstacle_ids=double(O.id);
                        S.unresolved_body_indices=find(~separated_parts);
                        break
                    elseif use_ancestral_shape_certificates
                        % This exact full-time convex-hull separation is
                        % now a certificate for EVERY descendant cell.
                        child_pending(child_pending==j)=[];
                        S.ancestral_shape_certified_parent_clears= ...
                            S.ancestral_shape_certified_parent_clears+1;
                    end
                end
            end
        end
        if taut&&forceok&&geomok
            S.checked_leaf_count=S.checked_leaf_count+1;
            S.certified_leaf_intervals_s(end+1,:)=[t,t+dt];
            S.minimum_tension_lower_bound_N=min( ...
                S.minimum_tension_lower_bound_N,mL*low);
            S.thrust_lower_bound_N=min(S.thrust_lower_bound_N,flo);
            S.thrust_upper_bound_N=max(S.thrust_upper_bound_N,fhi);
            S.maximum_cable_cone_padding_m=max( ...
                S.maximum_cable_cone_padding_m,pad);
            if ground_active
                S.ground_minimum_clearance_lower_bound_m=min( ...
                    S.ground_minimum_clearance_lower_bound_m,min(clearance));
                S.ground_status='CERTIFIED_ALL_TIME_ABOVE_GROUND';
            end
            ok=true;return
        end
        if ~taut
            S.status='UNKNOWN_JOINT_TENSION_NOT_PROVEN';
        elseif ~forceok
            S.status='UNKNOWN_JOINT_THRUST_NOT_PROVEN';
        else
            if ~strcmp(S.ground_status,'UNKNOWN_GROUND_CLEARANCE_NOT_PROVEN')
                S.status='UNKNOWN_JOINT_GEOMETRY_NOT_SEPARATED';
            else
                S.status='UNKNOWN_JOINT_GROUND_CLEARANCE_NOT_PROVEN';
            end
        end
        if depth>=maxdepth
            S.unresolved_time_interval_s=[t,t+dt];return
        end
        [Q0,Q1]=split_mid(Q);
        [V0,Vr]=split_mid(V);
        [A0,Ar]=split_mid(V1);
        [B0,Br]=split_mid(V2);
        [yes,S]=test_leaf(Q0,V0,A0,B0,t,dt/2,depth+1,S,child_pending);
        if ~yes,return;end
        [ok,S]=test_leaf(Q1,Vr,Ar,Br,t+dt/2,dt/2,depth+1,S,child_pending);
    end
end

function [left,right]=split_mid(P)
% Same exact midpoint Bernstein subdivision in real arithmetic; the
% shared precomputed Pascal matrix avoids nested MATLAB loops in each leaf.
[left,right]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
end

function [shapes,ids,outer,centers,hit]=validated_geometry_snapshot(observed)
% Exact single-entry memo: invalid data can never populate the cache.
% The original observed struct (not IDs alone or a detection mask) is
% compared at every call. MATLAB's value semantics isolate later edits.
persistent valid last_observed last_shapes last_ids last_outer last_centers
if isempty(valid),valid=false;end
hit=false;
if valid && isequaln(observed,last_observed)
    shapes=last_shapes;ids=last_ids;
    outer=last_outer;centers=last_centers;
    hit=true;
    return
end
% Fresh full validation and SO(3) normalization by the canonical source.
shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(observed);
ids=[shapes.id];
outer=zeros(1,numel(shapes));
for j=1:numel(shapes)
    outer(j)=outer_radius(shapes(j));
end
centers=reshape([shapes.center],3,[]);
% Commit atomically after EVERY validation and outer-radius check passed.
last_observed=observed;
last_shapes=shapes;
last_ids=ids;
last_outer=outer;
last_centers=centers;
valid=true;
end

function r=outer_radius(S)
% TRUE bounding sphere of the SOURCE convex-hull shape around its
% documented pose origin. Used ONLY to reject unreachable obstacles.
% Never replaces exact support separation for nearby obstacles.
d=S.dim;
if ~isempty(S.local_vertices)
    r=max(sqrt(sum(S.local_vertices.^2,2)));
    return
end
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
        error('TRAJECTORY_C9_JOINT:Shape','Unprepared or unknown shape.');
end
assert(isfinite(r)&&r>=0,'TRAJECTORY_C9_JOINT:Shape', ...
    'Shape outer radius invalid.');
end
