function [R,prepared_shapes]=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE(piece,observed,radii,authority,geometry_options,force_options)
% TRAJECTORY_C9_JOINT_FAST_CERTIFICATE
% One-pass SUFFICIENT continuous-time certificate, ideal taut-load model.
% Tension, thrust-magnitude, and UAV/cable/payload geometry are proved
% from ONE set of degree-23 payload Bernstein controls per time leaf.
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
    isequal(size(piece.control_points_m),[3 24])&& ...
    isnumeric(piece.control_points_m)&&isreal(piece.control_points_m)&& ...
    all(isfinite(piece.control_points_m(:)))&& ...
    isnumeric(piece.duration_s)&&isreal(piece.duration_s)&& ...
    isscalar(piece.duration_s)&&isfinite(piece.duration_s)&& ...
    piece.duration_s>0&&isnumeric(piece.t0_s)&& ...
    isscalar(piece.t0_s)&&isreal(piece.t0_s)&& ...
    isfinite(piece.t0_s)&&isfinite(piece.t0_s+piece.duration_s), ...
    'TRAJECTORY_C9_JOINT:Piece','Require finite degree-23 load piece.');
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
R=struct('status','UNKNOWN_NOT_CERTIFIED','certified',false, ...
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
W=repmat(g,1,22)-(23*22/T^2)*diff(P,2,2);
W1=-(23*22*21/T^3)*diff(P,3,2);
W2=-(23*22*21*20/T^4)*diff(P,4,2);
[passed,R]=test_leaf(P,W,W1,W2,double(piece.t0_s),T,0,R);
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

    function [ok,S]=test_leaf(Q,V,V1,V2,t,dt,depth,S)
        ok=false;
        if S.checked_node_count>=maxnodes
            S.status='UNKNOWN_JOINT_PROOF_NODE_BUDGET';
            S.unresolved_time_interval_s=[t,t+dt];return
        end
        S.checked_node_count=S.checked_node_count+1;
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
                delta=centers-m;
                distance=sqrt(sum(delta.*delta,1));
                definitely_far=isfinite(distance) & ...
                    (distance>broad+outer+outer_roundoff_guard);
                S.outer_rejections=S.outer_rejections+nnz(definitely_far);
                near_ids=find(~definitely_far);
                S.near_obstacle_leaf_checks= ...
                    S.near_obstacle_leaf_checks+numel(near_ids);
                for j=near_ids
                    if ~geomok,break;end
                    O=shapes(j);
                    % Rotation-equivariant NORMALS from actual load/UAV/
                    % cable centre geometry. No fixed world-axis direction.
                    normals=[m-O.center, m-L*direction-O.center, ...
                        m-(L/2)*direction-O.center];
                    separated_parts=false(1,3);
                    for a=1:3
                        n=normals(:,a);len=norm(n);
                        if ~(isfinite(len)&&len>0),continue;end
                        n=n/len;
                        h=TRAJECTORY_OBSTACLE_SUPPORT.value(O,n);
                        S.shape_support_calls=S.shape_support_calls+1;
                        mn=min(n.'*Q);
                        sh=L*(n.'*direction);
                        if mn-sh>h+rr(1)+pad+tol
                            separated_parts(1)=true;
                        end
                        if min(mn,mn-sh)>h+rr(2)+pad+tol
                            separated_parts(2)=true;
                        end
                        if mn>h+rr(3)+tol
                            separated_parts(3)=true;
                        end
                        if all(separated_parts),break;end
                    end
                    if ~all(separated_parts)
                        % No world-fixed bypass direction. Use the genuine
                        % supporting shape/witness for a data-derived
                        % minimum-distance normal for each unresolved body.
                        hulls={Q-L*direction,[Q,Q-L*direction],Q};
                        margins=[rr(1)+pad+tol,rr(2)+pad+tol,rr(3)+tol];
                        for body=find(~separated_parts)
                            [proof,~,~,ncalls]=TRAJECTORY_C9_SUPPORT_SEPARATOR( ...
                                hulls{body},O,margins(body),m, ...
                                geometry_options.max_node_count);
                            S.shape_support_calls=S.shape_support_calls+ncalls;
                            separated_parts(body)=proof;
                        end
                    end
                    if ~all(separated_parts)
                        geomok=false;
                        S.unresolved_obstacle_ids=double(O.id);
                        % The current full-time proof failed for these
                        % actual body occupancies: 1=UAV, 2=entire cable,
                        % 3=payload. This is a SEARCH DIAGNOSTIC, NOT a
                        % collision verdict. Avoid pushing payload hull
                        % cuts when only UAV/cable separation is unknown.
                        S.unresolved_body_indices=find(~separated_parts);
                        break
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
        [yes,S]=test_leaf(Q0,V0,A0,B0,t,dt/2,depth+1,S);
        if ~yes,return;end
        [ok,S]=test_leaf(Q1,Vr,Ar,Br,t+dt/2,dt/2,depth+1,S);
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
