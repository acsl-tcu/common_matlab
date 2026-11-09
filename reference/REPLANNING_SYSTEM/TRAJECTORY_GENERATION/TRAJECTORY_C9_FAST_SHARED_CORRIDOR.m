function C=TRAJECTORY_C9_FAST_SHARED_CORRIDOR(guide,observed,radii,authority,geometry_options,force_options,corridor_options)
% TRAJECTORY_C9_FAST_SHARED_CORRIDOR
% A sufficient continuous-time C9 coupled corridor from ONE joint proof.
% For a certified taut guide, obtain its DYADIC certified time leaves from
% the all-time geometry+tension+thrust certificate, rather than re-running
% old full-body proof followed by a general-purpose nonlinear plane search.
%
% On each leaf, W=g-pL'' is bounded by B(c,rho), with ||c||>rho.
% q=W/||W|| is within d(rho/||c||) of c/||c||. Consequently the UAV, load
% and full straight cable lie in the translated Bernstein control hulls,
% with ONLY L*d cone padding on UAV and cable. For each observed obstacle
% and each body, find a strict support halfspace n'*P >= h_O(n)+margin.
% These planes are chosen from the actual relative geometry (no fixed
% upward/leftward bypass), and are valid for the whole leaf. Candidate
% perturbations re-evaluate their own acceleration-cone padding in the
% existing TRAJECTORY_C9_TAUT_CORRIDOR_RAY.feasible() function.
%
% If a normal cannot establish separation, return UNKNOWN. Do NOT omit a
% far obstacle merely because the GUIDE was far: the new candidate could
% move toward it. Every observed obstacle gets three explicit halfspaces.
% Neither this corridor nor the joint certificate authorizes HLC commit.
start=tic;
req={'min_separation_m','max_normal_iterations','max_ray_iterations'};
assert(isstruct(corridor_options)&&isscalar(corridor_options)&& ...
    all(isfield(corridor_options,req)), ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Options','Explicit corridor options required.');
assert(isnumeric(corridor_options.min_separation_m)&& ...
    isreal(corridor_options.min_separation_m)&& ...
    isscalar(corridor_options.min_separation_m)&& ...
    isfinite(corridor_options.min_separation_m)&& ...
    corridor_options.min_separation_m>0 && ...
    isnumeric(corridor_options.max_ray_iterations)&& ...
    isscalar(corridor_options.max_ray_iterations)&& ...
    isfinite(corridor_options.max_ray_iterations)&& ...
    corridor_options.max_ray_iterations>=1&& ...
    fix(corridor_options.max_ray_iterations)==corridor_options.max_ray_iterations && ...
    isnumeric(corridor_options.max_normal_iterations)&& ...
    isscalar(corridor_options.max_normal_iterations)&& ...
    isfinite(corridor_options.max_normal_iterations)&& ...
    corridor_options.max_normal_iterations>=1&& ...
    fix(corridor_options.max_normal_iterations)==corridor_options.max_normal_iterations, ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Options','Invalid explicit proof budgets.');
% A/B switch affects only verified cross-body plane reuse;
% it never relaxes margins or the mandatory all-time joint proof.
reuse_body_plane=true;
if isfield(corridor_options,'reuse_verified_body_normal')
    reuse_body_plane=corridor_options.reuse_verified_body_normal;
    assert((islogical(reuse_body_plane)||isnumeric(reuse_body_plane))&& ...
        isscalar(reuse_body_plane)&& ...
        isfinite(double(reuse_body_plane))&& ...
        (double(reuse_body_plane)==0||double(reuse_body_plane)==1), ...
        'TRAJECTORY_C9_FAST_CORRIDOR:ReuseSwitch', ...
        'reuse_verified_body_normal must be boolean 0/1.');
    reuse_body_plane=logical(reuse_body_plane);
end
assert(isstruct(guide)&&isscalar(guide)&& ...
    all(isfield(guide,{'control_points_m','t0_s','duration_s', ...
       'start_jets','end_jets'}))&& ...
    isequal(size(guide.control_points_m),[3 24]), ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Guide','Require degree-23 C9 guide.');
assert(isfield(geometry_options,'numerical_guard_m')&& ...
    isfield(geometry_options,'robust_tube_enabled'), ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Geometry','Explicit proof mode required.');
[ground_active,trajectory_phase]=TRAJECTORY_GROUND_SCOPE(geometry_options);
if ground_active
    ground_snapshot=GROUND_INFORMATION.definition();
else
    ground_snapshot=struct([]);
end
C=struct('certified',false,'safe_to_commit',false, ...
    'status','UNKNOWN_NOT_CERTIFIED','scope', ...
    'IDEAL_TAUT_C9_SHARED_PROOF_CORRIDOR__NOT_HLC', ...
    'leaves',struct([]),'guide_piece',guide, ...
    'observed_snapshot',{observed},'radii',radii, ...
    'authority',authority,'ground_snapshot',ground_snapshot, ...
    'ground_applicable',ground_active,'trajectory_phase',trajectory_phase, ...
    'proof_options',geometry_options, ...
    'corridor_options',corridor_options, ...
    'obstacle_ids',zeros(1,0),'elapsed_ms',NaN, ...
    'joint_proof_ms',NaN,'plane_construction_ms',NaN, ...
    'shape_support_calls',0,'failed_obstacle_id',NaN, ...
    'failed_body_id',NaN,'failed_time_interval_s',[NaN NaN], ...
    'plane_refinement_count',0,'joint_status','UNKNOWN_NOT_RUN', ...
    'reused_body_separation_planes',0, ...
    'fallback_separator_searches',0, ...
    'shared_body_plane_reuse_enabled',reuse_body_plane);
joint=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
    guide,observed,radii,authority,geometry_options,force_options);
C.joint_status=joint.status;
C.joint_proof_ms=joint.elapsed_ms;
C.obstacle_ids=joint.considered_obstacle_ids;
if ~joint.certified
    C.status=['UNKNOWN_JOINT_' joint.status];
    C.elapsed_ms=toc(start)*1000;
    return
end
assert(isfield(joint,'certified_leaf_intervals_s')&& ...
    size(joint.certified_leaf_intervals_s,2)==2 && ...
    size(joint.certified_leaf_intervals_s,1)==joint.checked_leaf_count, ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Proof','Joint leaf witness missing.');
plane_clock=tic;
shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(observed);
rad=[radii.uav_m,radii.payload_m,radii.cable_m];
if geometry_options.robust_tube_enabled
    tube=geometry_options.uncertainty_bounds_m;
    rad=rad+[tube.uav_m,tube.payload_m,tube.cable_m];
end
L=double(authority.cable_length_m);
g=double(authority.gravity_world_mps2(:));
minw=double(authority.minimum_tension_N)/ ...
    double(authority.payload_mass_kg)+ ...
    double(authority.acceleration_guard_mps2);
delta=double(corridor_options.min_separation_m)+ ...
    double(geometry_options.numerical_guard_m);
assert(all(isfinite(rad))&&all(rad>=0)&&isfinite(delta)&&delta>0, ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Margin','Non-finite safety offsets.');
P0=double(guide.control_points_m);
tstart=double(guide.t0_s);T0=double(guide.duration_s);
leaves=struct([]);
intervals=joint.certified_leaf_intervals_s;
for k=1:size(intervals,1)
    ta=intervals(k,1);tb=intervals(k,2);dt=tb-ta;
    a=(ta-tstart)/T0;b=(tb-tstart)/T0;
    if ~(isfinite(a+b)&&a>=-32*eps&&b<=1+32*eps&& ...
            b>a&&dt>0)
        C.status='UNKNOWN_INVALID_PROOF_INTERVAL';
        C.elapsed_ms=toc(start)*1000;return
    end
    a=max(0,a);b=min(1,b);
    M=interval_map(24,a,b);
    P=P0*M;
    W=repmat(g,1,22)-(23*22/dt^2)*diff(P,2,2);
    c=mean(W,2);s=norm(c);
    rho=max(sqrt(sum((W-c).^2,1)));
    if ~(isfinite(rho)&&s>rho&&s-rho>minw)
        C.status='UNKNOWN_LEAF_TENSION_RECOMPUTE';
        C.elapsed_ms=toc(start)*1000;return
    end
    q=c/s;pad=L*chord(rho/s);
    hullU=P-L*q;
    hullL=P;
    hullC=[P,P-L*q];
    hulls={hullU,hullL,hullC};
    common_guide_center=mean(P,2);
    normals=cell(1,3);offsets=cell(1,3);
    for body=1:3
        normals{body}=zeros(3,numel(shapes));
        offsets{body}=zeros(numel(shapes),1);
    end
    for j=1:numel(shapes)
        S=shapes(j);
        for body=1:3
            H=hulls{body};
            % Test the direct centre axis first, then a common payload
            % reference axis. The latter is valuable for tall obstacles:
            % UAV/cable centres can be vertically displaced, although the
            % payload-to-obstacle horizontal separation is already strict.
            % Neither direction is a world-fixed bypass rule. If neither
            % proves separation, use a rotation-equivariant Gilbert/FW
            % closest-Minkowski-point iteration with the EXACT common shape
            % support and witness. Every accepted plane is independently
            % checked against ALL controls and the support function.
            % This one obstacle has the SAME true support h_O(n) for all
            % three physical bodies. An already STRICTLY certified normal
            % for an earlier body is a candidate for the next body too.
            % Reuse ONLY if the new body's complete Bernstein hull obeys
            % its OWN radius, cable cone and numerical safety offsets.
            % Otherwise run the original, independent 3-D separator.
            % This is construction-time reuse, never proof reuse or an
            % assumption that the UAV/payload/cable share one centre.
            margin=rad(body)+double(body==1||body==3)*pad+delta;
            ok=false;n=zeros(3,1);beta=NaN;calls=0;iters=0;
            if C.shared_body_plane_reuse_enabled && body>1
                for earlier=1:body-1
                    ne=normals{earlier}(:,j);
                    old_margin=rad(earlier)+ ...
                        double(earlier==1||earlier==3)*pad+delta;
                    % The previous beta is h_O(ne)+old_margin.
                    % A positive scale-relative guard makes the reuse
                    % test more conservative under ordinary rounding;
                    % exact outward-rounded certification is a separate
                    % still-unfinished numerical-proof requirement.
                    trial_beta=offsets{earlier}(j)+margin-old_margin;
                    round_guard=16*eps(max([1,abs(trial_beta), ...
                        abs(offsets{earlier}(j)),abs(margin), ...
                        abs(old_margin),max(abs(H(:)))]));
                    if all(isfinite(ne)) && norm(ne)>0 && ...
                            isfinite(trial_beta) && ...
                            min(ne.'*H)>trial_beta+round_guard
                        % Store the conservative threshold, not the
                        % potentially downward-rounded subtraction.
                        ok=true;n=ne;beta=trial_beta+round_guard;
                        C.reused_body_separation_planes= ...
                            C.reused_body_separation_planes+1;
                        break
                    end
                end
            end
            if ~ok
                C.fallback_separator_searches= ...
                    C.fallback_separator_searches+1;
                [ok,n,beta,calls,iters]= ...
                    TRAJECTORY_C9_SUPPORT_SEPARATOR( ...
                    H,S,margin,common_guide_center, ...
                    corridor_options.max_normal_iterations);
                C.shape_support_calls=C.shape_support_calls+calls;
                C.plane_refinement_count=C.plane_refinement_count+iters;
            end
            if ~ok
                C.status='UNKNOWN_NO_SUPPORT_SEPARATOR';
                C.failed_obstacle_id=S.id;
                C.failed_body_id=body;
                C.failed_time_interval_s=[ta tb];
                C.plane_construction_ms=toc(plane_clock)*1000;
                C.elapsed_ms=toc(start)*1000;return
            end
            normals{body}(:,j)=n;
            offsets{body}(j)=beta;
        end
    end
    leaf=struct('time_interval_s',[ta tb], 'map',M, ...
        'acceleration_center_mps2',c, ...
        'guide_acceleration_radius_mps2',rho, ...
        'guide_acceleration_controls_mps2',W, ...
        'gravity_minus_load_acceleration_center_mps2',c, ...
        'q0',q,'guide_cone_padding_m',pad, ...
        'guide_tension_lower_bound_N', ...
        double(authority.payload_mass_kg)*(s-rho), ...
        'uav_normals',normals{1},'uav_offsets',offsets{1}, ...
        'payload_normals',normals{2}, ...
        'payload_offsets',offsets{2}, ...
        'cable_normals',normals{3},'cable_offsets',offsets{3});
    if isempty(leaves),leaves=leaf;else,leaves(end+1)=leaf;end %#ok<AGROW>
end
C.leaves=leaves;
C.plane_construction_ms=toc(plane_clock)*1000;
C.certified=true;
C.status='CERTIFIED_C9_SHARED_JOINT_AND_LINEAR_CORRIDOR__NOT_HLC';
C.elapsed_ms=toc(start)*1000;
end

function c=chord(v)
assert(isfinite(v)&&v>=0&&v<1, ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Cone','Invalid taut cone ratio.');
c=v*sqrt(2/(1+sqrt(max(0,1-v*v))));
end

function M=interval_map(n,a,b)
assert(a>=0&&b<=1&&b>a&&isfinite(a+b), ...
    'TRAJECTORY_C9_FAST_CORRIDOR:Interval','Invalid leaf interval.');
M=eye(n);
if a>0,[~,M]=split(M,a);end
if b<1,[M,~]=split(M,(b-a)/(1-a));end
end

function [X,Y]=split(P,t)
n=size(P,2);Q=P;X=zeros(size(P));Y=zeros(size(P));
X(:,1)=Q(:,1);Y(:,end)=Q(:,end);
for j=2:n
    Q=(1-t)*Q(:,1:end-1)+t*Q(:,2:end);
    X(:,j)=Q(:,1);Y(:,n-j+1)=Q(:,end);
end
end
