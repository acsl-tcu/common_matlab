function R=TRAJECTORY_CORRIDOR_VALIDATE_CANDIDATE(C,pieces,observed,physical_radii,proof_options,max_rows)
%TRAJECTORY_CORRIDOR_VALIDATE_CANDIDATE  Verify candidate against corridor.
% No change of obstacle set, re-timing or different polynomial degree may
% silently reuse a corridor. An INDEPENDENT whole-interval geometry
% certificate is then rerun for geometry before a later DYNAMICS gate.
% This is a candidate gate; it never permits HLC COMMIT by itself.
R=struct('status','UNKNOWN_NOT_CERTIFIED','corridor_constraints_satisfied',false, ...
    'continuous_geometry_certified',false,'safe_to_commit',false, ...
    'worst_linear_violation_m',NaN,'reason','Not yet validated');
if ~(isstruct(C)&&isscalar(C)&&isfield(C,'certified')&&C.certified)
    R.reason='Uncertified corridor';return
end
assert(isstruct(pieces)&&numel(pieces)==numel(C.segment_info), ...
    'TRAJECTORY_CORRIDOR_VALIDATE:Segments','Wrong number of candidate segments.');
assert(isstruct(observed),'TRAJECTORY_CORRIDOR_VALIDATE:Observed','Observed shapes required.');
if isempty(observed),ids=zeros(1,0);
else,ids=reshape(double([observed.id]),1,[]);end
if ~isequal(ids,C.observed_obstacle_ids) || ~isequaln(observed,C.observed_snapshot)
    R.reason='Observed obstacle IDs/order changed: rebuild corridor';
    R.status='UNKNOWN_OBSERVATION_CHANGED';return
end
assert(isstruct(proof_options)&&isfield(proof_options,'robust_tube_enabled') && ...
    isfield(proof_options,'min_separation_m')&& ...
    isfield(proof_options,'numerical_guard_m'), ...
    'TRAJECTORY_CORRIDOR_VALIDATE:Authority','Proof options required.');
if (proof_options.min_separation_m~=C.proof_margin_m)|| ...
        (proof_options.numerical_guard_m~=C.numerical_guard_m)|| ...
        ~isequal(logical(proof_options.robust_tube_enabled), ...
          strcmp(C.scope,'POLYNOMIAL_WITH_SUPPLIED_UNCERTAINTY_RADII'))
    R.status='UNKNOWN_PROOF_OPTIONS_CHANGED';
    R.reason='Proof margins or robust mode changed';return
end
% Compare the externally supplied effective radii to the corridor's
% bound: a radius or tube change invalidates the old set of halfspaces.
fields={'uav_m','payload_m','cable_m'};
rr=zeros(1,3);
for j=1:3
    rr(j)=physical_radii.(fields{j});
    if proof_options.robust_tube_enabled
        rr(j)=rr(j)+proof_options.uncertainty_bounds_m.(fields{j});
    end
end
if ~isequal(rr,C.effective_radii_m)
    R.status='UNKNOWN_RADIUS_OR_TUBE_CHANGED';
    R.reason='Physical radii or error bounds changed';return
end
for j=1:numel(pieces)
    info=C.segment_info(j);
    if ~all(isfield(pieces(j),{'uav','payload'}))
        R.status='UNKNOWN_INCOMPLETE_PIECE';R.reason='Missing UAV/load';return
    end
    u=pieces(j).uav;l=pieces(j).payload;
    if u.t0_s~=info.t0_s||l.t0_s~=info.t0_s|| ...
            u.duration_s~=info.duration_s||l.duration_s~=info.duration_s|| ...
            size(u.control_points_m,2)~=info.uav_controls|| ...
            size(l.control_points_m,2)~=info.payload_controls
        R.status='UNKNOWN_CLOCK_OR_DEGREE_CHANGED';
        R.reason='Guide and candidate time grid or polynomial degree differ';return
    end
end
K=TRAJECTORY_CORRIDOR_LINEAR_CONSTRAINTS(C,max_rows);
x=zeros(C.variable_count,1);
for j=1:numel(pieces)
    info=C.segment_info(j);
    u=pieces(j).uav.control_points_m;
    l=pieces(j).payload.control_points_m;
    x(info.uav_variable_start+(0:numel(u)-1))=u(:);
    x(info.payload_variable_start+(0:numel(l)-1))=l(:);
end
if isempty(K.bineq)
    worst=-Inf;
else
    worst=max(K.Aineq*x-K.bineq);
end
R.worst_linear_violation_m=worst;
if ~(all(isfinite(x))&&worst<=0)
    R.status='UNKNOWN_CORRIDOR_CONSTRAINTS_VIOLATED';
    R.reason='Candidate left at least one prescribed convex corridor';return
end
R.corridor_constraints_satisfied=true;
% Independently inspect the actual nominal polynomial against all
% currently observed shapes (not a stale corridor-only assertion).
G=TRAJECTORY_CONTINUOUS_GEOMETRY_CERTIFICATE( ...
    pieces,observed,physical_radii,proof_options);
R.continuous_geometry_certified=G.certified;
if G.certified
    R.status='CERTIFIED_CORRIDOR_AND_CONTINUOUS_GEOMETRY__DYNAMICS_PENDING';
    R.reason='Geometry passed; force, torque, tension, tracking, yaw and C7 commit still required';
else
    R.status='UNKNOWN_INDEPENDENT_GEOMETRY_FAILED';
    R.reason='Independent geometry proof could not certify candidate';
end
end
