function R=TRAJECTORY_SIMULTANEOUS_C7_SEARCH(req)
%TRAJECTORY_SIMULTANEOUS_C7_SEARCH One NLP, joint C7 controls + 3-D planes.
% GEOMETRIC SEARCH ONLY: it does not certify taut-load dynamics or permit
% HLC commitment. All observed static obstacles are imposed SIMULTANEOUSLY.
%
% Variables are 3x8 free controls for each of UAV and load (degree 23,
% endpoint 0..7 jets fixed), and one unrestricted-orientation UNIT normal
% for EACH (time leaf, observed obstacle, UAV / payload / cable).
% All Bezier controls of each leaf are kept on the safe side of its plane.
% The cable uses the SAME normal for the UAV and load control hulls.
% The obstacle support is exact for 12 canonical convex hulls (nonconvex
% shapes are safely overapproximated by their convex hulls).
%
% If a certified warm start exists, a fixed-plane 1-D convex ray
% refinement is attempted BEFORE any NLP; its certified result may
% directly replace the expensive nonconvex solve. Otherwise at most ONE
% nonlinear optimization call (fmincon or supplied adapter) is allowed.
% This is not Stage1/2/3 and not a predefined left/right/up branch.
% No optimizer result is trusted: plane inequalities, exact 7-jets and an
% INDEPENDENT whole-time geometry certificate are checked afterwards.
% Exhausted budgets, solver failure or doubtful numerics => UNKNOWN.
%
% Required req fields:
%   t0_s, duration_s                    externally selected interval
%   start_jets, end_jets                struct .uav/.payload, 3x8 each
%   nominal_free                        struct .uav/.payload, 3x8 each
%   observed                            retained sensor-known static shapes
%   physical_radii                      .uav_m, .payload_m, .cable_m
%   proof_options                       standard corridor proof options
%   search_options                      struct with time_leaf_depth,
%      max_constraint_rows, feasibility_tolerance, endpoint_jet_tolerance
%   solver_options                      externally provided fmincon options
% Optional req.solver_function          @(fun,z0,nonlcon,opts) returns
%                                      [z,exitflag,output] (test/adapter)
% Optional req.initial_normals          3x(3*obstacles*leaves), nonzero
% Optional req.candidate_feasibility: caller-owned IN-SOLVE conditions,
%   .derivative_limits (.uav and .payload, each .derivative_norm_max 1..6)
%   .entry (EKF-now, live cable length and caller-owned tolerances)
%   .heading_options (continuous nonzero horizontal speed proof budget)
% In this mode derivative hull inequalities are imposed at EVERY NLP
% iteration, not merely checked on the final optimized trajectory.
% The independent final certificate remains mandatory. The load and UAV
% polynomials are still NOT a proof of taut coupled dynamics; use the pinned
% HLC/plant model for the separate actuation/tension/tracking requirements.
%
% An all-space collision-free guide is NOT assumed. However, like any
% nonconvex local NLP, a single solve cannot guarantee discovering every
% feasible detour/topology. No claim of a universal 5-ms upper bound.
start_tick=tic;
required={'t0_s','duration_s','start_jets','end_jets','nominal_free', ...
    'observed','physical_radii','proof_options','search_options','solver_options'};
assert(isstruct(req)&&isscalar(req)&&all(isfield(req,required)), ...
    'TRAJECTORY_SIMULTANEOUS:Request','Missing authoritative search input.');
assert(isnumeric(req.t0_s)&&isscalar(req.t0_s)&&isfinite(req.t0_s)&& ...
    isnumeric(req.duration_s)&&isscalar(req.duration_s)&& ...
    isfinite(req.duration_s)&&req.duration_s>0, ...
    'TRAJECTORY_SIMULTANEOUS:Time','Valid external time interval required.');
for fld={'start_jets','end_jets','nominal_free'}
    v=req.(fld{1});
    assert(isstruct(v)&&isscalar(v)&&all(isfield(v,{'uav','payload'})), ...
        'TRAJECTORY_SIMULTANEOUS:Jet','Missing UAV and payload data.');
    for b={'uav','payload'}
        P=v.(b{1});
        assert(isnumeric(P)&&isreal(P)&&isequal(size(P),[3,8])&& ...
            all(isfinite(P(:))), ...
            'TRAJECTORY_SIMULTANEOUS:Jet','Expected finite 3x8 data.');
    end
end
s=req.search_options;
fields={'time_leaf_depth','max_constraint_rows','max_leaf_count', ...
    'feasibility_tolerance','endpoint_jet_tolerance'};
assert(isstruct(s)&&isscalar(s)&&all(isfield(s,fields)), ...
    'TRAJECTORY_SIMULTANEOUS:Budget','External search budgets required.');
assert(isnumeric(s.time_leaf_depth)&&isscalar(s.time_leaf_depth)&& ...
    isfinite(s.time_leaf_depth)&&s.time_leaf_depth>=0&& ...
    s.time_leaf_depth==fix(s.time_leaf_depth)&& ...
    isfinite(2^s.time_leaf_depth)&&2^s.time_leaf_depth<=s.max_leaf_count&& ...
    isnumeric(s.max_constraint_rows)&&isscalar(s.max_constraint_rows)&& ...
    isfinite(s.max_constraint_rows)&&s.max_constraint_rows>=0&& ...
    s.max_constraint_rows==fix(s.max_constraint_rows)&& ...
    isnumeric(s.max_leaf_count)&&isscalar(s.max_leaf_count)&& ...
    isfinite(s.max_leaf_count)&&s.max_leaf_count>=1&& ...
    s.max_leaf_count==fix(s.max_leaf_count)&& ...
    isnumeric(s.feasibility_tolerance)&& ...
    isscalar(s.feasibility_tolerance)&&isfinite(s.feasibility_tolerance)&& ...
    s.feasibility_tolerance>=0&& ...
    isnumeric(s.endpoint_jet_tolerance)&& ...
    isscalar(s.endpoint_jet_tolerance)&&isfinite(s.endpoint_jet_tolerance)&& ...
    s.endpoint_jet_tolerance>=0, ...
    'TRAJECTORY_SIMULTANEOUS:Budget','Invalid explicit solver budgets.');
opt=req.proof_options;
needopt={'robust_tube_enabled','min_separation_m','numerical_guard_m', ...
    'max_subdivision_depth'};
assert(isstruct(opt)&&isscalar(opt)&&all(isfield(opt,needopt)), ...
    'TRAJECTORY_SIMULTANEOUS:Proof','External proof options required.');
assert(isequal(opt.robust_tube_enabled,0)||isequal(opt.robust_tube_enabled,1), ...
    'TRAJECTORY_SIMULTANEOUS:Tube','Tube flag must be 0 or 1.');
assert(isnumeric(opt.min_separation_m)&&isscalar(opt.min_separation_m)&& ...
    isfinite(opt.min_separation_m)&&opt.min_separation_m>0&& ...
    isnumeric(opt.numerical_guard_m)&&isscalar(opt.numerical_guard_m)&& ...
    isfinite(opt.numerical_guard_m)&&opt.numerical_guard_m>=0, ...
    'TRAJECTORY_SIMULTANEOUS:Proof','Positive clearance and guard required.');
parts={'uav_m','payload_m','cable_m'};
assert(isstruct(req.physical_radii)&& ...
    all(isfield(req.physical_radii,parts)), ...
    'TRAJECTORY_SIMULTANEOUS:Radii','All physical radii required.');
r=zeros(1,3);
for j=1:3
    a=req.physical_radii.(parts{j});
    assert(isnumeric(a)&&isscalar(a)&&isreal(a)&&isfinite(a)&&a>=0, ...
        'TRAJECTORY_SIMULTANEOUS:Radii','Invalid physical radius.');
    r(j)=double(a);
end
if opt.robust_tube_enabled
    assert(isfield(opt,'uncertainty_bounds_m')&& ...
        isstruct(opt.uncertainty_bounds_m)&& ...
        all(isfield(opt.uncertainty_bounds_m,parts)), ...
        'TRAJECTORY_SIMULTANEOUS:Tube','Missing external error tube.');
    for j=1:3
        a=opt.uncertainty_bounds_m.(parts{j});
        assert(isnumeric(a)&&isscalar(a)&&isfinite(a)&&a>=0, ...
            'TRAJECTORY_SIMULTANEOUS:Tube','Invalid error radius.');
        r(j)=r(j)+double(a);
    end
end
inflation=r+double(opt.min_separation_m)+double(opt.numerical_guard_m);
shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(req.observed);
all_obstacle_ids=reshape([shapes.id],1,[]);
pruned_ids=zeros(1,0);pruned_slacks=zeros(1,0);
nleaf=2^s.time_leaf_depth;
has_candidate_feasibility=isfield(req,'candidate_feasibility');
entry_result=struct();
if has_candidate_feasibility
    cf=req.candidate_feasibility;
    assert(isstruct(cf)&&isscalar(cf)&& ...
        all(isfield(cf,{'derivative_limits','entry','heading_options'})), ...
        'TRAJECTORY_SIMULTANEOUS:Feasibility', ...
        'Candidate feasibility requires live derivative limits, entry state and heading proof options.');
    entry_result=TRAJECTORY_C7_ENTRY_CONSISTENCY(req.start_jets,cf.entry);
    % Validate derivative limits before the potentially expensive NLP.
    for field={'uav','payload'}
        assert(isfield(cf.derivative_limits,field{1})&& ...
            isfield(cf.derivative_limits.(field{1}),'derivative_norm_max')&& ...
            isnumeric(cf.derivative_limits.(field{1}).derivative_norm_max)&& ...
            numel(cf.derivative_limits.(field{1}).derivative_norm_max)==6&& ...
            all(isfinite(cf.derivative_limits.(field{1}).derivative_norm_max(:)))&& ...
            all(cf.derivative_limits.(field{1}).derivative_norm_max(:)>=0), ...
            'TRAJECTORY_SIMULTANEOUS:Limits', ...
            'All derivative bounds must come from an external authority.');
    end
    assert(isstruct(cf.heading_options)&& ...
        all(isfield(cf.heading_options, ...
            {'max_subdivision_depth','numerical_guard_mps'})), ...
        'TRAJECTORY_SIMULTANEOUS:Heading', ...
        'The caller must supply a continuous-heading proof budget.');
    kinematic_rows=246*nleaf;
else
    kinematic_rows=0;
end
% Sound online pruning is possible only because the SAME external speed
% bounds are imposed on every candidate over the full time interval.
% This is NOT a bounding-sphere collision rule: pruned obstacles need
% not enter the NLP, but the final independent proof uses ALL observed.
if has_candidate_feasibility && entry_result.passed && ...
        ~isfield(req,'initial_normals')
    % Explicit caller-supplied normal layouts must not be silently remapped.
    [shapes,pruned_ids,pruned_slacks]= ...
        TRAJECTORY_C7_CERTIFIED_OBSTACLE_EXCLUSION( ...
        shapes,req.start_jets,cf.derivative_limits, ...
        req.duration_s,inflation);
end
ns=numel(shapes);
geom_rows=ns*nleaf*(24+24+48);
rows=geom_rows+kinematic_rows;
assert(rows<=s.max_constraint_rows, ...
    'TRAJECTORY_SIMULTANEOUS:Budget', ...
    'Single solve exceeds explicit geometry constraint budget.');
leaves=make_leaves(s.time_leaf_depth,req.t0_s,req.duration_s);
leaf_maps={leaves.map};
prepared_nlp=TRAJECTORY_C7_NLP_STATIC_PREPARE(leaf_maps,req.duration_s);
R=struct('status','UNKNOWN_NOT_SOLVED','candidate_available',false, ...
    'certified_geometry',false,'safe_to_commit',false,'pieces',struct([]), ...
    'corridor',struct([]),'solve_exitflag',NaN,'solve_output',struct(), ...
    'time_s',req.t0_s,'duration_s',req.duration_s, ...
    'considered_obstacle_ids',all_obstacle_ids, ...
    'optimized_obstacle_ids',reshape([shapes.id],1,[]), ...
    'proven_unreachable_obstacle_ids',pruned_ids, ...
    'unreachable_separation_slack_m',pruned_slacks, ...
    'free_variable_count',48,'plane_variable_count',9*ns*nleaf, ...
    'inequality_count',rows,'geometry_inequality_count',geom_rows, ...
    'kinematic_inequality_count',kinematic_rows, ...
    'solver_gradient_mode','ANALYTIC_OBJECTIVE_AND_CONSTRAINT_JACOBIAN', ...
    'optimization_scope','NONCONVEX_NLP_IF_NEEDED', ...
    'objective_scope','EXACT_INTEGRATED_SQUARED_SPATIAL_DEVIATION', ...
    'candidate_kinematics_enforced',has_candidate_feasibility, ...
    'entry_consistency',entry_result, ...
    'kinematic_certificate',struct(),'heading_certificate',struct(), ...
    'minimum_plane_slack_m',NaN, ...
    'maximum_endpoint_jet_error',NaN,'independent_geometry',struct(), ...
    'elapsed_ms',NaN,'reason','Solver not yet called', ...
    'solver_called',false,'nominal_fast_path_used',false, ...
    'nominal_global_deviation_minimum_proven',false, ...
    'initial_max_inequality_violation',NaN, ...
    'initial_max_geometry_violation_m',NaN, ...
    'initial_max_derivative_constraint_violation',NaN, ...
    'initial_max_normal_equality_violation',NaN, ...
    'final_max_inequality_violation',NaN, ...
    'final_max_geometry_violation_m',NaN, ...
    'final_max_derivative_constraint_violation',NaN, ...
    'final_max_normal_equality_violation',NaN, ...
    'solver_returned_finite_point',false, ...
    'solver_point_feasible',false, ...
    'solver_positive_exitflag',false, ...
    'solver_termination_message','', ...
    'warm_start_status','NOT_REQUESTED', ...
    'warm_start_used',false,'warm_start_attempts',0, ...
    'warm_start_ms',0,'warm_start_geometry_violation_m',NaN, ...
    'warm_start_candidate_selected',false, ...
    'certified_corridor_ray_selected',false, ...
    'corridor_ray_fraction',NaN, ...
    'corridor_ray_deviation_reduction',NaN, ...
    'corridor_ray_ms',NaN, ...
    'ray_constraint_stage_ms',NaN, ...
    'ray_independent_certificate_ms',0, ...
    'ray_corridor_certificate_ms',0, ...
    'nominal_independent_certificate_ms',NaN);
if has_candidate_feasibility && ~entry_result.passed
    R.status='REJECT_ENTRY_STATE_NOT_CONSISTENT';
    R.reason=entry_result.reason;
    R.elapsed_ms=toc(start_tick)*1000;
    return
end
% The original nominal may be colliding; it is only the objective anchor,
% not a precomputed safe corridor or a source of obstacle information.
u0=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets.uav,req.end_jets.uav,req.nominal_free.uav);
l0=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets.payload,req.end_jets.payload,req.nominal_free.payload);
zfree=[req.nominal_free.uav(:);req.nominal_free.payload(:)];
% Exact integral over the WHOLE interval of squared spatial deviation
% from the supplied original reference, rather than the arbitrary norm of
% control-point displacements. Endpoint controls are shared and cancel.
% The duration factor is fixed in this NLP and included in the metric.
gram=bernstein_free_gram_23()*req.duration_s;
% An unchanged reference has exactly zero integrated deviation; it is
% therefore the GLOBAL minimum of this objective. If it already passes
% EVERY independent continuous-time nominal certificate, running an NLP
% cannot improve it. This fast path is deliberately NOT used by solver
% adapters (including gradient-validation adapters); when the original is
% colliding, all subsequent nonlinear search remains enabled.
% No new path is committed and no corridor for a CHANGED path is claimed.
if ~isfield(req,'solver_function') && ~isfield(req,'initial_normals')
    nominal_proof_tick=tic;
    [unchanged_ok,unchanged_proof]=certify_unchanged_nominal(u0,l0);
    R.nominal_independent_certificate_ms=toc(nominal_proof_tick)*1000;
    if unchanged_ok
        R.status='CERTIFIED_UNCHANGED_NOMINAL__NO_NLP__DYNAMICS_PENDING';
        R.reason=['The unchanged reference has zero global deviation ' ...
            'and passes the independent full-time geometry, derivative, ' ...
            'heading and endpoint checks. No modified corridor was built; ' ...
            'physical HLC/plant feasibility remains uncertified.'];
        R.candidate_available=true;
        R.certified_geometry=true;
        R.pieces=struct('uav',u0,'payload',l0);
        R.independent_geometry=unchanged_proof.geometry;
        R.kinematic_certificate=unchanged_proof.kinematics;
        R.heading_certificate=unchanged_proof.heading;
        R.maximum_endpoint_jet_error=unchanged_proof.endpoint_error;
        R.nominal_fast_path_used=true;
        R.nominal_global_deviation_minimum_proven=true;
        R.elapsed_ms=toc(start_tick)*1000;
        return
    end
end
if ns==0
    if isempty(all_obstacle_ids)
        R.status='NO_OBSERVED_OBSTACLES__NO_AVOIDANCE_SOLVE_REQUIRED';
        R.reason='No obstacles known to sensor memory';
    else
        R.status='OBSERVED_OBSTACLES_PROVEN_OUTSIDE_SPEED_REACHABLE_SET';
        R.reason=['Every known obstacle is separated from every full-body ' ...
            'path satisfying the externally supplied and enforced speed bounds'];
    end
    R.elapsed_ms=toc(start_tick)*1000;return;
end
N=zeros(3,3*ns*nleaf);
% No additional NLP, no fixed global bypass direction. An explicit
% caller-owned warm-start budget may generate ONE independently verified
% corridor-compatible initial point before the single fmincon call.
% Never replace the ORIGINAL NOMINAL objective anchor with this seed.
anchor_free=zfree;
starting_free=anchor_free;
warm_used=false;
if isfield(s,'geometric_warm_start') && ~has_candidate_feasibility
    % No fabricated derivative limits: the initializer needs live bounds.
    R.warm_start_status='SKIPPED_NO_LIVE_DERIVATIVE_LIMITS';
elseif ~isfield(req,'initial_normals') && isfield(s,'geometric_warm_start')
    [seed,warm]=TRAJECTORY_C7_DIRECTION_NEUTRAL_SEED( ...
        req,shapes,u0,l0,inflation,nleaf);
    R.warm_start_status=warm.status;
    R.warm_start_attempts=warm.attempts;
    R.warm_start_ms=warm.elapsed_ms;
    if seed.available
        starting_free=seed.free_controls;
        N=seed.normals;
        warm_used=true;
        R.warm_start_used=true;
    end
end
if ~warm_used && isfield(req,'initial_normals')
    Ni=double(req.initial_normals);
    assert(isequal(size(Ni),size(N))&&all(isfinite(Ni(:))), ...
        'TRAJECTORY_SIMULTANEOUS:Normals','Invalid initial normal array.');
    mag=sqrt(sum(Ni.^2,1));
    assert(all(mag>0),'TRAJECTORY_SIMULTANEOUS:Normals', ...
        'No arbitrary direction assigned to a zero initial normal.');
    N=Ni./mag;
elseif ~warm_used
    P0=u0.control_points_m;Q0=l0.control_points_m;
    for k=1:nleaf
        U=P0*leaves(k).map;L=Q0*leaves(k).map;
        for j=1:ns
            for b=1:3
                switch b
                    case 1,P=U;
                    case 2,P=L;
                    otherwise,P=[U,L];
                end
                d=mean(P,2)-shapes(j).center;
                if norm(d)==0
                    % Equivariant seed from the ACTUAL tangent and cable.
                    % Under complete symmetry there is no unique side;
                    % return UNKNOWN instead of hardcoding world +Y / +Z.
                    tangent=U(:,end)-U(:,1);
                    cable=mean(L-U,2);
                    d=cross(tangent,cable);
                end
                if ~(all(isfinite(d))&&norm(d)>0)
                    R.status='UNKNOWN_SYMMETRY_REQUIRES_DIRECTION_SEED';
                    R.reason='No coordinate-free nonzero plane initialisation';
                    R.elapsed_ms=toc(start_tick)*1000;return
                end
                N(:,plane_index(k,j,b,ns))=d/norm(d);
            end
        end
    end
end
z0=[double(starting_free);N(:)];
F=@(z)objective(z,anchor_free);
G=@(z)nonlinear(z);
% Measure the actual initial feasibility of this ONE optimization.
% This exposes poor initialization without allocating persistent logs.
[ci,ei]=G(z0);
R.initial_max_inequality_violation=max([0;ci(:)]);
R.initial_max_geometry_violation_m=max([0;ci(1:geom_rows)]);
if warm_used,R.warm_start_geometry_violation_m= ...
    R.initial_max_geometry_violation_m;end
R.initial_max_derivative_constraint_violation=max([0;ci(geom_rows+1:end)]);
R.initial_max_normal_equality_violation=max([0;abs(ei(:))]);
% A certified geometric seed permits a MUCH smaller online problem:
% with its obstacle separation normals fixed, the corridor inequalities
% are affine in all 48 interior control coordinates. Rather than asking
% fmincon to optimize these coordinates and hundreds of plane variables
% again, move the seed toward the nominal along the maximum feasible ray.
% The derivative-norm inequalities are convex; independent whole-time
% geometry, heading, C7 and kinematic certificates remain mandatory.
% This does NOT solve the globally nonconvex multi-obstacle problem and
% never authorizes an HLC reference change. Solver ADAPTER tests keep the
% original fmincon-equivalent callback path for gradient regressions.
if warm_used && ~isfield(req,'solver_function')
    ray_tick=tic;
    znom=[double(anchor_free);N(:)];
    % G(z0) was already evaluated above as [ci,ei]. Repeating it merely
    % rebuilds every 3-D support constraint and derivative norm, without
    % changing the decision vector or any certificate.
    cs=ci;
    [cn,~]=G(znom);
    bound=TRAJECTORY_C7_CORRIDOR_RAY( ...
        cs,cn,geom_rows,starting_free,anchor_free);
    R.ray_constraint_stage_ms=toc(ray_tick)*1000;
    if bound.available && ...
            all(isfinite(ei(:))) && ...
            max([0;abs(ei(:))])<=s.feasibility_tolerance
        zray=[bound.free_controls;N(:)];
        % With FIXED unit normals, each geometry residual is affine in
        % the 48 free control coordinates. Hence the ray geometry rows
        % equal (1-alpha)*cs + alpha*cn, and the algebraic bound proves
        % they are <=0. Derivative control vectors are also affine;
        % their Euclidean norm is convex, so the ray satisfies every
        % derivative bound if BOTH endpoints satisfy it (checked by the
        % ray function). Unit-normal equalities do not change.
        % Thus another full G(zray) is redundant. In all cases below we
        % STILL require INDEPENDENT full-time kinematic/heading/geometry
        % certificates AND a fresh, exact support-plane corridor proof.
        % A floating-point mistake must not become an HLC commit.
        candidates={zray,z0};
        for candidate_index=1:numel(candidates)
            zz=candidates{candidate_index};
            if ~all(isfinite(zz(:)))
                continue
            end
            [u_cert,l_cert]=pieces(zz);
            independent_tick=tic;
            [cert_ok,cert_proof]=certify_unchanged_nominal(u_cert,l_cert);
            R.ray_independent_certificate_ms= ...
                R.ray_independent_certificate_ms+toc(independent_tick)*1000;
            if ~cert_ok,continue;end
            corridor_tick=tic;
            [cor_cert,minslack]=corridor_from_solution(zz,u_cert,l_cert);
            R.ray_corridor_certificate_ms= ...
                R.ray_corridor_certificate_ms+toc(corridor_tick)*1000;
            if ~cor_cert.certified,continue;end
            R.candidate_available=true;
            R.certified_geometry=true;
            R.solver_gradient_mode='NOT_APPLICABLE_NO_NLP';
            R.optimization_scope='CERTIFIED_CORRIDOR_SEED_OR_EXACT_RAY';
            R.pieces=struct('uav',u_cert,'payload',l_cert);
            R.corridor=cor_cert;
            R.minimum_plane_slack_m=minslack;
            R.independent_geometry=cert_proof.geometry;
            R.kinematic_certificate=cert_proof.kinematics;
            R.heading_certificate=cert_proof.heading;
            R.maximum_endpoint_jet_error=cert_proof.endpoint_error;
            R.certified_corridor_ray_selected=(candidate_index==1);
            R.warm_start_candidate_selected=(candidate_index==2);
            if candidate_index==1
                R.corridor_ray_fraction=bound.fraction;
                R.corridor_ray_deviation_reduction= ...
                    bound.integrated_deviation_reduction_proportion;
                R.status='CERTIFIED_CORRIDOR_RAY_REFINEMENT__DYNAMICS_PENDING';
                R.reason=['One-dimensional exact constrained refinement ' ...
                    'inside a certified fixed-normal full-body corridor. ' ...
                    'Continuous geometry and reference kinematics ' ...
                    'independently proven. Global optimality and coupled ' ...
                    'HLC/plant physical feasibility NOT proven.'];
            else
                R.status=['CERTIFIED_GEOMETRIC_WARM_START_NO_NLP__' ...
                    'DYNAMICS_PENDING'];
                R.reason=['The ray candidate did not pass every independent ' ...
                    'certificate. The original certified warm start is ' ...
                    'retained. No nonlinear solver was required; ' ...
                    'physical feasibility is NOT certified.'];
            end
            R.corridor_ray_ms=toc(ray_tick)*1000;
            R.elapsed_ms=toc(start_tick)*1000;
            return
        end
    end
    R.corridor_ray_ms=toc(ray_tick)*1000;
end
R.solver_called=true;
try
    if isfield(req,'solver_function')
        assert(isa(req.solver_function,'function_handle'), ...
            'TRAJECTORY_SIMULTANEOUS:Solver','Solver adapter must be callable.');
        [z,exitflag,output]=req.solver_function(F,z0,G,req.solver_options);
    else
        if exist('fmincon','file')~=2
            R.status='UNKNOWN_FMINCON_NOT_AVAILABLE';
            R.reason='Provide Optimization Toolbox fmincon or external solver adapter';
            R.elapsed_ms=toc(start_tick)*1000;return
        end
        % fmincon's default finite-difference Jacobian requires a full
        % multi-shape nonlinear evaluation for EACH optimization variable.
        % The derivatives below are exact at smooth points, with valid
        % subgradients at canonical shape edges / zero derivative norms.
        if isstruct(req.solver_options)
            o=optimset(req.solver_options,'GradObj','on','GradConstr','on');
        else
            o=optimoptions(req.solver_options, ...
                'SpecifyObjectiveGradient',true, ...
                'SpecifyConstraintGradient',true);
        end
        [z,~,exitflag,output]=fmincon(F,z0,[],[],[],[],[],[],G,o);
    end
catch ME
    R.status='UNKNOWN_SOLVER_EXCEPTION';
    R.reason=['Solver did not complete: ' ME.identifier];
    R.elapsed_ms=toc(start_tick)*1000;return
end
R.solve_exitflag=exitflag;R.solve_output=output;
R.solver_positive_exitflag=isnumeric(exitflag)&&isscalar(exitflag)&& ...
    isreal(exitflag)&&isfinite(exitflag)&&exitflag>0;
if isstruct(output) && isfield(output,'message')
    msg=output.message;
    if ischar(msg) || (isstring(msg)&&isscalar(msg))
        R.solver_termination_message=char(msg);
    end
end
R.solver_returned_finite_point=isnumeric(z)&&isreal(z)&& ...
    numel(z)==numel(z0)&&all(isfinite(z(:)));
if ~R.solver_returned_finite_point
    R.status='UNKNOWN_OPTIMIZER_FAILED';
    R.reason='Solver returned a missing or nonfinite decision vector';
    R.elapsed_ms=toc(start_tick)*1000;return
end
z=double(z(:));
[c,ceq]=G(z);
if all(isfinite(c))&&all(isfinite(ceq))
    R.final_max_inequality_violation=max([0;c(:)]);
    R.final_max_geometry_violation_m=max([0;c(1:geom_rows)]);
    R.final_max_derivative_constraint_violation=max([0;c(geom_rows+1:end)]);
    R.final_max_normal_equality_violation=max([0;abs(ceq(:))]);
    R.solver_point_feasible=R.final_max_inequality_violation<= ...
        s.feasibility_tolerance && ...
        R.final_max_normal_equality_violation<=s.feasibility_tolerance;
end
% A non-positive optimizer EXITFLAG is NOT proof of a collision or an
% infeasible candidate. Conversely positive EXITFLAG is NOT a collision
% certificate. Accept ONLY a point satisfying all constraints followed
% by the independent geometry, heading and kinematics certificates.
if ~R.solver_point_feasible
    % A local nonconvex solver may leave a valid initial point and return
    % an infeasible one. Keep that information, but do not discard an
    % independently certifiable C7 geometrical candidate. This is NOT a
    % claim of optimality, physical feasibility or HLC permission.
    if warm_used
        [seed_up,seed_lp]=pieces(z0);
        [seed_corridor,seed_slack]=corridor_from_solution( ...
            z0,seed_up,seed_lp);
        if seed_corridor.certified
            [seed_ok,seed_proof]=certify_unchanged_nominal( ...
                seed_up,seed_lp);
            if seed_ok
                R.status=['CERTIFIED_GEOMETRIC_WARM_START_FALLBACK__' ...
                    'NOT_OPTIMIZED__DYNAMICS_PENDING'];
                R.reason=['The actual nonlinear solver returned an ' ...
                    'infeasible point. A geometrically and kinematically ' ...
                    'certified initial candidate is retained instead. ' ...
                    'It is NOT optimal and CANNOT be committed to HLC.'];
                R.warm_start_candidate_selected=true;
                R.pieces=struct('uav',seed_up,'payload',seed_lp);
                R.corridor=seed_corridor;
                R.minimum_plane_slack_m=seed_slack;
                R.certified_geometry=true;
                R.candidate_available=true;
                R.independent_geometry=seed_proof.geometry;
                R.kinematic_certificate=seed_proof.kinematics;
                R.heading_certificate=seed_proof.heading;
                R.maximum_endpoint_jet_error=seed_proof.endpoint_error;
                R.elapsed_ms=toc(start_tick)*1000;
                return
            end
        end
    end
    R.status='UNKNOWN_OPTIMIZER_FAILED';
    R.reason=sprintf(['No certifiably feasible NLP point; exitflag=%g, ' ...
        'initial geometry=%.9g m, final geometry=%.9g m, ' ...
        'final derivative residual=%.9g (mixed derivative units), ' ...
        'normal error=%.9g.'],double(exitflag), ...
        R.initial_max_geometry_violation_m, ...
        R.final_max_geometry_violation_m, ...
        R.final_max_derivative_constraint_violation, ...
        R.final_max_normal_equality_violation);
    R.elapsed_ms=toc(start_tick)*1000;return
end
[up,lp]=pieces(z);
if has_candidate_feasibility
    % Independent whole-time check uses the SAME live bound source, but
    % a different implementation from the NLP constraint transcription.
    R.kinematic_certificate.uav=TRAJECTORY_KINEMATIC_CERTIFICATE( ...
        up,cf.derivative_limits.uav, ...
        struct('max_subdivision_depth',s.time_leaf_depth));
    R.kinematic_certificate.payload=TRAJECTORY_KINEMATIC_CERTIFICATE( ...
        lp,cf.derivative_limits.payload, ...
        struct('max_subdivision_depth',s.time_leaf_depth));
    if ~(R.kinematic_certificate.uav.certified && ...
            R.kinematic_certificate.payload.certified)
        R.status='REJECT_REFERENCE_DERIVATIVE_LIMITS';
        R.reason='No full-time derivative certificate from the optimized controls';
        R.elapsed_ms=toc(start_tick)*1000;return
    end
    R.heading_certificate=TRAJECTORY_HEADING_CERTIFICATE( ...
        up,cf.heading_options);
    if ~R.heading_certificate.certified
        R.status='UNKNOWN_HEADING_C6_SINGULARITY';
        R.reason='Path-tangent yaw is not certified across the full interval';
        R.elapsed_ms=toc(start_tick)*1000;return
    end
end
R.maximum_endpoint_jet_error=max([jet_error(up,req.start_jets.uav,req.end_jets.uav), ...
    jet_error(lp,req.start_jets.payload,req.end_jets.payload)]);
if R.maximum_endpoint_jet_error>s.endpoint_jet_tolerance
    R.status='UNKNOWN_C7_ENDPOINT_MISMATCH';
    R.reason='Reconstructed 7-jets exceed caller-owned numerical tolerance';
    R.elapsed_ms=toc(start_tick)*1000;return
end
% Rebuild a certified linear corridor DIRECTLY from the optimized normals.
% Do not re-run a second geometry search just to rediscover its planes.
[C,minslack]=corridor_from_solution(z,up,lp);
R.minimum_plane_slack_m=minslack;
if ~C.certified
    R.status='UNKNOWN_PLANE_PROOF_FAILED';
    R.reason='Independent plane/support check failed';
    R.elapsed_ms=toc(start_tick)*1000;return
end
pieces_pair=struct('uav',up,'payload',lp);
% Independent whole-time geometry check, necessary because NLP success or
% floating-point constraint tolerance alone is NOT a certificate.
try
    verification=TRAJECTORY_CONTINUOUS_GEOMETRY_CERTIFICATE( ...
        pieces_pair,req.observed,req.physical_radii,req.proof_options);
catch ME
    R.status='UNKNOWN_INDEPENDENT_CERTIFICATE_EXCEPTION';
    R.reason=['Independent geometry checker failed: ' ME.identifier];
    R.elapsed_ms=toc(start_tick)*1000;return
end
R.independent_geometry=verification;
if ~verification.certified
    R.status='UNKNOWN_INDEPENDENT_CONTINUOUS_GEOMETRY';
    R.reason='The independent whole-time geometry certificate did not pass';
    R.elapsed_ms=toc(start_tick)*1000;return
end
R.pieces=pieces_pair;
R.corridor=C;
R.certified_geometry=true;
R.candidate_available=true;
if has_candidate_feasibility
    R.status='CERTIFIED_GEOMETRY_AND_REFERENCE_KINEMATICS__DYNAMICS_PENDING';
else
    R.status='CERTIFIED_NOMINAL_CONTINUOUS_GEOMETRY__DYNAMICS_PENDING';
end
if ~R.solver_positive_exitflag
    % A feasible, independently certified point is useful for geometry,
    % even if fmincon stopped on its iteration budget. Never label it
    % an optimized solution or permit HLC commitment.
    R.status=[R.status '__FEASIBLE_BUT_OPTIMALITY_NOT_ESTABLISHED'];
end
R.reason=['One candidate passed every independent nominal geometry and ' ...
    'kinematics certificate. Solver termination is separately recorded; ' ...
    'HLC actuation, taut dynamics, tracking, tension and model errors remain pending.'];
R.elapsed_ms=toc(start_tick)*1000;

    function [ok,proof]=certify_unchanged_nominal(up,lp)
        % Runs only before a REAL solver call; no choice of bypass side,
        % no solver iteration and no approximation of the original curve.
        proof=struct('geometry',struct(), ...
            'kinematics',struct(),'heading',struct(),'endpoint_error',NaN);
        ok=false;
        proof.endpoint_error=max([jet_error(up,req.start_jets.uav,req.end_jets.uav), ...
            jet_error(lp,req.start_jets.payload,req.end_jets.payload)]);
        if proof.endpoint_error>s.endpoint_jet_tolerance,return;end
        if has_candidate_feasibility
            ku=TRAJECTORY_KINEMATIC_CERTIFICATE( ...
                up,cf.derivative_limits.uav, ...
                struct('max_subdivision_depth',s.time_leaf_depth));
            kl=TRAJECTORY_KINEMATIC_CERTIFICATE( ...
                lp,cf.derivative_limits.payload, ...
                struct('max_subdivision_depth',s.time_leaf_depth));
            proof.kinematics=struct('uav',ku,'payload',kl);
            if ~(ku.certified&&kl.certified),return;end
            proof.heading=TRAJECTORY_HEADING_CERTIFICATE(up,cf.heading_options);
            if ~proof.heading.certified,return;end
        end
        % The independent geometry checker adds tube inflation and its
        % numerical guard, but not the optimizer's explicit separation
        % margin. Transfer that positive margin into the effective body
        % radii so this no-solve result proves the SAME clearance.
        inflated_radii=req.physical_radii;
        for body_index=1:3
            key=parts{body_index};
            inflated_radii.(key)=inflated_radii.(key)+opt.min_separation_m;
        end
        proof.geometry=TRAJECTORY_CONTINUOUS_GEOMETRY_CERTIFICATE( ...
            struct('uav',up,'payload',lp),req.observed, ...
            inflated_radii,req.proof_options);
        ok=proof.geometry.certified;
    end

    function [val,grad]=objective(z,anchor)
        % Integral 0.5 * int (|U-U_nominal|^2 + |L-L_nominal|^2) dt.
        % This is a physical spatial trajectory deviation metric with NO
        % hand-tuned obstacle weights, and is computed EXACTLY in Bernstein.
        du=reshape(z(1:24)-double(anchor(1:24)),3,8);
        dl=reshape(z(25:48)-double(anchor(25:48)),3,8);
        gu=du*gram;gl=dl*gram;
        val=0.5*(sum(sum(du.*gu))+sum(sum(dl.*gl)));
        if nargout>1
            grad=[gu(:);gl(:);zeros(numel(z)-48,1)];
        end
    end
    function [U,L]=control_points(z)
        % Hot nonlinear-constraint path: do NOT reconstruct all seven
        % derivative polygons at every nonlinear solver evaluation.
        U=[u0.control_points_m(:,1:8),reshape(z(1:24),3,8), ...
            u0.control_points_m(:,17:24)];
        L=[l0.control_points_m(:,1:8),reshape(z(25:48),3,8), ...
            l0.control_points_m(:,17:24)];
    end
    function [up,lp]=pieces(z)
        A=reshape(z(1:24),3,8);B=reshape(z(25:48),3,8);
        up=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
            req.start_jets.uav,req.end_jets.uav,A);
        lp=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
            req.start_jets.payload,req.end_jets.payload,B);
    end
    function [c,ceq,GC,GCeq]=nonlinear(z)
        % Identical constraint values for both adapter and fmincon paths;
        % fmincon additionally receives analytic gradients directly.
        if nargout>2
            [c,ceq,GC,GCeq]=TRAJECTORY_C7_NLP_ANALYTIC( ...
                z,u0.control_points_m,l0.control_points_m,leaf_maps, ...
                shapes,inflation,req.duration_s,active_limits(),prepared_nlp);
        else
            [c,ceq]=TRAJECTORY_C7_NLP_ANALYTIC( ...
                z,u0.control_points_m,l0.control_points_m,leaf_maps, ...
                shapes,inflation,req.duration_s,active_limits(),prepared_nlp);
            GC=[];GCeq=[];
        end
    end
    function active=active_limits()
        active=[];
        if has_candidate_feasibility
            active=cf.derivative_limits;
        end
    end
    function [C,minslack]=corridor_from_solution(z,up,lp)
        nn=reshape(z(49:end),3,[]);
        U=up.control_points_m;L=lp.control_points_m;
        minslack=Inf;
        ll=repmat(struct('segment_index',1,'time_interval_s',[0 0], ...
            'uav_map',eye(24),'payload_map',eye(24), ...
            'uav_normals',zeros(3,0),'uav_offsets',zeros(0,1), ...
            'payload_normals',zeros(3,0),'payload_offsets',zeros(0,1), ...
            'cable_normals',zeros(3,0),'cable_offsets',zeros(0,1), ...
            'obstacle_ids',zeros(1,0)),1,nleaf);
        proven=true;
        for k=1:nleaf
            uk=U*leaves(k).map;lk=L*leaves(k).map;
            ll(k).time_interval_s=leaves(k).time_interval_s;
            ll(k).uav_map=leaves(k).map;
            ll(k).payload_map=leaves(k).map;
            ll(k).obstacle_ids=reshape([shapes.id],1,[]);
            for j=1:ns
                for b=1:3
                    h=plane_index(k,j,b,ns);
                    n=nn(:,h);mag=norm(n);
                    if ~(isfinite(mag)&&mag>0)
                        proven=false;continue
                    end
                    n=n/mag;
                    beta=TRAJECTORY_OBSTACLE_SUPPORT.value(shapes(j),n)+inflation(b);
                    switch b
                        case 1,P=uk;key='uav';
                        case 2,P=lk;key='payload';
                        otherwise,P=[uk lk];key='cable';
                    end
                    slack=min(n.'*P)-beta;
                    minslack=min(minslack,slack);
                    if ~(isfinite(slack)&&slack>=0)
                        proven=false;
                    end
                    switch key
                        case 'uav'
                            ll(k).uav_normals(:,end+1)=n;
                            ll(k).uav_offsets(end+1,1)=beta;
                        case 'payload'
                            ll(k).payload_normals(:,end+1)=n;
                            ll(k).payload_offsets(end+1,1)=beta;
                        otherwise
                            ll(k).cable_normals(:,end+1)=n;
                            ll(k).cable_offsets(end+1,1)=beta;
                    end
                end
            end
        end
        info=struct('t0_s',req.t0_s,'duration_s',req.duration_s, ...
            'uav_controls',24,'payload_controls',24, ...
            'uav_variable_start',1,'payload_variable_start',73);
        C=struct('certified',proven,'safe_to_commit',false, ...
            'status','UNKNOWN_PLANE_CERTIFICATE', ...
            'scope','NOMINAL_POLYNOMIAL_ONLY', ...
            'observed_obstacle_ids',reshape([shapes.id],1,[]), ...
            'all_observed_obstacle_ids',all_obstacle_ids, ...
            'proven_unreachable_obstacle_ids',pruned_ids, ...
            'speed_constraint_dependent',~isempty(pruned_ids), ...
            'observed_snapshot',{req.observed}, ...
            'effective_radii_m',r, ...
            'proof_margin_m',opt.min_separation_m, ...
            'numerical_guard_m',opt.numerical_guard_m, ...
            'leaves',ll,'segment_info',info,'variable_count',144, ...
            'halfspace_count',3*ns*nleaf,'leaf_count',nleaf, ...
            'min_plane_slack_m',minslack);
        if opt.robust_tube_enabled
            C.scope='POLYNOMIAL_WITH_SUPPLIED_UNCERTAINTY_RADII';
        end
        if proven
            if isempty(pruned_ids)
                C.status='CERTIFIED_SINGLE_SOLVE_FULL_BODY_CORRIDOR';
            else
                % This corridor alone does not constrain previously
                % excluded obstacles: their exclusion is valid ONLY with
                % the enforced whole-time speed bounds. The existing
                % standalone corridor validator fail-closes if called
                % with the full observed set, rather than silently
                % treating the conditional subset as a full corridor.
                C.status='CERTIFIED_ACTIVE_CORRIDOR__SPEED_BOUNDS_REQUIRED';
            end
        end
    end
end

function e=jet_error(p,start_jets,end_jets)
% Check the actually evaluated derivative control endpoints, not metadata.
e=0;
for k=1:8
    D=p.derivative_controls{k};
    e=max(e,max(abs(D(:,1)-start_jets(:,k))));
    e=max(e,max(abs(D(:,end)-end_jets(:,k))));
end
end
function idx=plane_index(k,j,b,ns)
idx=3*((k-1)*ns+j-1)+b;
end
function leaves=make_leaves(depth,t0,T)
% Exact de Casteljau linear transforms; no sampling approximation.
leaves=struct('map',eye(24),'time_interval_s',[t0,t0+T]);
for d=1:depth
    fresh=repmat(leaves(1),1,2*numel(leaves));
    for k=1:numel(leaves)
        [A,B]=split_mid(leaves(k).map);
        tm=mean(leaves(k).time_interval_s);
        fresh(2*k-1).map=A;
        fresh(2*k-1).time_interval_s= ...
            [leaves(k).time_interval_s(1),tm];
        fresh(2*k).map=B;
        fresh(2*k).time_interval_s= ...
            [tm,leaves(k).time_interval_s(2)];
    end
    leaves=fresh;
end
end
function [A,B]=split_mid(M)
% Works for arbitrary row count, including the 24x24 control mapping.
N=size(M,2);Q=M;A=zeros(size(M));B=zeros(size(M));
A(:,1)=Q(:,1);B(:,N)=Q(:,N);
for j=2:N
    Q=(Q(:,1:end-1)+Q(:,2:end))*0.5;
    A(:,j)=Q(:,1);B(:,N-j+1)=Q(:,end);
end
end

function G=bernstein_free_gram_23()
% Exact Gram matrix int_0^1 B_i^23(s)*B_j^23(s) ds, i,j=8..15.
% These are the eight controls FREE after fixing the boundary 7-jets.
n=23;free=8:15;G=zeros(8,8);
for a=1:8
    for b=1:8
        i=free(a);j=free(b);
        G(a,b)=nchoosek(n,i)*nchoosek(n,j)/ ...
            ((2*n+1)*nchoosek(2*n,i+j));
    end
end
G=0.5*(G+G.');
end
