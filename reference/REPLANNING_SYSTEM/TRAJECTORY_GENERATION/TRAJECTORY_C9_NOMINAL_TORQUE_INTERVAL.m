function R = TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL(piece,authority,options)
% TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL
% Continuous-time REAL-ARITHMETIC sufficient certificate for NOMINAL
% rigid-body torques implied by a taut-load C9 flat-output trajectory.
%
% Unlike sampled inverse-dynamics checks, every derivative of the payload
% polynomial is enclosed by its Bernstein convex hull on each time leaf.
% Interval differentiation encloses q=w/||w|| through derivative 4,
% F=-(mU+mL)*w - mU*L*q'' through derivative 2, and heading-exact R through
% derivative 2. M = J*omega' + cross(omega,J*omega) is then enclosed.
%
% Body yaw is EXACTLY the planar path heading: b1 has horizontal projection
% in the direction of pU'_xy provided F_z>0, rather than merely aligning a
% projected intermediate yaw vector with b3.
%
% The optional [mass_lo,mass_hi] range is over a CONSTANT physical mass
% chosen anywhere in that externally certified interval; it does NOT model
% an actual time-varying physical mass, derivatives of the EKF estimate,
% actuator dynamics, closed-loop tracking, or uncertain HLC/plant behavior.
% For actual mL(t), bounds on dmL/dt and d2mL/dt2 are additionally necessary.
% Hence no outcome from this function ever authorizes HLC COMMIT.
%
% Real-arithmetic Bernstein/interval certificate only. MATLAB floating-point
% interval arithmetic below DOES NOT use directed rounding; a separate
% numerical enclosure error budget must be certified before safety-critical
% use. Failure or exhausted budgets result in UNKNOWN, not collision.
clock=tic;
R=struct('status','UNKNOWN_NOMINAL_TORQUE_NOT_PROVEN', ...
    'conditional_nominal_all_time_torque_proven',false, ...
    'safe_to_commit',false,'actual_HLC_tracking_proven',false, ...
    'numerically_directed_rounding_proven',false, ...
    'constant_physical_mass_assumed',true, ...
    'certified_exact_stationary_yaw_hold',false, ...
    'stationary_yaw_anchor_rad',NaN, ...
    'mass_scope','NO_VALID_AUTHORITY_YET', ...
    'payload_mass_snapshot_kg',NaN, ...
    'checked_nodes',0,'accepted_leaves',0, ...
    'fast_norm_checked_nodes',0,'fast_norm_accepted_leaves',0, ...
    'fast_norm_exception_count',0,'fast_norm_enabled',true, ...
    'precomputed_binomial_enabled',true, ...
    'factored_diagonal_gyro_enabled',true, ...
    'infeasible_leaf_early_exit_enabled',true, ...
    'nominal_torque_limit_witness_nodes',0, ...
    'interval_product_kernel','VECTORIZED_DOT_CROSS', ...
    'symmetric_unit_jet_enabled',true, ...
    'cancellation_attitude_jet_enabled',false, ...
    'structural_torque_intersection_enabled',false, ...
    'structural_torque_attempted_nodes',0, ...
    'structural_torque_tightened_nodes',0, ...
    'structural_torque_certified_leaves',0, ...
    'structural_torque_unusable_nodes',0, ...
    'structural_torque_exception_nodes',0, ...
    'structural_torque_last_exception','', ...
    'structural_torque_intersection_max_shrink_Nm',zeros(3,2), ...
    'diagnostic_point_probe_enabled',false, ...
    'diagnostic_point_probe_times_s',zeros(1,0), ...
    'diagnostic_point_torque_lower_Nm',zeros(3,0), ...
    'diagnostic_point_torque_upper_Nm',zeros(3,0), ...
    'diagnostic_point_valid',false(1,0), ...
    'diagnostic_point_outside_limits_unrounded',false(1,0), ...
    'diagnostic_point_first_outside_time_s',NaN, ...
    'diagnostic_point_max_margin_violation_Nm',NaN, ...
    'diagnostic_point_elapsed_ms',0, ...
    'unknown_midpoint_diagnostic_enabled',false, ...
    'unresolved_midpoint_s',NaN, ...
    'unresolved_midpoint_torque_bounds_Nm',NaN(3,2), ...
    'unresolved_midpoint_valid',false, ...
    'unresolved_midpoint_outside_limits_unrounded',false, ...
    'unresolved_midpoint_inside_limits_unrounded',false, ...
    'unresolved_torque_bounds_Nm',NaN(3,2), ...
    'unresolved_torque_interval_valid',false, ...
    'unresolved_torque_interval_headroom_Nm',NaN, ...
    'detailed_interval_checked_nodes',0, ...
    'unresolved_interval_s',[NaN NaN], ...
    'unresolved_depth',NaN, ...
    'unresolved_leaf_duration_s',NaN, ...
    'last_leaf_reason','NOT_EVALUATED', ...
    'last_exception_identifier','','last_exception_message','', ...
    'torque_bounds_Nm',NaN(3,2), ...
    'minimum_torque_headroom_Nm',NaN, ...
    'elapsed_ms',NaN);
try
    assert(isstruct(piece)&&isscalar(piece)&&all(isfield(piece, ...
        {'t0_s','duration_s','derivative_controls'}))&& ...
        isfinite(piece.t0_s)&&isfinite(piece.duration_s)&&piece.duration_s>0, ...
        'C9_TORQUE:Piece','Invalid C9 piece');
    assert(isstruct(authority)&&isscalar(authority)&&all(isfield(authority, ...
        {'uav_mass_kg','payload_mass_kg','cable_length_m', ...
        'gravity_world_mps2','uav_inertia_kgm2','torque_limits_Nm'})), ...
        'C9_TORQUE:Authority','Need current HLC/plant physical authority');
    assert(isstruct(options)&&isscalar(options)&&all(isfield(options, ...
        {'max_subdivision_depth','max_node_count','minimum_force_z_N', ...
        'minimum_planar_speed_mps','torque_guard_Nm'})), ...
        'C9_TORQUE:Options','No invented physical or numerical guards');
    mu=double(authority.uav_mass_kg);
    m0=double(authority.payload_mass_kg);
    L=double(authority.cable_length_m);
    g=double(authority.gravity_world_mps2(:));
    J=double(authority.uav_inertia_kgm2(:));
    limits=double(authority.torque_limits_Nm);
    guard=double(options.torque_guard_Nm(:));
    minFz=double(options.minimum_force_z_N);
    minV=double(options.minimum_planar_speed_mps);
    dep=double(options.max_subdivision_depth);
    budget=double(options.max_node_count);
    enable_fast_norm=true;
    if isfield(options,'enable_fast_norm_sufficient_bound')
        user_switch=options.enable_fast_norm_sufficient_bound;
        assert((islogical(user_switch)||isnumeric(user_switch)) && ...
            isscalar(user_switch)&&isreal(user_switch)&& ...
            isfinite(double(user_switch))&& ...
            any(double(user_switch)==[0 1]), ...
            'C9_TORQUE:FastNormSwitch','Fast norm switch must be logical 0/1');
        enable_fast_norm=logical(user_switch);
    end
    R.fast_norm_enabled=enable_fast_norm;
    % Binomial orders are intrinsic to the fourth-order taut-load jet,
    % not tuneable robot or obstacle parameters. This saves repeated calls
    % to MATLAB nchoosek inside each interval subdivision.  The OFF
    % switch exists only to compare against the original arithmetic.
    fast_binomial=true;
    if isfield(options,'use_precomputed_binomial')
        flag=options.use_precomputed_binomial;
        assert((islogical(flag)||isnumeric(flag))&&isscalar(flag)&& ...
            isreal(flag)&&isfinite(double(flag))&& ...
            (double(flag)==0||double(flag)==1), ...
            'C9_TORQUE:BinomialSwitch', ...
            'use_precomputed_binomial must be 0 or 1.');
        fast_binomial=logical(flag);
    end
    R.precomputed_binomial_enabled=fast_binomial;
    % Algebraically exact rigid-body Euler identity for diagonal inertia:
    %   [Jx ax+(Jz-Jy)*wy*wz;
    %    Jy ay+(Jx-Jz)*wz*wx;
    %    Jz az+(Jy-Jx)*wx*wy].
    % Unlike a generic interval cross product, it preserves the shared
    % angular-velocity factor and cancels equal inertia terms BEFORE
    % enclosure.  The user switch permits direct A/B comparison.
    factor_gyro=true;
    if isfield(options,'use_factored_diagonal_gyro')
        v=options.use_factored_diagonal_gyro;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:GyroSwitch', ...
            'use_factored_diagonal_gyro must be logical 0/1.');
        factor_gyro=logical(v);
    end
    R.factored_diagonal_gyro_enabled=factor_gyro;
    % A rigorously disjoint torque interval at any nonempty time leaf
    % already rules out this NOMINAL C9 candidate. No need to refine the
    % leaf; report UNKNOWN as before, never a safety certificate.
    witness_exit=true;
    if isfield(options,'enable_infeasible_leaf_early_exit')
        v=options.enable_infeasible_leaf_early_exit;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:WitnessSwitch', ...
            'enable_infeasible_leaf_early_exit must be logical 0/1.');
        witness_exit=logical(v);
    end
    R.infeasible_leaf_early_exit_enabled=witness_exit;
    % The norm derivatives involve symmetric products X_k . X_(n-k).
    % Reusing identical pairs and enclosing X_k.X_k by sums of SQUARES
    % avoids duplicate interval work and tightens even-order derivatives.
    % The OFF branch is the original implementation for A/B auditing.
    sym_unit=true;
    if isfield(options,'use_symmetric_unit_jet_kernel')
        v=options.use_symmetric_unit_jet_kernel;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:UnitJetSwitch', ...
            'use_symmetric_unit_jet_kernel must be logical 0/1.');
        sym_unit=logical(v);
    end
    R.symmetric_unit_jet_enabled=sym_unit;
    % Exact attitude identity for F_z>0 and ||v_xy||>0:
    %   unit(cross(z x unit(v_xy), unit(F)))
    %        == unit(cross(z x v_xy, F)).
    % Both scalar denominators are strictly positive and cancel through
    % final normalization. The cancelled jet removes TWO nested interval
    % normalizations from the b1 path, preserving all time derivatives.
    % Keep this optional and OFF until MATLAB A/B confirms throughput and
    % proof-success behaviour on real fixtures.  This is NOT a sampling
    % check; each branch is a continuous-time interval inclusion.
    cancel_attitude=false;
    if isfield(options,'use_cancellation_attitude_jet')
        v=options.use_cancellation_attitude_jet;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:AttitudeCancellationSwitch', ...
            'use_cancellation_attitude_jet must be logical 0/1.');
        cancel_attitude=logical(v);
    end
    R.cancellation_attitude_jet_enabled=cancel_attitude;
    % Structural angular-velocity identity uses the UNNORMALIZED force F
    % and a=(ez x horizontal_velocity) x F. The two interval extensions
    % enclose the same nominal torque. Intersecting them cannot widen the
    % enclosure; if their floating-point intersections are inconsistent,
    % return UNKNOWN (directed-rounding proof is still outstanding).
    % Default OFF until MATLAB comparison on real fixtures.
    structural_torque=false;
    if isfield(options,'use_structural_torque_intersection')
        v=options.use_structural_torque_intersection;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:StructuralSwitch', ...
            'use_structural_torque_intersection must be logical 0/1.');
        structural_torque=logical(v);
    end
    R.structural_torque_intersection_enabled=structural_torque;
    % Diagnostics never run on the normal online proof path by default.
    midpoint_diag=false;
    if isfield(options,'enable_unknown_midpoint_diagnostic')
        v=options.enable_unknown_midpoint_diagnostic;
        assert((islogical(v)||isnumeric(v))&&isscalar(v)&& ...
            isreal(v)&&isfinite(double(v))&&any(double(v)==[0 1]), ...
            'C9_TORQUE:MidpointDiagSwitch', ...
            'enable_unknown_midpoint_diagnostic must be logical 0/1.');
        midpoint_diag=logical(v);
    end
    R.unknown_midpoint_diagnostic_enabled=midpoint_diag;
    % OFFLINE DIAGNOSTIC ONLY. The caller explicitly supplies times on the
    % ACTUAL Bernstein polynomial. No point sample may prove an all-time
    % property, and these non-outward-rounded values are not a numerical
    % certificate of a physical limit violation. No change to COMMIT.
    point_probe_times=zeros(1,0);
    if isfield(options,'diagnostic_point_probe_times_s')
        point_probe_times=double(options.diagnostic_point_probe_times_s);
        assert(isnumeric(options.diagnostic_point_probe_times_s)&& ...
            isreal(options.diagnostic_point_probe_times_s)&& ...
            isvector(point_probe_times)&& ...
            all(isfinite(point_probe_times(:)))&& ...
            all(point_probe_times(:)>=piece.t0_s)&& ...
            all(point_probe_times(:)<=piece.t0_s+piece.duration_s), ...
            'C9_TORQUE:PointProbeTimes', ...
            'Probe times must lie in the actual C9 piece.');
        point_probe_times=point_probe_times(:).';
    end
    R.diagnostic_point_probe_enabled=~isempty(point_probe_times);
    R.diagnostic_point_probe_times_s=point_probe_times;
    % Pascal rows n=0..4, coefficients k=0..n.  Exactly integer-valued
    % in binary64; the same numbers returned by nchoosek for these orders.
    B=[1 0 0 0 0; ...
       1 1 0 0 0; ...
       1 2 1 0 0; ...
       1 3 3 1 0; ...
       1 4 6 4 1];
    assert(all(isfinite([mu,m0,L,g(:).',J(:).',limits(:).',guard(:).',minFz,minV,dep,budget]))&& ...
        all([mu,m0,L]>0)&&numel(g)==3&&numel(J)==3&&all(J>0)&& ...
        isequal(size(limits),[3 2])&&all(limits(:,2)>limits(:,1))&& ...
        numel(guard)==3&&all(guard>=0)&&minFz>=0&&minV>=0&& ...
        dep>=0&&dep==fix(dep)&&budget>=1&&budget==fix(budget), ...
        'C9_TORQUE:Authority','Malformed external physical limits or guard');
    % Point mass is sourced from this candidate's current live authority.
    % No uncertainty width is synthesized from the estimator.  If an
    % externally certified constant-physical-mass interval is supplied,
    % evaluate its entire range; it is not an EKF prediction.
    R.payload_mass_snapshot_kg=m0;
    R.mass_scope='ONE_CURRENT_MASS_SNAPSHOT__NOT_FUTURE_MASS_PROOF';
    ml=[m0 m0];
    if isfield(options,'certified_constant_physical_mass_interval_kg')
        R.mass_scope='EXTERNALLY_PROVIDED_CONSTANT_MASS_INTERVAL';
        ml=double(options.certified_constant_physical_mass_interval_kg(:).');
        assert(numel(ml)==2&&all(isfinite(ml))&&ml(1)>0&& ...
            ml(2)>=ml(1)&&ml(1)<=m0&&m0<=ml(2), ...
            'C9_TORQUE:Mass','Externally certified constant-mass interval required');
    end
    % Bind the certified derivative intervals to the ACTUAL polynomial.
    % Checking only caller-supplied derivative_controls would allow the
    % certificate to prove a different curve from control_points_m.
    assert(isfield(piece,'control_points_m')&& ...
        isnumeric(piece.control_points_m)&& ...
        isreal(piece.control_points_m)&& ...
        isequal(size(piece.control_points_m),[3 24])&& ...
        all(isfinite(piece.control_points_m(:))), ...
        'C9_TORQUE:PolynomialBinding','Require real degree-23 control points');
    D=piece.derivative_controls;
    assert(iscell(D)&&numel(D)==10, ...
        'C9_TORQUE:Jets','Need every C9 derivative polygon');
    expected=double(piece.control_points_m);
    hull=cell(1,7);
    for k=1:10
        assert(isnumeric(D{k})&&isreal(D{k})&& ...
            isequal(size(D{k}),[3 25-k])&& ...
            all(isfinite(D{k}(:)))&& ...
            isequal(double(D{k}),expected), ...
            'C9_TORQUE:JetsBinding', ...
            'Derivative polygon does not match the certified C9 piece');
        if k<=7
            hull{k}=expected;
        end
        if k<10
            expected=(24-k)/piece.duration_s*diff(expected,1,2);
        end
    end
    % Exact closed-form stationary case (NOT a sampled yaw fallback).
    % If the ACTUAL flat-output polynomial has constant horizontal
    % position and world gravity is vertical, w_xy and q_xy vanish.
    % w_z<0 over the WHOLE piece implies F_z>0 and a constant body
    % attitude for a CONSTANT externally anchored yaw. Consequently
    % omega=dot(omega)=M=0 for ALL t. The proof uses actual Bernstein
    % coefficients and externally supplied live mass and actuator bounds.
    % It cannot establish real HLC tracking or mass evolution in flight.
    if isfield(options,'stationary_yaw_anchor_rad')
        anchor=options.stationary_yaw_anchor_rad;
        assert(isnumeric(anchor)&&isreal(anchor)&&isscalar(anchor)&& ...
            isfinite(anchor), 'C9_TORQUE:StationaryYawAnchor', ...
            'Stationary yaw hold requires a real finite external anchor.');
        exact_rest=all(all(hull{1}(1:2,:)==hull{1}(1:2,1))) && ...
            all(g(1:2)==0);
        if exact_rest
            wz=g(3)-hull{3}(3,:); % g_z minus exact payload a_z hull
            f_lower=(mu+ml(1))*(-max(wz));
            zero_torque_allowed=all(limits(:,1)+guard<=0) && ...
                all(limits(:,2)-guard>=0);
            if max(wz)<0 && f_lower>minFz && ...
                    zero_torque_allowed && minV==0
                R.certified_exact_stationary_yaw_hold=true;
                R.stationary_yaw_anchor_rad=double(anchor);
                R.checked_nodes=1;R.accepted_leaves=1;
                R.torque_bounds_Nm=zeros(3,2);
                R.minimum_torque_headroom_Nm=min([ ...
                    -limits(:,1)-guard;limits(:,2)-guard],[],'all');
                R.conditional_nominal_all_time_torque_proven=true;
                R.status='CONDITIONAL_NOMINAL_EXACT_STATIONARY_YAW_ZERO_TORQUE__HLC_PENDING';
                R.elapsed_ms=1000*toc(clock);
                return
            end
            % Noncertified stationary inputs continue to the legacy
            % interval proof; that may return UNKNOWN, never a fake PASS.
        end
    end
    [ok,reason]=node(hull,0,1,0);
    if ok
        R.conditional_nominal_all_time_torque_proven=true;
        R.status='CONDITIONAL_NOMINAL_RIGID_BODY_TORQUE_ALL_TIME__HLC_TRACKING_PENDING';
        R.minimum_torque_headroom_Nm=min([ ...
            R.torque_bounds_Nm(:,1)-limits(:,1)-guard, ...
            limits(:,2)-guard-R.torque_bounds_Nm(:,2)],[],'all');
    else
        R.status=reason;
    end
    % A point probe is always a NONCERTIFYING post-proof diagnostic. It
    % never causes a nominal PROVEN result or changes oracle inputs.
    if ~isempty(point_probe_times)
        R=run_point_diagnostics(R,hull,point_probe_times);
    end
catch ME
    R.status=['UNKNOWN_TORQUE_CERTIFICATE_INPUT_' ME.identifier];
    R.last_exception_identifier=ME.identifier;
    R.last_exception_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);

    function S=run_point_diagnostics(S,polyjets,times)
        sw=tic;
        nt=numel(times);
        S.diagnostic_point_torque_lower_Nm=NaN(3,nt);
        S.diagnostic_point_torque_upper_Nm=NaN(3,nt);
        S.diagnostic_point_valid=false(1,nt);
        S.diagnostic_point_outside_limits_unrounded=false(1,nt);
        old_structural=structural_torque;
        structural_torque=false; % do not alter the full-time proof counters
        for it=1:nt
            try
                u=(times(it)-piece.t0_s)/piece.duration_s;
                jet=cell(size(polyjets));
                for j=1:numel(polyjets)
                    Q=polyjets{j};
                    for k=size(Q,2):-1:2
                        Q=(1-u)*Q(:,1:k-1)+u*Q(:,2:k);
                    end
                    jet{j}=Q;
                end
                [T,good]=torque_box(jet);
                if good && all(isfinite([T.lo;T.hi]))
                    S.diagnostic_point_valid(it)=true;
                    S.diagnostic_point_torque_lower_Nm(:,it)=T.lo;
                    S.diagnostic_point_torque_upper_Nm(:,it)=T.hi;
                    S.diagnostic_point_outside_limits_unrounded(it)= ...
                        any(T.lo>limits(:,2)-guard) || ...
                        any(T.hi<limits(:,1)+guard);
                end
            catch
                % A diagnostic exception cannot change the proof result.
            end
        end
        structural_torque=old_structural;
        inds=find(S.diagnostic_point_outside_limits_unrounded);
        if ~isempty(inds)
            S.diagnostic_point_first_outside_time_s=times(inds(1));
        end
        if any(S.diagnostic_point_valid)
            sel=S.diagnostic_point_valid;
            upper=S.diagnostic_point_torque_lower_Nm(:,sel)- ...
                (limits(:,2)-guard);
            lower=(limits(:,1)+guard)- ...
                S.diagnostic_point_torque_upper_Nm(:,sel);
            S.diagnostic_point_max_margin_violation_Nm= ...
                max([upper(:);lower(:)]);
        end
        S.diagnostic_point_elapsed_ms=1000*toc(sw);
    end
    function [good,reason]=node(polys,lo,hi,depth)
        R.checked_nodes=R.checked_nodes+1;
        good=false;reason='UNKNOWN_TORQUE_INTERVAL_BOUNDS';
        if R.checked_nodes>budget
            R.unresolved_interval_s=piece.t0_s+piece.duration_s*[lo hi];
            R.unresolved_depth=depth;
            R.unresolved_leaf_duration_s=piece.duration_s*(hi-lo);
            reason='UNKNOWN_TORQUE_NODE_BUDGET';return
        end
        M=I(NaN(3,1));nondegenerate=false;
        try
            % Cheap scalar-norm sufficient certificate first.  This bound
            % covers the SAME exact C9 polynomial and all t in this leaf;
            % failure to prove merely falls back to the detailed interval
            % model.  It is not a second candidate-generator/stage.
            if enable_fast_norm
                R.fast_norm_checked_nodes=R.fast_norm_checked_nodes+1;
                quick_valid=false;
                try
                    [Mquick,quick_valid]=quick_norm_box(polys);
                catch
                    % A fast-path implementation exception never bypasses
                    % the pre-existing detailed interval certificate.
                    R.fast_norm_exception_count=R.fast_norm_exception_count+1;
                end
                if quick_valid && all(Mquick.lo>=limits(:,1)+guard) && ...
                        all(Mquick.hi<=limits(:,2)-guard)
                    good=true;reason='PROVEN_FAST_NORM_LEAF';
                    R.accepted_leaves=R.accepted_leaves+1;
                    R.fast_norm_accepted_leaves=R.fast_norm_accepted_leaves+1;
                    if any(isnan(R.torque_bounds_Nm(:)))
                        R.torque_bounds_Nm=[Mquick.lo Mquick.hi];
                    else
                        R.torque_bounds_Nm(:,1)=min(R.torque_bounds_Nm(:,1),Mquick.lo);
                        R.torque_bounds_Nm(:,2)=max(R.torque_bounds_Nm(:,2),Mquick.hi);
                    end
                    return
                end
            end
            R.detailed_interval_checked_nodes=R.detailed_interval_checked_nodes+1;
            [M,nondegenerate]=torque_box(polys);
            if ~nondegenerate
                R.last_leaf_reason='SINGULAR_FORCE_Z_OR_HORIZONTAL_HEADING';
            else
                R.last_leaf_reason='NOMINAL_TORQUE_INTERVAL_NOT_WITHIN_LIMITS';
            end
            if nondegenerate && all(M.lo>=limits(:,1)+guard) && ...
                    all(M.hi<=limits(:,2)-guard)
                good=true;reason='PROVEN_LEAF';
                R.accepted_leaves=R.accepted_leaves+1;
                if any(isnan(R.torque_bounds_Nm(:)))
                    R.torque_bounds_Nm=[M.lo M.hi];
                else
                    R.torque_bounds_Nm(:,1)=min(R.torque_bounds_Nm(:,1),M.lo);
                    R.torque_bounds_Nm(:,2)=max(R.torque_bounds_Nm(:,2),M.hi);
                end
                return
            end
            % If the entire enclosure for any torque component is
            % outside the certified HLC nominal limit, even one instant
            % is unsafe for this candidate.  This leaf is nonempty, so
            % continuing subdivision cannot make this C9 candidate safe.
            % Keep UNKNOWN (not COMMIT); don't call it physical collision.
            if witness_exit && nondegenerate && ...
                    (any(M.lo>limits(:,2)-guard)|| ...
                     any(M.hi<limits(:,1)+guard))
                R.nominal_torque_limit_witness_nodes= ...
                    R.nominal_torque_limit_witness_nodes+1;
                R.last_leaf_reason='PROVEN_NOMINAL_TORQUE_LIMIT_EXCLUSION';
                R.unresolved_interval_s=piece.t0_s+ ...
                    piece.duration_s*[lo hi];
                save_unresolved(M,nondegenerate);
                if midpoint_diag, diagnose_midpoint(polys,lo,hi);end
                reason='UNKNOWN_TORQUE_PROVEN_NOMINAL_LIMIT_EXCLUSION';
                return
            end
        catch ME
            R.last_leaf_reason=['INTERVAL_ARITHMETIC_UNKNOWN_' ME.identifier];
            R.last_exception_identifier=ME.identifier;
            R.last_exception_message=ME.message;
            % Degenerate/nonfinite interval and unsupported singularity:
            % subdivide rather than treating a failed bound as safe.
        end
        if depth>=dep
            R.unresolved_interval_s=piece.t0_s+piece.duration_s*[lo hi];
            R.unresolved_depth=depth;
            R.unresolved_leaf_duration_s=piece.duration_s*(hi-lo);
            save_unresolved(M,nondegenerate);
            if midpoint_diag, diagnose_midpoint(polys,lo,hi);end
            reason='UNKNOWN_TORQUE_LEAF_UNRESOLVED';return
        end
        left=cell(size(polys));right=cell(size(polys));
        for j=1:numel(polys)
            [left{j},right{j}]=bisect(polys{j});
        end
        mid=(lo+hi)/2;
        [g1,r1]=node(left,lo,mid,depth+1);
        if ~g1,reason=r1;return;end
        [g2,r2]=node(right,mid,hi,depth+1);
        good=g2;reason=r2;
    end
    function save_unresolved(box,valid)
        % Diagnostics only: these bounds are NOT a proof of violation.
        % A straddling interval can be an inclusion-overestimation artefact.
        if valid && all(isfinite([box.lo;box.hi]))
            R.unresolved_torque_bounds_Nm=[box.lo box.hi];
            R.unresolved_torque_interval_valid=true;
            R.unresolved_torque_interval_headroom_Nm=min([ ...
                box.lo-limits(:,1)-guard; ...
                limits(:,2)-guard-box.hi]);
        else
            R.unresolved_torque_bounds_Nm=NaN(3,2);
            R.unresolved_torque_interval_valid=false;
            R.unresolved_torque_interval_headroom_Nm=NaN;
        end
    end
    function diagnose_midpoint(polys,lo,hi)
        % DIAGNOSTIC ONLY. The midpoint is a single time instant and must
        % never be interpreted as an all-time certificate. The floating
        % interval arithmetic is not outward-rounded, either.
        R.unresolved_midpoint_s=piece.t0_s+piece.duration_s*(lo+hi)/2;
        try
            point=cell(size(polys));
            for z=1:numel(polys)
                [left,~]=bisect(polys{z});
                point{z}=left(:,end);  % Bernstein/de Casteljau midpoint
            end
            [T,valid]=torque_box(point);
            if valid && all(isfinite([T.lo;T.hi]))
                R.unresolved_midpoint_torque_bounds_Nm=[T.lo T.hi];
                R.unresolved_midpoint_valid=true;
                R.unresolved_midpoint_outside_limits_unrounded= ...
                    any(T.lo>limits(:,2)-guard) || ...
                    any(T.hi<limits(:,1)+guard);
                R.unresolved_midpoint_inside_limits_unrounded= ...
                    all(T.lo>=limits(:,1)+guard) && ...
                    all(T.hi<=limits(:,2)-guard);
            end
        catch
            % Only a diagnostic failed. The original UNKNOWN is preserved.
        end
    end
    function c=choose(n,k)
        if fast_binomial
            c=B(n+1,k+1);
        else
            c=nchoosek(n,k);
        end
    end
    function [M,valid]=quick_norm_box(P)
        % Real-arithmetic sufficient bounds; no sampling and no heuristic
        % clearance.  The existing floating-point directed-rounding caveat
        % applies to this fast proof just as to the detailed proof.
        M=I(NaN(3,1)); valid=false;
        % w = g - pL'', q = w/||w||, F = -(mu+m)w-mu L q''.
        wbox=sub(I(g),Iof(P{3}));
        aa=max(abs(wbox.lo),abs(wbox.hi));
        bb=max([wbox.lo,-wbox.hi,zeros(3,1)],[],2);
        wmin=norm(bb,2);
        if ~(isfinite(wmin)&&wmin>0), return; end
        W=zeros(1,5); W(1)=norm(aa,2);
        for kk=1:4
            % Bernstein convex-hull upper bounds of ||w^(kk)||_2.
            % w^(kk) = -pL^(kk+2) for kk >= 1.
            W(kk+1)=max(sqrt(sum(P{kk+3}.^2,1)));
        end
        % n=||w||; n^2=w.w; n*q=w. Differentiate both identities
        % recursively. All arrays below enclose absolute derivative norms.
        N=zeros(1,5);Q=zeros(1,5);Q(1)=1;
        for k=1:4
            acc=0;
            for j=0:k
                acc=acc+choose(k,j)*W(j+1)*W(k-j+1);
            end
            for j=1:k-1
                acc=acc+choose(k,j)*N(j+1)*N(k-j+1);
            end
            N(k+1)=acc/(2*wmin);
            acc=W(k+1);
            for j=1:k
                acc=acc+choose(k,j)*N(j+1)*Q(k-j+1);
            end
            Q(k+1)=acc/wmin;
        end
        mplus=mu+ml(2);
        F0=mplus*W(1)+mu*L*Q(3);
        F1=mplus*W(2)+mu*L*Q(4);
        F2=mplus*W(3)+mu*L*Q(5);
        % Exact interval product extremum: +(mu+m)*w_z.  The q''
        % component is enclosed in [-Q2,Q2]. No assumed gravity sign.
        wz=[wbox.lo(3),wbox.hi(3)];
        corners=[(mu+ml(1))*wz, (mu+ml(2))*wz];
        fzlo=-max(corners)-mu*L*Q(3);
        if ~(isfinite(fzlo)&&fzlo>minFz && F0>0),return;end
        % Horizontal UAV velocity hraw = (pL' - L q')_xy. The
        % distance from the origin to its complete interval box is a
        % continuous-time lower bound on ||hraw||.
        vb=Iof(P{2});
        vlo=vb.lo(1:2)-L*Q(2);
        vhi=vb.hi(1:2)+L*Q(2);
        vmin=norm(max([vlo,-vhi,zeros(2,1)],[],2),2);
        if ~(isfinite(vmin)&&vmin>minV && vmin>0),return;end
        V1=max(sqrt(sum(P{3}(1:2,:).^2,1)))+L*Q(3);
        V2=max(sqrt(sum(P{4}(1:2,:).^2,1)))+L*Q(4);
        % For u=x/||x||: ||u'||<=||x'||/x_min,
        % ||u''||<=||x''||/x_min+3||x'||^2/x_min^2.
        H1=V1/vmin;
        H2=V2/vmin+3*H1^2;
        Z1=F1/fzlo;
        Z2=F2/fzlo+3*Z1^2;
        % For a = horizontal-normal(hraw) x b3, ||a|| >= b3_z.
        % b3_z = F_z/||F|| >= fzlo/F0 > 0.
        amin=fzlo/F0;
        A1=H1+Z1;
        A2=H2+2*H1*Z1+Z2;
        B11=A1/amin;
        B12=A2/amin+3*B11^2;
        B21=Z1+B11;
        B22=Z2+2*Z1*B11+B12;
        % The orthonormal columns [b1,b2,b3] imply
        % ||omega|| <= ||R'||_F, ||dot(omega)|| <= ||R''||_F.
        omega_bound=hypot(hypot(B11,B21),Z1);
        alpha_bound=hypot(hypot(B12,B22),Z2);
        % Component-wise Euler rigid-body equation, diag(J):
        % Old |M_i|<=J_i*||dot(omega)||+max(J)*||omega||^2.
        % New factored: use each diagonal inertia difference and /2.
        if factor_gyro
            % |wi*wj| <= (wi^2+wj^2)/2 <= ||omega||^2/2.
            % This is a certified stronger enclosure than max(J)*||w||^2
            % for strictly positive diagonal inertia components.
            deltaJ=abs([J(3)-J(2);J(1)-J(3);J(2)-J(1)]);
            tau=J*alpha_bound+0.5*deltaJ*omega_bound^2;
        else
            tau=J*alpha_bound+max(J)*omega_bound^2;
        end
        if any(~isfinite([W,N,Q,F0,F1,F2,fzlo,vmin,tau(:).'])),return;end
        M=I(-tau,tau);valid=true;
    end

    function [M,valid]=torque_box(P)
        valid=false;
        w=cell(1,5);
        w{1}=sub(I(g),Iof(P{3}));
        for n=1:4
            w{n+1}=neg(Iof(P{n+3}));
        end
        q=unit_jet(w,4,B,fast_binomial,sym_unit);
        f=cell(1,3);v=cell(1,3);
        % [m_min,m_max] describes ONE uncertain SCALAR mass.  I(ml)
        % incorrectly creates a 1x2 point-vector and makes the
        % subsequent scalar-times-3D-force interval multiplication fail.
        mass=I(ml(1),ml(2));
        for n=0:2
            f{n+1}=sub(neg(mul(add(I(mu),mass),w{n+1})), ...
                mul(I(mu*L),q{n+3}));
            v{n+1}=sub(Iof(P{n+2}),mul(I(L),q{n+2}));
        end
        if ~(f{1}.lo(3)>minFz)
            M=I(zeros(3,1));return
        end
        h=cell(1,3);
        for n=1:3
            h{n}=I([v{n}.lo(1:2);0],[v{n}.hi(1:2);0]);
        end
        if planar_lower(h{1})<=minV
            M=I(zeros(3,1));return
        end
        hraw=h; % Preserve non-normalized horizontal jets for structural proof.
        if cancel_attitude
            % Let l = z x v_xy. Since ||v_xy|| and ||F|| are positive,
            % unit((z x unit(v_xy)) x unit(F)) == unit(l x F).
            % Differentiate the unnormalized jets through order 2; no
            % finite time sample or approximation enters this identity.
            lateral=cell(1,3);
            for n=1:3
                lateral{n}=I([-h{n}.hi(2);h{n}.lo(1);0], ...
                               [-h{n}.lo(2);h{n}.hi(1);0]);
            end
            raw=jet_cross(lateral,f,2,B,fast_binomial);
            b1=unit_jet(raw,2,B,fast_binomial,sym_unit);
            b3=unit_jet(f,2,B,fast_binomial,sym_unit);
        else
            % Original three-normalization formulation retained for A/B.
            h=unit_jet(h,2,B,fast_binomial,sym_unit);
            lateral=cell(1,3);
            for n=1:3
                lateral{n}=I([-h{n}.hi(2);h{n}.lo(1);0], ...
                               [-h{n}.lo(2);h{n}.hi(1);0]);
            end
            b3=unit_jet(f,2,B,fast_binomial,sym_unit);
            raw=jet_cross(lateral,b3,2,B,fast_binomial);
            b1=unit_jet(raw,2,B,fast_binomial,sym_unit);
        end
        b2=jet_cross(b3,b1,2,B,fast_binomial);
        % All-time heading-exact Euler yaw requires positive F_z.
        % If the attitude construction has a singular denominator, the
        % interval normalizations above throw and this leaf is UNKNOWN.
        omega=I(zeros(3,1));alpha=I(zeros(3,1));
        omega=join_components( ...
            dotI(b3{1},b2{2}), ...
            dotI(b1{1},b3{2}), ...
            dotI(b2{1},b1{2}));
        alpha=join_components( ...
            add(dotI(b3{2},b2{2}),dotI(b3{1},b2{3})), ...
            add(dotI(b1{2},b3{2}),dotI(b1{1},b3{3})), ...
            add(dotI(b2{2},b1{2}),dotI(b2{1},b1{3})));
        if factor_gyro
            % Exact identity in real arithmetic for positive diagonal J.
            % Enclose each common angular-velocity product only once.
            % When two inertias agree the relevant gyro component vanishes
            % EXACTLY, instead of adding two unnecessarily wide intervals.
            gy=diagonal_gyro_interval(omega,J);
        else
            gy=crossI(omega,mul(I(J),omega));
        end
        M=add(mul(I(J),alpha),gy);
        valid=all(isfinite([M.lo;M.hi]));
        if ~valid,return;end
        if structural_torque && ...
                (any(M.lo<limits(:,1)+guard) || ...
                 any(M.hi>limits(:,2)-guard))
            % Only invoke the second exact interval extension when the
            % regular detailed proof could not certify this time leaf.
            R.structural_torque_attempted_nodes= ...
                R.structural_torque_attempted_nodes+1;
            try
                [M2,v2]=structural_torque_box(f,hraw);
                if ~v2
                    R.structural_torque_unusable_nodes= ...
                        R.structural_torque_unusable_nodes+1;
                end
                if v2
                    candidate=I(max(M.lo,M2.lo),min(M.hi,M2.hi));
                    if any(candidate.lo>candidate.hi) || ...
                            any(~isfinite([candidate.lo;candidate.hi]))
                        valid=false;return
                    end
                    shrink=[candidate.lo-M.lo, M.hi-candidate.hi];
                    if any(shrink(:)>0)
                        R.structural_torque_tightened_nodes= ...
                            R.structural_torque_tightened_nodes+1;
                        R.structural_torque_intersection_max_shrink_Nm= ...
                            max(R.structural_torque_intersection_max_shrink_Nm,shrink);
                    end
                    if all(candidate.lo>=limits(:,1)+guard) && ...
                            all(candidate.hi<=limits(:,2)-guard)
                        R.structural_torque_certified_leaves= ...
                            R.structural_torque_certified_leaves+1;
                    end
                    M=candidate;
                end
            catch ME
                R.structural_torque_exception_nodes= ...
                    R.structural_torque_exception_nodes+1;
                R.structural_torque_last_exception=ME.identifier;
                % A failed OPTIONAL strengthening must not override the
                % original certificate or turn an UNKNOWN into PROVEN.
                % Keep the already valid original interval enclosure.
            end
        end
    end

    function [T,valid]=structural_torque_box(f,hraw)
        % Exact REAL-ARITHMETIC identities for F and a=(ez x v_xy) x F:
        % n=||F||, m=||a||, C=F x a.
        % wx = -(C.F')/(n^2 m)
        % wy =  (a.F')/(n m)
        % wz =  (C.a')/(n m^2)
        % Differentiating these yields alpha without ever forming the
        % nested normalizations b1,b2,b3 or their second derivatives.
        % Cross-product self-orthogonality removes terms that are exactly
        % zero, avoiding loss of dependencies in interval extensions.
        T=I(NaN(3,1));valid=false;
        lateral=cell(1,3);
        for ii=1:3
            lateral{ii}=I([-hraw{ii}.hi(2);hraw{ii}.lo(1);0], ...
                          [-hraw{ii}.lo(2);hraw{ii}.hi(1);0]);
        end
        a=jet_cross(lateral,f,2,B,fast_binomial);
        F0=f{1};F1=f{2};F2=f{3};
        A0=a{1};A1=a{2};A2=a{3};
        FF=normsqI(F0);AA=normsqI(A0);
        if ~(all(FF.lo>0)&&all(AA.lo>0)),return;end
        n=sqrtI(FF);m=sqrtI(AA);
        kF=div(dotI(F0,F1),FF);
        kA=div(dotI(A0,A1),AA);
        C=crossI(F0,A0);
        N1=dotI(C,F1);
        N2=dotI(A0,F1);
        N3=dotI(C,A1);
        % The vanished terms are (F' x a).F' and (F x a').a'.
        N1p=add(dotI(crossI(F0,A1),F1),dotI(C,F2));
        N2p=add(dotI(A1,F1),dotI(A0,F2));
        N3p=add(dotI(crossI(F1,A0),A1),dotI(C,A2));
        d1=mul(FF,m);d2=mul(n,m);d3=mul(n,AA);
        wx=neg(div(N1,d1));
        wy=div(N2,d2);
        wz=div(N3,d3);
        ax=neg(div(sub(N1p,mul(N1,add(mul(I(2),kF),kA))),d1));
        ay=div(sub(N2p,mul(N2,add(kF,kA))),d2);
        az=div(sub(N3p,mul(N3,add(kF,mul(I(2),kA)))),d3);
        om=join_components(wx,wy,wz);
        al=join_components(ax,ay,az);
        if factor_gyro
            gy=diagonal_gyro_interval(om,J);
        else
            gy=crossI(om,mul(I(J),om));
        end
        T=add(mul(I(J),al),gy);
        valid=all(isfinite([T.lo;T.hi]));
    end
end

function Z=I(a,b)
if nargin<2,b=a;end
Z=struct('lo',double(a),'hi',double(b));
end
function Z=Iof(A)
Z=I(min(A,[],2),max(A,[],2));
end
function Z=add(A,B)
Z=I(A.lo+B.lo,A.hi+B.hi);
end
function Z=neg(A)
Z=I(-A.hi,-A.lo);
end
function Z=sub(A,B)
Z=add(A,neg(B));
end
function Z=mul(A,B)
[a,b]=compatible(A,B);
x=[a.lo.*b.lo,a.lo.*b.hi,a.hi.*b.lo,a.hi.*b.hi];
Z=I(min(x,[],2),max(x,[],2));
end
function [A,B]=compatible(A,B)
if isscalar(A.lo)&&~isscalar(B.lo)
    A=I(repmat(A.lo,size(B.lo)),repmat(A.hi,size(B.hi)));
elseif isscalar(B.lo)&&~isscalar(A.lo)
    B=I(repmat(B.lo,size(A.lo)),repmat(B.hi,size(A.hi)));
end
assert(isequal(size(A.lo),size(B.lo)),'C9_TORQUE:IntervalShape','Interval size mismatch');
end
function Z=div(A,B)
assert(all(B.lo(:)>0)||all(B.hi(:)<0), ...
    'C9_TORQUE:Singular','Interval division through zero');
Z=mul(A,I(1./B.hi,1./B.lo));
end
function Z=dotI(A,B)
% A single vectorized four-corner enclosure for every dot-product term.
% Equivalent to the original scalar loop in exact arithmetic. No assumed
% correlation: each product retains all four interval endpoint products.
[A,B]=compatible(A,B);
al=A.lo(:);ah=A.hi(:);bl=B.lo(:);bh=B.hi(:);
products=[al.*bl,al.*bh,ah.*bl,ah.*bh];
Z=I(sum(min(products,[],2)),sum(max(products,[],2)));
end
function Z=crossI(A,B)
% The three skew-product components share one vectorized operation.
% x(1)=a(2)b(3)-a(3)b(2), cyclic for x(2), x(3).
assert(numel(A.lo)==3&&numel(B.lo)==3, ...
    'C9_TORQUE:CrossDimension','Cross product requires 3-vectors');
j=[2;3;1];k=[3;1;2];
p=mul(I(A.lo(j),A.hi(j)),I(B.lo(k),B.hi(k)));
q=mul(I(A.lo(k),A.hi(k)),I(B.lo(j),B.hi(j)));
Z=I(p.lo-q.hi,p.hi-q.lo);
end
function gy=diagonal_gyro_interval(omega,J)
% Interval extension of the exact Euler rigid-body gyroscopic products.
% Each scalar multiply encloses all values of its two angular intervals.
% It does not assume signs/correlations, and introduces no approximation.
wy=I(omega.lo(2),omega.hi(2));
wz=I(omega.lo(3),omega.hi(3));
wx=I(omega.lo(1),omega.hi(1));
g1=mul(I(J(3)-J(2)),mul(wy,wz));
g2=mul(I(J(1)-J(3)),mul(wz,wx));
g3=mul(I(J(2)-J(1)),mul(wx,wy));
gy=join_components(g1,g2,g3);
end
function Z=squareI(X)
a=X.lo;b=X.hi;
Z=I(min(a.^2,b.^2),max(a.^2,b.^2));
Z.lo(a<=0&b>=0)=0;
end
function Z=normsqI(X)
Z=I(0);
for i=1:numel(X.lo)
    Z=add(Z,squareI(I(X.lo(i),X.hi(i))));
end
end
function Z=sqrtI(A)
assert(all(A.lo(:)>0),'C9_TORQUE:ZeroNorm','No lower norm bound');
Z=I(sqrt(A.lo),sqrt(A.hi));
end
function U=unit_jet(X,nmax,B,fast_binomial,fast_symmetric)
assert(numel(X)>nmax,'C9_TORQUE:JetOrder','Missing derivative jet');
s=cell(1,nmax+1);U=cell(1,nmax+1);
s{1}=sqrtI(normsqI(X{1}));
U{1}=div(X{1},s{1});
for n=1:nmax
    rhs=I(0);
    if fast_symmetric
        % d^n(X.X) = 2 sum_{k<n/2} C(n,k) X_k.X_(n-k)
        % + (n even) C(n,n/2) X_(n/2).X_(n/2).
        % The repeated cross terms are algebraically IDENTICAL in real
        % arithmetic; the diagonal term is a sum of nonnegative squares.
        for k=0:floor(n/2)
            if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
            if 2*k==n
                product=normsqI(X{k+1});
                rhs=add(rhs,mul(I(c),product));
            else
                product=dotI(X{k+1},X{n-k+1});
                rhs=add(rhs,mul(I(2*c),product));
            end
        end
        for k=1:floor(n/2)
            if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
            if 2*k==n
                product=squareI(s{k+1});
                rhs=sub(rhs,mul(I(c),product));
            else
                product=mul(s{k+1},s{n-k+1});
                rhs=sub(rhs,mul(I(2*c),product));
            end
        end
    else
        rhs=I(0);
        for k=0:n
            if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
            rhs=add(rhs,mul(I(c),dotI(X{k+1},X{n-k+1})));
        end
        for k=1:n-1
            if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
            rhs=sub(rhs,mul(I(c),mul(s{k+1},s{n-k+1})));
        end
    end
    s{n+1}=div(rhs,mul(I(2),s{1}));
    num=X{n+1};
    for k=1:n
        if fast_binomial,c=B(n+1,k+1);else,c=nchoosek(n,k);end
        num=sub(num,mul(I(c),mul(s{k+1},U{n-k+1})));
    end
    U{n+1}=div(num,s{1});
end
end
function V=jet_cross(A,B,nmax,C,fast_binomial)
V=cell(1,nmax+1);
for n=0:nmax
    v=I(zeros(3,1));
    for k=0:n
        if fast_binomial,c=C(n+1,k+1);else,c=nchoosek(n,k);end
        v=add(v,mul(I(c),crossI(A{k+1},B{n-k+1})));
    end
    V{n+1}=v;
end
end
function V=join_components(A,B,C)
V=I([A.lo;B.lo;C.lo],[A.hi;B.hi;C.hi]);
end
function n=planar_lower(V)
q=normsqI(I([V.lo(1:2);0],[V.hi(1:2);0]));
n=sqrt(q.lo);
end
function [L,R]=bisect(P)
% Do not repeat de Casteljau MATLAB loops for every torque proof leaf.
[L,R]=TRAJECTORY_C9_BERNSTEIN_MIDPOINT_SPLIT(P);
end
