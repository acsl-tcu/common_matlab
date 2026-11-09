function R=TRAJECTORY_C9_13D_TIME_JOINT_SEARCH(req,R)
% Exclusive SINGLE-polynomial 13-variable C9 search, research-only.
% x=[four free 3D Bernstein controls; physical duration]. No discrete
% multistage path, no CBF/AP, and no fixed avoidance direction.
% A nonlinear update is NEVER accepted from local linearization alone:
% the independent full-time geometry/tension/thrust oracle runs on each
% candidate; nominal rigid-body torque proof runs only after that passes.
% Real HLC tracking/actuators and directed-rounding bounds remain pending.
% Numeric evaluations do not authorize HLC COMMIT.
start=tic;
R.safe_to_commit=false;R.candidate_available=false;
R.duration_13d_enabled=true;
R.duration_13d_qp_count=0;
R.duration_13d_qp_ms=0;
R.duration_13d_plane_ms=0;
R.duration_13d_proof_ms=0;
R.duration_13d_torque_ms=0;
R.duration_13d_metric_status='NOT_RUN';
R.duration_13d_candidate_history_s=double(req.duration_s);
R.duration_13d_nominal_torque_status='NOT_RUN';
R.duration_13d_conditional_nominal_torque_proven=false;
R.scope='ONE_C9_13D_SIMULTANEOUS_FULLBODY_SEARCH__HLC_NOT_CERTIFIED';
shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(req.observed);
radii=req.radii;
rad=[double(radii.uav_m),double(radii.cable_m),double(radii.payload_m)];
if req.geometry_options.robust_tube_enabled
    u=req.geometry_options.uncertainty_bounds_m;
    rad=rad+[u.uav_m,u.cable_m,u.payload_m];
end
assert(all(isfinite(rad))&&all(rad>=0), ...
    'C9_13D:Radius','Invalid physical body/tube radius');
[ground_active,phase]=TRAJECTORY_GROUND_SCOPE(req.geometry_options);
if ~strcmp(phase,'f')
    R.status='UNKNOWN_13D_NOT_FLIGHT';R.elapsed_ms=1000*toc(start);return
end
Tmax=req.flight_interval_s(2)-req.t0_s;
if ~(isfinite(Tmax)&&Tmax>0&&req.duration_s>0&&req.duration_s<=Tmax)
    R.status='UNKNOWN_13D_FLIGHT_TIME_DOMAIN';
    R.elapsed_ms=1000*toc(start);return
end
current=TRAJECTORY_C9_FLAT_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets,req.end_jets,req.nominal_free);
C0=current;
seen=zeros(13,0);
% Chebyshev-Lobatto probes are search locations ONLY, not certificate
% sample points or assumed collision-free intervals.
N=req.search_options.probe_count;
phases=(1-cos(pi*(0:N-1)/(N-1)))/2;
for iter=0:req.search_options.max_outer_iterations
    R.search_iterations=iter;
    oracle=TRAJECTORY_C9_JOINT_FAST_CERTIFICATE( ...
        current,req.observed,req.radii,req.authority, ...
        req.geometry_options,req.force_options);
    R.independent_proof_calls=R.independent_proof_calls+1;
    R.duration_13d_proof_ms=R.duration_13d_proof_ms+oracle.elapsed_ms;
    R.final_certificate=oracle;
    if iter==0,R.initial_certificate=oracle;end
    if oracle.certified
        % No nominal 3-axis torque assertion from geometry alone.
        if ~isfield(req,'torque_options') || ~isstruct(req.torque_options)
            R.status='UNKNOWN_13D_GEOMETRY_PASS_TORQUE_AUTHORITY_MISSING';
            R.failure_reason='Nominal 3-axis torque proof needs current HLC limits.';
            break
        end
        ttorque=tic;
        C=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
            current,req.authority,req.torque_options);
        R.duration_13d_torque_ms=R.duration_13d_torque_ms+1000*toc(ttorque);
        R.duration_13d_nominal_torque_status=C.status;
        R.duration_13d_conditional_nominal_torque_proven= ...
            C.conditional_nominal_all_time_torque_proven;
        if C.conditional_nominal_all_time_torque_proven
            % Only nominal real-arithmetic candidate available.
            R.status='CONDITIONAL_13D_NOMINAL_GEOMETRY_TORQUE__HLC_PENDING';
            R.candidate_available=true;
            R.candidate_piece=current;
            R.duration_s=current.duration_s;
            R.failure_reason='Actual HLC tube/4-input and rounding not certified.';
            break
        end
        % Feed a FAILED all-time torque certificate into the SAME 13D
        % search. No point test can turn UNKNOWN into a PASS.
        leaf=double(C.unresolved_interval_s(:));
        if numel(leaf)==2 && all(isfinite(leaf)) && ...
                leaf(2)>=leaf(1) && leaf(1)>=current.t0_s && ...
                leaf(2)<=current.t0_s+current.duration_s
            ts=[leaf(1),(leaf(1)+leaf(2))/2,leaf(2)];
            us=(ts-current.t0_s)/current.duration_s;
            phases=unique([phases(:);us(:)]).';
        end
        point=TRAJECTORY_C9_TORQUE_POINT_EVALUATE( ...
            current,req.authority,req.torque_options, ...
            current.t0_s+current.duration_s*phases);
        if ~all(point.valid) || ~any(point.outside_limits)
            R.status='UNKNOWN_13D_TORQUE_LEAF_WITHOUT_POINT_WITNESS';
            R.failure_reason=C.status;
            break
        end
        R.duration_13d_torque_witness_count=sum(point.outside_limits);
    end
    if ~oracle.certified && isfield(oracle,'unresolved_time_interval_s')
        % All shapes and all bodies are reconsidered at these search
        % probes. They are NEVER used as continuous-time certificates.
        leaf=double(oracle.unresolved_time_interval_s(:));
        if numel(leaf)==2 && all(isfinite(leaf)) && ...
                leaf(2)>leaf(1) && leaf(1)>=current.t0_s && ...
                leaf(2)<=current.t0_s+current.duration_s
            w=leaf(1)+(leaf(2)-leaf(1))*[0.25 0.50 0.75];
            us=(w-current.t0_s)/current.duration_s;
            phases=unique([phases(:);us(:)]).';
        end
    end
    if iter==req.search_options.max_outer_iterations
        R.status='UNKNOWN_13D_ITERATION_LIMIT';
        R.failure_reason=oracle.status;break
    end
    tplane=tic;
    local=req;
    local.duration_s=current.duration_s;
    local.nominal_free=current.control_points_m(:,11:14);
    Q=TRAJECTORY_C9_13D_NATURAL_QP(local,shapes,phases,rad,ground_active);
    R.duration_13d_plane_ms=R.duration_13d_plane_ms+1000*toc(tplane);
    R.duration_13d_metric_status=Q.status;
    if ~Q.solved
        R.status='UNKNOWN_13D_QP_MODEL_UNAVAILABLE';
        R.failure_reason=Q.status;break
    end
    tqp=tic;
    [z,S]=TRAJECTORY_C9_JOINT_SCREENED_PROJECTION( ...
        Q.A,Q.b,req.search_options.max_projection_sweeps, ...
        req.search_options.projection_tolerance_m);
    R.duration_13d_qp_ms=R.duration_13d_qp_ms+1000*toc(tqp);
    R.duration_13d_qp_count=R.duration_13d_qp_count+1;
    R.joint_qp_status=S.status;
    if ~S.solved
        R.status='UNKNOWN_13D_QP_NOT_SOLVED';
        R.failure_reason=S.status;break
    end
    dx=Q.chol_upper\z;
    Fnew=local.nominal_free+reshape(dx(1:12),3,4);
    Tnew=local.duration_s+dx(13);
    if ~(all(isfinite(Fnew(:)))&&isfinite(Tnew)&& ...
            Tnew>0&&Tnew<=Tmax)
        R.status='UNKNOWN_13D_NONPOSITIVE_OR_NONFLIGHT_DURATION';
        R.failure_reason='The QP tangent touched a time-domain boundary.';break
    end
    if isequal(Fnew,local.nominal_free)&&Tnew==local.duration_s
        R.status='UNKNOWN_13D_NO_LOCAL_QP_PROGRESS';
        R.failure_reason=oracle.status;break
    end
    x=[Fnew(:);Tnew];
    if any(all(seen==x,1))
        R.status='UNKNOWN_13D_QP_CYCLE';
        R.failure_reason='A duration/free candidate was already searched.';break
    end
    seen(:,end+1)=x; %#ok<AGROW>
    current=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
        req.t0_s,Tnew,req.start_jets,req.end_jets,Fnew);
    R.duration_13d_candidate_history_s(end+1)=Tnew;
end
R.duration_13d_elapsed_ms=1000*toc(start);
R.independent_proof_time_total_ms=R.duration_13d_proof_ms;
R.plane_construction_time_total_ms=R.duration_13d_plane_ms;
R.joint_qp_time_total_ms=R.duration_13d_qp_ms;
R.safe_to_commit=false;
% Even when the single C9 nominal candidate passes, real HLC tracking,
% time-varying mass and numerical directed-rounding are NOT certified.
end
