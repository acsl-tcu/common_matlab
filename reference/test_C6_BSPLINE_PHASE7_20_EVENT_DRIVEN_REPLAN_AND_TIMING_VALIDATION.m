function result = test_C6_BSPLINE_PHASE7_20_EVENT_DRIVEN_REPLAN_AND_TIMING_VALIDATION(scenario)
% =========================================================================
% Phase 7-20: EVENT-DRIVEN REPLAN + ONLINE TIMING VALIDATION
%
% Goal
%   1) Replan immediately at the first predicted future collision.
%   2) Never force a fixed 5 s nominal rejoin.
%   3) Estimate when the nominal path becomes safe again (t_clear).
%   4) Search delayed rejoin times from short to long.
%   5) Search small N first to avoid unnecessary QP size.
%   6) Accept ONLY a candidate that passes the independent global verifier.
%   7) Explicitly verify C6 at BOTH splice points (r=0,...,6).
%
% IMPORTANT
%   - This is an offline validation of the architecture, not the final 1-2 ms
%     implementation and not a WCET claim.
%   - The selected avoidance trajectory is a degree-7 B-spline with exact C6
%     matching to nominal at start and delayed end. Thus the complete trajectory
%       nominal -> avoidance -> nominal
%     is C6 at both switches.
%   - Candidate generation and independent certification remain separated.
% =========================================================================
clc;
if nargin<1 || isempty(scenario), scenario=setup_benchmark(); end
[cfg0,limits,obs,nom_fun,t_env0,t_env1]=unpack_scenario(scenario);
opt=setup_options(); alpha=1/sqrt(3);
validate_phase712e_inputs(cfg0,limits,obs,opt);

Tlook=5.0; pred_dt=0.025;
fprintf('================================================================================\n');
fprintf(' Phase 7-20: EVENT-DRIVEN REPLAN + ONLINE TIMING VALIDATION\n');
fprintf('================================================================================\n');

% A. Trigger exactly once for this offline benchmark.
trig=reconstruct_first_trigger(nom_fun,obs,cfg0,opt,t_env0,t_env1,Tlook,pred_dt);
if ~trig.found
    result=struct('execute',false,'reason','NO_TRIGGER_FOUND','trigger',trig); return;
end
fprintf('[A: TRIGGER]\n');
fprintf('t_now=%.6f  t_risk=%.6f  TTC=%.6f s\n',trig.t_now,trig.t_risk,trig.TTC);

% B. Determine nominal unsafe window.  Rejoin is not allowed before t_clear.
risk=audit_nominal_risk_window(nom_fun,obs,cfg0,opt,trig.t_now,t_env1,pred_dt);
fprintf('\n[B: NOMINAL RISK WINDOW]\n');
fprintf('first bad = %.6f s\n',risk.t_first_bad);
fprintf('worst     = %.6f s, g=%+.6f m\n',risk.t_worst,risk.g_worst);
fprintf('clear     = %.6f s\n',risk.t_clear);
if isnan(risk.t_clear)
    result=struct('execute',false,'reason','NO_NOMINAL_CLEAR_TIME','trigger',trig,'risk_window',risk); return;
end

% C. Adaptive rejoin search.
% Start at nominal-clear time, then give the degree-7 spline progressively more
% time to merge.  Fine 0.25 s spacing near the boundary; no arbitrary 5 s rejoin.
extra_coarse=[0 0.5 1.0 1.5 2.0 2.5 3.0];
Nlist=[18 22 26 30 34 38];
Tclear=risk.t_clear-trig.t_now;
Tlist=Tclear+extra_coarse;
Tlist=Tlist(Tlist>0 & trig.t_now+Tlist<=t_env1+1e-12);

fprintf('\n[C: COARSE ADAPTIVE SEARCH]\n');
fprintf('N | T_rejoin | extra_after_clear | G D GD cert | rho* | min_g | v/a/j | ms\n');
records=struct([]); krec=0; first_cert=[];
for iN=1:numel(Nlist)
    for iT=1:numel(Tlist)
        tt=tic;
        rr=run_plan_case(cfg0,Nlist(iN),limits,obs,nom_fun,trig.t_now,Tlist(iT),opt,alpha);
        ms=toc(tt)*1000;
        krec=krec+1;
        rec=struct('N',Nlist(iN),'T',Tlist(iT),'extra',Tlist(iT)-Tclear,'plan',rr,'ms',ms);
        if krec==1, records=rec; else, records(krec)=rec; end %#ok<AGROW>
        fprintf('%2d | %8.3f | %17.3f | %d %d %2d %4d | %.2e | %+6.3f | %.2f/%.2f/%.2f | %.2f\n', ...
            rec.N,rec.T,rec.extra,rr.G,rr.D,rr.GD,rr.cert,rr.rho,rr.min_g,rr.vUB,rr.aUB,rr.jUB,ms);
        if rr.cert && isempty(first_cert), first_cert=rec; end
    end
end

% D. Refine time around the earliest certified boundary for the smallest N that
% ever certified.  This chooses a natural short-enough avoidance time rather than
% hard-coding the Phase-7-12F answer.
fine_records=struct([]); selected=[];
certNs=[];
for iN=1:numel(Nlist)
    ix=arrayfun(@(x)x.N==Nlist(iN) && x.plan.cert,records);
    if any(ix), certNs(end+1)=Nlist(iN); end %#ok<AGROW>
end
if ~isempty(certNs)
    Nsel=min(certNs);
    ix=find(arrayfun(@(x)x.N==Nsel && x.plan.cert,records));
    [~,jj]=min([records(ix).T]); coarse=records(ix(jj));
    Tlo=max(Tclear,coarse.T-0.5); Thi=coarse.T;
    Tf=unique([Tlo:0.10:Thi Thi]);
    fprintf('\n[D: FINE REJOIN-TIME SEARCH, N=%d]\n',Nsel);
    fprintf('T | extra_after_clear | cert | rho* | min_g | C6start/C6end | ms\n');
    q=0;
    for it=1:numel(Tf)
        tt=tic;
        [rr,splice]=run_plan_case_with_splice_audit(cfg0,Nsel,limits,obs,nom_fun,trig.t_now,Tf(it),opt,alpha);
        ms=toc(tt)*1000; q=q+1;
        fr=struct('N',Nsel,'T',Tf(it),'extra',Tf(it)-Tclear,'plan',rr,'splice',splice,'ms',ms);
        if q==1, fine_records=fr; else, fine_records(q)=fr; end %#ok<AGROW>
        fprintf('%.3f | %17.3f | %4d | %.2e | %+6.3f | %.2e/%.2e | %.2f\n', ...
            fr.T,fr.extra,rr.cert,rr.rho,rr.min_g,splice.start_max,splice.end_max,ms);
    end
    ix=find(arrayfun(@(x)x.plan.cert && x.splice.pass,fine_records));
    if ~isempty(ix)
        [~,jj]=min([fine_records(ix).T]); selected=fine_records(ix(jj));
    end
end

% E. Final execution gate.
fprintf('\n[E: FINAL VALIDATION GATE]\n');
if isempty(selected)
    fprintf('NO certified C6 natural-rejoin candidate found in tested set.\n');
    execute=false; reason='NO_CERTIFIED_C6_REJOIN_IN_TESTED_SET';
else
    execute=true; reason='CERTIFIED_C6_NATURAL_REJOIN_FOUND';
    fprintf('SELECTED: N=%d, T=%.3f s, t_end=%.3f s, extra_after_clear=%.3f s\n', ...
        selected.N,selected.T,trig.t_now+selected.T,selected.extra);
    fprintf('certificate: min_g=%+.6f m, v/a/j=%.3f/%.3f/%.3f\n', ...
        selected.plan.min_g,selected.plan.vUB,selected.plan.aUB,selected.plan.jUB);
    fprintf('C6 splice max errors: start=%.3e, end=%.3e (tol %.1e)\n', ...
        selected.splice.start_max,selected.splice.end_max,opt.c6_tol);
end
fprintf('================================================================================\n');

% F. Integrated validation required by the final specification.
% Plotting is deliberately OUTSIDE all timing measurements.
integrated=struct();
if execute
    fprintf('\n[F: INTEGRATED REQUIREMENT VALIDATION]\n');
    integrated=evaluate_selected_integrated(selected,cfg0,limits,obs,nom_fun,opt,trig);
    fprintf('continuous C6 trajectory          : %d\n',integrated.c6_all_pass);
    fprintf('sampled derivative finite/smooth  : %d\n',integrated.derivative_sample_pass);
    fprintf('force/thrust proxy finite/smooth  : %d\n',integrated.input_proxy_pass);
    fprintf('independent obstacle certificate  : %d\n',integrated.safety_pass);
    fprintf('tracking bounds v/a/j             : %d\n',integrated.tracking_pass);
    fprintf('multiple-obstacle stress cases    : %d/%d certified\n', ...
        integrated.multi_certified,integrated.multi_total);
    fprintf('pose/size/location stress cases   : %d/%d certified\n', ...
        integrated.robust_certified,integrated.robust_total);
    fprintf(['NOTE: force/thrust proxy is NOT a proof of actual motor/torque-input ', ...
        'smoothness. Exact input proof requires the user''s full controller/actuator map.\n']);

    % Visualization for the user. Not included in planner timing.
    make_phase714_figures(selected,cfg0,limits,obs,nom_fun,opt,trig,risk,integrated);
end


% H. Phase 7-15 structured stress / replan / timing tests.
phase715=struct();
if execute
    fprintf('\n[H: PHASE 7-15 STRUCTURED STRESS TEST]\n');
    phase715=run_phase715_suite(selected,cfg0,limits,obs,nom_fun,opt,trig,risk);
    fprintf('structured static cases : %d/%d certified\n', ...
        phase715.static_certified,phase715.static_total);
    fprintf('multi-obstacle cases    : %d/%d certified\n', ...
        phase715.multi_certified,phase715.multi_total);
    fprintf('replan splice cases     : %d/%d C6-consistent\n', ...
        phase715.replan_c6_pass,phase715.replan_total);
    fprintf('replan certified cases  : %d/%d certified\n', ...
        phase715.replan_certified,phase715.replan_total);
    fprintf('timing trials           : %d\n',phase715.timing.n);
    fprintf('planner timing [ms] median/P95/P99/max = %.3f / %.3f / %.3f / %.3f\n', ...
        phase715.timing.median,phase715.timing.p95,phase715.timing.p99,phase715.timing.max);
    fprintf(['IMPORTANT: current MATLAB timing is diagnostic implementation timing, ', ...
        'not the final 3 ms implementation and not WCET.\n']);
    make_phase715_figures(phase715);
end

% I. Phase 7-16: diagnose ONLY failed structured cases and profile one
% online candidate path internally.  Plotting/printing are outside timers.
phase716=struct();
if execute
    fprintf('\n[I: PHASE 7-16 FAILURE BOUNDARY + INTERNAL TIMING PROFILE]\n');
    phase716=run_phase716_suite(selected,cfg0,limits,obs,nom_fun,opt,trig,phase715);

    fprintf('\n[I1: FAILED-CASE RECOVERY CLASSIFICATION]\n');
    fprintf('case | base(G/D/GD/cert) | recovery | N | T | dT | cert | rho*\n');
    for k=1:numel(phase716.failure_records)
        q=phase716.failure_records(k);
        fprintf('%-12s | %d/%d/%d/%d | %-20s | %2d | %6.3f | %4.1f | %d | %.2e\n', ...
            q.name,q.base_G,q.base_D,q.base_GD,q.base_cert,q.classification, ...
            q.recovery_N,q.recovery_T,q.recovery_dT,q.recovered,q.recovery_rho);
    end
    fprintf('failed structured cases = %d\n',phase716.n_failed);
    fprintf('recovered by T only     = %d\n',phase716.n_T_only);
    fprintf('recovered by N only     = %d\n',phase716.n_N_only);
    fprintf('recovered by N + T      = %d\n',phase716.n_NT);
    fprintf('not recovered tested set= %d\n',phase716.n_unrecovered);

    P=phase716.profile;
    fprintf('\n[I2: INTERNAL ONLINE-PATH TIMING]\n');
    fprintf('trials = %d\n',P.n);
    fprintf('stage                         median      P95       P99       max [ms]\n');
    fprintf('build_static_cache          %9.3f %9.3f %9.3f %9.3f\n',P.cache.median,P.cache.p95,P.cache.p99,P.cache.max);
    fprintf('active_corridor             %9.3f %9.3f %9.3f %9.3f\n',P.corridor.median,P.corridor.p95,P.corridor.p99,P.corridor.max);
    fprintf('dynamic_matrix              %9.3f %9.3f %9.3f %9.3f\n',P.dynamic_matrix.median,P.dynamic_matrix.p95,P.dynamic_matrix.p99,P.dynamic_matrix.max);
    fprintf('geometry_feasibility       %9.3f %9.3f %9.3f %9.3f\n',P.geometry_feas.median,P.geometry_feas.p95,P.geometry_feas.p99,P.geometry_feas.max);
    fprintf('dynamics_feasibility       %9.3f %9.3f %9.3f %9.3f\n',P.dynamics_feas.median,P.dynamics_feas.p95,P.dynamics_feas.p99,P.dynamics_feas.max);
    fprintf('combined_feasibility       %9.3f %9.3f %9.3f %9.3f\n',P.combined_feas.median,P.combined_feas.p95,P.combined_feas.p99,P.combined_feas.max);
    fprintf('apply_candidate             %9.3f %9.3f %9.3f %9.3f\n',P.apply.median,P.apply.p95,P.apply.p99,P.apply.max);
    fprintf('independent_verifier       %9.3f %9.3f %9.3f %9.3f\n',P.verifier.median,P.verifier.p95,P.verifier.p99,P.verifier.max);
    fprintf('TOTAL PROFILED PATH        %9.3f %9.3f %9.3f %9.3f\n',P.total.median,P.total.p95,P.total.p99,P.total.max);
    fprintf('median share: cache %.1f%%, corridor %.1f%%, dyn-matrix %.1f%%, QPs %.1f%%, verifier %.1f%%\n', ...
        P.share.cache,P.share.corridor,P.share.dynamic_matrix,P.share.qps,P.share.verifier);
    fprintf('problem size: nz=%d, spans=%d, active-pairs=%d, G rows=%d, D rows=%d\n', ...
        P.problem.nz,P.problem.nspans,P.problem.active_pairs,P.problem.G_rows,P.problem.D_rows);
    fprintf('NOTE: this isolates algorithmic MATLAB stages; it is not a 3 ms claim or WCET.\n');

    make_phase716_figures(phase716);
end

% J. Phase 7-17: use the already-cached quadratic trajectory objective for
% shape improvement, and perform TRUE recursive replanning from the current
% certified trajectory's derivatives 0..6.
phase717=struct();
if false && execute
    fprintf('\n[J: PHASE 7-17 RECURSIVE REPLAN + TRAJECTORY OPTIMIZATION]\n');
    phase717=run_phase717_suite(selected,cfg0,limits,obs,nom_fun,opt,trig);

    S=phase717.shape;
    fprintf('\n[J1: CERTIFIED TRAJECTORY SHAPE OPTIMIZATION]\n');
    fprintf('baseline certified              : %d\n',S.baseline_cert);
    fprintf('optimized certified             : %d\n',S.optimized_cert);
    fprintf('baseline/optimized path length  : %.4f / %.4f m\n',S.baseline_length,S.optimized_length);
    fprintf('baseline/optimized max nom dev  : %.4f / %.4f m\n',S.baseline_max_nom_dev,S.optimized_max_nom_dev);
    fprintf('baseline/optimized RMS nom dev  : %.4f / %.4f m\n',S.baseline_rms_nom_dev,S.optimized_rms_nom_dev);
    fprintf('baseline/optimized jerk energy  : %.4e / %.4e\n',S.baseline_jerk_energy,S.optimized_jerk_energy);
    fprintf('baseline/optimized snap energy  : %.4e / %.4e\n',S.baseline_snap_energy,S.optimized_snap_energy);
    fprintf('optimized min_g                 : %+.6f m\n',S.optimized_min_g);
    fprintf('optimized v/a/j                 : %.3f / %.3f / %.3f\n',S.optimized_v,S.optimized_a,S.optimized_j);
    fprintf('optimized C6 start/end error    : %.3e / %.3e\n',S.c6_start,S.c6_end);
    fprintf('shape QP time                   : %.3f ms\n',S.solve_ms);

    R=phase717.recursive;
    fprintf('\n[J2: TRUE RECURSIVE REPLANNING]\n');
    fprintf('case | fraction | C6 start | cert | min_g | N | T | ms\n');
    for k=1:numel(R.records)
        q=R.records(k);
        fprintf('%4d | %8.2f | %.2e | %4d | %+6.3f | %2d | %5.2f | %.2f\n', ...
            k,q.fraction,q.c6_start_error,q.cert,q.min_g,q.N,q.T,q.ms);
    end
    fprintf('recursive replans certified     : %d/%d\n',R.certified,R.total);
    fprintf('recursive C6 starts passed      : %d/%d\n',R.c6_pass,R.total);
    fprintf(['NOTE: every recursive candidate is independently certified against ', ...
        'all obstacles before it is counted as executable.\n']);

    make_phase717_figures(phase717,selected,obs,nom_fun,opt,trig);
end

% K. Phase 7-18: certified smooth-arc selection and recursive-replan diagnosis.
phase718=struct();
if false && execute
    fprintf('\n[K: PHASE 7-18 CERTIFIED SMOOTH ARC + REPLAN DIAGNOSTIC]\n');
    phase718=run_phase718_suite(selected,cfg0,limits,obs,nom_fun,opt,trig);

    A=phase718.arc;
    fprintf('\n[K1: CERTIFIED SMOOTH-ARC SELECTION]\n');
    fprintf('candidate | cert | lambda | min_g | length | maxdev | jerkE | snapE\n');
    for k=1:numel(A.records)
        q=A.records(k);
        fprintf('%-10s | %4d | %6.3f | %+6.3f | %6.3f | %6.3f | %.3e | %.3e\n', ...
            q.name,q.cert,q.lambda,q.min_g,q.length,q.maxdev,q.jerkE,q.snapE);
    end
    fprintf('selected                     : %s\n',A.selected_name);
    fprintf('selected independently cert. : %d\n',A.selected_cert);
    fprintf('selected C6 start/end        : %.3e / %.3e\n',A.c6_start,A.c6_end);
    fprintf('selected v/a/j               : %.3f / %.3f / %.3f\n',A.v,A.a,A.j);

    R=phase718.replan;
    fprintf('\n[K2: RECURSIVE REPLAN DIAGNOSTIC]\n');
    fprintf('fraction | boundary-only | +new obstacle | start C6 | reason\n');
    for k=1:numel(R.records)
        q=R.records(k);
        fprintf('%8.2f | %13d | %13d | %.2e | %s\n', ...
            q.fraction,q.boundary_cert,q.newobs_cert,q.c6_error,q.reason);
    end
    fprintf('boundary-only certified : %d/%d\n',R.boundary_certified,R.total);
    fprintf('+new-obstacle certified : %d/%d\n',R.newobs_certified,R.total);

    make_phase718_figures(phase718,obs,nom_fun);
end

% L. Phase 7-19: recursive replanning using ONLY the hard-feasible candidate.
% No shape/arc optimization is allowed to replace a certified feasibility
% solution. This phase diagnoses G, D, G+D, verifier, and C6 separately.
phase719=struct();
if execute
    fprintf('\n[L: PHASE 7-19 TRUE RECURSIVE C6 REPLAN VALIDATION]\n');
    phase719=run_phase719_recursive_validation(selected,cfg0,limits,obs,nom_fun,opt,trig);

    fprintf('\n[L1: BOUNDARY-ONLY REPLAN]\n');
    fprintf('frac | cert | G D GD | verify | C6s/C6e | min_g | N | T | reason\n');
    for k=1:numel(phase719.boundary)
        q=phase719.boundary(k);
        fprintf('%4.2f | %4d | %d %d  %d | %6d | %.1e/%.1e | %+6.3f | %2g | %5.2f | %s\n', ...
            q.fraction,q.cert,q.G,q.D,q.GD,q.verify,q.c6s,q.c6e,q.min_g,q.N,q.T,q.reason);
    end
    fprintf('boundary-only certified = %d/%d\n', ...
        sum([phase719.boundary.cert]),numel(phase719.boundary));

    fprintf('\n[L2: NEW-OBSTACLE REPLAN]\n');
    fprintf('frac | cert | G D GD | verify | C6s/C6e | min_g | N | T | reason\n');
    for k=1:numel(phase719.newobs)
        q=phase719.newobs(k);
        fprintf('%4.2f | %4d | %d %d  %d | %6d | %.1e/%.1e | %+6.3f | %2g | %5.2f | %s\n', ...
            q.fraction,q.cert,q.G,q.D,q.GD,q.verify,q.c6s,q.c6e,q.min_g,q.N,q.T,q.reason);
    end
    fprintf('new-obstacle certified  = %d/%d\n', ...
        sum([phase719.newobs.cert]),numel(phase719.newobs));

    fprintf('\n[L3: RECURSIVE C6 SUMMARY]\n');
    fprintf('No experimental shape optimizer is used in Phase 7-19.\n');
    fprintf('A candidate executes only after combined feasibility + independent verifier + C6 gates.\n');
    make_phase719_figures(phase719,selected,obs,nom_fun);
end

% M. Phase 7-20: production-style event-driven recursive replanning.
% Replanning is NOT forced at arbitrary fractions.  A new obstacle becomes
% known to the planner, the currently executing certified trajectory is
% scanned forward, and replanning starts only when future collision is
% predicted.  Phase 7-19 remains an offline arbitrary-boundary stress test.
phase720=struct();
if execute
    fprintf('\n[M: PHASE 7-20 EVENT-DRIVEN REPLAN + ONLINE TIMING]\n');
    phase720=run_phase720_event_driven(selected,cfg0,limits,obs,nom_fun,opt,trig);

    fprintf('\n[M1: EVENT-DRIVEN REPLAN]\n');
    fprintf('case | detect | trigger | risk | TTC | replan | cert | C6start | min_g | N | T | reason\n');
    for k=1:numel(phase720.events)
        q=phase720.events(k);
        fprintf('%4d | %6.3f | %7.3f | %5.3f | %4.2f | %6d | %4d | %.2e | %+6.3f | %2g | %5.2f | %s\n', ...
            k,q.t_detect,q.t_trigger,q.t_risk,q.TTC,q.replan_requested,q.cert, ...
            q.c6s,q.min_g,q.N,q.T,q.reason);
    end
    fprintf('event-triggered replans certified = %d/%d requested\n', ...
        sum([phase720.events.cert]),sum([phase720.events.replan_requested]));
    fprintf('IMPORTANT: obstacle detection alone does NOT force replanning; future predicted collision does.\n');

    P=phase720.timing;
    fprintf('\n[M2: HOT ONLINE-PATH TIMING, DIAGNOSTIC QPs REMOVED]\n');
    fprintf('trials = %d\n',P.trials);
    fprintf('stage                         median      P95       P99       max [ms]\n');
    print_phase720_timing('active_corridor',P.corridor);
    print_phase720_timing('combined_QP_only',P.combined);
    print_phase720_timing('apply_candidate',P.apply);
    print_phase720_timing('independent_verifier',P.verify);
    print_phase720_timing('HOT TOTAL',P.total);
    fprintf('Excluded from hot path: geometry-only QP, dynamics-only QP, plotting, stress sweeps.\n');
    fprintf('Precomputed in this benchmark: spline cache and dynamic matrix.\n');
    fprintf('3 ms target gap (median) = %.1fx\n',P.median_gap_to_3ms);

    fprintf('\n[M3: COMPUTATION-TIME BOTTLENECK PLAN]\n');
    fprintf('1) remove diagnostic G-only/D-only QPs from online execution\n');
    fprintf('2) precompute obstacle-independent B-spline/derivative/sparsity matrices\n');
    fprintf('3) use one warm-started fixed-size combined QP\n');
    fprintf('4) update only obstacle-dependent active-corridor rows\n');
    fprintf('5) reduce verifier work with cached span data; verifier remains global/sufficient\n');
    fprintf('6) after MATLAB algorithm is fixed, move hot path to generated C/C++/specialized solver\n');
    fprintf('NOTE: no 3 ms guarantee is claimed by this MATLAB diagnostic.\n');

    make_phase720_figures(phase720,selected,obs,nom_fun);
end

result=struct(); result.execute=execute; result.reason=reason;
result.trigger=trig; result.risk_window=risk; result.coarse_records=records;
result.fine_records=fine_records; result.selected=selected;
result.integrated=integrated;
result.phase715=phase715;
result.phase716=phase716;
result.phase717=phase717;
result.phase718=phase718;
result.phase719=phase719;
result.phase720=phase720;
end








% =========================================================================
% Phase 7-20: event-driven recursive replanning + hot-path timing
% =========================================================================
function O=run_phase720_event_driven(sel,cfg,limits,obs,nom_fun,opt,trig)
ce=sel.splice.cache; Pe=sel.splice.P;
alpha=1/sqrt(3);

% Three deterministic scenarios.  The fractions define only when a NEW
% obstacle becomes observable and where it lies on the CURRENT certified
% trajectory.  They do NOT directly command a replan.
detect_frac=[0.05 0.10 0.15];
risk_frac  =[0.42 0.55 0.68];
events=struct([]);

for k=1:numel(detect_frac)
    td=ce.t0+detect_frac(k)*ce.T;
    ur=risk_frac(k);
    pr=eval_spline(ce,Pe,ur,0);
    vr=eval_spline(ce,Pe,ur,1);

    obnew=obs(1);
    obnew.center=pr;
    if isfield(obnew,'radii')
        % Moderate obstacle, smaller than the original benchmark obstacle.
        obnew.radii=obs(1).radii.*[0.42;0.42;0.42];
        obnew.R=eul2rotm_local([0.08*k,-0.04*k,0.06*k]);
        obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
    end

    % Offset slightly across the path so the test is not a degenerate
    % obstacle exactly centered on the load trajectory.
    if norm(vr)>1e-9
        et=vr/norm(vr);
        side=[0;1;0]-et*(et'*[0;1;0]);
        if norm(side)<1e-8, side=[1;0;0]-et*(et'*[1;0;0]); end
        side=side/norm(side);
        obnew.center=obnew.center+0.20*(-1)^k*side;
    end

    oo=[obs obnew];

    % Production-style trigger: after detection, scan the currently executing
    % trajectory every control step.  Detection by itself is not a trigger.
    ev=find_future_collision_trigger_spline(ce,Pe,oo,cfg,opt,td,ce.t1,0.025,5.0);
    q=empty_phase720_event();
    q.t_detect=td;
    q.new_obstacle=obnew;

    if ~ev.found
        q.reason='DETECTED_BUT_CURRENT_TRAJECTORY_SAFE';
        q.replan_requested=0;
    else
        q.replan_requested=1;
        q.t_trigger=ev.t_now; q.t_risk=ev.t_risk; q.TTC=ev.TTC;
        ut=(ev.t_now-ce.t0)/ce.T;
        Dstart=zeros(3,7);
        for r=0:6, Dstart(:,r+1)=eval_spline(ce,Pe,ut,r); end

        Ntry=unique([sel.N 34 38 42 46]);
        % Rejoin duration is searched adaptively; no fixed 5 s rejoin.
        extraT=[0 0.5 1 2 3 5 7 9];
        qr=search_phase719_case(cfg,Ntry,extraT,limits,oo,nom_fun, ...
            ev.t_now,sel.T,Dstart,opt,alpha);

        q.cert=qr.cert; q.G=qr.G; q.D=qr.D; q.GD=qr.GD; q.verify=qr.verify;
        q.c6s=qr.c6s; q.c6e=qr.c6e; q.min_g=qr.min_g;
        q.N=qr.N; q.T=qr.T; q.reason=qr.reason;
        q.P=qr.P; q.cache=qr.cache; q.obstacles=oo;
    end

    if k==1, events=q; else, events(k)=q; end
end
O.events=events;

% Hot-path timing: quantify what remains after removing the two diagnostic
% feasibility QPs and after moving obstacle-independent cache/D construction
% out of the repeated online solve.
O.timing=profile_phase720_hot_path(sel,cfg,limits,obs,opt);
end

function ev=find_future_collision_trigger_spline(c,P,obs,cfg,opt,t_detect,t_end,dt,Tlook)
ev=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN,'g_risk',inf,'obs_idx',NaN);
dirs=fibonacci_sphere(opt.separator_grid_N);
if size(dirs,1)==3, dirs3=dirs; else, dirs3=dirs'; end

times=t_detect:dt:t_end;
for it=1:numel(times)
    tn=times(it);
    tf=min(tn+Tlook,t_end);
    future=tn+dt:dt:tf;
    for kk=1:numel(future)
        t=future(kk);
        u=(t-c.t0)/c.T;
        if u<0 || u>1, continue; end
        p=eval_spline(c,P,u,0);
        a=eval_spline(c,P,u,2);
        W=a+cfg.g_acc*cfg.e3; M=norm(W);
        if M<=1e-9, continue; end
        nT=W/M;

        for io=1:numel(obs)
            best=-inf;
            for id=1:size(dirs3,2)
                n=dirs3(:,id);
                Delta=relative_system_support_minus(n,nT,cfg);
                hE=ellipsoid_support(n,obs(io));
                g=n'*p-Delta-hE-cfg.d_margin;
                if g>best, best=g; end
            end
            if best<0
                ev.found=true; ev.t_now=tn; ev.t_risk=t; ev.TTC=t-tn;
                ev.g_risk=best; ev.obs_idx=io; return;
            end
        end
    end
end
end

function q=empty_phase720_event()
q=struct('t_detect',NaN,'t_trigger',NaN,'t_risk',NaN,'TTC',NaN, ...
    'replan_requested',0,'cert',0,'G',0,'D',0,'GD',0,'verify',0, ...
    'c6s',inf,'c6e',inf,'min_g',-inf,'N',NaN,'T',NaN,'reason','', ...
    'P',[],'cache',[],'obstacles',[],'new_obstacle',[]);
end

function P=profile_phase720_hot_path(sel,cfg,limits,obs,opt)
c=sel.splice.cache;
alpha=1/sqrt(3);
m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
ntrial=50;

tc=zeros(ntrial,1); tq=zeros(ntrial,1); ta=zeros(ntrial,1);
tv=zeros(ntrial,1); tt=zeros(ntrial,1);

% Warm-up calls are intentionally excluded.
[G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
fc=feas_E(c,[G;D],[h;d],opt);
if fc.flag>0 && ~isempty(fc.z)
    Pw=apply_z(c,fc.z); certify_candidate_global(Pw,c,cfg,limits,obs,opt);
end

for k=1:ntrial
    tall=tic;

    t=tic; [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt); tc(k)=toc(t)*1e3;

    t=tic; fc=feas_E(c,[G;D],[h;d],opt); tq(k)=toc(t)*1e3;

    if fc.flag>0 && ~isempty(fc.z)
        t=tic; Pc=apply_z(c,fc.z); ta(k)=toc(t)*1e3;
        t=tic; certify_candidate_global(Pc,c,cfg,limits,obs,opt); tv(k)=toc(t)*1e3;
    else
        ta(k)=NaN; tv(k)=NaN;
    end
    tt(k)=toc(tall)*1e3;
end

P=struct(); P.trials=ntrial;
P.corridor=timing_stats720(tc);
P.combined=timing_stats720(tq);
P.apply=timing_stats720(ta);
P.verify=timing_stats720(tv);
P.total=timing_stats720(tt);
P.median_gap_to_3ms=P.total.median/3.0;
end

function S=timing_stats720(x)
x=x(isfinite(x));
if isempty(x)
    S=struct('median',NaN,'p95',NaN,'p99',NaN,'max',NaN); return;
end
x=sort(x(:)); n=numel(x);
S.median=median(x);
S.p95=x(max(1,ceil(0.95*n)));
S.p99=x(max(1,ceil(0.99*n)));
S.max=max(x);
end

function print_phase720_timing(name,S)
fprintf('%-28s %9.3f %9.3f %9.3f %9.3f\n', ...
    name,S.median,S.p95,S.p99,S.max);
end

function make_phase720_figures(O,sel,obs,nom_fun)
ce=sel.splice.cache; Pe=sel.splice.P;
u=linspace(0,1,500); P0=zeros(3,numel(u)); PN=zeros(3,numel(u));
for k=1:numel(u)
    P0(:,k)=eval_spline(ce,Pe,u(k),0);
    D=nom_fun(ce.t0+u(k)*ce.T); PN(:,k)=D(:,1);
end

figure('Name','Phase 7-20: event-driven recursive replanning','Color','w');
h0=plot3(P0(1,:),P0(2,:),P0(3,:),'LineWidth',2.0); hold on;
hn=plot3(PN(1,:),PN(2,:),PN(3,:),'--','LineWidth',1.1);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
handles=[h0 hn]; labels={'Current certified avoidance','Nominal'};

for k=1:numel(O.events)
    q=O.events(k);
    if q.replan_requested
        % trigger marker is always meaningful
        ut=(q.t_trigger-ce.t0)/ce.T;
        pt=eval_spline(ce,Pe,ut,0);
        hm=plot3(pt(1),pt(2),pt(3),'o','MarkerSize',7,'LineWidth',1.5);
        handles(end+1)=hm; labels{end+1}=sprintf('Trigger %d',k); %#ok<AGROW>
    end
    if q.cert
        ur=linspace(0,1,350); Pr=zeros(3,numel(ur));
        for j=1:numel(ur), Pr(:,j)=eval_spline(q.cache,q.P,ur(j),0); end
        hp=plot3(Pr(1,:),Pr(2,:),Pr(3,:),'LineWidth',1.5);
        handles(end+1)=hp; labels{end+1}=sprintf('Certified replan %d',k); %#ok<AGROW>
        draw_ellipsoid_local(q.new_obstacle);
    end
end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(handles,labels,'Location','best');
title('Event-driven replanning: only future-risk triggers and certified replans');

figure('Name','Phase 7-20: hot-path timing','Color','w');
S=O.timing;
vals=[S.corridor.median S.combined.median S.apply.median S.verify.median];
bar(vals); grid on;
set(gca,'XTick',1:4,'XTickLabel',{'corridor','combined QP','apply','verifier'});
ylabel('median time [ms]');
title(sprintf('Hot online path; total median %.2f ms (target 3 ms)',S.total.median));
end


% =========================================================================
% Phase 7-19: recursive C6 replanning without experimental shape optimization
% =========================================================================
function O=run_phase719_recursive_validation(sel,cfg,limits,obs,nom_fun,opt,trig)
alpha=1/sqrt(3);
Pexec=sel.splice.P; cexec=sel.splice.cache;
fractions=[0.15 0.25 0.35 0.50 0.65 0.80];
Ntry=unique([sel.N 34 38 42 46]);
extraT=[0 0.5 1 2 3 5 7];

B=struct([]); W=struct([]);
for k=1:numel(fractions)
    frac=fractions(k);
    tr=trig.t_now+frac*sel.T;
    ur=(tr-cexec.t0)/cexec.T;

    Dstart=zeros(3,7);
    for r=0:6
        Dstart(:,r+1)=eval_spline(cexec,Pexec,ur,r);
    end

    % A: original obstacle set only. This must be understood before adding
    % another obstacle.
    qb=search_phase719_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,sel.T,Dstart,opt,alpha);
    qb.fraction=frac;
    if k==1, B=qb; else, B(k)=qb; end

    % B: only attempt a new obstacle after the arbitrary C6-boundary case.
    if qb.cert
        obnew=obs(1);
        pnow=Dstart(:,1); vnow=Dstart(:,2);
        if norm(vnow)>1e-9, edir=vnow/norm(vnow); else, edir=[0;0;1]; end

        % A moderate future obstacle. This is deliberately not placed almost
        % on top of the current state; the purpose is recursive functionality.
        lateral=[0;(-1)^k;0];
        if abs(edir'*lateral)>0.8, lateral=[(-1)^k;0;0]; end
        lateral=lateral-edir*(edir'*lateral);
        if norm(lateral)>1e-9, lateral=lateral/norm(lateral); end

        obnew.center=pnow+3.5*edir+1.25*lateral;
        if isfield(obnew,'radii')
            obnew.radii=obs(1).radii.*[0.50;0.50;0.50];
            obnew.R=eul2rotm_local([0.10*k,-0.05*k,0.04*k]);
            obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
        end
        oo=[obs obnew];
        qw=search_phase719_case(cfg,Ntry,extraT,limits,oo,nom_fun,tr,sel.T,Dstart,opt,alpha);
    else
        qw=empty_phase719_record('BOUNDARY_ONLY_FAILED_FIRST');
    end
    qw.fraction=frac;
    if k==1, W=qw; else, W(k)=qw; end
end
O.boundary=B; O.newobs=W;
end

function q=search_phase719_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,Torig,Dstart,opt,alpha)
best=empty_phase719_record('NO_TEST');
remaining=max(2.0,Torig-tr);

for ie=1:numel(extraT)
    Tnew=remaining+extraT(ie);
    Dend=nom_fun(tr+Tnew);

    for in=1:numel(Ntry)
        q=run_phase719_case(cfg,Ntry(in),limits,obs,nom_fun,tr,Tnew,Dstart,Dend,opt,alpha);

        % Keep the deepest diagnostic result even when it fails.
        if diagnostic_depth(q)>diagnostic_depth(best), best=q; end
        if q.cert
            best=q; return;
        end
    end
end
q=best;
end

function q=run_phase719_case(cfg0,N,limits,obs,nom_fun,t0,T,Dstart,Dend,opt,alpha)
q=empty_phase719_record('INIT');
q.N=N; q.T=T;
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache_boundary(cfg,t0,t0+T,nom_fun,Dstart,Dend,opt);

    % Audit endpoint construction BEFORE any optimization.
    es0=zeros(7,1); ee0=zeros(7,1);
    for r=0:6
        es0(r+1)=norm(eval_spline(c,c.Pbase,0,r)-Dstart(:,r+1),inf);
        ee0(r+1)=norm(eval_spline(c,c.Pbase,1,r)-Dend(:,r+1),inf);
    end
    q.base_c6s=max(es0); q.base_c6e=max(ee0);
    if q.base_c6s>opt.c6_tol || q.base_c6e>opt.c6_tol
        q.reason='BOUNDARY_CONSTRUCTION_ERROR'; return;
    end

    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);

    % Separate solves are diagnostic only.
    fg=feas_E(c,G,h,opt);
    fd=feas_E(c,D,d,opt);
    q.G=fg.flag>0; q.D=fd.flag>0;

    % The actual online-relevant candidate is the combined hard-feasible one.
    fc=feas_E(c,[G;D],[h;d],opt);
    q.GD=fc.flag>0; q.rho=fc.rho;
    if fc.flag<=0 || isempty(fc.z)
        if ~q.G, q.reason='GEOMETRY_INFEASIBLE';
        elseif ~q.D, q.reason='DYNAMICS_INFEASIBLE';
        else, q.reason='COMBINED_INFEASIBLE';
        end
        return;
    end

    % IMPORTANT FIX: do NOT replace fc.z by the experimental shape-QP result.
    P=apply_z(c,fc.z);

    cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
    q.verify=cert.pass; q.min_g=cert.min_g_lower;
    q.v=cert.v_bound; q.a=cert.a_bound; q.j=cert.j_bound;

    es=zeros(7,1); ee=zeros(7,1);
    for r=0:6
        es(r+1)=norm(eval_spline(c,P,0,r)-Dstart(:,r+1),inf);
        ee(r+1)=norm(eval_spline(c,P,1,r)-Dend(:,r+1),inf);
    end
    q.c6s=max(es); q.c6e=max(ee);

    if ~q.verify
        q.reason='INDEPENDENT_VERIFIER_REJECT'; return;
    end
    if q.c6s>opt.c6_tol
        q.reason='START_C6_FAIL'; return;
    end
    if q.c6e>opt.c6_tol
        q.reason='END_C6_FAIL'; return;
    end

    q.cert=1; q.reason='CERTIFIED_RECURSIVE_C6';
    q.P=P; q.cache=c; q.obstacles=obs;
catch ME
    q.reason=['ERROR: ' ME.message];
end
end

function q=empty_phase719_record(reason)
q=struct('fraction',NaN,'cert',0,'G',0,'D',0,'GD',0,'verify',0, ...
    'rho',inf,'min_g',-inf,'v',inf,'a',inf,'j',inf, ...
    'c6s',inf,'c6e',inf,'base_c6s',inf,'base_c6e',inf, ...
    'N',NaN,'T',NaN,'reason',reason,'P',[],'cache',[],'obstacles',[]);
end

function d=diagnostic_depth(q)
d=0;
if isfinite(q.base_c6s), d=1; end
if q.G || q.D, d=2; end
if q.GD, d=3; end
if q.verify, d=4; end
if isfinite(q.c6s), d=5; end
if q.cert, d=6; end
end

function make_phase719_figures(O,sel,obs,nom_fun)
% Only certified trajectories are drawn. No orange/unverified shape candidate.
ce=sel.splice.cache; Pe=sel.splice.P;
u=linspace(0,1,500); P0=zeros(3,numel(u)); PN=zeros(3,numel(u));
for k=1:numel(u)
    P0(:,k)=eval_spline(ce,Pe,u(k),0);
    D=nom_fun(ce.t0+u(k)*ce.T); PN(:,k)=D(:,1);
end

figure('Name','Phase 7-19: certified recursive replanning','Color','w');
h0=plot3(P0(1,:),P0(2,:),P0(3,:),'LineWidth',1.8); hold on;
hn=plot3(PN(1,:),PN(2,:),PN(3,:),'--','LineWidth',1.1);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end

handles=[h0 hn]; labels={'Original certified avoidance','Nominal'};
for k=1:numel(O.boundary)
    q=O.boundary(k);
    if q.cert
        ur=linspace(0,1,300); Pr=zeros(3,numel(ur));
        for j=1:numel(ur), Pr(:,j)=eval_spline(q.cache,q.P,ur(j),0); end
        hp=plot3(Pr(1,:),Pr(2,:),Pr(3,:),'LineWidth',1.2);
        handles(end+1)=hp; %#ok<AGROW>
        labels{end+1}=sprintf('Recursive %.0f%%',100*q.fraction); %#ok<AGROW>
    end
end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(handles,labels,'Location','best');
title('Only independently certified recursive trajectories are displayed');

figure('Name','Phase 7-19: recursive diagnostic','Color','w');
f=[O.boundary.fraction];
M=zeros(numel(f),5);
for k=1:numel(f)
    M(k,:)=[O.boundary(k).G O.boundary(k).D O.boundary(k).GD ...
            O.boundary(k).verify O.boundary(k).cert];
end
bar(f,M,'grouped'); grid on; ylim([0 1.2]);
xlabel('Replan fraction of executing avoidance trajectory');
ylabel('pass = 1');
legend('G','D','G+D','Verifier','Final cert','Location','best');
title('Recursive replanning failure-stage diagnosis');
end


% =========================================================================
% Phase 7-18: certified smooth arc + recursive replanning diagnosis
% =========================================================================
function O=run_phase718_suite(sel,cfg,limits,obs,nom_fun,opt,trig)
alpha=1/sqrt(3);
O=struct();

c=sel.splice.cache; P0=sel.splice.P;
[G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
Ahd=[G;D]; bhd=[h;d];

% Baseline metrics.
M0=trajectory_metrics(c,P0,nom_fun);
C0=certify_candidate_global(P0,c,cfg,limits,obs,opt);

% -------------------------------------------------------------------------
% Build two SOFT objectives:
%   1) nominal tracking, but with much stronger derivative regularization;
%   2) smooth one-sided arc guide around the obstacle.
% Neither guide is a safety proof. Every accepted result must pass the
% independent global verifier.
% -------------------------------------------------------------------------
z0=control_points_to_z(c,P0);

% Smooth nominal objective: do not allow position tracking to dominate jerk.
optS=opt;
optS.w_position=0.20;
optS.w_jerk=5e-2;
optS.w_snap=5e-3;
[Hs,fs]=build_objective_cache_with_weights(c,nom_fun,optS);
[zs,fls]=solve_custom_qp(c,Ahd,bhd,z0,Hs,fs,opt);
if fls<=0, zs=z0; end

% Arc guide objective.  Guide is generated geometrically from start/end and
% obstacle center, then blended to zero at both ends so C6 endpoint equations
% remain exactly those already embedded in the spline parameterization.
guide_fun=make_arc_guide(c,P0,obs(1),nom_fun,cfg);
[Ha,fa]=build_guide_objective(c,guide_fun,optS);
[za,fla]=solve_custom_qp(c,Ahd,bhd,z0,Ha,fa,opt);
if fla<=0, za=z0; end

% Independent verifier may reject the nonlinear support geometry even when
% the fixed-corridor QP is feasible.  Since z0 is certified, search along the
% segment z(lambda)=z0+lambda(z*-z0).  Linear QP constraints remain feasible
% by convexity; certification is checked independently at every lambda.
[Ps,rs]=certified_segment_search(c,z0,zs,cfg,limits,obs,opt,nom_fun,'smooth');
[Pa,ra]=certified_segment_search(c,z0,za,cfg,limits,obs,opt,nom_fun,'arc');

r0=metric_record('baseline',P0,c,C0,M0,1.0);
records=[r0 rs ra];

% Select among CERTIFIED candidates using a shape score.  Safety is a gate,
% never a term that can be traded against appearance.
score=inf(1,numel(records));
for k=1:numel(records)
    if records(k).cert
        score(k)=0.45*records(k).maxdev + 0.10*records(k).length + ...
                 0.025*sqrt(max(records(k).jerkE,0)) + ...
                 0.005*sqrt(max(records(k).snapE,0));
    end
end
[~,ib]=min(score);
best=records(ib);
if strcmp(best.name,'baseline'), Pbest=P0;
elseif strcmp(best.name,'smooth'), Pbest=Ps;
else, Pbest=Pa; end

D0=nom_fun(c.t0); D1=nom_fun(c.t1);
es=zeros(7,1); ee=zeros(7,1);
for r=0:6
    es(r+1)=norm(eval_spline(c,Pbest,0,r)-D0(:,r+1),inf);
    ee(r+1)=norm(eval_spline(c,Pbest,1,r)-D1(:,r+1),inf);
end
Cb=certify_candidate_global(Pbest,c,cfg,limits,obs,opt);

O.arc.records=records;
O.arc.selected_name=best.name;
O.arc.selected_cert=Cb.pass;
O.arc.P=Pbest; O.arc.cache=c;
O.arc.c6_start=max(es); O.arc.c6_end=max(ee);
O.arc.v=Cb.v_bound; O.arc.a=Cb.a_bound; O.arc.j=Cb.j_bound;

% -------------------------------------------------------------------------
% Recursive replanning diagnosis.
% First test arbitrary C6 boundary with NO newly added obstacle.
% Only if that works, test the same boundary after adding a new obstacle.
% This distinguishes boundary-generalization failure from obstacle difficulty.
% -------------------------------------------------------------------------
fractions=[0.20 0.35 0.50 0.65];
Ntry=unique([sel.N 34 38 42]);
extraT=[0 1 2 3 5];
rec=struct([]);

for k=1:numel(fractions)
    frac=fractions(k);
    tr=trig.t_now+frac*sel.T;
    ur=(tr-c.t0)/c.T;
    Dstart=zeros(3,7);
    for r=0:6, Dstart(:,r+1)=eval_spline(c,Pbest,ur,r); end

    % Stage A: arbitrary C6 boundary, original obstacles only.
    [okA,rrA,cA,PA]=search_recursive_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,sel.T,Dstart,opt,alpha);

    c6err=inf;
    if okA
        ee=zeros(7,1);
        for r=0:6, ee(r+1)=norm(eval_spline(cA,PA,0,r)-Dstart(:,r+1),inf); end
        c6err=max(ee);
    end

    % Stage B: add a less arbitrary obstacle only after Stage A.
    okB=false; rrB=[];
    if okA
        obnew=obs(1);
        pnow=Dstart(:,1); vnow=Dstart(:,2);
        if norm(vnow)>1e-9, edir=vnow/norm(vnow); else, edir=[0;0;1]; end
        % Put obstacle farther ahead than 7-17 and offset it laterally.
        side=[0;(-1)^k;0];
        obnew.center=pnow+3.0*edir+1.15*side;
        if isfield(obnew,'radii')
            obnew.radii=obs(1).radii.*[0.55;0.55;0.55];
            obnew.R=eul2rotm_local([0.12*k,-0.06*k,0.05*k]);
            obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
        end
        oo=[obs obnew];
        [okB,rrB,~,~]=search_recursive_case(cfg,Ntry,extraT,limits,oo,nom_fun,tr,sel.T,Dstart,opt,alpha);
    end

    if ~okA
        reason='BOUNDARY_REPLAN_FAILED';
    elseif ~okB
        reason='NEW_OBSTACLE_CASE_FAILED';
    else
        reason='CERTIFIED_RECURSIVE_REPLAN';
    end
    x=struct('fraction',frac,'boundary_cert',okA,'newobs_cert',okB, ...
        'c6_error',c6err,'reason',reason,'boundary_result',rrA,'newobs_result',rrB);
    if k==1, rec=x; else, rec(k)=x; end
end
O.replan.records=rec; O.replan.total=numel(rec);
O.replan.boundary_certified=sum([rec.boundary_cert]);
O.replan.newobs_certified=sum([rec.newobs_cert]);
end

function [ok,rrbest,cbest,Pbest]=search_recursive_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,Torig,Dstart,opt,alpha)
ok=false; rrbest=[]; cbest=[]; Pbest=[];
remaining=max(2.0,Torig-tr);
for ie=1:numel(extraT)
    Tnew=remaining+extraT(ie);
    Dend=nom_fun(tr+Tnew);
    for in=1:numel(Ntry)
        [rr,cc,PP]=run_recursive_case(cfg,Ntry(in),limits,obs,nom_fun,tr,Tnew,Dstart,Dend,opt,alpha);
        if rr.cert
            ok=true; rrbest=rr; cbest=cc; Pbest=PP; return;
        end
    end
end
end

function [Pbest,rec]=certified_segment_search(c,z0,zstar,cfg,limits,obs,opt,nom_fun,name)
% Search from the optimized endpoint back toward the known-certified z0.
% This fixes the Phase-7-17 issue where a visually better but uncertified
% nonlinear-support candidate was plotted as if executable.
lams=[1.0 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1 0.0];
Pbest=apply_z(c,z0); Cbest=certify_candidate_global(Pbest,c,cfg,limits,obs,opt);
bestlam=0;
for lam=lams
    z=z0+lam*(zstar-z0);
    P=apply_z(c,z);
    C=certify_candidate_global(P,c,cfg,limits,obs,opt);
    if C.pass
        Pbest=P; Cbest=C; bestlam=lam; break;
    end
end
M=trajectory_metrics(c,Pbest,nom_fun);
rec=metric_record(name,Pbest,c,Cbest,M,bestlam);
end

function r=metric_record(name,P,c,C,M,lambda)
r=struct('name',name,'cert',C.pass,'lambda',lambda,'min_g',C.min_g_lower, ...
    'length',M.length,'maxdev',M.max_nom_dev,'rmsdev',M.rms_nom_dev, ...
    'jerkE',M.jerk_energy,'snapE',M.snap_energy);
end

function [H,f]=build_objective_cache_with_weights(cache,nom_fun,opt)
H=opt.qp_reg*eye(cache.nz); f=zeros(cache.nz,1);
us=linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k); t=cache.t0+u*cache.T; DD=nom_fun(t);

    J0=derivative_map(cache,u,0); q0=eval_spline(cache,cache.Pbase,u,0);
    e0=q0-DD(:,1);
    H=H+2*opt.w_position*(J0'*J0)/numel(us);
    f=f+2*opt.w_position*J0'*e0/numel(us);

    % Velocity and acceleration energies suppress lateral waviness.
    J1=derivative_map(cache,u,1); q1=eval_spline(cache,cache.Pbase,u,1);
    J2=derivative_map(cache,u,2); q2=eval_spline(cache,cache.Pbase,u,2);
    H=H+2*2e-3*(J1'*J1)/numel(us); f=f+2*2e-3*J1'*q1/numel(us);
    H=H+2*1e-2*(J2'*J2)/numel(us); f=f+2*1e-2*J2'*q2/numel(us);

    J3=derivative_map(cache,u,3); q3=eval_spline(cache,cache.Pbase,u,3);
    H=H+2*opt.w_jerk*(J3'*J3)/numel(us);
    f=f+2*opt.w_jerk*J3'*q3/numel(us);

    J4=derivative_map(cache,u,4); q4=eval_spline(cache,cache.Pbase,u,4);
    H=H+2*opt.w_snap*(J4'*J4)/numel(us);
    f=f+2*opt.w_snap*J4'*q4/numel(us);
end
H=(H+H')/2;
end

function guide_fun=make_arc_guide(cache,Pbase,o,nom_fun,cfg)
% Geometric one-sided half-ellipse-like guide in the plane formed by the
% nominal travel direction and the obstacle lateral direction.  It is only
% a generator preference, never a certificate.
tm=0.5*(cache.t0+cache.t1);
Dm=nom_fun(tm);
v=Dm(:,2);
if norm(v)<1e-9, v=[0;0;1]; end
eT=v/norm(v);
pm=Dm(:,1);
dc=o.center-pm;
eL=dc-eT*(eT'*dc);
if norm(eL)<1e-8
    tmp=[1;0;0]; if abs(eT'*tmp)>0.9, tmp=[0;1;0]; end
    eL=tmp-eT*(eT'*tmp);
end
eL=eL/norm(eL);

% Choose the side opposite the obstacle-center lateral projection.
if eL'*(o.center-pm)>0, eL=-eL; end

% Radius is geometry-based, not a fixed path shape.
if isfield(o,'radii'), ro=max(o.radii); else, ro=sqrt(max(eig(o.S))); end
Rlat=ro+cfg.d_margin+0.55;
guide_fun=@guide_eval;

    function pg=guide_eval(t)
        tau=(t-cache.t0)/cache.T;
        tau=min(max(tau,0),1);
        D=nom_fun(t);
        % Smooth half-sine lateral bypass; zero at both ends and one-sided.
        % Endpoint C6 is still imposed exactly by the spline fixed controls.
        bump=sin(pi*tau)^2;
        pg=D(:,1)+Rlat*bump*eL;
    end
end

function [H,f]=build_guide_objective(cache,guide_fun,opt)
H=opt.qp_reg*eye(cache.nz); f=zeros(cache.nz,1);
us=linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k); t=cache.t0+u*cache.T;
    J0=derivative_map(cache,u,0); q0=eval_spline(cache,cache.Pbase,u,0);
    e=q0-guide_fun(t);
    % Arc guide is soft; smoothness has substantial weight.
    H=H+2*0.35*(J0'*J0)/numel(us); f=f+2*0.35*J0'*e/numel(us);

    J2=derivative_map(cache,u,2); q2=eval_spline(cache,cache.Pbase,u,2);
    H=H+2*2e-2*(J2'*J2)/numel(us); f=f+2*2e-2*J2'*q2/numel(us);
    J3=derivative_map(cache,u,3); q3=eval_spline(cache,cache.Pbase,u,3);
    H=H+2*8e-2*(J3'*J3)/numel(us); f=f+2*8e-2*J3'*q3/numel(us);
    J4=derivative_map(cache,u,4); q4=eval_spline(cache,cache.Pbase,u,4);
    H=H+2*8e-3*(J4'*J4)/numel(us); f=f+2*8e-3*J4'*q4/numel(us);
end
H=(H+H')/2;
end

function [z,flag]=solve_custom_qp(cache,A,b,z0,H,f,opt)
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',500);
try
    [z,~,flag]=quadprog(H,f,A,b,[],[],lb,ub,z0,qo);
catch
    z=[]; flag=-99;
end
end

function make_phase718_figures(O,obs,nom_fun)
A=O.arc; c=A.cache; ns=500; u=linspace(0,1,ns); t=c.t0+u*c.T;
Pbase=zeros(3,ns); Pbest=zeros(3,ns); Pnom=zeros(3,ns);
for k=1:ns
    % baseline is recoverable from the first metric only through selected cache;
    % use cache.Pbase only for nominal fitting is not the certified baseline, so
    % reconstruct selected and nominal here; Phase 7-17 figure already retains
    % the exact old baseline comparison.
    Pbest(:,k)=eval_spline(c,A.P,u(k),0);
    DD=nom_fun(t(k)); Pnom(:,k)=DD(:,1);
end

figure('Name','Phase 7-18: selected certified smooth trajectory','Color','w');
hs=plot3(Pbest(1,:),Pbest(2,:),Pbest(3,:),'LineWidth',2.2); hold on;
hn=plot3(Pnom(1,:),Pnom(2,:),Pnom(3,:),'--','LineWidth',1.3);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend([hs hn],{['Selected certified: ' A.selected_name],'Nominal'},'Location','best');
title('Phase 7-18: only independently certified shape candidates are selectable');

figure('Name','Phase 7-18: certified candidate metrics','Color','w');
R=A.records;
vals=[[R.maxdev]' [R.length]' sqrt([R.jerkE]')];
bar(vals); grid on;
set(gca,'XTick',1:numel(R),'XTickLabel',{R.name});
legend('Max nominal deviation [m]','Path length [m]','sqrt(jerk energy)','Location','best');
title('Certified shape alternatives (baseline retained)');
end


% =========================================================================
% Phase 7-17: true recursive replanning + certified shape optimization
% =========================================================================
function O=run_phase717_suite(sel,cfg,limits,obs,nom_fun,opt,trig)
alpha=1/sqrt(3);
O=struct();

% -------------------------------------------------------------------------
% J1. Improve trajectory shape INSIDE the same hard certified corridor.
% Safety and dynamics remain hard constraints.  The objective already built
% in build_objective_cache is:
%   nominal deviation + jerk energy + snap energy + tiny regularization.
% The old Phase-7 feasibility point is retained as a feasible warm start.
% -------------------------------------------------------------------------
c=sel.splice.cache;
Pbase=sel.splice.P;
[G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
A=[G;D]; b=[h;d];

% Recover the z corresponding to the selected feasible trajectory.
z0=control_points_to_z(c,Pbase);
tt=tic;
[zopt,flag,~]=solve_shape_qp_from_feasible(c,A,b,z0,opt);
solve_ms=toc(tt)*1000;

if flag>0 && ~isempty(zopt)
    Popt=apply_z(c,zopt);
    certopt=certify_candidate_global(Popt,c,cfg,limits,obs,opt);
else
    Popt=Pbase;
    certopt=certify_candidate_global(Pbase,c,cfg,limits,obs,opt);
end

Mb=trajectory_metrics(c,Pbase,nom_fun);
Mo=trajectory_metrics(c,Popt,nom_fun);
D0=nom_fun(c.t0); D1=nom_fun(c.t1);
es=zeros(7,1); ee=zeros(7,1);
for r=0:6
    es(r+1)=norm(eval_spline(c,Popt,0,r)-D0(:,r+1),inf);
    ee(r+1)=norm(eval_spline(c,Popt,1,r)-D1(:,r+1),inf);
end

S=struct();
S.baseline_cert=certify_candidate_global(Pbase,c,cfg,limits,obs,opt).pass;
S.optimized_cert=certopt.pass && max(es)<=opt.c6_tol && max(ee)<=opt.c6_tol;
S.baseline_length=Mb.length; S.optimized_length=Mo.length;
S.baseline_max_nom_dev=Mb.max_nom_dev; S.optimized_max_nom_dev=Mo.max_nom_dev;
S.baseline_rms_nom_dev=Mb.rms_nom_dev; S.optimized_rms_nom_dev=Mo.rms_nom_dev;
S.baseline_jerk_energy=Mb.jerk_energy; S.optimized_jerk_energy=Mo.jerk_energy;
S.baseline_snap_energy=Mb.snap_energy; S.optimized_snap_energy=Mo.snap_energy;
S.optimized_min_g=certopt.min_g_lower;
S.optimized_v=certopt.v_bound; S.optimized_a=certopt.a_bound; S.optimized_j=certopt.j_bound;
S.c6_start=max(es); S.c6_end=max(ee); S.solve_ms=solve_ms;
S.P_baseline=Pbase; S.P_optimized=Popt; S.cache=c;
O.shape=S;

% Use optimized trajectory only if it independently certifies; otherwise the
% already-certified baseline remains the executing trajectory.
if S.optimized_cert, Pexec=Popt; else, Pexec=Pbase; end

% -------------------------------------------------------------------------
% J2. TRUE recursive replanning.
% Start derivatives 0..6 come from the CURRENT certified trajectory at tr.
% End derivatives 0..6 match the nominal trajectory at the new rejoin time.
% A newly detected obstacle is added ahead of the executing trajectory.
% -------------------------------------------------------------------------
fractions=[0.20 0.35 0.50 0.65];
Ntry=unique([sel.N 34 38 42]);
extraT=[0 1 2 3];
rec=struct([]);

for k=1:numel(fractions)
    frac=fractions(k);
    tr=trig.t_now+frac*sel.T;
    ur=(tr-c.t0)/c.T;
    Dstart=zeros(3,7);
    for r=0:6, Dstart(:,r+1)=eval_spline(c,Pexec,ur,r); end

    % New obstacle ahead of the current trajectory. Alternating lateral side
    % avoids validating only one preferred bypass direction.
    obnew=obs(1);
    pnow=Dstart(:,1);
    vnow=Dstart(:,2);
    if norm(vnow)>1e-9, edir=vnow/norm(vnow); else, edir=[0;0;1]; end
    side=[0;(-1)^k;0];
    obnew.center=pnow+1.6*edir+0.75*side;
    if isfield(obnew,'radii')
        obnew.radii=obs(1).radii.*[0.72;0.72;0.72];
        obnew.R=eul2rotm_local([0.18*k,-0.10*k,0.08*k]);
        obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
    end
    oo=[obs obnew];

    found=false; best=[]; bestc=[]; bestP=[]; bestms=NaN;
    for ie=1:numel(extraT)
        Tnew=max(2.0,(sel.T-(tr-trig.t_now))+extraT(ie));
        tend=tr+Tnew;
        Dend=nom_fun(tend);
        for in=1:numel(Ntry)
            tt=tic;
            [rr,cc,PP]=run_recursive_case(cfg,Ntry(in),limits,oo,nom_fun,tr,Tnew,Dstart,Dend,opt,alpha);
            ms=toc(tt)*1000;
            if rr.cert
                found=true; best=rr; bestc=cc; bestP=PP; bestms=ms; break;
            end
        end
        if found, break; end
    end

    if found
        estart=zeros(7,1);
        for r=0:6
            estart(r+1)=norm(eval_spline(bestc,bestP,0,r)-Dstart(:,r+1),inf);
        end
        x=struct('fraction',frac,'cert',1,'c6_start_error',max(estart), ...
            'min_g',best.min_g,'N',best.N,'T',best.T,'ms',bestms, ...
            'P',bestP,'cache',bestc,'obstacles',oo);
    else
        x=struct('fraction',frac,'cert',0,'c6_start_error',inf, ...
            'min_g',-inf,'N',NaN,'T',NaN,'ms',NaN, ...
            'P',[],'cache',[],'obstacles',oo);
    end
    if k==1, rec=x; else, rec(k)=x; end
end
R.records=rec; R.total=numel(rec);
R.certified=sum([rec.cert]);
R.c6_pass=sum([rec.c6_start_error]<=opt.c6_tol);
O.recursive=R;
end

function [rr,c,P]=run_recursive_case(cfg0,N,limits,obs,nom_fun,t0,T,Dstart,Dend,opt,alpha)
rr=struct('N',N,'T',T,'G',0,'D',0,'GD',0,'cert',0,'rho',inf, ...
    'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
c=[]; P=[];
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache_boundary(cfg,t0,t0+T,nom_fun,Dstart,Dend,opt);
    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);

    % Online-relevant combined feasibility first.
    fc=feas_E(c,[G;D],[h;d],opt);
    rr.GD=fc.flag>0; rr.rho=fc.rho;
    if fc.flag<=0 || isempty(fc.z), return; end

    % Shape-optimize from the known feasible point, preserving hard constraints.
    [zopt,fl,~]=solve_shape_qp_from_feasible(c,[G;D],[h;d],fc.z,opt);
    if fl>0 && ~isempty(zopt), z=zopt; else, z=fc.z; end
    P=apply_z(c,z);

    cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
    rr.cert=cert.pass; rr.min_g=cert.min_g_lower;
    rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;

    % Exact recursive start and nominal end C6 gates.
    es=zeros(7,1); ee=zeros(7,1);
    for r=0:6
        es(r+1)=norm(eval_spline(c,P,0,r)-Dstart(:,r+1),inf);
        ee(r+1)=norm(eval_spline(c,P,1,r)-Dend(:,r+1),inf);
    end
    rr.cert=rr.cert && max(es)<=opt.c6_tol && max(ee)<=opt.c6_tol;
    rr.G=rr.cert; rr.D=rr.cert; % diagnostic fields not separately solved here
catch ME
    rr.error=ME.message;
end
end

function cache=build_static_cache_boundary(cfg,t0,t1,nom_fun,D0,D1,opt)
% Same cache mathematics as build_static_cache, except endpoint derivatives
% are supplied explicitly.  This is what enables recursive C6 replanning.
p=cfg.p; N=cfg.N_ctrl; T=t1-t0;
if p~=7, error('Phase7-17 recursive boundary implementation assumes p=7.'); end
if N<=2*p, error('N_ctrl must be > 2*p.'); end

knots=clamped_uniform_knots(N,p);
free_idx=(p+1):(N-p); nf=numel(free_idx); nz=3*nf;
Pfixed=solve_endpoint_fixed(p,N,knots,T,D0,D1);

u_fit=linspace(0,1,max(80,5*N))';
A=zeros(numel(u_fit),nf); Y=zeros(numel(u_fit),3);
for k=1:numel(u_fit)
    B=basis_derivative_all(u_fit(k),knots,p,N,0);
    Dn=nom_fun(t0+u_fit(k)*T);
    A(k,:)=B(free_idx);
    Y(k,:)=Dn(:,1)'-B*Pfixed;
end
Pbase=Pfixed; Pbase(free_idx,:)=A\Y;

cache=struct('p',p,'N',N,'T',T,'t0',t0,'t1',t1,'knots',knots, ...
    'free_idx',free_idx,'nf',nf,'nz',nz,'Pbase',Pbase);
uk=unique(knots);
cache.spans=[uk(1:end-1)',uk(2:end)'];
cache.spans=cache.spans(cache.spans(:,2)>cache.spans(:,1),:);
cache.nspans=size(cache.spans,1);

xi=linspace(0,1,p+1)'; Bern=zeros(p+1);
for j=1:p+1, Bern(j,:)=bernstein_row(p,xi(j)); end
cache.Bern_inv=inv(Bern);

cache.B0=cell(cache.nspans,1); cache.Bz=cell(cache.nspans,1);
for sp=1:cache.nspans
    [B0,Bz]=span_bezier_affine(cache,cache.spans(sp,1),cache.spans(sp,2));
    cache.B0{sp}=B0; cache.Bz{sp}=Bz;
end
[cache.H,cache.f]=build_objective_cache(cache,nom_fun,opt);

cache.mid=zeros(cache.nspans,1); cache.pmid=zeros(3,cache.nspans); cache.amid=zeros(3,cache.nspans);
for sp=1:cache.nspans
    u=mean(cache.spans(sp,:)); cache.mid(sp)=u;
    cache.pmid(:,sp)=eval_spline(cache,Pbase,u,0);
    cache.amid(:,sp)=eval_spline(cache,Pbase,u,2);
end
end

function z=control_points_to_z(cache,P)
z=zeros(cache.nz,1);
for j=1:cache.nf
    ii=cache.free_idx(j);
    dz=(P(ii,:)-cache.Pbase(ii,:))';
    z(3*(j-1)+(1:3))=dz;
end
end

function [z,flag,info]=solve_shape_qp_from_feasible(cache,A,b,z0,opt)
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
% interior-point-convex is used here for robust offline validation of the
% convex quadratic objective. Deployment solver choice is a later timing phase.
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',500);
try
    [z,~,flag,out]=quadprog(cache.H,cache.f,A,b,[],[],lb,ub,z0,qo);
    info=out;
catch ME
    z=[]; flag=-99; info=struct('message',ME.message);
end
end

function M=trajectory_metrics(cache,P,nom_fun)
ns=500; u=linspace(0,1,ns); t=cache.t0+u*cache.T;
Q=zeros(3,ns); J=zeros(3,ns); S=zeros(3,ns); dev=zeros(1,ns);
for k=1:ns
    Q(:,k)=eval_spline(cache,P,u(k),0);
    J(:,k)=eval_spline(cache,P,u(k),3);
    S(:,k)=eval_spline(cache,P,u(k),4);
    D=nom_fun(t(k)); dev(k)=norm(Q(:,k)-D(:,1));
end
ds=vecnorm(diff(Q,1,2),2,1);
M.length=sum(ds);
M.max_nom_dev=max(dev);
M.rms_nom_dev=sqrt(mean(dev.^2));
M.jerk_energy=trapz(t,sum(J.^2,1));
M.snap_energy=trapz(t,sum(S.^2,1));
end

function make_phase717_figures(O,sel,obs,nom_fun,opt,trig)
S=O.shape; c=S.cache;
ns=500; u=linspace(0,1,ns); t=c.t0+u*c.T;
Pb=zeros(3,ns); Po=zeros(3,ns); Pn=zeros(3,ns);
for k=1:ns
    Pb(:,k)=eval_spline(c,S.P_baseline,u(k),0);
    Po(:,k)=eval_spline(c,S.P_optimized,u(k),0);
    D=nom_fun(t(k)); Pn(:,k)=D(:,1);
end

figure('Name','Phase 7-17: trajectory shape comparison','Color','w');
hb=plot3(Pb(1,:),Pb(2,:),Pb(3,:),'--','LineWidth',1.2); hold on;
ho=plot3(Po(1,:),Po(2,:),Po(3,:),'LineWidth',2.0);
hn=plot3(Pn(1,:),Pn(2,:),Pn(3,:),':','LineWidth',1.4);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend([hb ho hn],{'Certified baseline','Unverified shape-QP candidate','Nominal'},'Location','best');
title(sprintf('Certified shape optimization: max nominal deviation %.2f -> %.2f m', ...
    S.baseline_max_nom_dev,S.optimized_max_nom_dev));

figure('Name','Phase 7-17: nominal deviation comparison','Color','w');
db=vecnorm(Pb-Pn,2,1); do=vecnorm(Po-Pn,2,1);
plot(t,db,'--','LineWidth',1.2); hold on;
plot(t,do,'LineWidth',1.8); grid on;
xlabel('t [s]'); ylabel('||p-p_{nom}|| [m]');
legend('Feasible baseline','Optimized certified','Location','best');
title('Nominal-deviation reduction inside the certified feasible set');

R=O.recursive;
figure('Name','Phase 7-17: recursive replanning','Color','w');
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for k=1:min(4,R.total)
    nexttile;
    q=R.records(k);
    plot3(Po(1,:),Po(2,:),Po(3,:),'LineWidth',1.2); hold on;
    if q.cert
        ur=linspace(0,1,350); Pr=zeros(3,numel(ur));
        for j=1:numel(ur), Pr(:,j)=eval_spline(q.cache,q.P,ur(j),0); end
        plot3(Pr(1,:),Pr(2,:),Pr(3,:),'LineWidth',2);
    end
    for io=1:numel(q.obstacles), draw_ellipsoid_local(q.obstacles(io)); end
    grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('replan %.0f%%: cert=%d',100*q.fraction,q.cert));
end
title(tl,'Recursive C6 replanning from the currently executing certified trajectory');
end


% =========================================================================
% Phase 7-16: failed-case boundary diagnosis + internal timing profile
% =========================================================================
function O=run_phase716_suite(sel,cfg,limits,obs,nom_fun,opt,trig,P715)
alpha=1/sqrt(3);
O=struct();

% ---- Failed-case boundary diagnosis --------------------------------------
failed=find(~[P715.static_records.cert]);
Nbase=sel.N; Tbase=sel.T;
Ntest=unique([Nbase 34 38 42]);
dTtest=[0 0.5 1.0 2.0 3.0];
frec=struct([]);

for ii=1:numel(failed)
    k=failed(ii);
    obcase=P715.static_cases{k};
    base=P715.static_records(k);

    % Test T-only first (same N), then N-only (same T), then N+T.
    recovered=false; rN=NaN; rT=NaN; rdT=NaN; rrbest=[]; cls='UNRECOVERED_TESTED_SET';

    for jt=2:numel(dTtest)
        rr=run_plan_case(cfg,Nbase,limits,obcase,nom_fun,trig.t_now,Tbase+dTtest(jt),opt,alpha);
        if rr.cert
            recovered=true; rN=Nbase; rT=Tbase+dTtest(jt); rdT=dTtest(jt); rrbest=rr;
            cls='RECOVERED_BY_T_ONLY'; break;
        end
    end

    if ~recovered
        for jn=2:numel(Ntest)
            rr=run_plan_case(cfg,Ntest(jn),limits,obcase,nom_fun,trig.t_now,Tbase,opt,alpha);
            if rr.cert
                recovered=true; rN=Ntest(jn); rT=Tbase; rdT=0; rrbest=rr;
                cls='RECOVERED_BY_N_ONLY'; break;
            end
        end
    end

    if ~recovered
        % Search smallest dT first, then smallest N.
        for jt=2:numel(dTtest)
            for jn=2:numel(Ntest)
                rr=run_plan_case(cfg,Ntest(jn),limits,obcase,nom_fun,trig.t_now,Tbase+dTtest(jt),opt,alpha);
                if rr.cert
                    recovered=true; rN=Ntest(jn); rT=Tbase+dTtest(jt); rdT=dTtest(jt); rrbest=rr;
                    cls='RECOVERED_BY_N_AND_T'; break;
                end
            end
            if recovered, break; end
        end
    end

    if recovered
        rrho=rrbest.rho; rG=rrbest.G; rD=rrbest.D; rGD=rrbest.GD;
    else
        rrho=inf; rG=0; rD=0; rGD=0;
    end

    x=struct('name',P715.static_names{k}, ...
        'base_G',base.G,'base_D',base.D,'base_GD',base.GD,'base_cert',base.cert, ...
        'base_rho',base.rho,'classification',cls,'recovered',recovered, ...
        'recovery_N',rN,'recovery_T',rT,'recovery_dT',rdT,'recovery_rho',rrho, ...
        'recovery_G',rG,'recovery_D',rD,'recovery_GD',rGD);
    if ii==1, frec=x; else, frec(ii)=x; end
end

O.failure_records=frec; O.n_failed=numel(failed);
if isempty(frec)
    O.n_T_only=0; O.n_N_only=0; O.n_NT=0; O.n_unrecovered=0;
else
    C={frec.classification};
    O.n_T_only=sum(strcmp(C,'RECOVERED_BY_T_ONLY'));
    O.n_N_only=sum(strcmp(C,'RECOVERED_BY_N_ONLY'));
    O.n_NT=sum(strcmp(C,'RECOVERED_BY_N_AND_T'));
    O.n_unrecovered=sum(strcmp(C,'UNRECOVERED_TESTED_SET'));
end

% ---- Internal timing profile --------------------------------------------
% Warm up first so JIT/first-call setup does not contaminate the distribution.
for k=1:3
    run_plan_case_profiled(cfg,sel.N,limits,obs,nom_fun,trig.t_now,sel.T,opt,alpha);
end

ntrial=50;
names={'cache','corridor','dynamic_matrix','geometry_feas','dynamics_feas', ...
       'combined_feas','apply','verifier','total'};
raw=struct();
for j=1:numel(names), raw.(names{j})=nan(ntrial,1); end
meta=[];
for k=1:ntrial
    [~,tp,mm]=run_plan_case_profiled(cfg,sel.N,limits,obs,nom_fun,trig.t_now,sel.T,opt,alpha);
    for j=1:numel(names), raw.(names{j})(k)=tp.(names{j}); end
    meta=mm;
end
prof=struct(); prof.n=ntrial; prof.samples=raw;
for j=1:numel(names)
    prof.(names{j})=stats_local(raw.(names{j}));
end
medtot=max(prof.total.median,eps);
prof.share.cache=100*prof.cache.median/medtot;
prof.share.corridor=100*prof.corridor.median/medtot;
prof.share.dynamic_matrix=100*prof.dynamic_matrix.median/medtot;
prof.share.qps=100*(prof.geometry_feas.median+prof.dynamics_feas.median+prof.combined_feas.median)/medtot;
prof.share.verifier=100*prof.verifier.median/medtot;
prof.problem=meta;
O.profile=prof;
end

function [rr,tp,meta]=run_plan_case_profiled(cfg0,N,limits,obs,nom_fun,t0,T,opt,alpha)
rr=struct('N',N,'T',T,'policy','','risk_fraction',NaN,'G',0,'D',0,'GD',0, ...
    'cert',0,'rho',inf,'z_inf',inf,'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
tp=struct('cache',0,'corridor',0,'dynamic_matrix',0,'geometry_feas',0, ...
    'dynamics_feas',0,'combined_feas',0,'apply',0,'verifier',0,'total',0);
meta=struct('nz',NaN,'nspans',NaN,'active_pairs',NaN,'G_rows',NaN,'D_rows',NaN);
if T<=0, rr.error='NONPOSITIVE_HORIZON'; return; end

Tall=tic;
try
    cfg=cfg0; cfg.N_ctrl=N;

    q=tic; c=build_static_cache(cfg,t0,t0+T,nom_fun,opt); tp.cache=toc(q)*1000;

    q=tic; [G,h,ci]=build_certified_active_corridor(c,cfg,obs,opt); tp.corridor=toc(q)*1000;

    q=tic; m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha); tp.dynamic_matrix=toc(q)*1000;

    q=tic; fg=feas_E(c,G,h,opt); tp.geometry_feas=toc(q)*1000;
    q=tic; fd=feas_E(c,D,d,opt); tp.dynamics_feas=toc(q)*1000;
    q=tic; fc=feas_E(c,[G;D],[h;d],opt); tp.combined_feas=toc(q)*1000;

    rr.G=fg.flag>0; rr.D=fd.flag>0; rr.GD=fc.flag>0; rr.rho=fc.rho; rr.z_inf=fc.z_inf;

    meta.nz=c.nz; meta.nspans=c.nspans; meta.active_pairs=ci.n_active_pairs;
    meta.G_rows=size(G,1); meta.D_rows=size(D,1);

    if fc.flag>0 && ~isempty(fc.z)
        q=tic; P=apply_z(c,fc.z); tp.apply=toc(q)*1000;
        q=tic; cert=certify_candidate_global(P,c,cfg,limits,obs,opt); tp.verifier=toc(q)*1000;
        rr.cert=cert.pass; rr.min_g=cert.min_g_lower;
        rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;
    end
catch ME
    rr.error=ME.message;
end
tp.total=toc(Tall)*1000;
end

function S=stats_local(x)
x=x(isfinite(x));
if isempty(x)
    S=struct('median',NaN,'p95',NaN,'p99',NaN,'max',NaN); return;
end
S=struct('median',median(x),'p95',percentile_local(x,95), ...
    'p99',percentile_local(x,99),'max',max(x));
end

function make_phase716_figures(O)
P=O.profile;
figure('Name','Phase 7-16: internal timing profile','Color','w');
labels={'Cache','Corridor','Dyn matrix','G feas','D feas','G+D feas','Apply','Verifier'};
med=[P.cache.median P.corridor.median P.dynamic_matrix.median P.geometry_feas.median ...
     P.dynamics_feas.median P.combined_feas.median P.apply.median P.verifier.median];
bar(med); grid on; ylabel('Median time [ms]');
set(gca,'XTick',1:numel(labels),'XTickLabel',labels,'XTickLabelRotation',30);
title(sprintf('Internal timing breakdown, total median %.2f ms',P.total.median));

figure('Name','Phase 7-16: timing distribution','Color','w');
histogram(P.samples.total); grid on;
xlabel('Profiled online path [ms]'); ylabel('Count');
title(sprintf('median %.1f / P95 %.1f / P99 %.1f / max %.1f ms', ...
    P.total.median,P.total.p95,P.total.p99,P.total.max));

if O.n_failed>0
    figure('Name','Phase 7-16: failed-case recovery','Color','w');
    vals=zeros(O.n_failed,1);
    for k=1:O.n_failed
        c=O.failure_records(k).classification;
        if strcmp(c,'RECOVERED_BY_T_ONLY'), vals(k)=1;
        elseif strcmp(c,'RECOVERED_BY_N_ONLY'), vals(k)=2;
        elseif strcmp(c,'RECOVERED_BY_N_AND_T'), vals(k)=3;
        else, vals(k)=4;
        end
    end
    bar(vals); ylim([0 4.5]); grid on;
    set(gca,'YTick',1:4,'YTickLabel',{'T only','N only','N+T','Unrecovered'});
    xlabel('Failed structured case'); ylabel('Recovery class');
    title('Why the original fixed N,T test failed');
end
end


% =========================================================================
% Phase 7-15 structured validation suite
% =========================================================================
function O=run_phase715_suite(sel,cfg,limits,obs,nom_fun,opt,trig,risk)
alpha=1/sqrt(3);
O=struct();

% -------------------------------------------------------------------------
% H1. Structured obstacle stress matrix.
% Each case changes ONE interpretable geometric feature where possible.
% A failed case is recorded, not silently discarded.
% -------------------------------------------------------------------------
cases={}; names={};
add=@(name,oo) assignin_local(name,oo); %#ok<NASGU>

% lateral / longitudinal / vertical translations
shiftset={[0;-1;0],[0;-0.5;0],[0;0.5;0],[0;1;0], ...
          [-0.75;0;0],[0.75;0;0],[0;0;-0.75],[0;0;0.75]};
for k=1:numel(shiftset)
    oo=obs; oo(1).center=obs(1).center+shiftset{k};
    cases{end+1}=oo; names{end+1}=sprintf('shift_%02d',k); %#ok<AGROW>
end

% size / aspect-ratio changes
scales=[0.75 0.9 1.1 1.25];
for q=scales
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).radii=obs(1).radii*q;
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
    end
    cases{end+1}=oo; names{end+1}=sprintf('scale_%.2f',q); %#ok<AGROW>
end
aspect={[1.6;0.7;0.7],[0.7;1.6;0.7],[0.7;0.7;1.6]};
for k=1:numel(aspect)
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).radii=obs(1).radii.*aspect{k};
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
    end
    cases{end+1}=oo; names{end+1}=sprintf('aspect_%d',k); %#ok<AGROW>
end

% attitude changes
angles=[-pi/3 -pi/6 pi/6 pi/3];
for q=angles
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).R=eul2rotm_local([pi/7+q,pi/10-q/2,pi/8+q/3]);
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
    end
    cases{end+1}=oo; names{end+1}=sprintf('att_%+.2f',q); %#ok<AGROW>
end

rec=struct([]);
for k=1:numel(cases)
    tic;
    rr=run_plan_case(cfg,sel.N,limits,cases{k},nom_fun,trig.t_now,sel.T,opt,alpha);
    ms=toc*1000;
    x=struct('name',names{k},'cert',rr.cert,'G',rr.G,'D',rr.D,'GD',rr.GD, ...
        'rho',rr.rho,'min_g',rr.min_g,'ms',ms);
    if k==1, rec=x; else, rec(k)=x; end
end
O.static_records=rec;
O.static_cases=cases;
O.static_names=names;
O.static_total=numel(rec); O.static_certified=sum([rec.cert]);

% -------------------------------------------------------------------------
% H2. Diverse multiple-obstacle layouts: 2, 3, 4, 6 obstacles.
% Separate ellipsoids are retained; no enclosing system/obstacle mega-sphere.
% -------------------------------------------------------------------------
counts=[2 3 4 6];
layouts=cell(size(counts));
mrec=struct([]);
for ic=1:numel(counts)
    m=counts(ic); oo=obs;
    for j=2:m
        ob2=obs(1);
        side=(-1)^j;
        ob2.center=obs(1).center + [0.55*(j-1); side*(1.1+0.35*mod(j,2)); 0.35*(j-1)];
        if isfield(ob2,'radii')
            ob2.radii=obs(1).radii.*[0.70+0.04*j;0.62+0.03*j;0.72];
            ob2.R=eul2rotm_local([pi/7+0.13*j,pi/10-0.09*j,pi/8+0.07*j]);
            ob2.S=ob2.R*diag(ob2.radii.^2)*ob2.R';
        end
        oo(j)=ob2;
    end
    layouts{ic}=oo;
    tic; rr=run_plan_case(cfg,sel.N,limits,oo,nom_fun,trig.t_now,sel.T,opt,alpha); ms=toc*1000;
    x=struct('nobs',m,'cert',rr.cert,'G',rr.G,'D',rr.D,'GD',rr.GD, ...
        'rho',rr.rho,'min_g',rr.min_g,'ms',ms);
    if ic==1, mrec=x; else, mrec(ic)=x; end
end
O.multi_records=mrec; O.multi_total=numel(mrec); O.multi_certified=sum([mrec.cert]);

% -------------------------------------------------------------------------
% H3. Replan-during-avoidance splice test.
% This validates the C6 splice mathematics against the CURRENT certified
% trajectory.  A new obstacle is introduced at several replanning times.
% Full successful replanning requires a generator accepting current-trajectory
% boundary data; if the inherited run_plan_case is nominal-boundary-only, the
% test explicitly records that limitation instead of claiming success.
% -------------------------------------------------------------------------
fractions=[0.20 0.35 0.50 0.65];
rrec=struct([]);
P=sel.splice.P; c=sel.splice.cache;
for k=1:numel(fractions)
    tr=trig.t_now+fractions(k)*sel.T;
    ur=(tr-trig.t_now)/sel.T;
    boundary=zeros(3,7);
    for r=0:6, boundary(:,r+1)=eval_spline(c,P,ur,r); end

    % Numerically re-evaluate the same current certified trajectory at splice.
    berr=zeros(7,1);
    for r=0:6
        q=eval_spline(c,P,ur,r);
        berr(r+1)=norm(q-boundary(:,r+1),inf);
    end
    c6ok=max(berr)<=opt.c6_tol;

    % New obstacle placed ahead of current payload position, alternating side.
    obnew=obs(1);
    pnow=boundary(:,1);
    obnew.center=pnow+[0.4;(-1)^k*0.9;1.2];
    if isfield(obnew,'radii')
        obnew.radii=obs(1).radii.*[0.75;0.75;0.75];
        obnew.R=eul2rotm_local([0.2*k,-0.12*k,0.1*k]);
        obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
    end
    oo=[obs obnew];

    % Current inherited generator cannot impose arbitrary current-trajectory
    % 0..6 boundary values.  Therefore do NOT mislabel nominal-boundary solve
    % as a recursive-replan proof.  We only check whether a nominal-boundary
    % candidate exists and flag generator_boundary_supported=false.
    Tleft=max(2.0,sel.T-(tr-trig.t_now));
    tic; rr=run_plan_case(cfg,sel.N,limits,oo,nom_fun,tr,Tleft,opt,alpha); ms=toc*1000;
    x=struct('fraction',fractions(k),'t_replan',tr,'c6_boundary_error',max(berr), ...
        'c6_boundary_pass',c6ok,'candidate_cert',rr.cert,'candidate_ms',ms, ...
        'generator_boundary_supported',false);
    if k==1, rrec=x; else, rrec(k)=x; end
end
O.replan_records=rrec; O.replan_total=numel(rrec);
O.replan_c6_pass=sum([rrec.c6_boundary_pass]);
% Deliberately zero until arbitrary current-trajectory boundary conditions are
% implemented. This prevents a false recursive-replanning claim.
O.replan_certified=sum([rrec.candidate_cert] & [rrec.generator_boundary_supported]);

% -------------------------------------------------------------------------
% H4. Timing distribution for one ONLINE candidate path.
% No plots, fprintf, stress sweep, or coarse/fine horizon search inside timer.
% -------------------------------------------------------------------------
ntrial=50;
tm=zeros(ntrial,1);
for k=1:ntrial
    tic;
    run_plan_case(cfg,sel.N,limits,obs,nom_fun,trig.t_now,sel.T,opt,alpha);
    tm(k)=toc*1000;
end
O.timing.samples_ms=tm; O.timing.n=ntrial;
O.timing.median=median(tm); O.timing.p95=percentile_local(tm,95);
O.timing.p99=percentile_local(tm,99); O.timing.max=max(tm);

% Categorize failures for interpretation.
O.failure_static_geometry=sum(~[rec.G]);
O.failure_static_combined=sum([rec.G] & ~[rec.GD]);
O.failure_static_certificate=sum([rec.GD] & ~[rec.cert]);
end

function make_phase715_figures(O)
figure('Name','Phase 7-15: structured stress','Color','w');
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
bar(double([O.static_records.cert])); ylim([0 1.2]); grid on;
xlabel('Structured obstacle case'); ylabel('Certified'); title('Static stress matrix');
nexttile;
bar([O.multi_records.nobs],[O.multi_records.cert]); ylim([0 1.2]); grid on;
xlabel('Number of obstacles'); ylabel('Certified'); title('Multiple-obstacle layouts');
nexttile;
plot([O.multi_records.nobs],[O.multi_records.ms],'-o','LineWidth',1.5); grid on;
xlabel('Number of obstacles'); ylabel('Diagnostic solve time [ms]');
title('Obstacle-count timing');
nexttile;
histogram(O.timing.samples_ms); grid on;
xlabel('Single candidate online-path time [ms]'); ylabel('Count');
title(sprintf('Timing: med %.1f / P99 %.1f / max %.1f ms', ...
    O.timing.median,O.timing.p99,O.timing.max));
title(tl,'Phase 7-15 stress and timing validation');

figure('Name','Phase 7-15: failure classification','Color','w');
bar([O.failure_static_geometry,O.failure_static_combined,O.failure_static_certificate]);
set(gca,'XTickLabel',{'Geometry','G+D intersection','Independent cert'});
ylabel('Failed cases'); grid on; title('Why structured cases fail');

figure('Name','Phase 7-15: recursive splice diagnostic','Color','w');
plot([O.replan_records.t_replan],[O.replan_records.c6_boundary_error],'-o','LineWidth',1.5);
yline(1e-7,'--'); set(gca,'YScale','log'); grid on;
xlabel('Replan time [s]'); ylabel('C6 boundary error');
title('C6 state available for recursive replanning');
end

function y=percentile_local(x,p)
x=sort(x(:)); n=numel(x);
if n==1, y=x; return; end
q=1+(n-1)*p/100; i=floor(q); f=q-i;
if i>=n, y=x(end); else, y=x(i)*(1-f)+x(i+1)*f; end
end

function z=assignin_local(name,oo) %#ok<INUSD>
z=[];
end


% =========================================================================
% Phase 7-14 integrated validation / visualization
% =========================================================================
function out=evaluate_selected_integrated(sel,cfg,limits,obs,nom_fun,opt,trig)
P=sel.splice.P; c=sel.splice.cache;
tt=linspace(trig.t_now,trig.t_now+sel.T,801);
uu=(tt-trig.t_now)/sel.T;
Q=cell(7,1);
for r=0:6
    Q{r+1}=zeros(3,numel(uu));
    for k=1:numel(uu), Q{r+1}(:,k)=eval_spline(c,P,uu(k),r); end
end
finite_all=all(cellfun(@(x)all(isfinite(x(:))),Q));
% C6 is structural inside a degree-7 spline with simple internal knots; splice
% equality was independently checked at both endpoints.
out.c6_all_pass=sel.splice.pass;
out.derivative_sample_pass=finite_all;

vnorm=sqrt(sum(Q{2}.^2,1)); anorm=sqrt(sum(Q{3}.^2,1)); jnorm=sqrt(sum(Q{4}.^2,1));
out.tracking_pass=max(vnorm)<=limits.v_max+opt.cert_tol && ...
    max(anorm)<=limits.a_max+opt.cert_tol && max(jnorm)<=limits.j_max+opt.cert_tol;
out.safety_pass=sel.plan.cert;

% Translational feed-forward force/thrust-direction proxy only.
W=Q{3}+cfg.g_acc*cfg.e3;
Fmag=sqrt(sum(W.^2,1));
nT=W./max(Fmag,eps);
out.input_proxy_pass=all(isfinite(Fmag)) && all(isfinite(nT(:)));
out.t=tt; out.Q=Q; out.vnorm=vnorm; out.anorm=anorm; out.jnorm=jnorm;
out.Fmag=Fmag; out.nT=nT;

% Deterministic robustness sweep: obstacle location, size, and attitude.
cases={};
dy=[-1 -0.5 0.5 1.0];
sc=[0.8 0.9 1.1 1.2];
ang=[-pi/6 pi/6];
for x=dy
    oo=obs; oo(1).center=oo(1).center+[0;x;0]; cases{end+1}=oo; %#ok<AGROW>
end
for x=sc
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).radii=obs(1).radii*x;
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
        cases{end+1}=oo; %#ok<AGROW>
    end
end
for x=ang
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).R=eul2rotm_local([pi/7+x,pi/10,pi/8]);
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
        cases{end+1}=oo; %#ok<AGROW>
    end
end
pass=false(1,numel(cases)); rho=inf(1,numel(cases));
for i=1:numel(cases)
    rr=run_plan_case(cfg,sel.N,limits,cases{i},nom_fun,trig.t_now,sel.T,opt,1/sqrt(3));
    pass(i)=rr.cert; rho(i)=rr.rho;
end
out.robust_total=numel(cases); out.robust_certified=sum(pass);
out.robust_pass=pass; out.robust_rho=rho;

% Multiple-obstacle stress test. These are intentionally separate convex
% ellipsoids; no enclosing "whole-system sphere" is introduced.
multi=cell(1,3);
for m=1:3
    oo=obs;
    for j=2:m
        ob2=obs(1);
        ob2.center=obs(1).center+[0.35*(j-1); (-1)^j*1.8; 0.55*(j-1)];
        if isfield(ob2,'radii')
            ob2.radii=obs(1).radii.*[0.75;0.70;0.80];
            ob2.R=eul2rotm_local([pi/7+0.18*j,pi/10-0.12*j,pi/8]);
            ob2.S=ob2.R*diag(ob2.radii.^2)*ob2.R';
        end
        oo(j)=ob2;
    end
    multi{m}=oo;
end
mpass=false(1,3); mtime=nan(1,3);
for m=1:3
    tic;
    rr=run_plan_case(cfg,sel.N,limits,multi{m},nom_fun,trig.t_now,sel.T,opt,1/sqrt(3));
    mtime(m)=toc*1000; mpass(m)=rr.cert;
end
out.multi_total=3; out.multi_certified=sum(mpass);
out.multi_pass=mpass; out.multi_ms=mtime;
end

function make_phase714_figures(sel,cfg,limits,obs,nom_fun,opt,trig,risk,I)
P=sel.splice.P; c=sel.splice.cache; t=I.t; Q=I.Q;

% Figure 1: 3-D geometry and trajectory.
figure('Name','Phase 7-14: 3D trajectory','Color','w');
hAvoid=plot3(Q{1}(1,:),Q{1}(2,:),Q{1}(3,:),'LineWidth',2); hold on;
Pn=zeros(3,numel(t));
for k=1:numel(t), D=nom_fun(t(k)); Pn(:,k)=D(:,1); end
hNom=plot3(Pn(1,:),Pn(2,:),Pn(3,:),'--','LineWidth',1.5);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
hStart=plot3(Q{1}(1,1),Q{1}(2,1),Q{1}(3,1),'o','MarkerSize',8,'LineWidth',1.5);
hEnd=plot3(Q{1}(1,end),Q{1}(2,end),Q{1}(3,end),'s','MarkerSize',8,'LineWidth',1.5);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend([hAvoid hNom hStart hEnd],{'Certified avoidance','Nominal','Start','C6 rejoin'},'Location','best');
title(sprintf('Certified trajectory: N=%d, T=%.3f s',sel.N,sel.T));

% Figure 2: safety + dynamics.
figure('Name','Phase 7-14: safety and tracking','Color','w');
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
g=zeros(1,numel(t));
for k=1:numel(t)
    a=Q{3}(:,k); W=a+cfg.g_acc*cfg.e3; nT=W/norm(W);
    gg=inf(1,numel(obs));
    for io=1:numel(obs), gg(io)=best_nominal_point_g(Q{1}(:,k),nT,obs(io),cfg,opt); end
    g(k)=min(gg);
end
nexttile; plot(t,g,'LineWidth',1.5); yline(0,'--'); grid on;
xlabel('t [s]'); ylabel('g sample [m]'); title('Sampled safety margin (certificate is separate)');
nexttile; plot(t,I.vnorm,'LineWidth',1.5); hold on; yline(limits.v_max,'--'); grid on;
xlabel('t [s]'); ylabel('|v| [m/s]'); title('Velocity');
nexttile; plot(t,I.anorm,'LineWidth',1.5); hold on; yline(limits.a_max,'--'); grid on;
xlabel('t [s]'); ylabel('|a| [m/s^2]'); title('Acceleration');
nexttile; plot(t,I.jnorm,'LineWidth',1.5); hold on; yline(limits.j_max,'--'); grid on;
xlabel('t [s]'); ylabel('|j| [m/s^3]'); title('Jerk');
title(tl,'Safety and tracking diagnostics');

% Figure 3: derivatives 0..6 and feed-forward proxy.
figure('Name','Phase 7-14: C6 derivatives and input proxy','Color','w');
tl=tiledlayout(4,2,'TileSpacing','compact','Padding','compact');
for r=0:6
    nexttile; plot(t,Q{r+1}','LineWidth',1.0); grid on;
    xlabel('t [s]'); ylabel(sprintf('p^{(%d)}',r)); title(sprintf('Derivative order %d',r));
end
nexttile; plot(t,I.Fmag,'LineWidth',1.5); grid on;
xlabel('t [s]'); ylabel('|a+ge_3|'); title('Feed-forward thrust/force proxy');
title(tl,'C6 trajectory and translational input proxy');

% Figure 4: robustness and obstacle-count timing.
figure('Name','Phase 7-14: robustness and timing','Color','w');
tl=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; bar(1:I.robust_total,double(I.robust_pass)); ylim([0 1.2]); grid on;
xlabel('Stress case'); ylabel('Certified'); title('Pose / size / location robustness');
nexttile; plot(1:I.multi_total,I.multi_ms,'-o','LineWidth',1.5); grid on;
xlabel('Number of obstacles'); ylabel('Diagnostic time [ms]');
title('Obstacle-count scaling (not WCET)');
title(tl,'Robustness validation');

fprintf('[G: FIGURES]\n');
fprintf('Generated 4 figures. Plot time is excluded from all planner timing.\n');
fprintf('Risk window: first=%.3f, worst=%.3f, clear=%.3f s\n', ...
    risk.t_first_bad,risk.t_worst,risk.t_clear);
end

function draw_ellipsoid_local(ob)
if ~isfield(ob,'radii') || ~isfield(ob,'R'), return; end
[x,y,z]=sphere(24);
X=[x(:)';y(:)';z(:)'];
Y=ob.R*diag(ob.radii)*X+ob.center;
surf(reshape(Y(1,:),size(x)),reshape(Y(2,:),size(y)),reshape(Y(3,:),size(z)), ...
    'FaceAlpha',0.18,'EdgeAlpha',0.12,'HandleVisibility','off');
end


% =========================================================================
% Phase 7-13 wrapper: generate exactly as the validated both-end-C6 generator,
% then independently audit both splice points r=0..6.
% =========================================================================
function [rr,splice]=run_plan_case_with_splice_audit(cfg0,N,limits,obs,nom_fun,t0,T,opt,alpha)
rr=struct('N',N,'T',T,'policy','','risk_fraction',NaN,'G',0,'D',0,'GD',0, ...
    'cert',0,'rho',inf,'z_inf',inf,'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
splice=struct('pass',false,'start_max',inf,'end_max',inf,'start_by_order',inf(7,1),'end_by_order',inf(7,1),'P',[],'cache',[]);
if T<=0, rr.error='NONPOSITIVE_HORIZON'; return; end
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
    fg=feas_E(c,G,h,opt); fd=feas_E(c,D,d,opt); fc=feas_E(c,[G;D],[h;d],opt);
    rr.G=fg.flag>0; rr.D=fd.flag>0; rr.GD=fc.flag>0; rr.rho=fc.rho; rr.z_inf=fc.z_inf;
    if fc.flag>0 && ~isempty(fc.z)
        P=apply_z(c,fc.z);
        cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
        rr.cert=cert.pass; rr.min_g=cert.min_g_lower;
        rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;
        D0=nom_fun(t0); D1=nom_fun(t0+T);
        for r=0:6
            qs=eval_spline(c,P,0,r); qe=eval_spline(c,P,1,r);
            splice.start_by_order(r+1)=norm(qs-D0(:,r+1),inf);
            splice.end_by_order(r+1)=norm(qe-D1(:,r+1),inf);
        end
        splice.start_max=max(splice.start_by_order);
        splice.end_max=max(splice.end_by_order);
        splice.pass=(splice.start_max<=opt.c6_tol)&&(splice.end_max<=opt.c6_tol);
        splice.P=P; splice.cache=c;
        rr.cert=rr.cert && splice.pass;
    end
catch ME
    rr.error=ME.message;
end
end

function validate_phase712e_inputs(cfg,limits,obs,opt)
req_cfg={'p','N_ctrl','g_acc','e3','sys','d_margin'};
for k=1:numel(req_cfg)
    assert(isfield(cfg,req_cfg{k}), 'Phase7-12E:MissingCfgField', ...
        'cfg.%s is required.',req_cfg{k});
end
req_sys={'L_cable','R_payload','R_cable','R_uav'};
for k=1:numel(req_sys)
    assert(isfield(cfg.sys,req_sys{k}), 'Phase7-12E:MissingSysField', ...
        'cfg.sys.%s is required.',req_sys{k});
end
req_lim={'v_max','a_max','j_max'};
for k=1:numel(req_lim)
    assert(isfield(limits,req_lim{k}), 'Phase7-12E:MissingLimitField', ...
        'limits.%s is required.',req_lim{k});
end
req_opt={'box_z_max','guide_extra','active_margin_trigger','separator_grid_N', ...
         'cert_tol','c6_tol','a_net_min'};
for k=1:numel(req_opt)
    assert(isfield(opt,req_opt{k}), 'Phase7-12E:MissingOptField', ...
        'opt.%s is required.',req_opt{k});
end
assert(~isempty(obs),'Phase7-12E:NoObstacle','At least one obstacle is required.');
for i=1:numel(obs)
    assert(isfield(obs(i),'center'),'Phase7-12E:MissingObstacleCenter', ...
        'obs(%d).center is required.',i);
    assert(isfield(obs(i),'S') || isfield(obs(i),'support_fun'), ...
        'Phase7-12E:MissingObstacleSupport', ...
        'obs(%d) requires S or support_fun.',i);
end
end

% =========================================================================
% Trigger reconstruction using the same research-level whole-system
% fixed-normal certificate geometry used by the audit pipeline.
% =========================================================================
function trig=reconstruct_first_trigger(nom_fun,obs,cfg,opt,tmin,tmax,Tlook,dt)
trig=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN, ...
    'g_risk',inf,'obs_idx',NaN);
% Search at control-cycle resolution. At each t_now, scan future from near to far.
times=tmin:dt:tmax;
for it=1:numel(times)
    tn=times(it);
    tf=min(tn+Tlook,tmax);
    if tf<=tn+dt, continue; end
    future=tn+dt:dt:tf;
    hit=false;
    for k=1:numel(future)
        t=future(k);
        D=nom_fun(t);
        p=D(:,1); a=D(:,3);
        W=a+cfg.g_acc*cfg.e3;
        M=norm(W);
        if M<=1e-9, continue; end
        nT=W/M;
        dirs=fibonacci_sphere(opt.separator_grid_N);
        if size(dirs,1)==3
            dirs3=dirs;
        elseif size(dirs,2)==3
            dirs3=dirs';
        else
            error('fibonacci_sphere must return 3xN or Nx3 directions.');
        end
        for io=1:numel(obs)
            best=-inf;
            for id=1:size(dirs3,2)
                n=dirs3(:,id);
                Delta=relative_system_support_minus(n,nT,cfg);
                hE=ellipsoid_support(n,obs(io));
                g=n'*p-Delta-hE-cfg.d_margin;
                if g>best, best=g; end
            end
            if best<0
                trig.found=true; trig.t_now=tn; trig.t_risk=t;
                trig.TTC=t-tn; trig.g_risk=best; trig.obs_idx=io;
                hit=true; break;
            end
        end
        if hit, break; end
    end
    if hit, return; end
end
end

function tl=build_timeline_diagnostics(nom_fun,obs,cfg,opt,trig,tmin,Tlook)
% Approximate the production 15 m sensing timeline using obstacle surface
% distance from the nominal UAV position. The trigger itself remains the
% audit whole-system future-collision trigger, so both times are reported
% separately rather than treated as identical.
dt=0.025;
sensor_radius=15.0;
ts=tmin:dt:trig.t_now;
t_sensor=NaN; range_sensor=inf;
for k=1:numel(ts)
    D=nom_fun(ts(k));
    pL=D(:,1); aL=D(:,3);
    W=aL+cfg.g_acc*cfg.e3;
    nT=W/norm(W);
    pQ=pL+cfg.sys.L_cable*nT;
    rr=ellipsoid_surface_distance_approx(pQ,obs(trig.obs_idx));
    if rr<=sensor_radius
        t_sensor=ts(k); range_sensor=rr; break;
    end
end
D=nom_fun(trig.t_now); pL=D(:,1); aL=D(:,3);
nT=(aL+cfg.g_acc*cfg.e3)/norm(aL+cfg.g_acc*cfg.e3);
range_trigger=ellipsoid_surface_distance_approx(pL+cfg.sys.L_cable*nT,obs(trig.obs_idx));
g_now=best_nominal_point_g(pL,nT,obs(trig.obs_idx),cfg,opt);

D=nom_fun(trig.t_risk); pLr=D(:,1); aLr=D(:,3);
nTr=(aLr+cfg.g_acc*cfg.e3)/norm(aLr+cfg.g_acc*cfg.e3);
range_risk=ellipsoid_surface_distance_approx(pLr+cfg.sys.L_cable*nTr,obs(trig.obs_idx));
g_risk=best_nominal_point_g(pLr,nTr,obs(trig.obs_idx),cfg,opt);

tl=struct();
tl.t_sensor=t_sensor; tl.range_sensor=range_sensor;
tl.range_trigger=range_trigger; tl.range_risk=range_risk;
tl.g_now=g_now; tl.g_risk=g_risk;
if isnan(t_sensor)
    tl.sensor_to_trigger=NaN; tl.sensor_to_risk=NaN;
else
    tl.sensor_to_trigger=trig.t_now-t_sensor;
    tl.sensor_to_risk=trig.t_risk-t_sensor;
end
tl.Tlook=Tlook;
end

function g=best_nominal_point_g(p,nT,o,cfg,opt)
dirs=fibonacci_sphere(opt.separator_grid_N);
if size(dirs,1)==3, dirs3=dirs; else, dirs3=dirs'; end
g=-inf;
for k=1:size(dirs3,2)
    n=dirs3(:,k);
    val=n'*p-relative_system_support_minus(n,nT,cfg)-ellipsoid_support(n,o)-cfg.d_margin;
    g=max(g,val);
end
end

function d=ellipsoid_surface_distance_approx(p,o)
% Fast radial surface-distance diagnostic only (not a certificate).
% Exact production code may use its iterative point-to-ellipsoid routine.
p=p(:); c=o.center(:);
v=p-c; nv=norm(v);
if nv<1e-12, d=-min(sqrt(eig(o.S))); return; end
u=v/nv;
rad=sqrt(max(0,u'*o.S*u));
d=nv-rad;
end

function policies=make_policies(TTC)
names={'FIXED_5','TTC','TTC_PLUS_1','TTC_PLUS_2','FIXED_7_5','FIXED_10'};
vals=[5,max(TTC,0.5),max(TTC+1,0.5),max(TTC+2,0.5),7.5,10];
for k=1:numel(names)
    policies(k)=struct('name',names{k},'T',vals(k)); %#ok<AGROW>
end
end

function rr=run_plan_case(cfg0,N,limits,obs,nom_fun,t0,T,opt,alpha)
rr=struct('N',N,'T',T,'policy','','risk_fraction',NaN,'G',0,'D',0,'GD',0, ...
    'cert',0,'rho',inf,'z_inf',inf,'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
if T<=0, rr.error='NONPOSITIVE_HORIZON'; return; end
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c);
    [D,d]=dyn_E(m,limits,alpha);
    fg=feas_E(c,G,h,opt); fd=feas_E(c,D,d,opt); fc=feas_E(c,[G;D],[h;d],opt);
    rr.G=fg.flag>0; rr.D=fd.flag>0; rr.GD=fc.flag>0; rr.rho=fc.rho; rr.z_inf=fc.z_inf;
    if fc.flag>0 && ~isempty(fc.z)
        P=apply_z(c,fc.z);
        cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
        rr.cert=cert.pass;
        rr.min_g=cert.min_g_lower;
        rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;
    end
catch ME
    rr.error=ME.message;
end
end

function print_plan(r)
fprintf('N=%d T=%.3f | G=%d D=%d G+D=%d cert=%d rho*=%.3e zinf=%.3f min_g=%+.4f v/a/j=%.3f/%.3f/%.3f\n', ...
    r.N,r.T,r.G,r.D,r.GD,r.cert,r.rho,r.z_inf,r.min_g,r.vUB,r.aUB,r.jUB);
if ~isempty(r.error), fprintf('error: %s\n',r.error); end
end

function m=build_dyn_E(cache)
P0=cache.Pbase; [A0,V0,J0]=derivative_control_polygons(P0,cache);
nz=cache.nz; VM=zeros(numel(V0),nz); AM=zeros(numel(A0),nz); JM=zeros(numel(J0),nz);
for q=1:nz
    e=zeros(nz,1); e(q)=1; Pq=apply_z(cache,e);
    [Aq,Vq,Jq]=derivative_control_polygons(Pq,cache);
    VM(:,q)=Vq(:)-V0(:); AM(:,q)=Aq(:)-A0(:); JM(:,q)=Jq(:)-J0(:);
end
m.VM=VM; m.V0=V0(:); m.AM=AM; m.A0=A0(:); m.JM=JM; m.J0=J0(:);
end

function [A,b]=dyn_E(m,lim,alpha)
[A1,b1]=two_E(m.VM,m.V0,alpha*lim.v_max);
[A2,b2]=two_E(m.AM,m.A0,alpha*lim.a_max);
[A3,b3]=two_E(m.JM,m.J0,alpha*lim.j_max);
A=[A1;A2;A3]; b=[b1;b2;b3];
end
function [A,b]=two_E(M,d0,L), A=[M;-M]; b=[L-d0;L+d0]; end

function r=feas_E(cache,A,b,opt)
r=struct('flag',-99,'rho',inf,'z_inf',inf,'z',[]);
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',1500);
H=1e-10*eye(cache.nz); f=zeros(cache.nz,1);
try
    [z,~,r.flag]=quadprog(H,f,A,b,[],[],lb,ub,[],qo);
    if r.flag>0, r.z=z; r.z_inf=norm(z,inf); end
catch
end
Hs=1e-12*eye(cache.nz+1); fs=[zeros(cache.nz,1);1];
try
    [xs,~,fl]=quadprog(Hs,fs,[A,-ones(size(A,1),1)],b,[],[],[lb;0],[ub;inf],[],qo);
    if fl>0, r.rho=max(0,xs(end)); end
catch
end
end

function d=relative_system_support_minus(n,nT,cfg)
% lower-support loss in direction n: Delta_h(-n,nT)
n=n(:); nT=nT(:);
q=(-n)'*nT;
vL=cfg.sys.R_payload;
vQ=cfg.sys.L_cable*q+cfg.sys.R_uav;
vC=max(0,cfg.sys.L_cable*q)+cfg.sys.R_cable;
d=max([vL,vQ,vC]);
end

function h=ellipsoid_support(n,o)
% obstacle support h_E(n) = n'c + sqrt(n'Sn)
n=n(:); c=o.center(:);
h=n'*c+sqrt(max(0,n'*o.S*n));
end


% =========================================================================
% OPTIONS
% =========================================================================
function opt = setup_options()
opt.time_budget_typical_ms = 1.0;
opt.time_budget_worst_ms   = 2.0;

% Generator
opt.qp_reg       = 1e-8;
opt.w_position   = 1.0;
opt.w_jerk       = 2e-3;
opt.w_snap       = 2e-4;
opt.box_z_max    = 8.0;
opt.guide_extra  = 0.02;
opt.active_margin_trigger = 0.35; % generator heuristic only; verifier remains global
opt.separator_grid_N      = 162;  % candidate normal search, not a safety proof

% Corridor/certificate
opt.cert_tol     = 1e-10;
opt.c6_tol       = 1e-7;
opt.a_net_min    = 1e-5;

% Small fixed number of objective samples; matrices are cached.
opt.N_obj = 17;

% quadprog settings.  "active-set" accepts x0 and is useful for small dense
% online QPs.  For deployment, replace this call with a generated/warm-start
% QP solver if profiling shows quadprog dominates the 2 ms budget.
opt.qp_options = optimoptions('quadprog', ...
    'Display','off', ...
    'Algorithm','active-set', ...
    'ConstraintTolerance',1e-8, ...
    'OptimalityTolerance',1e-8, ...
    'MaxIterations',40);
end

% =========================================================================
% STATIC CACHE
% =========================================================================
function cache = build_static_cache(cfg, t0, t1, nom_fun, opt)
p = cfg.p;
N = cfg.N_ctrl;
T = t1-t0;

if p ~= 7
    error('This Phase-7 implementation assumes p=7 for C6 endpoint matching.');
end
if N <= 2*p
    error('N_ctrl must be > 2*p.');
end

knots = clamped_uniform_knots(N,p);
free_idx = (p+1):(N-p);
nf = numel(free_idx);
nz = 3*nf;

D0 = nom_fun(t0);
D1 = nom_fun(t1);
Pfixed = solve_endpoint_fixed(p,N,knots,T,D0,D1);

% Fit free baseline CPs to nominal trajectory.
u_fit = linspace(0,1,max(80,5*N))';
A = zeros(numel(u_fit),nf);
Y = zeros(numel(u_fit),3);
for k=1:numel(u_fit)
    B = basis_derivative_all(u_fit(k),knots,p,N,0);
    D = nom_fun(t0+u_fit(k)*T);
    A(k,:) = B(free_idx);
    Y(k,:) = D(:,1)' - B*Pfixed;
end
Pbase = Pfixed;
Pbase(free_idx,:) = A\Y;

cache = struct();
cache.p = p;
cache.N = N;
cache.T = T;
cache.t0 = t0;
cache.t1 = t1;
cache.knots = knots;
cache.free_idx = free_idx;
cache.nf = nf;
cache.nz = nz;
cache.Pbase = Pbase;

% Nonzero knot spans.
uk = unique(knots);
cache.spans = [uk(1:end-1)', uk(2:end)'];
cache.spans = cache.spans(cache.spans(:,2)>cache.spans(:,1),:);
cache.nspans = size(cache.spans,1);

% Exact polynomial interpolation matrix used only during cache construction.
xi = linspace(0,1,p+1)';
Bern = zeros(p+1);
for j=1:p+1
    Bern(j,:) = bernstein_row(p,xi(j));
end
cache.Bern_inv = inv(Bern);

% Precompute affine Bezier maps for every position span:
% B_j(z) = B0(:,j) + Bz(:,:,j)*z.
cache.B0 = cell(cache.nspans,1);
cache.Bz = cell(cache.nspans,1);
for s=1:cache.nspans
    ua=cache.spans(s,1); ub=cache.spans(s,2);
    [B0,Bz] = span_bezier_affine(cache,ua,ub);
    cache.B0{s}=B0;
    cache.Bz{s}=Bz;
end

% Objective Hessian/f vector cached.
[cache.H, cache.f] = build_objective_cache(cache,nom_fun,opt);

% Nominal midpoint geometry cache.
cache.mid = zeros(cache.nspans,1);
cache.pmid = zeros(3,cache.nspans);
cache.amid = zeros(3,cache.nspans);
for s=1:cache.nspans
    u=mean(cache.spans(s,:));
    cache.mid(s)=u;
    cache.pmid(:,s)=eval_spline(cache,Pbase,u,0);
    cache.amid(:,s)=eval_spline(cache,Pbase,u,2);
end
end

function [H,f] = build_objective_cache(cache,nom_fun,opt)
H = opt.qp_reg*eye(cache.nz);
f = zeros(cache.nz,1);
us = linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k);
    t=cache.t0+u*cache.T;
    D=nom_fun(t);

    J0=derivative_map(cache,u,0);
    q0=eval_spline(cache,cache.Pbase,u,0);
    e0=q0-D(:,1);
    H=H+2*opt.w_position*(J0'*J0)/numel(us);
    f=f+2*opt.w_position*J0'*e0/numel(us);

    J3=derivative_map(cache,u,3);
    q3=eval_spline(cache,cache.Pbase,u,3);
    H=H+2*opt.w_jerk*(J3'*J3)/numel(us);
    f=f+2*opt.w_jerk*J3'*q3/numel(us);

    J4=derivative_map(cache,u,4);
    q4=eval_spline(cache,cache.Pbase,u,4);
    H=H+2*opt.w_snap*(J4'*J4)/numel(us);
    f=f+2*opt.w_snap*J4'*q4/numel(us);
end
H=(H+H')/2;
end


% =========================================================================
% PHASE 7-11 ACTIVE CORRIDOR GENERATOR
% =========================================================================
function [A,b,info] = build_certified_active_corridor(cache,cfg,obs,opt)
% Candidate generation only.
% A span/obstacle pair is activated when the nominal trajectory is not
% comfortably separated.  Normal selection searches a small deterministic
% sphere and chooses the best nominal certified lower-bound direction.
%
% SAFETY DOES NOT DEPEND ON THIS PRUNING:
% certify_candidate_global() later checks ALL spans x ALL obstacles.

dirs=fibonacci_sphere(opt.separator_grid_N);
rowsA={}; rowsb={};
info.normal=zeros(3,cache.nspans,numel(obs));
info.active=false(cache.nspans,numel(obs));
info.nominal_g=-inf(cache.nspans,numel(obs));
info.n_constraints=0;

for sp=1:cache.nspans
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=cache.B0{sp};
    Abez=span_bezier_points(cache,cache.Pbase,ua,ub,2);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));
    Mlb=compute_certified_tension_lower_bound_cols(Wbez);

    for o=1:numel(obs)
        [g_best,n_best]=best_separator_grid(Pbez,Wbez,Mlb,obs(o),cfg,dirs);
        info.normal(:,sp,o)=n_best;
        info.nominal_g(sp,o)=g_best;

        % Generator heuristic: activate unsafe/near obstacle spans.
        % Omitted spans are still checked by the independent verifier.
        active = (g_best < opt.active_margin_trigger);
        info.active(sp,o)=active;
        if ~active, continue; end

        % Freeze nominal cable direction ONLY for convex candidate generation.
        um=mean([ua ub]);
        a0=eval_spline(cache,cache.Pbase,um,2);
        w0=a0+cfg.g_acc*cfg.e3;
        if norm(w0)<=opt.a_net_min
            nT0=cfg.e3;
        else
            nT0=w0/norm(w0);
        end
        rhs=obstacle_support(obs(o),n_best)+ ...
            delta_sep_exact(n_best,nT0,cfg.sys)+cfg.d_margin+opt.guide_extra;

        B0=cache.B0{sp}; Bz=cache.Bz{sp};
        for j=1:cache.p+1
            rowsA{end+1,1}=-n_best'*Bz(:,:,j); %#ok<AGROW>
            rowsb{end+1,1}= n_best'*B0(:,j)-rhs; %#ok<AGROW>
        end
    end
end

if isempty(rowsA)
    A=zeros(0,cache.nz); b=zeros(0,1);
else
    A=vertcat(rowsA{:}); b=vertcat(rowsb{:});
end
info.n_constraints=size(A,1);
info.n_active_pairs=nnz(info.active);
end

function [bg,bn]=best_separator_grid(Pbez,Wbez,Mlb,ob,cfg,dirs)
bg=-inf; bn=dirs(:,1);
for k=1:size(dirs,2)
    n=dirs(:,k);
    g=span_system_g_lower_M(n,Pbez,Wbez,Mlb,ob,cfg);
    if g>bg
        bg=g; bn=n;
    end
end
end

% =========================================================================
% PHASE 7-11 INDEPENDENT GLOBAL CONTINUOUS-TIME VERIFIER
% =========================================================================
function cert=certify_candidate_global(P,cache,cfg,limits,obs,opt)
% Independent execution gate.
% - checks EVERY span x EVERY obstacle
% - does not trust active-span pruning
% - does not trust frozen n_T from generator
% - uses Phase-5 M_LB to certify ||a_L+g e3|| away from zero
% - separator directions are candidate witnesses; for each chosen fixed n,
%   the resulting g_LB is a continuous-time sufficient certificate.

[Actrl,Vctrl,Jctrl]=derivative_control_polygons(P,cache);
v_bound=max(vecnorm(Vctrl,2,2));
a_bound=max(vecnorm(Actrl,2,2));
j_bound=max(vecnorm(Jctrl,2,2));
dyn_pass=(v_bound<=limits.v_max)&&(a_bound<=limits.a_max)&&(j_bound<=limits.j_max);

dirs=fibonacci_sphere(max(162,opt.separator_grid_N));
all_safe=true; cable_safe=true; min_g=inf; min_M=inf;
worst=struct('span',0,'obs',0,'g_lower',inf,'M_LB',inf,'n',[0;0;0]);

for sp=1:cache.nspans
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=span_bezier_points(cache,P,ua,ub,0);
    Abez=span_bezier_points(cache,P,ua,ub,2);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));

    Mlb=compute_certified_tension_lower_bound_cols(Wbez);
    min_M=min(min_M,Mlb);
    if Mlb<=opt.a_net_min
        cable_safe=false; all_safe=false;
    end

    for o=1:numel(obs)
        [g,n]=best_separator_grid(Pbez,Wbez,Mlb,obs(o),cfg,dirs);
        if g < -opt.cert_tol
            all_safe=false;
        end
        if g<min_g
            min_g=g;
            worst=struct('span',sp,'obs',o,'g_lower',g,'M_LB',Mlb,'n',n);
        end
    end
end

cert=struct();
cert.pass=all_safe && cable_safe && dyn_pass;
cert.geometry_pass=all_safe;
cert.dynamic_pass=dyn_pass;
cert.cable_attitude_defined=cable_safe;
cert.min_g_lower=min_g;
cert.min_M_LB=min_M;
cert.worst=worst;
cert.v_bound=v_bound;
cert.a_bound=a_bound;
cert.j_bound=j_bound;
end

% =========================================================================
% FAST SYSTEM-AWARE CORRIDOR GENERATOR
% =========================================================================
function [A,b,info] = build_fast_system_corridor(cache,cfg,obs,opt)
% One separating normal per span/obstacle.
% No hard-coded +Y direction.  Normal comes from local nominal geometry.
%
% The QP uses n_T frozen at the nominal span midpoint:
%   n'p_L >= h_O(n) + Delta_sep(n,n_T_nom) + margin.
%
% This frozen model is ONLY a candidate generator.  Safety is established
% later by certify_candidate(), which bounds n_T continuously.

max_rows = cache.nspans*numel(obs)*(cache.p+1);
A = zeros(max_rows,cache.nz);
b = zeros(max_rows,1);
row=0;

info.normal = zeros(3,cache.nspans,numel(obs));
info.h_obs = zeros(cache.nspans,numel(obs));
info.delta_frozen = zeros(cache.nspans,numel(obs));

for s=1:cache.nspans
    p0=cache.pmid(:,s);
    a0=cache.amid(:,s);
    w0=a0+cfg.g_acc*cfg.e3;
    if norm(w0) <= opt.a_net_min
        nT0=cfg.e3;
    else
        nT0=w0/norm(w0);
    end

    for o=1:numel(obs)
        n = choose_separating_normal(p0,obs(o));
        h = obstacle_support(obs(o),n);
        Delta = delta_sep_exact(n,nT0,cfg.sys);
        rhs = h + Delta + cfg.d_margin + opt.guide_extra;

        info.normal(:,s,o)=n;
        info.h_obs(s,o)=h;
        info.delta_frozen(s,o)=Delta;

        B0=cache.B0{s};
        Bz=cache.Bz{s};
        for j=1:cache.p+1
            % n'*(B0+Bz*z) >= rhs  ->  -n'Bz*z <= n'B0-rhs
            row=row+1;
            A(row,:)=-n'*Bz(:,:,j);
            b(row)=n'*B0(:,j)-rhs;
        end
    end
end

A=A(1:row,:);
b=b(1:row);
info.n_constraints=row;
end

function n = choose_separating_normal(p,ob)
% General convex obstacle interface still needs a reference center to choose
% a useful candidate normal.  Safety itself uses only support_fun.
if ~isfield(ob,'center')
    error('Each obstacle needs center for normal generation.');
end
v=p-ob.center;
if norm(v)<1e-12
    if isfield(ob,'R')
        v=ob.R(:,1);
    else
        v=[1;0;0];
    end
end
n=v/norm(v);
end


% =========================================================================
% PHASE 7-09 FEASIBILITY DIAGNOSTIC
% =========================================================================
function d = diagnose_qp_feasibility(cache,A,b,cfg,obs,corridor,opt)
% Diagnose the SAME linear corridor used by Phase 7-08.
% This function is diagnostic/offline: its runtime is NOT part of the final
% 1 ms / 2 ms target.
%
% Tests:
% D1 row-wise reachability under |z_i| <= box_z_max
% D2 zero/near-zero authority rows
% D3 global feasibility of A*z<=b with box
% D4 minimum uniform slack rho*
% D5 box-size sensitivity
% D6 separator-normal quality on problematic spans (Phase-6 idea)

nz=cache.nz;
zmax=opt.box_z_max;

% ----- D1: exact row-wise minimum over a box ------------------------------
% min a_i*z = -zmax*sum(abs(a_i))
row_min = -zmax*sum(abs(A),2);
row_margin = b-row_min; % >=0 means row can be satisfied individually
bad_row = row_margin < -1e-10;

authority = sum(abs(A),2);
zero_auth = authority < 1e-12;
zero_auth_bad = zero_auth & (b < -1e-10);

d.n_rows=size(A,1);
d.n_row_unreachable=sum(bad_row);
d.n_zero_authority=sum(zero_auth);
d.n_zero_authority_violated=sum(zero_auth_bad);
d.worst_row_margin=min(row_margin);

% Map row -> span, obstacle, Bezier point.
p1=cache.p+1;
nobs=numel(obs);
d.row_span=zeros(d.n_rows,1);
d.row_obs=zeros(d.n_rows,1);
d.row_bez=zeros(d.n_rows,1);
for r=1:d.n_rows
    block=ceil(r/p1);
    d.row_bez(r)=r-(block-1)*p1;
    d.row_span(r)=ceil(block/nobs);
    d.row_obs(r)=block-(d.row_span(r)-1)*nobs;
end

% ----- D2/D3: pure feasibility QP ----------------------------------------
H=1e-10*eye(nz);
f=zeros(nz,1);
lb=-zmax*ones(nz,1);
ub= zmax*ones(nz,1);
qopt=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',200);
try
    [zf,~,flagf,outf]=quadprog(H,f,A,b,[],[],lb,ub,[],qopt);
catch
    zf=[]; flagf=-99; outf=struct();
end
d.global_feasible=(flagf>0);
d.feasibility_flag=flagf;
if isempty(zf)
    d.max_violation_at_feas=NaN;
else
    d.max_violation_at_feas=max(A*zf-b);
end
if isfield(outf,'iterations'), d.feasibility_iterations=outf.iterations; else, d.feasibility_iterations=NaN; end

% ----- D4: minimum uniform relaxation rho* -------------------------------
% min rho
% s.t. A*z - rho <= b, box(z), rho>=0.
Hsl=1e-12*eye(nz+1);
fsl=[zeros(nz,1);1];
Asl=[A,-ones(size(A,1),1)];
lbsl=[lb;0];
ubsl=[ub;inf];
try
    [xs,~,flags]=quadprog(Hsl,fsl,Asl,b,[],[],lbsl,ubsl,[],qopt);
catch
    xs=[]; flags=-99;
end
d.slack_flag=flags;
if isempty(xs)
    d.rho_star=inf;
else
    d.rho_star=xs(end);
end

% ----- D5: box sensitivity -----------------------------------------------
box_list=[0.5 1 2 4 8 12 20 40];
d.box_list=box_list;
d.box_row_unreachable=zeros(size(box_list));
d.box_global_flag=zeros(size(box_list));
for k=1:numel(box_list)
    zm=box_list(k);
    rm=b-(-zm*sum(abs(A),2));
    d.box_row_unreachable(k)=sum(rm < -1e-10);
    lbk=-zm*ones(nz,1); ubk=zm*ones(nz,1);
    try
        [~,~,fk]=quadprog(H,f,A,b,[],[],lbk,ubk,[],qopt);
    catch
        fk=-99;
    end
    d.box_global_flag(k)=fk;
end

% ----- D6: separator diagnostic on only the worst/problematic spans -------
% Phase 6 used a global sphere diagnostic.  Here we use a compact Fibonacci
% sphere (162 directions) only on spans implicated by unreachable rows.
bad_spans=unique(d.row_span(bad_row));
if isempty(bad_spans)
    % If rows are individually reachable but globally incompatible, inspect
    % the spans with the smallest row margins.
    [~,idx]=sort(row_margin,'ascend');
    bad_spans=unique(d.row_span(idx(1:min(16,numel(idx)))));
end
bad_spans=bad_spans(1:min(4,numel(bad_spans)));
d.separator=struct('span',{},'obs',{},'g_current',{},'g_best_nominal',{}, ...
    'improvement',{},'n_current',{},'n_best',{});

dirs=fibonacci_sphere(162);
for ss=1:numel(bad_spans)
    s=bad_spans(ss);
    Pbez0=cache.B0{s}; % baseline trajectory
    Abez0=span_bezier_points(cache,cache.Pbase,cache.spans(s,1),cache.spans(s,2),2);
    Wbez0=Abez0+repmat(cfg.g_acc*cfg.e3,1,size(Abez0,2));
    for o=1:nobs
        nc=corridor.normal(:,s,o);
        gc=span_system_g_lower(nc,Pbez0,Wbez0,obs(o),cfg);
        gb=-inf; nb=nc;
        for q=1:size(dirs,2)
            n=dirs(:,q);
            gq=span_system_g_lower(n,Pbez0,Wbez0,obs(o),cfg);
            if gq>gb, gb=gq; nb=n; end
        end
        e=struct();
        e.span=s; e.obs=o; e.g_current=gc; e.g_best_nominal=gb;
        e.improvement=gb-gc; e.n_current=nc; e.n_best=nb;
        d.separator(end+1)=e; %#ok<AGROW>
    end
end

% Diagnosis label.
if d.n_zero_authority_violated>0
    d.primary_cause='FIXED_C6_ZERO_AUTHORITY';
elseif d.n_row_unreachable>0
    d.primary_cause='ROW_UNREACHABLE_WITHIN_Z_BOX';
elseif ~d.global_feasible && isfinite(d.rho_star) && d.rho_star>1e-8
    d.primary_cause='COLLECTIVE_HALFSPACE_CONFLICT';
elseif ~d.global_feasible
    d.primary_cause='SOLVER_OR_NUMERICAL_FAILURE';
else
    d.primary_cause='LINEAR_CORRIDOR_FEASIBLE';
end
end

function g=span_system_g_lower(n,Pbez,Wbez,ob,cfg)
n=n/norm(n);
p_low=min(n'*Pbez);
proj=n'*Wbez;
wmin=min(proj);
Mmax=max(vecnorm(Wbez,2,1));
if Mmax<=1e-12
    qlow=-1;
elseif wmin>=0
    qlow=wmin/Mmax;
else
    qlow=-1;
end
Dup=delta_sep_from_q_lower(qlow,cfg.sys);
h=obstacle_support(ob,n);
g=p_low-Dup-h-cfg.d_margin;
end

function D=fibonacci_sphere(N)
% Nearly uniform deterministic unit directions.
D=zeros(3,N);
phi=(1+sqrt(5))/2;
for k=0:N-1
    z=1-2*(k+0.5)/N;
    r=sqrt(max(0,1-z*z));
    th=2*pi*k/phi;
    D(:,k+1)=[r*cos(th);r*sin(th);z];
end
end

function print_diagnostic(d)
fprintf('\n================================================================================\n');
fprintf('[Phase 7-10 FULL MATHEMATICAL AUDIT]\n');
fprintf('================================================================================\n');
fprintf('rows                              : %d\n',d.n_rows);
fprintf('individually unreachable rows     : %d\n',d.n_row_unreachable);
fprintf('zero-authority rows               : %d\n',d.n_zero_authority);
fprintf('violated zero-authority rows      : %d\n',d.n_zero_authority_violated);
fprintf('worst row reachability margin     : %+.6e m\n',d.worst_row_margin);
fprintf('pure feasibility QP flag          : %d\n',d.feasibility_flag);
fprintf('global linear corridor feasible   : %d\n',d.global_feasible);
fprintf('minimum uniform slack rho*        : %.6e m\n',d.rho_star);
fprintf('primary diagnosis                 : %s\n',d.primary_cause);

fprintf('\n[box sensitivity]\n');
fprintf(' zmax [m] | unreachable rows | feasibility flag\n');
for k=1:numel(d.box_list)
    fprintf(' %8.2f | %16d | %d\n',d.box_list(k),d.box_row_unreachable(k),d.box_global_flag(k));
end

if ~isempty(d.separator)
    fprintf('\n[separator diagnostic: baseline trajectory]\n');
    fprintf(' span | obs | current g_LB | best g_LB(162 dirs) | improvement\n');
    for k=1:numel(d.separator)
        e=d.separator(k);
        fprintf(' %4d | %3d | %+12.6f | %+19.6f | %+11.6f\n', ...
            e.span,e.obs,e.g_current,e.g_best_nominal,e.improvement);
    end
end

if d.n_row_unreachable>0
    fprintf('\n[worst unreachable rows]\n');
    % compact list from already stored row metadata is intentionally omitted
    % from runtime state; cause is summarized above.
end
fprintf('================================================================================\n');
end


% =========================================================================
% PHASE 7-10 FULL MATHEMATICAL AUDIT
% =========================================================================
function a = run_full_mathematical_audit(cache,A,b,cfg,limits,obs,corridor,opt)
% OFFLINE diagnostic.  Not part of final online latency.
%
% A1 exact row metadata and top violated/reachability rows
% A2 unconstrained-box feasibility (tests whether z-box is the only cause)
% A3 per-span authority / nominal certified geometry / tension M_LB
% A4 1250-direction global separator diagnostic (Phase-6 inheritance)
% A5 active-span necessity classification
% A6 N_ctrl feasibility sweep with the SAME endpoint-C6 construction
%
% IMPORTANT:
% "safe_nominal" below means certified by at least one tested separating
% direction.  A negative grid maximum means NOT CERTIFIED by this finite
% diagnostic search; it is not a collision proof.

zmax=opt.box_z_max;
nr=size(A,1);
row_min=-zmax*sum(abs(A),2);
margin=b-row_min;
authority=sum(abs(A),2);

p1=cache.p+1; nobs=numel(obs);
meta=struct('row',{},'span',{},'obs',{},'bez',{},'b',{},'authority',{}, ...
    'row_min',{},'margin',{},'normal',{});
for r=1:nr
    block=ceil(r/p1);
    bez=r-(block-1)*p1;
    sp=ceil(block/nobs);
    ob=block-(sp-1)*nobs;
    e.row=r; e.span=sp; e.obs=ob; e.b=b(r); e.bez=bez;
    e.authority=authority(r); e.row_min=row_min(r); e.margin=margin(r);
    e.normal=corridor.normal(:,sp,ob);
    meta(end+1)=e; %#ok<AGROW>
end
[~,ord]=sort(margin,'ascend');
a.worst_rows=meta(ord(1:min(20,nr)));
a.n_unreachable=sum(margin<0);

% A2: remove z box entirely.
H=1e-10*eye(cache.nz); f=zeros(cache.nz,1);
qopt=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',500);
try
    [zu,~,fu]=quadprog(H,f,A,b,[],[],[],[],[],qopt);
catch
    zu=[]; fu=-99;
end
a.unboxed_flag=fu;
a.unboxed_feasible=fu>0;
if isempty(zu), a.unboxed_z_inf=inf; else, a.unboxed_z_inf=norm(zu,inf); end

% A3-A5 span audit.
a.span=struct('span',{},'u0',{},'u1',{},'authority_sum',{}, ...
    'M_LB',{},'v_UB',{},'a_UB',{},'j_UB',{},'snap_UB',{},'aQ_UB',{}, ...
    'g_current',{},'g_global_grid',{},'n_global',{}, ...
    'nominal_certified_grid',{},'constraint_needed_nominal',{});

dirs=phase6_sphere_grid();
for sp=1:cache.nspans
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=span_bezier_points(cache,cache.Pbase,ua,ub,0);
    Vbez=span_bezier_points(cache,cache.Pbase,ua,ub,1);
    Abez=span_bezier_points(cache,cache.Pbase,ua,ub,2);
    Jbez=span_bezier_points(cache,cache.Pbase,ua,ub,3);
    Sbez=span_bezier_points(cache,cache.Pbase,ua,ub,4);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));

    Mlb=compute_certified_tension_lower_bound_cols(Wbez);
    vub=max(vecnorm(Vbez,2,1));
    aub=max(vecnorm(Abez,2,1));
    jub=max(vecnorm(Jbez,2,1));
    sub=max(vecnorm(Sbez,2,1));
    if Mlb>0
        omega_up=jub/Mlb;
        alpha_up=(sub/Mlb)+2*(jub/Mlb)*omega_up+omega_up^2;
        aQub=aub+cfg.sys.L_cable*alpha_up;
    else
        aQub=inf;
    end

    % Report worst obstacle in this span.
    gcur=inf; gglob=inf; nglob=[1;0;0];
    for ob=1:nobs
        nc=corridor.normal(:,sp,ob);
        gc=span_system_g_lower_M(nc,Pbez,Wbez,Mlb,obs(ob),cfg);
        [gg,ng]=global_separator_grid(Pbez,Wbez,Mlb,obs(ob),cfg,dirs);
        if gc<gcur, gcur=gc; end
        if gg<gglob, gglob=gg; nglob=ng; end
    end

    rows_sp=find([meta.span]==sp);
    auth_sp=sum(authority(rows_sp));

    e.span=sp; e.u0=ua; e.u1=ub; e.authority_sum=auth_sp;
    e.M_LB=Mlb; e.v_UB=vub; e.a_UB=aub; e.j_UB=jub; e.snap_UB=sub; e.aQ_UB=aQub;
    e.g_current=gcur; e.g_global_grid=gglob; e.n_global=nglob;
    e.nominal_certified_grid=(gglob>=0);
    % If nominal is already certified, forcing the QP to move that span to a
    % newly chosen halfspace is not necessary for nominal safety.
    e.constraint_needed_nominal=~e.nominal_certified_grid;
    a.span(end+1)=e; %#ok<AGROW>
end

a.min_M_LB=min([a.span.M_LB]);
a.min_g_current=min([a.span.g_current]);
a.min_g_global_grid=min([a.span.g_global_grid]);
a.n_nominal_certified=sum([a.span.nominal_certified_grid]);
a.n_nominal_uncertified=cache.nspans-a.n_nominal_certified;

% A6: identify whether fixed dimension is structurally too small.
Nlist=unique([18 22 26 30 34]);
a.Nctrl=struct('N',{},'nz',{},'rows',{},'unreachable',{},'feas_flag',{},'rho',{});
for k=1:numel(Nlist)
    N=Nlist(k);
    if N<=2*cfg.p, continue; end
    cfgk=cfg; cfgk.N_ctrl=N;
    try
        ck=build_static_cache(cfgk,cache.t0,cache.t1,@benchmark_nominal,opt);
        [Ak,bk,~]=build_fast_system_corridor(ck,cfgk,obs,opt);
        rm=bk-(-zmax*sum(abs(Ak),2));
        Hk=1e-10*eye(ck.nz); fk=zeros(ck.nz,1);
        lb=-zmax*ones(ck.nz,1); ub=zmax*ones(ck.nz,1);
        [~,~,fl]=quadprog(Hk,fk,Ak,bk,[],[],lb,ub,[],qopt);

        Hs=1e-12*eye(ck.nz+1); fs=[zeros(ck.nz,1);1];
        As=[Ak,-ones(size(Ak,1),1)];
        [xs,~,~]=quadprog(Hs,fs,As,bk,[],[],[lb;0],[ub;inf],[],qopt);
        rho=xs(end);
        ek.N=N; ek.nz=ck.nz; ek.rows=size(Ak,1);
        ek.unreachable=sum(rm<0); ek.feas_flag=fl; ek.rho=rho;
    catch
        ek.N=N; ek.nz=NaN; ek.rows=NaN; ek.unreachable=NaN; ek.feas_flag=-99; ek.rho=NaN;
    end
    a.Nctrl(end+1)=ek; %#ok<AGROW>
end

% Overall interpretation.
if ~a.unboxed_feasible
    a.interpretation='HALFSPACE_SET_INCOMPATIBLE_EVEN_WITHOUT_Z_BOX';
elseif a.n_unreachable>0
    a.interpretation='BOX_AND_CORRIDOR_GEOMETRY_LIMIT_REACHABILITY';
elseif a.n_nominal_uncertified>0
    a.interpretation='CORRIDOR_FEASIBLE_BUT_NOMINAL_HAS_UNCERTIFIED_SPANS';
else
    a.interpretation='LINEAR_CORRIDOR_AND_NOMINAL_CERTIFICATE_DIAGNOSTIC_PASS';
end
end

function M=compute_certified_tension_lower_bound_cols(Wc)
% Phase-5 formula, adapted from row-wise W to columns Wc.
W=Wc';
Q=W*W';
nv=size(W,1);
lam=ones(nv,1)/nv;
y=lam;
L=max(eig(Q));
t_step=1/(L+1e-6);
for iter=1:20
    prev=lam;
    grad=Q*y;
    lam=project_to_simplex_local(y-t_step*grad);
    y=lam+((iter-1)/(iter+2))*(lam-prev);
end
what=W'*lam;
Mhat=norm(what);
if Mhat>0
    v=what/Mhat;
    M=max(0,min(W*v));
else
    M=0;
end
end

function x=project_to_simplex_local(v)
n=length(v); u=sort(v,'descend'); cssv=cumsum(u);
rho=find(u>(cssv-1)./(1:n)',1,'last');
theta=(cssv(rho)-1)/rho;
x=max(v-theta,0);
end

function g=span_system_g_lower_M(n,Pbez,Wbez,Mlb,ob,cfg)
n=n/norm(n);
p_low=min(n'*Pbez);
wmin=min(n'*Wbez);
Mmax=max(vecnorm(Wbez,2,1));
if Mmax<=1e-12
    qlow=-1;
elseif wmin>=0
    qlow=wmin/Mmax;
elseif Mlb>0
    qlow=max(-1,wmin/Mlb);
else
    qlow=-1;
end
Dup=delta_sep_from_q_lower(qlow,cfg.sys);
g=p_low-Dup-obstacle_support(ob,n)-cfg.d_margin;
end

function [bg,bn]=global_separator_grid(Pbez,Wbez,Mlb,ob,cfg,dirs)
bg=-inf; bn=[1;0;0];
for k=1:size(dirs,2)
    n=dirs(:,k);
    g=span_system_g_lower_M(n,Pbez,Wbez,Mlb,ob,cfg);
    if g>bg, bg=g; bn=n; end
end
end

function D=phase6_sphere_grid()
Ntheta=25; Nphi=50;
th=linspace(0.01,pi-0.01,Ntheta);
ph=linspace(0,2*pi,Nphi+1); ph(end)=[];
D=zeros(3,Ntheta*Nphi); k=0;
for i=1:Ntheta
    for j=1:Nphi
        k=k+1;
        D(:,k)=[sin(th(i))*cos(ph(j));sin(th(i))*sin(ph(j));cos(th(i))];
    end
end
end

function print_full_audit(a)
fprintf('\n================================================================================\n');
fprintf('[Phase 7-10 FULL MATHEMATICAL AUDIT]\n');
fprintf('================================================================================\n');
fprintf('unreachable rows @ current box : %d\n',a.n_unreachable);
fprintf('unboxed feasibility flag       : %d\n',a.unboxed_flag);
fprintf('unboxed feasible               : %d\n',a.unboxed_feasible);
fprintf('minimum ||z||_inf unboxed sol. : %.6f m\n',a.unboxed_z_inf);
fprintf('min Phase-5 M_LB               : %.6e m/s^2\n',a.min_M_LB);
fprintf('min current-normal g_LB        : %+.6e m\n',a.min_g_current);
fprintf('min global-grid g_LB           : %+.6e m\n',a.min_g_global_grid);
fprintf('nominal certified spans        : %d\n',a.n_nominal_certified);
fprintf('nominal uncertified spans      : %d\n',a.n_nominal_uncertified);
fprintf('interpretation                 : %s\n',a.interpretation);

fprintf('\n[20 worst linear rows]\n');
fprintf(' row | span | obs | bez | authority | row_min | b | reach.margin\n');
for k=1:numel(a.worst_rows)
    e=a.worst_rows(k);
    fprintf('%4d | %4d | %3d | %3d | %9.3e | %+8.4f | %+8.4f | %+10.6f\n', ...
        e.row,e.span,e.obs,e.bez,e.authority,e.row_min,e.b,e.margin);
end

fprintf('\n[span-by-span mathematical audit]\n');
fprintf('span | [u0,u1] | M_LB | g_current | g_grid1250 | vUB | aUB | jUB | aQUB | nominal cert\n');
for k=1:numel(a.span)
    e=a.span(k);
    fprintf('%4d | [%.3f,%.3f] | %.4f | %+8.4f | %+9.4f | %.3f | %.3f | %.3f | %.3f | %d\n', ...
        e.span,e.u0,e.u1,e.M_LB,e.g_current,e.g_global_grid,e.v_UB,e.a_UB,e.j_UB,e.aQ_UB,e.nominal_certified_grid);
end

fprintf('\n[N_ctrl structural sweep]\n');
fprintf(' Nctrl | nz | rows | unreachable | feas.flag | rho*\n');
for k=1:numel(a.Nctrl)
    e=a.Nctrl(k);
    fprintf('%6d | %3.0f | %4.0f | %11.0f | %9d | %.6e\n', ...
        e.N,e.nz,e.rows,e.unreachable,e.feas_flag,e.rho);
end
fprintf('================================================================================\n');
end

% =========================================================================
% SMALL QP
% =========================================================================
function [z,flag,info] = solve_fast_qp(cache,A,b,cfg,opt)
lb=-opt.box_z_max*ones(cache.nz,1);
ub= opt.box_z_max*ones(cache.nz,1);

% Warm-start hook.  In the test function use zero; online integration should
% replace x0 by the previous cycle's certified/solved z after horizon shift.
x0=zeros(cache.nz,1);

try
    [z,~,flag,out]=quadprog(cache.H,cache.f,A,b,[],[],lb,ub,x0,opt.qp_options);
catch ME
    z=[];
    flag=-99;
    info=struct('message',ME.message,'iterations',NaN);
    return;
end

info=struct();
if isfield(out,'iterations'), info.iterations=out.iterations; else, info.iterations=NaN; end
if isfield(out,'message'), info.message=out.message; else, info.message=''; end
end

% =========================================================================
% INDEPENDENT CONTINUOUS-TIME CERTIFICATE
% =========================================================================
function cert = certify_candidate(P,cache,cfg,limits,obs,corridor,opt)
% Sound sufficient certificate for each span/obstacle and the exact normal
% selected by the corridor generator.
%
% For a fixed n:
%   g(t;n)=n'p_L(t)-Delta_sep(n,n_T(t))-h_O(n)-d_margin.
%
% Position:
%   p_low = min BezierCP(n'p_L) <= n'p_L(t), all t in span.
%
% Cable attitude:
%   W(t)=a_L(t)+g e3.
%   W(t) lies in convex hull of acceleration Bezier CPs.
%   If w_min=min n'W_i >=0:
%       n'n_T = n'W/||W|| >= w_min/M_max.
%   Otherwise q_lower=-1 is always valid.
%
% Delta_sep is monotonically non-increasing in q=n'n_T:
%   Delta <= Delta_from_q(q_lower).
%
% Hence g_lower is a continuous-time lower bound.

[Actrl, Vctrl, Jctrl] = derivative_control_polygons(P,cache);

% Continuous-time dynamic feasibility by convex hull + convexity of norm.
v_bound=max(vecnorm(Vctrl,2,2));
a_bound=max(vecnorm(Actrl,2,2));
j_bound=max(vecnorm(Jctrl,2,2));

dyn_pass=(v_bound<=limits.v_max) && ...
         (a_bound<=limits.a_max) && ...
         (j_bound<=limits.j_max);

min_g=inf;
worst=struct('span',0,'obs',0,'g_lower',inf,'q_lower',NaN,'Delta_upper',NaN);
all_safe=true;
singular_safe=true;

% Build Bezier polygons of candidate position and acceleration span-by-span.
for s=1:cache.nspans
    ua=cache.spans(s,1); ub=cache.spans(s,2);
    Pbez=span_bezier_points(cache,P,ua,ub,0);
    Abez=span_bezier_points(cache,P,ua,ub,2);
    Wbez=Abez + repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));

    % Sufficient non-singularity check:
    % if origin cannot be excluded by this simple lower bound, certificate
    % refuses execution.  It never silently normalizes a near-zero vector.
    Wnorm_upper=max(vecnorm(Wbez,2,1));
    if Wnorm_upper <= opt.a_net_min
        singular_safe=false;
        all_safe=false;
    end

    for o=1:numel(obs)
        n=corridor.normal(:,s,o);
        h=obstacle_support(obs(o),n);

        p_low=min(n'*Pbez);
        projW=n'*Wbez;
        w_min=min(projW);
        M_max=max(vecnorm(Wbez,2,1));

        if M_max <= opt.a_net_min
            q_lower=-1.0;
            singular_safe=false;
        elseif w_min>=0
            q_lower=w_min/M_max;
        else
            q_lower=-1.0;
        end

        Delta_up=delta_sep_from_q_lower(q_lower,cfg.sys);
        g_low=p_low-Delta_up-h-cfg.d_margin;

        if g_low < -opt.cert_tol
            all_safe=false;
        end
        if g_low < min_g
            min_g=g_low;
            worst=struct('span',s,'obs',o,'g_lower',g_low, ...
                'q_lower',q_lower,'Delta_upper',Delta_up);
        end
    end
end

cert=struct();
cert.pass=all_safe && dyn_pass && singular_safe;
cert.geometry_pass=all_safe;
cert.dynamic_pass=dyn_pass;
cert.cable_attitude_defined=singular_safe;
cert.min_g_lower=min_g;
cert.worst=worst;
cert.v_bound=v_bound;
cert.a_bound=a_bound;
cert.j_bound=j_bound;
end

% =========================================================================
% SYSTEM GEOMETRY / OBSTACLE SUPPORT
% =========================================================================
function d = delta_sep_exact(n,nT,sys)
% Phase-3 lower-support separation envelope:
% Delta_sep(n,nT)=Delta_h(-n,nT)
q=n'*nT;
d=delta_sep_from_q_lower(q,sys);
end

function d = delta_sep_from_q_lower(q,sys)
% q is a certified lower bound on n'*nT.
val_L=sys.R_payload;
val_Q=sys.R_uav - sys.L_cable*q;
val_C=sys.R_cable + max(0.0,-sys.L_cable*q);
d=max([val_L,val_Q,val_C]);
end

function h = obstacle_support(ob,n)
% Generic convex obstacle support.
if isfield(ob,'support_fun') && ~isempty(ob.support_fun)
    h=ob.support_fun(n);
elseif isfield(ob,'S') && isfield(ob,'center')
    % Rotated ellipsoid E={c+S^(1/2)u: ||u||<=1}
    h=n'*ob.center + sqrt(max(0,n'*ob.S*n));
else
    error('Obstacle requires support_fun or {center,S}.');
end
end

% =========================================================================
% C6 AUDIT
% =========================================================================
function [err,by] = audit_c6(P,cache,nom_fun,opt)
D0=nom_fun(cache.t0);
D1=nom_fun(cache.t1);
by=zeros(7,1);
for r=0:6
    qs=eval_spline(cache,P,0,r);
    qe=eval_spline(cache,P,1,r);
    by(r+1)=max(norm(qs-D0(:,r+1),inf),norm(qe-D1(:,r+1),inf));
end
err=max(by);
end

% =========================================================================
% B-SPLINE / BEZIER
% =========================================================================
function P = apply_z(cache,z)
P=cache.Pbase;
nf=cache.nf;
P(cache.free_idx,1)=P(cache.free_idx,1)+z(1:nf);
P(cache.free_idx,2)=P(cache.free_idx,2)+z(nf+(1:nf));
P(cache.free_idx,3)=P(cache.free_idx,3)+z(2*nf+(1:nf));
end

function J = derivative_map(cache,u,r)
B=basis_derivative_all(u,cache.knots,cache.p,cache.N,r)/cache.T^r;
bf=B(cache.free_idx);
J=zeros(3,cache.nz);
nf=cache.nf;
J(1,1:nf)=bf;
J(2,nf+(1:nf))=bf;
J(3,2*nf+(1:nf))=bf;
end

function q = eval_spline(cache,P,u,r)
B=basis_derivative_all(u,cache.knots,cache.p,cache.N,r)/cache.T^r;
q=(B*P)';
end

function [B0,Bz] = span_bezier_affine(cache,ua,ub)
p=cache.p;
xi=linspace(0,1,p+1)';
Q0=zeros(p+1,3);
Qz=zeros(p+1,3,cache.nz);
for j=1:p+1
    u=ua+(ub-ua)*xi(j);
    Q0(j,:)=eval_spline(cache,cache.Pbase,u,0)';
    J=derivative_map(cache,u,0);
    for iz=1:cache.nz
        Qz(j,:,iz)=J(:,iz)';
    end
end
Bez0=cache.Bern_inv*Q0;
B0=Bez0';
Bz=zeros(3,cache.nz,p+1);
for iz=1:cache.nz
    tmp=cache.Bern_inv*squeeze(Qz(:,:,iz));
    for j=1:p+1
        Bz(:,iz,j)=tmp(j,:)';
    end
end
end

function Bez = span_bezier_points(cache,P,ua,ub,r)
% Exact degree-(p-r) polynomial reconstruction on one original knot span.
deg=cache.p-r;
xi=linspace(0,1,deg+1)';
M=zeros(deg+1);
Q=zeros(deg+1,3);
for j=1:deg+1
    u=ua+(ub-ua)*xi(j);
    M(j,:)=bernstein_row(deg,xi(j));
    Q(j,:)=eval_spline(cache,P,u,r)';
end
Bez=(M\Q)';
end

function b = bernstein_row(p,x)
b=zeros(1,p+1);
for k=0:p
    b(k+1)=nchoosek(p,k)*x^k*(1-x)^(p-k);
end
end

function [Actrl,Vctrl,Jctrl] = derivative_control_polygons(P,cache)
% Derivative B-spline control polygons in physical time.
Vctrl=derive_ctrl(P,cache.knots,cache.p)/cache.T;
U1=cache.knots(2:end-1);
Actrl=derive_ctrl(Vctrl,U1,cache.p-1)/cache.T;
U2=U1(2:end-1);
Jctrl=derive_ctrl(Actrl,U2,cache.p-2)/cache.T;
end

function D = derive_ctrl(P,U,p)
N=size(P,1);
D=zeros(N-1,3);
for i=1:N-1
    den=U(i+p+1)-U(i+1);
    D(i,:)=p*(P(i+1,:)-P(i,:))/den;
end
end

function P = solve_endpoint_fixed(p,N,U,T,D0,D1)
P=zeros(N,3);
A0=zeros(p); A1=zeros(p);
for r=0:p-1
    q0=basis_derivative_all(0,U,p,N,r)/T^r;
    q1=basis_derivative_all(1,U,p,N,r)/T^r;
    A0(r+1,:)=q0(1:p);
    A1(r+1,:)=q1(end-p+1:end);
end
for ax=1:3
    P(1:p,ax)=A0\D0(ax,1:p)';
    P(end-p+1:end,ax)=A1\D1(ax,1:p)';
end
end

function U = clamped_uniform_knots(N,p)
ni=N-p-1;
if ni>0
    internal=(1:ni)/(ni+1);
else
    internal=[];
end
U=[zeros(1,p+1),internal,ones(1,p+1)];
end

function dN = basis_derivative_all(u,U,p,N,r)
% Cox-de Boor derivative recursion.  r<=p.
if r>p
    dN=zeros(1,N);
    return;
end

% degree-0 basis
M=length(U)-1;
B=zeros(p+1,M);
for i=1:M
    if (u>=U(i) && u<U(i+1)) || (u==1 && U(i+1)==1 && U(i)<1)
        B(1,i)=1;
    end
end

% build basis degrees
for d=1:p
    for i=1:(M-d)
        a=0; c=0;
        den1=U(i+d)-U(i);
        den2=U(i+d+1)-U(i+1);
        if den1~=0, a=(u-U(i))/den1*B(d,i); end
        if den2~=0, c=(U(i+d+1)-u)/den2*B(d,i+1); end
        B(d+1,i)=a+c;
    end
end

if r==0
    dN=B(p+1,1:N);
    return;
end

% Recursive derivative coefficients using lower-degree bases.
dN=zeros(1,N);
for i=1:N
    dN(i)=basis_derivative_single(i,p,r,u,U,B);
end
end

function v = basis_derivative_single(i,p,r,u,U,B)
if r==0
    v=B(p+1,i);
    return;
end
if p==0
    v=0;
    return;
end

% derivative identity:
% d^r N_{i,p} = p/(U_{i+p}-U_i) d^(r-1)N_{i,p-1}
%             - p/(U_{i+p+1}-U_{i+1}) d^(r-1)N_{i+1,p-1}
den1=U(i+p)-U(i);
den2=U(i+p+1)-U(i+1);
a=0; b=0;
if den1~=0
    a=p/den1*basis_derivative_single_general(i,p-1,r-1,u,U);
end
if den2~=0
    b=p/den2*basis_derivative_single_general(i+1,p-1,r-1,u,U);
end
v=a-b;
end

function v = basis_derivative_single_general(i,p,r,u,U)
if r>p
    v=0;
    return;
end
if r==0
    v=basis_single(i,p,u,U);
    return;
end
den1=U(i+p)-U(i);
den2=U(i+p+1)-U(i+1);
a=0; b=0;
if den1~=0
    a=p/den1*basis_derivative_single_general(i,p-1,r-1,u,U);
end
if den2~=0
    b=p/den2*basis_derivative_single_general(i+1,p-1,r-1,u,U);
end
v=a-b;
end

function v = basis_single(i,p,u,U)
if p==0
    v=double((u>=U(i) && u<U(i+1)) || ...
        (u==1 && U(i+1)==1 && U(i)<1));
    return;
end
a=0; b=0;
den1=U(i+p)-U(i);
den2=U(i+p+1)-U(i+1);
if den1~=0
    a=(u-U(i))/den1*basis_single(i,p-1,u,U);
end
if den2~=0
    b=(U(i+p+1)-u)/den2*basis_single(i+1,p-1,u,U);
end
v=a+b;
end


% =========================================================================
% PHASE 7-12F DIAGNOSTIC HELPERS
% =========================================================================
function r=audit_nominal_risk_window(nom_fun,obs,cfg,opt,t0,t1,dt)
ts=t0:dt:t1;
gv=inf(size(ts));
for k=1:numel(ts)
    D=nom_fun(ts(k)); p=D(:,1); a=D(:,3);
    W=a+cfg.g_acc*cfg.e3;
    if norm(W)<=opt.a_net_min
        gv(k)=-inf; continue;
    end
    nT=W/norm(W);
    gobs=inf(1,numel(obs));
    for io=1:numel(obs)
        dirs=fibonacci_sphere(opt.separator_grid_N);
        best=-inf;
        for q=1:size(dirs,2)
            n=dirs(:,q);
            val=n'*p-relative_system_support_minus(n,nT,cfg)-obstacle_support(obs(io),n)-cfg.d_margin;
            best=max(best,val);
        end
        gobs(io)=best;
    end
    gv(k)=min(gobs);
end
bad=gv<0;
ib=find(bad,1,'first');
[gw,iw]=min(gv);
r=struct('t_first_bad',NaN,'t_worst',ts(iw),'g_worst',gw,'t_clear',NaN, ...
    'unsafe_duration',NaN,'t',ts,'g',gv);
if isempty(ib), return; end
r.t_first_bad=ts(ib);
ic=find(~bad & ((1:numel(ts))>ib),1,'first');
if ~isempty(ic), r.t_clear=ts(ic); r.unsafe_duration=r.t_clear-r.t_first_bad; end
end

function d=diagnose_geometry_case(cfg0,N,obs,nom_fun,t0,T,opt)
cfg=cfg0; cfg.N_ctrl=N;
d=struct('N',N,'T',T,'n_active',0,'n_rows',0,'feasible',0,'rho',inf, ...
    'worst_span',0,'worst_row',0,'worst_row_margin',-inf,'after_clear',false);
try
    c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
    [A,b,info]=build_certified_active_corridor(c,cfg,obs,opt);
    d.n_active=info.n_active_pairs; d.n_rows=size(A,1);
    if isempty(A), d.feasible=1; d.rho=0; return; end
    zmax=opt.box_z_max;
    row_min=-zmax*sum(abs(A),2);
    margin=b-row_min;
    [d.worst_row_margin,d.worst_row]=min(margin);
    % Active-corridor rows are appended in groups of p+1 per active pair.
    d.worst_span=0;
    cnt=0;
    for sp=1:c.nspans
        for io=1:numel(obs)
            if info.active(sp,io)
                rows=cnt+(1:(c.p+1));
                if any(rows==d.worst_row), d.worst_span=sp; end
                cnt=cnt+c.p+1;
            end
        end
    end
    f=feas_E(c,A,b,opt);
    d.feasible=f.flag>0; d.rho=f.rho;
catch
end
end

function a=audit_span_authority(cfg0,N,nom_fun,t0,T,opt)
cfg=cfg0; cfg.N_ctrl=N;
c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
auth=zeros(c.nspans,1);
for sp=1:c.nspans
    Bz=c.Bz{sp};
    auth(sp)=sum(abs(Bz(:)));
end
[~,ord]=sort(auth,'ascend');
a=struct('N',N,'T',T,'span_authority',auth,'low_spans',ord(1:min(5,numel(ord)))');
end


% =========================================================================
% BENCHMARK / I/O
% =========================================================================
function s = setup_benchmark()
cfg.p=7;
cfg.N_ctrl=26;
cfg.g_acc=9.80665;
cfg.e3=[0;0;1];
cfg.sys.L_cable=1.0;
cfg.sys.R_payload=0.15;
cfg.sys.R_cable=0.02;
cfg.sys.R_uav=0.30;
cfg.d_margin=0.20;

limits.v_max=4.5;
limits.a_max=3.5;
limits.j_max=5.0;

obs(1).center=[1;0.3;18.5];
obs(1).radii=[1.3;1.0;1.6];
obs(1).R=eul2rotm_local([pi/7,pi/10,pi/8]);
obs(1).S=obs(1).R*diag(obs(1).radii.^2)*obs(1).R';

s=struct();
s.cfg=cfg;
s.limits=limits;
s.obs=obs;
s.nom_fun=@benchmark_nominal;
s.t_start=0.0;
s.t_end=15.26;
end

function D = benchmark_nominal(t)
% D(:,r+1) = r-th physical-time derivative, r=0..6.
D=zeros(3,7);
D(:,1)=[0.12*t; 0.15*sin(0.35*t); 15+0.48*t];
D(:,2)=[0.12; 0.0525*cos(0.35*t); 0.48];
D(:,3)=[0; -0.018375*sin(0.35*t); 0];
D(:,4)=[0; -0.00643125*cos(0.35*t); 0];
D(:,5)=[0; 0.0022509375*sin(0.35*t); 0];
D(:,6)=[0; 0.000787828125*cos(0.35*t); 0];
D(:,7)=[0; -0.00027573984375*sin(0.35*t); 0];
end

function [cfg,limits,obs,nom_fun,t0,t1] = unpack_scenario(s)
cfg=s.cfg;
limits=s.limits;
obs=s.obs;
nom_fun=s.nom_fun;
t0=s.t_start;
t1=s.t_end;

if ~isfield(cfg,'p'), cfg.p=7; end
if ~isfield(cfg,'N_ctrl'), cfg.N_ctrl=22; end
if ~isfield(cfg,'g_acc'), cfg.g_acc=9.80665; end
if ~isfield(cfg,'e3'), cfg.e3=[0;0;1]; end

for i=1:numel(obs)
    if ~isfield(obs(i),'support_fun') || isempty(obs(i).support_fun)
        if ~isfield(obs(i),'S')
            if isfield(obs(i),'R') && isfield(obs(i),'radii')
                obs(i).S=obs(i).R*diag(obs(i).radii.^2)*obs(i).R';
            else
                error('Obstacle %d lacks support_fun or ellipsoid S.',i);
            end
        end
    end
end
end

function R = eul2rotm_local(eul)
y=eul(1); p=eul(2); r=eul(3);
Rz=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
Ry=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rx=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
R=Rz*Ry*Rx;
end

function result = make_failure_result(reason,tms,flag)
result=struct('execute',false,'reason',reason,'qpflag',flag, ...
    'time_generate_ms',tms,'time_cert_ms',0,'time_total_ms',tms, ...
    'time_typical_target_pass',false,'time_hard_budget_pass',false);
end

function print_result(r,corridor,cert)
fprintf('\n================================================================================\n');
fprintf('[Phase 7-11 RESULT]\n');
fprintf('================================================================================\n');
fprintf('corridor constraints : %d\n',corridor.n_constraints);
if isfield(corridor,'n_active_pairs')
    fprintf('active span/obs pairs : %d\n',corridor.n_active_pairs);
end
fprintf('QP flag              : %d\n',r.qpflag);
fprintf('generator time       : %.4f ms\n',r.time_generate_ms);

if ~isempty(cert)
    fprintf('certificate time     : %.4f ms\n',r.time_cert_ms);
    fprintf('total online time    : %.4f ms\n',r.time_total_ms);
    fprintf('C6 max error         : %.3e\n',r.c6_err);
    fprintf('certificate min g_LB : %+.6e m\n',cert.min_g_lower);
if isfield(cert,'min_M_LB')
    fprintf('certificate min M_LB : %.6e m/s^2\n',cert.min_M_LB);
end
    fprintf('certified v bound    : %.6f m/s\n',cert.v_bound);
    fprintf('certified a bound    : %.6f m/s^2\n',cert.a_bound);
    fprintf('certified j bound    : %.6f m/s^3\n',cert.j_bound);
    fprintf('geometry certificate : %d\n',cert.geometry_pass);
    fprintf('dynamic certificate  : %d\n',cert.dynamic_pass);
    fprintf('hard 2ms budget      : %d\n',r.time_hard_budget_pass);
end

fprintf('reason               : %s\n',r.reason);
fprintf('EXECUTE              : %d\n',r.execute);
if ~r.execute
    fprintf('online action        : KEEP PREVIOUS CERTIFIED BACKUP\n');
end
fprintf('================================================================================\n');
end
