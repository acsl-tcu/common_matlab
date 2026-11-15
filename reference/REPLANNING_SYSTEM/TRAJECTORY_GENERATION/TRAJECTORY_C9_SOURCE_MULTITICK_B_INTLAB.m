function R=TRAJECTORY_C9_SOURCE_MULTITICK_B_INTLAB(agent,piece, ...
    prior_ekf_25x2,prior_cov_25x25x2,plant_24x2,prior_held_4x2, ...
    observed,radii,dt,n_ticks)
% Ideal-source multi-tick EKF->HLC->ZOH plant reachability (OFFLINE audit).
% This composes the actual source-linked ONE-TICK enclosures; at each new
% tick, noiseless MOTIVE readings are computed from the preceding PLANT
% reachable set, not guessed or copied from EKF covariance.  All 25-state
% estimator, 12 measurements and 24-state plant boxes are carried forward.
% Tube-B position radii are computed from whole-sample reachable sets and
% directed full-segment C9 nominal enclosures.  Cable error is the maximum
% of endpoint errors, since the cable is a common convex combination.
% This is a mathematical ideal-model *finite-horizon* inclusion if it PASSES
% with INTLAB. It is NOT an RPI certificate, not actual MATLAB floating
% error inclusion, and NEVER licenses switching or real Flight execution.
% No future true plant state, unknown obstacle, or fixed mass is used.
R=struct('status','UNKNOWN_MULTITICK_B_NOT_STARTED', ...
    'conditionally_proven_ticks',0,'requested_ticks',n_ticks, ...
    'ideal_closed_loop_full_horizon_proven',false, ...
    'tube_B_ideal_full_horizon_proven',false, ...
    'HLC_preclamp_ideal_full_horizon_proven',false, ...
    'actual_double_execution_roundoff_proven',false, ...
    'Flight_reference_switch_authorized',false, ...
    'safe_to_commit',false,'stage','INIT', ...
    'per_tick_ms',NaN(1,n_ticks),'per_tick_status',{cell(1,n_ticks)}, ...
    'tube_B_max_radii_m',NaN(1,3),'max_input_interval_4x2',NaN(4,2), ...
    'failure_tick_index',NaN,'elapsed_ms',NaN);
clk=tic;
if ~(isnumeric(n_ticks)&&isscalar(n_ticks)&& ...
        isfinite(n_ticks)&&n_ticks==floor(n_ticks)&&n_ticks>=1&& ...
        isfinite(dt)&&isscalar(dt)&&dt>0&& ...
        isfield(piece,'duration_s')&&isfield(piece,'t0_s')&& ...
        n_ticks*dt<=piece.duration_s && ...
        isobject(agent)&&isprop(agent,'cha')&&strcmp(char(agent.cha),'f'))
    R.status='UNKNOWN_MULTITICK_B_DOMAIN';R.elapsed_ms=1000*toc(clk);return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_MULTITICK_B_INTLAB_MISSING';
    R.elapsed_ms=1000*toc(clk);return
end
X=plant_24x2;E=prior_ekf_25x2;P=prior_cov_25x25x2;
U=prior_held_4x2;
L=double(agent.parameter.get('cableL'));
g=[0;0;-double(agent.parameter.get('gravity'))];
if ~(isscalar(L)&&isfinite(L)&&L>0&& ...
        all(isfinite(g))&& ...
        isequal(size(X),[24,2])&&isequal(size(E),[25,2])&& ...
        isequal(size(U),[4,2])&&isequal(size(P),[25,25,2])&& ...
        all(isfinite([X(:);E(:);P(:);U(:)])))
    R.status='UNKNOWN_MULTITICK_B_INVALID_INITIAL_BOX';
    R.elapsed_ms=1000*toc(clk);return
end
maxerr=[0 0 0];
mininput=inf(4,1);maxinput=-inf(4,1);
for k=1:n_ticks
    t=piece.t0_s+(k-1)*dt;te=piece.t0_s+k*dt;
    R.stage='OBSERVATION';R.failure_tick_index=k;
    M=TRAJECTORY_C9_SOURCE_IDEAL_MOTIVE_OBSERVATION_INTLAB(agent,X);
    if ~M.valid
        R.status=['UNKNOWN_MULTITICK_B_MOTIVE_' M.status];break
    end
    R.stage='REFERENCE';
    A=TRAJECTORY_C9_REFERENCE_28_FROM_PIECE(piece,t,L);
    if ~A.valid
        R.status=['UNKNOWN_MULTITICK_B_REF_' A.status];break
    end
    % Reference is the actual 28-vector after the original Flight sload
    % position offset, as consumed by HLC.  Its future generation and
    % active switching remain separate certification obligations.
    ref=[A.xd_at_HLC_28,A.xd_at_HLC_28];
    R.stage='CLOSED_TICK';
    C=TRAJECTORY_C9_SOURCE_CLOSED_TICK_B_INTLAB( ...
        agent,E,P,M.ideal_measurement_12x2,U,X,ref,observed,radii,dt);
    R.per_tick_ms(k)=C.elapsed_ms;
    R.per_tick_status{k}=C.status;
    if ~C.closed_one_tick_conditionally_proven
        R.status=['UNKNOWN_MULTITICK_B_TICK_' C.status];break
    end
    R.stage='TUBE_B_SEGMENT';
    B=TRAJECTORY_C9_27_INTERVAL_REFERENCE_SEGMENT_INTLAB( ...
        piece,t,te,g,L);
    if ~B.valid
        R.status=['UNKNOWN_MULTITICK_B_NOMINAL_' B.status];break
    end
    try
        Pbox=infsup(C.plant_all_24x2(:,1),C.plant_all_24x2(:,2));
        Uref=infsup(B.uav_3x2(:,1),B.uav_3x2(:,2));
        Lref=infsup(B.payload_3x2(:,1),B.payload_3x2(:,2));
        eu=Pbox(1:3)-Uref;
        el=Pbox(13:15)-Lref;
        ru=double(sup(sqrt(sum(eu.^2))));
        rl=double(sup(sqrt(sum(el.^2))));
        rc=max(ru,rl);
        if ~all(isfinite([ru,rl,rc]))
            R.status='UNKNOWN_MULTITICK_B_NONFINITE_RADIUS';break
        end
        maxerr=max(maxerr,[ru,rc,rl]);
    catch ME
        R.status=['UNKNOWN_MULTITICK_B_RADIUS_' ME.identifier];break
    end
    mininput=min(mininput,C.unsaturated_four_inputs_4x2(:,1));
    maxinput=max(maxinput,C.unsaturated_four_inputs_4x2(:,2));
    E=C.estimated_next_25x2;
    P=C.cov_next_25x25x2;
    X=C.plant_next_24x2;
    U=C.unsaturated_four_inputs_4x2;
    R.conditionally_proven_ticks=k;
end
if R.conditionally_proven_ticks==n_ticks
    R.ideal_closed_loop_full_horizon_proven=true;
    R.tube_B_ideal_full_horizon_proven=true;
    R.HLC_preclamp_ideal_full_horizon_proven=true;
    R.tube_B_max_radii_m=maxerr;
    R.max_input_interval_4x2=[mininput,maxinput];
    R.failure_tick_index=NaN;
    R.stage='COMPLETE_IDEAL_HORIZON';
    R.status='CONDITIONAL_IDEAL_SOURCE_MULTITICK_B_WITH_DIRECTED_INTERVALS__DOUBLE_ROUNDOFF_AND_FLIGHT_SWITCH_PENDING';
end
R.elapsed_ms=1000*toc(clk);
end
