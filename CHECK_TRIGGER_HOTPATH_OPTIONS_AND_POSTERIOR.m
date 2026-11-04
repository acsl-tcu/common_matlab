function R=CHECK_TRIGGER_HOTPATH_OPTIONS_AND_POSTERIOR()
% Disposable common_matlab auditor for a SAME-result hotpath and a new
% ORIGINAL-plant-to-ACTUAL-EKF-posterior recursion. No Flight safety claim.
% Results always saved next to this CHECK_* file, not the current pwd.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_HOTPATH_OPTIONS_AND_POSTERIOR','cases',0, ...
    'failures',0,'reason','', 'parity_cases',0, ...
    'snapshot_config_case',false, ...
    'picard_attempted',false,'picard_conditional_success',false, ...
    'invalid_physics_refused',false, 'stale_box_invalidated',false, ...
    'safe_to_commit',false);
old_rng=rng; clean_rng=onCleanup(@()rng(old_rng)); %#ok<NASGU>
try
    rng(20261105,'twister');
    opt=struct('diagnostic_timing_enabled', ...
        strcmp(getenv('REPLANNING_STAGE_TIMING'),'1'), ...
        'sampled_block_experiment_enabled', ...
        strcmp(getenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT'),'1'));
    core=FUTURE_COLLISION_TRIGGER(struct('horizon_s',4));
    assert(isequal(core.sampled_geometry_options,opt), ...
        'CHECK_TRIGGER_HOTPATH:OptionsSnapshot');
    R.snapshot_config_case=true;
    R.cases=R.cases+1;
    types={'sphere','box','cylinder','ellipsoid','torus'};
    for k=1:240
        type=types{1+mod(k-1,numel(types))};
        shape=make_shape(type,k);
        n=1+mod(k,29);
        u=shape.center0+randn(3,n)*2.8;
        l=u+repmat([0;0;-2],1,n);
        early=mod(k,2)==0;
        [a,b,c,d,info1]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            u,l,shape,0.21,0.19,0.015,0,early);
        [e,f,g,h,info2]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            u,l,shape,0.21,0.19,0.015,0,early,opt);
        % Timing values are intentionally wall-clock measurements: their
        % numerical values cannot be identical for sequential calls.
        % Every classification and every work COUNT still must agree.
        if isfield(info1,'diagnostic_stage_ms')
            info1=rmfield(info1,'diagnostic_stage_ms');
        end
        if isfield(info2,'diagnostic_stage_ms')
            info2=rmfield(info2,'diagnostic_stage_ms');
        end
        assert(isequal(a,e)&&isequal(b,f)&&isequal(c,g)&& ...
            isequal(d,h)&&isequaln(info1,info2), ...
            'CHECK_TRIGGER_HOTPATH:ClassificationChanged', ...
            'Runtime options changed the exact sampled output case %d.',k);
        R.cases=R.cases+1;
        R.parity_cases=R.parity_cases+1;
    end
    % The output must not be silently promoted to a Flight safety proof.
    assert(~R.safe_to_commit,'CHECK_TRIGGER_HOTPATH:UnsafeCommit');
    R.cases=R.cases+1;
    % A real seeded true-state enclosure can be propagated with the
    % ORIGINAL Picard RHS; no more hand-fed true NEXT-state box is accepted.
    % A 1-step synthetic trial uses explicit parameters, never estimates
    % them from EKF covariance. Its success is CONDITIONAL, not a proof.
    x0=zeros(24,1);x0(3)=2;x0(19:21)=[0;0;-1];
    x0(13:15)=[0;0;0];
    param=zeros(21,1);
    param(1)=1.0;param(6:8)=[0.05;0.05;0.08];
    param(9)=9.80665;param(20)=0.5;param(21)=2.0;
    uhold=[14.709975;0;0;0];
    tracker=TRIGGER_EKF_TRUE_SET_TRACKER();
    tracker.seed(0,x0,[x0 x0],'SYNTHETIC_PHYSICAL_FIXTURE_NOT_TRUE_PROOF');
    step=tracker.advance_original_held_plant(0,1e-6,x0, ...
        uhold,uhold,param,param,'SYNTHETIC_HELD_INPUT');
    R.picard_attempted=true;
    R.picard_conditional_success=step.conditional_advanced;
    if step.conditional_advanced
        s=tracker.snapshot();
        assert(s.initialized && isequal(s.time_s,1e-6) && ...
            isequal(size(s.error_lower),[24 1]) && ...
            all(s.true_lower<=s.true_upper) && ...
            all(s.error_lower<=s.error_upper) && ...
            all(s.error_lower<=s.true_lower-x0+1e-10) && ...
            all(s.error_upper>=s.true_upper-x0-1e-10) && ...
            ~s.safe_to_commit && ~s.actual_closed_loop_proven, ...
            'CHECK_TRIGGER_HOTPATH:PosteriorInterval');
    else
        % A failed Picard proof MUST invalidate the previous true box.
        failed_snapshot=tracker.snapshot();
        assert(~failed_snapshot.initialized, ...
            'CHECK_TRIGGER_HOTPATH:StaleTrueBoxRetained');
    end
    R.cases=R.cases+1;
    tracker=TRIGGER_EKF_TRUE_SET_TRACKER();
    tracker.seed(0,x0,[x0 x0],'SYNTHETIC_PHYSICAL_FIXTURE_NOT_TRUE_PROOF');
    bad=tracker.advance_original_held_plant(0,0.01,x0, ...
        zeros(3,1),zeros(3,1),param,param,'BAD_ACTUATOR_SIZE');
    failed_snapshot=tracker.snapshot();
    assert(~bad.conditional_advanced && ~failed_snapshot.initialized, ...
        'CHECK_TRIGGER_HOTPATH:InvalidActuatorAccepted');
    R.invalid_physics_refused=true;R.stale_box_invalidated=true;
    R.cases=R.cases+1;
    tracker.seed(0,x0,[x0 x0],'SYNTHETIC_PHYSICAL_FIXTURE_NOT_TRUE_PROOF');
    bad=tracker.advance_original_held_plant(0.01,0.02,x0, ...
        uhold,uhold,param,param,'BAD_HOLD_CLOCK');
    failed_snapshot=tracker.snapshot();
    assert(~bad.conditional_advanced && ~failed_snapshot.initialized, ...
        'CHECK_TRIGGER_HOTPATH:StaleHoldAccepted');
    R.cases=R.cases+1;
    % A short conditional time grid must NEVER count as a full 4-second
    % continuous-time trigger, independent of whether physics is known.
    D=struct('observation_provenance','ACTUAL_SENSOR_HISTORY_ONLY', ...
        'observed_prepared_shapes',struct('id',91), ...
        'time_grid_s',[0 3.999]);
    short=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(D);
    assert(strcmp(short.status,'UNKNOWN_INCOMPLETE_FOUR_SECOND_HORIZON') && ...
        isequal(short.possible_risk_ids,91) && ~short.safe_to_commit, ...
        'CHECK_TRIGGER_HOTPATH:ShortHorizonAccepted');
    R.cases=R.cases+1;
    D.time_grid_s=[0 4];
    complete_but_unproved=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(D);
    assert(~complete_but_unproved.safe_to_commit && ...
        ~strcmp(complete_but_unproved.status, ...
        'UNKNOWN_INCOMPLETE_FOUR_SECOND_HORIZON'), ...
        'CHECK_TRIGGER_HOTPATH:NoPhysicalBoundsAccepted');
    R.cases=R.cases+1;
    % Connect the stateful ORIGINAL-plant/EKF error recursion to the
    % 4-second source/mass/geometry assembly, never a separate paper-only
    % error box. Here the seed is synthetic: no physical certificate.
    linked=TRIGGER_EKF_TRUE_SET_TRACKER();
    linked.seed(0,x0,[x0 x0],'SYNTHETIC_NOT_PHYSICALLY_PROVED');
    names={'p','q','v','w','pL','vL','pT','wL'};
    e=struct();
    for j=1:8
        e.(names{j})=x0(3*j-2:3*j);
    end
    D.ekf_now=e;
    D.true_state_tracker=linked;
    bridge=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(D);
    assert(bridge.tracker_error_derived && ...
        bridge.tracker_error_generation==uint64(1) && ...
        ~bridge.safe_to_commit && ~bridge.full_physical_4s_safety_proven, ...
        'CHECK_TRIGGER_HOTPATH:NoTrackerToClosedLoopLink');
    R.cases=R.cases+1;
    badD=D;badD.ekf_now.p(1)=badD.ekf_now.p(1)+1;
    bad=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(badD);
    assert(~bad.tracker_error_derived && ~bad.safe_to_commit, ...
        'CHECK_TRIGGER_HOTPATH:MismatchedActualEKFAccepted');
    R.cases=R.cases+1;
    badD=D;badD.time_grid_s=[1 5];
    bad=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(badD);
    assert(~bad.tracker_error_derived && ~bad.safe_to_commit, ...
        'CHECK_TRIGGER_HOTPATH:StaleTrackerTimeAccepted');
    R.cases=R.cases+1;
    badD=D;badD.error_lower=zeros(24,1);
    bad=TRIGGER_CLOSED_LOOP_FROM_SOURCE_LIMITS(badD);
    assert(~bad.tracker_error_derived && ~bad.safe_to_commit, ...
        'CHECK_TRIGGER_HOTPATH:ConflictingErrorBoxesAccepted');
    R.cases=R.cases+1;
    R.status='PASS_HOTPATH_PARITY_AND_CONDITIONAL_POSTERIOR__NO_FLIGHT_CERT';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_HOTPATH_POSTERIOR_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_HOTPATH_POSTERIOR_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\nparity_cases=%d\n', ...
        R.status,R.cases,R.failures,R.parity_cases);
    fprintf(f,'picard_conditional_success=%d\ninvalid_physics_refused=%d\n', ...
        R.picard_conditional_success,R.invalid_physics_refused);
    fprintf(f,'reason=%s\n',R.reason);fclose(f);
end
end
function s=make_shape(type,k)
ang=2*pi*rand;R=[cos(ang) -sin(ang) 0;sin(ang) cos(ang) 0;0 0 1];
c=randn(3,1);
s=struct('id',k,'type',type,'dimensions',[], ...
    'center0',c,'rotation',R,'velocity',zeros(3,1), ...
    'reference_time_s',0,'bounding_radius_m',NaN, ...
    'is_stationary',true,'center_norm_m',norm(c), ...
    'outline',zeros(0,2),'vertices',zeros(0,3),'faces',zeros(0,3));
switch type
    case 'sphere'
        s.dimensions=0.5+rand;s.bounding_radius_m=s.dimensions;
    case 'box'
        s.dimensions=[1;2;0.8];s.bounding_radius_m=norm(s.dimensions)/2;
    case 'cylinder'
        s.dimensions=[0.8;2.3];s.bounding_radius_m=hypot(0.8,1.15);
    case 'ellipsoid'
        s.dimensions=[0.5;1.4;0.9];s.bounding_radius_m=1.4;
    case 'torus'
        s.dimensions=[3.8;0.6];s.bounding_radius_m=4.4;
end
end
