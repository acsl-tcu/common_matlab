function R=CHECK_TRIGGER_EKF_TRUE_SET_CHAIN()
% Check that the REAL EKF posterior is used, and the true-state Picard
% induction cannot advance without the required interval data.
% The test source is synthetic and does NOT prove the actual plant bound.
R=struct('status','NOT_TESTED','cases',0,'failures',0,'reason','');
try
    % One-ULP operations: zero, negative and positive neighborhoods.
    a=[-2;-eps;0;eps;2];
    up=TRIGGER_FLOAT_OUTWARD_ONE_ULP(a,+1);
    dn=TRIGGER_FLOAT_OUTWARD_ONE_ULP(a,-1);
    assert(all(up>a)&&all(dn<a));
    s=TRIGGER_EKF_TRUE_SET_TRACKER();
    rngsave=rng;restore=onCleanup(@()rng(rngsave)); %#ok<NASGU>
    rng(818,'twister');
    for k=1:100
        s.invalidate('TEST_RESET');
        xhat0=randn(24,1);
        rad=0.001+0.1*rand(24,1);
        xlo=xhat0-rad;xhi=xhat0+rad;
        S=s.seed(0,xhat0,[xlo xhi],'SYNTHETIC_TEST_ONLY');
        assert(S.initialized && ~S.safe_to_commit && ...
            all(S.error_lower<=xlo-xhat0) && all(S.error_upper>=xhi-xhat0));
        h=0.01+0.02*rand;
        fl=-0.8*ones(24,1);fu=0.8*ones(24,1);
        B=struct('h_s',h,'x0_lower',xlo,'x0_upper',xhi, ...
            'rhs_lower',fl,'rhs_upper',fu, ...
            'tube_lower',xlo-2, 'tube_upper',xhi+2);
        xhat1=xhat0+0.1*randn(24,1);
        C=TRIGGER_EKF_PICARD_POSTERIOR_STEP(s,h,xhat1,B, ...
            'SYNTHETIC_BOUNDED_TRUE_RHS_ONLY');
        assert(strcmp(C.status, ...
            'CONDITIONAL_PICARD_TRUE_STATE_TO_LIVE_EKF_POSTERIOR') && ...
            C.strict_inclusion && ~C.safe_to_commit);
        E=s.snapshot();
        actual_x0=xlo+(xhi-xlo).*rand(24,1);
        actual_rhs=fl+(fu-fl).*rand(24,1);
        actual_x1=actual_x0+h*actual_rhs;
        actual_e=actual_x1-xhat1;
        assert(all(actual_e>=E.error_lower) && ...
               all(actual_e<=E.error_upper), ...
            'CHECK_TRIGGER_EKF_TRUE_SET_CHAIN:Containment', ...
            'Synthetic true state fell outside the EKF error enclosure.');
        T=TRIGGER_EKF_TAUT_ANCHOR_ERROR(E);
        eq=actual_e(1:3);el=actual_e(13:15);
        alpha=rand;
        ec=(1-alpha)*eq+alpha*el;
        assert(norm(ec)<=T.rho_full_cable_m && ...
            T.rho_full_cable_m>=T.rho_uav_m && ...
            T.rho_full_cable_m>=T.rho_payload_m && ...
            ~T.safe_to_commit);
        R.cases=R.cases+1;
    end
    % A failed strict-inclusion candidate must never update the tracker.
    S=s.snapshot();
    B.tube_lower=B.x0_lower;
    B.tube_upper=B.x0_upper;
    B.x0_lower=S.true_lower;
    B.x0_upper=S.true_upper;
    B.tube_lower=S.true_lower;
    B.tube_upper=S.true_upper;
    C=TRIGGER_EKF_PICARD_POSTERIOR_STEP(s,S.time_s+h,xhat1,B,'FAIL_TEST');
    assert(~C.safe_to_commit && ~C.strict_inclusion && ...
        isequal(s.generation,S.generation));
    R.status='PASS_ACTUAL_POSTERIOR_ERROR_SET_CHAIN__EXTERNAL_PHYSICS_UNPROVEN';
catch ME
    R.failures=R.failures+1;
    R.status='FAIL_INTERVAL_LINK';
    R.reason=[ME.identifier ' | ' ME.message];
end
f=fopen('TRIGGER_EKF_TRUE_SET_CHAIN_RESULT.txt','w');
assert(f>=0);cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'status=%s\ncases=%d\nfailures=%d\nreason=%s\n', ...
    R.status,R.cases,R.failures,R.reason);
end
