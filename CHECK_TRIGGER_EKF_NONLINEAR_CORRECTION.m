function R=CHECK_TRIGGER_EKF_NONLINEAR_CORRECTION()
% Synthetic enclosure of the ORIGINAL EKF's nonlinear innovation algebra.
% Does not certify actual noises, gain, projection or rounded arithmetic.
R=struct('status','NOT_COMPLETED_EKF_NONLINEAR_CORRECTION', ...
    'reason','','tested',0,'violations',0,'projection_missing_rejected',false, ...
    'original_ekf_proven',false,'safe_to_commit',false);
try
    e=zeros(24,1); e(1)=0.25; e(2)=-0.31;
    C=struct();
    C.estimated_prediction_lower=e-0.02;
    C.estimated_prediction_upper=e+0.02;
    C.measurement_lower=[1;0]-0.04;
    C.measurement_upper=[1;0]+0.04;
    C.predicted_measurement_lower=[0.8;0.1]-0.03;
    C.predicted_measurement_upper=[0.8;0.1]+0.03;
    G=zeros(24,2);G(1:2,1:2)=[0.2 0.1; -0.2 0.3];
    C.gain_lower=G-0.005;
    C.gain_upper=G+0.005;
    C.true_next_lower=e-0.1;
    C.true_next_upper=e+0.1;
    % Identity projection ONLY in this disposable numeric fixture.
    % Real EKF projection cannot be assumed identity.
    A=TRIGGER_EKF_NONLINEAR_CORRECTION_BOUND(C);
    assert(A.preprojection_enclosed && ...
        strcmp(A.status,'UNKNOWN_REAL_EKF_PROJECTION_NOT_BOUNDED') && ...
        ~A.safe_to_commit, ...
        'CHECK_TRIGGER_EKF_NONLINEAR_CORRECTION:ProjectionMissing');
    R.projection_missing_rejected=true;
    C.projected_lower=A.preprojection_lower;
    C.projected_upper=A.preprojection_upper;
    A=TRIGGER_EKF_NONLINEAR_CORRECTION_BOUND(C);
    assert(startsWith(A.status,'CONDITIONAL_EXACT_') && ...
        ~A.safe_to_commit, ...
        'CHECK_TRIGGER_EKF_NONLINEAR_CORRECTION:NotConditional');
    previous_rng=rng; rng_guard=onCleanup(@()rng(previous_rng)); %#ok<NASGU>
    rng(2468,'twister');
    for k=1:256
        pp=C.estimated_prediction_lower+rand(24,1).* ...
            (C.estimated_prediction_upper-C.estimated_prediction_lower);
        y=C.measurement_lower+rand(2,1).* ...
            (C.measurement_upper-C.measurement_lower);
        yh=C.predicted_measurement_lower+rand(2,1).* ...
            (C.predicted_measurement_upper-C.predicted_measurement_lower);
        gain=C.gain_lower+rand(24,2).* ...
            (C.gain_upper-C.gain_lower);
        corrected=pp+gain*(y-yh);
        truth=C.true_next_lower+rand(24,1).* ...
            (C.true_next_upper-C.true_next_lower);
        err=truth-corrected;
        if any(err<A.conditional_error_lower) || ...
                any(err>A.conditional_error_upper)
            R.violations=R.violations+1;
        end
        R.tested=k;
    end
    assert(R.violations==0, ...
        'CHECK_TRIGGER_EKF_NONLINEAR_CORRECTION:Violation', ...
        'Synthetic interval corrections must enclose numeric samples.');
    R.status='PASS_CONDITIONAL_NONLINEAR_EKF_INNOVATION__NOT_EKF_PROOF';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_EKF_NONLINEAR_CORRECTION_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ntested=%d\nviolations=%d\nprojection_missing_rejected=%d\nreason=%s\n', ...
        R.status,R.tested,R.violations,R.projection_missing_rejected,R.reason);
    fclose(fid);
end
end
