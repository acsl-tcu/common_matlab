function T=CHECK_TRIGGER_EKF_ERROR_SET_RECURSION()
% Disposable synthetic numerical regression, NOT an original-EKF proof.
T=struct('status','NOT_COMPLETED_EKF_ERROR_FIXTURE', ...
    'tested',0,'violations',0,'missing_rejected',false, ...
    'safe_to_commit',false,'reason','');
try
    previous_rng=rng;
    rng_guard=onCleanup(@()rng(previous_rng)); %#ok<NASGU>
    rng(16031,'twister');
    A=0.01*randn(24,24);
    al=A-0.002;au=A+0.002;
    el=-0.01*ones(24,1);eu=0.02*ones(24,1);
    dl=-1e-3*ones(24,1);du=2e-3*ones(24,1);
    C=struct('time_before_s',3.271,'time_after_s',3.2815, ...
        'error_lower',el,'error_upper',eu, ...
        'A_lower',al,'A_upper',au, ...
        'disturbance_lower',dl,'disturbance_upper',du);
    R=TRIGGER_EKF_ERROR_SET_RECURSION(C);
    assert(R.conditional_enclosure && ~R.safe_to_commit, ...
        'CHECK_TRIGGER_EKF_ERROR_SET_RECURSION:ConditionalOnly');
    for k=1:256
        ew=el+(eu-el).*rand(24,1);
        Aw=al+(au-al).*rand(24,24);
        dw=dl+(du-dl).*rand(24,1);
        y=Aw*ew+dw;
        if any(y<R.next_error_lower)||any(y>R.next_error_upper)
            T.violations=T.violations+1;
        end
        T.tested=k;
    end
    bad=C;bad=rmfield(bad,'disturbance_upper');
    failure=TRIGGER_EKF_ERROR_SET_RECURSION(bad);
    T.missing_rejected=~failure.conditional_enclosure && ...
        ~failure.safe_to_commit;
    assert(T.violations==0 && T.missing_rejected, ...
        'CHECK_TRIGGER_EKF_ERROR_SET_RECURSION:Fixture', ...
        'The outer error-box recurrence or the missing-data gate failed.');
    T.status='PASS_CONDITIONAL_EKF_ERROR_STEP__NOT_ORIGINAL_EKF_THEOREM';
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_EKF_ERROR_STEP_RESULT.mat','T');
f=fopen('TRIGGER_EKF_ERROR_STEP_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\ntested=%d\nviolations=%d\nmissing_rejected=%d\nreason=%s\n', ...
        T.status,T.tested,T.violations,T.missing_rejected,T.reason);
    fclose(f);
end
end
