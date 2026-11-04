function A=CHECK_TRIGGER_EKF_INTERVAL_RISK_DECISION()
A=struct('status','NOT_TESTED','cases',0,'failures',0,'reason','');
try
    % Two observed obstacles, arbitrary positive time grid and error bounds.
    C=struct('sample_times_s',[0 0.11 0.43 0.71], ...
        'observed_ids',[911 17], ...
        'nominal_clearance_lower_m',10*ones(2,4), ...
        'full_body_endpoint_error_upper_m',0.02*ones(1,4), ...
        'whole_body_speed_upper_mps',ones(1,3));
    R=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(strcmp(R.status,'CONDITIONAL_ALL_OBSERVED_INTERVALS_SEPARATED') && ...
        ~R.conditional_replan_required && R.certified_clear_intervals==3 && ...
        ~R.safe_to_commit);
    A.cases=A.cases+1;
    % Unknown first interval of the second obstacle; return only its ID.
    C.nominal_clearance_lower_m(2,1:2)=0;
    R=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(R.conditional_replan_required && R.obstacle_id==17 && ...
        isequal([R.first_possible_ttc_left_s R.first_possible_ttc_right_s], ...
                [0 .11]));
    A.cases=A.cases+1;
    % First interval now clear, second uncertain; no time-0 false alarm.
    C.nominal_clearance_lower_m(2,1:2)=10;
    C.nominal_clearance_lower_m(1,2:3)=0;
    R=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(R.obstacle_id==911 && ...
        isequal([R.first_possible_ttc_left_s R.first_possible_ttc_right_s], ...
                [.11 .43]));
    A.cases=A.cases+1;
    % No observed IDs means unknown world, not demonstrated safe.
    C.observed_ids=[];
    C.nominal_clearance_lower_m=zeros(0,4);
    R=TRIGGER_EKF_INTERVAL_RISK_DECISION(C);
    assert(~R.conditional_replan_required && contains(R.status,'UNKNOWN_WORLD') && ...
        ~R.safe_to_commit);
    A.cases=A.cases+1;
    A.status='PASS_EKF_INTERVAL_TO_EARLIEST_RISK__EXTERNAL_BOUNDS_UNPROVEN';
catch ME
    A.status='FAIL_INTERVAL_GATE';
    A.failures=A.failures+1;
    A.reason=[ME.identifier ' | ' ME.message];
end
f=fopen('TRIGGER_EKF_INTERVAL_RISK_RESULT.txt','w');
assert(f>=0);cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'status=%s\ncases=%d\nfailures=%d\nreason=%s\n', ...
    A.status,A.cases,A.failures,A.reason);
end
