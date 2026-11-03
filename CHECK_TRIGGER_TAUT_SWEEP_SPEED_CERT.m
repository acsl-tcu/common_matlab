function T=CHECK_TRIGGER_TAUT_SWEEP_SPEED_CERT()
% Disposable check in common_matlab; no real Flight proof.
T=struct('status','UNKNOWN_TAUT_SWEEP_FIXTURE','pass_cases',0, ...
    'reject_cases',0,'reason','');
try
    lo=zeros(24,2);hi=zeros(24,2);
    lo(19:21,:)=repmat([0;0;-1],1,2);
    hi(19:21,:)=lo(19:21,:);
    hi(7:9,:)=1;
    lo(7:9,:)=-1;
    hi(22:24,:)=0.2;
    lo(22:24,:)=-0.2;
    D=struct('tube_lower',lo,'tube_upper',hi, ...
        'hold_duration_s',[0.01 0.035], ...
        'cable_length_upper_m',2.0, ...
        'start_clearance_lower_m',[1 1]);
    P=TRIGGER_TAUT_SWEEP_SPEED_CERT(D);
    assert(P.conditional_clearance_pass && ~P.safe_to_commit && ...
        P.speed_swept_upper_mps(1)>sqrt(3), ...
        'CHECK_TRIGGER_TAUT_SWEEP_SPEED_CERT:Pass');
    T.pass_cases=T.pass_cases+1;
    D.start_clearance_lower_m=[1 0.005];
    N=TRIGGER_TAUT_SWEEP_SPEED_CERT(D);
    assert(~N.conditional_clearance_pass && ~N.safe_to_commit, ...
        'CHECK_TRIGGER_TAUT_SWEEP_SPEED_CERT:Reject');
    T.reject_cases=T.reject_cases+1;
    D.cable_length_upper_m=-2;
    U=TRIGGER_TAUT_SWEEP_SPEED_CERT(D);
    assert(~U.conditional_swept_bound && ~U.safe_to_commit, ...
        'CHECK_TRIGGER_TAUT_SWEEP_SPEED_CERT:Invalid');
    T.reject_cases=T.reject_cases+1;
    T.status='PASS_CONDITIONAL_TAUT_SWEEP_SPEED_FIXTURES__NOT_FLIGHT_SAFETY';
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_TAUT_SWEEP_SPEED_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\npass_cases=%d\nreject_cases=%d\nreason=%s\n', ...
        T.status,T.pass_cases,T.reject_cases,T.reason);
    fclose(fid);
end
end
