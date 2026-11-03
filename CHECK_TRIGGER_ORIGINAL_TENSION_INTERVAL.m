function R=CHECK_TRIGGER_ORIGINAL_TENSION_INTERVAL()
% Disposable fixture test; no real EKF/controller/obstacle/world data.
R=struct('status','NOT_TESTED','hover_positive',false, ...
    'free_fall_not_certified',false,'pT_unit_invariant_identity',false, ...
    'never_commit',false,'reason','');
try
    x=zeros(24,1);p=zeros(21,1);
    p(1)=1.4;p(6:8)=[0.05;0.06;0.08];p(9)=9.80665;
    p(20)=0.5;p(21)=2;
    x(13:15)=[0;0;-2];x(19:21)=[0;0;-1];
    X=struct('lower',x,'upper',x);
    P=struct('lower',p,'upper',p);
    U=struct('lower',[(p(1)+p(20))*p(9);0;0;0], ...
        'upper',[(p(1)+p(20))*p(9);0;0;0]);
    A=TRIGGER_ORIGINAL_TENSION_INTERVAL(X,U,P);
    R.hover_positive=A.conditional_tension_positive && ...
        A.tension_lower_N>0 && ...
        A.tension_upper_N>=A.tension_lower_N;
    U.lower(1)=0;U.upper(1)=0;
    B=TRIGGER_ORIGINAL_TENSION_INTERVAL(X,U,P);
    R.free_fall_not_certified=~B.conditional_tension_positive;
    R.pT_unit_invariant_identity=A.original_pT_unit_invariant_identity;
    R.never_commit=~A.safe_to_commit && ~B.safe_to_commit && ...
        ~A.taut_cable_validated;
    assert(R.hover_positive && R.free_fall_not_certified && ...
        R.pT_unit_invariant_identity && R.never_commit, ...
        'CHECK_TRIGGER_ORIGINAL_TENSION_INTERVAL:Test', ...
        'Tension projection, loss-of-tension refusal or fail-closed status failed.');
    R.status='PASS_CONDITIONAL_ORIGINAL_TENSION_IDENTITY__NOT_FLIGHT_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_ORIGINAL_TENSION_INTERVAL_RESULT.mat','R');
f=fopen('TRIGGER_ORIGINAL_TENSION_INTERVAL_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\nhover_positive=%d\nfree_fall_rejected=%d\nunit_invariant_identity=%d\nnever_commit=%d\nreason=%s\n', ...
        R.status,R.hover_positive,R.free_fall_not_certified, ...
        R.pT_unit_invariant_identity,R.never_commit,R.reason);
    fclose(f);
end
end
