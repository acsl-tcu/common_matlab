function R=CHECK_TRIGGER_TAUT_CABLE_MANIFOLD()
% Disposable theorem precondition test, not a Flight certificate.
R=struct('status','NOT_TESTED','cases',0,'reason','', ...
    'safe_to_commit',false);
try
    x=zeros(24,1);x(13:15)=[0;0;-2];x(19:21)=[0;0;-1];
    p=zeros(21,1);p(21)=2;
    X=struct('lower',x,'upper',x);
    P=struct('lower',p,'upper',p);
    A=TRIGGER_TAUT_CABLE_MANIFOLD_OBLIGATIONS(X,P);
    assert(A.compatible && ~A.exact_manifold_verified && ~A.safe_to_commit);
    R.cases=R.cases+1;
    X.lower(13)=0.5;X.upper(13)=0.5;
    B=TRIGGER_TAUT_CABLE_MANIFOLD_OBLIGATIONS(X,P);
    assert(~B.compatible && ~B.position_zero_possible);
    R.cases=R.cases+1;
    X.lower=x;X.upper=x;X.lower(16)=0.4;X.upper(16)=0.4;
    B=TRIGGER_TAUT_CABLE_MANIFOLD_OBLIGATIONS(X,P);
    assert(~B.compatible && ~B.velocity_zero_possible);
    R.cases=R.cases+1;
    X.lower=x;X.upper=x;X.lower(21)=-0.6;X.upper(21)=-0.6;
    B=TRIGGER_TAUT_CABLE_MANIFOLD_OBLIGATIONS(X,P);
    assert(~B.compatible && ~B.unit_direction_possible);
    R.cases=R.cases+1;
    R.status='PASS_NECESSARY_CABLE_CONSTRAINTS__NOT_CLOSED_LOOP_SAFETY';
catch ME
    R.status='FAIL_CABLE_CONSTRAINT_THEOREM_FIXTURES';
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_CABLE_MANIFOLD_RESULT.mat','R');
fid=fopen('TRIGGER_CABLE_MANIFOLD_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncases=%d\nreason=%s\n',R.status,R.cases,R.reason);
    fclose(fid);
end
end
