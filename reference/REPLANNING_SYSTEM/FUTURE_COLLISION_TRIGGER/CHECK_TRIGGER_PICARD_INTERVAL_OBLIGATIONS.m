function T=CHECK_TRIGGER_PICARD_INTERVAL_OBLIGATIONS()
% Mathematical inclusion fixtures only: NEVER validates the actual plant.
T=struct('status','FAIL_CONDITIONAL_PICARD_FIXTURES', ...
    'pass',false,'checks',0,'reason','');
try
    base=struct('h_s',0.025,'x0_lower',zeros(24,1), ...
        'x0_upper',zeros(24,1),'tube_lower',-ones(24,1), ...
        'tube_upper',ones(24,1), ...
        'rhs_lower',-ones(24,1),'rhs_upper',ones(24,1));
    C=TRIGGER_PICARD_INTERVAL_OBLIGATIONS(base);
    assert(C.conditional_strict_inclusion && ...
        ~C.safe_to_commit && ~C.continuous_closed_loop_safe_certified);
    T.checks=T.checks+1;
    base.rhs_upper(1)=50;C=TRIGGER_PICARD_INTERVAL_OBLIGATIONS(base);
    assert(~C.conditional_strict_inclusion && ~C.safe_to_commit);
    T.checks=T.checks+1;
    base.rhs_upper(1)=1;base.x0_lower(3)=NaN;
    C=TRIGGER_PICARD_INTERVAL_OBLIGATIONS(base);
    assert(startsWith(C.status,'UNKNOWN') && ~C.safe_to_commit);
    T.checks=T.checks+1;
    base.x0_lower(3)=0;base.x0_upper=ones(23,1);
    C=TRIGGER_PICARD_INTERVAL_OBLIGATIONS(base);
    assert(startsWith(C.status,'UNKNOWN') && ~C.safe_to_commit);
    T.checks=T.checks+1;
    T.status='PASS_CONDITIONAL_PICARD_FIXTURES__NOT_CLOSED_LOOP_SAFETY';
    T.pass=true;
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_PICARD_AUDIT_RESULT.mat','T');
fid=fopen('TRIGGER_PICARD_AUDIT_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nchecks=%d\nreason=%s\n', ...
        T.status,T.checks,T.reason);fclose(fid);
end
end
