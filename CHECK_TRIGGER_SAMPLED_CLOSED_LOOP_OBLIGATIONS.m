function R=CHECK_TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS()
% Disposable ROOT-LEVEL test: algebraic reach chaining ONLY. No flight.
R=struct('status','FAIL_CONDITIONAL_CHAIN_FIXTURES','cases',0,'reason','');
try
    n=4;
    P=struct('time_grid_s',0:0.25:1, ...
        'initial_lower',zeros(24,1),'initial_upper',zeros(24,1), ...
        'tube_lower',-ones(24,n),'tube_upper',ones(24,n), ...
        'rhs_lower',zeros(24,n),'rhs_upper',zeros(24,n));
    P.initial_lower(15)=-2;P.initial_upper(15)=-2;
    P.tube_lower(15,:)=-3;P.tube_upper(15,:)=-1;
    far=OBSTACLE_INFORMATION.define(70001,'sphere',[100;0;0],1);
    P.observed_prepared_shapes=OBSTACLE_GEOMETRY_QUERY.prepare(far);
    P.body=struct('r_uav_m',0.2,'r_payload_m',0.2,'r_cable_m',0.01);
    C=TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS(P);
    assert(C.chain_algebra_pass && C.conditional_geometry_pass && ...
        ~C.safe_to_commit && ~C.continuous_closed_loop_safe_certified, ...
        'CHECK_TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Far', ...
        'Algebraically enclosed distant obstacle fixture failed.');
    R.cases=R.cases+1;
    P.tube_lower(1,2)=-0.1;P.tube_upper(1,2)=-0.05;
    C=TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS(P);
    assert(~C.chain_algebra_pass && ~C.safe_to_commit, ...
        'CHECK_TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Reject', ...
        'Invalid interval must fail closed.');
    R.cases=R.cases+1;
    P.tube_lower(1,2)=-1;P.tube_upper(1,2)=1;
    near=OBSTACLE_INFORMATION.define(70002,'sphere',[0;0;0],1);
    P.observed_prepared_shapes=OBSTACLE_GEOMETRY_QUERY.prepare(near);
    C=TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS(P);
    assert(C.chain_algebra_pass && ~C.conditional_geometry_pass && ...
        ~C.safe_to_commit, ...
        'CHECK_TRIGGER_SAMPLED_CLOSED_LOOP_OBLIGATIONS:Contact', ...
        'Nearby obstacle cannot be certified separated.');
    R.cases=R.cases+1;
    R.status='PASS_CONDITIONAL_CHAIN_FIXTURES__NOT_CLOSED_LOOP_SAFETY';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_CLOSED_LOOP_CHAIN_RESULT.mat','R');
f=fopen('TRIGGER_CLOSED_LOOP_CHAIN_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nreason=%s\n',R.status,R.cases,R.reason);
    fclose(f);
end
end
