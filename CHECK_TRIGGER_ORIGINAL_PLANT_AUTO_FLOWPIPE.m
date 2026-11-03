function T=CHECK_TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE()
% Disposable common_matlab/ diagnostic; no actual Flight/obstacles/estimator.
T=struct('status','NOT_RUN_AUTO_PLANT_FLOWPIPE', ...
    'one_step_pass',false,'two_step_pass',false, ...
    'endpoint_inclusion',false,'invalid_mass_rejected',false, ...
    'unknown_mode_safe_to_commit',false, ...
    'iterations',zeros(1,0),'reason','');
try
    p=zeros(21,1);
    p(1)=1.4; p(6:8)=[0.05;0.06;0.08];
    p(9)=9.80665; p(20)=0.5; p(21)=2;
    x=zeros(24,1);
    x(13:15)=[0;0;-p(21)];
    x(19:21)=[0;0;-1];
    u=[(p(1)+p(20))*p(9);0;0;0];
    % These are synthetic fixtures; not measured HLC/plant uncertainty.
    r=1e-8;
    xlo=x-r;xhi=x+r;
    P=struct('time_grid_s',[0,0.002,0.004], ...
        'initial_lower',xlo,'initial_upper',xhi, ...
        'held_input_lower',repmat(u-r,1,2), ...
        'held_input_upper',repmat(u+r,1,2), ...
        'param_lower',repmat(p,1,2), ...
        'param_upper',repmat(p,1,2));
    P.param_lower(20,:)=p(20)-r;
    P.param_upper(20,:)=p(20)+r;
    A=TRIGGER_ORIGINAL_PLANT_AUTO_CHAIN(P);
    T.two_step_pass=A.conditional_chain_pass;
    T.iterations=A.iteration_counts;
    assert(A.conditional_chain_pass && A.completed_intervals==2, ...
        'CHECK_TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE:Chain',A.reason);
    B=P;B.time_grid_s=[0,0.002];
    B.held_input_lower=P.held_input_lower(:,1);
    B.held_input_upper=P.held_input_upper(:,1);
    B.param_lower=P.param_lower(:,1);B.param_upper=P.param_upper(:,1);
    C=TRIGGER_ORIGINAL_PLANT_AUTO_CHAIN(B);
    T.one_step_pass=C.conditional_chain_pass;
    assert(T.one_step_pass,'CHECK_TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE:OneStep', ...
        'Synthetic single step did not find a tube.');
    % Independent one-step computation must reproduce the exact first
    % segment endpoint used to seed the second segment, with no reset.
    T.endpoint_inclusion=isequal(C.endpoint_lower(:,2),A.endpoint_lower(:,2)) && ...
        isequal(C.endpoint_upper(:,2),A.endpoint_upper(:,2));
    assert(T.endpoint_inclusion, ...
        'CHECK_TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE:Seam', ...
        'The two-step chain does not inherit the first-step endpoint.');
    E=B;E.param_lower(20)=-0.1;
    bad=TRIGGER_ORIGINAL_PLANT_AUTO_CHAIN(E);
    T.invalid_mass_rejected=~bad.conditional_chain_pass && ~bad.safe_to_commit;
    assert(T.invalid_mass_rejected,'CHECK_TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE:Mass', ...
        'Negative payload mass was not rejected.');
    T.unknown_mode_safe_to_commit=~A.safe_to_commit && ...
        ~C.safe_to_commit;
    assert(T.unknown_mode_safe_to_commit, ...
        'CHECK_TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE:UnsafeApproval', ...
        'A conditional result must never authorize Flight.');
    T.status='PASS_SYNTHETIC_AUTO_PICARD_CHAIN__NOT_CLOSED_LOOP_SAFETY';
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE_RESULT.mat','T');
fid=fopen('TRIGGER_ORIGINAL_PLANT_AUTO_FLOWPIPE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\none_step=%d\ntwo_step=%d\nseam=%d\ninvalid_mass_rejected=%d\nnever_commit=%d\niterations=%s\nreason=%s\n', ...
        T.status,T.one_step_pass,T.two_step_pass,T.endpoint_inclusion, ...
        T.invalid_mass_rejected,T.unknown_mode_safe_to_commit, ...
        mat2str(T.iterations),T.reason);
    fclose(fid);
end
end
