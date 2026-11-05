function R=CHECK_TRIGGER_DIRECT_SOURCE_TENSION()
% Compare derived source formula to ORIGINAL MATLAB load acceleration.
% Exact unit pT and original plant are necessary; test does not certify
% an experimental physical cable or numerical ODE solver.
R=struct('status','FAIL_DIRECT_SOURCE_TENSION','cases',0, ...
    'failures',0,'point_cases',0,'box_cases',0, ...
    'positive_example',false,'reason','');
try
    rng(29011,'twister');
    for k=1:260
        x=zeros(24,1); u=zeros(4,1); p=zeros(21,1);
        x(4:6)=0.2*randn(3,1);
        x(22:24)=0.6*randn(3,1);
        v=randn(3,1);x(19:21)=v/norm(v);
        x(10:12)=0.5*randn(3,1);
        p([1,6,7,8,9,20,21])= ...
            [1+rand;0.03;0.04;0.05;9.80665;0.1+rand/3;2];
        u=[20*rand;randn(3,1)];
        X=struct('lower',x,'upper',x);
        U=struct('lower',u,'upper',u);
        P=struct('lower',p,'upper',p);
        T=TRIGGER_ORIGINAL_TENSION_INTERVAL(X,U,P);
        assert(T.direct_original_source_formula && ...
            all(isfinite([T.tension_lower_N T.tension_upper_N])), ...
            'Direct-source tension interval failed.');
        dx=with_load_model_euler_for_HL(x,u,p.');
        N=-p(20)*dot(x(19:21),dx(16:18)+[0;0;p(9)]);
        tol=1e-9*max(1,abs(N));
        assert(N>=T.tension_lower_N-tol && ...
            N<=T.tension_upper_N+tol, ...
            'Original source tension not included by derived interval.');
        R.point_cases=R.point_cases+1;R.cases=R.cases+1;
    end
    for k=1:80
        x=zeros(24,1);x(4:6)=0.03*randn(3,1);
        v=randn(3,1);x(19:21)=v/norm(v);x(22:24)=0.1*randn(3,1);
        p=zeros(21,1);p([1 6 7 8 9 20 21])= ...
            [1.1,0.03,0.04,0.05,9.80665,0.14,2];
        u=[10;0;0;0];
        r=0.004;
        lo=x;hi=x;lo(4:6)=x(4:6)-r;hi(4:6)=x(4:6)+r;
        lo(22:24)=x(22:24)-r;hi(22:24)=x(22:24)+r;
        ul=u;uh=u;ul(1)=9.7;uh(1)=10.3;
        pl=p;pu=p;pl(20)=.13;pu(20)=.15;
        T=TRIGGER_ORIGINAL_TENSION_INTERVAL( ...
            struct('lower',lo,'upper',hi), ...
            struct('lower',ul,'upper',uh), ...
            struct('lower',pl,'upper',pu));
        assert(T.direct_original_source_formula,'Tension interval invalid.');
        for j=1:8
            xx=lo+rand(24,1).*(hi-lo);
            pp=pl+rand(21,1).*(pu-pl);
            uu=ul+rand(4,1).*(uh-ul);
            dx=with_load_model_euler_for_HL(xx,uu,pp.');
            N=-pp(20)*dot(xx(19:21),dx(16:18)+[0;0;pp(9)]);
            tol=1e-9*max(1,abs(N));
            assert(N>=T.tension_lower_N-tol && ...
                N<=T.tension_upper_N+tol, ...
                'Sample from original plant not inside source-direct tension.');
        end
        R.box_cases=R.box_cases+1;R.cases=R.cases+1;
    end
    x=zeros(24,1);x(19:21)=[0;0;-1];
    u=[10;0;0;0];
    p=zeros(21,1);p([1,6,7,8,9,20,21])= ...
        [1.1,0.03,0.04,0.05,9.80665,0.14,2];
    T=TRIGGER_ORIGINAL_TENSION_INTERVAL( ...
        struct('lower',x,'upper',x), ...
        struct('lower',u,'upper',u), ...
        struct('lower',p,'upper',p));
    assert(T.conditional_tension_positive && T.tension_lower_N>0, ...
        'Exact vertical taut source thrust case must have positive tension.');
    R.positive_example=true;R.cases=R.cases+1;
    R.status='PASS_ORIGINAL_SOURCE_DIRECT_TENSION__TAUT_MODE_CONDITIONAL';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    save('TRIGGER_DIRECT_SOURCE_TENSION_RESULT.mat','R');
    fid=fopen('TRIGGER_DIRECT_SOURCE_TENSION_RESULT.txt','w');
    if fid>0
        fprintf(fid,'%s\ncases=%d failures=%d points=%d boxes=%d\n%s\n', ...
            R.status,R.cases,R.failures,R.point_cases,R.box_cases,R.reason);
        fclose(fid);
    end
catch
end
end
