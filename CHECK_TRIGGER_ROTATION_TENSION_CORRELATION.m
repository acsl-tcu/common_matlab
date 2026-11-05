function R=CHECK_TRIGGER_ROTATION_TENSION_CORRELATION()
% Verify the EXACT original-plant Euler thrust projection and a global
% rotation-Lipschitz enclosure.  These are conditional taut-mode checks.
% No physical true-state or IEEE754/libm certification is claimed.
R=struct('status','FAIL_ROTATION_CORRELATION', ...
    'cases',0,'failures',0,'point_cases',0, ...
    'box_cases',0,'geometry_cases',0,'reason','');
try
    rng(416209,'twister');
    for k=1:200
        x=zeros(24,1);p=zeros(21,1);
        x(4:6)=1.0*(2*rand(3,1)-1);
        x(22:24)=randn(3,1);
        d=randn(3,1);d=d/norm(d);x(19:21)=d;
        p([1,6,7,8,9,20,21])=[1.1,0.03,0.04,0.05,9.80665,0.14,2];
        u=[0.2+18*rand;0;0;0];
        T=TRIGGER_ORIGINAL_TENSION_INTERVAL( ...
            struct('lower',x,'upper',x), ...
            struct('lower',u,'upper',u), ...
            struct('lower',p,'upper',p));
        assert(T.direct_original_source_formula && ...
            isfinite(T.thrust_axis_dot_lower) && ...
            isfinite(T.thrust_axis_dot_upper) && ...
            T.thrust_axis_dot_lower>=-1 && ...
            T.thrust_axis_dot_upper<=1,'Invalid axis projection box.');
        dx=with_load_model_euler_for_HL(x,u,p.');
        N=-p(20)*dot(d,dx(16:18)+[0;0;p(9)]);
        tol=1e-9*max(1,abs(N));
        assert(T.tension_lower_N<=N+tol && ...
               T.tension_upper_N>=N-tol, ...
            'Original-plant point tension excluded.');
        R.point_cases=R.point_cases+1;R.cases=R.cases+1;
    end
    for k=1:160
        x=zeros(24,1);p=zeros(21,1);
        x(4:6)=0.8*(2*rand(3,1)-1);
        x(22:24)=0.3*randn(3,1);
        d=randn(3,1);d=d/norm(d);x(19:21)=d;
        p([1,6,7,8,9,20,21])=[1.1,0.03,0.04,0.05,9.80665,0.14,2];
        u=[8+4*rand;0;0;0];
        rad=0.003+0.015*rand;
        lo=x;hi=x;
        lo(4:6)=x(4:6)-rad;hi(4:6)=x(4:6)+rad;
        lo(19:21)=d-2*rad;hi(19:21)=d+2*rad;
        lo(22:24)=x(22:24)-rad;hi(22:24)=x(22:24)+rad;
        T=TRIGGER_ORIGINAL_TENSION_INTERVAL( ...
            struct('lower',lo,'upper',hi), ...
            struct('lower',u,'upper',u), ...
            struct('lower',p,'upper',p));
        assert(T.direct_original_source_formula && ...
            T.thrust_axis_dot_lower<=T.thrust_axis_dot_upper && ...
            T.thrust_axis_dot_lower>=-1 && T.thrust_axis_dot_upper<=1, ...
            'Source axis correlation bound invalid.');
        for sample=1:8
            xx=x;xx(4:6)=lo(4:6)+rand(3,1).*(hi(4:6)-lo(4:6));
            v=d+0.1*rad*(2*rand(3,1)-1);
            xx(19:21)=v/norm(v);
            assert(all(xx(19:21)>=lo(19:21)) && ...
                all(xx(19:21)<=hi(19:21)), ...
                'Test unit-vector sample outside prescribed box.');
            xx(22:24)=lo(22:24)+rand(3,1).*(hi(22:24)-lo(22:24));
            dx=with_load_model_euler_for_HL(xx,u,p.');
            N=-p(20)*dot(xx(19:21),dx(16:18)+[0;0;p(9)]);
            tol=1e-9*max(1,abs(N));
            assert(T.tension_lower_N<=N+tol && ...
                   T.tension_upper_N>=N-tol, ...
                'Original-plant true unit-vector sample excluded.');
        end
        R.box_cases=R.box_cases+1;R.cases=R.cases+1;
    end
    for k=1:20
        x=zeros(24,1);p=zeros(21,1);
        x(19:21)=[0;0;-1];
        p([1,6,7,8,9,20,21])=[1.1,0.03,0.04,0.05,9.80665,0.14,2];
        u=[10;0;0;0];
        lo=x;hi=x;
        angleRad=0.04+0.004*k;
        lo(4:6)=-angleRad;hi(4:6)=angleRad;
        T=TRIGGER_ORIGINAL_TENSION_INTERVAL( ...
            struct('lower',lo,'upper',hi), ...
            struct('lower',u,'upper',u), ...
            struct('lower',p,'upper',p));
        assert(T.direct_original_source_formula && ...
            T.conditional_tension_positive && ...
            T.thrust_axis_dot_upper<0, ...
            'A vertical downward taut cable with moderate tilt must remain tension-positive.');
        R.geometry_cases=R.geometry_cases+1;R.cases=R.cases+1;
    end
    R.status='PASS_SOURCE_EULER_AXIS_AND_TAUT_TENSION__PHYSICAL_MODE_CONDITIONAL';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    save('TRIGGER_ROTATION_TENSION_CORRELATION_RESULT.mat','R');
    fid=fopen('TRIGGER_ROTATION_TENSION_CORRELATION_RESULT.txt','w');
    if fid>0
        fprintf(fid,'%s\ncases=%d failures=%d points=%d boxes=%d geometry=%d\n%s\n', ...
            R.status,R.cases,R.failures,R.point_cases,R.box_cases,R.geometry_cases,R.reason);
        fclose(fid);
    end
catch
end
end
