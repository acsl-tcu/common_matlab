function R=CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS()
% Offline audit: exact source-pinned manifold identities and 128 independent
% original 24-state interval RHS consistency cases. No Flight or MEX.
R=struct('status','FAIL_TAUT_SOURCE_INVARIANTS','reason','', ...
    'cases',0,'failures',0,'source_pinned',false, ...
    'formal_exact',false,'numeric_cases',0, ...
    'initial_tautness_claimed',false,'physical_safety_claimed',false);
try
    proof=TRIGGER_TAUT_SOURCE_INVARIANTS();
    R.cases=R.cases+1;
    assert(proof.algebra_exact && proof.unit_invariant_exact && ...
        proof.position_velocity_invariant_exact, ...
        'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:Algebra',proof.reason);
    R.formal_exact=true;
    R.cases=R.cases+1;
    assert(proof.source_bound_invariance, ...
        'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:SourcePin',proof.reason);
    R.source_pinned=true;
    R.cases=R.cases+1;
    assert(~proof.initial_on_manifold_proven && ...
        ~proof.positive_tension_proven && ...
        ~proof.hybrid_physical_mode_proven && ~proof.safe_to_commit, ...
        'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:Overclaim', ...
        'An algebraic ODE identity cannot certify true initial conditions.');
    for k=1:128
        x=zeros(24,1);u=zeros(4,1);param=ones(21,1);
        % Exactly representable dyadic states satisfying the joint taut
        % manifold: this is a synthetic algebra test, NOT a true-state seed.
        axis=mod(k-1,3)+1;
        p=zeros(3,1);p(axis)=(-1)^k;
        w=(mod((1:3)'*k,17)-8)/16;
        v=(mod((1:3)'*(k+7),19)-9)/16;
        q=(mod((1:3)'*(k+3),23)-11)/16;
        L=2;
        x(1:3)=q;x(4:6)=[0;mod(k,5)/32;0];
        x(7:9)=v;x(10:12)=w/2;
        x(13:15)=q+L*p;
        x(16:18)=v+L*cross(w,p);
        x(19:21)=p;x(22:24)=w;
        param(1)=1.5;param(6:8)=[1/4;1/2;3/4];
        param(9)=9.80665;param(20)=1/2;param(21)=L;
        u=[mod(k,9)+1;1/8;-1/16;1/4];
        Q=struct('lower',x,'upper',x);
        U=struct('lower',u,'upper',u);
        P=struct('lower',param,'upper',param);
        F=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS(Q,U,P);
        assert(F.outward_elementary_operations, ...
            'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:IntervalRHS',F.reason);
        d=cell(24,1);
        for j=1:24
            d{j}=TRIGGER_OUTWARD_INTERVAL(F.rhs_lower(j),F.rhs_upper(j));
        end
        direction=cell(3,1);wbox=cell(3,1);
        for j=1:3
            direction{j}=TRIGGER_OUTWARD_INTERVAL(p(j),p(j));
            wbox{j}=TRIGGER_OUTWARD_INTERVAL(w(j),w(j));
        end
        dp=crossi(wbox,direction);
        dw=d(22:24);
        prediction=crossi(dw,direction);
        second=crossi(wbox,dp);
        dotunit=TRIGGER_OUTWARD_INTERVAL(0,0);
        for j=1:3
            dotunit=dotunit+direction{j}.*d{18+j};
            % d/dt (l-q-L pT) must contain zero on source RHS.
            r0=d{12+j}-d{j}-L.*d{18+j};
            assert(r0.lo<=0&&r0.hi>=0, ...
                'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:Position', ...
                'Source kinematic identity fails at case %d, coordinate %d.',k,j);
            % d/dt(vL-vQ-L*(wL CROSS pT)) must contain zero.
            r1=d{15+j}-d{6+j}-L.*(prediction{j}+second{j});
            assert(r1.lo<=0&&r1.hi>=0, ...
                'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:Acceleration', ...
                'Source dynamic identity fails at case %d, coordinate %d.',k,j);
        end
        assert(dotunit.lo<=0&&dotunit.hi>=0, ...
            'CHECK_TRIGGER_TAUT_SOURCE_INVARIANTS:UnitNorm', ...
            'pT dot pT derivative did not enclose zero at case %d.',k);
        R.numeric_cases=R.numeric_cases+1;
        R.cases=R.cases+1;
    end
    R.status='PASS_EXACT_TAUT_ODE_INVARIANTS_SOURCE_BOUND__INITIAL_TRUTH_OPEN';
    R.reason=['Original pinned 24-state RHS has exact invariant taut manifold ' ...
        'for constant L, independently checked with original interval RHS ' ...
        'in 128 dyadic synthetic states. Real initial constraint, true ' ...
        'EKF set, applied inputs, tension and physical model still open.'];
catch ME
    R.failures=1;
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    save('TRIGGER_TAUT_SOURCE_INVARIANTS_RESULT.mat','R');
    fid=fopen('TRIGGER_TAUT_SOURCE_INVARIANTS_RESULT.txt','wt');
    if fid>=0
        fprintf(fid,'%s\ncases=%d failures=%d numeric=%d\n%s\n', ...
            R.status,R.cases,R.failures,R.numeric_cases,R.reason);
        fclose(fid);
    end
catch
end
end
function out=crossi(a,b)
out={a{2}.*b{3}-a{3}.*b{2}; ...
    a{3}.*b{1}-a{1}.*b{3}; ...
    a{1}.*b{2}-a{2}.*b{1}};
end
