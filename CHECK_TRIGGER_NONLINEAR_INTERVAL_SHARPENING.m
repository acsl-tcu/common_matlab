function A=CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING()
% Analytic interval-extension regression for the original 24-state RHS.
% An actual noncontact certificate is NOT implied by this isolated audit.
% In particular, this does not certify the numerical ode15s trajectory,
% full observed-geometry distance or true hybrid taut-cable mode.
A=struct('status','FAIL_NONLINEAR_INTERVAL_SHARPENING', ...
    'reason','','cases',0,'failures',0, ...
    'even_power_cases',0,'odd_power_cases',0, ...
    'trigonometric_cases',0,'chart_restored',false, ...
    'original_rhs_finite',false,'safe_to_commit',false);
try
    % Representative exact dyadic inputs include intervals crossing zero.
    % Since these bounds are exact dyadics, test point-power witnesses
    % against the rigorous outward formulas at 0, endpoints and midpoints.
    exponent=[0 1 2 3 4 5 6 7 8];
    for k=1:280
        left=(mod(71*k,513)-256)/128;
        right=left+(mod(23*k,127))/128;
        x=TRIGGER_OUTWARD_INTERVAL(left,right);
        for n=exponent
            y=x.^n;
            pts=[left,right,0.5*(left+right)];
            if left<=0&&right>=0,pts(end+1)=0;end %#ok<AGROW>
            for t=pts
                % Ordinary double powers are used ONLY as witnesses,
                % not as the proof of directed transcendental rounding.
                v=t^n;
                tol=64*eps(max(1,abs(v)));
                assert(y.lo<=v+tol && y.hi>=v-tol, ...
                    'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:PowerWitness', ...
                    'Failure on k=%d n=%d x=[%.17g,%.17g].',k,n,left,right);
            end
            if n>0 && mod(n,2)==0
                assert(y.lo>=0, ...
                    'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:EvenNegative', ...
                    'Even power has a negative mathematical interval lower bound.');
                if left<0&&right>0
                    assert(y.lo==0, ...
                        'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:ZeroMinimum');
                end
                A.even_power_cases=A.even_power_cases+1;
            elseif mod(n,2)==1
                A.odd_power_cases=A.odd_power_cases+1;
            end
            A.cases=A.cases+1;
        end
    end
    s=TRIGGER_OUTWARD_INTERVAL(-1,1).^2;
    assert(s.lo==0 && s.hi>=1, ...
        'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:SquareSharp', ...
        'The square [-1,1]^2 must retain nonnegative enclosure.');
    A.cases=A.cases+1;

    % Mathematical Taylor with certified remainder applies to |x|<=4,
    % so a singleton pitch of 1.2 no longer collapses cos to [-1,1].
    for k=0:192
        t=(k-96)/32;  % exactly represented dyadic, [-3,3]
        X=TRIGGER_OUTWARD_INTERVAL(t,t);
        S=sin(X);C=cos(X);
        realS=sin(t);realC=cos(t);
        % libm sin/cos are diagnostic witnesses with generous tolerance;
        % the analytical remainder provides the mathematical enclosure.
        tol=256*eps(1);
        assert(S.lo<=realS+tol && S.hi>=realS-tol && ...
               C.lo<=realC+tol && C.hi>=realC-tol, ...
            'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:TaylorWitness', ...
            'Trig witness outside analytic interval at t=%.17g.',t);
        A.trigonometric_cases=A.trigonometric_cases+2;
        A.cases=A.cases+2;
    end
    pitch=TRIGGER_OUTWARD_INTERVAL(1.2,1.2);
    pc=cos(pitch);
    assert(pc.lo>0.3 && pc.hi<0.5, ...
        'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:PitchChart', ...
        'RHS cos(pitch) was lost even though the point pitch is nonsingular.');
    A.chart_restored=true;A.cases=A.cases+1;

    % The original 24-state interval RHS, at a nonsingular pitch >1,
    % must now return a finite enclosure. Previously universal [-1,1]
    % for cos(pitch) made 1/cos(pitch) fail unconditionally.
    xx=zeros(24,1);xx(5)=1.2;xx(19:21)=[0;0;-1];xx(15)=-2;
    uu=[10;0;0;0];pp=zeros(21,1);
    pp(1)=1;pp(6:8)=[0.05;0.05;0.08];pp(9)=9.80665;
    pp(20)=0.14;pp(21)=2;
    R=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS( ...
        struct('lower',xx,'upper',xx), ...
        struct('lower',uu,'upper',uu), ...
        struct('lower',pp,'upper',pp));
    assert(R.outward_elementary_operations && ...
        all(isfinite([R.rhs_lower;R.rhs_upper])) && ...
        all(R.rhs_lower<=R.rhs_upper), ...
        'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:PlantRHS', ...
        'Original 24-state RHS still loses the valid pitch chart: %s',R.reason);
    A.original_rhs_finite=true;A.cases=A.cases+1;

    % A truly unknown/singular denominator must still refuse division.
    failed=false;
    try
        z=TRIGGER_OUTWARD_INTERVAL(1,1) ./ ...
            TRIGGER_OUTWARD_INTERVAL(-1,1); %#ok<NASGU>
    catch
        failed=true;
    end
    assert(failed, ...
        'CHECK_TRIGGER_NONLINEAR_INTERVAL_SHARPENING:SingularAccepted');
    A.cases=A.cases+1;
    A.status='PASS_ANALYTIC_POWER_AND_TAYLOR_RHS_SHARPENING__NO_FLIGHT_CERT';
    A.reason=['Exact nonnegative even-power range and analytic Taylor ' ...
        'remainder remove unnecessary interval blowup. The actual 5s ' ...
        'Flight recovery and physical solver/mode containment are NOT proven.'];
catch ME
    A.failures=1;
    A.reason=[ME.identifier ' | ' ME.message];
end
try
    save('TRIGGER_NONLINEAR_INTERVAL_SHARPENING_RESULT.mat','A');
    fid=fopen('TRIGGER_NONLINEAR_INTERVAL_SHARPENING_RESULT.txt','w');
    if fid>0
        fprintf(fid,'%s\ncases=%d failures=%d even=%d odd=%d trig=%d restored=%d rhs=%d\n%s\n', ...
            A.status,A.cases,A.failures,A.even_power_cases, ...
            A.odd_power_cases,A.trigonometric_cases, ...
            A.chart_restored,A.original_rhs_finite,A.reason);
        fclose(fid);
    end
catch
end
end
