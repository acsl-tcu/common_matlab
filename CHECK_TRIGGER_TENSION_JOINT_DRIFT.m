function R=CHECK_TRIGGER_TENSION_JOINT_DRIFT()
% Exact constant-angular-rate SO(3) witnesses and fail-closed input checks.
% Existing direct-tension 341-case audit must ALSO pass before Flight.
R=struct('status','FAIL_TENSION_JOINT_DRIFT','cases',0,'failures',0, ...
    'dot_witnesses',0,'tension_tighten_cases',0,'reason','');
try
    rng(137,'twister');
    for k=1:400
        ang=(rand(3,1)-0.5)*0.7;
        d=randn(3,1);d=d/norm(d);
        wQ=(rand(3,1)-0.5)*4;
        wL=(rand(3,1)-0.5)*4;
        h=0.0001+0.02*rand;
        x=zeros(24,1);x(4:6)=ang;x(19:21)=d;
        x(10:12)=wQ;x(22:24)=wL;
        Z=struct('lower',x,'upper',x);
        S=struct('x0_lower',x,'x0_upper',x,'h_upper_s',h);
        W=TRIGGER_TENSION_JOINT_ROTATION_BOUND(S,Z);
        assert(W.valid,W.reason);
        % Independently evolve both UNIT directions with Rodrigues' SO(3)
        % formula: these are exact source kinematics for fixed body rates.
        R0=rot_euler(ang);
        for t=[0,0.25,0.5,0.75,1]*h
            % R(t)=R0*exp([omega_body]*t) is the exact original
            % Euler-chart orientation dynamics for constant body rates.
            bt=R0*(rodrigues(wQ,t)*[0;0;1]);
            dt=rodrigues(wL,t)*d;
            D=dot(bt,dt);
            assert(D>=W.dot_lower-5e-13 && ...
                D<=W.dot_upper+5e-13, ...
                'CHECK_TRIGGER_TENSION_JOINT_DRIFT:FalseExclusion', ...
                'Exact unit-axis witness excluded.');
            R.dot_witnesses=R.dot_witnesses+1;
        end
        R.cases=R.cases+1;
    end
    % A wide full-time Euler/direction box can be tightened by its true
    % starting box and an exact small angular-rate cap. The new bound
    % is an intersection and can NEVER enlarge the old dot interval.
    for k=1:24
        x=zeros(24,1);x(19:21)=[0;0;-1];
        lo=x;hi=x;zlo=x;zhi=x;
        zlo(4:6)=-0.6; zhi(4:6)=0.6;
        zlo(19:21)=-1;zhi(19:21)=1;
        zlo(10:12)=-0.05; zhi(10:12)=0.05;
        zlo(22:24)=-0.05; zhi(22:24)=0.05;
        h=0.0005+0.0001*k;
        W=TRIGGER_TENSION_JOINT_ROTATION_BOUND(struct( ...
            'x0_lower',lo,'x0_upper',hi,'h_upper_s',h), ...
            struct('lower',zlo,'upper',zhi));
        assert(W.valid,W.reason);
        if W.dot_lower>-0.5 || W.dot_upper<0.5
            R.tension_tighten_cases=R.tension_tighten_cases+1;
        end
        R.cases=R.cases+1;
    end
    % An unrelated starting state must never be accepted as a source
    % for a CURRENT all-time tube, because that would be a false proof.
    x=zeros(24,1);Z=struct('lower',x,'upper',x);
    S=struct('x0_lower',ones(24,1),'x0_upper',ones(24,1), ...
        'h_upper_s',0.01);
    W=TRIGGER_TENSION_JOINT_ROTATION_BOUND(S,Z);
    assert(~W.valid,'CHECK_TRIGGER_TENSION_JOINT_DRIFT:InvalidAccepted');
    R.cases=R.cases+1;
    % Zero duration is not a proof of a positive full-time slab.
    S=struct('x0_lower',x,'x0_upper',x,'h_upper_s',0);
    W=TRIGGER_TENSION_JOINT_ROTATION_BOUND(S,Z);
    assert(~W.valid,'CHECK_TRIGGER_TENSION_JOINT_DRIFT:ZeroAccepted');
    R.cases=R.cases+1;
    % Check the actual PRODUCTION tension function, not just the helper.
    % Initial pT=-e3 and body axis=+e3 with bounded rotation gives D<0.
    % The old unconstrained 24D interval permits fictitious pT=+e3,
    % but the EXACT starting direction and angular-speed drift exclude it.
    x=zeros(24,1); x(19:21)=[0;0;-1];
    zlo=x;zhi=x;
    zlo(4:6)=-0.6; zhi(4:6)=0.6;
    zlo(19:21)=-1; zhi(19:21)=1;
    zlo(10:12)=-0.05;zhi(10:12)=0.05;
    zlo(22:24)=-0.05;zhi(22:24)=0.05;
    Z=struct('lower',zlo,'upper',zhi);
    P=zeros(21,1);P(1)=0.5;P(20)=0.14;P(21)=2;
    PB=struct('lower',P,'upper',P);
    UB=struct('lower',[10;0;0;0],'upper',[10;0;0;0]);
    S=struct('x0_lower',x,'x0_upper',x,'h_upper_s',0.001);
    Told=TRIGGER_ORIGINAL_TENSION_INTERVAL(Z,UB,PB);
    Tnew=TRIGGER_ORIGINAL_TENSION_INTERVAL(Z,UB,PB,S);
    assert(~Told.conditional_tension_positive && ...
        Tnew.conditional_tension_positive && ...
        Tnew.joint_drift_tightening_used && ...
        Tnew.tension_lower_N>0, ...
        'CHECK_TRIGGER_TENSION_JOINT_DRIFT:ProductionTensionNotTightened');
    R.cases=R.cases+1;
    % The universal HLC [0,20] thrust range INCLUDES zero: it must NOT
    % be falsely claimed to imply N>0, even with correlated axes.
    UB.lower(1)=0;UB.upper(1)=20;
    Tzero=TRIGGER_ORIGINAL_TENSION_INTERVAL(Z,UB,PB,S);
    assert(~Tzero.conditional_tension_positive, ...
        'CHECK_TRIGGER_TENSION_JOINT_DRIFT:ZeroThrustUnsafeClearance');
    R.cases=R.cases+1;
    assert(R.dot_witnesses==2000&&R.tension_tighten_cases==24);
    R.status='PASS_TAUT_JOINT_ROTATION_CONE__CONDITIONAL_SOURCE';
    R.reason=['All-time dot drift follows exact unit SO(3) axis ' ...
        'derivatives; no physical tension or full 4s proof is implied.'];
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_TENSION_JOINT_DRIFT_RESULT.mat','R');
fid=fopen('TRIGGER_TENSION_JOINT_DRIFT_RESULT.txt','w');
if fid>=0
    fprintf(fid,'%s\ncases=%d failures=%d witnesses=%d tightened=%d\n%s\n', ...
        R.status,R.cases,R.failures,R.dot_witnesses,R.tension_tighten_cases,R.reason);
    fclose(fid);
end
end
function R=rot_euler(a)
r=a(1);p=a(2);y=a(3);
Rz=[cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1];
Ry=[cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)];
Rx=[1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
R=Rz*Ry*Rx;
end
function R=rodrigues(w,t)
q=norm(w);
if q<1e-12,R=eye(3);return;end
u=w/q;
K=[0,-u(3),u(2);u(3),0,-u(1);-u(2),u(1),0];
R=eye(3)+sin(q*t)*K+(1-cos(q*t))*(K*K);
end
