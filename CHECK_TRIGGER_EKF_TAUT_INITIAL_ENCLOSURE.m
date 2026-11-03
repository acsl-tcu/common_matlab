function A=CHECK_TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE()
% Disposable common_matlab/ test. Synthetic fixtures only, never sensor
% truth or guessed EKF error certification. NOT flight safety.
A=struct('status','UNKNOWN_EKF_TAUT_FIXTURES','tested',0, ...
    'violations',0,'inconsistent_rejected',false, ...
    'missing_error_rejected',false,'reason','');
try
    rngstate=rng;cleanup=onCleanup(@()rng(rngstate)); %#ok<NASGU>
    rng(1327,'twister');
    p=[0.2;-0.4;1.7];v=[0.3;0.15;-0.1];
    w=[0.08;-0.04;0.02];d=[0;0;-1];
    L=2; % TEST FIXTURE ONLY, not Flight cable default
    e=struct('p',p,'q',[0;0;0],'v',v,'w',[0;0;0], ...
        'pL',p+L*d,'vL',v+L*cross(w,d),'pT',d,'wL',w);
    C=struct('ekf_now',e,'error_lower',-0.02*ones(24,1), ...
        'error_upper',0.02*ones(24,1), ...
        'cable_length_lower_m',L,'cable_length_upper_m',L);
    R=TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE(C);
    assert(R.conditional_enclosure && ~R.safe_to_commit, ...
        'CHECK_TRIGGER_EKF_TAUT_INITIAL:Pass','Expected conditional, not certified initial set.');
    for k=1:500
        pp=p+0.001*(2*rand(3,1)-1);
        vv=v+0.001*(2*rand(3,1)-1);
        ww=w+0.001*(2*rand(3,1)-1);
        dd=d+0.001*(2*rand(3,1)-1);dd=dd/norm(dd);
        q=0.001*(2*rand(3,1)-1);bodyw=0.001*(2*rand(3,1)-1);
        state=[pp;q;vv;bodyw;pp+L*dd;vv+L*cross(ww,dd);dd;ww];
        if any(state<R.initial_lower) || any(state>R.initial_upper)
            A.violations=A.violations+1;
        end
        A.tested=A.tested+1;
    end
    assert(A.violations==0, ...
        'CHECK_TRIGGER_EKF_TAUT_INITIAL:Enclosure', ...
        'Taut synthetic true state was excluded.');
    bad=C;bad.ekf_now.pL=e.pL+[100;0;0];
    Z=TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE(bad);
    A.inconsistent_rejected=~Z.conditional_enclosure && ...
        strcmp(Z.status,'UNKNOWN_EKF_TAUT_MANIFOLD_INCONSISTENT');
    noerr=rmfield(C,'error_lower');
    N=TRIGGER_EKF_TAUT_INITIAL_ENCLOSURE(noerr);
    A.missing_error_rejected=~N.conditional_enclosure;
    assert(A.inconsistent_rejected && A.missing_error_rejected,...
        'CHECK_TRIGGER_EKF_TAUT_INITIAL:Reject', ...
        'Missing or physically inconsistent witness was accepted.');
    A.status='PASS_CONDITIONAL_EKF_TAUT_INITIAL_FIXTURES__NOT_EKF_PROOF';
catch ME
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_EKF_TAUT_INITIAL_RESULT.mat','A');
f=fopen('TRIGGER_EKF_TAUT_INITIAL_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\ntested=%d\nviolations=%d\ninconsistent_rejected=%d\nmissing_error_rejected=%d\nreason=%s\n', ...
        A.status,A.tested,A.violations,A.inconsistent_rejected,...
        A.missing_error_rejected,A.reason);
    fclose(f);
end
end
