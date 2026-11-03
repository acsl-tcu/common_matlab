function A=CHECK_TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS()
% Disposable diagnostic in common_matlab/ root, never a Flight safety pass.
% Compare the original 24-state source with an outward-interval translation
% on point and small-box synthetic physical states. This is a numerical
% source parity test, not a rigorous proof of the complete closed loop.
A=struct('status','NOT_RUN_ORIGINAL_PLANT_INTERVAL_RHS', ...
    'original_source_found',false,'tested_boxes',0,'tested_samples',0, ...
    'enclosure_violations',0,'singular_chart_rejected',false, ...
    'safe_to_commit',false,'reason','');
try
    original=which('with_load_model_euler_for_HL');
    assert(~isempty(original), 'CHECK_TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:OriginalMissing', ...
        'The actual original plant RHS must be on MATLAB path.');
    A.original_source_found=true;
    original_f=@with_load_model_euler_for_HL;
    x=zeros(24,1);
    x(13:15)=[0;0;-2];
    x(19:21)=[0;0;1];
    p=zeros(21,1);
    p(1)=1.4;p(6:8)=[0.05;0.06;0.08];
    p(9)=9.80665;p(20)=0.5;p(21)=2;
    u=[(p(1)+p(20))*p(9);0;0;0];
    previous_rng=rng; rng_cleanup=onCleanup(@()rng(previous_rng)); %#ok<NASGU>
    rng(101,'twister'); % fixture repeatability only; no sensor truths
    for trial=1:20
        xp=x;
        xp(4:6)=0.03*randn(3,1);
        xp(7:12)=0.03*randn(6,1);
        xp(16:18)=0.03*randn(3,1);
        xp(19:21)=[0;0;1]+0.003*randn(3,1);
        xp(22:24)=0.03*randn(3,1);
        u2=u+[0.1*randn;0.01*randn(3,1)];
        radius=1e-5;
        X=struct('lower',xp-radius,'upper',xp+radius);
        U=struct('lower',u2-radius,'upper',u2+radius);
        P=struct('lower',p,'upper',p);
        % Explicit interval of unknown payload mass variation; not a
        % claimed physical estimator bound.
        P.lower(20)=p(20)-radius;
        P.upper(20)=p(20)+radius;
        R=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS(X,U,P);
        assert(R.outward_elementary_operations && ~R.safe_to_commit, ...
            'CHECK_TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:IntervalUnknown', ...
            'Interval RHS did not produce finite bounds.');
        A.tested_boxes=A.tested_boxes+1;
        for sample=1:12
            xs=X.lower+rand(24,1).*(X.upper-X.lower);
            us=U.lower+rand(4,1).*(U.upper-U.lower);
            ps=P.lower+rand(21,1).*(P.upper-P.lower);
            f=original_f(xs,us,ps.');
            f=double(f(:));
            assert(isequal(size(f),[24 1])&&all(isfinite(f)), ...
                'CHECK_TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:OriginalValue', ...
                'Original numerical RHS produced invalid data.');
            if any(f<R.rhs_lower)||any(f>R.rhs_upper)
                A.enclosure_violations=A.enclosure_violations+1;
            end
            A.tested_samples=A.tested_samples+1;
        end
    end
    % Pitch chart singularity: the interval covers pitch=pi/2.
    X.lower(5)=pi/2-0.1; X.upper(5)=pi/2+0.1;
    S=TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS(X,U,P);
    A.singular_chart_rejected=~S.outward_elementary_operations;
    assert(A.enclosure_violations==0 && A.singular_chart_rejected,...
        'CHECK_TRIGGER_ORIGINAL_PLANT_INTERVAL_RHS:Mismatch',...
        'An original RHS sample escaped or a chart singularity was not rejected.');
    A.status='PASS_INTERVAL_RHS_SAMPLED_ENCLOSURES__NO_CLOSED_LOOP_PROOF';
catch ME
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_ORIGINAL_PLANT_INTERVAL_RESULT.mat','A');
fid=fopen('TRIGGER_ORIGINAL_PLANT_INTERVAL_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\noriginal_found=%d\nboxes=%d\nsamples=%d\nviolations=%d\nsingular_rejected=%d\nreason=%s\n',...
        A.status,A.original_source_found,A.tested_boxes,A.tested_samples,...
        A.enclosure_violations,A.singular_chart_rejected,A.reason);
    fclose(fid);
end
end
