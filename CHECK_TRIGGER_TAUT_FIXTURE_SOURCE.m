function A=CHECK_TRIGGER_TAUT_FIXTURE_SOURCE()
% Disposable signed cable-geometry test. The ORIGINAL plant convention is
% d=pT from UAV TO payload; true taut manifold pL=pQ+L*d.
A=struct('status','NOT_RUN_TAUT_SOURCE_SIGN','reason','', ...
  'on_manifold',false,'old_fixture_detected_wrong',false, ...
  'source_rate_residual',NaN,'samples',0,'safe_to_commit',false);
try
    f=which('with_load_model_euler_for_HL');
    assert(~isempty(f),'CHECK_TRIGGER_TAUT_FIXTURE_SOURCE:Plant', ...
        'Original 24-state plant source is required.');
    L=2; p=[0;0;0]; d=[0;0;-1];
    pL=p+L*d; v=zeros(3,1);wL=zeros(3,1);
    A.on_manifold=norm(pL-p-L*d,inf)==0 && ...
        norm(v-v-L*cross(wL,d),inf)==0 && norm(d)==1;
    A.old_fixture_detected_wrong= ...
        norm(pL-p-L*[0;0;1],inf)>=2*L;
    x=zeros(24,1);
    x(13:15)=pL;x(19:21)=d;
    P=zeros(1,21);P(1)=1.4;P(6:8)=[0.05 0.06 0.08];
    P(9)=9.80665;P(20)=0.5;P(21)=L;
    U=[(P(1)+P(20))*P(9);0;0;0];
    % Regression over PHYSICALLY CONSISTENT random states, not arbitrary
    % off-manifold synthetic pT/pL pairs. The algebraic proof is separate.
    saved_rng=rng;restore_rng=onCleanup(@()rng(saved_rng)); %#ok<NASGU>
    rng(303,'twister');
    largest=0;
    for j=1:100
        x=zeros(24,1);
        x(1:3)=randn(3,1)*0.4;
        x(4:6)=randn(3,1)*0.2;
        x(7:9)=randn(3,1)*0.3;
        x(10:12)=randn(3,1)*0.15;
        d=randn(3,1);d=d/norm(d);
        omega=randn(3,1)*0.2;
        x(13:15)=x(1:3)+L*d;
        x(16:18)=x(7:9)+L*cross(omega,d);
        x(19:21)=d;x(22:24)=omega;
        dx=with_load_model_euler_for_HL(x,U,P);
        c0rate=dx(13:15)-dx(1:3)-L*dx(19:21);
        c1rate=dx(16:18)-dx(7:9)-L*( ...
            cross(dx(22:24),d)+cross(omega,dx(19:21)));
        snormrate=2*d.'*dx(19:21);
        largest=max(largest,max([norm(c0rate,inf), ...
            norm(c1rate,inf),abs(snormrate)]));
        A.samples=A.samples+1;
    end
    A.source_rate_residual=largest;
    assert(A.on_manifold && A.old_fixture_detected_wrong && ...
        A.source_rate_residual<1e-10, ...
        'CHECK_TRIGGER_TAUT_FIXTURE_SOURCE:Mismatch', ...
        'Original plant and fixture geometry are inconsistent.');
    A.status='PASS_ORIGINAL_TAUT_SIGN_AND_SOURCE_RATE__NOT_SAFETY';
catch ME
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_TAUT_FIXTURE_SOURCE_RESULT.mat','A');
f=fopen('TRIGGER_TAUT_FIXTURE_SOURCE_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\non_manifold=%d\nold_fixture_detected_wrong=%d\nsamples=%d\nsource_rate_residual=%.17g\nreason=%s\n', ...
        A.status,A.on_manifold,A.old_fixture_detected_wrong, ...
        A.samples,A.source_rate_residual,A.reason);
    fclose(f);
end
end
