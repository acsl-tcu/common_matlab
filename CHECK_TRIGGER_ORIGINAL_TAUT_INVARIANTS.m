function R=CHECK_TRIGGER_ORIGINAL_TAUT_INVARIANTS()
% Disposable source-parity audit: original 24-state plant derivatives.
% The algebraic theorem is conditional on the source equations, fixed L,
% finite smooth motion and an initially consistent taut state. Numeric
% sampling here is only a regression check; NOT a 4-s Flight certificate.
R=struct('status','NOT_RUN_TAUT_INVARIANTS','sample_count',0, ...
    'max_position_derivative_residual',NaN, ...
    'max_velocity_derivative_residual',NaN, ...
    'max_unit_derivative_residual',NaN, ...
    'original_source_found',false, ...
    'safe_to_commit',false,'reason','');
try
    assert(~isempty(which('with_load_model_euler_for_HL')), ...
        'CHECK_TRIGGER_ORIGINAL_TAUT_INVARIANTS:Missing', ...
        'The actual original 24-state model is not on MATLAB path.');
    R.original_source_found=true;
    orig=rng;cleanup=onCleanup(@()rng(orig));
    rng(927,'twister');
    n=200; maxi=zeros(1,3);
    for k=1:n
        L=2; m=1.4; ml=0.15+0.7*rand();
        x=zeros(24,1);x(1:3)=randn(3,1);
        x(4:6)=0.3*randn(3,1);
        x(7:9)=randn(3,1);x(10:12)=randn(3,1);
        w=randn(3,1);pt=randn(3,1);pt=pt/norm(pt);
        x(19:21)=pt;x(22:24)=w;
        x(13:15)=x(1:3)+L*pt;
        x(16:18)=x(7:9)+L*cross(w,pt);
        p=zeros(21,1);p(1)=m;p(6:8)=[.05;.06;.08];
        p(9)=9.80665;p(20)=ml;p(21)=L;
        u=[20*rand();2*rand(3,1)-1];
        f=with_load_model_euler_for_HL(x,u,p.');f=double(f(:));
        assert(numel(f)==24 && all(isfinite(f)), ...
            'CHECK_TRIGGER_ORIGINAL_TAUT_INVARIANTS:RHS', ...
            'Original RHS must return 24 finite derivatives.');
        r0=f(13:15)-f(1:3)-L*f(19:21);
        r1=f(16:18)-f(7:9)-L*(cross(f(22:24),pt)+cross(w,f(19:21)));
        r2=2*pt.'*f(19:21);
        maxi=max(maxi,[max(abs(r0)),max(abs(r1)),abs(r2)]);
        tol=1e-10*max(1,norm(f,inf));
        assert(all(abs(r0)<tol)&&all(abs(r1)<tol)&&abs(r2)<tol, ...
            'CHECK_TRIGGER_ORIGINAL_TAUT_INVARIANTS:Mismatch', ...
            'Original 24-state derivative contradicts taut-manifold identity.');
        R.sample_count=k;
    end
    R.max_position_derivative_residual=maxi(1);
    R.max_velocity_derivative_residual=maxi(2);
    R.max_unit_derivative_residual=maxi(3);
    R.status='PASS_SAMPLED_ORIGINAL_TAUT_DERIVATIVES__NOT_FLIGHT_SAFETY';
catch ME
    R.status='FAIL_OR_UNKNOWN_TAUT_IDENTITY';
    R.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_TAUT_DERIVATIVE_RESULT.mat','R');
fid=fopen('TRIGGER_TAUT_DERIVATIVE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nsamples=%d\nmax_c0dot=%g\nmax_c1dot=%g\nmax_unit_dot=%g\nreason=%s\n', ...
        R.status,R.sample_count,R.max_position_derivative_residual, ...
        R.max_velocity_derivative_residual,R.max_unit_derivative_residual,R.reason);
    fclose(fid);
end
end
