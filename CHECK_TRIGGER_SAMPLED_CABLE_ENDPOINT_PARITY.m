function R=CHECK_TRIGGER_SAMPLED_CABLE_ENDPOINT_PARITY()
% Ensure sampled geometry contact/unknown results are unchanged by adding
% the purely sufficient two-endpoint 1-Lipschitz cable clearance witness.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_SAMPLED_CABLE_ENDPOINT_PARITY', ...
    'cases',0,'failures',0,'reason','', ...
    'baseline_median_ms',NaN,'optimized_median_ms',NaN, ...
    'actual_flight_safety_proved',false);
old_rng=rng;restore_rng=onCleanup(@()rng(old_rng)); %#ok<NASGU>
try
    rng(421104,'twister');
    for k=1:200
        switch mod(k,5)
            case 0,typ='sphere';
            case 1,typ='ellipsoid';
            case 2,typ='box';
            case 3,typ='cylinder';
            otherwise,typ='torus';
        end
        sh=local_shape(typ,randn(3,1));
        n=1+mod(k,13);
        q=sh.center0+2*randn(3,n);
        l=q+0.5*randn(3,n);
        f=mod(k,2)==0;
        [a,b,c,d]=CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
            q,l,sh,0.15,0.15,0.02,0,f);
        [e,g,h,j]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            q,l,sh,0.15,0.15,0.02,0,f);
        assert(isequal(a,e)&&isequal(b,g)&&isequal(c,h)&&isequal(d,j), ...
            'CHECK_TRIGGER_CABLE:Parity', ...
            'Sampled geometric contact/unknown mismatch in trial %d.',k);
        R.cases=R.cases+1;
    end
    % The torus hole requires spatial cable resolution but can be shown
    % separate using the shape-neutral endpoint theorem.
    sh=local_shape('torus',zeros(3,1));
    q=[0;0;1];l=[0;0;-1];
    [a,b,c,d]=CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
        q,l,sh,0.15,0.15,0.02,0,true);
    [e,g,h,j]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        q,l,sh,0.15,0.15,0.02,0,true);
    assert(isequal(a,e)&&isequal(b,g)&&isequal(c,h)&&isequal(d,j), ...
        'CHECK_TRIGGER_CABLE:TorusHole','Torus hole changed class.');
    R.cases=R.cases+1;
    % Paired alternating microbenchmark; results are NOT WCET or proof of
    % whole-flight improvement. Deliberately no fixed performance PASS bar.
    x=zeros(1,40);y=zeros(1,40);
    for k=1:44
        if mod(k,2)
            tt=tic;CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
                q,l,sh,0.15,0.15,0.02,0,true);u=1000*toc(tt);
            tt=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,sh,0.15,0.15,0.02,0,true);v=1000*toc(tt);
        else
            tt=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,sh,0.15,0.15,0.02,0,true);v=1000*toc(tt);
            tt=tic;CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
                q,l,sh,0.15,0.15,0.02,0,true);u=1000*toc(tt);
        end
        if k>4,x(k-4)=u;y(k-4)=v;end
    end
    R.baseline_median_ms=median(x);
    R.optimized_median_ms=median(y);
    R.status='PASS_SAMPLED_CABLE_PARITY__NOT_CONTINUOUS_SAFETY';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_SAMPLED_CABLE_ENDPOINT_PARITY_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_SAMPLED_CABLE_ENDPOINT_PARITY_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\n', ...
        R.status,R.cases,R.failures);
    fprintf(f,'baseline_median_ms=%.7f\noptimized_median_ms=%.7f\n', ...
        R.baseline_median_ms,R.optimized_median_ms);
    fprintf(f,'reason=%s\n',R.reason);fclose(f);
end
end
function sh=local_shape(typ,c)
sh=struct('id',1,'type',typ,'dimensions',[], ...
    'center0',double(c(:)),'rotation',eye(3), ...
    'velocity',zeros(3,1),'reference_time_s',0, ...
    'bounding_radius_m',NaN,'is_stationary',true, ...
    'center_norm_m',norm(c),'outline',zeros(0,2), ...
    'vertices',zeros(0,3),'faces',zeros(0,3));
switch typ
    case 'sphere',sh.dimensions=1;sh.bounding_radius_m=1;
    case 'ellipsoid',sh.dimensions=[0.5;1.2;0.8];sh.bounding_radius_m=1.2;
    case 'box',sh.dimensions=[1;2;0.8];sh.bounding_radius_m=norm(sh.dimensions)/2;
    case 'cylinder',sh.dimensions=[0.8;2];sh.bounding_radius_m=hypot(0.8,1);
    case 'torus',sh.dimensions=[4;0.7];sh.bounding_radius_m=4.7;
end
end
