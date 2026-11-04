function R=CHECK_TRIGGER_BLOCK_AND_NONCONVEX()
% COMMON_ROOT disposable audit. All reports written to common_matlab root.
% No Flight is run and no true closed-loop safety is claimed.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_BLOCK_NONCONVEX_AUDIT','cases',0, ...
    'failures',0,'block_cases',0,'block_exclusions',0, ...
    'distance_cases',0,'median_baseline_ms',NaN, ...
    'median_new_ms',NaN,'reason','', ...
    'real_flight_safety_proved',false);
old_rng=rng;restore_rng=onCleanup(@()rng(old_rng)); %#ok<NASGU>
% The block gate is NOT used in production by default after the measured
% baseline regression. This historical stress test deliberately enables
% it, then restores the caller's environment on all exit paths.
old_block_flag=getenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT');
restore_block_flag=onCleanup(@()setenv( ...
    'TRIGGER_SAMPLED_BLOCK_EXPERIMENT',old_block_flag)); %#ok<NASGU>
setenv('TRIGGER_SAMPLED_BLOCK_EXPERIMENT','1');
try
    rng(70604,'twister');
    types={'sphere','ellipsoid','box','cylinder','capsule', ...
        'cone','frustum','torus','regular_prism','polygon_prism'};
    for k=1:100
        type=types{1+mod(k-1,numel(types))};
        s=local_shape(type,local_R(),[0.4;0.3;0.2]*randn,k);
        n=41;
        switch mod(k,4)
            case 0
                q=s.center0+[5;0;0]+2*randn(3,n);
            case 1
                q=s.center0+[1;0;0]+1.4*randn(3,n);
            case 2
                q=s.center0+0.1*randn(3,n);
            otherwise
                q=s.center0+[0;0;2]+0.6*randn(3,n);
        end
        l=q+[0;0;-1.8];
        first=mod(k,2)==0;
        [a,b,c,d]=CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
            q,l,s,0.14,0.14,0.02,0,first);
        [aa,bb,cc,dd,info]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            q,l,s,0.14,0.14,0.02,0,first);
        assert(isequal(a,aa)&&isequal(b,bb)&& ...
               isequal(c,cc)&&isequal(d,dd), ...
            'CHECK_TRIGGER_BLOCK:Parity', ...
            'Original all-shape geometry differs in trial %d (%s).',k,type);
        R.cases=R.cases+1;
        if info.sampled_block_support_probes>0
            R.block_cases=R.block_cases+1;
        end
        R.block_exclusions=R.block_exclusions+info.sampled_block_support_excluded;
    end
    % Curved, arbitrarily oriented, thin obstacle; global coordinate AABB
    % overlaps its sphere. Several LOCAL path blocks are separated.
    aa=pi/4;
    Rxy=[cos(aa) -sin(aa) 0;sin(aa) cos(aa) 0;0 0 1];
    s=local_shape('ellipsoid',Rxy,zeros(3,1),101);
    yy=linspace(-6,6,161);
    q=s.rotation*[4+0.05*sin(yy);yy;0.1*cos(yy)];
    l=q+s.rotation*[zeros(1,161);zeros(1,161);-1.5*ones(1,161)];
    [a,b,c,d]=CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
        q,l,s,0.2,0.2,0.02,0,true);
    [aa,bb,cc,dd,f]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        q,l,s,0.2,0.2,0.02,0,true);
    assert(isequal(a,aa)&&isequal(b,bb)&&isequal(c,cc)&&isequal(d,dd), ...
        'CHECK_TRIGGER_BLOCK:CurvedParity','Curved-path contact result changed.');
    assert(f.sampled_block_support_excluded>0, ...
        'CHECK_TRIGGER_BLOCK:NoLocalExclusion', ...
        'Curved, rotated, elongated benchmark did not exercise block gate.');
    R.cases=R.cases+1;
    R.block_exclusions=R.block_exclusions+f.sampled_block_support_excluded;
    % Paired alternating timing is a MICROBENCHMARK ONLY.
    x=zeros(1,40);y=zeros(1,40);
    for z=1:44
        if mod(z,2)
            tt=tic;CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);a_ms=1000*toc(tt);
            tt=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);b_ms=1000*toc(tt);
        else
            tt=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);b_ms=1000*toc(tt);
            tt=tic;CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
                q,l,s,0.2,0.2,0.02,0,true);a_ms=1000*toc(tt);
        end
        if z>4,x(z-4)=a_ms;y(z-4)=b_ms;end
    end
    R.median_baseline_ms=median(x);R.median_new_ms=median(y);
    % Nonconvex torus HOLE. Its convex hull includes the safe cable.
    % The exact ORIGINAL shape distance and 1-Lipschitz subdivision must
    % be able to establish a conditional WHOLE-INTERVAL certificate.
    for k=1:24
        s=local_shape('torus',local_R(),randn(3,1),200+k);
        q=s.center0+s.rotation*[0;0;1];
        l=s.center0+s.rotation*[0;0;-1];
        D=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
            q,l,s,0.15,0.15,0.02,0.05,0.05,0);
        assert(D.conditionally_separated && D.safe_to_commit==false, ...
            'CHECK_TRIGGER_BLOCK:TorusHole', ...
            'Torus hole was lost by its convex-hull approximation.');
        % Rotation-preserving exact tube throughout [0,1].
        U=[q q];L=[l l];
        G=TRIGGER_CONTINUOUS_INTERVAL_GATE.evaluate( ...
            U,L,[0 1],s,0.15,0.15,0.02,0.05,0.05);
        assert(G.conditionally_clear && ...
            ~G.continuous_closed_loop_safe_certified, ...
            'CHECK_TRIGGER_BLOCK:TorusTime', ...
            'Nonconvex whole-time fallback failed.');
        R.distance_cases=R.distance_cases+2;R.cases=R.cases+2;
        % Same obstacle, cable through physical torus material: UNKNOWN.
        near=s.center0+s.rotation*[4;0;1];
        far=s.center0+s.rotation*[4;0;-1];
        E=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
            near,far,s,0.15,0.15,0.02,0.05,0.05,0);
        assert(~E.conditionally_separated, ...
            'CHECK_TRIGGER_BLOCK:FalseClear','Torus ring falsely cleared.');
        R.distance_cases=R.distance_cases+1;R.cases=R.cases+1;
    end
    % Unknown shape and invalid distance never grant clearance.
    s=local_shape('sphere',eye(3),zeros(3,1),600);
    q=[0;0;0];l=[0;0;-1];
    D=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE(q,l,s,0.2,0.2,0,0,0,0);
    assert(~D.conditionally_separated, 'CHECK_TRIGGER_BLOCK:Intersection');
    R.cases=R.cases+1;R.distance_cases=R.distance_cases+1;
    s.type='future_unknown_distance_shape';
    D=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE([5;0;0],[5;0;-1], ...
        s,0.2,0.2,0,0,0,0);
    assert(~D.conditionally_separated, 'CHECK_TRIGGER_BLOCK:UnknownShape');
    R.cases=R.cases+1;R.distance_cases=R.distance_cases+1;
    R.status='PASS_SHAPE_INDEPENDENT_SAMPLED_BLOCK_AND_NONCONVEX_TUBE__CONDITIONAL';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_BLOCK_AND_NONCONVEX_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_BLOCK_AND_NONCONVEX_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\nblock_cases=%d\n', ...
        R.status,R.cases,R.failures,R.block_cases);
    fprintf(f,'block_exclusions=%d\ndistance_cases=%d\n', ...
        R.block_exclusions,R.distance_cases);
    fprintf(f,'median_baseline_ms=%.7f\nmedian_new_ms=%.7f\n', ...
        R.median_baseline_ms,R.median_new_ms);
    fprintf(f,'reason=%s\n',R.reason);fclose(f);
end
end
function R=local_R()
a=(rand*2-1)*pi;b=(rand*2-1)*pi;c=(rand*2-1)*pi;
Rx=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rz=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
R=Rz*Ry*Rx;
end
function s=local_shape(type,R,c,id)
s=struct('id',double(id),'type',type,'dimensions',[], ...
    'center0',double(c(:)),'rotation',double(R), ...
    'velocity',zeros(3,1),'reference_time_s',0, ...
    'bounding_radius_m',NaN,'is_stationary',true, ...
    'center_norm_m',norm(c),'outline',zeros(0,2), ...
    'vertices',zeros(0,3),'faces',zeros(0,3));
switch type
    case 'sphere'
        s.dimensions=1;s.bounding_radius_m=1;
    case 'ellipsoid'
        s.dimensions=[0.5;8;0.6];s.bounding_radius_m=8;
    case 'box'
        s.dimensions=[1;7;2];s.bounding_radius_m=norm(s.dimensions)/2;
    case 'cylinder'
        s.dimensions=[1;6];s.bounding_radius_m=hypot(1,3);
    case 'capsule'
        s.dimensions=[0.8;5];s.bounding_radius_m=0.8+2.5;
    case 'cone'
        s.dimensions=[1.2;6];s.bounding_radius_m=hypot(1.2,3);
    case 'frustum'
        s.dimensions=[1.2;0.3;6];s.bounding_radius_m=hypot(1.2,3);
    case 'torus'
        s.dimensions=[4;0.7];s.bounding_radius_m=4.7;
    case 'regular_prism'
        s.dimensions=[6;1.2;6];n=6;th=(0:n-1)'*2*pi/n;
        s.outline=1.2*[cos(th),sin(th)];s.bounding_radius_m=hypot(1.2,3);
    case 'polygon_prism'
        s.outline=[-1 -1;2 -1;2 0;0 0;0 1;-1 1];
        s.dimensions=struct('outline',s.outline,'height',4);
        s.bounding_radius_m=hypot(max(sqrt(sum(s.outline.^2,2))),2);
end
end
