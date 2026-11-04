function R=CHECK_TRIGGER_ENDPOINT_LIPSCHITZ()
% COMMON_ROOT disposable mathematical and geometry oracle regression.
% No claims about EKF, HLC, interval tubes or floating-point proof.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_ENDPOINT_LIPSCHITZ_AUDIT', ...
    'cases',0,'failures',0,'endpoint_witnesses',0, ...
    'sampled_oracle_checks',0,'distance_query_savings',0, ...
    'reason','','physical_safety_proved',false);
old_rng=rng;clean_rng=onCleanup(@()rng(old_rng)); %#ok<NASGU>
try
    rng(91021,'twister');
    types={'sphere','ellipsoid','box','cylinder','torus'};
    for k=1:500
        typ=types{mod(k-1,numel(types))+1};
        ax=2*pi*rand; ay=pi*rand; az=pi*rand;
        Qx=[1 0 0;0 cos(ax) -sin(ax);0 sin(ax) cos(ax)];
        Qy=[cos(ay) 0 sin(ay);0 1 0;-sin(ay) 0 cos(ay)];
        Qz=[cos(az) -sin(az) 0;sin(az) cos(az) 0;0 0 1];
        rot=Qz*Qy*Qx;
        sh=local_shape(typ,rot,randn(3,1));
        q=sh.center0+rot*(3.0*randn(3,1));
        l=q+randn(3,1);
        rr=0.02+0.15*rand;
        eu=0.01*rand;el=0.01*rand;
        X=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
             q,l,sh,rr,rr,rr,eu,el,0);
        assert(~X.safe_to_commit && X.distance_queries<=48, ...
            'CHECK_TRIGGER_ENDPOINT:Contract','Invalid diagnostic contract.');
        R.cases=R.cases+1;
        if X.endpoint_lipschitz_used
            R.endpoint_witnesses=R.endpoint_witnesses+1;
            assert(X.conditionally_separated && X.distance_queries==2, ...
                'CHECK_TRIGGER_ENDPOINT:FastWitness', ...
                'Two-endpoint witness incorrectly returned UNKNOWN.');
            for j=0:120
                pt=q+(j/120)*(l-q);
                d=OBSTACLE_GEOMETRY_QUERY.distance(pt,sh,0,sh.center0);
                assert(isfinite(d) && d>rr+max(eu,el), ...
                    'CHECK_TRIGGER_ENDPOINT:FalseClear', ...
                    'A true-shape spatial cable sample violated clearance.');
                R.sampled_oracle_checks=R.sampled_oracle_checks+1;
            end
            R.distance_query_savings=R.distance_query_savings+1;
        end
    end
    % A curved torus hole (nonconvex). Here support convex-hull exclusion
    % fails in general, but endpoint Lipschitz often gives clearance.
    sh=local_shape('torus',eye(3),zeros(3,1));
    D=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
        [0;0;1],[0;0;-1],sh,0.15,0.15,0.02,0.05,0.05,0);
    assert(D.conditionally_separated && D.endpoint_lipschitz_used && ...
        D.distance_queries==2 && ~D.safe_to_commit, ...
        'CHECK_TRIGGER_ENDPOINT:Torus','Safe spatial torus hole rejected.');
    R.cases=R.cases+1;R.endpoint_witnesses=R.endpoint_witnesses+1;
    % Intersection must remain unresolved. No amount of endpoint distance
    % can by itself infer a collision; it can only certify separation.
    C=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
        [4;0;1],[4;0;-1],sh,0.15,0.15,0.02,0.05,0.05,0);
    assert(~C.conditionally_separated && ~C.safe_to_commit, ...
        'CHECK_TRIGGER_ENDPOINT:FalseRingClear','Near torus material cleared.');
    R.cases=R.cases+1;
    sh.type='future_unknown_shape';
    E=TRIGGER_TAUT_DISTANCE_INTERVAL_GATE( ...
        [5;0;1],[5;0;-1],sh,0.15,0.15,0.02,0.05,0.05,0);
    assert(~E.conditionally_separated && ~E.safe_to_commit, ...
        'CHECK_TRIGGER_ENDPOINT:UnknownShape','Unsupported geometry cleared.');
    R.cases=R.cases+1;
    R.status='PASS_ENDPOINT_LIPSCHITZ_SHAPE_NEUTRAL__CONDITIONAL_ONLY';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_ENDPOINT_LIPSCHITZ_RESULT.mat'),'R');
f=fopen(fullfile(root,'TRIGGER_ENDPOINT_LIPSCHITZ_RESULT.txt'),'w');
if f>=0
    fprintf(f,'status=%s\ncases=%d\nfailures=%d\n', ...
        R.status,R.cases,R.failures);
    fprintf(f,['endpoint_witnesses=%d\nsampled_oracle_checks=%d\n' ...
        'distance_query_savings=%d\nreason=%s\n'], ...
        R.endpoint_witnesses,R.sampled_oracle_checks, ...
        R.distance_query_savings,R.reason);fclose(f);
end
end
function sh=local_shape(typ,rotation,c)
sh=struct('id',1,'type',typ,'dimensions',[], ...
    'center0',double(c(:)),'rotation',double(rotation), ...
    'velocity',zeros(3,1),'reference_time_s',0, ...
    'bounding_radius_m',NaN,'is_stationary',true, ...
    'center_norm_m',norm(c),'outline',zeros(0,2), ...
    'vertices',zeros(0,3),'faces',zeros(0,3));
switch typ
    case 'sphere'
        sh.dimensions=0.8;sh.bounding_radius_m=0.8;
    case 'ellipsoid'
        sh.dimensions=[0.5;1.2;0.8];sh.bounding_radius_m=1.2;
    case 'box'
        sh.dimensions=[1;2;0.8];sh.bounding_radius_m=norm(sh.dimensions)/2;
    case 'cylinder'
        sh.dimensions=[0.8;2];sh.bounding_radius_m=hypot(0.8,1);
    case 'torus'
        sh.dimensions=[4;0.7];sh.bounding_radius_m=4.7;
    otherwise
        error('CHECK_TRIGGER_ENDPOINT:Shape','Unsupported audit fixture.');
end
end
