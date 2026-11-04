function R=CHECK_TRIGGER_SAMPLED_AXIS_PLANES()
% Disposable COMMON_ROOT auditor. Compare unchanged legacy geometry oracle
% against the new nominal sample-only axis support exclusion.
% No changes to Flight source/trajectory, no world knowledge, no SAFE claim.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_SAMPLED_AXIS_PLANES', ...
    'cases',0,'failures',0,'axis_exclusion_cases',0, ...
    'paired_median_baseline_ms',NaN,'paired_median_new_ms',NaN, ...
    'paired_max_baseline_ms',NaN,'paired_max_new_ms',NaN, ...
    'reason','','continuous_time_safety_certified',false);
old=rng;
restore_rng=onCleanup(@()rng(old)); %#ok<NASGU>
try
    rng(36107,'twister');
    flags={'0','1'}; %#ok<NASGU>
    for k=1:160
        typ=mod(k,4);
        ang=2*pi*rand;
        rot=[cos(ang) -sin(ang) 0;sin(ang) cos(ang) 0;0 0 1];
        switch typ
            case 0, type='ellipsoid';d=[0.4+rand;1+10*rand;0.3+rand];b=max(d);
            case 1, type='box';d=[1+2*rand;1+9*rand;1+rand];b=norm(d)/2;
            case 2, type='cylinder';d=[0.4+rand;2+15*rand];b=hypot(d(1),d(2)/2);
            otherwise, type='sphere';d=0.5+rand;b=d;
        end
        s=make_shape(type,d,b,rot,randn(3,1),k);
        q=s.center0+4*randn(3,13);
        l=q+[0;0;-2];
        [A1,A2,A3,A4]=CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
            q,l,s,0.2,0.18,0.02,0,true);
        [B1,B2,B3,B4,info]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            q,l,s,0.2,0.18,0.02,0,true);
        R.cases=R.cases+1;
        if ~(isequal(A1,B1)&&isequal(A2,B2)&& ...
                isequal(A3,B3)&&isequal(A4,B4))
            error('CHECK_TRIGGER_SAMPLED_AXIS_PLANES:Parity', ...
                'Original oracle differs from new axis filter for case %d.',k);
        end
        if info.sampled_axis_plane_excluded
            R.axis_exclusion_cases=R.axis_exclusion_cases+1;
        end
    end
    % Exact tangent must not be excluded. Equality stays in narrow phase.
    e=make_shape('ellipsoid',[0.75;12;0.5],12,eye(3),zeros(3,1),1);
    [yes,~]=TRIGGER_SAMPLED_AXIS_SUPPORT_GATE( ...
        e,[0.75;0;0],[0.75;0;0],0,0,0);
    assert(~yes,'CHECK_TRIGGER_SAMPLED_AXIS_PLANES:Tangent', ...
        'Tangent was incorrectly cleared by a strict plane.');
    R.cases=R.cases+1;
    % The original outer sphere overlaps the entire sampled path; the
    % true ellipsoid can nevertheless be globally plane-separated.
    N=161;u=[2+0.05*sin(1:N);2*sin((1:N)/25);0.1*cos((1:N)/13)];
    l=u+[0;0;-2];
    [v,axis_id]=TRIGGER_SAMPLED_AXIS_SUPPORT_GATE( ...
        e,min([u,l],[],2),max([u,l],[],2),0.2,0.18,0.02);
    assert(v && axis_id>=1 && axis_id<=6, ...
        'CHECK_TRIGGER_SAMPLED_AXIS_PLANES:NoExclusion', ...
        'Thin true ellipsoid should have a separating axis.');
    [oldU,oldL,oldC,oldUnknown]= ...
        CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
        u,l,e,0.2,0.18,0.02,0,true);
    [newU,newL,newC,newUnknown,fast]= ...
        TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        u,l,e,0.2,0.18,0.02,0,true);
    assert(fast.sampled_axis_plane_excluded && ...
        isequal(oldU,newU)&&isequal(oldL,newL)&& ...
        isequal(oldC,newC)&&isequal(oldUnknown,newUnknown), ...
        'CHECK_TRIGGER_SAMPLED_AXIS_PLANES:NoParity', ...
        'Global axis exclusion disagrees with original exact geometry.');
    R.cases=R.cases+1;R.axis_exclusion_cases=R.axis_exclusion_cases+1;
    % Paired synthetic microbenchmark. No assertion of Flight speedup;
    % order alternates to reduce drift, with identical input/shape.
    T0=zeros(1,60);T1=zeros(1,60);
    for k=1:65
        if mod(k,2)
            a=tic;CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
                u,l,e,0.2,0.18,0.02,0,true);ta=1000*toc(a);
            a=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                u,l,e,0.2,0.18,0.02,0,true);tb=1000*toc(a);
        else
            a=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                u,l,e,0.2,0.18,0.02,0,true);tb=1000*toc(a);
            a=tic;CHECK_TRIGGER_SAMPLED_GEOMETRY_BASELINE.evaluate( ...
                u,l,e,0.2,0.18,0.02,0,true);ta=1000*toc(a);
        end
        if k>5,T0(k-5)=ta;T1(k-5)=tb;end
    end
    R.paired_median_baseline_ms=median(T0);
    R.paired_median_new_ms=median(T1);
    R.paired_max_baseline_ms=max(T0);
    R.paired_max_new_ms=max(T1);
    R.status='PASS_AXIS_PLANE_SAMPLE_PARITY__NOT_CONTINUOUS_PROOF';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
end
save(fullfile(root,'TRIGGER_SAMPLED_AXIS_PLANES_RESULT.mat'),'R');
fp=fopen(fullfile(root,'TRIGGER_SAMPLED_AXIS_PLANES_RESULT.txt'),'w');
if fp>=0
    fprintf(fp,'status=%s\ncases=%d\nfailures=%d\naxis_exclusions=%d\n', ...
        R.status,R.cases,R.failures,R.axis_exclusion_cases);
    fprintf(fp,'paired_median_ms=%.8f %.8f\npaired_max_ms=%.8f %.8f\n', ...
        R.paired_median_baseline_ms,R.paired_median_new_ms, ...
        R.paired_max_baseline_ms,R.paired_max_new_ms);
    fprintf(fp,'reason=%s\n',R.reason);fclose(fp);
end
end
function s=make_shape(type,dimensions,b,rotation,center,id)
s=struct('id',double(id),'type',type, ...
    'dimensions',double(dimensions), ...
    'center0',double(center(:)),'rotation',double(rotation), ...
    'velocity',zeros(3,1),'reference_time_s',0, ...
    'bounding_radius_m',double(b),'is_stationary',true, ...
    'center_norm_m',norm(center),'outline',zeros(0,2), ...
    'vertices',zeros(0,3),'faces',zeros(0,3));
end
