function A=CHECK_TRIGGER_SUPPORT_FUSION()
% First compare EXACT sampled contact/unknown outcomes against the source
% class copied under a temporary diagnostic name. Then report paired timing.
% Do NOT accept a slower or non-parity Flight kernel just because it ran.
A=struct('status','NOT_TESTED','cases',0,'failures',0, ...
    'legacy_median_ms',NaN,'fused_median_ms',NaN,'reason','');
prior_rng=rng; restore=onCleanup(@()rng(prior_rng)); %#ok<NASGU>
try
    rng(78631,'twister');
    types={'sphere','ellipsoid','box','cylinder','capsule', ...
           'cone','frustum','torus','regular_prism','polygon_prism'};
    for case_ix=1:60
        n=25+mod(case_ix,27);
        q=2*randn(3,n);
        l=q+0.4*randn(3,n)+[zeros(2,n);-2*ones(1,n)];
        for j=1:numel(types)
            S=make_shape(types{j},j,case_ix);
            for first=[false true]
                [a0,b0,c0,d0,info0]= ...
                    TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY_LEGACY.evaluate( ...
                    q,l,S,0.2,0.24,0.02,0,first);
                [a1,b1,c1,d1,info1]= ...
                    TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                    q,l,S,0.2,0.24,0.02,0,first);
                keys={'body_exact_queries','cable_exact_queries', ...
                      'body_support_excluded','cable_support_excluded', ...
                      'broadphase_excluded','first_hit_only','sampled_only'};
                assert(isequal(a0,a1)&&isequal(b0,b1) && ...
                    isequal(c0,c1)&&isequal(d0,d1), ...
                    'CHECK_TRIGGER_SUPPORT_FUSION:ContactParity', ...
                    'Fused support changed a physical contact/unknown bit.');
                for k=1:numel(keys)
                    key=keys{k};
                    assert(isequal(info0.(key),info1.(key)), ...
                        'CHECK_TRIGGER_SUPPORT_FUSION:WorkParity', ...
                        'Fused support changed geometric work/exclusion counts.');
                end
                A.cases=A.cases+1;
            end
        end
    end
    % Same input workload in A/B/A/B order, no claims of WCET on Windows.
    S=make_shape('ellipsoid',99,17);
    q=1.7*randn(3,161);
    l=q+[zeros(2,161);-2*ones(1,161)];
    ta=zeros(1,60);tb=zeros(1,60);
    for k=1:numel(ta)
        if mod(k,2)==1
            clk=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY_LEGACY.evaluate( ...
                q,l,S,.2,.24,.02,0,true);ta(k)=1000*toc(clk);
            clk=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,S,.2,.24,.02,0,true);tb(k)=1000*toc(clk);
        else
            clk=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                q,l,S,.2,.24,.02,0,true);tb(k)=1000*toc(clk);
            clk=tic;TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY_LEGACY.evaluate( ...
                q,l,S,.2,.24,.02,0,true);ta(k)=1000*toc(clk);
        end
    end
    A.legacy_median_ms=median(ta(11:end));
    A.fused_median_ms=median(tb(11:end));
    A.status='PASS_ALL_SHAPES_SAMPLED_PARITY__TIMING_NOT_WCET';
catch ME
    A.failures=A.failures+1;
    A.status='FAIL_SOURCE_PARITY_OR_RUNTIME';
    A.reason=[ME.identifier ' | ' ME.message];
end
f=fopen('TRIGGER_SUPPORT_FUSION_RESULT.txt','w');
assert(f>=0);cl=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'status=%s\ncases=%d\nfailures=%d\n', ...
    A.status,A.cases,A.failures);
fprintf(f,'legacy_median_ms=%.8g\nfused_median_ms=%.8g\n', ...
    A.legacy_median_ms,A.fused_median_ms);
fprintf(f,'reason=%s\n',A.reason);
end
function S=make_shape(type,j,k)
S=struct('id',j,'type',type,'dimensions',[], ...
    'center0',[cos(j+k);sin(j-k);0.2*j], ...
    'rotation',eye(3), 'bounding_radius_m',8, ...
    'is_stationary',true,'vertices',[], 'faces',[], 'outline',[]);
% Fixed SO(3) is varied independently from sensor position or flight epoch.
a=0.137*j+0.023*k;
b=0.071*k-0.093*j;
c=0.049*(j+k);
Rz=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rx=[1 0 0;0 cos(c) -sin(c);0 sin(c) cos(c)];
S.rotation=Rz*Ry*Rx;
switch type
    case 'sphere',S.dimensions=0.8;
    case 'ellipsoid',S.dimensions=[1.3;0.6;1.1];
    case 'box',S.dimensions=[2.1;0.9;1.7];
    case 'cylinder',S.dimensions=[1.1;1.8];
    case 'capsule',S.dimensions=[0.5;1.3];
    case 'cone',S.dimensions=[1.3;2.0];
    case 'frustum',S.dimensions=[1.2;0.7;2.0];
    case 'torus',S.dimensions=[1.2;0.3];
    case 'regular_prism'
        S.dimensions=[5;1.0;1.7];
        theta=(0:4)'*2*pi/5;
        S.outline=[cos(theta),sin(theta)];
    case 'polygon_prism'
        S.dimensions=struct('height',2.0);
        S.outline=[-1 -1;1 -1;1 1;-1 1];
    otherwise,error('CHECK_TRIGGER_SUPPORT_FUSION:Type','Unknown test type');
end
end
