function R=CHECK_TRIGGER_NATIVE_SUPPORT_BATCH()
% Checks mathematical support UPPER ENCLOSURE, all supported static shapes.
% Geometry contact still uses original OBSTACLE_GEOMETRY_QUERY.distance.
% Does not imply continuous flight safety or WCET. Writes in common root.
R=struct('status','FAIL_NATIVE_SUPPORT_AUDIT','failures',0, ...
 'cases',0,'shapes',0,'normals',0,'worst_upper_gap',NaN, ...
 'matlab_wall_median_ms',NaN,'native_wall_median_ms',NaN, ...
 'conservative_pass',false,'reason','', ...
 'failure_stage','NOT_STARTED','failure_shape','','failure_trial',NaN, ...
 'failure_normal_index',NaN,'zero_normal_witnesses',0,'mex_path','');
try
    assert(exist('TRIGGER_SUPPORT_UPPER_NATIVE','file')==3, ...
        'TRIGGER_SUPPORT_UPPER_NATIVE MEX must be compiled first.');
    R.mex_path=which('TRIGGER_SUPPORT_UPPER_NATIVE');
    R.failure_stage='RNG_SETUP';
    prior=rng;
    cleanup=onCleanup(@()rng(prior)); %#ok<NASGU>
    rng(481773,'twister');
    types={'sphere','ellipsoid','box','cylinder','capsule', ...
        'cone','frustum','pyramid','regular_prism', ...
        'polygon_prism','torus','polyhedron','unknown_test'};
    shapes=numel(types);
    % Regression for a real bug: a zero normal gives a subnormal negative
    % lower endpoint after outward rounding 0+0; sqrt must intersect the
    % argument with the mathematically nonnegative range.
    R.failure_stage='ZERO_NORMAL_NATIVE_PROBE';
    probe=fixture('sphere',eye(3));
    zn=[zeros(3,1),[1;0;0],[0;1;0],[0;0;1]];
    pv=TRIGGER_SUPPORT_UPPER_NATIVE(probe,zn);
    assert(isequal(size(pv),[1 4])&&all(isfinite(pv)) && ...
        all(pv>=0),'Native support zero-normal probe failed.');
    assert(pv(1)<1e-12, ...
        'Native zero-normal support upper is unexpectedly inflated.');
    R.zero_normal_witnesses=1;
    worst=0;
    for j=1:shapes
        for trial=1:60
            R.failure_shape=types{j};
            R.failure_trial=trial;
            R.failure_normal_index=NaN;
            R.failure_stage='FIXTURE';
            [Q,~]=qr(randn(3));
            if det(Q)<0,Q(:,1)=-Q(:,1);end
            s=fixture(types{j},Q);
            normal=randn(3,161);
            if trial==1,normal(:,1)=0;end
            R.failure_stage='MATLAB_ORACLE';
            oracle=matlab_support(s,normal);
            R.failure_stage='NATIVE_SUPPORT_CALL';
            upper=TRIGGER_SUPPORT_UPPER_NATIVE(s,normal);
            R.failure_stage='SUPPORT_PARITY';
            assert(isequal(size(upper),size(oracle))&&all(isfinite(upper)));
            % Standard MATLAB formula itself is rounded. Demand native
            % upper covers it to within its separately allowed 32 ULPs;
            % additionally demand no gross over-inflation.
            gap=upper-oracle;
            tol=32*eps(max(1,abs(oracle)));
            bad=find(gap< -tol | ...
                gap>tol+1e-9*max(1,abs(oracle)),1,'first');
            if ~isempty(bad)
                R.failure_normal_index=bad;
                error('CHECK_TRIGGER_NATIVE_SUPPORT_BATCH:UpperParity', ...
                    'type=%s trial=%d j=%d oracle=%.17g upper=%.17g gap=%.17g allowed=%.17g', ...
                    types{j},trial,bad,oracle(bad),upper(bad), ...
                    gap(bad),tol(bad));
            end
            worst=max(worst,max(abs(gap)));
            R.cases=R.cases+1;
        end
    end
    R.shapes=shapes;R.normals=R.cases*161;
    R.failure_shape='';R.failure_trial=NaN;R.failure_normal_index=NaN;
    R.worst_upper_gap=worst;
    R.failure_stage='BENCHMARK';
    test=fixture('ellipsoid',eye(3));nn=randn(3,161);
    ta=zeros(1,200);tb=zeros(1,200);
    for k=1:200
        start=tic;matlab_support(test,nn);ta(k)=1000*toc(start);
        start=tic;TRIGGER_SUPPORT_UPPER_NATIVE(test,nn);tb(k)=1000*toc(start);
    end
    R.matlab_wall_median_ms=median(ta);
    R.native_wall_median_ms=median(tb);
    R.failure_stage='COMPLETE';
    R.conservative_pass=true;
    R.status='PASS_NATIVE_SUPPORT_UPPER_13_SHAPES__NOT_FLIGHT_OR_WCET';
catch ME
    R.failures=R.failures+1;
    R.reason=[ME.identifier ' | ' ME.message];
    if ~isempty(ME.stack)
        R.reason=sprintf('%s | at %s:%d',R.reason, ...
            ME.stack(1).name,ME.stack(1).line);
    end
end
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'TRIGGER_NATIVE_SUPPORT_BATCH_RESULT.mat'),'R');
fid=fopen(fullfile(root,'TRIGGER_NATIVE_SUPPORT_BATCH_RESULT.txt'),'w');
if fid>0
 fprintf(fid,'status=%s\nfailures=%d\ncases=%d\nshapes=%d\nnormals=%d\nupper_gap=%g\nmatlab_median_ms=%g\nnative_median_ms=%g\nzero_normal_witnesses=%d\nfailure_stage=%s\nfailure_shape=%s\nfailure_trial=%g\nfailure_normal_index=%g\nmex_path=%s\nreason=%s\n', ...
    R.status,R.failures,R.cases,R.shapes,R.normals, ...
    R.worst_upper_gap,R.matlab_wall_median_ms,R.native_wall_median_ms, ...
    R.zero_normal_witnesses,R.failure_stage,R.failure_shape, ...
    R.failure_trial,R.failure_normal_index,R.mex_path,R.reason);
 fclose(fid);
end
end
function s=fixture(typ,Q)
d=[0.3+rand, 0.4+rand, 0.5+rand];
s=struct('type',typ,'rotation',Q,'dimensions',d(:), ...
   'outline',[],'vertices',[],'bounding_radius_m',3.5);
if strcmp(typ,'polygon_prism')
 s.dimensions=struct('height',d(3));
end
if strcmp(typ,'pyramid')||strcmp(typ,'polyhedron')
 s.vertices=randn(9,3)*0.4;
end
if strcmp(typ,'regular_prism')||strcmp(typ,'polygon_prism')
 a=(0:8)'*2*pi/9;
 s.outline=[cos(a) sin(a)]*(0.2+d(1));
end
end
function r=matlab_support(s,n)
q=double(s.rotation)'*double(n);d=s.dimensions;
switch char(s.type)
 case 'sphere',r=double(d(1))*sqrt(sum(q.^2,1));
 case 'ellipsoid',r=sqrt(sum((double(d(:)).*q).^2,1));
 case 'box',r=sum(abs(q).*double(d(:))/2,1);
 case 'cylinder',r=double(d(1))*hypot(q(1,:),q(2,:))+double(d(2))/2*abs(q(3,:));
 case 'capsule',r=double(d(1))*sqrt(sum(q.^2,1))+double(d(2))/2*abs(q(3,:));
 case 'cone',r=max(double(d(1))*hypot(q(1,:),q(2,:))-double(d(2))/2*q(3,:),double(d(2))/2*q(3,:));
 case 'frustum',r=max(double(d(1))*hypot(q(1,:),q(2,:))-double(d(3))/2*q(3,:),double(d(2))*hypot(q(1,:),q(2,:))+double(d(3))/2*q(3,:));
 case {'pyramid','polyhedron'},r=max(double(s.vertices)*q,[],1);
 case 'regular_prism',r=max(double(s.outline)*q(1:2,:),[],1)+double(d(3))/2*abs(q(3,:));
 case 'polygon_prism',r=max(double(s.outline)*q(1:2,:),[],1)+double(d.height)/2*abs(q(3,:));
 case 'torus',r=double(d(1))*hypot(q(1,:),q(2,:))+double(d(2))*sqrt(sum(q.^2,1));
 otherwise,r=double(s.bounding_radius_m)*sqrt(sum(q.^2,1));
end
end
