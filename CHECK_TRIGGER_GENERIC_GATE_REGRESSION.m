function A = CHECK_TRIGGER_GENERIC_GATE_REGRESSION()
% Offline, read-only regression for the trigger's shape-independent engine.
% Does NOT execute Flight, change run/plant/HLC/EKF, or modify sensor memory.
% Compare body-sphere hits with the original sensor distance oracle at EVERY
% sample; check early-first contact against exhaustive processing; test cable
% crossings and a torus hole. Geometry is sampled-only, NOT a time proof.
root=fileparts(mfilename('fullpath'));
names={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
       'frustum','pyramid','regular_prism','torus', ...
       'polygon_prism','polyhedron'};
polygon=struct('outline',[-1 -1;1 -1;1 0;.25 0;.25 1;-1 1], 'height',2);
verts=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
       -1 -1 1;1 -1 1;1 1 1;-1 1 1];
faces=[1 3 2;1 4 3;5 6 7;5 7 8;1 2 6;1 6 5; ...
       4 7 3;4 8 7;1 5 8;1 8 4;2 3 7;2 7 6];
mesh=struct('vertices',verts,'faces',faces);
dims={1.2,[1.2 .8 .7],[2 2 2],[1 2],[.5 2], ...
      [1 2],[1.2 .7 2],[2 2],[5 1 2],[1.2 .25],polygon,mesh};
rz=[cos(.6) -sin(.6) 0;sin(.6) cos(.6) 0;0 0 1];
ry=[cos(.35) 0 sin(.35);0 1 0;-sin(.35) 0 cos(.35)];
R=rz*ry;
N=161; x=linspace(-4,4,N);
u=R*[x;.3*ones(1,N);.2*ones(1,N)];
l=u+repmat(R*[0;0;-2],1,N);
far=repmat([40;40;40],1,N);
records=repmat(struct('shape','','passed',false,'body_oracle',false, ...
    'first_contact_consistent',false,'crossing',false, ...
    'global_far_excluded',false,'first_ms',NaN, ...
    'full_ms',NaN,'exact_body_queries',0,'exact_cable_queries',0, ...
    'support_exclusions',0,'reason',''),1,numel(names));
for j=1:numel(names)
    rec=records(j);rec.shape=names{j};
    try
        o=OBSTACLE_INFORMATION.define(100+j,names{j},[0 0 0],dims{j}, ...
                'rotation',R);
        sh=OBSTACLE_GEOMETRY_QUERY.prepare(o);
        sh=sh(1);
        % Compare EVERY predicted sample against the current sensor oracle.
        oracle_u=false(1,N);oracle_l=false(1,N);
        for k=1:N
            oracle_u(k)=OBSTACLE_GEOMETRY_QUERY.distance( ...
                u(:,k),sh,0,sh.center0)<=0.5;
            oracle_l(k)=OBSTACLE_GEOMETRY_QUERY.distance( ...
                l(:,k),sh,0,sh.center0)<=0.5;
        end
        full_clock=tic;
        [fu,fl,fc,fn,fi]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            u,l,sh,.5,.5,0,0,false);
        rec.full_ms=1000*toc(full_clock);
        rec.body_oracle=isequal(fu,oracle_u) && isequal(fl,oracle_l);
        first_clock=tic;
        [pu,pl,pc,pn,pi]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            u,l,sh,.5,.5,0,0,true);
        rec.first_ms=1000*toc(first_clock);
        idxfull=find(fu|fl|fc,1,'first');
        idxfirst=find(pu|pl|pc,1,'first');
        rec.first_contact_consistent=isequal(idxfull,idxfirst) && ...
            (~any(fn) || any(pn) || ~isempty(idxfirst));
        % A thin finite cable crossing the occupied solid.
        [bu,bl,bc,bn]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            R*[-5;0;0],R*[5;0;0],sh,.5,.5,0,0);
        rec.crossing=~any(bu)&&~any(bl)&&any(bc)&&~any(bn);
        [au,al,ac,an,ai]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            far,far+repmat([0;0;-2],1,N),sh,.5,.5,0,0);
        rec.global_far_excluded=~any([au(:);al(:);ac(:);an(:)]) && ...
            ai.broadphase_excluded==3*N;
        rec.exact_body_queries=pi.body_exact_queries;
        rec.exact_cable_queries=pi.cable_exact_queries;
        rec.support_exclusions=pi.body_support_excluded+ ...
            pi.cable_support_excluded;
        rec.passed=rec.body_oracle && rec.first_contact_consistent && ...
            rec.crossing && rec.global_far_excluded;
        if ~rec.passed
            rec.reason='Oracle, first-event order, crossing, or far-pruning disagreement.';
        end
    catch ME
        rec.passed=false;
        rec.reason=sprintf('%s: %s',ME.identifier,ME.message);
    end
    records(j)=rec;
end
% Concave torus hole: crossing the empty center MUST NOT be called contact.
try
    torus=OBSTACLE_INFORMATION.define(999,'torus',[0 0 0],[1.2 .25], ...
                                      'rotation',R);
    sh=OBSTACLE_GEOMETRY_QUERY.prepare(torus);
    [hu,hl,hc,hn]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        R*[0;0;-.2],R*[0;0;.2],sh(1),0,0,0,0);
    hole=~any([hu(:);hl(:);hc(:);hn(:)]);
catch
    hole=false;
end
% A future shape with no specialized support expression must conservatively
% fall back to the prepared enclosing radius (never an invented shape).
try
    newtype=sh(1);newtype.type='future_sensor_defined_shape';
    v=[1 0 0;0 1 0;0 0 1];
    h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(newtype,v);
    fallback=isequal(h,double(newtype.bounding_radius_m)*ones(1,3));
catch
    fallback=false;
end
A=struct('status','FAIL_GENERIC_GATE_REGRESSION', ...
  'shape_count',numel(records),'passed',nnz([records.passed]), ...
  'torus_hole_pass',hole,'future_type_support_fallback',fallback, ...
  'records',records,'flight_tested',false, ...
  'continuous_time_certified',false,'no_warmup',true);
if A.passed==A.shape_count&&hole&&fallback
    A.status='PASS_GENERIC_SAMPLED_GEOMETRY_REGRESSION__NOT_FLIGHT_OR_TIME_PROOF';
end
save(fullfile(root,'TRIGGER_GENERIC_GATE_REGRESSION.mat'),'A');
fid=fopen(fullfile(root,'TRIGGER_GENERIC_GATE_REGRESSION.txt'),'w');
if fid>=0
    cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nshapes=%d\npassed=%d\n', ...
        A.status,A.shape_count,A.passed);
    fprintf(fid,'torus_hole=%d\nfuture_type_support_fallback=%d\n',hole,fallback);
    fprintf(fid,'flight_tested=0\ncontinuous_time_certified=0\n');
    for j=1:numel(records)
        x=records(j);
        fprintf(fid,'%s pass=%d oracle=%d first=%d crossing=%d far=%d ', ...
            x.shape,x.passed,x.body_oracle,x.first_contact_consistent, ...
            x.crossing,x.global_far_excluded);
        fprintf(fid,'first_ms=%.6f full_ms=%.6f body_calls=%d cable_calls=%d pruned=%d %s\n', ...
            x.first_ms,x.full_ms,x.exact_body_queries, ...
            x.exact_cable_queries,x.support_exclusions,x.reason);
    end
end
fprintf('[GENERIC TRIGGER] %s | %d/%d | torus hole=%d\n', ...
    A.status,A.passed,A.shape_count,A.torus_hole_pass);
end
