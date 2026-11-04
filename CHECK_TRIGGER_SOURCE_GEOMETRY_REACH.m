function R=CHECK_TRIGGER_SOURCE_GEOMETRY_REACH()
% Validates source-derived shape bounds, rejects forged geometry claims and
% checks the batch 4-second conditional geometry path on 13 shape families.
R=struct('status','FAIL_SOURCE_GEOMETRY_REACH', 'cases',0, ...
 'failures',0,'shape_families',0,'source_radii_valid',false, ...
 'forged_flags_ignored',false,'four_second_geometry',false,'reason','');
try
 old=rng; cleanup=onCleanup(@()rng(old)); %#ok<NASGU>
 rng(76845,'twister');
 names={'sphere','ellipsoid','box','cylinder','capsule', ...
  'cone','frustum','pyramid','regular_prism','polygon_prism', ...
  'torus','polyhedron'};
 K=numel(names);S=repmat(shape(names{1},eye(3)),1,K);
 for i=1:K
    [Q,~]=qr(randn(3));if det(Q)<0,Q(:,1)=-Q(:,1);end
    S(i)=shape(names{i},Q);S(i).id=double(i);
    % Deliberately forged caller fields: certification MUST derive bounds.
    S(i).bounding_radius_m=0;
    S(i).rigorous_outer_ball_certified=true;
    W=TRIGGER_SHAPE_SOURCE_OUTER_BALL(S(i));
    assert(W.source_geometry_enclosed && W.radius_upper_m>0);
    assert(W.rotation_norm_upper>=1-1e-12 && ...
        W.rotation_norm_upper<1+1e-9);
    % Repeatedly check R*v against the analytical local occupancy envelope.
    pts=local_occupied_samples(S(i),120);
    world=Q*pts;
    assert(all(sqrt(sum(world.^2,1))<=W.radius_upper_m+1e-11), ...
        'Analytic source outer ball excluded an occupied sample.');
    R.cases=R.cases+1;
 end
 R.shape_families=K;
 R.source_radii_valid=true;
 F=struct('conditional_chain_pass',true, ...
 'conditional_tension_positive_all_intervals',true, ...
 'manifold_compatible_intervals',true(1,2), ...
 'completed_intervals',2, ...
 'tube_lower',zeros(24,2),'tube_upper',zeros(24,2));
 F.tube_lower(1:3,:)=repmat([15;0;1],1,2);
 F.tube_upper(1:3,:)=F.tube_lower(1:3,:);
 F.tube_lower(13:15,:)=repmat([15;0;0],1,2);
 F.tube_upper(13:15,:)=F.tube_lower(13:15,:);
 C=struct('flowpipe',F,'time_grid_s',[0 2 4], ...
   'observed_ids',double(1:K), 'observed_prepared_shapes',S, ...
   'body',struct('r_uav_m',0.1,'r_payload_m',0.1,'r_cable_m',0.01), ...
   'observation_provenance','ACTUAL_SENSOR_HISTORY_ONLY');
 W=TRIGGER_SOURCE_GEOMETRY_REACH_GATE(C);
 assert(W.conditional_separation && isempty(W.possible_risk_ids) && ...
     all(W.conditional_geometry.separated_intervals(:)) && ...
     ~W.safe_to_commit && numel(W.source_balls)==K);
 R.four_second_geometry=true;R.cases=R.cases+1;
 % Forged tiny bounding radius cannot certify overlapping cable/obstacle.
 C.flowpipe.tube_lower(1:3,:)=0;
 C.flowpipe.tube_upper(1:3,:)=0;
 C.flowpipe.tube_lower(13:15,:)=0;
 C.flowpipe.tube_upper(13:15,:)=0;
 W=TRIGGER_SOURCE_GEOMETRY_REACH_GATE(C);
 assert(~W.conditional_separation && ...
     numel(W.possible_risk_ids)==K && ~W.safe_to_commit);
 R.forged_flags_ignored=true;R.cases=R.cases+1;
 % Unsupported shape must remain UNKNOWN, not accept forged radius or flag.
 C.observed_prepared_shapes(1).type='unlisted_new_shape';
 W=TRIGGER_SOURCE_GEOMETRY_REACH_GATE(C);
 assert(~W.conditional_separation && ~W.safe_to_commit);
 R.cases=R.cases+1;
 R.status='PASS_SOURCE_DERIVED_12_SHAPES_4S__INITIAL_PHYSICS_NOT_PROVEN';
catch ME
 R.failures=1;
 R.reason=[ME.identifier ' | ' ME.message];
end
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_SOURCE_GEOMETRY_REACH_RESULT.mat'),'R');
fid=fopen(fullfile(folder,'TRIGGER_SOURCE_GEOMETRY_REACH_RESULT.txt'),'w');
if fid>0
 fprintf(fid,'status=%s\nfailures=%d\ncases=%d\nshape_families=%d\nforged_flags_ignored=%d\nfour_second_geometry=%d\nreason=%s\n', ...
    R.status,R.failures,R.cases,R.shape_families,R.forged_flags_ignored,R.four_second_geometry,R.reason);
 fclose(fid);
end
end
function s=shape(typ,Q)
d=[0.3,0.4,0.5];
s=struct('id',1,'type',typ,'rotation',Q,'center0',[0;0;0], ...
 'dimensions',d(:),'outline',zeros(0,2),'vertices',zeros(0,3), ...
 'is_stationary',true,'bounding_radius_m',0, ...
 'rigorous_outer_ball_certified',true);
if strcmp(typ,'polygon_prism')
 s.dimensions=struct('height',d(3));
end
if any(strcmp(typ,{'polyhedron','pyramid'}))
 s.vertices=[0 0 0;0.3 0.3 0.2;0.3 -0.3 0.2; ...
 -0.3 0.3 0.2;-0.3 -0.3 0.2];
end
if any(strcmp(typ,{'regular_prism','polygon_prism'}))
 angle=(0:7)'*2*pi/8;
 s.outline=0.35*[cos(angle) sin(angle)];
end
end
function v=local_occupied_samples(s,N)
 d=s.dimensions;
 switch char(s.type)
 case 'sphere',v=randn(3,N);v=v./max(1,sqrt(sum(v.^2,1)))*d(1);
 case 'ellipsoid',v=d(1:3).*randn(3,N);v=v./max(1,sqrt(sum((v./d(1:3)).^2,1)));
 case 'box',v=(rand(3,N)-0.5).*d(1:3);
 case 'cylinder',theta=2*pi*rand(1,N);r=sqrt(rand(1,N))*d(1);v=[r.*cos(theta);r.*sin(theta);(rand(1,N)-0.5)*d(2)];
 case 'capsule',v=randn(3,N);v=v./max(1,sqrt(sum(v.^2,1)))*d(1);v(3,:)=v(3,:)+(rand(1,N)-0.5)*d(2);
 case 'cone',theta=2*pi*rand(1,N);z=(rand(1,N)-0.5)*d(2);r=sqrt(rand(1,N)).*d(1).*(0.5-z./d(2));v=[r.*cos(theta);r.*sin(theta);z];
 case 'frustum',theta=2*pi*rand(1,N);z=(rand(1,N)-0.5)*d(3);rmax=d(1)+(d(2)-d(1)).*(z/d(3)+0.5);r=sqrt(rand(1,N)).*rmax;v=[r.*cos(theta);r.*sin(theta);z];
 case 'torus',theta=2*pi*rand(1,N);phi=2*pi*rand(1,N);small=sqrt(rand(1,N))*d(2);rho=d(1)+small.*cos(phi);v=[rho.*cos(theta);rho.*sin(theta);small.*sin(phi)];
 case {'pyramid','polyhedron'},ids=randi(size(s.vertices,1),1,N);v=s.vertices(ids,:).';
 case {'polygon_prism','regular_prism'},ids=randi(size(s.outline,1),1,N);vv=s.outline(ids,:).'; if isstruct(d),h=d.height;else,h=d(3);end;v=[vv;(rand(1,N)-0.5)*h];
 otherwise,v=zeros(3,N);
 end
end
