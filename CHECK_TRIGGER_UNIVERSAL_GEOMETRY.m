function A = CHECK_TRIGGER_UNIVERSAL_GEOMETRY()
% All supported shapes use the SAME sensor geometry query, without changing
% sensor, obstacle definitions, runner, controller, estimator or plant.
% Synthetic finite-time geometry only; not a continuous-time flight proof.
root=fileparts(mfilename('fullpath'));
started=tic;
names={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
       'frustum','pyramid','regular_prism','torus', ...
       'polygon_prism','polyhedron'};
poly=struct('outline',[-1 -1;1 -1;1 0;0.25 0;0.25 1;-1 1], ...
            'height',2.0);
verts=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
       -1 -1 1;1 -1 1;1 1 1;-1 1 1];
faces=[1 3 2;1 4 3;5 6 7;5 7 8;1 2 6;1 6 5; ...
       4 7 3;4 8 7;1 5 8;1 8 4;2 3 7;2 7 6];
mesh=struct('vertices',verts,'faces',faces);
dims={1.2,[1.2 0.8 0.7],[2 2 2],[1 2],[0.5 2], ...
      [1 2],[1.2 0.7 2],[2 2],[5 1 2],[1.2 0.25],poly,mesh};
% SO(3) rotation of every shape, not a per-type special case.
rz=[cos(0.6) -sin(0.6) 0;sin(0.6) cos(0.6) 0;0 0 1];
ry=[cos(0.35) 0 sin(0.35);0 1 0;-sin(0.35) 0 cos(0.35)];
R=rz*ry;
records=repmat(struct('name','','passed',false,'clear',false, ...
      'sphere_contact',false,'cable_contact',false,'cable_unresolved',false, ...
      'elapsed_ms',NaN,'reason',''),1,numel(names));
for i=1:numel(names)
    tic_one=tic;
    rec=records(i);rec.name=names{i};
    try
        obs=OBSTACLE_INFORMATION.define(100+i,names{i},[0 0 0], ...
                                         dims{i},'rotation',R);
        prepared=OBSTACLE_GEOMETRY_QUERY.prepare(obs);
        shape=prepared(1);
        % Away from all original solids; no contact, no unclear result.
        [au,al,ac,an]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            R*[0;0;8],R*[0;0;9],shape,0.5,0.5,0,0);
        rec.clear=~any([au(:);al(:);ac(:);an(:)]);
        % A sphere whose center lies in the original solid.
        if strcmp(names{i},'torus')
            on_body=R*[1.2;0;0];
        else
            on_body=[0;0;0];
        end
        [au,~,~,~]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            on_body,R*[0;0;8],shape,0.5,0.5,0,0);
        rec.sphere_contact=any(au);
        % Cable endpoints outside. Centerline passes through true solid.
        [au,al,ac,an]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            R*[-5;0;0],R*[5;0;0],shape,0.5,0.5,0,0);
        rec.cable_contact=any(ac)&&~any(au)&&~any(al);
        rec.cable_unresolved=any(an);
        rec.passed=rec.clear && rec.sphere_contact && ...
            rec.cable_contact && ~rec.cable_unresolved;
        if ~rec.passed
            rec.reason='Unexpected clear/contact/unresolved result.';
        end
    catch ME
        rec.passed=false;rec.reason=sprintf('%s: %s',ME.identifier,ME.message);
    end
    rec.elapsed_ms=1000*toc(tic_one);
    records(i)=rec;
end
% Torus hole test: a cable lying entirely in the empty hole must be clear.
try
    torus=OBSTACLE_INFORMATION.define(999,'torus',[0 0 0],[1.2 0.25], ...
                                      'rotation',R);
    shapes=OBSTACLE_GEOMETRY_QUERY.prepare(torus);
    [u,l,c,n]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
        R*[0;0;-0.2],R*[0;0;0.2],shapes(1),0,0,0,0);
    hole_pass=~any([u(:);l(:);c(:);n(:)]);
catch
    hole_pass=false;
end
A=struct('status','FAIL_SYNTHETIC_GENERAL_GEOMETRY', ...
    'cases',numel(records),'passed',sum([records.passed]), ...
    'all_shapes_pass',all([records.passed]),'torus_hole_pass',hole_pass, ...
    'records',records,'elapsed_ms',1000*toc(started), ...
    'flight_tested',false,'continuous_time_certified',false);
if A.all_shapes_pass && A.torus_hole_pass
    A.status='PASS_SYNTHETIC_ALL_SENSOR_SHAPES__NOT_FLIGHT_OR_TIME_CERTIFICATE';
end
save(fullfile(root,'TRIGGER_UNIVERSAL_GEOMETRY_AUDIT.mat'),'A');
fid=fopen(fullfile(root,'TRIGGER_UNIVERSAL_GEOMETRY_AUDIT.txt'),'w');
if fid>=0
    cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nshapes=%d\npassed=%d\n', ...
        A.status,A.cases,A.passed);
    fprintf(fid,'torus_hole_pass=%d\ncontinuous_time_certified=0\n',hole_pass);
    for j=1:numel(records)
        fprintf(fid,'%s pass=%d clear=%d sphere=%d cable=%d unresolved=%d ms=%.6f %s\n', ...
            records(j).name,records(j).passed,records(j).clear, ...
            records(j).sphere_contact,records(j).cable_contact, ...
            records(j).cable_unresolved,records(j).elapsed_ms,records(j).reason);
    end
end
fprintf('[TRIGGER ALL SHAPES] %s | %d/%d | torus hole=%d\n', ...
        A.status,A.passed,A.cases,A.torus_hole_pass);
end
