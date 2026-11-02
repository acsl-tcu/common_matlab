function B = CHECK_TRIGGER_UNIVERSAL_GEOMETRY_SPEED()
% Bench a shape-agnostic narrow-phase candidate WITHOUT any Flight execution.
% Uses only synthetic shapes. No early horizon cut, no warmup, no source edits.
% This is NOT a certified real-time benchmark; no speed PASS is emitted.
root=fileparts(mfilename('fullpath'));
names={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
       'frustum','pyramid','regular_prism','torus', ...
       'polygon_prism','polyhedron'};
outline=struct('outline',[-1 -1;1 -1;1 0;0.25 0;0.25 1;-1 1], 'height',2);
verts=[-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
       -1 -1 1;1 -1 1;1 1 1;-1 1 1];
faces=[1 3 2;1 4 3;5 6 7;5 7 8;1 2 6;1 6 5; ...
       4 7 3;4 8 7;1 5 8;1 8 4;2 3 7;2 7 6];
mesh=struct('vertices',verts,'faces',faces);
dims={1.2,[1.2 .8 .7],[2 2 2],[1 2],[.5 2], ...
      [1 2],[1.2 .7 2],[2 2],[5 1 2],[1.2 .25],outline,mesh};
rz=[cos(.6) -sin(.6) 0; sin(.6) cos(.6) 0;0 0 1];
ry=[cos(.35) 0 sin(.35);0 1 0;-sin(.35) 0 cos(.35)];
R=rz*ry;
N=161; xs=linspace(-4,4,N);
far_u=repmat([40;40;40],1,N);
far_l=far_u+repmat([0;0;-2],1,N);
near_u=R*[xs; .3*ones(1,N); .2*ones(1,N)];
near_l=near_u+repmat(R*[0;0;-2],1,N);
records=repmat(struct('name','','prepare_ms',NaN,'far_ms',NaN, ...
    'near_ms',NaN,'near_contacts',0,'near_unknowns',0, ...
    'status','ERROR','error',''),1,numel(names));
for k=1:numel(names)
    r=records(k);r.name=names{k};
    try
        o=OBSTACLE_INFORMATION.define(300+k,names{k},[0 0 0], ...
                                       dims{k},'rotation',R);
        p_clock=tic;
        sh=OBSTACLE_GEOMETRY_QUERY.prepare(o);
        r.prepare_ms=1000*toc(p_clock);
        far_clock=tic;
        [uf,lf,cf,nf]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            far_u,far_l,sh,0.5,0.5,0,0);
        r.far_ms=1000*toc(far_clock);
        assert(~any([uf(:);lf(:);cf(:);nf(:)]), ...
            'CHECK_TRIGGER_UNIVERSAL_GEOMETRY_SPEED:Far', ...
            'The far exclusion oracle reported a contact/unknown.');
        near_clock=tic;
        [un,ln,cn,nn]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
            near_u,near_l,sh,0.5,0.5,0,0);
        r.near_ms=1000*toc(near_clock);
        r.near_contacts=nnz(un|ln|cn);
        r.near_unknowns=nnz(nn);
        r.status='MEASURED_SYNTHETIC_ONLY';
    catch ME
        r.status='ERROR';r.error=[ME.identifier ' ' ME.message];
    end
    records(k)=r;
end
B=struct('status','SYNTHETIC_GEOMETRY_TIMING__NOT_FLIGHT', ...
         'sample_count',N,'shape_count',numel(names), ...
         'error_count',sum(strcmp({records.status},'ERROR')), ...
         'records',records,'not_continuous_time_proof',true, ...
         'no_warmup',true,'no_runtime_changes',true);
if B.error_count>0
    B.status='SYNTHETIC_GEOMETRY_BENCHMARK_ERRORS';
end
save(fullfile(root,'TRIGGER_UNIVERSAL_GEOMETRY_SPEED.mat'),'B');
fid=fopen(fullfile(root,'TRIGGER_UNIVERSAL_GEOMETRY_SPEED.txt'),'w');
if fid>=0
    fclose_guard=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nshapes=%d\nsamples=%d\nerrors=%d\n', ...
        B.status,B.shape_count,N,B.error_count);
    fprintf(fid,'NOT a 4s flight timing or numerical/continuous-time safety proof.\n');
    for k=1:numel(records)
        r=records(k);
        fprintf(fid,'%s prepare_ms=%.6f far_ms=%.6f near_ms=%.6f ', ...
            r.name,r.prepare_ms,r.far_ms,r.near_ms);
        fprintf(fid,'near_contact_samples=%d unknown_samples=%d status=%s %s\n', ...
            r.near_contacts,r.near_unknowns,r.status,r.error);
    end
end
fprintf('[GENERIC GEO] %s | shapes=%d samples=%d errors=%d\n', ...
    B.status,B.shape_count,N,B.error_count);
end
