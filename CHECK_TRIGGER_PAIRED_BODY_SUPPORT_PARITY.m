function R=CHECK_TRIGGER_PAIRED_BODY_SUPPORT_PARITY()
% Nonflight A/B of OLD and optimized SAME exact-sampled geometry classes.
% Accept only exact outputs/diagnostic counters on every case. This is not
% flight safety or runtime certification. Reproducible seed, no live objects.
R=struct('status','NOT_COMPLETED','reason','', 'cases',0, ...
    'shapes',{{}},'all_identical',false, 'legacy_ms',NaN, ...
    'paired_ms',NaN,'paired_faster',false,'retain_optimization',false);
try
    oldrng=rng; cleanup=onCleanup(@()rng(oldrng));rng(17923,'twister');
    names={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
        'frustum','torus','regular_prism','pyramid','polygon_prism', ...
        'polyhedron'};
    dims={0.4,[0.5 0.3 0.7],[0.9 0.5 0.7],[0.35 0.9], ...
        [0.28 0.7],[0.45 0.9],[0.45 0.25 0.8], ...
        [0.65 0.15],[6 0.5 0.8],[0.8 0.9], ...
        struct('outline',[0 0;0.8 0;0.6 0.7;0.1 0.5],'height',0.8), ...
        struct('vertices',[-0.4 -0.4 -0.4;0.4 -0.4 -0.4; ...
            0 0.4 -0.4;0 0 0.4], ...
            'faces',[1 3 2;1 2 4;2 3 4;3 1 4])};
    n=21; dt=0; radii=[0.35 0.30 0.018];
    for s=1:numel(names)
        geom=OBSTACLE_SHAPES.define(names{s},dims{s});
        a=0.34*s; b=0.21*s;
        Qz=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
        Qy=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
        sh=struct('id',s,'center0',zeros(3,1), ...
            'rotation',Qz*Qy,'dimensions',geom.dimensions, ...
            'type',geom.type,'is_stationary',true, ...
            'bounding_radius_m',3.0, 'reference_time_s',0, ...
            'velocity',zeros(3,1));
        if strcmp(names{s},'regular_prism')
            z=2*pi*(0:5)'/6; sh.outline=0.5*[cos(z),sin(z)];
        elseif strcmp(names{s},'pyramid')
            B=0.4;h=0.45;
            sh.vertices=[-B -B -h;B -B -h;B B -h;-B B -h;0 0 h];
            sh.faces=[1 2 5;2 3 5;3 4 5;4 1 5;1 4 3;1 3 2];
        elseif strcmp(names{s},'polygon_prism')
            sh.outline=dims{s}.outline;
        elseif strcmp(names{s},'polyhedron')
            sh.vertices=dims{s}.vertices;
            sh.faces=dims{s}.faces;
        end
        for scenario=1:6
            angle=linspace(0,2*pi,n);
            if scenario<=2
                offset=[7;1;0];
            elseif scenario<=4
                offset=[0.6;0.8;0.25];
            else
                offset=[0;0;0];
            end
            u=offset+0.3*[cos(angle);sin(angle);0.6*cos(2*angle)];
            l=u+[0.13*sin(angle);0.15*cos(angle);-2+0*angle];
            if mod(scenario,2)==0
                l=l+0.15*randn(3,n);u=u+0.1*randn(3,n);
            end
            first=mod(scenario,2)==1;
            [a1,b1,c1,d1,e1]=CHECK_TRIGGER_LEGACY_SAMPLED_GEOMETRY. ...
                evaluate(u,l,sh,radii(1),radii(2),radii(3),dt,first);
            [a2,b2,c2,d2,e2]=TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY. ...
                evaluate(u,l,sh,radii(1),radii(2),radii(3),dt,first);
            assert(isequal(a1,a2)&&isequal(b1,b2)&&isequal(c1,c2)&& ...
                isequal(d1,d2)&&isequaln(e1,e2), ...
                'CHECK_TRIGGER_PAIRED_BODY_SUPPORT_PARITY:Mismatch', ...
                'Different sampled collision outcome for %s case %d', ...
                names{s},scenario);
            R.cases=R.cases+1;
        end
    end
    R.shapes=names; R.all_identical=true;
    % Compare repeated identical workloads after both paths have executed.
    % Do not claim a timing bound or universal speedup from this diagnostic.
    sh=struct('id',1,'center0',zeros(3,1), ...
        'rotation',eye(3),'dimensions',[0.5 0.4 0.8], ...
        'type','box','is_stationary',true, ...
        'bounding_radius_m',3, 'reference_time_s',0, ...
        'velocity',zeros(3,1));
    kk=161;angle=linspace(0,pi,kk);
    u=[0.9+0.1*cos(angle);0.7+0.1*sin(angle);0.4+0*angle];
    l=u+[0.05+0*angle;0*angle;-2+0*angle];
    % Use A/B/A order to reduce simple warming/order bias; the resulting
    % numbers are still environment-dependent, not hard deadlines.
    nrep=10; batches=5;
    times_a=zeros(1,batches);times_b=zeros(1,batches);
    for batch=1:batches
        tic;for t=1:nrep
            CHECK_TRIGGER_LEGACY_SAMPLED_GEOMETRY.evaluate( ...
                u,l,sh,radii(1),radii(2),radii(3),0,true);
        end
        a1=1000*toc/nrep;
        tic;for t=1:nrep
            TRIGGER_UNIVERSAL_SAMPLED_GEOMETRY.evaluate( ...
                u,l,sh,radii(1),radii(2),radii(3),0,true);
        end
        times_b(batch)=1000*toc/nrep;
        tic;for t=1:nrep
            CHECK_TRIGGER_LEGACY_SAMPLED_GEOMETRY.evaluate( ...
                u,l,sh,radii(1),radii(2),radii(3),0,true);
        end
        a2=1000*toc/nrep;
        times_a(batch)=(a1+a2)/2;
    end
    R.legacy_ms=median(times_a);
    R.paired_ms=median(times_b);
    R.paired_faster=R.paired_ms<R.legacy_ms;
    % A single A/B is insufficient for a production speed claim.
    R.retain_optimization=false;
    R.status='PASS_EXACT_SAMPLED_PARITY__TIMING_NEEDS_REPEATED_ABA';
catch ME
    R.status='FAIL_OR_UNKNOWN_GEOMETRY_PARITY';
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    fid=fopen('TRIGGER_PAIRED_BODY_SUPPORT_PARITY_RESULT.txt','w');
    if fid>0
        fprintf(fid,'status=%s\n',R.status);
        fprintf(fid,'cases=%d\n',R.cases);
        fprintf(fid,'legacy_ms=%.9f paired_ms=%.9f\n',R.legacy_ms,R.paired_ms);
        fprintf(fid,'all_identical=%d paired_faster=%d\n',R.all_identical,R.paired_faster);
        fprintf(fid,'reason=%s\n',R.reason);fclose(fid);
    end
catch
end
end
