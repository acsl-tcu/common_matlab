function R=TRAJECTORY_C9_25D_RISK_SEED_PREFLIGHT()
%TRAJECTORY_C9_25D_RISK_SEED_PREFLIGHT Flight-independent JIT warmup ONLY.
% Synthetic math data; NO live EKF, sensor, obstacle, HLC, mass, trajectory,
% actuator limits, trigger or detection masks are read, cached or modified.
% This warmup is optional and NOT a proof or a runtime parameter fallback.
clock=tic;
R=struct('ready',false,'status','NOT_RUN','error_message','', ...
         'shape_count',0,'support_batches',0,'elapsed_ms',NaN, ...
         'safe_to_commit',false,'source_runtime_authority_cached',false);
try
    kinds={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
        'frustum','pyramid','regular_prism','torus', ...
        'polygon_prism','polyhedron'};
    shapes=repmat(struct('id',0,'kind','','center',zeros(3,1), ...
        'R',eye(3),'dim',[],'local_vertices',zeros(0,3)),1,numel(kinds));
    % Rotate a purely synthetic test geometry to exercise SO(3) input code.
    a=0.37;b=-0.23;c=0.41;
    Rx=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
    Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
    Rz=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
    d={.12,[.2 .1 .3],[.3 .4 .5],[.1 .6],[.1 .3], ...
       [.2 .5],[.2 .12 .5],[.4 .5],[5 .2 .4], ...
       [.2 .05],struct('outline',[-.2 -.1;.2 -.1;.2 .1;-.2 .1], ...
       'height',.3),struct('vertices', ...
       [0 0 .2;.2 0 -.1;-.1 .2 -.1;-.1 -.2 -.1])};
    for k=1:numel(kinds)
        s=shapes(k);
        s.id=k;s.kind=kinds{k};s.center=[.05*k;1.3+.07*k;3];
        s.R=Rz*Ry*Rx;s.dim=d{k};
        switch s.kind
            case 'pyramid'
                aa=s.dim(1)/2;h=s.dim(2)/2;
                s.local_vertices=[aa aa -h;aa -aa -h;-aa aa -h; ...
                    -aa -aa -h;0 0 h];
            case 'regular_prism'
                theta=2*pi*(0:s.dim(1)-1)/s.dim(1);
                V=s.dim(2)*[cos(theta(:)),sin(theta(:))];
                h=s.dim(3)/2;
                s.local_vertices=[V,-h*ones(size(V,1),1); ...
                    V,h*ones(size(V,1),1)];
            case 'polygon_prism'
                V=s.dim.outline;h=s.dim.height/2;
                s.local_vertices=[V,-h*ones(size(V,1),1); ...
                    V,h*ones(size(V,1),1)];
            case 'polyhedron'
                s.local_vertices=s.dim.vertices;
        end
        shapes(k)=s;
    end
    % A degree27 3D polynomial used for JIT only, never a stored guide.
    u=(0:27)/27;
    piece=struct('t0_s',0,'duration_s',5, ...
        'control_points_m',[.1*u;5*u;3+0*u]);
    base=(1-cos(pi*(0:4)/4))/2;
    synthetic=struct('gravity_world_mps2',[0;0;-9.80665], ...
                     'cable_length_m',2);
    W=TRAJECTORY_C9_25D_SUPPORT_RISK_SEEDS(piece,shapes,base, ...
                                           [.3 .01 .2],synthetic,0,0);
    assert(W.valid,['Preflight risk kernel failed: ',W.status]);
    R.ready=true;R.status='READY_RISK_KERNEL_WARMED__NO_FLIGHT_CERTIFICATE';
    R.shape_count=numel(shapes);R.support_batches=W.support_batch_calls;
catch ME
    R.status=['UNKNOWN_RISK_PREFLIGHT_' ME.identifier];
    R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
