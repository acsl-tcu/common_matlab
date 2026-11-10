function R=CHECK_TRAJECTORY_C9_WHOLE_NATIVE_AB()
% Development audit: 2 genuinely nominal-certified 3-obstacle C9 fixtures,
% each 5 repeats; former leaf MEX vs single whole native proof tree.
% Neither can certify execution, HLC tracking or actual Flight 25ms WCET.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_WHOLE_NATIVE_AB_RESULT';
R=struct('status','NOT_STARTED','stage','SETUP','error_message','', ...
    'error_report','','safe_to_commit',false,'completed_scenes',0, ...
    'scene_names',{{'cylinder_capsule_cone','torus_polygon_polyhedron'}}, ...
    'leaf_mex_ms',NaN(2,5),'whole_mex_ms',NaN(2,5), ...
    'leaf_p95_ms',NaN(2,1),'whole_p95_ms',NaN(2,1), ...
    'leaf_max_ms',NaN(2,1),'whole_max_ms',NaN(2,1), ...
    'leaf_nodes',NaN(2,5),'whole_nodes',NaN(2,5), ...
    'whole_certified',false(2,5),'whole_attempted',false(2,5), ...
    'whole_fallbacks',NaN(2,5),'whole_reasons',NaN(2,5), ...
    'whole_proof_only_ms',NaN(2,5),'whole_status',{cell(2,5)}, ...
    'strict_status_parity',true, 'nominal_certificate_regressions',0, ...
    'max_bound_difference_Nm',0, ...
    'singular_rejected',false, 'budget_rejected',false, ...
    'source_path','','native_path','', 'preflight_status','NOT_CHECKED', ...
    'result_zip',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    module_dir=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
    addpath(fullfile(root,'model'),fullfile(root,'controller'),module_dir, ...
        fullfile(root,'reference','REPLANNING_SYSTEM','OBSTACLE_INFORMATION'),'-begin');
    R.source_path=which('TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL');
    assert(strcmpi(R.source_path,fullfile(module_dir,'TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL.m')), ...
        'C9_WHOLE_AB:SourceShadowed','Wrong production MATLAB source');
    assert(contains(fileread(R.source_path),'use_native_whole_torque_kernel'), ...
        'C9_WHOLE_AB:SourceOld','Missing whole-tree native option');
    B=TRAJECTORY_C9_TORQUE_WHOLE_PREFLIGHT(false);
    R.preflight_status=B.status;R.native_path=B.native_path;
    assert(B.ready,'C9_WHOLE_AB:NoMEX', ...
        'Run TRAJECTORY_C9_TORQUE_WHOLE_PREFLIGHT(true) before this CHECK.');
    Leaf=TRAJECTORY_C9_NATIVE_TORQUE_PREFLIGHT(false);
    assert(Leaf.ready,'C9_WHOLE_AB:LeafBaselineMissing', ...
        'Previous leaf MEX must remain built for the same-candidate baseline.');
    [lo,hi]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
    lo=double(lo(:));hi=double(hi(:));
    P=DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
    d=double(P.get(["mass","jx","jy","jz","gravity","cableL"]));d=d(:);
    A=struct('uav_mass_kg',d(1),'payload_mass_kg',0.12, ...
        'uav_inertia_kgm2',d(2:4), ...
        'gravity_world_mps2',[0;0;-d(5)], ...
        'cable_length_m',d(6), ...
        'thrust_limits_N',[lo(1) hi(1)], ...
        'torque_limits_Nm',[lo(2:4) hi(2:4)], ...
        'minimum_tension_N',0,'acceleration_guard_mps2',0);
    catalog=c9_matrix_catalog();
    for j=1:2
        sc=[2 4];R.stage=sprintf('NOMINAL_SCENE_%d',sc(j));
        req=c9_matrix_request(A,1,sc(j),catalog);
        req=rmfield(req,'audit_pose_angles_deg');
        req.search_options.enable_13d_oracle_focused_qp_shapes=1;
        req.search_options.enable_13d_reuse_nominal_torque_certificate=1;
        G=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(req);
        assert(G.nominal_model_complete&&~G.safe_to_commit, ...
            'C9_WHOLE_AB:Fixture','Certified nominal fixture unavailable');
        for k=1:5
            opt=req.nominal_torque_options;
            opt.enable_fast_norm_sufficient_bound=1;
            opt.use_cancellation_attitude_jet=1;
            opt.use_bernstein_force_vector_hull=0;
            opt.use_bernstein_force_support_lower=0;
            opt.use_native_torque_leaf_kernel=1;
            opt.use_native_whole_torque_kernel=0;
            optwhole=opt;
            optwhole.use_native_torque_leaf_kernel=0;
            optwhole.use_native_whole_torque_kernel=1;
            if mod(j+k,2),order=[1 0];else,order=[0 1];end
            C=cell(1,2);ms=zeros(1,2);
            for m=order
                if m==0,o=opt;else,o=optwhole;end
                t=tic;
                C{m+1}=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
                    G.candidate_piece,req.authority,o);
                ms(m+1)=1000*toc(t);
            end
            b=C{1};n=C{2};
            R.leaf_mex_ms(j,k)=ms(1);R.whole_mex_ms(j,k)=ms(2);
            R.leaf_nodes(j,k)=b.checked_nodes;
            R.whole_nodes(j,k)=n.checked_nodes;
            R.whole_certified(j,k)=n.native_whole_torque_certified;
            R.whole_attempted(j,k)=n.native_whole_torque_attempted;
            R.whole_fallbacks(j,k)=n.native_whole_torque_fallbacks;
            R.whole_reasons(j,k)=n.native_whole_torque_reason_code;
            R.whole_proof_only_ms(j,k)=n.native_whole_torque_elapsed_ms;
            R.whole_status{j,k}=n.native_whole_torque_status;
            assert(n.native_whole_torque_ready&&n.native_whole_torque_attempted, ...
                'C9_WHOLE_AB:NotUsed','Compiled whole MEX was not used');
            assert(~b.safe_to_commit&&~n.safe_to_commit&& ...
                ~b.actual_HLC_tracking_proven&&~n.actual_HLC_tracking_proven, ...
                'C9_WHOLE_AB:Safety','MEX cannot enable Flight COMMIT');
            if b.conditional_nominal_all_time_torque_proven && ...
                    ~n.conditional_nominal_all_time_torque_proven
                R.nominal_certificate_regressions= ...
                    R.nominal_certificate_regressions+1;
            end
            if ~strcmp(b.status,n.status),R.strict_status_parity=false;end
            if all(isfinite(b.torque_bounds_Nm(:)))&& ...
                    all(isfinite(n.torque_bounds_Nm(:)))
                e=max(abs(b.torque_bounds_Nm(:)-n.torque_bounds_Nm(:)));
                R.max_bound_difference_Nm=max(R.max_bound_difference_Nm,e);
            end
        end
        R.completed_scenes=j;
        R.leaf_p95_ms(j)=percentile(R.leaf_mex_ms(j,:),0.95);
        R.whole_p95_ms(j)=percentile(R.whole_mex_ms(j,:),0.95);
        R.leaf_max_ms(j)=max(R.leaf_mex_ms(j,:));
        R.whole_max_ms(j)=max(R.whole_mex_ms(j,:));
    end
    % Two negative controls. The whole kernel must never grant PASS for
    % stationary heading singularity or an exhausted proof node budget.
    p0=G.candidate_piece;Pzero=double(p0.control_points_m);
    Pzero(1:2,:)=0;Pzero(3,:)=3;
    phys=[double(A.uav_mass_kg);double(A.payload_mass_kg); ...
       double(A.payload_mass_kg);double(A.cable_length_m); ...
       double(A.gravity_world_mps2(:));double(A.uav_inertia_kgm2(:));0;0];
    [~,yes,~,~,~]=TRAJECTORY_C9_TORQUE_WHOLE_NATIVE(Pzero,8, ...
        phys,[1;1;1],double(A.torque_limits_Nm),zeros(3,1),[8;511]);
    R.singular_rejected=~yes;
    [~,yes,~,~,~]=TRAJECTORY_C9_TORQUE_WHOLE_NATIVE( ...
        double(G.candidate_piece.control_points_m), ...
        double(G.candidate_piece.duration_s),phys,[1;1;1], ...
        double(A.torque_limits_Nm),zeros(3,1),[0;1]);
    R.budget_rejected=~yes;
    if R.nominal_certificate_regressions==0 && R.strict_status_parity&& ...
            all(R.whole_certified(:))&&all(R.whole_fallbacks(:)==0)&& ...
            R.singular_rejected&&R.budget_rejected
        R.status='PASS_WHOLE_NATIVE_NOMINAL_PARITY__FLIGHT_AND_HLC_PENDING';
    else
        R.status='REVIEW_WHOLE_NATIVE__DO_NOT_ENABLE_AUTOMATICALLY';
    end
catch ME
    R.status='FAIL_WHOLE_NATIVE_AB';
    R.error_message=[ME.identifier ' | ' ME.message];
    R.error_report=getReport(ME,'extended','hyperlinks','off');
end
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nerror=%s\nstage=%s\n',R.status,R.error_message,R.stage);
        fprintf(fid,'preflight=%s\n',R.preflight_status);
        fprintf(fid,'leaf_ms=%s\nwhole_ms=%s\n', ...
            mat2str(R.leaf_mex_ms,6),mat2str(R.whole_mex_ms,6));
        fprintf(fid,'leaf_P95=%s whole_P95=%s\n',mat2str(R.leaf_p95_ms,6),mat2str(R.whole_p95_ms,6));
        fprintf(fid,'leaf_MAX=%s whole_MAX=%s\n',mat2str(R.leaf_max_ms,6),mat2str(R.whole_max_ms,6));
        fprintf(fid,'whole_nodes=%s certified=%s fallback=%s reasons=%s\n', ...
            mat2str(R.whole_nodes),mat2str(R.whole_certified), ...
            mat2str(R.whole_fallbacks),mat2str(R.whole_reasons));
        fprintf(fid,'regressions=%d parity=%d difference=%.17g singular=%d budget=%d\n', ...
            R.nominal_certificate_regressions,R.strict_status_parity, ...
            R.max_bound_difference_Nm,R.singular_rejected,R.budget_rejected);
        if ~isempty(R.error_report),fprintf(fid,'%s\n',R.error_report);end
        fclose(fid);
    end
    zip(R.result_zip,{fullfile(root,[name '.mat']),fullfile(root,[name '.txt'])});
catch ME
    if isempty(R.error_message),R.error_message=['ZIP: ' ME.message];end
end
R.safe_to_commit=false;
end
function v=percentile(x,p)
x=sort(x(:));k=1+(numel(x)-1)*p;
i=floor(k);j=ceil(k);
v=x(i)+(k-i)*(x(j)-x(i));
end

function req=c9_matrix_request(base,pathid,scene,catalog)
t0=4;T=8;
[J0,J1,free]=c9_matrix_path(pathid,T);
req=struct('t0_s',t0,'duration_s',T, ...
    'flight_interval_s',[t0,t0+2*T], ...
    'start_jets',J0,'end_jets',J1,'nominal_free',free, ...
    'observed',struct([]), ...
    'radii',struct('uav_m',0.1,'cable_m',0.02,'payload_m',0.1), ...
    'authority',base, ...
    'geometry_options',struct('max_subdivision_depth',6, ...
        'max_node_count',127,'numerical_guard_m',0, ...
        'robust_tube_enabled',false,'trajectory_phase','f'), ...
    'force_options',struct('max_subdivision_depth',6, ...
        'max_node_count',127,'force_guard_N',0), ...
    'search_options',struct('probe_count',7,'max_outer_iterations',2, ...
        'max_projection_sweeps',60,'projection_tolerance_m',1e-8, ...
        'min_separation_m',1e-5, ...
        'enable_13d_variable_time_joint_search',1, ...
        'use_cached_13d_bernstein_gram',1, ...
        'enable_13d_witness_transverse_plane',1, ...
        'enable_13d_coherent_cable_corridor',1, ...
        'use_cached_13d_hull_cells',1, ...
        'enable_13d_nonlinear_merit_line_search',1, ...
        'enable_13d_interval_hull_line_search',1, ...
        'enable_13d_torque_point_proof_prefilter',1), ...
    'heading_options',struct('max_subdivision_depth',8, ...
        'max_node_count',511,'horizontal_velocity_guard_mps',0, ...
        'w_guard_mps2',0,'payload_jet_tolerance',1e-7*ones(3,10), ...
        'uav_jet_tolerance',1e-7*ones(3,8), ...
        'yaw_jet_tolerance',1e-7*ones(1,7)), ...
    'nominal_torque_options',struct('max_subdivision_depth',8, ...
        'max_node_count',511,'minimum_force_z_N',0, ...
        'minimum_planar_speed_mps',0,'torque_guard_Nm',zeros(3,1), ...
        'enable_fast_norm_sufficient_bound',true));
req.torque_options=req.nominal_torque_options;
% Each independent test case has an explicit current model mass; this
% is an AUDIT fixture and NOT a production default or future-mass forecast.
masses=[0.12 0.14 0.18];
req.authority.payload_mass_kg=masses(pathid);
ang=zeros(0,3);O=struct([]);
if scene==5
    indices=[1,4]; fractions=[0.35,0.72];
else
    indices=(3*(scene-1)+1):(3*scene);
    fractions=[0.30,0.50,0.70];
end
for j=1:numel(indices)
    s=fractions(j)*T;
    X=c9_matrix_analytic(pathid,s,0);
    if scene==5
        center=X+[0;2.5*(-1)^j;0];
    else
        center=X+[0;0.13*(-1)^j;0.04*(j-2)];
    end
    % Test all three axes; no collision-avoidance direction is prescribed.
    eul=[11*scene+5*j, 8*pathid-7*j, 13*scene-9*j];
    o=OBSTACLE_INFORMATION.define(100+100*pathid+10*scene+j, ...
        catalog(indices(j)).type,center.',catalog(indices(j)).dimensions, ...
        'euler_deg',eul);
    if isempty(O),O=o;else,O(end+1)=o;end %#ok<AGROW>
    ang(end+1,:)=eul; %#ok<AGROW>
end
req.observed=O;req.audit_pose_angles_deg=ang;
end

function [J0,J1,free]=c9_matrix_path(pathid,T)
J0=zeros(3,10);J1=zeros(3,10);
for r=0:9
    J0(:,r+1)=c9_matrix_analytic(pathid,0,r);
    J1(:,r+1)=c9_matrix_analytic(pathid,T,r);
end
free=zeros(3,4);
for j=1:4
    free(:,j)=c9_matrix_analytic(pathid,T*(9+j)/23,0);
end
end

function x=c9_matrix_analytic(pathid,t,r)
x=zeros(3,1);
switch pathid
    case 1 % Straight, no hand-coded obstacle exception.
        if r==0,x=[-2.6+0.65*t;0.15;3.65];
        elseif r==1,x=[0.65;0;0];end
    case 2 % Curved horizontal arc, analytic 0..9 endpoint jets.
        w=0.22;radius=2.8;
        if r==0
            x=[radius*sin(w*t);radius*(1-cos(w*t));3.6];
        else
            x=[radius*w^r*sin(w*t+r*pi/2); ...
               -radius*w^r*cos(w*t+r*pi/2);0];
        end
    case 3 % Saddle-like vertical cubic and lateral quadratic.
        v=t-4;
        switch r
            case 0,x=[-2.6+0.65*t;0.016*v^2-0.10;3.7+0.012*v^3-0.08*v];
            case 1,x=[0.65;0.032*v;0.036*v^2-0.08];
            case 2,x=[0;0.032;0.072*v];
            case 3,x=[0;0;0.072];
        end
    otherwise
        error('CHECK_C9_MATRIX:Path','Unknown audit trajectory family.');
end
end

function S=c9_matrix_catalog()
poly=struct('outline',[-0.26 -0.18;0.27 -0.18;0.29 0.15;-0.22 0.23], ...
    'height',0.65);
V=[0 0 0.4; -0.32 -0.28 -0.3;0.33 -0.27 -0.3;0 0.35 -0.3];
F=[2 3 4;1 3 2;1 4 3;1 2 4];
mesh=struct('vertices',V,'faces',F);
names={'sphere','ellipsoid','box','cylinder','capsule','cone', ...
    'frustum','pyramid','regular_prism','torus','polygon_prism','polyhedron'};
dims={0.26,[0.32 0.22 0.43],[0.54 0.42 0.68], ...
    [0.25 0.75],[0.20 0.48],[0.33 0.75], ...
    [0.29 0.18 0.65],[0.53 0.70],[5 0.29 0.70], ...
    [0.34 0.11],poly,mesh};
S=repmat(struct('type','','dimensions',[]),1,numel(names));
for i=1:numel(names)
    S(i).type=names{i};S(i).dimensions=dims{i};
end
end

