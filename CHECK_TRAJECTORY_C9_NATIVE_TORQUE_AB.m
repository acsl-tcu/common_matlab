function R=CHECK_TRAJECTORY_C9_NATIVE_TORQUE_AB()
% DEVELOPMENT ONLY; compilable fused interval torque leaf. No Flight commit.
% Actual certified nominal 2 multi-obstacle fixtures, each 5 repeats.
% Native and MATLAB both use non-outward-rounded floating-point interval
% operations, therefore mathematical real-arithmetic conditional only.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_NATIVE_TORQUE_AB_RESULT';
R=struct('status','NOT_STARTED','stage','SETUP','error_message','', ...
    'error_report','','safe_to_commit',false,'completed_scenes',0, ...
    'scene_names',{{'cylinder_capsule_cone','torus_polygon_polyhedron'}}, ...
    'baseline_ms',NaN(2,5),'native_ms',NaN(2,5), ...
    'baseline_p95_ms',NaN(2,1),'native_p95_ms',NaN(2,1), ...
    'baseline_max_ms',NaN(2,1),'native_max_ms',NaN(2,1), ...
    'baseline_nodes',NaN(2,5),'native_nodes',NaN(2,5), ...
    'native_calls',NaN(2,5),'native_valid_leaves',NaN(2,5), ...
    'native_fallbacks',NaN(2,5), ...
    'nominal_certificate_regressions',0, ...
    'strict_status_parity',true,'max_bound_difference_Nm',0, ...
    'source_path','','native_path','', ...
    'preflight_status','NOT_CHECKED', ...
    'result_zip',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    module_dir=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
    addpath(fullfile(root,'model'),fullfile(root,'controller'),module_dir, ...
        fullfile(root,'reference','REPLANNING_SYSTEM','OBSTACLE_INFORMATION'),'-begin');
    R.source_path=which('TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL');
    assert(strcmpi(R.source_path,fullfile(module_dir,'TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL.m')), ...
        'C9_NATIVE_AB:SourceShadowed','Wrong MATLAB torque source');
    assert(contains(fileread(R.source_path),'use_native_torque_leaf_kernel'), ...
        'C9_NATIVE_AB:SourceOld','Missing native torque feature');
    B=TRAJECTORY_C9_NATIVE_TORQUE_PREFLIGHT(false);
    R.preflight_status=B.status;
    R.native_path=B.native_path;
    assert(B.ready,'C9_NATIVE_AB:NoMEX', ...
        'Run TRAJECTORY_C9_NATIVE_TORQUE_PREFLIGHT(true) BEFORE this CHECK.');
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
        sc=[2 4];R.stage=sprintf('NOMINAL_PROOF_SCENE_%d',sc(j));
        req=c9_matrix_request(A,1,sc(j),catalog);
        req=rmfield(req,'audit_pose_angles_deg');
        req.search_options.enable_13d_oracle_focused_qp_shapes=1;
        req.search_options.enable_13d_reuse_nominal_torque_certificate=1;
        G=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(req);
        assert(G.nominal_model_complete && ~G.safe_to_commit, ...
            'C9_NATIVE_AB:Fixture','Previously certified fixture unavailable');
        for k=1:5
            base=req.nominal_torque_options;
            base.enable_fast_norm_sufficient_bound=1;
            base.use_cancellation_attitude_jet=1;
            base.use_bernstein_force_vector_hull=0;
            base.use_bernstein_force_support_lower=0;
            if mod(j+k,2),order=[1 0];else,order=[0 1];end
            C=cell(1,2);ms=zeros(1,2);
            for m=order
                opt=base;opt.use_native_torque_leaf_kernel=m;
                t=tic;
                C{m+1}=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
                    G.candidate_piece,req.authority,opt);
                ms(m+1)=1000*toc(t);
            end
            b=C{1};n=C{2};
            R.baseline_ms(j,k)=ms(1);R.native_ms(j,k)=ms(2);
            R.baseline_nodes(j,k)=b.checked_nodes;
            R.native_nodes(j,k)=n.checked_nodes;
            R.native_calls(j,k)=n.native_torque_leaf_calls;
            R.native_valid_leaves(j,k)=n.native_torque_leaf_valid;
            R.native_fallbacks(j,k)=n.native_torque_leaf_matlab_fallbacks;
            assert(n.native_torque_leaf_enabled && n.native_torque_leaf_valid>0, ...
                'C9_NATIVE_AB:NotUsed','Compiled native kernel was not used');
            assert(~b.safe_to_commit && ~n.safe_to_commit && ...
                ~b.numerically_directed_rounding_proven && ...
                ~n.numerically_directed_rounding_proven, ...
                'C9_NATIVE_AB:Safety','Native math cannot enable COMMIT');
            if b.conditional_nominal_all_time_torque_proven && ...
                    ~n.conditional_nominal_all_time_torque_proven
                R.nominal_certificate_regressions= ...
                    R.nominal_certificate_regressions+1;
            end
            if ~strcmp(b.status,n.status) || ...
                    b.checked_nodes~=n.checked_nodes
                R.strict_status_parity=false;
            end
            if all(isfinite(b.torque_bounds_Nm(:))) && ...
                    all(isfinite(n.torque_bounds_Nm(:)))
                e=max(abs(b.torque_bounds_Nm(:)-n.torque_bounds_Nm(:)));
                R.max_bound_difference_Nm=max(R.max_bound_difference_Nm,e);
            end
        end
        R.completed_scenes=j;
        R.baseline_p95_ms(j)=percentile(R.baseline_ms(j,:),0.95);
        R.native_p95_ms(j)=percentile(R.native_ms(j,:),0.95);
        R.baseline_max_ms(j)=max(R.baseline_ms(j,:));
        R.native_max_ms(j)=max(R.native_ms(j,:));
    end
    if R.nominal_certificate_regressions==0 && R.strict_status_parity && ...
            R.max_bound_difference_Nm<=1e-9 && ...
            all(R.native_fallbacks(:)==0)
        R.status='PASS_NATIVE_TORQUE_PARITY__REALTIME_HLC_ROUNDING_PENDING';
    else
        R.status='DO_NOT_ENABLE_NATIVE__PARITY_OR_FALLBACK_UNRESOLVED';
    end
catch ME
    R.status='FAIL_NATIVE_TORQUE_AB';
    R.error_message=[ME.identifier ' | ' ME.message];
    R.error_report=getReport(ME,'extended','hyperlinks','off');
end
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nerror=%s\nstage=%s\n', ...
            R.status,R.error_message,R.stage);
        fprintf(fid,'preflight=%s native_path=%s\n',R.preflight_status,R.native_path);
        fprintf(fid,'baseline_ms=%s\nnative_ms=%s\n', ...
            mat2str(R.baseline_ms,6),mat2str(R.native_ms,6));
        fprintf(fid,'baseline_P95=%s native_P95=%s\n', ...
            mat2str(R.baseline_p95_ms,6),mat2str(R.native_p95_ms,6));
        fprintf(fid,'baseline_MAX=%s native_MAX=%s\n', ...
            mat2str(R.baseline_max_ms,6),mat2str(R.native_max_ms,6));
        fprintf(fid,'baseline_nodes=%s native_nodes=%s\n', ...
            mat2str(R.baseline_nodes),mat2str(R.native_nodes));
        fprintf(fid,'native_calls=%s valid=%s fallbacks=%s\n', ...
            mat2str(R.native_calls),mat2str(R.native_valid_leaves), ...
            mat2str(R.native_fallbacks));
        fprintf(fid,'regressions=%d strict_status=%d bounds_diff=%.17g\n', ...
            R.nominal_certificate_regressions,R.strict_status_parity, ...
            R.max_bound_difference_Nm);
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

