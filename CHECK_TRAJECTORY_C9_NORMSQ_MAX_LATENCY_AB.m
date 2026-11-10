function R=CHECK_TRAJECTORY_C9_NORMSQ_MAX_LATENCY_AB()
% Development-only complete torque interval A/B. No full-Flight or HLC PASS.
% Uses the SAME certified multi-obstacle C9 piece for both numerical kernels.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_NORMSQ_MAX_LATENCY_AB_RESULT';
R=struct('status','NOT_RUN','stage','SETUP','error_message','', ...
    'error_report','','safe_to_commit',false,'scenes',{{'cylinder_capsule_cone','torus_polygon_polyhedron'}}, ...
    'baseline_ms',NaN(2,5),'vectorized_ms',NaN(2,5), ...
    'baseline_p95_ms',NaN(2,1),'vectorized_p95_ms',NaN(2,1), ...
    'baseline_max_ms',NaN(2,1),'vectorized_max_ms',NaN(2,1), ...
    'baseline_status',{{}},'vectorized_status',{{}}, ...
    'proof_regressions',0,'proof_nodes_equal',true, ...
    'max_bounds_abs_difference_Nm',0, ...
    'source_path','','source_feature_present',false, ...
    'integration_dependencies_valid',false, ...
    'integration_missing_features',{{}},'completed_scenes',0, ...
    'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    module_dir=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
    addpath(fullfile(root,'model'),fullfile(root,'controller'),module_dir, ...
        fullfile(root,'reference','REPLANNING_SYSTEM','OBSTACLE_INFORMATION'),'-begin');
    src=which('TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL');
    R.source_path=src;
    assert(strcmp(src,fullfile(module_dir,'TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL.m')), ...
        'C9_NORMSQ:ShadowedSource','MATLAB is resolving a different torque certificate source');
    actual=fileread(src);
    R.source_feature_present=contains(actual,'use_vectorized_normsq_kernel')&& ...
        contains(actual,'stationary_yaw_anchor_rad')&& ...
        contains(actual,'use_cancellation_attitude_jet');
    assert(R.source_feature_present,'C9_NORMSQ:PatchNotApplied', ...
        'Current source is missing the tested and prior required features.');
    % Integration gate: reject a mixed set of historical patch versions.
    % This tests feature presence in the LIVE resolved modules; it does
    % not replace their numerical or all-time mathematics audits.
    required={ ...
        'TRAJECTORY_C9_13D_TIME_JOINT_SEARCH', ...
             {'duration_13d_torque_repair_qp_attempts', ...
              'enable_13d_torque_only_qp_after_geometry_pass'}, ...
        'TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT', ...
             {'adjugate_preconditioner_3x3','NEUMANN'}, ...
        'TRAJECTORY_C9_HLC_EXECUTION_PROOF_BRIDGE', ...
             {'IDEAL_VIRTUAL_SIMULATION','guarantee_tier','DIRECT_INTERVAL'}, ...
        'TRAJECTORY_C9_13D_NATURAL_QP', ...
             {'use_shared_13d_duration_cache','use_cached_13d_bernstein_gram'}};
    missing={};
    for i=1:size(required,1)
        fname=required{i,1};features=required{i,2};
        resolved=which(fname);
        if ~strcmp(resolved,fullfile(module_dir,[fname '.m']))
            missing{end+1}=[fname ': SOURCE_SHADOWED_OR_MISSING']; %#ok<AGROW>
            continue
        end
        contents=fileread(resolved);
        for j=1:numel(features)
            if ~contains(contents,features{j})
                missing{end+1}=[fname ': MISSING_' features{j}]; %#ok<AGROW>
            end
        end
    end
    R.integration_missing_features=missing;
    R.integration_dependencies_valid=isempty(missing);
    assert(R.integration_dependencies_valid, ...
        'C9_NORMSQ:MixedPatchVersions', ...
        'Trajectory generation has a missing/shadowed dependency; see R.integration_missing_features.');
    [lo,hi]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
    lo=double(lo(:));hi=double(hi(:));
    P=DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
    d=double(P.get(["mass","jx","jy","jz","gravity","cableL"]));d=d(:);
    A=struct('uav_mass_kg',d(1),'payload_mass_kg',0.12, ...
        'uav_inertia_kgm2',d(2:4),'gravity_world_mps2',[0;0;-d(5)], ...
        'cable_length_m',d(6),'thrust_limits_N',[lo(1) hi(1)], ...
        'torque_limits_Nm',[lo(2:4) hi(2:4)], ...
        'minimum_tension_N',0,'acceleration_guard_mps2',0);
    catalog=c9_matrix_catalog();
    for j=1:2
        scene=[2 4];scene=scene(j);
        R.stage=sprintf('FULL_TORQUE_SCENE_%02d',scene);
        req=c9_matrix_request(A,1,scene,catalog);
        req=rmfield(req,'audit_pose_angles_deg');
        req.search_options.enable_13d_oracle_focused_qp_shapes=1;
        req.search_options.enable_13d_reuse_nominal_torque_certificate=1;
        S=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(req);
        assert(S.nominal_model_complete && ~S.safe_to_commit, ...
            'C9_NORMSQ:Fixture','A prior nominally certified multi-obstacle fixture regressed.');
        for k=1:5
            options=req.nominal_torque_options;
            options.use_cancellation_attitude_jet=1;
            modes=[0 1];if mod(j+k,2),modes=[1 0];end
            proof=cell(1,2);elapsed=zeros(1,2);
            for im=1:2
                mode=modes(im);
                options.use_vectorized_normsq_kernel=mode;
                t=tic;
                proof{mode+1}=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
                    S.candidate_piece,req.authority,options);
                elapsed(mode+1)=1000*toc(t);
            end
            old=proof{1};new=proof{2};
            R.baseline_ms(j,k)=elapsed(1);
            R.vectorized_ms(j,k)=elapsed(2);
            R.proof_nodes_equal=R.proof_nodes_equal && ...
                old.checked_nodes==new.checked_nodes && ...
                old.accepted_leaves==new.accepted_leaves;
            if old.conditional_nominal_all_time_torque_proven && ...
                    ~new.conditional_nominal_all_time_torque_proven
                R.proof_regressions=R.proof_regressions+1;
            end
            assert(~old.safe_to_commit && ~new.safe_to_commit);
            if all(isfinite([old.torque_bounds_Nm(:);new.torque_bounds_Nm(:)]))
                R.max_bounds_abs_difference_Nm=max( ...
                    R.max_bounds_abs_difference_Nm, ...
                    max(abs(old.torque_bounds_Nm(:)-new.torque_bounds_Nm(:))));
            end
            R.baseline_status{j,k}=old.status;
            R.vectorized_status{j,k}=new.status;
        end
        R.completed_scenes=j;
        R.baseline_p95_ms(j)=c9_matrix_percentile(R.baseline_ms(j,:),0.95);
        R.vectorized_p95_ms(j)=c9_matrix_percentile(R.vectorized_ms(j,:),0.95);
        R.baseline_max_ms(j)=max(R.baseline_ms(j,:));
        R.vectorized_max_ms(j)=max(R.vectorized_ms(j,:));
    end
    if R.proof_regressions>0 || ~R.proof_nodes_equal
        R.status='DO_NOT_ENABLE_VECTOR_NORMSQ__CERTIFICATE_PARITY_REGRESSION';
    else
        R.status='PASS_NORMSQ_CERTIFICATE_PARITY__REALTIME_AND_HLC_PENDING';
    end
catch ME
    R.status='FAIL_C9_NORMSQ_SETUP_OR_CERTIFICATE';
    R.error_message=[ME.identifier ' | ' ME.message];
    R.error_report=getReport(ME,'extended','hyperlinks','off');
end
try
    save(fullfile(root,[name '.mat']),'R');
    f=fopen(fullfile(root,[name '.txt']),'w');
    if f>=0
        fprintf(f,'status=%s; stage=%s; error=%s\n',R.status,R.stage,R.error_message);
        fprintf(f,'source=%s; feature=%d; scenes=%d; regressions=%d; nodes_equal=%d\n', ...
            R.source_path,R.source_feature_present,R.completed_scenes,R.proof_regressions,R.proof_nodes_equal);
        for j=1:2
            fprintf(f,'scene=%s; old_ms=%s; new_ms=%s; old_max=%.7f; new_max=%.7f; old_p95=%.7f; new_p95=%.7f\n', ...
                R.scenes{j},mat2str(R.baseline_ms(j,:),8),mat2str(R.vectorized_ms(j,:),8), ...
                R.baseline_max_ms(j),R.vectorized_max_ms(j),R.baseline_p95_ms(j),R.vectorized_p95_ms(j));
        end
        if ~isempty(R.error_report),fprintf(f,'%s\n',R.error_report);end
        fclose(f);
    end
    zip(R.zip_path,{fullfile(root,[name '.mat']),fullfile(root,[name '.txt'])});
catch ME
    if isempty(R.error_message),R.error_message=['RESULT_ZIP: ' ME.message];end
end
R.safe_to_commit=false;
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

function Q=c9_matrix_rot3(axis,theta)
a=axis(:)/norm(axis);
K=[0 -a(3) a(2);a(3) 0 -a(1);-a(2) a(1) 0];
Q=eye(3)+sin(theta)*K+(1-cos(theta))*K*K;
end

function v=c9_matrix_percentile(x,p)
x=sort(x(:));k=1+(numel(x)-1)*p;
i=floor(k);j=ceil(k);
v=x(i)+(k-i)*(x(j)-x(i));
end

function gap=c9_check_hull_upper_bound(A)
% Numerical sanity test only. The mathematical sufficient bound is the
% Bernstein convex-hull/interval construction, not this finite sampling.
[J0,J1,F]=c9_matrix_path(2,8);
P=TRAJECTORY_C9_FLAT_BERNSTEIN.build(4,8,J0,J1,F);
u=[0.15 0.42 0.78];
n=[1 0.3 -0.4; 0.3 1 -0.1; 0.1 -0.6 1];
n=bsxfun(@rdivide,n,sqrt(sum(n.^2,1)));
ns=repmat(n,1,3);
phase=repelem(u,3).';
lam=repmat([0;0.5;1],3,1);
Q=struct('geometry_plane_normals_3xm',ns, ...
    'geometry_plane_thresholds_m',zeros(9,1), ...
    'geometry_plane_lambdas',lam, ...
    'geometry_plane_phases',phase);
req=struct('authority',A);
B=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE(P,Q,req);
assert(B.valid&&isfinite(B.bound_m));
phases=sort(unique(phase));
edges=[0;(phases(1:end-1)+phases(2:end))/2;1];
PC=double(P.control_points_m);
AC=(23*22/P.duration_s^2)*diff(PC,2,2);
g=double(A.gravity_world_mps2(:));L=double(A.cable_length_m);
gap=Inf;
for j=1:3
    inds=find(abs(phase-phases(j))<1e-13);
    for t=linspace(edges(j),edges(j+1),101)
        p=c9_point(PC,t);
        a=c9_point(AC,t);
        w=g-a;q=w/norm(w);
        for z=inds.'
            x=p-lam(z)*L*q;
            residual=-ns(:,z).'*x;
            gap=min(gap,B.per_plane_upper_m(z)-residual);
        end
    end
end
end

function v=c9_point(P,u)
% de Casteljau polynomial point evaluator for CHECK only.
D=P;
for k=size(P,2)-1:-1:1
    D=(1-u)*D(:,1:k)+u*D(:,2:k+1);
end
v=D(:,1);
end

function pass=c9_check_point_false_zero(A)
% A degree-23 Bernstein polynomial with p_y(u)=-(u-1/2)^2.
% At the sampled phase u=1/2 the plane violation is exactly 0; at
% other times it is positive. The cell-hull score MUST expose this.
i=0:23;
P=zeros(3,24);P(1,:)=i/23;
P(2,:)=-((i.*(i-1))/(23*22)-i/23+1/4);
P(3,:)=3.5;
piece=struct('t0_s',4,'duration_s',8,'control_points_m',P);
Q=struct('geometry_plane_normals_3xm',[0;1;0], ...
    'geometry_plane_thresholds_m',0, ...
    'geometry_plane_lambdas',0, ...
    'geometry_plane_phases',0.5);
req=struct('authority',A);
S=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE(piece,Q,req);
p_mid=c9_point(P,0.5);
p_left=c9_point(P,0);
pass=S.valid&&abs(p_mid(2))<1e-12&& ...
    -p_left(2)>0.20&&S.bound_m>=-p_left(2)-1e-12;
end


function slack=c9_check_coherent_segment(A,catalog)
% Mathematical invariant: common supporting plane at the two endpoints
% excludes the ENTIRE straight cable, including arbitrary radii.
slack=Inf;
radii=[0.12,0.032,0.18];
for j=1:numel(catalog)
    o=OBSTACLE_INFORMATION.define(1000+j,catalog(j).type, ...
        [0.11*j,-0.07*j,3.6],catalog(j).dimensions, ...
        'euler_deg',[13*j,-9*j,7*j]);
    shape=TRAJECTORY_OBSTACLE_SUPPORT.prepare(o);
    n=[1+sin(j);0.3+cos(j);0.5+sin(2*j)];n=n/norm(n);
    h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,n);
    % Two distinct endpoint clearances, no assumption cable radius is
    % smaller than either endpoint radius.
    effective_u=max(radii(1),radii(2));
    effective_l=max(radii(3),radii(2));
    pU=(h+effective_u+0.28)*n;
    pL=(h+effective_l+0.34)*n;
    assert(n.'*pU>=h+effective_u && n.'*pL>=h+effective_l);
    for lam=linspace(0,1,101)
        x=(1-lam)*pL+lam*pU;
        slack=min(slack,n.'*x-h-radii(2));
    end
end
end

function [err,uncached_ms,cached_ms]=c9_check_cached_cell_equivalence(A)
% Compare full-cell Bernstein subdivision to the SAME fixed linear cell
% restriction at multiple C9 candidate controls; SEARCH score only.
[J0,J1,F]=c9_matrix_path(2,8);
base=TRAJECTORY_C9_FLAT_BERNSTEIN.build(4,8,J0,J1,F);
phase=repelem([0.10 0.31 0.53 0.77 0.91],2).';
n=[1 0.3 -0.4;0.3 1 -0.1;0.1 -0.6 1];
n=bsxfun(@rdivide,n,sqrt(sum(n.^2,1)));
ns=repmat(n(:,1:2),1,5);
Q=struct('geometry_plane_normals_3xm',ns, ...
    'geometry_plane_thresholds_m',zeros(10,1), ...
    'geometry_plane_lambdas',repmat([0;1],5,1), ...
    'geometry_plane_phases',phase);
req=struct('authority',A);
N=18;pieces=cell(1,N);
for k=1:N
    piece=base;
    piece.control_points_m(:,11:14)= ...
        piece.control_points_m(:,11:14)+0.01*reshape(sin((1:12)+k),3,4);
    pieces{k}=piece;
end
U=cell(1,N);V=cell(1,N);
t=tic;
for k=1:N
    U{k}=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE(pieces{k},Q,req);
end
uncached_ms=1000*toc(t);
cache=struct('enabled',true);
t=tic;
for k=1:N
    [V{k},cache]=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE( ...
        pieces{k},Q,req,cache);
end
cached_ms=1000*toc(t);
err=0;
for k=1:N
    assert(U{k}.valid==V{k}.valid);
    assert(strcmp(U{k}.status,V{k}.status));
    if U{k}.valid
        err=max(err,max(abs(U{k}.per_plane_upper_m- ...
            V{k}.per_plane_upper_m)));
        err=max(err,abs(U{k}.bound_m-V{k}.bound_m));
    end
end
end
