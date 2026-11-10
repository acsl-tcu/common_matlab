function R=CHECK_TRAJECTORY_C9_THREE_FRONTS_AB()
% Development-only: three scoped improvements, not a Flight certificate.
% 1) Two multi-obstacle SAME C9 candidates: exact attitude cancellation ON/OFF.
% 2) Torque-only constraint-generation A/B, mandatory full oracle on updates.
% 3) Direct HLC beta2\\rhs interval: Cramer vs preconditioned solve.
% All sources/results are under common_matlab. Never put CHECK in production.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_THREE_FRONTS_AB_RESULT';
R=struct('status','NOT_STARTED','stage','SETUP', ...
    'error_message','','error_report','', ...
    'safe_to_commit',false,'matlab_validated_scenes',0, ...
    'repeats',5,'torque_scene_names',{{'cylinder_capsule_cone','torus_polygon_polyhedron'}}, ...
    'torque_time_old_ms',NaN(2,5),'torque_time_new_ms',NaN(2,5), ...
    'torque_old_status',{{}},'torque_new_status',{{}}, ...
    'torque_old_nodes',NaN(2,5),'torque_new_nodes',NaN(2,5), ...
    'torque_old_proven',false(2,5),'torque_new_proven',false(2,5), ...
    'torque_old_max_ms',NaN(2,1),'torque_new_max_ms',NaN(2,1), ...
    'torque_old_p95_ms',NaN(2,1),'torque_new_p95_ms',NaN(2,1), ...
    'torque_proof_regressions',0, ...
    'search_torque_only_status',{{}},'search_baseline_status',{{}}, ...
    'search_torque_only_uses',NaN(2,1), ...
    'search_torque_only_skipped_planes_upper',NaN(2,1), ...
    'search_baseline_ms',NaN(2,1),'search_torque_only_ms',NaN(2,1), ...
    'search_baseline_max_proof_ms',NaN(2,1), ...
    'search_torque_only_max_proof_ms',NaN(2,1), ...
    'hlc_cramer_status','','hlc_neumann_status','', ...
    'hlc_combined_status','','hlc_neumann_eta',NaN, ...
    'hlc_neumann_input',NaN(4,2), ...
    'hlc_neumann_random_sample_max_outside',NaN, ...
    'singular_beta_rejected',false, ...
    'singular_beta_warning_free',false, ...
    'hlc_exact_midpoint_regular_proven',false, ...
    'search_torque_repair_witness_count',NaN(2,1), ...
    'search_torque_repair_qp_attempts',NaN(2,1), ...
    'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    addpath(fullfile(root,'model'),fullfile(root,'controller'), ...
       fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION'), ...
       fullfile(root,'reference','REPLANNING_SYSTEM','OBSTACLE_INFORMATION'),'-begin');
    R.stage='NEUMANN_INTERVAL_TEST';
    B0=[0.8484878351651727 0.28314127824093827 0.1913004957337876; ...
        0.09523494937486476 0.6551589226038557 0.4915649145006196; ...
        0.26972300690458634 0.8869938388803055 1.0090338761319464];
    width=0.03695855065803905;
    BB=cat(3,B0-width,B0+width);
    rhs=0.01*[-ones(3,1),ones(3,1)]; thrust=[10 10];
    inputlow=[0;-1;-1;-1];inputhigh=[20;1;1;1];
    I0=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
       inputlow,inputhigh,BB,rhs,thrust,'CRAMER');
    I1=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
       inputlow,inputhigh,BB,rhs,thrust,'NEUMANN');
    I2=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
       inputlow,inputhigh,BB,rhs,thrust,'COMBINED');
    R.hlc_cramer_status=I0.status;
    R.hlc_neumann_status=I1.status;
    R.hlc_combined_status=I2.status;
    R.hlc_neumann_eta=I1.neumann_contraction_inf;
    R.hlc_neumann_input=I1.conditional_raw_input_enclosure;
    assert(~I0.conditional_headroom_all_positive && ...
        I1.conditional_headroom_all_positive && ...
        I2.conditional_headroom_all_positive && ...
        I1.neumann_regular_certified, ...
        'C9_TRIAGE:Neumann','Regular interval case did not improve.');
    rng(20261110,'twister');
    far=-Inf;
    for q=1:500
        b=BB(:,:,1)+rand(3,3).*(BB(:,:,2)-BB(:,:,1));
        rv=rhs(:,1)+rand(3,1).*(rhs(:,2)-rhs(:,1));
        x=b\rv;
        far=max(far,max([I1.conditional_raw_input_enclosure(2:4,1)-x; ...
                    x-I1.conditional_raw_input_enclosure(2:4,2)]));
    end
    R.hlc_neumann_random_sample_max_outside=far;
    assert(far<=0,'C9_TRIAGE:Sample','Algebraic interval contradiction.');
    zeroB=zeros(3,3,2);
    lastwarn('');
    IZ=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
       inputlow,inputhigh,zeroB,rhs,thrust,'NEUMANN');
    [warntext,~]=lastwarn;
    R.singular_beta_warning_free=isempty(warntext);
    R.singular_beta_rejected=~IZ.conditional_headroom_all_positive && ...
        ~IZ.neumann_regular_certified && ~IZ.interval_invertibility_certified;
    assert(R.singular_beta_rejected && R.singular_beta_warning_free, ...
        'C9_TRIAGE:SingularBeta','Singular midpoint must return UNKNOWN without matrix warning.');
    % Exact nonsingular midpoint box has zero Neumann error; this guards
    % against rejecting every HLC candidate after the warning fix.
    Ireg=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
       inputlow,inputhigh,cat(3,eye(3),eye(3)), ...
       [-0.01*ones(3,1),0.01*ones(3,1)],thrust,'NEUMANN');
    R.hlc_exact_midpoint_regular_proven= ...
        Ireg.neumann_regular_certified && ...
        Ireg.conditional_headroom_all_positive;
    assert(R.hlc_exact_midpoint_regular_proven,'C9_TRIAGE:IdentityBeta');

    R.stage='HLC_LIMITS_AND_MULTIOBSTACLE_TORQUE';
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
    scenes=[2 4];
    for j=1:2
        scene=scenes(j);
        R.stage=sprintf('TORQUE_SCENE_%02d',scene);
        req=c9_matrix_request(A,1,scene,catalog);
        req=rmfield(req,'audit_pose_angles_deg');
        req.search_options.enable_13d_oracle_focused_qp_shapes=1;
        req.search_options.enable_13d_reuse_nominal_torque_certificate=1;
        req.search_options.enable_13d_pre_torque_point_rejection=0;
        S=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(req);
        % The original CHECK used these two scenes as its certified
        % candidate fixtures. If they cease to work, report regression.
        assert(S.nominal_model_complete&&~S.safe_to_commit, ...
             'C9_TRIAGE:Fixture','Previously certified fixture no longer works.');
        for k=1:R.repeats
            options=req.nominal_torque_options;
            vals=cell(1,2);times=zeros(1,2);
            orders=[0,1];if mod(j+k,2),orders=[1,0];end
            for ix=1:2
                sw=orders(ix);options.use_cancellation_attitude_jet=sw;
                ticId=tic;
                vals{sw+1}=TRAJECTORY_C9_NOMINAL_TORQUE_INTERVAL( ...
                    S.candidate_piece,req.authority,options);
                times(sw+1)=toc(ticId)*1000;
            end
            T0=vals{1};T1=vals{2};
            R.torque_time_old_ms(j,k)=times(1);
            R.torque_time_new_ms(j,k)=times(2);
            R.torque_old_nodes(j,k)=T0.checked_nodes;
            R.torque_new_nodes(j,k)=T1.checked_nodes;
            R.torque_old_proven(j,k)= ...
                T0.conditional_nominal_all_time_torque_proven;
            R.torque_new_proven(j,k)= ...
                T1.conditional_nominal_all_time_torque_proven;
            if R.torque_old_proven(j,k)&&~R.torque_new_proven(j,k)
                R.torque_proof_regressions=R.torque_proof_regressions+1;
            end
            assert(~T0.safe_to_commit&&~T1.safe_to_commit);
        end
        R.matlab_validated_scenes=j;
        R.torque_old_max_ms(j)=max(R.torque_time_old_ms(j,:));
        R.torque_new_max_ms(j)=max(R.torque_time_new_ms(j,:));
        R.torque_old_p95_ms(j)=c9_matrix_percentile(R.torque_time_old_ms(j,:),0.95);
        R.torque_new_p95_ms(j)=c9_matrix_percentile(R.torque_time_new_ms(j,:),0.95);
    end
    % Measure changed 13D constraint generation on one known geometry-
    % problematic and one already certifiable multi-shape scene.
    R.stage='TORQUE_ONLY_CONSTRAINT_GENERATION';
    for j=1:2
        sc=[1,2];scene=sc(j);
        req=c9_matrix_request(A,1,scene,catalog);
        req=rmfield(req,'audit_pose_angles_deg');
        req.search_options.enable_13d_oracle_focused_qp_shapes=1;
        req.torque_options=req.nominal_torque_options;
        for mode=0:1
            req.search_options.enable_13d_torque_only_qp_after_geometry_pass=mode;
            t=tic;S=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(req);
            if mode==0
                R.search_baseline_ms(j)=1000*toc(t);
                R.search_baseline_status{j}=S.status;
                R.search_baseline_max_proof_ms(j)=S.search_result.duration_13d_max_oracle_call_ms;
            else
                R.search_torque_only_ms(j)=1000*toc(t);
                R.search_torque_only_status{j}=S.status;
                R.search_torque_only_uses(j)=S.search_result.duration_13d_torque_only_qp_uses;
                R.search_torque_repair_witness_count(j)= ...
                    S.search_result.duration_13d_torque_repair_point_witnesses;
                R.search_torque_repair_qp_attempts(j)= ...
                    S.search_result.duration_13d_torque_repair_qp_attempts;
                assert(R.search_torque_only_uses(j)== ...
                    R.search_torque_repair_qp_attempts(j), ...
                    'C9_TRIAGE:RepairCounter','Torque repair counters disagree.');
                R.search_torque_only_skipped_planes_upper(j)= ...
                    S.search_result.duration_13d_torque_only_qp_skipped_planes_upper;
                R.search_torque_only_max_proof_ms(j)= ...
                    S.search_result.duration_13d_max_oracle_call_ms;
            end
            assert(~S.safe_to_commit);
        end
    end
    if R.torque_proof_regressions>0
        R.status='COMPLETED_WITH_TORQUE_PROOF_REGRESSION__DO_NOT_ENABLE_FAST_DEFAULT';
    else
        R.status='PASS_COMPONENT_PARITY__HLC_AND_REALTIME_STILL_PENDING';
    end
catch ME
    R.status='FAIL_CORE_AB_OR_SETUP';
    R.error_message=[ME.identifier ' | ' ME.message];
    R.error_report=getReport(ME,'extended','hyperlinks','off');
end
R.safe_to_commit=false;
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s; stage=%s; error=%s\n',R.status,R.stage,R.error_message);
        fprintf(fid,'HLC_CRAMER=%s; HLC_NEUMANN=%s; HLC_COMBINED=%s; eta=%.12g; singular_rejected=%d; singular_warning_free=%d; exact_midpoint_regular=%d\n', ...
            R.hlc_cramer_status,R.hlc_neumann_status,R.hlc_combined_status, ...
            R.hlc_neumann_eta,R.singular_beta_rejected, ...
            R.singular_beta_warning_free,R.hlc_exact_midpoint_regular_proven);
        for j=1:2
            fprintf(fid,'scene=%s; torque_old_ms=%s; torque_new_ms=%s; old_proven=%s; new_proven=%s\n', ...
                R.torque_scene_names{j},mat2str(R.torque_time_old_ms(j,:),8), ...
                mat2str(R.torque_time_new_ms(j,:),8), ...
                mat2str(R.torque_old_proven(j,:)),mat2str(R.torque_new_proven(j,:)));
            fprintf(fid,'qp_case=%d; baseline=%s (%.8fms); changed=%s (%.8fms); torque_only_uses=%g\n', ...
                j,R.search_baseline_status{j},R.search_baseline_ms(j), ...
                R.search_torque_only_status{j},R.search_torque_only_ms(j), ...
                R.search_torque_only_uses(j));
            fprintf(fid,'qp_case=%d; torque_only_repair_point_witnesses=%g; torque_only_qp_attempts=%g\n', ...
                j,R.search_torque_repair_witness_count(j), ...
                R.search_torque_repair_qp_attempts(j));
        end
        if ~isempty(R.error_report),fprintf(fid,'%s\n',R.error_report);end
        fclose(fid);
    end
    zip(R.zip_path,{fullfile(root,[name '.mat']),fullfile(root,[name '.txt'])});
catch ME
    if isempty(R.error_message),R.error_message=['RESULT_PACKAGING: ' ME.message];end
end
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
