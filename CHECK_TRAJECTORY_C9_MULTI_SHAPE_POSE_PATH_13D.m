function R=CHECK_TRAJECTORY_C9_MULTI_SHAPE_POSE_PATH_13D()
% Development-only matrix audit. Install in common_matlab/ (not production).
% 3 reference families x 4 multi-obstacle scenes = 12 actual integrated
% nominal pipeline invocations, covering the canonical 12 static shapes.
% Additional clear-path controls on the same 3 trajectory families.
% No sampled observation is accepted as a continuous-time certificate.
% NO change to flight runner, HLC, EKF, plant or production obstacles.
% Results and zipped evidence are written to the CHECK's own folder.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_MULTI_SHAPE_POSE_PATH_13D_RESULT';
R=struct('status','NOT_RUN','error_message','', ...
    'error_identifier','','error_report','','stage','NOT_STARTED', ...
    'safe_to_commit',false, ...
    'shape_support_max_rotation_error',NaN, ...
    'shape_support_max_point_error',NaN, ...
    'shape_count',12,'reference_families',{{'straight','arc','saddle'}}, ...
    'scene_groups',{{'sphere_ellipsoid_box','cylinder_capsule_cone', ...
                     'frustum_pyramid_prism','torus_polygon_polyhedron', ...
                     'clear_control'}}, ...
    'expected_cases',15,'completed_cases',0, ...
    'conditional_nominal_complete_count',0, ...
    'changed_duration_integrated_count',0, ...
    'unknown_count',0,'exception_count',0, ...
    'median_ms',NaN,'p95_ms',NaN,'max_ms',NaN, ...
    'gram_max_relative_matrix_difference',NaN, ...
    'gram_off_median_ms',NaN,'gram_on_median_ms',NaN, ...
    'gram_qp_model_off_median_ms',NaN,'gram_qp_model_on_median_ms',NaN, ...
    'gram_model_status_equal',false,'cases',struct([]), ...
    'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    addpath(fullfile(root,'model'),fullfile(root,'controller'), ...
        fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION'), ...
        fullfile(root,'reference','REPLANNING_SYSTEM','OBSTACLE_INFORMATION'),'-begin');
    R.stage='READ_ACTUAL_HLC_FOUR_INPUT_LIMITS';
    [lo,hi]=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.source_input_limits();
    lo=double(lo(:));hi=double(hi(:));
    assert(numel(lo)==4&&numel(hi)==4&&all(isfinite([lo;hi])) ...
        &&all(hi>lo),'CHECK_C9_MATRIX:Limits', ...
        'Current HLC pre-clamp four-input limits are required.');
    R.stage='READ_AIRFRAME_PARAMETER__NO_MUTATION';
    P=DRONE_PARAM_SUSPENDED_LOAD("DIATONE");
    % Critical fix: DRONE_PARAM_SUSPENDED_LOAD.set loops length(p).
    % A CHAR 'loadmass' is 8 elements while v=0.14 has only 1.
    % DO NOT call P.set for this audit at all: the payload fixture is
    % supplied in request.authority per scene; airframe source is read-only.
    d=double(P.get(["mass","jx","jy","jz","gravity","cableL"]));
    d=d(:);
    assert(numel(d)==6&&all(isfinite(d))&&all(d>0), ...
        'CHECK_C9_MATRIX:Physics','Flight-model source data unavailable.');
    R.parameter_source=which('DRONE_PARAM_SUSPENDED_LOAD');
    R.payload_fixture_policy='REQUEST_LOCAL_ONLY__NO_PARAMETER_MUTATION';
    A=struct('uav_mass_kg',d(1),'payload_mass_kg',0.14, ...
        'uav_inertia_kgm2',d(2:4),'gravity_world_mps2',[0;0;-d(5)], ...
        'cable_length_m',d(6),'thrust_limits_N',[lo(1) hi(1)], ...
        'torque_limits_Nm',[lo(2:4) hi(2:4)], ...
        'minimum_tension_N',0,'acceleration_guard_mps2',0);
    R.stage='VALIDATE_TWELVE_SHAPES_AND_ROTATIONS';
    catalog=c9_matrix_catalog();
    assert(numel(catalog)==12);
    % Canonical shape support and SO(3) consistency, independent of C9.
    Q=c9_matrix_rot3([1;2;3],0.63);
    ns=[1 2 -1; -2 1 1; 0.7 -0.9 2.1];
    maxrot=0;maxpoint=0;
    for j=1:12
        obs=OBSTACLE_INFORMATION.define(800+j,catalog(j).type, ...
            [0.17*j,-0.13*j,3.6],catalog(j).dimensions, ...
            'euler_deg',[11+2*j, -17+j, 7*j]);
        S=TRAJECTORY_OBSTACLE_SUPPORT.prepare(obs);
        S2=S;S2.center=Q*S.center;S2.R=Q*S.R;
        for k=1:size(ns,2)
            n=ns(:,k);
            [h,p]=TRAJECTORY_OBSTACLE_SUPPORT.value_and_point(S,n);
            h2=TRAJECTORY_OBSTACLE_SUPPORT.value(S,n);
            hrot=TRAJECTORY_OBSTACLE_SUPPORT.value(S2,Q*n);
            maxpoint=max(maxpoint,abs(h-n.'*p));
            maxrot=max([maxrot,abs(hrot-h),abs(h-h2)]);
        end
    end
    R.shape_support_max_rotation_error=maxrot;
    R.shape_support_max_point_error=maxpoint;
    assert(maxrot<1e-9&&maxpoint<1e-9, ...
        'CHECK_C9_MATRIX:ShapeSupport','SO(3) support consistency failed.');
    R.stage='RUN_15_INTEGRATED_CASES';
    C=struct('reference','','scene','','obstacle_count',0, ...
        'shape_names',{{}},'pose_angles_deg',zeros(0,3), ...
        'payload_mass_fixture_kg',NaN,'initial_duration_s',NaN, ...
        'candidate_duration_s',NaN,'status','','search_status','', ...
        'failure_detail','','nominal_complete',false, ...
        'four_input_headroom',NaN(4,1),'four_input_bounds',NaN(4,2), ...
        'independent_proof_calls',NaN,'joint_proof_ms',NaN, ...
        'plane_ms',NaN,'qp_ms',NaN,'heading_ms',NaN, ...
        'torque_ms',NaN,'elapsed_ms',NaN, ...
        'safe_to_commit',false,'exception_identifier','','exception_message','', ...
        'exception_report','');
    rows=repmat(C,1,R.expected_cases);
    for pathid=1:3
        for scene=1:5
            row=5*(pathid-1)+scene;
            ticrow=tic;
            E=C;
            E.reference=R.reference_families{pathid};
            E.scene=R.scene_groups{scene};
            R.stage=sprintf('CASE_%02d_%s_%s',row,E.reference,E.scene);
            E.initial_duration_s=8;
            masses=[0.12 0.14 0.18];
            E.payload_mass_fixture_kg=masses(pathid);
            try
                req=c9_matrix_request(A,pathid,scene,catalog);
                E.obstacle_count=numel(req.observed);
                E.shape_names=arrayfun(@(o)char(o.geometry.type), ...
                    req.observed,'UniformOutput',false);
                E.pose_angles_deg=req.audit_pose_angles_deg;
                req=rmfield(req,'audit_pose_angles_deg');
                % Full integrated path: one 13D guide search plus the
                % independent all-time proofs and heading/torque checks.
                Z=TRAJECTORY_C9_INTEGRATED_NOMINAL_PIPELINE(req);
                E.status=char(Z.status);
                E.search_status=char(Z.search_status);
                E.failure_detail=char(Z.failure_detail);
                E.nominal_complete=logical(Z.nominal_model_complete);
                E.four_input_headroom=Z.nominal_four_input_headroom_4_N_Nm;
                E.four_input_bounds=Z.nominal_four_input_bounds_N_Nm;
                E.candidate_duration_s=Z.candidate_duration_s;
                E.independent_proof_calls=Z.proof_calls;
                E.joint_proof_ms=Z.joint_geometry_force_proof_ms;
                E.plane_ms=Z.separation_planes_ms;
                E.qp_ms=Z.joint_QP_ms;
                E.heading_ms=Z.heading_join_ms;
                E.torque_ms=Z.nominal_torque_ms;
                E.safe_to_commit=Z.safe_to_commit;
                assert(~E.safe_to_commit, ...
                    'CHECK_C9_MATRIX:UnsafeCommit', ...
                    'Nominal result incorrectly enabled flight execution.');
                if E.nominal_complete
                    assert(Z.nominal_geometry_tension_thrust_proven&& ...
                        Z.C7_yaw_C6_all_time_and_join_proven&& ...
                        Z.nominal_three_axis_torque_proven&& ...
                        Z.nominal_flat_feedforward_four_within_limits, ...
                        'CHECK_C9_MATRIX:FalseNominal', ...
                        'Integrated nominal PASS skipped an obligation.');
                    R.conditional_nominal_complete_count= ...
                        R.conditional_nominal_complete_count+1;
                    if isfinite(E.candidate_duration_s)&& ...
                            E.candidate_duration_s~=req.duration_s
                        R.changed_duration_integrated_count= ...
                            R.changed_duration_integrated_count+1;
                    end
                else
                    R.unknown_count=R.unknown_count+1;
                end
            catch ME
                E.status='EXCEPTION';
                E.exception_identifier=ME.identifier;
                E.exception_message=ME.message;
                E.exception_report=getReport(ME,'extended','hyperlinks','off');
                R.exception_count=R.exception_count+1;
            end
            E.elapsed_ms=1000*toc(ticrow);
            rows(row)=E;
            R.completed_cases=row;
            fprintf('[C9 MATRIX %02d/%02d] %s / %s | %s | %.2f ms\n', ...
                row,R.expected_cases,E.reference,E.scene,E.status,E.elapsed_ms);
        end
    end
    R.cases=rows;
    v=[rows.elapsed_ms];
    R.median_ms=median(v);
    R.p95_ms=c9_matrix_percentile(v,0.95);
    R.max_ms=max(v);
    % A/B comparison of the SAME 13D Gram and local model, with a
    % subsequent warm-cache call. Not a flight-time benchmark.
    R.stage='GRAM_CACHED_VS_UNCACHED_SAME_13D_QP';
    req=c9_matrix_request(A,2,1,catalog);
    req=rmfield(req,'audit_pose_angles_deg');
    shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(req.observed);
    phases=[0.17 0.39 0.61 0.83];
    radii=[req.radii.uav_m,req.radii.cable_m,req.radii.payload_m];
    [~,~]=TRAJECTORY_GROUND_SCOPE(req.geometry_options);
    ms_off=zeros(1,5);ms_on=zeros(1,5);
    full_off=zeros(1,5);full_on=zeros(1,5);
    maxdiff=0;same=true;
    for rep=1:5
        req.search_options.use_cached_13d_bernstein_gram=0;
        tt=tic;Off=TRAJECTORY_C9_13D_NATURAL_QP(req,shapes,phases,radii,true);
        full_off(rep)=1000*toc(tt);ms_off(rep)=Off.gram_build_ms;
        req.search_options.use_cached_13d_bernstein_gram=1;
        tt=tic;On=TRAJECTORY_C9_13D_NATURAL_QP(req,shapes,phases,radii,true);
        full_on(rep)=1000*toc(tt);ms_on(rep)=On.gram_build_ms;
        maxdiff=max(maxdiff,max(abs(On.H-Off.H),[],'all')/ ...
            max(1,max(abs(Off.H),[],'all')));
        same=same&&strcmp(On.status,Off.status)&& ...
            isequal(On.solved,Off.solved);
        if On.solved&&Off.solved
            maxdiff=max([maxdiff, ...
                max(abs(On.raw_A-Off.raw_A),[],'all')/ ...
                    max(1,max(abs(Off.raw_A),[],'all')), ...
                max(abs(On.raw_b-Off.raw_b),[],'all')/ ...
                    max(1,max(abs(Off.raw_b),[],'all'))]);
        end
    end
    R.gram_max_relative_matrix_difference=maxdiff;
    R.gram_off_median_ms=median(ms_off);
    R.gram_on_median_ms=median(ms_on);
    R.gram_qp_model_off_median_ms=median(full_off);
    R.gram_qp_model_on_median_ms=median(full_on);
    R.gram_model_status_equal=same;
    assert(same&&maxdiff<1e-10, ...
        'CHECK_C9_MATRIX:GramParity','Cached 13D model changed.');
    R.stage='COMPLETED_CASES_AND_GRAM_AB';
    if R.exception_count==0
        R.status='COMPLETED_MATRIX__NOMINAL_PASS_AND_UNKNOWN_REPORTED_SEPARATELY';
    else
        R.status='COMPLETED_MATRIX_WITH_EXCEPTIONS__INSPECT_CASE_LOG';
    end
catch ME
    R.status='FAIL_AUDIT_SETUP_OR_PARITY';
    R.error_identifier=ME.identifier;
    R.error_message=[ME.identifier ' | ' ME.message];
    R.error_report=getReport(ME,'extended','hyperlinks','off');
end
try
    save(fullfile(root,[name '.mat']),'R','-v7.3');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid<0,error('CHECK_C9_MATRIX:Save','Unable to write result TXT.');end
    fprintf(fid,'status=%s\nstage=%s\nerror=%s\n', ...
        R.status,R.stage,R.error_message);
    fprintf(fid,'payload_fixture_policy=REQUEST_LOCAL_ONLY__NO_PARAMETER_MUTATION\n');
    if ~isempty(R.error_report),fprintf(fid,'error_stack=\n%s\n',R.error_report);end
    fprintf(fid,'cases=%d/%d conditional_nominal=%d unknown=%d exceptions=%d\n', ...
        R.completed_cases,R.expected_cases,R.conditional_nominal_complete_count, ...
        R.unknown_count,R.exception_count);
    fprintf(fid,'elapsed_median_ms=%.10f p95_ms=%.10f max_ms=%.10f\n', ...
        R.median_ms,R.p95_ms,R.max_ms);
    fprintf(fid,'gram_off_ms=%.10f gram_on_ms=%.10f qp_off_ms=%.10f qp_on_ms=%.10f gram_relative=%.12g\n', ...
        R.gram_off_median_ms,R.gram_on_median_ms, ...
        R.gram_qp_model_off_median_ms,R.gram_qp_model_on_median_ms, ...
        R.gram_max_relative_matrix_difference);
    for i=1:numel(R.cases)
        x=R.cases(i);
        fprintf(fid,'case=%02d path=%s scene=%s mass=%.4g obs=%d status=%s search=%s Tnew=%.8g total_ms=%.5f proof_ms=%.5f planes_ms=%.5f qp_ms=%.5f heading_ms=%.5f torque_ms=%.5f\n', ...
            i,x.reference,x.scene,x.payload_mass_fixture_kg, ...
            x.obstacle_count,x.status,x.search_status, ...
            x.candidate_duration_s,x.elapsed_ms,x.joint_proof_ms, ...
            x.plane_ms,x.qp_ms,x.heading_ms,x.torque_ms);
        if ~isempty(x.exception_message)
            fprintf(fid,'exception=%s | %s\n',x.exception_identifier,x.exception_message);
            fprintf(fid,'exception_stack=\n%s\n',x.exception_report);
        end
    end
    fprintf(fid,'safe_to_commit=0\n');fclose(fid);
    zip(R.zip_path,{[name '.mat'],[name '.txt']},root);
catch ME
    R.status=['FAIL_RESULT_EXPORT_' ME.identifier];
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
        'use_cached_13d_bernstein_gram',1), ...
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
