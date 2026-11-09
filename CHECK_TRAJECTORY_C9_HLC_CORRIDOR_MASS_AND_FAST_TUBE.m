function R=CHECK_TRAJECTORY_C9_HLC_CORRIDOR_MASS_AND_FAST_TUBE()
% Separate mathematics-only fixture; no assertion of actual flight safety.
% Place at common_matlab/ directly; results saved there independently of pwd.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_HLC_CORRIDOR_MASS_AND_FAST_TUBE_RESULT';
R=struct('status','FAIL_NOT_STARTED','error_message','', ...
    'safe_to_commit',false,'actual_HLC_tracking_proven',false, ...
    'virtual_status','','corridor_status','','mass_status','', ...
    'min_obstacle_slack_m',NaN,'min_ground_slack_m',NaN, ...
    'conditional_tension_lower_N',NaN, ...
    'collision_tube_rejected',false,'ground_tube_rejected',false, ...
    'mass_zero_lower_rejected',false,'exaggerated_w_bound_rejected',false, ...
    'time_gap_rejected',false,'geometric_series_max_relative_error',NaN, ...
    'corridor_gate_median_ms',NaN,'total_elapsed_ms',NaN, ...
    'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
total=tic;
try
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');
    % No fixed source parameters are placed in production: these are only
    % transparent mathematical fixtures for the new implications.
    F=struct('F1',[9 6],'F2',[1 6 15 20 15 6], ...
        'F3',[1 6 15 20 15 6],'F4',[9 6]);
    V=TRAJECTORY_C9_HLC_SAMPLED_VIRTUAL_TUBE( ...
        F,.025,4,[1e-5;1e-5;1e-5;1e-5], ...
        [1e-7;1e-7;1e-7;1e-7]);
    R.virtual_status=V.status;
    assert(startsWith(V.status,'CONDITIONAL_')&&~V.safe_to_commit);
    % Closed-form recurrence tested against direct finite recursion.
    rel=0;
    gammas=[0,0.1,.5,.999999,1-1e-13];
    for gamma=gammas
        for n=[0,1,2,5,20,160,500]
            a0=3.1;d=0.37;a=a0;
            for k=1:n,a=gamma*a+d;end
            if n==0
                fast=a0;
            elseif gamma==0
                fast=d;
            else
                fast=gamma^n*a0+ ...
                    d*(-expm1(n*log(gamma))/(1-gamma));
            end
            rel=max(rel,abs(a-fast)/max(1,abs(a)));
        end
    end
    R.geometric_series_max_relative_error=rel;
    assert(rel<1e-11,'C9_HLC:Recurrence','Closed-form scalar tube mismatch.');
    % Independent conditional physical tube, no estimated-flight data used.
    P=struct('status', ...
       'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN', ...
       'uav_m',.12,'payload_m',.04,'cable_m',.10, ...
       'nominal_w_lower_mps2',9.0);
    B=repmat([0;0;3],1,24);
    N=[-1 1;0 0;0 0];offs=[-1;-1];
    leaf=struct('time_interval_s',[0 1],'map',eye(24), ...
        'q0',[0;0;-1],'guide_cone_padding_m',0, ...
        'uav_normals',N,'payload_normals',N,'cable_normals',N, ...
        'uav_offsets',offs,'payload_offsets',offs,'cable_offsets',offs);
    C=struct('certified',true,'leaves',leaf, ...
        'guide_piece',struct('control_points_m',B,'t0_s',0, ...
                              'duration_s',1), ...
        'radii',struct('uav_m',.15,'payload_m',.1,'cable_m',.05), ...
        'authority',struct('cable_length_m',2, ...
            'gravity_world_mps2',[0;0;-9.80665]), ...
        'obstacle_ids',[11 29],'ground_applicable',true, ...
        'proof_options',struct('robust_tube_enabled',true, ...
             'uncertainty_bounds_m',struct( ...
               'uav_m',.01,'payload_m',.01,'cable_m',.01)));
    A=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
    R.corridor_status=A.status;
    R.min_obstacle_slack_m=A.min_obstacle_residual_m;
    R.min_ground_slack_m=A.min_ground_residual_m;
    assert(A.conditional_all_body_separated&& ...
        A.checked_obstacle_body_pairs==6&&A.checked_leaves==1&& ...
        isfinite(A.min_total_residual_m)&&A.min_total_residual_m>0&& ...
        ~A.safe_to_commit);
    % Mass lower bound is tied to latest estimator, and requires an
    % independent all-horizon estimate-error bound (not the EKF covariance).
    M=TRAJECTORY_C9_HLC_TRUE_MASS_TENSION_GATE(.5,.1, ...
         A.recomputed_nominal_w_lower_mps2,.02,.01);
    R.mass_status=M.status;
    R.conditional_tension_lower_N=M.conditional_tension_lower_N;
    assert(startsWith(M.status,'CONDITIONAL_')&& ...
         M.conditional_tension_lower_N>.01&&~M.safe_to_commit);
    bad=TRAJECTORY_C9_HLC_TRUE_MASS_TENSION_GATE(.5,.5,9,.1,.01);
    R.mass_zero_lower_rejected=startsWith(bad.status,'UNKNOWN_');
    CB=C;CB.leaves.uav_offsets(1)=+.05;
    bad=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(CB,P);
    R.collision_tube_rejected=strcmp(bad.status, ...
        'UNKNOWN_TUBE_OUTSIDE_OBSTACLE_HALFSPACE');
    CG=C;CG.guide_piece.control_points_m(3,:)=.1;
    bad=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(CG,P);
    R.ground_tube_rejected=strcmp(bad.status, ...
        'UNKNOWN_TUBE_INTERSECTS_GROUND');
    PW=P;PW.nominal_w_lower_mps2=10;
    bad=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,PW);
    R.exaggerated_w_bound_rejected=strcmp(bad.status, ...
        'UNKNOWN_HLC_NOMINAL_W_BOUND_EXCEEDS_CORRIDOR_PROOF');
    CL=C;CL.leaves=repmat(C.leaves,1,2);
    CL.leaves(1).time_interval_s=[0,.4];
    CL.leaves(2).time_interval_s=[.6,1];
    bad=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(CL,P);
    R.time_gap_rejected=strcmp(bad.status, ...
        'UNKNOWN_CORRIDOR_LEAF_TIME_COVERAGE');
    assert(all([R.mass_zero_lower_rejected,R.collision_tube_rejected, ...
        R.ground_tube_rejected,R.exaggerated_w_bound_rejected, ...
        R.time_gap_rejected]));
    % Warmed local gate cost only. No flight loop, no model-proof cost.
    tms=zeros(7,1);
    for j=1:numel(tms)
        tt=tic;TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
        tms(j)=1000*toc(tt);
    end
    R.corridor_gate_median_ms=median(tms);
    R.status='PASS_CONDITIONAL_CORRIDOR_MASS_AND_CLOSED_FORM__REAL_HLC_UNPROVEN';
catch ME
    R.status='FAIL_HLC_CORRIDOR_MASS_OR_TUBE';
    R.error_message=[ME.identifier ': ' ME.message];
end
R.total_elapsed_ms=1000*toc(total);
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nerror=%s\n',R.status,R.error_message);
        fprintf(fid,'virtual=%s\ncorridor=%s\nmass=%s\n', ...
            R.virtual_status,R.corridor_status,R.mass_status);
        fprintf(fid,'tube_clearance_obstacle_m=%g\n',R.min_obstacle_slack_m);
        fprintf(fid,'tube_clearance_ground_m=%g\n',R.min_ground_slack_m);
        fprintf(fid,'conditional_tension_lower_N=%g\n',R.conditional_tension_lower_N);
        fprintf(fid,'closed_form_relative_error=%g\n', ...
             R.geometric_series_max_relative_error);
        fprintf(fid,'corridor_gate_median_ms=%g\n',R.corridor_gate_median_ms);
        fprintf(fid,'safe_to_commit=0 actual_HLC_tracking_proven=0\n');
        fclose(fid);
    end
    zip(R.zip_path,{[name '.mat'],[name '.txt']},root);
catch ME
    R.status=['FAIL_SAVE_RESULT_' ME.identifier];
end
end
