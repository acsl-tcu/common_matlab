function R=CHECK_TRAJECTORY_C9_SIM_MODEL_SCOPE()
% Mathematical fixture for explicit ideal simulation-model scope.
% Place in common_matlab/; save all results there independent of pwd.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_SIM_MODEL_SCOPE_RESULT';
R=struct('status','FAIL_NOT_STARTED','error_message','', ...
    'model_tension_status','', 'model_tension_lower_N',NaN, ...
    'zero_mass_rejected',false, 'degenerate_w_rejected',false, ...
    'tube_off_nominal_pass',false,'tube_on_insufficient_margin_rejected',false, ...
    'tube_on_clearance_m',NaN,'corridor_median_ms',NaN, ...
    'actual_HLC_tracking_proven',false,'safe_to_commit',false, ...
    'result_zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');
    % MODEL authoritative mass is 0.5 kg, not a true-mass estimate interval.
    M=TRAJECTORY_C9_HLC_MODEL_MASS_TENSION_GATE(.5,9,.02,.01);
    R.model_tension_status=M.status;
    R.model_tension_lower_N=M.conditional_tension_lower_N;
    assert(strcmp(M.status,'CONDITIONAL_IDEAL_MODEL_POSITIVE_TAUT_TENSION'));
    assert(abs(M.conditional_tension_lower_N-4.49)<1e-12);
    B=TRAJECTORY_C9_HLC_MODEL_MASS_TENSION_GATE(0,9,.02,.01);
    R.zero_mass_rejected=startsWith(B.status,'UNKNOWN_');
    B=TRAJECTORY_C9_HLC_MODEL_MASS_TENSION_GATE(.5,.01,.02,.01);
    R.degenerate_w_rejected=startsWith(B.status,'UNKNOWN_');
    assert(R.zero_mass_rejected&&R.degenerate_w_rejected);
    % A nominally certified fixture with only .05m available clearance;
    % a .12m execution tube violates it, while zero tube fits nominally.
    H=repmat([0;0;3],1,24);
    leaf=struct('time_interval_s',[0 1],'map',eye(24), ...
        'q0',[0;0;-1],'guide_cone_padding_m',0, ...
        'uav_normals',[1;0;0],'payload_normals',[1;0;0], ...
        'cable_normals',[1;0;0], ...
        'uav_offsets',-.05,'payload_offsets',-.05, ...
        'cable_offsets',-.05);
    C=struct('certified',true,'leaves',leaf, ...
        'guide_piece',struct('control_points_m',H, ...
             't0_s',0,'duration_s',1), ...
        'radii',struct('uav_m',.1,'payload_m',.1,'cable_m',.05), ...
        'authority',struct('cable_length_m',2, ...
             'gravity_world_mps2',[0;0;-9.80665]), ...
        'obstacle_ids',11,'ground_applicable',true, ...
        'proof_options',struct('robust_tube_enabled',false));
    P=struct('status', ...
        'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN', ...
        'uav_m',0,'payload_m',0,'cable_m',0, ...
        'nominal_w_lower_mps2',9.0);
    V0=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
    R.tube_off_nominal_pass=V0.conditional_all_body_separated;
    P.uav_m=.12;
    V1=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
    R.tube_on_insufficient_margin_rejected=strcmp(V1.status, ...
       'UNKNOWN_TUBE_OUTSIDE_OBSTACLE_HALFSPACE');
    assert(R.tube_off_nominal_pass&& ...
        R.tube_on_insufficient_margin_rejected);
    % Show a separate valid tube-on fixture with larger nominal clearance.
    C.leaves.uav_offsets=-.50;
    C.leaves.payload_offsets=-.50;
    C.leaves.cable_offsets=-.50;
    V2=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
    assert(V2.conditional_all_body_separated);
    R.tube_on_clearance_m=V2.min_total_residual_m;
    times=zeros(7,1);
    for k=1:7
        t=tic;TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
        times(k)=1000*toc(t);
    end
    R.corridor_median_ms=median(times);
    R.status='PASS_IDEAL_MODEL_MASS_AND_TUBE_SCOPE__NOT_REAL_HLC_PROOF';
catch ME
    R.status='FAIL_IDEAL_MODEL_SCOPE';
    R.error_message=[ME.identifier ': ' ME.message];
end
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nerror=%s\n',R.status,R.error_message);
        fprintf(fid,'model_mass_tension=%s\nmodel_tension_lower_N=%.16g\n', ...
            R.model_tension_status,R.model_tension_lower_N);
        fprintf(fid,'zero_mass_rejected=%d\ndegenerate_w_rejected=%d\n', ...
            R.zero_mass_rejected,R.degenerate_w_rejected);
        fprintf(fid,'tube_off_nominal_pass=%d\ntube_on_rejected=%d\n', ...
            R.tube_off_nominal_pass,R.tube_on_insufficient_margin_rejected);
        fprintf(fid,'tube_on_clearance_m=%.16g\ncorridor_median_ms=%.16g\n', ...
            R.tube_on_clearance_m,R.corridor_median_ms);
        fprintf(fid,'safe_to_commit=0 actual_HLC_tracking_proven=0\n');
        fclose(fid);
    end
    zip(R.result_zip_path,{[name '.mat'],[name '.txt']},root);
catch ME
    R.status=['FAIL_RESULT_SAVE_' ME.identifier];
end
end
