function R=CHECK_TRAJECTORY_C9_DIRECT_HLC_AND_AFFINE_CABLE()
%CHECK_TRAJECTORY_C9_DIRECT_HLC_AND_AFFINE_CABLE
% Research fixture, no Flight loop changes. Output all logs in common_matlab.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_DIRECT_HLC_AND_AFFINE_CABLE_RESULT';
R=struct('status','FAIL_NOT_STARTED','error_message','', ...
    'direct_four_input_status','NOT_RUN', ...
    'direct_four_input_headroom',NaN, ...
    'direct_four_input_interval',NaN(4,2), ...
    'direct_determinant_interval',NaN(1,2), ...
    'direct_singular_rejected',false,'direct_torque_violation_rejected',false, ...
    'direct_invalid_interval_rejected',false, ...
    'affine_old_status','NOT_RUN','affine_new_status','NOT_RUN', ...
    'affine_old_slack_m',NaN,'affine_new_slack_m',NaN, ...
    'affine_nonworsening_analytic',false, ...
    'affine_exact_sampled_sweep',false, ...
    'direct_median_ms',NaN,'corridor_old_median_ms',NaN, ...
    'corridor_new_median_ms',NaN, ...
    'safe_to_commit',false,'actual_HLC_tracking_proven',false, ...
    'result_zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');
    lo=[0;-1;-1;-1];hi=[20;1;1;1];
    beta=zeros(3,3,2);
    B=diag([2,3,4]);
    beta(:,:,1)=B-1e-5;
    beta(:,:,2)=B+1e-5;
    rhs=[.4,-.6,.8]';rhs=[rhs-.001,rhs+.001];
    thrust=[4.99,5.01];
    I=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
        lo,hi,beta,rhs,thrust);
    R.direct_four_input_status=I.status;
    R.direct_four_input_headroom=I.minimum_conditional_headroom;
    R.direct_four_input_interval=I.conditional_raw_input_enclosure;
    R.direct_determinant_interval=I.determinant_interval;
    assert(I.conditional_headroom_all_positive, ...
        'C9_DIRECT:FourInput','Expected direct interval to prove.');
    % Check random points only as a diagnostic, NOT a proof replacement.
    rng(1802);
    for k=1:200
        G=beta(:,:,1)+(beta(:,:,2)-beta(:,:,1)).*rand(3);
        y=rhs(:,1)+(rhs(:,2)-rhs(:,1)).*rand(3,1);
        u=G\y;
        assert(all(u>=I.conditional_raw_input_enclosure(2:4,1)-1e-12));
        assert(all(u<=I.conditional_raw_input_enclosure(2:4,2)+1e-12));
    end
    bad=beta;
    bad(3,3,:)=0;bad(3,1:2,:)=0;
    bad(1:2,3,:)=0;
    F=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT(lo,hi,bad,rhs,thrust);
    R.direct_singular_rejected=strcmp(F.status, ...
        'UNKNOWN_HLC_BETA2_INTERVAL_MAY_BE_SINGULAR');
    ybad=rhs;ybad(3,:)=[12,13];
    F=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT(lo,hi,beta,ybad,thrust);
    R.direct_torque_violation_rejected= ...
        strcmp(F.status,'UNKNOWN_HLC_DIRECT_INTERVAL_FOUR_INPUT_HEADROOM');
    bad=beta;bad(1,1,1)=bad(1,1,2)+1;
    F=TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT(lo,hi,bad,rhs,thrust);
    R.direct_invalid_interval_rejected= ...
        strcmp(F.status,'UNKNOWN_DIRECT_INTERVAL_INVALID_BOUNDS');
    assert(R.direct_singular_rejected&& ...
        R.direct_torque_violation_rejected&& ...
        R.direct_invalid_interval_rejected);

    H=repmat([0;0;3],1,24);
    L=2; n=[0;0;1];
    leaf=struct('time_interval_s',[0 1], 'map',eye(24), ...
        'q0',[0;0;-1],'guide_cone_padding_m',0, ...
        'uav_normals',n,'payload_normals',n,'cable_normals',n, ...
        'uav_offsets',4.5,'payload_offsets',2.7,'cable_offsets',2.95);
    C=struct('certified',true,'leaves',leaf, ...
        'guide_piece',struct('control_points_m',H, ...
            't0_s',0,'duration_s',1), ...
        'radii',struct('uav_m',.1,'payload_m',.1,'cable_m',.05), ...
        'authority',struct('cable_length_m',L, ...
            'gravity_world_mps2',[0;0;-9.80665]), ...
        'obstacle_ids',11,'ground_applicable',true, ...
        'proof_options',struct('robust_tube_enabled',false, ...
            'use_affine_cable_execution_tube',false));
    P=struct('status', ...
        'CONDITIONAL_SEPARATE_UAV_CABLE_PAYLOAD_TUBE__EXECUTION_UNPROVEN', ...
        'uav_m',.25,'payload_m',.02,'cable_m',.25, ...
        'nominal_w_lower_mps2',9);
    C0=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
    C.proof_options.use_affine_cable_execution_tube=true;
    C1=TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);
    R.affine_old_status=C0.status;
    R.affine_new_status=C1.status;
    R.affine_old_slack_m=3-2.95-.25;
    R.affine_new_slack_m=C1.min_obstacle_residual_m;
    assert(strcmp(C0.status,'UNKNOWN_TUBE_OUTSIDE_OBSTACLE_HALFSPACE'));
    assert(C1.conditional_all_body_separated);
    assert(abs(C1.min_obstacle_residual_m-.03)<1e-12);
    % Exact affine lemma at every point of a cable (independent test):
    % endpoint minima = minimum of the entire interval lambda in [0,1].
    exact=true;notworse=true;
    for k=1:800
        ep=rand();eu=rand();q=randn(3,1);q=q/norm(q);
        p=randn(3,1);nn=randn(3,1);nn=nn/norm(nn);
        cable_offset=randn();
        valL=nn'*p-ep-cable_offset;
        valU=nn'*(p-L*q)-eu-cable_offset;
        endpoint_min=min(valL,valU);
        lambdas=linspace(0,1,21);
        v=(1-lambdas)*valL+lambdas*valU;
        exact=exact&&abs(min(v)-endpoint_min)<1e-12;
        old=min(nn'*p,nn'*(p-L*q))-max(ep,eu)-cable_offset;
        notworse=notworse && endpoint_min>=old-1e-12;
    end
    R.affine_exact_sampled_sweep=exact;
    R.affine_nonworsening_analytic=notworse;
    assert(exact&&notworse);
    td=zeros(7,1);tc0=zeros(7,1);tc1=zeros(7,1);
    for k=1:7
        t=tic;TRAJECTORY_C9_HLC_FOUR_INPUT_INTERVAL_DIRECT( ...
            lo,hi,beta,rhs,thrust);td(k)=1000*toc(t);
        C.proof_options.use_affine_cable_execution_tube=false;
        t=tic;TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);tc0(k)=1000*toc(t);
        C.proof_options.use_affine_cable_execution_tube=true;
        t=tic;TRAJECTORY_C9_HLC_CORRIDOR_TUBE_GATE(C,P);tc1(k)=1000*toc(t);
    end
    R.direct_median_ms=median(td);
    R.corridor_old_median_ms=median(tc0);
    R.corridor_new_median_ms=median(tc1);
    R.status='PASS_DIRECT_HLC_INTERVAL_AND_AFFINE_CABLE__MODEL_CONDITIONAL_ONLY';
catch ME
    R.status='FAIL_DIRECT_HLC_OR_AFFINE_CABLE';
    R.error_message=[ME.identifier ': ' ME.message];
end
try
    save(fullfile(root,[name '.mat']),'R');
    fid=fopen(fullfile(root,[name '.txt']),'w');
    if fid>=0
        fprintf(fid,'status=%s\nerror=%s\n',R.status,R.error_message);
        fprintf(fid,'input_status=%s\ninput_headroom=%.16g\n', ...
            R.direct_four_input_status,R.direct_four_input_headroom);
        fprintf(fid,'direct_4x2=%s\ndet=%s\n', ...
            mat2str(R.direct_four_input_interval,14), ...
            mat2str(R.direct_determinant_interval,14));
        fprintf(fid,'singular_rejected=%d\ntorque_violation_rejected=%d\n', ...
            R.direct_singular_rejected,R.direct_torque_violation_rejected);
        fprintf(fid,'invalid_interval_rejected=%d\n',R.direct_invalid_interval_rejected);
        fprintf(fid,'cable_legacy=%s\ncable_affine=%s\n', ...
            R.affine_old_status,R.affine_new_status);
        fprintf(fid,'old_cable_slack=%.16g\naffine_slack=%.16g\n', ...
            R.affine_old_slack_m,R.affine_new_slack_m);
        fprintf(fid,'affine_exact=%d\nnonworsening=%d\n', ...
            R.affine_exact_sampled_sweep,R.affine_nonworsening_analytic);
        fprintf(fid,'ms_direct=%.8g\nms_cable_old=%.8g\nms_cable_new=%.8g\n', ...
            R.direct_median_ms,R.corridor_old_median_ms, ...
            R.corridor_new_median_ms);
        fprintf(fid,'safe_to_commit=0 actual_HLC_tracking_proven=0\n');
        fclose(fid);
    end
    zip(R.result_zip_path,{[name '.mat'],[name '.txt']},root);
catch ME
    R.status=['FAIL_RESULT_SAVE_' ME.identifier];
end
end
