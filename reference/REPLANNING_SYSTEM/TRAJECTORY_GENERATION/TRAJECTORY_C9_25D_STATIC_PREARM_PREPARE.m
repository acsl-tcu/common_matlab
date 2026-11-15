function R=TRAJECTORY_C9_25D_STATIC_PREARM_PREPARE()
%TRAJECTORY_C9_25D_STATIC_PREARM_PREPARE Pure Flight-independent software prep.
% Use once after clear classes / rehash and BEFORE any Flight/Arming.
% Warms only 27C9 basis/degree-25 acceleration quadrature caches and
% the actual 25D QP representation functions. NO Flight parameters,
% obstacle IDs, obstacle counts/positions, sensor, EKF or plant states.
% A genuine source/MEX Flight bind is still required separately.
R=struct('status','UNKNOWN_25D_STATIC_PREARM_NOT_RUN', ...
 'ready',false,'source_flight_parameters_read',false, ...
 'real_obstacle_positions_accessed',false, ...
 'static_reuse_valid',false,'group_corridor_ready',false, ...
 'group_corridor_status','NOT_RUN', ...
 'group_corridor_warm_ms',NaN,'block_metric_ms',NaN, ...
 'unblocked_metric_ms',NaN,'source_error',NaN, ...
 'elapsed_ms',NaN,'safe_to_commit',false);
clk=tic;
try
    T=1;A=zeros(3,26,25);
    for j=1:8
        bj=zeros(1,28);bj(j+10)=1;
        da=27*26*diff(bj,2,2);
        for axis=1:3
            col=3*(j-1)+axis;
            A(axis,:,col)=da;
        end
    end
    % Generic algebraic fixture: nonzero duration column outside the
    % 24-dimensional free-control span. NOT a Flight candidate.
    A(1,:,25)=sin((1:26)*0.13);
    A(2,:,25)=cos((1:26)*0.17);
    A(3,:,25)=sin((1:26)*0.19);
    [B0,Q0]=deal(TRAJECTORY_C9_25D_BLOCK_ACCELERATION_METRIC(A,T), ...
                 TRAJECTORY_C9_25D_EXACT_ACCELERATION_QR(A,T));
    if ~(B0.valid && Q0.valid)
        R.status=['UNKNOWN_25D_PREARM_METRIC_' B0.status '_' Q0.status];
        R.elapsed_ms=1000*toc(clk);return
    end
    R.source_error=norm(B0.H-Q0.H,'fro')/max(1,norm(Q0.H,'fro'));
    if R.source_error>2e-10
        R.status='UNKNOWN_25D_PREARM_BLOCK_METRIC_PARITY';
        R.elapsed_ms=1000*toc(clk);return
    end
    B1=TRAJECTORY_C9_25D_BLOCK_ACCELERATION_METRIC(A,T);
    Q1=TRAJECTORY_C9_25D_EXACT_ACCELERATION_QR(A,T);
    if ~(B1.valid&&Q1.valid&&isequal(B1.H,B0.H)&& ...
            isequal(Q1.H,Q0.H))
        R.status='UNKNOWN_25D_PREARM_CACHE_REUSE';
        R.elapsed_ms=1000*toc(clk);return
    end
    % Warm only general shape-support/group-QP SOFTWARE paths using a
    % synthetic two-obstacle fixture. Actual Flight geometry must still be
    % obtained from the sensor when a real replan is triggered.
    C=TRAJECTORY_C9_25D_GROUP_CORRIDOR_PREFLIGHT();
    % Optional research corridor. Its warmup failure must NOT block the
    % established nominal/trigger Flight preflight when this mode is OFF.
    R.group_corridor_ready=C.ready;
    R.group_corridor_status=C.status;
    R.group_corridor_warm_ms=C.elapsed_ms;
    R.block_metric_ms=B1.elapsed_ms;
    R.unblocked_metric_ms=Q1.elapsed_ms;
    R.static_reuse_valid=true;
    R.ready=true;
    R.status='PASS_25D_STATIC_PREARM_SOFTWARE_CACHES__NO_FLIGHT_PROOF';
catch ME
    R.status=['UNKNOWN_25D_PREARM_EXCEPTION_' ME.identifier];
    R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clk);
end
