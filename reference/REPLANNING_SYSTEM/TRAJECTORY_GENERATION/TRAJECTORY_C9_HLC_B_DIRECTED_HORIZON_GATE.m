function R=TRAJECTORY_C9_HLC_B_DIRECTED_HORIZON_GATE(agent,piece,request,initial)
%TRAJECTORY_C9_HLC_B_DIRECTED_HORIZON_GATE Full-horizon research proof gate.
% This composes existing source-based interval EKF -> unsaturated 4-input
% HLC -> ZOH plant -> UAV/full cable/payload and independent directed
% degree-27 C9 nominal geometry/force proof. All actual obstacles are from
% request.observed; no future true plant state or fixed payload mass.
% Critical: even a mathematical ideal-model PASS is NOT an assertion that
% MATLAB double/ode15s executions are outward-rounded. Flight COMMIT needs
% separate implementation-roundoff and reference-source provenance.
clock=tic;
R=struct('status','UNKNOWN_C9_FULL_HLC_B_NOT_STARTED', ...
 'valid_ideal_horizon',false,'nominal_directed_proven',false, ...
 'source_4inputs_ideal_proven',false,'B_alltime_ideal_proven',false, ...
 'body_alltime_ideal_proven',false,'actual_double_roundoff_proven',false, ...
 'source_reference_activated',false,'safe_to_commit',false, ...
 'requested_ticks',NaN,'completed_ticks',0, ...
 'maximum_B_radii_m',[NaN NaN NaN], ...
 'nominal_status','NOT_RUN','source_status','NOT_RUN', ...
 'source_failure_tick',NaN,'source_failure_stage','', ...
 'elapsed_ms',NaN);
try
 if exist('infsup','file')~=2 || exist('intval','file')~=2
     R.status='UNKNOWN_C9_FULL_HLC_B_INTLAB_MISSING';return
 end
 fields={'observed','radii','authority','geometry_options', ...
         'force_options','heading_options'};
 if ~(isstruct(request)&&all(isfield(request,fields))&& ...
       isstruct(initial)&&all(isfield(initial,{'ekf_box_25x2', ...
       'cov_box_25x25x2','plant_box_24x2','held_input_box_4x2', ...
       'dt','horizon_s'})))
     R.status='UNKNOWN_C9_FULL_HLC_B_REQUIRED_PROVENANCE';return
 end
 if ~(isstruct(piece)&&isequal(size(piece.control_points_m),[3,28])&& ...
       all(isfinite(piece.control_points_m(:)))&& ...
       isfinite(piece.duration_s)&&piece.duration_s>0)
     R.status='UNKNOWN_C9_FULL_HLC_B_REQUIRE_DEG27';return
 end
 % First rule out a degenerate C7/C6 heading reference, independently.
 H=TRAJECTORY_C9_UAV_C7_YAW_C6_AUDIT( ...
    piece,request.authority,request.heading_options);
 if ~H.certified
     R.status=['UNKNOWN_C9_FULL_HLC_B_HEADING_' char(H.status)];return
 end
 D=TRAJECTORY_C9_27_DIRECTED_ALLBODY_FORCE_INTLAB( ...
    piece,request.observed,request.radii,request.authority, ...
    request.geometry_options,request.force_options);
 R.nominal_status=D.status;
 if ~D.directed_nominal_proven
    R.status=['UNKNOWN_C9_FULL_HLC_B_DIRECTED_NOMINAL_' char(D.status)];return
 end
 R.nominal_directed_proven=true;
 dt=double(initial.dt);t=double(initial.horizon_s);
 if ~(isfinite(dt)&&dt>0&&isfinite(t)&&t>0)
     R.status='UNKNOWN_C9_FULL_HLC_B_TICK_TIME';return
 end
 n=t/dt;
 if ~(isfinite(n)&&abs(n-round(n))<=16*eps(max(1,n)))
     R.status='UNKNOWN_C9_FULL_HLC_B_NONINTEGER_TICK_HORIZON';return
 end
 R.requested_ticks=round(n);
 B=TRAJECTORY_C9_SOURCE_B_HORIZON_INDUCTION( ...
    agent,piece,initial.ekf_box_25x2,initial.cov_box_25x25x2, ...
    initial.plant_box_24x2,initial.held_input_box_4x2, ...
    request.observed,request.radii,dt,t);
 R.source_status=B.status;
 R.completed_ticks=B.completed_ticks;
 R.source_failure_tick=B.failed_tick;
 R.source_failure_stage=B.failed_stage;
 if ~(B.ideal_model_conditional_horizon_reach_proven && ...
      B.conditional_B_fulltime_error_tube_proven && ...
      B.completed_ticks==R.requested_ticks)
    R.status=['UNKNOWN_C9_FULL_HLC_B_CLOSED_LOOP_' char(B.status)];return
 end
 R.source_4inputs_ideal_proven=true;
 R.B_alltime_ideal_proven=true;
 R.body_alltime_ideal_proven=true;
 R.maximum_B_radii_m=max(B.body_error_B_radii_m,[],1);
 R.valid_ideal_horizon=true;
 R.status='CONDITIONAL_IDEAL_MODEL_DIRECTED_HLC_FOUR_INPUT_B_ALLTIME__DOUBLE_IMPLEMENTATION_AND_SWITCH_NOT_PROVED';
catch ME
 R.status=['UNKNOWN_C9_FULL_HLC_B_EXCEPTION_' ME.identifier];
 R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
