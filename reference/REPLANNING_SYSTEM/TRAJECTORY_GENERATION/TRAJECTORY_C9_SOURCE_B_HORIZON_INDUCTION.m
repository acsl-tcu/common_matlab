function R=TRAJECTORY_C9_SOURCE_B_HORIZON_INDUCTION(agent,piece, ...
    prior_ekf_25x2,prior_cov_25x25x2,plant_24x2,prior_held_4x2, ...
    observed,radii,dt,requested_horizon_s)
%TRAJECTORY_C9_SOURCE_B_HORIZON_INDUCTION  Experimental source-based B.
% Attempts EXACTLY the original sample order over the requested horizon:
%   ideal MOTIVE at xplant_k -> EKF (previous held input, gain, mass)
%   -> SOURCE HLC 4 preclamp inputs -> ZOH plant interval flow
%   -> whole UAV, straight cable and payload continuous-time separation.
% Uses actual source expressions + INTLAB. No guessed noise or covariance
% as deterministic state error. The current EKF mass is the ideal-model
% truth each tick. Any failed inclusion returns UNKNOWN with the exact tick.
%
% This is OFFLINE RESEARCH ONLY: may require hundreds of interval solves,
% is not the <=5 ms online candidate search, and does NOT certify the
% finite precision error of actual non-interval MATLAB/EKF/HLC/MEX code.
% No change to HLC, EKF, plant, runner or reference allocation.
clock=tic;
% Keep the provenance boxes available even when an early source check fails.
xhat=prior_ekf_25x2;P=prior_cov_25x25x2;
X=plant_24x2;u=prior_held_4x2;
R=struct('status','UNKNOWN_B_HORIZON_NOT_STARTED', ...
 'ideal_model_conditional_horizon_reach_proven',false, ...
 'actual_matlab_double_roundoff_proven',false, ...
 'current_Flight_reference_switch_authorized',false, ...
 'safe_to_commit',false,'completed_ticks',0,'requested_ticks',0, ...
 'failed_tick',NaN,'failed_stage','', ...
 'max_tick_ms',NaN,'median_tick_ms',NaN, ...
 'tick_ms',zeros(1,0),'mass_estimate_enclosure_kg',zeros(0,2), ...
 'body_error_B_radii_m',NaN(0,3), ...
 'conditional_B_fulltime_error_tube_proven',false, ...
 'u_preclamp_bounds_by_tick',NaN(4,2,0), ...
 'last_plant_24x2',plant_24x2,'last_ekf_25x2',prior_ekf_25x2, ...
 'last_cov_25x25x2',prior_cov_25x25x2, ...
 'last_held_4x2',prior_held_4x2,'elapsed_ms',NaN, ...
 'scope','CONDITIONAL_IDEAL_MODEL_FULL_HORIZON_SOURCE_INDUCTION_ONLY');
try
 if exist('infsup','file')~=2 || exist('intval','file')~=2
     R.status='UNKNOWN_B_HORIZON_INTLAB_NOT_INSTALLED';return
 end
 if ~(isstruct(piece)&&isfield(piece,'t0_s') && ...
         isfield(piece,'duration_s') && ...
         isa(agent,'DRONE') && ...
         strcmp(char(agent.cha),'f') && ...
         isnumeric(dt)&&isscalar(dt)&&isfinite(dt)&&dt>0 && ...
         isnumeric(requested_horizon_s)&&isscalar(requested_horizon_s)&& ...
         isfinite(requested_horizon_s)&&requested_horizon_s>0)
     R.status='UNKNOWN_B_HORIZON_FLIGHT_OR_AUTHORITY';return
 end
 n=requested_horizon_s/dt;
 if ~(abs(n-round(n))<=16*eps(max(1,n)) && n>=1)
     R.status='UNKNOWN_B_HORIZON_NOT_INTEGER_NUMBER_OF_REAL_TICKS';return
 end
 n=round(n);R.requested_ticks=n;
 if ~(piece.duration_s>=n*dt && ...
         all(isfinite([piece.t0_s,piece.duration_s])))
     R.status='UNKNOWN_B_HORIZON_PIECE_DOES_NOT_COVER_REQUEST';return
 end
 if ~(valid_box(prior_ekf_25x2,25) && ...
         valid_box(prior_held_4x2,4) && ...
         valid_box(plant_24x2,24) && ...
         isequal(size(prior_cov_25x25x2),[25 25 2]) && ...
         all(isfinite(prior_cov_25x25x2(:))) && ...
         all(prior_cov_25x25x2(:,:,1)<=prior_cov_25x25x2(:,:,2),'all'))
     R.status='UNKNOWN_B_HORIZON_INVALID_INITIAL_CERTIFIED_BOX';return
 end
 if ~isprop(agent,'reference')
     R.status='UNKNOWN_B_HORIZON_REFERENCE_SOURCE';return
 end
 % The input boxes are mathematical preconditions and MUST enclose the
 % current actual EKF, covariance, plant and held input. This routine
 % does not infer a deterministic error bound from EKF covariance.
 lastYaw=[];
 R.tick_ms=NaN(1,n);
 R.mass_estimate_enclosure_kg=NaN(n,2);
 R.u_preclamp_bounds_by_tick=NaN(4,2,n);
 R.body_error_B_radii_m=NaN(n,3);
 for k=1:n
     current_t=double(piece.t0_s)+(k-1)*dt;
     H=TRAJECTORY_C9_REFERENCE_28_FROM_PIECE( ...
         piece,current_t,double(agent.parameter.get('cableL')),lastYaw);
     if ~H.valid
         R.status=['UNKNOWN_B_HORIZON_REFERENCE_' H.status];
         R.failed_stage='REFERENCE';R.failed_tick=k;break
     end
     lastYaw=H.yaw_derivatives_0_to_6(1);
     % At each sample the actual HLC receives the 28-component (post-SLOAD)
     % sample reference. This does not assume reference is constant between
     % samples: only the control input is held by the source plant.
     sample=double(H.xd_at_HLC_28(:));
     box=[sample,sample];
     M=TRAJECTORY_C9_SOURCE_IDEAL_MOTIVE_OBSERVATION_INTLAB(agent,X);
     if ~M.valid
         R.status=['UNKNOWN_B_HORIZON_MOTIVE_' M.status];
         R.failed_stage='MOTIVE';R.failed_tick=k;break
     end
     C=TRAJECTORY_C9_SOURCE_CLOSED_TICK_B_INTLAB( ...
         agent,xhat,P,M.ideal_measurement_12x2,u,X,box,observed,radii,dt);
     R.tick_ms(k)=C.elapsed_ms;
     if ~C.closed_one_tick_conditionally_proven
         R.status=['UNKNOWN_B_HORIZON_TICK_' C.status];
         R.failed_stage=C.stage;R.failed_tick=k;break
     end
     tube=TRAJECTORY_C9_SOURCE_TICK_ERROR_B_INTERVAL( ...
         piece,current_t,current_t+dt,C.plant_all_24x2, ...
         [0;0;-double(agent.parameter.get('gravity'))], ...
         double(agent.parameter.get('cableL')));
     if ~tube.valid
         R.status=['UNKNOWN_B_HORIZON_TUBE_' tube.status];
         R.failed_stage='SOURCE_ALLTIME_ERROR_TUBE';R.failed_tick=k;break
     end
     R.body_error_B_radii_m(k,:)=tube.error_radii_m;
     xhat=C.estimated_next_25x2;
     P=C.cov_next_25x25x2;
     X=C.plant_next_24x2;
     u=C.unsaturated_four_inputs_4x2;
     R.completed_ticks=k;
     R.mass_estimate_enclosure_kg(k,:)=xhat(25,:);
     R.u_preclamp_bounds_by_tick(:,:,k)=u;
 end
 if R.completed_ticks==n
 R.ideal_model_conditional_horizon_reach_proven=true;
 R.conditional_B_fulltime_error_tube_proven=true;
 R.status='CONDITIONAL_B_FULL_REQUESTED_HORIZON_IDEAL_SOURCE_MODEL__DOUBLE_ARITHMETIC_AND_FLIGHT_SWITCH_NOT_PROVED';
 end
catch ME
 R.status=['UNKNOWN_B_HORIZON_EXCEPTION_' ME.identifier];
 R.error_message=ME.message;
end
R.last_plant_24x2=X;
R.last_ekf_25x2=xhat;
R.last_cov_25x25x2=P;
R.last_held_4x2=u;
if any(isfinite(R.tick_ms))
 R.max_tick_ms=max(R.tick_ms(isfinite(R.tick_ms)));
 R.median_tick_ms=median(R.tick_ms(isfinite(R.tick_ms)));
end
R.elapsed_ms=1000*toc(clock);
end
function tf=valid_box(X,n)
tf=isnumeric(X)&&isreal(X)&&isequal(size(X),[n 2])&& ...
    all(isfinite(X(:)))&&all(X(:,1)<=X(:,2));
end
