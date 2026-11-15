function R=TRAJECTORY_C9_POSTPLANT_PROOF_SNAPSHOT(agent,time,horizon_s)
%TRAJECTORY_C9_POSTPLANT_PROOF_SNAPSHOT Synchronous next-EKF proof origin.
% Call ONLY just after plant.do and before any next EKF.do, i.e. in the
% known run_flight_headless ordering EKF -> HLC -> plant.do -> replanner.
% Current EKF xhat/P and current held input u are from t_k, while the plant
% is already at t_{k+1}. These are exactly the prior estimator/input and
% start plant inputs for the NEXT EKF update at t_{k+1}. NEVER pretend all
% three belong to the same physical timestamp. The next C9 piece therefore
% starts at t_{k+1}. No model-mismatch or floating-roundoff proof here.
R=struct('valid',false,'status','UNKNOWN_POSTPLANT_SOURCE_SNAPSHOT', ...
 'next_epoch_s',NaN,'initial',struct(), ...
 'actual_double_roundoff_proven',false,'safe_to_commit',false, ...
 'HLC_B_horizon_proven',false);
try
 if ~(isobject(agent)&&isprop(agent,'cha') && ...
      strcmp(char(agent.cha),'f') && isobject(time) && ...
      isprop(time,'t')&&isprop(time,'dt') && ...
      isfinite(double(time.t))&&isfinite(double(time.dt))&& ...
      double(time.dt)>0 && ...
      isnumeric(horizon_s)&&isscalar(horizon_s)&& ...
      isfinite(horizon_s)&&horizon_s>0)
     R.status='UNKNOWN_POSTPLANT_TIME_OR_FLIGHT';return
 end
 ekf=agent.estimator.ekf;
 ctrl=agent.controller.hlc_suspended;
 plant=agent.plant;
 if ~(isa(ekf,'EKF')&&isa(ctrl,'HLC_SUSPENDED_LOAD')&& ...
      isa(plant,'MODEL_CLASS'))
    R.status='UNKNOWN_POSTPLANT_SOURCE_CLASSES';return
 end
 xhat=double(ekf.result.state.get());xhat=xhat(:);
 P=double(ekf.result.P);
 X=double(plant.state.get());X=X(:);
 u=double(ctrl.result.input);u=u(:);
 if ~(numel(xhat)==25&&numel(X)==24&&numel(u)==4&& ...
      isequal(size(P),[25 25])&& ...
      all(isfinite([xhat;P(:);X;u])))
     R.status='UNKNOWN_POSTPLANT_STATE_LAYOUT';return
 end
 dt=double(time.dt);
 if ~(isequal(double(ekf.dt),dt)&&isequal(double(ctrl.param.dt),dt))
     R.status='UNKNOWN_POSTPLANT_PERIOD_MISMATCH';return
 end
 R.next_epoch_s=double(time.t)+dt;
 R.initial=struct('ekf_box_25x2',[xhat,xhat], ...
      'cov_box_25x25x2',cat(3,P,P), ...
      'plant_box_24x2',[X,X], ...
      'held_input_box_4x2',[u,u], ...
      'dt',dt,'horizon_s',double(horizon_s));
 R.valid=true;
 R.status='PASS_SNAPSHOT_NEXT_EKF_EPOCH_IDEAL_SOURCE_INPUT__NOT_PROOF';
catch ME
 R.status=['UNKNOWN_POSTPLANT_SNAPSHOT_EXCEPTION_' ME.identifier];
end
end
