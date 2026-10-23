function R=AF7R58R43_TRIGGER_ORIGIN_CONTRACT(P, predictor)
% Source-bound estimator origin; closed-loop callback is REQUIRED for prediction.
% This function does not certify dynamics, tracking, or continuous collision avoidance.
assert(isa(P,'REPLANNING_SYSTEM'),'Live REPLANNING_SYSTEM object required.');
S=P.self.estimator.result.state;
assert(isprop(S,'p')&&isprop(S,'pT'), ...
    'Estimated UAV p and cable pT unavailable.');
pQ=double(S.p(:));q=double(S.pT(:));
L=double(P.L_cable);
assert(numel(pQ)==3&&numel(q)==3&&all(isfinite([pQ;q]))&& ...
    isfinite(L)&&L>0,'Bad estimator geometry.');
assert(norm(q)>0,'Cable direction norm is zero.');
q=q/norm(q);
if isprop(S,'pL')&&~isempty(S.pL)
 pL=double(S.pL(:));
else
 pL=pQ+L*q;
end
residual=norm(pL-pQ-L*q);
R=struct('schema','R43_ESTIMATED_ORIGIN', ...
 'estimated_pQ',pQ,'estimated_pL',pL,'estimated_q',q, ...
 'geometry_residual_m',residual, ...
 'nominal_H_s',double(P.horizon_s), ...
 'sample_dt_s',double(P.prediction_dt), ...
 'estimated_current_state_used',true, ...
 'future_prediction_verified',false, ...
 'production_trigger_switched',false);
% Missing actual HLC and plant RHS is a known blocker; never synthesize or
% accept a reference packet as though it originated at the estimate.
if nargin<2 || isempty(predictor)
 R.status='MISSING_CLOSED_LOOP_DYNAMICS_PREDICTOR';
 return;
end
assert(isa(predictor,'function_handle'), 'predictor must be a function handle.');
Y=predictor(P); % callback must use estimator, live HLC and plant RHS
assert(isstruct(Y)&&all(isfield(Y,{'t','pQ','pL','q'})), ...
    'Prediction packet must contain t,pQ,pL,q.');
assert(isfield(Y,'mass_used_kg'), ...
    'Predictor must identify the current estimated payload mass it used.');
R.latest_estimated_mass_kg=double(P.loadmass);
R.predictor_mass_used_kg=double(Y.mass_used_kg);
assert(isequal(R.latest_estimated_mass_kg,R.predictor_mass_used_kg), ...
    'Predictor mass does not equal latest estimator-authoritative mass.');
Y.t=double(Y.t(:));Y.pQ=double(Y.pQ);Y.pL=double(Y.pL);Y.q=double(Y.q);
assert(size(Y.pQ,1)==3&&isequal(size(Y.pQ),size(Y.pL))&& ...
    isequal(size(Y.pL),size(Y.q))&&size(Y.pQ,2)==numel(Y.t), ...
    'Prediction packet dimensions incorrect.');
N=round(double(P.horizon_s)/double(P.prediction_dt))+1;
assert(numel(Y.t)==N && ...
    abs((Y.t(end)-Y.t(1))-double(P.horizon_s))<=...
    1e-8*max(1,double(P.horizon_s)), ...
    'Prediction horizon is not the externally configured H.');
assert(all(isfinite([Y.pQ(:);Y.pL(:);Y.q(:);Y.t(:)])) && ...
  all(diff(Y.t)>0),'Prediction contains invalid times/state.');
% The packet start is compared with the *actual* estimator state.
R.start_error_uav_m=norm(Y.pQ(:,1)-pQ);
R.start_error_payload_m=norm(Y.pL(:,1)-pL);
R.start_error_q=norm(Y.q(:,1)-q);
R.max_cable_residual_m=max(vecnorm(Y.pL-Y.pQ-L*Y.q,2,1));
R.actual_initial_time_s=Y.t(1);
R.prediction_horizon_s=Y.t(end)-Y.t(1);
R.max_q_norm_error=max(abs(vecnorm(Y.q,2,1)-1));
R.status='INITIAL_GEOMETRY_DIAGNOSTIC_ONLY_DYNAMICS_UNCERTIFIED';
% Geometric checks cannot prove that Y follows HLC closed-loop ODE,
% or that whole-body obstacles are avoided in continuous time.
end
