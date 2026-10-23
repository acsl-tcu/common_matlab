function R=AF7R58R39_TRIGGER_ORIGIN_AUDIT(x_est,Y,L,H,dt)
% Read-only structural authority check on ideal predictor, not a trigger.
% No nominal-reference substitution or invented model parameters.
assert(isstruct(x_est)&&isstruct(Y),'Expected estimated state and prediction.');
for f={'pQ','pL','q'}
 name=f{1};assert(isfield(x_est,name)&&isfield(Y,name),'Missing %s',name);
 v=x_est.(name);a=Y.(name);
 assert(isa(v,'double')&&isreal(v)&&isequal(size(v),[3 1]),'Invalid estimator %s',name);
 assert(isa(a,'double')&&isreal(a)&&size(a,1)==3,'Invalid predictor %s',name);
end
validateattributes(L,{'double'},{'real','finite','positive','scalar'});
validateattributes(H,{'double'},{'real','finite','positive','scalar'});
validateattributes(dt,{'double'},{'real','finite','positive','scalar'});
N=size(Y.pQ,2);assert(size(Y.pL,2)==N&&size(Y.q,2)==N,'Prediction lengths differ');
assert(abs((N-1)*dt-H)<=32*eps(max(1,H)),'Incorrect 4 s grid');
res=[norm(Y.pQ(:,1)-x_est.pQ),norm(Y.pL(:,1)-x_est.pL),norm(Y.q(:,1)-x_est.q)];
geometry=max(vecnorm(Y.pQ-Y.pL+L*Y.q));
qnorm=max(abs(vecnorm(Y.q)-1));
R=struct('schema','AF7R58R39_TRIGGER_ORIGIN_AUDIT_V1', ...
 'initial_position_direction_residuals',res, ...
 'max_cable_geometry_residual',geometry,'max_q_unit_residual',qnorm, ...
 'number_of_samples',N,'prediction_horizon',H, ...
 'estimated_state_origin_exact',all(res==0), ...
 'closed_loop_dynamics_certified',false,'continuous_time_trigger_certified',false, ...
 'production_trigger_switched',false);
% Nonzero residual is a FAIL to resolve, not fixed by overwriting sample 1.
end
