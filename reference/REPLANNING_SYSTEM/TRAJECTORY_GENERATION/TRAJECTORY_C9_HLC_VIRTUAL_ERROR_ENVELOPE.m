function R=TRAJECTORY_C9_HLC_VIRTUAL_ERROR_ENVELOPE(F,e0_bound,d_bound,horizon_s)
% TRAJECTORY_C9_HLC_VIRTUAL_ERROR_ENVELOPE
% A conditional Lyapunov bound on ONE HLC hierarchical linear virtual
% subsystem e'=(A-B*F)e + d. F is the LIVE per-Flight HLC row gain.
% e0_bound is a certified 2/6-state initial ERROR NORM bound, d_bound a
% certified *continuous-time* norm bound on ALL unmodeled terms, including
% estimator/mass mismatch for this subsystem, for the entire requested
% horizon. No bound is invented from a measured maximum. This function
% DOES NOT establish that actual nonlinear HLC channels obey this LTI
% equation, nor does it certify actual pre-clamp 4-input or body tracking.
% It is a reusable mathematical LEMMA, not an HLC COMMIT authority.
R=struct('status','UNKNOWN_VIRTUAL_ERROR_DYNAMICS_NOT_PROVEN', ...
    'safe_to_commit',false,'physical_HLC_input_certified',false, ...
    'closed_loop_physical_tracking_certified',false, ...
    'max_alltime_error_2norm',NaN,'terminal_error_2norm',NaN, ...
    'max_virtual_feedback_absolute',NaN, ...
    'decay_rate_per_s',NaN,'disturbance_gain',NaN, ...
    'lyapunov_matrix',[],'linear_error_matrix',[], ...
    'scope','EXACT_LINEAR_VIRTUAL_SUBSYSTEM_IF_EXTERNAL_ASSUMPTIONS_HOLD');
if ~(isnumeric(F)&&isreal(F)&&isvector(F)&&all(isfinite(F(:))))
    return
end
F=double(F(:)).';n=numel(F);
if ~ismember(n,[2 6])|| ...
        ~(isscalar(e0_bound)&&isnumeric(e0_bound)&&isreal(e0_bound)&& ...
          isfinite(e0_bound)&&e0_bound>=0)|| ...
        ~(isscalar(d_bound)&&isnumeric(d_bound)&&isreal(d_bound)&& ...
          isfinite(d_bound)&&d_bound>=0)|| ...
        ~(isscalar(horizon_s)&&isnumeric(horizon_s)&& ...
          isreal(horizon_s)&&isfinite(horizon_s)&&horizon_s>=0)
    return
end
A=diag(ones(n-1,1),1);B=zeros(n,1);B(end)=1;
Acl=A-B*F;
R.linear_error_matrix=Acl;
if any(real(eig(Acl))>=0)
    R.status='UNKNOWN_VIRTUAL_CLOSED_LOOP_NOT_HURWITZ';return
end
% P solves Acl'*P+P*Acl=-I, using current Flight gains.
try
    P=lyap(Acl.',eye(n));
catch
    R.status='UNKNOWN_LYAPUNOV_SOLUTION_NOT_AVAILABLE';return
end
P=0.5*(P+P.');
Q=-(Acl.'*P+P*Acl);Q=0.5*(Q+Q.');
pl=eig(P);ql=eig(Q);
if any(~isfinite([pl;ql]))||min(pl)<=0||min(ql)<=0
    R.status='UNKNOWN_LYAPUNOV_POSITIVITY_UNPROVEN';return
end
% For V=e'*P*e, Young's inequality with Q gives
% dV/dt <= -alpha*V + beta*d_bound^2, with
% alpha=lambda_min(Q)/(2 lambda_max(P)),
% beta=2 ||P||_2^2/lambda_min(Q).
alpha=min(ql)/(2*max(pl));beta=2*norm(P,2)^2/min(ql);
V0=max(pl)*double(e0_bound)^2;
Ve=exp(-alpha*double(horizon_s))*V0+ ...
  (-expm1(-alpha*double(horizon_s)))*beta/alpha*double(d_bound)^2;
Vmax=max(V0,Ve);
R.max_alltime_error_2norm=sqrt(max(0,Vmax)/min(pl));
R.terminal_error_2norm=sqrt(max(0,Ve)/min(pl));
R.max_virtual_feedback_absolute=norm(F,2)*R.max_alltime_error_2norm;
R.decay_rate_per_s=alpha;R.disturbance_gain=beta;
R.lyapunov_matrix=P;
if all(isfinite([R.max_alltime_error_2norm, ...
        R.terminal_error_2norm,R.max_virtual_feedback_absolute]))
    R.status='CONDITIONAL_VIRTUAL_ERROR_ENVELOPE__ACTUAL_HLC_MAP_UNPROVEN';
else
    R.status='UNKNOWN_NONFINITE_VIRTUAL_ERROR_ENVELOPE';
end
end
