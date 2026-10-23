function A = AF7R58R47_TOWING_LOG_AUDIT(log_source)
% AF7R58R47_TOWING_LOG_AUDIT  Read-only sampled towing-log analysis.
% Required struct fields (columns are chronological, in world frame):
%   t_s (Nx1), pQ_m (Nx3), pL_m (Nx3), q_cable (Nx3), L_m (Nx1 or scalar)
%   mL_EKF_kg (Nx1), mL_HLC_kg (Nx1)
% The two mass channels are retained separately, never cross-written.
% Optional explicit logged fields: pQ_ref_m, pL_ref_m, qdot_logged_s,
%   measured_tension_N, thrust_command_N, torque_command_Nm.
% THIS IS NOT a plant propagation and cannot certify between-sample safety,
% ellipsoid clearance, force/tension feasibility, or HLC stability.

if ischar(log_source) || (isstring(log_source) && isscalar(log_source))
    file=char(log_source);
    assert(isfile(file),'[R47] Logged-data .mat not found: %s',file);
    log_source=load(file);
end
assert(isstruct(log_source) && isscalar(log_source), ...
    '[R47] Input must be an explicitly mapped scalar log struct.');
required={'t_s','pQ_m','pL_m','q_cable','L_m', ...
          'mL_EKF_kg','mL_HLC_kg'};
for i=1:numel(required)
    assert(isfield(log_source,required{i}), ...
        '[R47] Missing explicitly required live logged field: %s',required{i});
end
L=log_source;
t=double(L.t_s(:)); n=numel(t);
assert(n>=2 && all(isfinite(t)) && all(diff(t)>0), ...
    '[R47] Log timestamps must be finite, strictly increasing and have N>=2.');
pQ=vec3(L.pQ_m,n,'pQ_m');
pL=vec3(L.pL_m,n,'pL_m');
q=vec3(L.q_cable,n,'q_cable');
len=double(L.L_m(:));
if isscalar(len),len=repmat(len,n,1);end
assert(numel(len)==n && all(isfinite(len)) && all(len>0), ...
    '[R47] Cable lengths are not valid logged values.');
mEKF=column(L.mL_EKF_kg,n,'mL_EKF_kg');
mHLC=column(L.mL_HLC_kg,n,'mL_HLC_kg');
assert(all(mEKF>0) && all(mHLC>0), ...
    '[R47] Logged masses must be positive; no fallback mass permitted.');
qd=sqrt(sum(q.^2,2));
assert(all(qd>0),'[R47] Zero cable-direction vector found.');
d=pL-pQ; dn=sqrt(sum(d.^2,2));
qhat=q./qd;
axial=sum(d.*qhat,2);
perp=sqrt(sum((d-axial.*qhat).^2,2));
A=struct();
A.schema='AF7R58R47_TOWING_SAMPLED_LOG_AUDIT_V1';
A.samples=n;
A.first_t_s=t(1);A.last_t_s=t(end);
A.max_cable_length_abs_residual_m=max(abs(dn-len));
A.max_cable_collinearity_residual_m=max(perp);
A.max_cable_unit_norm_residual=max(abs(qd-1));
A.min_cable_signed_projection_m=min(axial);
A.max_sampled_UAV_secant_speed_mps=max(sqrt(sum((diff(pQ)./diff(t)).^2,2)));
A.max_sampled_load_secant_speed_mps=max(sqrt(sum((diff(pL)./diff(t)).^2,2)));
A.latest_EKF_payload_mass_kg=mEKF(end);
A.latest_HLC_logged_mass_kg=mHLC(end);
A.max_logged_mass_channel_difference_kg=max(abs(mEKF-mHLC));
A.mass_authority_overridden=false;
A.samples_are_not_continuous_time_bounds=true;
A.execution_or_obstacle_certificate=false;
A.production_modified=false;
if isfield(L,'qdot_logged_s')
    qdot=vec3(L.qdot_logged_s,n,'qdot_logged_s');
    A.max_logged_qdot_per_s=max(sqrt(sum(qdot.^2,2)));
else
    A.max_logged_qdot_per_s=NaN;
end
if isfield(L,'measured_tension_N')
    tn=column(L.measured_tension_N,n,'measured_tension_N');
    A.min_measured_tension_N=min(tn);
    A.max_measured_tension_N=max(tn);
else
    A.min_measured_tension_N=NaN;
    A.max_measured_tension_N=NaN;
end
if isfield(L,'pQ_ref_m')
    er=pQ-vec3(L.pQ_ref_m,n,'pQ_ref_m');
    A.max_sampled_UAV_position_error_m=max(sqrt(sum(er.^2,2)));
else
    A.max_sampled_UAV_position_error_m=NaN;
end
if isfield(L,'pL_ref_m')
    er=pL-vec3(L.pL_ref_m,n,'pL_ref_m');
    A.max_sampled_load_position_error_m=max(sqrt(sum(er.^2,2)));
else
    A.max_sampled_load_position_error_m=NaN;
end
if isfield(L,'thrust_command_N')
    A.max_logged_thrust_command_N=max(abs(column(L.thrust_command_N,n,'thrust_command_N')));
else
    A.max_logged_thrust_command_N=NaN;
end
if isfield(L,'torque_command_Nm')
    m=vec3(L.torque_command_Nm,n,'torque_command_Nm');
    A.max_logged_abs_torque_components_Nm=max(abs(m),[],1);
else
    A.max_logged_abs_torque_components_Nm=[NaN NaN NaN];
end
A.status='SAMPLED_TOWING_DIAGNOSTIC_ONLY__NO_CONTINUOUS_CERTIFICATE';
fprintf('\n[R47/TOWING] samples=%d | [%.9f,%.9f] s\n',n,t(1),t(end));
fprintf('[R47/TOWING] cable length residual max %.12g m; lateral %.12g m\n', ...
    A.max_cable_length_abs_residual_m,A.max_cable_collinearity_residual_m);
fprintf('[R47/TOWING] UAV/load sampled secant speed maxima %.9f / %.9f m/s\n', ...
    A.max_sampled_UAV_secant_speed_mps,A.max_sampled_load_secant_speed_mps);
fprintf('[R47/TOWING] latest EKF/HLC mass %.9f / %.9f kg (distinct channels)\n', ...
    A.latest_EKF_payload_mass_kg,A.latest_HLC_logged_mass_kg);
fprintf('[R47/TOWING] %s\n',A.status);
end

function x=vec3(v,n,name)
x=double(v);
assert(ismatrix(x) && size(x,1)==n && size(x,2)==3 && ...
    all(isfinite(x(:))), '[R47] Field %s must be finite Nx3.',name);
end
function x=column(v,n,name)
x=double(v(:));
assert(numel(x)==n && all(isfinite(x)), ...
    '[R47] Field %s must be finite Nx1.',name);
end
