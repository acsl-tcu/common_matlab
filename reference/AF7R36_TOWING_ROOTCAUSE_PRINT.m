function S = AF7R36_TOWING_ROOTCAUSE_PRINT(R)
% AF7R36_TOWING_ROOTCAUSE_PRINT
% Print the diagnostic-only fields added by AF7R36_TOWING_DIAG_INSTALL.
%
% R may be the native RAW result directly or a proposal/result containing
% field native_raw.

if isfield(R,'native_raw') && isstruct(R.native_raw)
    X=R.native_raw;
else
    X=R;
end

req={ ...
    'candidate_family','t1','finite_sensing_limit_m_s', ...
    'speed_payload','speed_accel','speed_jerk','speed_sigma','speed_qdot','speed_uav', ...
    'max_control_norm_m','primary_candidate_count','primary_speed_pass_count', ...
    'primary_rejoin_pass_count','primary_geometry_test_count', ...
    'fallback_candidate_count','fallback_speed_pass_count', ...
    'primary_min_stageA_uav_speed_UB_m_s', ...
    'primary_min_stageA_specific_force_LB_m_s2', ...
    'primary_min_stageA_correction_speed_UB_m_s', ...
    'primary_min_stageA_correction_accel_UB_m_s2', ...
    'primary_min_stageA_correction_jerk_UB_m_s3', ...
    'primary_min_envelope_m','primary_stageA_certified_count', ...
    'primary_stageB_certified_count','primary_stageB_finite_witness_count', ...
    'primary_stageB_repr_fail_count','towing_diag_revision'};
missing=req(~cellfun(@(f)isfield(X,f),req));
assert(isempty(missing), ...
    '[TOWDIAG] Missing diagnostic fields: %s. Rebuild/install diagnostic patch first.', ...
    strjoin(missing,', '));

lim=double(X.finite_sensing_limit_m_s);
pvq=double(X.primary_min_stageA_uav_speed_UB_m_s);
selVQ=double(X.speed_uav);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING ROOT-CAUSE RESULT | %s\n',char(X.towing_diag_revision));
fprintf('====================================================================================================\n');
fprintf('candidate family                       = %s\n',char(X.candidate_family));
fprintf('t1                                     = %.12g s\n',double(X.t1));
fprintf('finite-sensing UAV-speed limit         = %.12g m/s\n',lim);
fprintf('\n[SELECTED CERTIFIED CANDIDATE]\n');
fprintf('payload speed UB                       = %.12g m/s\n',double(X.speed_payload));
fprintf('payload accel UB                       = %.12g m/s^2\n',double(X.speed_accel));
fprintf('payload jerk UB                        = %.12g m/s^3\n',double(X.speed_jerk));
fprintf('specific-force norm LB                 = %.12g m/s^2\n',double(X.speed_sigma));
fprintf('cable qdot UB                          = %.12g rad/s\n',double(X.speed_qdot));
fprintf('UAV speed UB                           = %.12g m/s\n',selVQ);
fprintf('speed utilization VQ/limit             = %.9f\n',selVQ/lim);
fprintf('Bezier correction envelope             = %.12g m\n',double(X.max_control_norm_m));

fprintf('\n[PRIMARY MINIMUM-DEVIATION FAMILY]\n');
fprintf('candidate / speed / rejoin / geom      = %d / %d / %d / %d\n', ...
    round(X.primary_candidate_count),round(X.primary_speed_pass_count), ...
    round(X.primary_rejoin_pass_count),round(X.primary_geometry_test_count));
fprintf('minimum Stage-A UAV-speed UB           = %.12g m/s\n',pvq);
fprintf('minimum Stage-A VQ/limit               = %.9f\n',pvq/lim);
fprintf('associated specific-force LB           = %.12g m/s^2\n', ...
    double(X.primary_min_stageA_specific_force_LB_m_s2));
fprintf('associated correction |d_dot| UB       = %.12g m/s\n', ...
    double(X.primary_min_stageA_correction_speed_UB_m_s));
fprintf('associated correction |d_ddot| UB      = %.12g m/s^2\n', ...
    double(X.primary_min_stageA_correction_accel_UB_m_s2));
fprintf('associated correction |d_jerk| UB      = %.12g m/s^3\n', ...
    double(X.primary_min_stageA_correction_jerk_UB_m_s3));
fprintf('minimum primary correction envelope    = %.12g m\n',double(X.primary_min_envelope_m));
fprintf('Stage-A certified count                = %d\n',round(X.primary_stageA_certified_count));
fprintf('Stage-B certified count                = %d\n',round(X.primary_stageB_certified_count));
fprintf('Stage-B finite speed-witness rejects   = %d\n',round(X.primary_stageB_finite_witness_count));
fprintf('Stage-B representation/inconclusive    = %d\n',round(X.primary_stageB_repr_fail_count));

fprintf('\n[FALLBACK]\n');
fprintf('candidate / speed pass                 = %d / %d\n', ...
    round(X.fallback_candidate_count),round(X.fallback_speed_pass_count));

% Diagnostic interpretation. This never changes acceptance.
finiteWitness=double(X.primary_stageB_finite_witness_count)>0;
reprFail=double(X.primary_stageB_repr_fail_count)>0;
if finiteWitness
    cause='PRIMARY_HAS_FINITE_POINT_SPEED_VIOLATION_WITNESS';
elseif reprFail
    cause='PRIMARY_SPEED_CERT_INCONCLUSIVE_REPRESENTATION_LIMIT';
elseif double(X.primary_speed_pass_count)==0 && isfinite(pvq) && pvq>=lim
    cause='PRIMARY_REJECTED_BY_CERTIFIED_SPEED_UPPER_BOUND';
elseif double(X.primary_speed_pass_count)==0
    cause='PRIMARY_SPEED_REJECTION_NEEDS_NEXT_INTERNAL_SPLIT_AUDIT';
else
    cause='PRIMARY_SPEED_GATE_PASSES_AT_LEAST_ONE_CANDIDATE';
end
fprintf('\nDIAGNOSTIC CLASSIFICATION               = %s\n',cause);
fprintf('NOTE: finite witness is rejection-only evidence; it is never sampled acceptance.\n');
fprintf('====================================================================================================\n');

S=struct();
S.classification=cause;
S.limit_m_s=lim;
S.primary_min_stageA_vq_m_s=pvq;
S.primary_vq_ratio=pvq/lim;
S.selected_vq_ratio=selVQ/lim;
S.primary_min_envelope_m=double(X.primary_min_envelope_m);
S.selected_envelope_m=double(X.max_control_norm_m);
S.primary_stageB_finite_witness_count=double(X.primary_stageB_finite_witness_count);
S.primary_stageB_repr_fail_count=double(X.primary_stageB_repr_fail_count);
end
