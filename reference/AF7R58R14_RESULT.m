function R14 = AF7R58R14_RESULT(S)
% AF7R58R14: read-only, source-bound low-jet C7 seed audit.
% No safety acceptance/commit authority. No guessed physical parameters.
assert(isstruct(S) && isscalar(S), 'R14 requires staged status struct S.');
req={'r58_shadow_ran','r58r14_lowjet_seed_only', ...
    'r58r14_active_reduced_dof','r58r14_full_reduced_dof', ...
    'r58_bnb_full_assignments','r58_bnb_qp_solves', ...
    'r58_bnb_feasible_full','r58_bnb_speed_fail', ...
    'r58_speed_pass','r58_heading_pass','r58_geometry_pass', ...
    'r58_speed_gate_rows','r58_shadow_ms','r58_homotopy_ms', ...
    'r58_build_ms','last_stage1_ms','r58_reference_only', ...
    'r58_commit_attempted'};
for k=1:numel(req)
    assert(isfield(S,req{k}), 'R14 missing required field: %s',req{k});
end
R14=struct();
R14.schema='AF7R58R14_LOWJET_C7_SEED_RESULT_V1';
R14.reference_only=logical(S.r58_reference_only);
R14.commit_attempted=logical(S.r58_commit_attempted);
R14.shadow_ran=logical(S.r58_shadow_ran);
R14.lowjet_seed_only=logical(S.r58r14_lowjet_seed_only);
R14.full_dof=double(S.r58r14_full_reduced_dof);
R14.active_dof=double(S.r58r14_active_reduced_dof);
R14.qp_solve_count=double(S.r58_bnb_qp_solves);
R14.full_assignments=double(S.r58_bnb_full_assignments);
R14.speed_fail_count=double(S.r58_bnb_speed_fail);
R14.speed_pass=logical(S.r58_speed_pass);
R14.heading_pass=logical(S.r58_heading_pass);
R14.geometry_pass=logical(S.r58_geometry_pass);
R14.full_certified=logical(S.r58_bnb_feasible_full) && ...
    R14.speed_pass && R14.heading_pass && R14.geometry_pass;
R14.build_ms=double(S.r58_build_ms);
R14.qp_and_search_ms=double(S.r58_homotopy_ms);
R14.shadow_total_ms=double(S.r58_shadow_ms);
R14.stage1_ms=double(S.last_stage1_ms);
R14.rows=double(S.r58_speed_gate_rows);
assert(isempty(R14.rows) || size(R14.rows,2)==38, ...
    'R14 expects unmodified R13 38-column speed-gate provenance.');
R14.speed_limit_m_s=nan(size(R14.rows,1),1);
R14.finite_speed_witness_m_s=nan(size(R14.rows,1),1);
R14.before_velocity_ub_m_s=nan(size(R14.rows,1),1);
R14.after_velocity_ub_m_s=nan(size(R14.rows,1),1);
R14.quantized_physical_jet_abs_error=nan(size(R14.rows,1),1);
if ~isempty(R14.rows)
    R14.speed_limit_m_s=R14.rows(:,6);
    R14.finite_speed_witness_m_s=R14.rows(:,29);
    R14.before_velocity_ub_m_s=R14.rows(:,36);
    R14.after_velocity_ub_m_s=R14.rows(:,9);
    R14.quantized_physical_jet_abs_error=R14.rows(:,8);
end
if ~R14.shadow_ran
    R14.classification='SHADOW_NOT_RUN';
elseif ~R14.lowjet_seed_only
    R14.classification='FULL_FREE_JET_MODE_NOT_R14';
elseif R14.full_certified
    R14.classification='LOWJET_C7_REFERENCE_ONLY_FULL_CERTIFICATE_PASSED';
elseif R14.full_assignments==0
    R14.classification='LOWJET_QP_NO_FULL_ASSIGNMENT';
elseif R14.speed_fail_count>0
    R14.classification='LOWJET_SPEED_FAIL_NEEDS_FREE_TIME_OR_NEW_SHAPE';
else
    R14.classification='LOWJET_NON_SPEED_HARD_GATE_NOT_CLOSED';
end
% Pass here means only the existing native REFERENCE_ONLY gates closed.
% Never promote to execution-aware/full-system safety or change production.
assert(~R14.commit_attempted,'R14 shadow unexpectedly attempted commit.');
fprintf('\n============================================================\n');
fprintf(' AF7R58R14 LOW-JET C7 SEED RESULT | READ-ONLY\n');
fprintf('============================================================\n');
fprintf(' classification             = %s\n',R14.classification);
fprintf(' shadow / seed / commit     = %d / %d / %d\n', ...
    R14.shadow_ran,R14.lowjet_seed_only,R14.commit_attempted);
fprintf(' active / full QP DOF       = %g / %g\n',R14.active_dof,R14.full_dof);
fprintf(' QP solves / full branches  = %g / %g\n',R14.qp_solve_count,R14.full_assignments);
fprintf(' speed / heading / geometry = %d / %d / %d\n', ...
    R14.speed_pass,R14.heading_pass,R14.geometry_pass);
fprintf(' build / search / shadow / Stage1 ms = %.6f / %.6f / %.6f / %.6f\n', ...
    R14.build_ms,R14.qp_and_search_ms,R14.shadow_total_ms,R14.stage1_ms);
for k=1:numel(R14.speed_limit_m_s)
    fprintf([' branch %d | finite VQ witness = %.9g | speed limit = %.9g | ', ...
        'pre-CP velocity UB = %.9g | stored-CP velocity UB = %.9g | ', ...
        'physical jet quant error = %.9g\n'],k, ...
        R14.finite_speed_witness_m_s(k), R14.speed_limit_m_s(k), ...
        R14.before_velocity_ub_m_s(k),R14.after_velocity_ub_m_s(k), ...
        R14.quantized_physical_jet_abs_error(k));
end
fprintf(' WARNING: this is a non-committable reference-only diagnostic.\n');
reportPath=fullfile(fileparts(mfilename('fullpath')),'AF7R58R14_RESULT.txt');
matPath=fullfile(fileparts(mfilename('fullpath')),'AF7R58R14_CAPTURE.mat');
fid=fopen(reportPath,'w');
assert(fid>=0,'Cannot write R14 result text.');
fprintf(fid,'AF7R58R14 | %s\n',R14.classification);
fprintf(fid,'shadow=%d lowjet=%d committable=0\n',R14.shadow_ran,R14.lowjet_seed_only);
fprintf(fid,'DOF %g / %g | QP %g | full branches %g\n', ...
    R14.active_dof,R14.full_dof,R14.qp_solve_count,R14.full_assignments);
fprintf(fid,'speed/heading/geometry=%d/%d/%d\n', ...
    R14.speed_pass,R14.heading_pass,R14.geometry_pass);
fprintf(fid,'ms build=%.9f QP=%.9f shadow=%.9f Stage1=%.9f\n', ...
    R14.build_ms,R14.qp_and_search_ms,R14.shadow_total_ms,R14.stage1_ms);
for k=1:numel(R14.speed_limit_m_s)
    fprintf(fid,'branch %d limit %.16g witness %.16g rawUB %.16g CPUB %.16g jetErr %.16g\n', ...
        k,R14.speed_limit_m_s(k),R14.finite_speed_witness_m_s(k), ...
        R14.before_velocity_ub_m_s(k),R14.after_velocity_ub_m_s(k), ...
        R14.quantized_physical_jet_abs_error(k));
end
fclose(fid);
save(matPath,'R14','S','-v7.3');
end
