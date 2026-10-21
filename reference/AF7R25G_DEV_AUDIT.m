function S=AF7R25G_DEV_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();
L=R.get_last_replan_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R25G DEVELOPMENT STAGED-COMMIT / RUNTIME AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('candidate success                            = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                             = %s\n',char(P.predeadline_candidate_family));
fprintf('exact C7                                     = %d\n',logical(P.predeadline_c7_exact));
fprintf('speed / rejoin / geometry                    = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('geometry pass                                = %d\n',logical(P.predeadline_geometry_pass));
fprintf('\n[STAGED PIPELINE]\n');
fprintf('state / last reason                          = %s / %s\n',char(P.state),char(P.last_reason));
fprintf('stage1 / stage2 / stage3 [ms]                = %.6g / %.6g / %.6g\n', ...
    double(P.last_stage1_ms),double(P.last_stage2_ms),double(P.last_stage3_ms));
fprintf('development per-stage budget                 = %.6g ms\n',double(P.development_stage_budget_ms));
fprintf('final target                                 = %.6g ms\n',double(P.final_target_ms));
fprintf('meets final target S1/S2/S3                  = %d / %d / %d\n', ...
    logical(P.stage1_meets_final_target), ...
    logical(P.stage2_meets_final_target), ...
    logical(P.stage3_meets_final_target));
fprintf('pipeline commit / fail counts                = %.0f / %.0f\n', ...
    double(P.commit_count),double(P.fail_count));

fprintf('\n[ACTIVE REFERENCE]\n');
fprintf('reference-only commit count                  = %.0f\n',double(A.reference_only_commit_count));
fprintf('execution-aware commit count                 = %.0f\n',double(A.execution_aware_commit_count));
fprintf('active repaired safety level                 = %s\n',char(A.active_repaired_safety_level));

if isstruct(L) && ~isempty(fieldnames(L))
    fprintf('\n[LAST REPLAN]\n');
    if isfield(L,'committed'),fprintf('committed                                    = %d\n',logical(L.committed));end
    if isfield(L,'reason'),fprintf('reason                                       = %s\n',char(L.reason));end
    if isfield(L,'commit_time'),fprintf('commit time                                  = %.9g s\n',double(L.commit_time));end
    if isfield(L,'rejoin_time'),fprintf('rejoin time                                  = %.9g s\n',double(L.rejoin_time));end
end

V=AF7R25G_FIX_3D_AXES(3.0);
fprintf('\n[3-D VIEW]\n');
fprintf('minimum span x/y/z                           = 3.0 m each\n');
fprintf('3-D axes adjusted                            = %.0f\n',double(V.axes_changed));
fprintf('====================================================================================================\n');

S=struct('schema','AF7R25G_DEV_AUDIT','pipeline',P,'avoidance',A, ...
    'last_replan',L,'view',V);
end
