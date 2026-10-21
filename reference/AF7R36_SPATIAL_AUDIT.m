function S=AF7R36_SPATIAL_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 CURVED SPATIAL C7 + DELAY-PRESERVING RETIMING + PATH-TANGENT YAW AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('runner/display modified                       = 0\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('family-7 unsafe-span affine spatial model     = 0\n');

fprintf('\n[CURVED SPATIAL C7]\n');
fprintf('spatial QP solutions                          = %.0f\n',double(P.r36_spatial_qp_solutions));
fprintf('spatial synthesis-row rejects                 = %.0f\n',double(P.r36_spatial_row_rejects));
fprintf('unsafe-span midpoint progress                 = %.12g s\n',double(P.r36_mid_progress_s));
fprintf('timing pre-trials                             = %.0f\n',double(P.r36_timing_trials));
fprintf('exact curved C7 candidates                    = %.0f\n',double(P.r36_exact_curved_candidates));
fprintf('curved candidates with yaw-C6 theorem         = %.0f\n',double(P.r36_curved_heading_pass));
fprintf('best spatial path-speed UB before retiming    = %.9g m/s\n',double(P.r36_best_spatial_path_speed_ub));
fprintf('best total velocity UB after retiming         = %.9g m/s\n',double(P.r36_best_total_velocity_ub));
fprintf('best acceleration UB                          = %.9g m/s^2\n',double(P.r36_best_accel_ub));
fprintf('best jerk UB                                  = %.9g m/s^3\n',double(P.r36_best_jerk_ub));

fprintf('\n[RETAINED R34/R35]\n');
fprintf('best progress scale alpha                     = %.12g\n',double(P.r32_best_progress_scale));
fprintf('best entry / flow / exit [s]                  = %.9g / %.9g / %.9g\n', ...
    double(P.r32_best_entry_T),double(P.r32_best_flow_T),double(P.r32_best_exit_T));
fprintf('heading candidates                            = %.0f\n',double(P.r35_heading_candidate_count));
fprintf('heading yaw-C6 pass                           = %.0f\n',double(P.r35_heading_pass_count));
fprintf('best heading gamma                            = %.12g m/s\n',double(P.r35_best_heading_gamma_m_s));

fprintf('\n[HARD CERTIFICATES]\n');
fprintf('candidate success                             = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                              = %s\n',char(P.predeadline_candidate_family));
fprintf('exact C7                                      = %d\n',logical(P.predeadline_c7_exact));
fprintf('speed / rejoin / geometry                     = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('geometry pass                                 = %d\n',logical(P.predeadline_geometry_pass));

fprintf('\n[PIPELINE]\n');
fprintf('state / reason                                = %s / %s\n',char(P.state),char(P.last_reason));
fprintf('stage1 / stage2 / stage3 [ms]                 = %.6g / %.6g / %.6g\n', ...
    double(P.last_stage1_ms),double(P.last_stage2_ms),double(P.last_stage3_ms));
fprintf('commit / fail                                 = %.0f / %.0f\n',double(P.commit_count),double(P.fail_count));
fprintf('active repaired safety level                  = %s\n',char(A.active_repaired_safety_level));
fprintf('====================================================================================================\n');

S=struct('schema','AF7R36_SPATIAL_AUDIT','pipeline',P,'avoidance',A);
end
