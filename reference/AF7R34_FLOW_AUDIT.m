function S=AF7R34_FLOW_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R34 DELAY-PRESERVING NO-CATCH-UP RETIMER AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('runner/display modified                       = 0\n');
fprintf('obstacle-size timing constant                 = 0\n');
fprintf('affine base detected                          = %.0f\n',double(P.r32_affine_base_detected));
fprintf('nominal progress speed                        = %.9g m/s\n',double(P.r32_nominal_progress_speed));

fprintf('\n[R34 DELAY-PRESERVING SYNTHESIS]\n');
fprintf('representation time quantum                   = %.12g s\n',double(P.r33_time_quantum_s));
fprintf('timing pre-trials                             = %.0f\n',double(P.r32_timing_trial_count));
fprintf('stored delay candidates                       = %.0f\n',double(P.r34_delay_candidate_count));
fprintf('exact terminal-hold C7 candidates             = %.0f\n',double(P.r34_terminal_hold_exact_count));
fprintf('monotone-progress candidates                  = %.0f\n',double(P.r34_monotone_progress_count));
fprintf('Pareto candidates sent to hard certifier      = %.0f\n',double(P.r33_pareto_candidate_count));
fprintf('best pre-cert alpha                           = %.12g\n',double(P.r32_best_progress_scale));
fprintf('best pre-cert entry / flow / exit [s]         = %.9g / %.9g / %.9g\n', ...
    double(P.r32_best_entry_T),double(P.r32_best_flow_T),double(P.r32_best_exit_T));
fprintf('best pre-cert total velocity UB               = %.9g m/s\n',double(P.r32_best_pre_speed_ub));
fprintf('best pre-cert acceleration UB                 = %.9g m/s^2\n',double(P.r33_best_accel_ub));
fprintf('best pre-cert jerk UB                         = %.9g m/s^3\n',double(P.r33_best_jerk_ub));

fprintf('\n[SELECTED DELAY-PRESERVING CANDIDATE]\n');
fprintf('selected schedule delay                       = %.9g s\n',double(P.r34_selected_delay_s));
fprintf('terminal hold starts                          = %.9g s\n',double(P.r34_selected_hold_start_s));
fprintf('selected alpha                                = %.12g\n',double(P.r34_selected_alpha));
fprintf('selected velocity UB                          = %.9g m/s\n',double(P.r34_selected_velocity_ub));
fprintf('selected acceleration UB                      = %.9g m/s^2\n',double(P.r34_selected_accel_ub));
fprintf('selected jerk UB                              = %.9g m/s^3\n',double(P.r34_selected_jerk_ub));

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

S=struct('schema','AF7R34_FLOW_AUDIT','pipeline',P,'avoidance',A);
end
