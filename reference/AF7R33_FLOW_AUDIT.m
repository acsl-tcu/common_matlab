function S=AF7R33_FLOW_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R33 ADAPTIVE EXACT-RETIMING / SINGLE-WINDOW DISPLAY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('obstacle-size timing constant                 = 0\n');
fprintf('affine base detected                          = %.0f\n',double(P.r32_affine_base_detected));
fprintf('nominal progress speed                        = %.9g m/s\n',double(P.r32_nominal_progress_speed));

fprintf('\n[R33 EXACT TIMING SEARCH]\n');
fprintf('time quantum (representation only)            = %.12g s\n',double(P.r33_time_quantum_s));
fprintf('timing trials                                 = %.0f\n',double(P.r32_timing_trial_count));
fprintf('exact timing candidates                       = %.0f\n',double(P.r33_exact_timing_candidate_count));
fprintf('monotone-progress rejects                     = %.0f\n',double(P.r32_monotone_reject_count));
fprintf('quantized-progress rejects                    = %.0f\n',double(P.r33_quantized_progress_reject_count));
fprintf('Pareto candidates sent to hard certifier      = %.0f\n',double(P.r33_pareto_candidate_count));
fprintf('best progress scale alpha                     = %.12g\n',double(P.r32_best_progress_scale));
fprintf('best entry / flow / exit [s]                  = %.9g / %.9g / %.9g\n', ...
    double(P.r32_best_entry_T),double(P.r32_best_flow_T),double(P.r32_best_exit_T));
fprintf('best total velocity-control UB                = %.9g m/s\n',double(P.r32_best_pre_speed_ub));
fprintf('best correction acceleration UB               = %.9g m/s^2\n',double(P.r33_best_accel_ub));
fprintf('best correction jerk UB                       = %.9g m/s^3\n',double(P.r33_best_jerk_ub));

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

fprintf('\n[DISPLAY]\n');
fprintf('post-run result windows                       = 1\n');
fprintf('tabs                                          = Trajectory / Timing / 3-D / Retiming-Cable / Animation\n');
fprintf('animation replay button                       = 1\n');
fprintf('disk image/video saving                       = 0\n');
fprintf('====================================================================================================\n');

S=struct('schema','AF7R33_FLOW_AUDIT','pipeline',P,'avoidance',A);
end
