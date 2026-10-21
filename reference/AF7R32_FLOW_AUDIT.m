function S=AF7R32_FLOW_AUDIT()
R=evalin('base','replanner_instance');P=R.get_staged_pipeline_status();A=R.get_avoidance_mode_status();L=R.get_last_replan_status();
fprintf('\n====================================================================================================\n');
fprintf(' AF7R32B PROGRESS-RETIMED C7 / DISPLAY-ONLY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('candidate success                             = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                              = %s\n',char(P.predeadline_candidate_family));
fprintf('exact C7                                      = %d\n',logical(P.predeadline_c7_exact));
fprintf('speed / rejoin / geometry                     = %.0f / %.0f / %.0f\n',double(P.predeadline_speed_pass_count),double(P.predeadline_rejoin_pass_count),double(P.predeadline_geometry_test_count));
fprintf('geometry pass                                 = %d\n',logical(P.predeadline_geometry_pass));
fprintf('\n[R32 RETIMING]\n');
fprintf('affine base detected                          = %.0f\n',double(P.r32_affine_base_detected));
fprintf('timing trials                                 = %.0f\n',double(P.r32_timing_trial_count));
fprintf('exact timing candidates                       = %.0f\n',double(P.r32_exact_candidate_count));
fprintf('monotone-progress rejects                     = %.0f\n',double(P.r32_monotone_reject_count));
fprintf('nominal progress speed                        = %.9g m/s\n',double(P.r32_nominal_progress_speed));
fprintf('best progress scale alpha                     = %.9g\n',double(P.r32_best_progress_scale));
fprintf('best entry / flow / exit [s]                  = %.9g / %.9g / %.9g\n',double(P.r32_best_entry_T),double(P.r32_best_flow_T),double(P.r32_best_exit_T));
fprintf('best affine velocity-control UB               = %.9g m/s\n',double(P.r32_best_pre_speed_ub));
fprintf('\n[PIPELINE]\n');
fprintf('state / reason                                = %s / %s\n',char(P.state),char(P.last_reason));
fprintf('stage1 / stage2 / stage3 [ms]                 = %.6g / %.6g / %.6g\n',double(P.last_stage1_ms),double(P.last_stage2_ms),double(P.last_stage3_ms));
fprintf('commit / fail                                 = %.0f / %.0f\n',double(P.commit_count),double(P.fail_count));
fprintf('active repaired safety level                  = %s\n',char(A.active_repaired_safety_level));
if isstruct(L)&&isfield(L,'committed'),fprintf('last committed                                = %d\n',logical(L.committed));end
fprintf('\n[POST-RUN DISPLAY]\n');
fprintf('Figure 5                                      = trajectory / avoidance dashboard\n');
fprintf('Figure 10                                     = timing dashboard\n');
fprintf('Figure 12                                     = dedicated large 3-D view\n');
fprintf('Figure 13                                     = retiming + cable diagnostics\n');
fprintf('Figure 14                                     = display-only 3-D cable animation\n');
fprintf('disk image/video saving                       = 0\n');
fprintf('====================================================================================================\n');
S=struct('schema','AF7R32B_FLOW_AUDIT','pipeline',P,'avoidance',A,'last_replan',L);
end
