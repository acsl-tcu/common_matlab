function S=AF7R35_YAW_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R35 PATH-TANGENT YAW C6 + DELAY-PRESERVING RETIMER AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('runner/display modified                       = 0\n');
fprintf('fixed-H trigger changed                       = 0\n');

fprintf('\n[PATH-TANGENT YAW THEOREM]\n');
fprintf('heading candidates                            = %.0f\n',double(P.r35_heading_candidate_count));
fprintf('strict-half-plane yaw-C6 pass                 = %.0f\n',double(P.r35_heading_pass_count));
fprintf('heading regularity rejects                    = %.0f\n',double(P.r35_heading_reject_count));
fprintf('best certified horizontal projection gamma   = %.12g m/s\n',double(P.r35_best_heading_gamma_m_s));
fprintf('selected heading gamma                        = %.12g m/s\n',double(P.r35_selected_heading_gamma_m_s));
fprintf('selected heading anchor                       = %.12g rad\n',double(P.r35_selected_heading_anchor_rad));
fprintf('yaw derivative authority                      = analytic Im(log(vx+i*vy)), orders 0..6\n');

fprintf('\n[R34 DELAY-PRESERVING SYNTHESIS RETAINED]\n');
fprintf('stored delay candidates                       = %.0f\n',double(P.r34_delay_candidate_count));
fprintf('exact terminal-hold C7 candidates             = %.0f\n',double(P.r34_terminal_hold_exact_count));
fprintf('monotone-progress candidates                  = %.0f\n',double(P.r34_monotone_progress_count));
fprintf('best pre-cert alpha                           = %.12g\n',double(P.r32_best_progress_scale));
fprintf('best pre-cert entry / flow / exit [s]         = %.9g / %.9g / %.9g\n', ...
    double(P.r32_best_entry_T),double(P.r32_best_flow_T),double(P.r32_best_exit_T));
fprintf('best pre-cert total velocity UB               = %.9g m/s\n',double(P.r32_best_pre_speed_ub));

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

try
    AR=R.active_repaired_reference;
    if isstruct(AR)&&isfield(AR,'yaw_policy')
        fprintf('active yaw policy                             = %s\n',char(AR.yaw_policy));
        if isfield(AR,'yaw_c6_certificate') && ...
                isfield(AR.yaw_c6_certificate,'heading_gamma_m_s')
            fprintf('active yaw heading gamma                      = %.12g m/s\n', ...
                double(AR.yaw_c6_certificate.heading_gamma_m_s));
        end
    end
catch
end

fprintf('====================================================================================================\n');
S=struct('schema','AF7R35_YAW_AUDIT','pipeline',P,'avoidance',A);
end
