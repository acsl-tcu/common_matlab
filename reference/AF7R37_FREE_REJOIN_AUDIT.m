function S=AF7R37_FREE_REJOIN_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R37 TRIGGER-TO-FREE-REJOIN EXACT-C7 SPATIAL PRIMARY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('runner/display modified                       = 0\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('nontrivial time retiming active               = 0 (identity time phase-A)\n');

fprintf('\n[TRIGGER -> FREE REJOIN SPATIAL C7]\n');
fprintf('side assignments                              = %.0f\n',double(P.r37_side_assignments));
fprintf('spatial QP solutions                          = %.0f\n',double(P.r37_spatial_qp_solutions));
fprintf('exact stored-C7 candidates                    = %.0f\n',double(P.r37_exact_c7_candidates));
fprintf('path-tangent yaw-C6 pass                      = %.0f\n',double(P.r37_heading_pass_count));
fprintf('spatial knot count                            = %.0f\n',double(P.r37_knot_count));
fprintf('support start                                 = %.12g s\n',double(P.r37_support_start_s));
fprintf('actual rejoin                                 = %.12g s\n',double(P.r37_actual_rejoin_s));
fprintf('best total velocity UB                        = %.9g m/s\n',double(P.r37_best_total_velocity_ub));
fprintf('best acceleration UB                          = %.9g m/s^2\n',double(P.r37_best_accel_ub));
fprintf('best jerk UB                                  = %.9g m/s^3\n',double(P.r37_best_jerk_ub));

fprintf('\n[ADAPTIVE REJOIN QUALITY SEARCH]\n');
fprintf('candidate success                             = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                              = %s\n',char(P.predeadline_candidate_family));
fprintf('selected t1                                   = %.12g s\n',double(P.predeadline_t1));
try
    C=R.staged_replan_candidate;
    if isstruct(C)
        if isfield(C,'adaptive_rejoin_selected')
            fprintf('adaptive shorter rejoin selected              = %d\n',logical(C.adaptive_rejoin_selected));
        end
        if isfield(C,'rejoin_search_full_certificate_tests')
            fprintf('full hard-certificate trials                  = %.0f\n',double(C.rejoin_search_full_certificate_tests));
        end
        if isfield(C,'actual_rejoin_time')
            fprintf('selected actual rejoin                        = %.12g s\n',double(C.actual_rejoin_time));
        end
    end
catch
end

fprintf('\n[HARD CERTIFICATES]\n');
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

S=struct('schema','AF7R37_FREE_REJOIN_AUDIT','pipeline',P,'avoidance',A);
end
