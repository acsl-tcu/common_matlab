function S=AF7R29_FLOW_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();
L=R.get_last_replan_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R29 GENERAL FLOW / PRE-CERT / ACTIVE-COMMIT AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('obstacle attitude                             = arbitrary SO(3), oriented exact-support geometry\n');
fprintf('candidate success                             = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                              = %s\n',char(P.predeadline_candidate_family));
fprintf('exact C7                                      = %d\n',logical(P.predeadline_c7_exact));
fprintf('speed / rejoin / geometry                     = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('geometry pass                                 = %d\n',logical(P.predeadline_geometry_pass));

fprintf('\n[LATEST ATTEMPT]\n');
fprintf('state / reason                                = %s / %s\n',char(P.state),char(P.last_reason));
fprintf('stage1 / stage2 / stage3 [ms]                 = %.6g / %.6g / %.6g\n', ...
    double(P.last_stage1_ms),double(P.last_stage2_ms),double(P.last_stage3_ms));
fprintf('pipeline commit / fail                        = %.0f / %.0f\n', ...
    double(P.commit_count),double(P.fail_count));

fprintf('\n[ACTIVE COMMITTED REFERENCE]\n');
fprintf('reference-only commit count                   = %.0f\n',double(A.reference_only_commit_count));
fprintf('execution-aware commit count                  = %.0f\n',double(A.execution_aware_commit_count));
fprintf('active repaired safety level                  = %s\n',char(A.active_repaired_safety_level));
if isstruct(L)&&~isempty(fieldnames(L))
    if isfield(L,'committed'),fprintf('last committed                                = %d\n',logical(L.committed));end
    if isfield(L,'commit_time'),fprintf('last commit time                              = %.9g s\n',double(L.commit_time));end
    if isfield(L,'rejoin_time'),fprintf('last rejoin time                              = %.9g s\n',double(L.rejoin_time));end
end

if isfield(P,'r24e_family_seen')
    seen=double(P.r24e_family_seen(:).');
    exact=double(P.r24e_family_exact(:).');
    xr=double(P.r24e_family_exact_reject(:).');
    sp=double(P.r24e_family_speed_pass(:).');
    sf=double(P.r24e_family_speed_fail(:).');
    names={'UNKNOWN','MINIMUM_DEVIATION_AFFINE','LEGACY_MINIMUM_SNAP','FAMILY_3', ...
        'TOWING_C7_MIN_NORM_CONSTANT','TOWING_C7_SPEED_OPT_SPLIT', ...
        'CURVED_C7_ZERO_JET_BELL','CURVED_C7_FLOW_AFFINE'};
    fprintf('\n[LATEST ATTEMPT FAMILY BREAKDOWN]\n');
    fprintf('%-31s %6s %6s %7s %9s %9s\n', ...
        'family','seen','exact','C7rej','speedPASS','speedFAIL');
    for k=1:min(8,numel(seen))
        fprintf('%d %-29s %6.0f %6.0f %7.0f %9.0f %9.0f\n', ...
            k-1,names{k},seen(k),exact(k),xr(k),sp(k),sf(k));
    end
end

fprintf('\n[R29 FLOW PRE-CERT DIAGNOSTICS]\n');
fprintf('side assignments                              = %.0f\n',double(P.r28_flow_side_assignments));
fprintf('QP solutions                                  = %.0f\n',double(P.r28_flow_qp_solutions));
fprintf('stored candidates built                       = %.0f\n',double(P.r28_flow_built));
fprintf('stored-row rejects                            = %.0f\n',double(P.r28_flow_row_reject));
fprintf('exact-grid rejects                            = %.0f\n',double(P.r28_flow_grid_reject));
fprintf('finite-support rejects                        = %.0f\n',double(P.r28_flow_support_reject));
fprintf('common exact step h                           = %.9g s\n',double(P.r28_flow_step_s));
fprintf('flow start                                    = %.9g s\n',double(P.r28_flow_entry_end_s));
fprintf('flow end / exit start                         = %.9g s\n',double(P.r28_flow_exit_start_s));
fprintf('exit end                                      = %.9g s\n',double(P.r28_flow_exit_end_s));

% 3-D presentation must never make the audit fail.
try
    V=AF7R29_OPEN_3D_WINDOW();
catch ME
    V=struct('schema','AF7R29_DEDICATED_3D','created',false, ...
        'figure',NaN,'message',char(ME.message));
end
fprintf('\n[DEDICATED 3-D WINDOW]\n');
fprintf('created                                       = %d\n',logical(V.created));
if isfield(V,'figure'),fprintf('figure                                        = %.0f\n',double(V.figure));end
if isfield(V,'message'),fprintf('view status                                   = %s\n',char(V.message));end
fprintf('====================================================================================================\n');

S=struct('schema','AF7R29_FLOW_AUDIT','pipeline',P,'avoidance',A, ...
    'last_replan',L,'view3d',V);
end
