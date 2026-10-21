function S=AF7R27_FLOW_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();
L=R.get_last_replan_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R27 FLOWING EXACT-C7 PRIMARY AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('fixed-H trigger changed                       = 0\n');
fprintf('candidate success                             = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                              = %s\n',char(P.predeadline_candidate_family));
fprintf('exact C7                                      = %d\n',logical(P.predeadline_c7_exact));
fprintf('speed / rejoin / geometry                     = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('geometry pass                                 = %d\n',logical(P.predeadline_geometry_pass));

fprintf('\n[PIPELINE / ACTIVE]\n');
fprintf('state / last reason                           = %s / %s\n',char(P.state),char(P.last_reason));
fprintf('stage1 / stage2 / stage3 [ms]                 = %.6g / %.6g / %.6g\n', ...
    double(P.last_stage1_ms),double(P.last_stage2_ms),double(P.last_stage3_ms));
fprintf('pipeline commit / fail                        = %.0f / %.0f\n', ...
    double(P.commit_count),double(P.fail_count));
fprintf('reference-only commit count                   = %.0f\n',double(A.reference_only_commit_count));
fprintf('active repaired safety level                  = %s\n',char(A.active_repaired_safety_level));

if isfield(P,'r24e_family_seen')
    seen=double(P.r24e_family_seen(:).');
    exact=double(P.r24e_family_exact(:).');
    xr=double(P.r24e_family_exact_reject(:).');
    sp=double(P.r24e_family_speed_pass(:).');
    sf=double(P.r24e_family_speed_fail(:).');
    names={'UNKNOWN','MINIMUM_DEVIATION_AFFINE','LEGACY_MINIMUM_SNAP','FAMILY_3', ...
        'TOWING_C7_MIN_NORM_CONSTANT','TOWING_C7_SPEED_OPT_SPLIT', ...
        'CURVED_C7_ZERO_JET_BELL','CURVED_C7_FLOW_AFFINE'};
    fprintf('\n[FAMILY BREAKDOWN]\n');
    fprintf('%-31s %6s %6s %7s %9s %9s\n', ...
        'family','seen','exact','C7rej','speedPASS','speedFAIL');
    for k=1:min(8,numel(seen))
        fprintf('%d %-29s %6.0f %6.0f %7.0f %9.0f %9.0f\n', ...
            k-1,names{k},seen(k),exact(k),xr(k),sp(k),sf(k));
    end
end

if isstruct(L)&&~isempty(fieldnames(L))
    fprintf('\n[LAST REPLAN]\n');
    if isfield(L,'committed'),fprintf('committed                                     = %d\n',logical(L.committed));end
    if isfield(L,'commit_time'),fprintf('commit time                                   = %.9g s\n',double(L.commit_time));end
    if isfield(L,'rejoin_time'),fprintf('rejoin time                                   = %.9g s\n',double(L.rejoin_time));end
    if isfield(L,'reason'),fprintf('reason                                        = %s\n',char(L.reason));end
end
fprintf('====================================================================================================\n');

S=struct('schema','AF7R27_FLOW_AUDIT','pipeline',P,'avoidance',A,'last_replan',L);
end
