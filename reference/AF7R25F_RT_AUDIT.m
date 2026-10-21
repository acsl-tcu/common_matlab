function S=AF7R25F_RT_AUDIT()
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
L=R.get_last_replan_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R25F HARD-REAL-TIME / COMMIT AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('hard online deadline                         = 4 ms maximum\n');
fprintf('staged candidate success                     = %d\n',logical(P.predeadline_candidate_success));
fprintf('staged candidate family                      = %s\n',char(P.predeadline_candidate_family));
fprintf('staged exact C7                              = %d\n',logical(P.predeadline_c7_exact));
fprintf('reference-only commit count                  = %.0f\n',double(A.reference_only_commit_count));
fprintf('execution-aware commit count                 = %.0f\n',double(A.execution_aware_commit_count));
fprintf('active repaired safety level                 = %s\n',char(A.active_repaired_safety_level));

if isstruct(L)
    if isfield(L,'committed'),fprintf('last committed                               = %d\n',logical(L.committed));end
    if isfield(L,'reason'),fprintf('last reason                                  = %s\n',char(L.reason));end
    if isfield(L,'cycle_ms'),fprintf('last cycle                                   = %.9g ms\n',double(L.cycle_ms));end
    if isfield(L,'deadline_ms'),fprintf('last deadline                                = %.9g ms\n',double(L.deadline_ms));end
end

fprintf('\n[FAMILY]\n');
if isfield(P,'r24e_family_seen')
    seen=double(P.r24e_family_seen(:).');
    exact=double(P.r24e_family_exact(:).');
    sp=double(P.r24e_family_speed_pass(:).');
    sf=double(P.r24e_family_speed_fail(:).');
    fprintf('family6 seen/exact/speedPASS/speedFAIL       = %.0f / %.0f / %.0f / %.0f\n', ...
        seen(7),exact(7),sp(7),sf(7));
end
fprintf('====================================================================================================\n');

S=struct('schema','AF7R25F_RT_AUDIT','pipeline',P,'last_replan',L,'avoidance',A);
end
