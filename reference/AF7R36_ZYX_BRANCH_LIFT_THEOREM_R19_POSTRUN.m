function S = AF7R36_ZYX_BRANCH_LIFT_THEOREM_R19_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R19-POST] replanner_instance unavailable.');

R=evalin('base','replanner_instance');
O=R.get_observer_hard_support_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 ZYX BRANCH-LIFT THEOREM R19 RESULT\n');
fprintf('====================================================================================================\n');

fprintf('observer initialized                       = %d\n',logical(O.initialized));
fprintf('observer fail count                        = %.0f\n',double(O.fail_count));
fprintf('observer update count                      = %.0f\n',double(O.update_count));
fprintf('last event time                            = %.9g s\n',double(O.last_event_time));

if isstruct(O.observer_support) && isfield(O.observer_support,'consecutive_measurement_contraction_used')
    B=O.observer_support;
    fprintf('last contraction used                      = %d\n', ...
        logical(B.consecutive_measurement_contraction_used));
    if isfield(B,'hidden_true_support') && isstruct(B.hidden_true_support)
        H=B.hidden_true_support;
        if isfield(H,'w_norm_ub')
            fprintf('current true omega norm UB                 = %.15g rad/s\n',double(H.w_norm_ub));
        end
    end
end

% R17 candidate result remains the trajectory-side authority of interest.
P=R.get_staged_pipeline_status();
if isfield(P,'predeadline_candidate_seen')
    fprintf('\n[R17 TRAJECTORY SIDE]\n');
    fprintf('candidate seen                             = %d\n',logical(P.predeadline_candidate_seen));
    fprintf('candidate success                          = %d\n',logical(P.predeadline_candidate_success));
    fprintf('candidate family                           = %s\n',char(P.predeadline_candidate_family));
    fprintf('correction envelope                        = %.9g m\n',double(P.predeadline_envelope_m));
    fprintf('payload / UAV speed UB                     = %.9g / %.9g m/s\n', ...
        double(P.predeadline_payload_speed),double(P.predeadline_uav_speed));
    fprintf('qdot UB                                    = %.9g 1/s\n',double(P.predeadline_qdot));
    fprintf('geometry pass                              = %d\n',logical(P.predeadline_geometry_pass));
    fprintf('speed/rejoin/geometry counts               = %.0f / %.0f / %.0f\n', ...
        double(P.predeadline_speed_pass_count), ...
        double(P.predeadline_rejoin_pass_count), ...
        double(P.predeadline_geometry_test_count));
end

if O.fail_count==0 && double(O.last_event_time)>=20.95
    cls='ZYX_PITCH_CHART_REMAINED_CLOSED__ROLL_YAW_BRANCH_HANDLED_BY_UNIQUE_PERIODIC_LIFT';
elseif O.fail_count>0 && isfield(O,'last_forward_failure') && ...
        isstruct(O.last_forward_failure) && isfield(O.last_forward_failure,'pitch_closed') && ...
        ~logical(O.last_forward_failure.pitch_closed)
    cls='TRUE_PITCH_CHART_PROOF_FAILURE_REMAINS';
else
    cls='OBSERVER_STOPPED_FOR_ANOTHER_FAIL_CLOSED_REASON';
end

fprintf('\n[DECISION]\n');
fprintf('classification                             = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_ZYX_BRANCH_LIFT_THEOREM_R19';
S.observer=O;
S.pipeline=P;
S.classification=cls;
end
