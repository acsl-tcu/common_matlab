function S = AF7R36_ROT_ENERGY_RECURSIVE_SUPPORT_R20_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R20-POST] replanner_instance unavailable.');

R=evalin('base','replanner_instance');
O=R.get_observer_hard_support_status();
P=R.get_staged_pipeline_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 ROTATIONAL ENERGY RECURSIVE SUPPORT R20 RESULT\n');
fprintf('====================================================================================================\n');

fprintf('\n[OBSERVER]\n');
fprintf('initialized                                = %d\n',logical(O.initialized));
fprintf('fail count                                 = %.0f\n',double(O.fail_count));
fprintf('update count                               = %.0f\n',double(O.update_count));
fprintf('last event time                            = %.9g s\n',double(O.last_event_time));

if isstruct(O.observer_support) && isfield(O.observer_support,'hidden_true_support')
    H=O.observer_support.hidden_true_support;
    if isfield(H,'w_norm_ub')
        fprintf('current omega norm UB                      = %.15g rad/s\n',double(H.w_norm_ub));
    end
    if isfield(H,'w_sqrt_energy_ub')
        sv=double(H.w_sqrt_energy_ub);
        fprintf('current sqrt rotational-energy UB          = %.15g sqrt(J)\n',sv);
    else
        fprintf('current sqrt rotational-energy UB          = NOT AVAILABLE\n');
    end
end

if isfield(O,'last_forward_support') && isstruct(O.last_forward_support) && ...
        isfield(O.last_forward_support,'w_sqrt_energy_ub_start')
    F=O.last_forward_support;
    fprintf('\n[LAST FORWARD ENERGY STEP]\n');
    fprintf('sqrt(V) start / end                       = %.15g / %.15g\n', ...
        double(F.w_sqrt_energy_ub_start),double(F.w_sqrt_energy_ub_end));
    fprintf('omega UB start / end                      = %.15g / %.15g rad/s\n', ...
        double(F.w_norm_ub_energy_start),double(F.w_norm_ub_end));
    fprintf('Iw                                        = %.15g rad\n',double(F.Iw));
    fprintf('Theta                                     = %.15g rad\n',double(F.Theta));
    fprintf('pitch margin                              = %+.15e rad\n',double(F.pitch_margin_rad));
    fprintf('energy seeded from ball this step         = %d\n',logical(F.energy_seed_from_ball));
end

if isfield(O,'last_forward_failure') && isstruct(O.last_forward_failure) && ...
        isfield(O.last_forward_failure,'Theta')
    Ff=O.last_forward_failure;
    fprintf('\n[LAST FORWARD FAILURE]\n');
    fprintf('reason                                     = %s\n',char(Ff.closure_reason));
    fprintf('Theta / pitch margin                       = %.15g / %+.15e rad\n', ...
        double(Ff.Theta),double(Ff.pitch_margin_rad));
    fprintf('Iw                                         = %.15g rad\n',double(Ff.Iw));
end

fprintf('\n[R17 TRAJECTORY SIDE -- LAST RETRY DIAGNOSTIC ONLY]\n');
if isfield(P,'predeadline_candidate_seen')
    fprintf('candidate seen                             = %d\n',logical(P.predeadline_candidate_seen));
    fprintf('candidate success                          = %d\n',logical(P.predeadline_candidate_success));
    fprintf('candidate family                           = %s\n',char(P.predeadline_candidate_family));
    fprintf('reason code                                = %.0f\n',double(P.predeadline_reason_code));
    fprintf('geometry pass                              = %d\n',logical(P.predeadline_geometry_pass));
    fprintf('speed/rejoin/geometry counts               = %.0f / %.0f / %.0f\n', ...
        double(P.predeadline_speed_pass_count), ...
        double(P.predeadline_rejoin_pass_count), ...
        double(P.predeadline_geometry_test_count));
end

if O.fail_count==0 && double(O.last_event_time)>20.975
    obsCls='RECURSIVE_ENERGY_SUPPORT_CLOSED_PAST_PREVIOUS_FAILURE';
elseif O.fail_count>0
    obsCls='OBSERVER_FAIL_CLOSED_REMAINS__INSPECT_RECORDED_WITNESS';
else
    obsCls='OBSERVER_RUN_ENDED_BEFORE_PREVIOUS_FAILURE_TIME';
end

trajCls='LAST_PREDEADLINE_RETRY_ONLY__NOT_R17_REPLACEMENT_AUTHORITY';

fprintf('\n[DECISION]\n');
fprintf('observer classification                    = %s\n',obsCls);
fprintf('trajectory diagnostic                      = %s\n',trajCls);
fprintf('NOTE: predeadline candidate fields are overwritten on each Stage1 retry; do not use this last value to decide R17 replacement capability.\n');
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_ROT_ENERGY_RECURSIVE_SUPPORT_R20';
S.observer=O;
S.pipeline=P;
S.observer_classification=obsCls;
S.trajectory_classification=trajCls;
end

function s=clean(x)
s=upper(char(string(x)));
if isempty(s),s='NONE';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end
