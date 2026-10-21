function S = AF7R36_R16B_PREDEADLINE_CANDIDATE_AUDIT_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R16B-POST] replanner_instance unavailable.');

R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 R16B PRE-DEADLINE CANDIDATE AUDIT RESULT\n');
fprintf(' timing gate unchanged; this reads the candidate returned immediately before that gate\n');
fprintf('====================================================================================================\n');

fprintf('candidate seen                             = %d\n',logical(P.predeadline_candidate_seen));
fprintf('candidate success                          = %d\n',logical(P.predeadline_candidate_success));
fprintf('candidate family                           = %s\n',char(P.predeadline_candidate_family));
fprintf('reason code                                = %.0f\n',double(P.predeadline_reason_code));
fprintf('t0 / t1                                    = %.9g / %.9g s\n', ...
    double(P.predeadline_t0),double(P.predeadline_t1));
fprintf('correction envelope                        = %.9g m\n',double(P.predeadline_envelope_m));
fprintf('payload speed UB                           = %.9g m/s\n',double(P.predeadline_payload_speed));
fprintf('UAV speed UB                               = %.9g m/s\n',double(P.predeadline_uav_speed));
fprintf('cable qdot UB                              = %.9g 1/s\n',double(P.predeadline_qdot));
fprintf('geometry pass                              = %d\n',logical(P.predeadline_geometry_pass));
fprintf('speed/rejoin/geometry counts               = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('constant-translation family flag           = %d\n', ...
    logical(P.predeadline_constant_translation));
fprintf('Stage1 wall time                           = %.9g ms\n',double(P.last_stage1_ms));
fprintf('pipeline last reason                       = %s\n',char(P.last_reason));

fam=char(P.predeadline_candidate_family);
if logical(P.predeadline_candidate_success) && ...
        strcmp(fam,'TOWING_C7_MIN_NORM_CONSTANT') && ...
        logical(P.predeadline_geometry_pass)
    cls='NEW_TOWING_FALLBACK_HARD_CERTIFIED_BEFORE_TIMING_GATE';
elseif logical(P.predeadline_candidate_success) && strcmp(fam,'MINIMUM_SNAP_FALLBACK')
    cls='LEGACY_FALLBACK_STILL_REQUIRED__NEW_CONSTANT_FAMILY_DID_NOT_CLOSE_HARD_GATES';
elseif logical(P.predeadline_candidate_success)
    cls=['OTHER_HARD_CERTIFIED_FAMILY__' clean(fam)];
else
    cls='NATIVE_PLANNER_RETURNED_NO_HARD_CERTIFIED_CANDIDATE';
end

fprintf('\n[DECISION]\n');
fprintf('classification                             = %s\n',cls);

if strcmp(cls,'NEW_TOWING_FALLBACK_HARD_CERTIFIED_BEFORE_TIMING_GATE')
    fprintf(['Interpretation: trajectory/safety side is closed by the new fallback. ' ...
             'The present failure is scheduling only; legacy minimum-snap can proceed to removal audit.\n']);
elseif contains(cls,'LEGACY_FALLBACK_STILL_REQUIRED')
    fprintf(['Interpretation: constant-offset fallback is not sufficient for this hard case. ' ...
             'Do not remove legacy yet; redesign the new fallback shape family.\n']);
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_R16B_PREDEADLINE_CANDIDATE_AUDIT';
S.pipeline=P;
S.classification=cls;
end

function s=clean(x)
s=upper(char(string(x)));
if isempty(s),s='NONE';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end
