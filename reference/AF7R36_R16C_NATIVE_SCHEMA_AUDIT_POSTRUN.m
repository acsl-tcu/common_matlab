function S = AF7R36_R16C_NATIVE_SCHEMA_AUDIT_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R16C-POST] replanner_instance unavailable.');

R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 R16C PRE-DEADLINE NATIVE-CANDIDATE RESULT\n');
fprintf(' planner/safety/timing gates unchanged\n');
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
fprintf('constant-translation flag                  = %d\n', ...
    logical(P.predeadline_constant_translation));
fprintf('Stage1 wall time                           = %.9g ms\n',double(P.last_stage1_ms));
fprintf('pipeline last reason                       = %s\n',char(P.last_reason));

fam=char(P.predeadline_candidate_family);
success=logical(P.predeadline_candidate_success);
geom=logical(P.predeadline_geometry_pass);

if success && strcmp(fam,'TOWING_C7_MIN_NORM_CONSTANT') && geom
    cls='NEW_TOWING_FALLBACK_HARD_CERTIFIED__TIMING_GATE_ONLY_BLOCKED_PIPELINE';
elseif success && strcmp(fam,'MINIMUM_SNAP_FALLBACK') && geom
    cls='LEGACY_MINIMUM_SNAP_STILL_WINS__NEW_CONSTANT_FALLBACK_INSUFFICIENT';
elseif success && geom
    cls=['OTHER_HARD_CERTIFIED_FAMILY__' clean(fam)];
elseif ~success
    cls='NO_HARD_CERTIFIED_NATIVE_CANDIDATE';
else
    cls=['SUCCESS_WITH_UNEXPECTED_GEOMETRY_STATE__' clean(fam)];
end

fprintf('\n[DECISION]\n');
fprintf('classification                             = %s\n',cls);

if strcmp(cls,'NEW_TOWING_FALLBACK_HARD_CERTIFIED__TIMING_GATE_ONLY_BLOCKED_PIPELINE')
    fprintf('Trajectory/safety side is closed by the new fallback. Runtime optimization may be deferred.\n');
elseif strcmp(cls,'LEGACY_MINIMUM_SNAP_STILL_WINS__NEW_CONSTANT_FALLBACK_INSUFFICIENT')
    fprintf('The constant-offset fallback is too restrictive for this hard case; redesign the new fallback family before removing legacy minimum-snap.\n');
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_R16C_NATIVE_SCHEMA_AUDIT';
S.pipeline=P;
S.classification=cls;
end

function s=clean(x)
s=upper(char(string(x)));
if isempty(s),s='NONE';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end
