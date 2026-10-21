function S = AF7R36_TOWING_C7_SPEED_OPT_SPLIT_R17_POSTRUN()
% Uses the R16C pre-deadline latch. Timing gate remains authoritative and
% unchanged; this evaluates trajectory-family/safety closure separately.

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R17-POST] replanner_instance unavailable.');

R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

fam=char(P.predeadline_candidate_family);
succ=logical(P.predeadline_candidate_success);
geom=logical(P.predeadline_geometry_pass);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING C7 SPEED-OPT SPLIT R17 RESULT\n');
fprintf(' trajectory/safety assessment first | timing gate unchanged\n');
fprintf('====================================================================================================\n');

fprintf('\n[RETURNED HARD-CERTIFIED CANDIDATE]\n');
fprintf('candidate seen                             = %d\n',logical(P.predeadline_candidate_seen));
fprintf('candidate success                          = %d\n',succ);
fprintf('candidate family                           = %s\n',fam);
fprintf('reason code                                = %.0f\n',double(P.predeadline_reason_code));
fprintf('t0 / t1                                    = %.9g / %.9g s\n', ...
    double(P.predeadline_t0),double(P.predeadline_t1));
fprintf('correction envelope                        = %.9g m\n',double(P.predeadline_envelope_m));
fprintf('payload speed UB                           = %.9g m/s\n',double(P.predeadline_payload_speed));
fprintf('UAV speed UB                               = %.9g m/s\n',double(P.predeadline_uav_speed));
fprintf('cable qdot UB                              = %.9g 1/s\n',double(P.predeadline_qdot));
fprintf('geometry pass                              = %d\n',geom);
fprintf('speed/rejoin/geometry counts               = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('constant unsafe-span correction flag       = %d\n', ...
    logical(P.predeadline_constant_translation));

fprintf('\n[TIMING -- RECORDED ONLY]\n');
fprintf('Stage1 wall time                           = %.9g ms\n',double(P.last_stage1_ms));
fprintf('pipeline last reason                       = %s\n',char(P.last_reason));

if succ && geom && strcmp(fam,'TOWING_C7_SPEED_OPT_SPLIT')
    cls='NEW_SPEED_OPT_SPLIT_FALLBACK_HARD_CERTIFIED__READY_TO_REMOVE_LEGACY_MINIMUM_SNAP';
elseif succ && geom && strcmp(fam,'TOWING_C7_MIN_NORM_CONSTANT')
    cls='R16_CONSTANT_FALLBACK_HARD_CERTIFIED__R17_SPLIT_NOT_NEEDED';
elseif succ && geom && strcmp(fam,'MINIMUM_SNAP_FALLBACK')
    cls='LEGACY_MINIMUM_SNAP_STILL_REQUIRED__R17_NOT_SUFFICIENT';
elseif succ && geom
    cls=['OTHER_HARD_CERTIFIED_FAMILY__' clean(fam)];
else
    cls='NO_HARD_CERTIFIED_CANDIDATE_RETURNED';
end

fprintf('\n[DECISION]\n');
fprintf('classification                             = %s\n',cls);

if strcmp(cls,'NEW_SPEED_OPT_SPLIT_FALLBACK_HARD_CERTIFIED__READY_TO_REMOVE_LEGACY_MINIMUM_SNAP')
    fprintf(['R17 closed the unchanged finite-sensing/rejoin/whole-body stack. ' ...
             'Next revision can remove historical minimum-snap from production hierarchy.\n']);
elseif strcmp(cls,'LEGACY_MINIMUM_SNAP_STILL_REQUIRED__R17_NOT_SUFFICIENT')
    fprintf(['R17 remains insufficient for this hard case. Do not tune weights; ' ...
             'the next redesign must add safe-span/unsafe-span shape freedom while preserving hard certificates.\n']);
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_TOWING_C7_SPEED_OPT_SPLIT_R17';
S.pipeline=P;
S.family=fam;
S.classification=cls;
end

function s=clean(x)
s=upper(char(string(x)));
if isempty(s),s='NONE';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end
