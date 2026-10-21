function S = AF7R36_TOWING_MINNORM_FALLBACK_R16_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R16-POST] replanner_instance is unavailable.');

R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

mode='';ver=NaN;commitCount=NaN;
try,mode=char(R.active_reference_mode);catch,end
try,ver=double(R.active_reference_version);catch,end
try,commitCount=double(R.replan_commit_count);catch,end

family='';
if isfield(P,'last_candidate_family')
    family=char(P.last_candidate_family);
end

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING MIN-NORM CONSTANT FALLBACK R16 RESULT\n');
fprintf('====================================================================================================\n');

fprintf('\n[SELECTED LIVE FAMILY]\n');
fprintf('last candidate family                      = %s\n',family);

fprintf('\n[PIPELINE]\n');
fprintf('state                                      = %s\n',char(P.state));
fprintf('Stage1 count                               = %.0f\n',double(P.stage1_count));
fprintf('Stage2 count                               = %.0f\n',double(P.stage2_count));
fprintf('Stage3 commit count                        = %.0f\n',double(P.commit_count));
fprintf('fail count                                 = %.0f\n',double(P.fail_count));
fprintf('last Stage1 ms                             = %.9g\n',double(P.last_stage1_ms));
fprintf('last Stage2 ms                             = %.9g\n',double(P.last_stage2_ms));
fprintf('last Stage3 ms                             = %.9g\n',double(P.last_stage3_ms));
fprintf('last reason                                = %s\n',char(P.last_reason));

fprintf('\n[AUTHORITY]\n');
fprintf('commit policy                              = %s\n',char(A.commit_policy));
fprintf('reference-only evaluation count           = %.0f\n',double(A.reference_only_evaluation_count));
fprintf('reference-only commit count               = %.0f\n',double(A.reference_only_commit_count));
fprintf('active mode                                = %s\n',mode);
fprintf('active version                             = %.0f\n',ver);
fprintf('replan_commit_count                        = %.0f\n',commitCount);

newSelected=strcmp(family,'TOWING_C7_MIN_NORM_CONSTANT');
legacySelected=strcmp(family,'MINIMUM_SNAP_FALLBACK');
commitClosed=double(P.commit_count)>0 && strcmpi(mode,'REPAIRED') && ver>0;

if newSelected && commitClosed
    cls='NEW_TOWING_FALLBACK_LIVE_COMMIT_CLOSED__READY_FOR_LEGACY_REMOVAL_AUDIT';
elseif legacySelected && commitClosed
    cls='LEGACY_MINIMUM_SNAP_STILL_SELECTED__NEW_FALLBACK_NOT_YET_REPLACEMENT_CAPABLE';
elseif commitClosed
    cls=['LIVE_COMMIT_CLOSED_WITH_OTHER_FAMILY__' clean(family)];
elseif newSelected
    cls=['NEW_TOWING_FALLBACK_GENERATED_BUT_NO_COMMIT__' clean(P.last_reason)];
else
    cls=['NEW_TOWING_FALLBACK_NOT_SELECTED__' clean(P.last_reason)];
end

fprintf('\n[DECISION]\n');
fprintf('new fallback selected                      = %d\n',newSelected);
fprintf('legacy minimum-snap selected               = %d\n',legacySelected);
fprintf('live commit closed                         = %d\n',commitClosed);
fprintf('classification                             = %s\n',cls);

if newSelected && commitClosed
    fprintf('Next step: remove historical minimum-snap from production hierarchy and retain only an explicit safety backup layer.\n');
elseif legacySelected
    fprintf('Do NOT remove the legacy branch yet; the new fallback did not close this hard case.\n');
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_TOWING_MINNORM_CONSTANT_R16';
S.pipeline=P;
S.policy=A;
S.family=family;
S.new_fallback_selected=newSelected;
S.legacy_selected=legacySelected;
S.commit_closed=commitClosed;
S.classification=cls;
end

function s=clean(x)
s=upper(char(string(x)));
if isempty(s),s='NONE';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end
