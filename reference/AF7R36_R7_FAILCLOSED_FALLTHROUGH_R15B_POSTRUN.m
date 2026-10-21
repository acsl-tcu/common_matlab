function S = AF7R36_R7_FAILCLOSED_FALLTHROUGH_R15B_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R15B-POST] replanner_instance is unavailable.');

R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

mode='';ver=NaN;commitCount=NaN;
try,mode=char(R.active_reference_mode);catch,end
try,ver=double(R.active_reference_version);catch,end
try,commitCount=double(R.replan_commit_count);catch,end

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 R7 FAIL-CLOSED FALLTHROUGH R15B RESULT\n');
fprintf('====================================================================================================\n');

fprintf('\n[PIPELINE]\n');
fprintf('state                                      = %s\n',char(P.state));
fprintf('Stage1 count                               = %.0f\n',double(P.stage1_count));
fprintf('Stage2 count                               = %.0f\n',double(P.stage2_count));
fprintf('Stage3 commit count                        = %.0f\n',double(P.commit_count));
fprintf('stale reject count                         = %.0f\n',double(P.stale_reject_count));
fprintf('fail count                                 = %.0f\n',double(P.fail_count));
fprintf('last Stage1 ms                             = %.9g\n',double(P.last_stage1_ms));
fprintf('last Stage2 ms                             = %.9g\n',double(P.last_stage2_ms));
fprintf('last Stage3 ms                             = %.9g\n',double(P.last_stage3_ms));
fprintf('last reason                                = %s\n',char(P.last_reason));

if isfield(P,'native_error_count')
    fprintf('\n[R14C PERSISTED NATIVE ERROR]\n');
    fprintf('native error count                         = %.0f\n',double(P.native_error_count));
    fprintf('first identifier                           = %s\n',char(P.first_native_error_identifier));
    fprintf('first message                              = %s\n',flatten(P.first_native_error_message));
    fprintf('last identifier                            = %s\n',char(P.last_native_error_identifier));
    fprintf('last message                               = %s\n',flatten(P.last_native_error_message));
end

fprintf('\n[AUTHORITY]\n');
fprintf('commit policy                              = %s\n',char(A.commit_policy));
fprintf('reference-only evaluation count           = %.0f\n',double(A.reference_only_evaluation_count));
fprintf('reference-only commit count               = %.0f\n',double(A.reference_only_commit_count));
fprintf('active mode                                = %s\n',mode);
fprintf('active version                             = %.0f\n',ver);
fprintf('replan_commit_count                        = %.0f\n',commitCount);

if double(P.commit_count)>0 && strcmpi(mode,'REPAIRED') && ver>0
    cls='R7_FAILURE_FELL_THROUGH_AND_LIVE_COMMIT_CLOSED';
elseif double(P.stage2_count)>0
    cls=['R7_FATAL_EXCEPTION_REMOVED__STAGE2_REACHED__' clean(P.last_reason)];
elseif double(P.stage1_count)>0
    cls=['R7_FATAL_EXCEPTION_REMOVED__STAGE1_SUCCEEDS__' clean(P.last_reason)];
elseif isfield(P,'native_error_count') && double(P.native_error_count)>0
    cls=['NATIVE_EXCEPTION_STILL_EXISTS__' clean(P.first_native_error_message)];
else
    cls=['NO_STAGE1_SUCCESS__' clean(P.last_reason)];
end

fprintf('\n[DECISION]\n');
fprintf('classification                             = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_R7_FAILCLOSED_FALLTHROUGH_R15B';
S.pipeline=P;
S.policy=A;
S.active_mode=mode;
S.active_version=ver;
S.replan_commit_count=commitCount;
S.classification=cls;
end

function s=flatten(x)
s=char(string(x));
s=strrep(s,sprintf('\r'),' ');
s=strrep(s,sprintf('\n'),' | ');
end

function s=clean(x)
s=upper(char(string(x)));
if isempty(s),s='NO_REASON';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end
