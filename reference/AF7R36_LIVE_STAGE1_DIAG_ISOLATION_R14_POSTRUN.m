function S = AF7R36_LIVE_STAGE1_DIAG_ISOLATION_R14_POSTRUN()
% POSTRUN for R14. Reads actual live pipeline/commit authority.

assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[R14-POST] replanner_instance is unavailable.');
R=evalin('base','replanner_instance');

P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 LIVE STAGE1 DIAGNOSTIC ISOLATION R14 RESULT\n');
fprintf('====================================================================================================\n');

fprintf('\n[POLICY / AUTHORITY]\n');
fprintf('commit policy                              = %s\n',char(A.commit_policy));
fprintf('auto commit                                = %d\n',logical(A.auto_commit));
fprintf('reference-only evaluation count           = %.0f\n',double(A.reference_only_evaluation_count));
fprintf('reference-only commit count               = %.0f\n',double(A.reference_only_commit_count));

fprintf('\n[STAGED PIPELINE]\n');
fprintf('final state                                = %s\n',char(P.state));
fprintf('Stage1 count                               = %.0f\n',double(P.stage1_count));
fprintf('Stage2 count                               = %.0f\n',double(P.stage2_count));
fprintf('Stage3 commit count                        = %.0f\n',double(P.commit_count));
fprintf('stale reject count                         = %.0f\n',double(P.stale_reject_count));
fprintf('fail count                                 = %.0f\n',double(P.fail_count));
fprintf('last Stage1 ms                             = %.9g\n',double(P.last_stage1_ms));
fprintf('last Stage2 ms                             = %.9g\n',double(P.last_stage2_ms));
fprintf('last Stage3 ms                             = %.9g\n',double(P.last_stage3_ms));
fprintf('last reason                                = %s\n',char(P.last_reason));

fprintf('\n[PERSISTED STAGE1 NATIVE ERROR]\n');
fprintf('native error count                         = %.0f\n',double(P.native_error_count));
fprintf('first error time                           = %.9g s\n',double(P.first_native_error_time));
fprintf('first error identifier                     = %s\n',char(P.first_native_error_identifier));
fprintf('first error message                        = %s\n',flatten(P.first_native_error_message));
fprintf('last error time                            = %.9g s\n',double(P.last_native_error_time));
fprintf('last error identifier                      = %s\n',char(P.last_native_error_identifier));
fprintf('last error message                         = %s\n',flatten(P.last_native_error_message));

mode='';ver=NaN;commits=NaN;
try,mode=char(R.active_reference_mode);catch,end
try,ver=double(R.active_reference_version);catch,end
try,commits=double(R.replan_commit_count);catch,end

fprintf('\n[FINAL ACTIVE REFERENCE]\n');
fprintf('mode                                       = %s\n',mode);
fprintf('version                                    = %.0f\n',ver);
fprintf('replan_commit_count                        = %.0f\n',commits);

if double(P.commit_count)>0 && strcmpi(mode,'REPAIRED') && ver>0
    cls='LIVE_STAGE1_STAGE2_STAGE3_COMMIT_CLOSED_AFTER_DIAGNOSTIC_ISOLATION';
elseif double(P.stage2_count)>0 && double(P.commit_count)==0
    cls=['STAGE1_AND_STAGE2_NOW_REACHED__NEXT_FAILURE_' clean(P.last_reason)];
elseif double(P.stage1_count)>0 && double(P.stage2_count)==0
    cls=['STAGE1_NOW_SUCCEEDS__NEXT_FAILURE_' clean(P.last_reason)];
elseif double(P.native_error_count)>0
    cls=['STAGE1_NATIVE_ERROR_PERSISTS__' clean(P.first_native_error_identifier)];
else
    cls=['NO_COMMIT__' clean(P.last_reason)];
end

fprintf('\n[DECISION]\n');
fprintf('classification                             = %s\n',cls);

if strcmp(cls,'LIVE_STAGE1_STAGE2_STAGE3_COMMIT_CLOSED_AFTER_DIAGNOSTIC_ISOLATION')
    fprintf('Interpretation: R8/R9/R11 diagnostic work was interfering with live op1; live commit is now closed.\n');
elseif double(P.native_error_count)>0
    fprintf('Interpretation: diagnostic interference is no longer the only cause; the persisted first exception is now the exact repair target.\n');
elseif double(P.stage1_count)>0
    fprintf('Interpretation: Stage1 native generation now closes. Repair only the newly exposed later-stage gate.\n');
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_LIVE_STAGE1_DIAG_ISOLATION_R14';
S.policy=A;
S.pipeline=P;
S.active_mode=mode;
S.active_version=ver;
S.replan_commit_count=commits;
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
