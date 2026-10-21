function S = AF7R36_LIVE_COMMIT_GATE_AUDIT_R13_POSTRUN()
% AF7R36_LIVE_COMMIT_GATE_AUDIT_R13_POSTRUN
%
% POSTRUN ONLY. No source modification and no rerun required.
%
% Determines whether the live Flight staged pipeline actually reached:
%   Stage 1 generation
%   Stage 2 reference certificate / bundle preparation
%   Stage 3 atomic activation
%
% Also reads LOGGER-published generator counters/status codes so warmup/MEX
% diagnostic candidate generation is not confused with an actual live commit.
%
% No sampled data is used for safety acceptance.

assert(evalin('base','exist(''logger'',''var'')==1'), ...
    '[COMMIT-R13] logger is unavailable.');
assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
    '[COMMIT-R13] replanner_instance is unavailable.');

logger=evalin('base','logger');
R=evalin('base','replanner_instance');

P=R.get_staged_pipeline_status();
A=R.get_avoidance_mode_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 LIVE COMMIT GATE AUDIT R13 RESULT\n');
fprintf(' postrun only | no source modification | no rerun\n');
fprintf('====================================================================================================\n');

fprintf('\n[POLICY]\n');
fprintf('commit policy                              = %s\n',char(A.commit_policy));
fprintf('reference-only evaluation enabled         = %d\n',logical(A.reference_only_evaluation_enabled));
fprintf('execution-aware evaluation enabled        = %d\n',logical(A.execution_aware_evaluation_enabled));
fprintf('auto commit                                = %d\n',logical(A.auto_commit));
fprintf('reference-only evaluation count           = %.0f\n',double(A.reference_only_evaluation_count));
fprintf('reference-only commit count               = %.0f\n',double(A.reference_only_commit_count));
fprintf('execution-aware evaluation count          = %.0f\n',double(A.execution_aware_evaluation_count));
fprintf('execution-aware commit count              = %.0f\n',double(A.execution_aware_commit_count));

fprintf('\n[STAGED PIPELINE]\n');
fprintf('enabled                                    = %d\n',logical(P.enabled));
fprintf('final state                                = %s\n',char(P.state));
fprintf('Stage1 count                               = %.0f\n',double(P.stage1_count));
fprintf('Stage2 count                               = %.0f\n',double(P.stage2_count));
fprintf('Stage3 commit count                        = %.0f\n',double(P.commit_count));
fprintf('stale reject count                         = %.0f\n',double(P.stale_reject_count));
fprintf('fail count                                 = %.0f\n',double(P.fail_count));
fprintf('last Stage1 runtime                        = %.9g ms\n',double(P.last_stage1_ms));
fprintf('last Stage2 runtime                        = %.9g ms\n',double(P.last_stage2_ms));
fprintf('last Stage3 runtime                        = %.9g ms\n',double(P.last_stage3_ms));
fprintf('last reason                                = %s\n',char(P.last_reason));
fprintf('pipeline delay                             = %.9g s\n',double(P.pipeline_delay_s));
fprintf('remaining TTC LB after activation         = %.9g s\n',double(P.remaining_ttc_after_scheduled_activation_LB_s));

% -------------------------------------------------------------------------
% Read actual LOGGER-published generator diagnostics.
% -------------------------------------------------------------------------
D=logger.Data;
AG=D.agent;
if numel(AG)>1, AG=AG(1); end

if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    n=result_count(AG,'reference');
    t=(0:n-1)'*0.025;
end

C=get_result_cells(AG,'reference');
n=min(numel(t),numel(C));
t=t(1:n);

names={ ...
    'generator_request_count', ...
    'generator_candidate_count', ...
    'generator_validation_pass_count', ...
    'generator_validation_fail_count', ...
    'generator_deadline_fail_count', ...
    'generator_last_status_code', ...
    'generator_last_runtime_ms', ...
    'replan_commit_count', ...
    'active_reference_version'};

X=nan(n,numel(names));
for i=1:n
    st=get_member(C{i},'state');
    for j=1:numel(names)
        q=get_member(st,names{j});
        if (isnumeric(q)||islogical(q)) && isscalar(q) && isfinite(double(q))
            X(i,j)=double(q);
        end
    end
end

flight=t>=5 & t<47;
F=X(flight,:);
tf=t(flight);

fprintf('\n[LIVE LOGGER GENERATOR COUNTERS -- FINAL FLIGHT VALUES]\n');
for j=1:numel(names)
    v=lastfinite(F(:,j));
    fprintf('%-42s = %.9g\n',names{j},v);
end

% Status-code transitions in Flight.
status=F(:,6);
chg=false(size(status));
last=NaN;
for i=1:numel(status)
    if isfinite(status(i))
        if ~isfinite(last) || status(i)~=last
            chg(i)=true;
            last=status(i);
        end
    end
end
I=find(chg);

fprintf('\n[GENERATOR STATUS-CODE TRANSITIONS IN FLIGHT]\n');
if isempty(I)
    fprintf('No finite generator status code was logged in Flight.\n');
else
    for k=1:numel(I)
        i=I(k);
        fprintf('t=%9.4f s | code=%6.0f | %s | requests=%g candidates=%g deadlineFails=%g commits=%g version=%g\n', ...
            tf(i),F(i,6),status_text(F(i,6)),F(i,1),F(i,2),F(i,5),F(i,8),F(i,9));
    end
end

req=lastfinite(F(:,1));
cand=lastfinite(F(:,2));
vpass=lastfinite(F(:,3));
vfail=lastfinite(F(:,4));
dfail=lastfinite(F(:,5));
codeLast=lastfinite(F(:,6));
commits=lastfinite(F(:,8));

if ~logical(P.enabled)
    cls='STAGED_PIPELINE_DISABLED';
elseif double(P.stage1_count)==0
    if isfinite(req) && req>0
        cls='LIVE_REQUEST_EXISTS_BUT_STAGED_STAGE1_NEVER_COMPLETED';
    else
        cls='NO_LIVE_REPLAN_REQUEST__PRIOR_MEX_CANDIDATES_WERE_NOT_LIVE_COMMIT_ATTEMPTS';
    end
elseif double(P.stage1_count)>0 && double(P.stage2_count)==0
    cls=['STAGE1_ONLY__' sanitize_reason(P.last_reason)];
elseif double(P.stage2_count)>0 && double(P.commit_count)==0
    cls=['STAGE2_REACHED_BUT_NO_STAGE3_COMMIT__' sanitize_reason(P.last_reason)];
elseif double(P.commit_count)>0 && (~isfinite(commits) || commits==0)
    cls='PIPELINE_REPORTS_COMMIT_BUT_LOGGER_AUTHORITY_DID_NOT_ADVANCE';
elseif double(P.commit_count)>0
    cls='STAGED_PIPELINE_COMMIT_OCCURRED';
else
    cls='LIVE_COMMIT_GATE_INCONCLUSIVE';
end

fprintf('\n[DECISION]\n');
fprintf('live generator request count               = %.9g\n',req);
fprintf('live candidate count                       = %.9g\n',cand);
fprintf('live validation pass/fail                  = %.9g / %.9g\n',vpass,vfail);
fprintf('live deadline fail count                   = %.9g\n',dfail);
fprintf('last generator status code                 = %.9g (%s)\n',codeLast,status_text(codeLast));
fprintf('classification                             = %s\n',cls);

if double(P.stage1_count)==0 && (~isfinite(req) || req==0)
    fprintf(['Interpretation: the moving candidates seen in the MEX diagnostic were not an actual live Flight commit attempt.\n' ...
             'The next target is trigger/request-edge authority, not commit assembly.\n']);
elseif double(P.stage1_count)>0 && double(P.stage2_count)==0
    fprintf(['Interpretation: a real staged candidate attempt started, but the pipeline died before Stage2 closure.\n' ...
             'Use last_reason/status code as the exact repair target.\n']);
elseif double(P.stage2_count)>0 && double(P.commit_count)==0
    fprintf(['Interpretation: generation and at least one Stage2 evaluation occurred; activation/commit did not close.\n' ...
             'Use last_reason/status code to repair only the failing Stage2/Stage3 gate.\n']);
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_LIVE_COMMIT_GATE_AUDIT_R13';
S.policy=A;
S.pipeline=P;
S.time=t;
S.logged=X;
S.logged_names=names;
S.classification=cls;
end

function n=result_count(A,name)
n=0;
if isstruct(A)&&isfield(A,name)
    Q=A.(name);
    if isstruct(Q)&&isfield(Q,'result'),n=numel(Q.result);end
end
end

function C=get_result_cells(A,name)
C={};
if ~isstruct(A)||~isfield(A,name),return;end
Q=A.(name);
if ~isstruct(Q)||~isfield(Q,'result'),return;end
V=Q.result;
if iscell(V),C=V(:);else,C=num2cell(V(:));end
end

function x=get_member(S,name)
x=[];
try
    if isempty(S),return;end
    if isstruct(S)
        if isfield(S,name),x=S.(name);end
    elseif isobject(S)
        if isprop(S,name),x=S.(name);end
    end
catch
    x=[];
end
end

function v=lastfinite(x)
i=find(isfinite(x),1,'last');
if isempty(i),v=NaN;else,v=x(i);end
end

function s=sanitize_reason(x)
s=upper(char(string(x)));
if isempty(s),s='NO_REASON';end
s=regexprep(s,'[^A-Z0-9_]+','_');
end

function s=status_text(c)
if ~isfinite(c),s='NO_STATUS';return;end
switch round(c)
    case 0,   s='IDLE_OR_NONE';
    case 20,  s='AUTO_COMMIT_DISABLED';
    case 21,  s='PROPOSAL_NOT_CERTIFIED';
    case 30,  s='COMMIT_CLOSED';
    case 31,  s='COMMIT_OR_PREP_REJECT';
    case 32,  s='REFERENCE_LAYER_DISABLED';
    case 33,  s='REFERENCE_CERTIFICATE_REJECT';
    case 40,  s='STAGE1_GENERATED_OR_EXEC_CERT_OPEN';
    case 41,  s='STAGE2_PREPARED_OR_EXEC_CERT_CLOSED';
    case 42,  s='EXECUTION_LAYER_DISABLED';
    case -2,  s='PRE_GENERATOR_DEADLINE_EXHAUSTED';
    case -20, s='GENERATOR_OR_PRECOMMIT_DEADLINE_EXCEEDED';
    case -30, s='COMMIT_DEADLINE_EXCEEDED';
    case -60, s='INSUFFICIENT_LEAD_FOR_PIPELINE';
    case -61, s='STAGE1_NATIVE_ERROR';
    case -62, s='STAGE1_CONTROL_PERIOD_EXCEEDED';
    case -63, s='PIPELINE_SOURCE_OR_SENSOR_STALE';
    case -64, s='STAGE2_GRID_MISSED';
    case -65, s='STAGE2_CONTROL_PERIOD_EXCEEDED';
    case -66, s='STAGE3_ACTIVATION_GRID_MISSED';
    case -67, s='STAGE3_COMMIT_REJECT';
    otherwise,s='UNMAPPED_STATUS';
end
end
