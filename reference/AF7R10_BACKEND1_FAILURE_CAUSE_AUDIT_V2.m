function OUT = AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_V2(logger,replanner)
% AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_V2
% =========================================================================
% Read-only diagnosis of backend-1 (BEZIER_D15_C7) request path.
%
% IMPORTANT:
%   * No production state is modified.
%   * No sampled safety acceptance.
%   * Existing logs only.
%   * status=-1 is correctly identified as CALLBACK_EXCEPTION_CAUGHT.
%   * The exact caught exception message cannot be reconstructed from an
%     already completed run because REPLANNING_SYSTEM currently catches it
%     without saving the MException object.
%
% Usage after an existing run:
%   AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_V2
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R10 V2 | BACKEND-1 FAILURE CAUSE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | backend1Only=1 | timingDiagnosis=1 | safetyPromotion=0\n');

if nargin<1 || isempty(logger)
    assert(evalin('base','exist(''logger'',''var'')==1'),'[AF7R10V2] logger missing.');
    logger=evalin('base','logger');
end
if nargin<2 || isempty(replanner)
    assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
        '[AF7R10V2] replanner_instance missing.');
    replanner=evalin('base','replanner_instance');
end

CAT=replanner.get_generator_backend_catalog();

fprintf('\n[A. BACKEND ROUTER]\n');
fprintf(' locked code/name                      = %.0f / %s\n',CAT.locked_code,CAT.locked_name);
fprintf(' selection locked                      = %d\n',CAT.selection_locked);
fprintf(' configuration frozen                  = %d\n',CAT.configuration_frozen);
fprintf(' selected callback installed           = %d\n',CAT.selected_callback_installed);
fprintf(' expected current investigation        = 1 / BEZIER_D15_C7\n');

assert(CAT.locked_code==1, ...
    '[AF7R10V2] This audit is intentionally backend-1 only.');

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells(member(member(A,'reference'),'result'));
N=numel(R);
assert(N>0,'[AF7R10V2] no reference log.');

if isfield(D,'t') && numel(D.t)>=N
    T=double(D.t(1:N)); T=T(:);
else
    T=(0:N-1).';
end

req=series(R,'state.generator_request_count');
status=series(R,'state.generator_last_status_code');
ttc=series(R,'state.trigger_ttc');
coll=series(R,'state.trigger_collision_time');
obs=series(R,'state.trigger_obstacle_id');
comp=series(R,'state.trigger_component_code');

before=series(R,'state.generator_last_elapsed_before_ms');
budget=series(R,'state.generator_last_budget_remaining_ms');
ctx=series(R,'state.generator_last_context_ms');
cb=series(R,'state.generator_last_callback_ms');
val=series(R,'state.generator_last_validation_ms');
crit=series(R,'state.generator_last_critical_ms');
cycle=series(R,'state.generator_last_cycle_critical_ms');
evt=series(R,'state.trigger_event_capture_ms');

reason=series(R,'state.bezier_shadow_last_reason_code');
generated=series(R,'state.bezier_shadow_last_generated');
t0=series(R,'state.bezier_shadow_last_t0');
tm=series(R,'state.bezier_shadow_last_tm');
t1=series(R,'state.bezier_shadow_last_t1');

% Exact field template fixes the old "dissimilar structures" bug.
emptyAttempt=struct( ...
    'index',NaN, ...
    'request_count',NaN, ...
    't',NaN, ...
    'ttc',NaN, ...
    'collision_time',NaN, ...
    'obstacle',NaN, ...
    'component',NaN, ...
    'status',NaN, ...
    'status_text','', ...
    'elapsed_before_ms',NaN, ...
    'budget_remaining_ms',NaN, ...
    'event_capture_ms',NaN, ...
    'context_ms',NaN, ...
    'callback_ms',NaN, ...
    'validation_ms',NaN, ...
    'critical_ms',NaN, ...
    'cycle_ms',NaN, ...
    'generated',NaN, ...
    'reason',NaN, ...
    't0',NaN, ...
    'tm',NaN, ...
    't1',NaN);
ATT=repmat(emptyAttempt,1,0);

fprintf('\n[B. EVERY GENERATOR REQUEST]\n');
fprintf([' #   t[s]   TTC   obs comp  status/status-text                  | ' ...
         'before  budget  eventCap  context  callback  validation  cycle | gen reason\n']);

prev=-inf;
ia=0;
for k=1:N
    if ~isfinite(req(k)), continue; end

    % New request whenever the cumulative request count increases.
    if req(k)>prev
        if prev==-inf
            % logger may begin after request count has already been zero.
            if req(k)<=0
                prev=req(k);
                continue;
            end
        end

        ia=ia+1;
        a=emptyAttempt;
        a.index=ia;
        a.request_count=req(k);
        a.t=T(k);
        a.ttc=ttc(k);
        a.collision_time=coll(k);
        a.obstacle=obs(k);
        a.component=comp(k);
        a.status=status(k);
        a.status_text=status_text(status(k));
        a.elapsed_before_ms=before(k);
        a.budget_remaining_ms=budget(k);
        a.event_capture_ms=evt(k);
        a.context_ms=ctx(k);
        a.callback_ms=cb(k);
        a.validation_ms=val(k);
        a.critical_ms=crit(k);
        a.cycle_ms=cycle(k);
        a.generated=generated(k);
        a.reason=reason(k);
        a.t0=t0(k);
        a.tm=tm(k);
        a.t1=t1(k);

        ATT(end+1)=a; %#ok<AGROW>

        fprintf('%2d %7.3f %5.3f %4.0f %4.0f %6.0f %-28s | %7.3f %7.3f %9.3f %8.3f %9.3f %10.3f %7.3f | %3.0f %6.0f\n', ...
            a.index,a.t,a.ttc,a.obstacle,a.component,a.status,a.status_text, ...
            a.elapsed_before_ms,a.budget_remaining_ms,a.event_capture_ms, ...
            a.context_ms,a.callback_ms,a.validation_ms,a.cycle_ms, ...
            a.generated,a.reason);

        prev=req(k);
    end
end

fprintf('\n[C. FIRST POSITIVE-LEAD REQUEST]\n');
if isempty(ATT)
    ipos=[];
else
    ipos=find([ATT.ttc]>0 & isfinite([ATT.ttc]),1,'first');
end

FIRST=emptyAttempt;
if isempty(ipos)
    fprintf(' positive-lead request                  = NONE\n');
else
    FIRST=ATT(ipos);
    fprintf(' t / TTC [s]                           = %.6f / %.6f\n',FIRST.t,FIRST.ttc);
    fprintf(' obstacle / component                  = %.0f / %.0f\n',FIRST.obstacle,FIRST.component);
    fprintf(' status                                = %.0f (%s)\n',FIRST.status,FIRST.status_text);
    fprintf(' pre-generator elapsed [ms]            = %.6f\n',FIRST.elapsed_before_ms);
    fprintf(' generator budget remaining [ms]       = %.6f\n',FIRST.budget_remaining_ms);
    fprintf(' trigger-event capture [ms]            = %.6f\n',FIRST.event_capture_ms);
    fprintf(' context / callback / validation [ms]  = %.6f / %.6f / %.6f\n', ...
        FIRST.context_ms,FIRST.callback_ms,FIRST.validation_ms);
    fprintf(' full hard-path cycle [ms]             = %.6f\n',FIRST.cycle_ms);
    fprintf(' generated / proposal reason           = %.0f / %.0f\n', ...
        FIRST.generated,FIRST.reason);
end

fprintf('\n[D. SOURCE SEMANTICS OF CURRENT STATUS]\n');
src=which('REPLANNING_SYSTEM');
txt='';
if ~isempty(src) && isfile(src)
    txt=fileread(src);
end

hasSilentCatch=contains(txt,'catch') && ...
    contains(txt,'callback_ok=false') && ...
    contains(txt,'generator_last_status_code=-1');
hasPreGate=contains(txt,'if budget_remaining<=0') && ...
    contains(txt,'generator_last_status_code=-2');

fprintf(' status -2 source meaning              = PRE_GENERATOR_DEADLINE_EXHAUSTED\n');
fprintf(' status -1 source meaning              = CALLBACK_EXCEPTION_CAUGHT\n');
fprintf(' callback exception currently saved    = %d\n',0);
fprintf(' silent callback catch found           = %d\n',hasSilentCatch);
fprintf(' pre-generator deadline gate found     = %d\n',hasPreGate);

fprintf('\n[E. CAUSE CLASSIFICATION]\n');
CLASS='NO_POSITIVE_LEAD_REQUEST';
NEXT='CHECK_TRIGGER';

if ~isempty(ipos)
    if FIRST.status==-2
        CLASS='PRE_GENERATOR_PATH_EXHAUSTED_4MS_BEFORE_BACKEND1';
        NEXT='MEASURE_PRE_GENERATOR_STAGES_ONLY';
    elseif FIRST.status==-1
        CLASS='BACKEND1_ENTERED__CALLBACK_THROWN_EXCEPTION_CAUGHT';
        NEXT='CAPTURE_CAUGHT_EXCEPTION_WITH_DEBUG_TRAP';
    elseif FIRST.status==1 || FIRST.status==-7
        CLASS='BACKEND1_RETURNED_EMPTY_CANDIDATE';
        NEXT='INSPECT_BACKEND_RETURN_BRANCH';
    elseif FIRST.status==21
        CLASS='BACKEND1_RETURNED_PROPOSAL__REFERENCE_CERTIFICATE_FAILED';
        NEXT='INSPECT_PROPOSAL_REASON_AND_CERTIFICATE';
    elseif FIRST.status==20 || FIRST.status==-20 || FIRST.status==30 || ...
            FIRST.status==-30 || FIRST.status==31 || FIRST.status==33
        CLASS='BACKEND1_GENERATED_REFERENCE_PATH_REACHED_POST_CALLBACK_LOGIC';
        NEXT='INSPECT_REFERENCE_CERT_AND_COMMIT_GATE';
    else
        CLASS='BACKEND1_PATH_REACHED__STATUS_REQUIRES_SOURCE_MAPPING';
        NEXT='INSPECT_STATUS';
    end
end

fprintf(' classification                         = %s\n',CLASS);
fprintf(' next diagnostic                        = %s\n',NEXT);

if strcmp(CLASS,'BACKEND1_ENTERED__CALLBACK_THROWN_EXCEPTION_CAUGHT')
    fprintf('\n[F. IMPORTANT LIMIT OF THIS OLD-RUN LOG]\n');
    fprintf([' The exact MATLAB exception message/line is NOT recoverable from this completed run,\n' ...
             ' because the production callback wrapper catches the exception without storing ME.\n']);
    fprintf([' Do NOT modify the avoidance mathematics yet. Run the separate caught-error trap once;\n' ...
             ' MATLAB will stop at the exact original throw site inside backend-1.\n']);
end

OUT=struct();
OUT.schema='AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_V2';
OUT.backend=CAT;
OUT.attempts=ATT;
OUT.first_positive_lead=FIRST;
OUT.classification=CLASS;
OUT.next_diagnostic=NEXT;

fprintf('====================================================================================================\n');
assignin('base','AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_V2_OUT',OUT);
end

function txt=status_text(code)
if ~isfinite(code)
    txt='NO_STATUS';
    return;
end
switch round(code)
    case -30, txt='REF_COMMIT_DEADLINE_FAIL';
    case -20, txt='BUILTIN_PROPOSAL_DEADLINE_FAIL';
    case -7,  txt='EMPTY_CANDIDATE_DEADLINE_FAIL';
    case -6,  txt='GENERIC_VALIDATION_DEADLINE_FAIL';
    case -5,  txt='GENERIC_VALIDATION_FAIL';
    case -3,  txt='MALFORMED_GENERIC_CANDIDATE';
    case -2,  txt='PRE_GENERATOR_DEADLINE_EXHAUSTED';
    case -1,  txt='CALLBACK_EXCEPTION_CAUGHT';
    case 0,   txt='NO_REQUEST';
    case 1,   txt='EMPTY_CANDIDATE';
    case 4,   txt='GENERIC_VALID_SHADOW_ONLY';
    case 20,  txt='BUILTIN_PROPOSAL_AUTO_COMMIT_OFF';
    case 21,  txt='REFERENCE_PROPOSAL_NOT_CERTIFIED';
    case 30,  txt='REFERENCE_ONLY_COMMIT_SUCCESS';
    case 31,  txt='REFERENCE_ONLY_COMMIT_FAIL';
    case 32,  txt='REFERENCE_LAYER_DISABLED';
    case 33,  txt='REFERENCE_CERT_NOT_CLOSED';
    case 40,  txt='EXECUTION_CERT_OPEN';
    case 41,  txt='EXEC_CERT_CLOSED_BACKUP_OPEN';
    case 42,  txt='EXECUTION_LAYER_DISABLED';
    otherwise
        txt=sprintf('STATUS_%g',code);
end
end

function x=member(S,name)
x=[];
try
    if isempty(S), return; end
    if isstruct(S)
        if isfield(S,name), x=S.(name); end
    elseif isobject(S)
        if isprop(S,name), x=S.(name); end
    end
catch
end
end

function C=result_cells(X)
if isempty(X)
    C={};
elseif iscell(X)
    C=X(:);
else
    C=num2cell(X(:));
end
end

function x=nested(S,path)
x=S;
P=strsplit(path,'.');
for i=1:numel(P)
    x=member(x,P{i});
    if isempty(x), return; end
end
end

function v=series(C,path)
v=nan(numel(C),1);
for k=1:numel(C)
    x=nested(C{k},path);
    try
        if (isnumeric(x)||islogical(x))&&isscalar(x)
            v(k)=double(x);
        end
    catch
    end
end
end
