function OUT = AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT(logger,replanner)
% AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT
% Read-only diagnosis of backend-1 (BEZIER_D15_C7) request path.
%
% Usage after the existing run:
%   AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT
%
% No sampled safety acceptance.  This is timing/control-flow diagnosis only.

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R10 | BACKEND-1 FAILURE CAUSE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | backend1Only=1 | timingDiagnosis=1 | safetyPromotion=0\n');

if nargin<1 || isempty(logger)
    assert(evalin('base','exist(''logger'',''var'')==1'),'[AF7R10] logger missing.');
    logger=evalin('base','logger');
end
if nargin<2 || isempty(replanner)
    assert(evalin('base','exist(''replanner_instance'',''var'')==1'), ...
        '[AF7R10] replanner_instance missing.');
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
    '[AF7R10] This audit is intentionally backend-1 only.');

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells(member(member(A,'reference'),'result'));
N=numel(R);
assert(N>0,'[AF7R10] no reference log.');

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

fprintf('\n[B. EVERY GENERATOR REQUEST]\n');
fprintf([' #   t[s]   TTC   obs comp  status | before  budget  eventCap  context  callback  validation  cycle | ' ...
         'gen reason  t0/tm/t1\n']);

ATT=repmat(struct(),1,0);
prev=0;
ia=0;
for k=1:N
    if ~isfinite(req(k)), continue; end
    if req(k)>prev
        ia=ia+1;
        fprintf('%2d %7.3f %5.3f %4.0f %4.0f %7.0f | %7.3f %7.3f %9.3f %8.3f %9.3f %10.3f %7.3f | %3.0f %6.0f  %.3f/%.3f/%.3f\n', ...
            ia,T(k),ttc(k),obs(k),comp(k),status(k),before(k),budget(k),evt(k),ctx(k),cb(k),val(k),cycle(k), ...
            generated(k),reason(k),t0(k),tm(k),t1(k));

        a=struct();
        a.t=T(k); a.ttc=ttc(k); a.obstacle=obs(k); a.component=comp(k);
        a.status=status(k); a.elapsed_before_ms=before(k);
        a.budget_remaining_ms=budget(k); a.event_capture_ms=evt(k);
        a.context_ms=ctx(k); a.callback_ms=cb(k); a.validation_ms=val(k);
        a.cycle_ms=cycle(k); a.generated=generated(k); a.reason=reason(k);
        a.t0=t0(k); a.tm=tm(k); a.t1=t1(k);
        ATT(end+1)=a; %#ok<AGROW>
        prev=req(k);
    end
end

fprintf('\n[C. FIRST POSITIVE-LEAD REQUEST]\n');
ipos=find([ATT.ttc]>0 & isfinite([ATT.ttc]),1,'first');
if isempty(ipos)
    fprintf(' positive-lead request                  = NONE\n');
    FIRST=struct();
else
    FIRST=ATT(ipos);
    fprintf(' t / TTC [s]                           = %.6f / %.6f\n',FIRST.t,FIRST.ttc);
    fprintf(' status                                = %.0f\n',FIRST.status);
    fprintf(' pre-generator elapsed [ms]            = %.6f\n',FIRST.elapsed_before_ms);
    fprintf(' generator budget remaining [ms]       = %.6f\n',FIRST.budget_remaining_ms);
    fprintf(' trigger-event capture [ms]            = %.6f\n',FIRST.event_capture_ms);
    fprintf(' context / callback / validation [ms]  = %.6f / %.6f / %.6f\n', ...
        FIRST.context_ms,FIRST.callback_ms,FIRST.validation_ms);
    fprintf(' full hard-path cycle [ms]             = %.6f\n',FIRST.cycle_ms);

    if FIRST.status==-2
        fprintf(' diagnosis                              = PRE_GENERATOR_BUDGET_EXHAUSTED\n');
        fprintf(' backend-1 callback actually executed   = 0\n');
    elseif FIRST.callback_ms>0
        fprintf(' backend-1 callback actually executed   = 1\n');
    end
end

fprintf('\n[D. CONTROL-FLOW DIAGNOSIS]\n');
src=which('REPLANNING_SYSTEM');
txt='';
if ~isempty(src) && isfile(src), txt=fileread(src); end

preBudgetGate=contains(txt,'if budget_remaining<=0') && ...
    contains(txt,'obj.generator_last_status_code=-2');
edgeLatchedAfterFailure=contains(txt,'obj.replan_request_active=true;') && ...
    contains(txt,'obj.replan_request_reference_version=requestVersion;');
captureBeforeInvoke=false;
if ~isempty(txt)
    p1=strfind(txt,'obj.latch_real_trigger_event(');
    p2=strfind(txt,'obj.invoke_replan_generator_shadow(');
    if ~isempty(p1)&&~isempty(p2)
        captureBeforeInvoke=p1(1)<p2(1);
    end
end

fprintf(' budget gate before callback            = %d\n',preBudgetGate);
fprintf(' real-event capture before invoke       = %d\n',captureBeforeInvoke);
fprintf(' request marked active after -2 path    = %d\n',edgeLatchedAfterFailure);
fprintf(' full real request-edge path prewarmed  = %d\n',0);

fprintf('\n[E. CAUSE CLASSIFICATION]\n');
C1=false; C2=false; C3=false; C4=false;
if ~isempty(ipos)
    C1=(FIRST.status==-2);
    if C1 && isfinite(FIRST.event_capture_ms) && FIRST.event_capture_ms>0
        C2=true;
    end
end
late=sum([ATT.ttc]<=0 & isfinite([ATT.ttc]));
C3=late>0;
C4=CAT.locked_code==1;

fprintf(' C1 first useful request died before backend-1 = %d\n',C1);
fprintf(' C2 first-edge event capture on hard path      = %d\n',C2);
fprintf(' C3 later requests are zero-TTC                = %d [%d attempts]\n',C3,late);
fprintf(' C4 only backend-1 selected                    = %d\n',C4);

if C1
    CLASS='INTEGRATION_DEADLINE_GATE_BLOCKS_BACKEND1_BEFORE_IT_IS_TESTED';
elseif ~isempty(ipos) && FIRST.generated==0
    CLASS='BACKEND1_EXECUTED_BUT_PROPOSAL_GENERATION_FAILED';
elseif ~isempty(ipos) && FIRST.generated==1 && FIRST.reason~=0
    CLASS='BACKEND1_GENERATED_BUT_REFERENCE_CERTIFICATE_FAILED';
else
    CLASS='BACKEND1_PATH_NEEDS_FURTHER_RUNTIME_INSPECTION';
end
fprintf(' classification                         = %s\n',CLASS);

fprintf('\n[F. NEXT FIX ORDER]\n');
fprintf(' 1) warm the REAL request-edge/invoke path before Flight (not kernel-only).\n');
fprintf(' 2) do not let diagnostic trigger-event capture consume the 4-ms trajectory-modification budget.\n');
fprintf(' 3) a transient -2 must remain retry-eligible while TTC>0; do not mark it permanently handled.\n');
fprintf(' 4) only then judge BEZIER_D15_C7 generation/certification.\n');
fprintf(' 5) do not activate backends 2..4 until backend-1 normal-case path is understood.\n');

OUT=struct();
OUT.schema='AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_V1';
OUT.backend=CAT;
OUT.attempts=ATT;
OUT.first_positive_lead=FIRST;
OUT.classification=CLASS;
fprintf('====================================================================================================\n');
assignin('base','AF7R10_BACKEND1_FAILURE_CAUSE_AUDIT_OUT',OUT);
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
if isempty(X), C={};
elseif iscell(X), C=X(:);
else, C=num2cell(X(:));
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
        if (isnumeric(x)||islogical(x))&&isscalar(x), v(k)=double(x); end
    catch
    end
end
end
