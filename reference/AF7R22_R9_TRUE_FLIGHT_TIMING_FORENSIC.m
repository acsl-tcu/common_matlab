function OUT = AF7R22_R9_TRUE_FLIGHT_TIMING_FORENSIC(logger)
% AF7R22_R9_TRUE_FLIGHT_TIMING_FORENSIC
% =========================================================================
% Read-only extraction of true-Flight generator timing at every request edge.
%
% Purpose:
%   Separate context construction from backend callback time and total hard
%   cycle time.  In the current source, status -20 is decided immediately
%   after generator_backend_locked_callback(CTX) returns, before the later
%   reference/execution-aware read-only certificate evaluation.  Therefore
%   this audit identifies the actual online generator bottleneck without
%   changing any certificate.
%
% Usage after run_flight_headless:
%   T22 = AF7R22_R9_TRUE_FLIGHT_TIMING_FORENSIC(logger);
% =========================================================================

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R22 | R9 TRUE-FLIGHT GENERATOR TIMING FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | hardDeadlineInterpretation=1 | sampledSafetyAcceptance=0\n');

assert(nargin>=1 && ~isempty(logger),'[AF7R22] logger required.');

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells_local(member_local(member_local(A,'reference'),'result'));
N=numel(R);
assert(N>0,'[AF7R22] no reference result history.');

if isfield(D,'t') && numel(D.t)>=N
    tt=double(D.t(1:N)); tt=tt(:);
else
    tt=(0:N-1).';
end

req=series_local(R,'state.generator_request_count');
status=series_local(R,'state.generator_last_status_code');
reason=series_local(R,'state.bezier_shadow_last_reason_code');
contextMs=series_local(R,'state.generator_last_context_ms');
callbackMs=series_local(R,'state.generator_last_callback_ms');
runtimeMs=series_local(R,'state.generator_last_runtime_ms');
criticalMs=series_local(R,'state.generator_last_critical_ms');
cycleMs=series_local(R,'state.generator_last_cycle_critical_ms');
elapsedBeforeMs=series_local(R,'state.generator_last_elapsed_before_ms');
budgetRemainMs=series_local(R,'state.generator_last_budget_remaining_ms');
genCount=series_local(R,'state.bezier_shadow_generated_count');

ATT=repmat(struct( ...
    'index',NaN,'t',NaN,'request_count',NaN,'status',NaN,'reason',NaN, ...
    'generated',NaN,'context_ms',NaN,'callback_ms',NaN,'runtime_ms',NaN, ...
    'critical_ms',NaN,'cycle_ms',NaN,'elapsed_before_ms',NaN, ...
    'budget_remaining_ms',NaN),1,0);

prevReq=0;
prevGen=0;
for k=1:N
    if ~isfinite(req(k)), continue; end
    if req(k)>prevReq
        a=struct();
        a.index=numel(ATT)+1;
        a.t=tt(k);
        a.request_count=req(k);
        a.status=status(k);
        a.reason=reason(k);
        if isfinite(genCount(k))
            a.generated=double(genCount(k)>prevGen);
            prevGen=max(prevGen,genCount(k));
        else
            a.generated=NaN;
        end
        a.context_ms=contextMs(k);
        a.callback_ms=callbackMs(k);
        a.runtime_ms=runtimeMs(k);
        a.critical_ms=criticalMs(k);
        a.cycle_ms=cycleMs(k);
        a.elapsed_before_ms=elapsedBeforeMs(k);
        a.budget_remaining_ms=budgetRemainMs(k);
        ATT(end+1)=a; %#ok<AGROW>
        prevReq=req(k);
    end
end

fprintf(' #   t[s]    status reason gen   context   callback    critical      cycle   budgetRem [ms]\n');
for i=1:numel(ATT)
    a=ATT(i);
    fprintf('%2d %7.3f %7.0f %6.0f %3.0f %9.4f %10.4f %11.4f %10.4f %12.4f\n', ...
        i,a.t,a.status,a.reason,a.generated, ...
        a.context_ms,a.callback_ms,a.critical_ms,a.cycle_ms,a.budget_remaining_ms);
end

deadline=4.0;
validCb=[ATT.callback_ms]; validCb=validCb(isfinite(validCb));
validCtx=[ATT.context_ms]; validCtx=validCtx(isfinite(validCtx));
validCycle=[ATT.cycle_ms]; validCycle=validCycle(isfinite(validCycle));

deadlineIdx=find([ATT.status]==-20);
generatedIdx=find([ATT.generated]==1);

fprintf('\n[SUMMARY]\n');
fprintf(' request edges                            = %d\n',numel(ATT));
fprintf(' generated proposal edges                 = %d\n',numel(generatedIdx));
fprintf(' status -20 deadline edges                = %d\n',numel(deadlineIdx));
fprintf(' hard deadline [ms]                       = %.6g\n',deadline);
fprintf(' context min / med / max [ms]             = %.6g / %.6g / %.6g\n', ...
    fmin(validCtx),fmed(validCtx),fmax(validCtx));
fprintf(' callback min / med / max [ms]            = %.6g / %.6g / %.6g\n', ...
    fmin(validCb),fmed(validCb),fmax(validCb));
fprintf(' cycle min / med / max [ms]               = %.6g / %.6g / %.6g\n', ...
    fmin(validCycle),fmed(validCycle),fmax(validCycle));

if ~isempty(deadlineIdx)
    cb=[ATT(deadlineIdx).callback_ms];
    cy=[ATT(deadlineIdx).cycle_ms];
    fprintf(' -20 callback min / med / max [ms]        = %.6g / %.6g / %.6g\n', ...
        fmin(cb),fmed(cb),fmax(cb));
    fprintf(' -20 cycle min / med / max [ms]           = %.6g / %.6g / %.6g\n', ...
        fmin(cy),fmed(cy),fmax(cy));
end

% Source-order interpretation. This is not measured inference; it reflects
% the production control-flow contract being audited.
fprintf('\n[CONTROL-FLOW INTERPRETATION]\n');
fprintf(' status -20 decision point                = immediately after backend callback\n');
fprintf(' reference/execution read-only eval       = occurs after deadlineExceeded is latched\n');
fprintf(' therefore -20 root timing layer          = context + backend callback, not later read-only eval\n');

OUT=struct();
OUT.schema='AF7R22_R9_TRUE_FLIGHT_TIMING_FORENSIC_V1';
OUT.read_only=true;
OUT.hard_deadline_ms=deadline;
OUT.attempts=ATT;
OUT.generated_attempt_count=numel(generatedIdx);
OUT.deadline_attempt_count=numel(deadlineIdx);

assignin('base','AF7R22_R9_TRUE_FLIGHT_TIMING_FORENSIC_OUT',OUT);
fprintf('====================================================================================================\n');
end


function x=member_local(S,name)
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


function C=result_cells_local(X)
if isempty(X)
    C={};
elseif iscell(X)
    C=X(:);
elseif isstruct(X)||isobject(X)
    C=num2cell(X(:));
else
    C={};
end
end


function x=nested_local(S,path)
x=S;
P=strsplit(path,'.');
for k=1:numel(P)
    x=member_local(x,P{k});
    if isempty(x), return; end
end
end


function v=series_local(C,path)
v=nan(numel(C),1);
for k=1:numel(C)
    x=nested_local(C{k},path);
    try
        if (isnumeric(x)||islogical(x))&&isscalar(x)
            v(k)=double(x);
        end
    catch
    end
end
end


function x=fmin(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=min(v); end
end


function x=fmax(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=max(v); end
end


function x=fmed(v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v), x=NaN; else, x=median(v); end
end
