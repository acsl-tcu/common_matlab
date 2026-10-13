function OUT = G27V73R6_POSTRUN_RUNTIME_ANALYSIS()
% G27V73R6_POSTRUN_RUNTIME_ANALYSIS
% =========================================================================
% Post-run runtime analysis for the current REPLANNING_SYSTEM direct rt_* log.
%
% Run AFTER one normal simulation, while `logger` remains in base workspace.
%
% This audit:
%   1) reads LOGGER-safe direct scalar timings,
%   2) analyzes only cycles where rt_total_ms is finite,
%   3) decomposes runtime into:
%        live binding
%        dependency check
%        base reference
%        sensor
%        future-reference packet
%        trigger
%        residual inside rt_total_ms
%   4) separates MEX time from trigger time,
%   5) correlates dependency spikes with exact loadmass changes,
%   6) lists the largest runtime cycles and trigger cycles.
%
% IMPORTANT:
%   - e2e_trigger_proxy_ms = rt_live_binding_ms + rt_dependency_ms + rt_total_ms
%   - This is a CURRENT trigger-path diagnostic only.
%   - It is NOT the future complete sensor->trajectory-modification->commit WCET.
%   - Measured p99/max are diagnostics, not formal WCET proof.
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' G27-V73R6 | POST-RUN RUNTIME ANALYSIS\n');
fprintf('====================================================================================================\n');

assert(evalin('base','exist(''logger'',''var'')')==1, ...
    '[G27-V73R6 ERROR] base-workspace logger not found.');

logger=evalin('base','logger');
D=getmember(logger,'Data');
assert(~isempty(D),'[G27-V73R6 ERROR] logger.Data unavailable.');

A=getmember(D,'agent');
assert(~isempty(A),'[G27-V73R6 ERROR] logger.Data.agent unavailable.');
if numel(A)>1
    A=A(1);
end

REF=getmember(A,'reference');
assert(~isempty(REF),'[G27-V73R6 ERROR] logger.Data.agent.reference unavailable.');

C=getmember(REF,'result');
assert(~isempty(C),'[G27-V73R6 ERROR] logger.Data.agent.reference.result unavailable.');
if ~iscell(C)
    C=num2cell(C);
end
C=C(:);

t=getmember(D,'t');
if isempty(t)
    t=(0:numel(C)-1)'*0.025;
else
    t=double(t(:));
end

N=min(numel(C),numel(t));
C=C(1:N);
t=t(1:N);

names={ ...
    'rt_live_binding_ms', ...
    'rt_dependency_ms', ...
    'rt_base_ref_ms', ...
    'rt_sensor_ms', ...
    'rt_reference_packet_ms', ...
    'rt_kernel_ms', ...
    'rt_trigger_ms', ...
    'rt_total_ms'};

X=nan(N,numel(names));
need=false(N,1);
ttc=nan(N,1);
mL=nan(N,1);
minDist=nan(N,1);
kernelCalled=false(N,1);
refSource=strings(N,1);
activeMode=strings(N,1);

for k=1:N
    rr=C{k};
    if isempty(rr)
        continue;
    end

    st=getmember(rr,'state');
    if isempty(st)
        continue;
    end

    for j=1:numel(names)
        [v,ok]=scalarvalue(getmember(st,names{j}));
        if ok
            X(k,j)=v;
        end
    end

    [v,ok]=scalarvalue(getmember(st,'need_replan'));
    if ok
        need(k)=logical(v);
    end

    [v,ok]=scalarvalue(getmember(st,'trigger_ttc'));
    if ok
        ttc(k)=v;
    end

    [v,ok]=scalarvalue(getmember(st,'loadmass'));
    if ok
        mL(k)=v;
    end

    [v,ok]=scalarvalue(getmember(st,'min_dist_point'));
    if ok
        minDist(k)=v;
    end

    [v,ok]=scalarvalue(getmember(st,'trigger_kernel_called'));
    if ok
        kernelCalled(k)=logical(v);
    end

    s=getmember(st,'trigger_reference_source');
    if ischar(s) || (isstring(s) && isscalar(s))
        refSource(k)=string(s);
    end

    s=getmember(st,'active_reference_mode');
    if ischar(s) || (isstring(s) && isscalar(s))
        activeMode(k)=string(s);
    end
end

jLive=col(names,'rt_live_binding_ms');
jDep =col(names,'rt_dependency_ms');
jBase=col(names,'rt_base_ref_ms');
jSen =col(names,'rt_sensor_ms');
jPkt =col(names,'rt_reference_packet_ms');
jMex =col(names,'rt_kernel_ms');
jTrig=col(names,'rt_trigger_ms');
jTot =col(names,'rt_total_ms');

live=X(:,jLive);
dep =X(:,jDep);
base=X(:,jBase);
sensor=X(:,jSen);
packet=X(:,jPkt);
mex=X(:,jMex);
trigger=X(:,jTrig);
total=X(:,jTot);

% Direct timing mirrors are only populated on the protected Flight path.
flight=isfinite(total);
idx=find(flight);

fprintf('[A. LOGGER / FLIGHT COVERAGE]\n');
fprintf(' total logger samples            = %d\n',N);
fprintf(' samples with finite rt_total_ms = %d\n',numel(idx));
assert(~isempty(idx), ...
    ['[G27-V73R6 ERROR] No finite rt_total_ms samples. ' ...
     'Check that the modified REPLANNING_SYSTEM was active during this run.']);

% -------------------------------------------------------------------------
% Derived quantities.
% -------------------------------------------------------------------------
live0=zero_if_nan(live);
dep0=zero_if_nan(dep);
base0=zero_if_nan(base);
sensor0=zero_if_nan(sensor);
packet0=zero_if_nan(packet);
trigger0=zero_if_nan(trigger);
mex0=zero_if_nan(mex);

e2e=live0+dep0+total;

% rt_total starts after live-binding/dependency and includes base/sensor/
% packet/trigger plus other current-cycle work.
residual=total-base0-sensor0-packet0-trigger0;

% trigger contains MEX + decode/other trigger work.
triggerNonMex=trigger-mex0;

% Exact mass-change semantics matching current source (~=).
massChanged=false(N,1);
prev=NaN;
for k=1:N
    if isfinite(mL(k))
        if isfinite(prev)
            massChanged(k)=(mL(k)~=prev);
        end
        prev=mL(k);
    end
end

dmL=nan(N,1);
last=NaN;
for k=1:N
    if isfinite(mL(k))
        if isfinite(last)
            dmL(k)=mL(k)-last;
        end
        last=mL(k);
    end
end

fprintf('\n[B. TIMING SUMMARY -- DIAGNOSTIC, NOT WCET]\n');
print_stats('live binding',live(idx));
print_stats('dependency',dep(idx));
print_stats('base reference',base(idx));
print_stats('sensor',sensor(idx));
print_stats('reference packet',packet(idx));
print_stats('MEX kernel',mex(idx));
print_stats('trigger total',trigger(idx));
print_stats('trigger non-MEX',triggerNonMex(idx));
print_stats('rt_total',total(idx));
print_stats('residual inside rt_total',residual(idx));
print_stats('e2e trigger proxy',e2e(idx));

fprintf('\n current project hard modification budget = 4.000000 ms\n');
fprintf(' e2e trigger proxy >4 ms observed         = %d / %d\n', ...
    nnz(e2e(idx)>4.0),numel(idx));
fprintf(' measured timing used as formal WCET      = 0\n');
fprintf(' current complete trajectory modification included = 0\n');

% -------------------------------------------------------------------------
% Reference source behavior.
% -------------------------------------------------------------------------
fprintf('\n[C. REFERENCE PACKET SOURCE]\n');
src=refSource(idx);
u=unique(src);
for i=1:numel(u)
    if strlength(u(i))==0
        continue;
    end
    ii=idx(refSource(idx)==u(i));
    fprintf(' %-42s : %d cycles | packet max %.6f ms\n', ...
        char(u(i)),numel(ii),max(packet(ii),[],'omitnan'));
end
if all(strlength(src)==0)
    fprintf(' trigger_reference_source was not available in LOGGER.\n');
end

% -------------------------------------------------------------------------
% Mass / dependency correlation.
% -------------------------------------------------------------------------
fprintf('\n[D. LOAD-MASS CHANGE vs DEPENDENCY TIMING]\n');
im=idx(isfinite(mL(idx)));
fprintf(' finite flight loadmass samples = %d\n',numel(im));

if ~isempty(im)
    fprintf(' first mL = %.17g kg\n',mL(im(1)));
    fprintf(' last  mL = %.17g kg\n',mL(im(end)));
    fprintf(' exact mL-change cycles in Flight = %d / %d\n', ...
        nnz(massChanged(idx)),numel(idx));

    q=abs(dmL(idx));
    q=q(isfinite(q));
    if ~isempty(q)
        fprintf(' |delta mL| max = %.17g kg\n',max(q));
        fprintf(' |delta mL| nonzero count = %d\n',nnz(q~=0));
    end

    changedIdx=idx(massChanged(idx));
    stableIdx=idx(~massChanged(idx));

    if ~isempty(changedIdx)
        fprintf(' dependency timing WHEN mL changed:\n');
        print_stats('  dependency',dep(changedIdx));
    end
    if ~isempty(stableIdx)
        fprintf(' dependency timing WHEN mL unchanged:\n');
        print_stats('  dependency',dep(stableIdx));
    end
else
    fprintf(' loadmass was not available in LOGGER.\n');
end

% -------------------------------------------------------------------------
% Top cycles.
% -------------------------------------------------------------------------
fprintf('\n[E. TOP 25 E2E TRIGGER-PATH CYCLES]\n');
[~,ord]=sort(e2e(idx),'descend','MissingPlacement','last');
K=min(25,numel(ord));

REC=repmat(struct(),K,1);

for r=1:K
    k=idx(ord(r));
    cause=classify_cycle( ...
        live(k),dep(k),base(k),sensor(k),packet(k),trigger(k),residual(k));

    REC(r).sample=k;
    REC(r).time=t(k);
    REC(r).e2e_trigger_proxy_ms=e2e(k);
    REC(r).rt_total_ms=total(k);
    REC(r).live_binding_ms=live(k);
    REC(r).dependency_ms=dep(k);
    REC(r).base_ref_ms=base(k);
    REC(r).sensor_ms=sensor(k);
    REC(r).packet_ms=packet(k);
    REC(r).kernel_ms=mex(k);
    REC(r).trigger_ms=trigger(k);
    REC(r).residual_ms=residual(k);
    REC(r).mass_changed=massChanged(k);
    REC(r).loadmass=mL(k);
    REC(r).need_replan=need(k);
    REC(r).ttc=ttc(k);
    REC(r).reference_source=char(refSource(k));
    REC(r).cause=cause;

    fprintf([' #%02d t=%8.3f | e2e=%8.4f total=%8.4f | ' ...
        'bind=%7.4f dep=%7.4f base=%7.4f sensor=%7.4f packet=%7.4f ' ...
        'trig=%7.4f mex=%7.4f residual=%7.4f | ' ...
        'mLchg=%d mL=%.17g | replan=%d TTC=%g | CAUSE=%s\n'], ...
        r,t(k),e2e(k),total(k),live(k),dep(k),base(k),sensor(k),packet(k), ...
        trigger(k),mex(k),residual(k),massChanged(k),mL(k), ...
        need(k),ttc(k),cause);
end

fprintf('\n[F. TOP 15 DEPENDENCY CYCLES]\n');
validDep=idx(isfinite(dep(idx)));
if isempty(validDep)
    fprintf(' no finite dependency timing samples.\n');
else
    [~,od]=sort(dep(validDep),'descend');
    KD=min(15,numel(od));
    for r=1:KD
        k=validDep(od(r));
        fprintf([' #%02d t=%8.3f | dependency=%8.4f ms | ' ...
            'mLchg=%d mL=%.17g delta_mL=% .17g | e2e=%8.4f ms\n'], ...
            r,t(k),dep(k),massChanged(k),mL(k),dmL(k),e2e(k));
    end
end

fprintf('\n[G. TOP 15 REFERENCE-PACKET CYCLES]\n');
validPkt=idx(isfinite(packet(idx)));
if isempty(validPkt)
    fprintf(' no finite packet timing samples.\n');
else
    [~,op]=sort(packet(validPkt),'descend');
    KP=min(15,numel(op));
    for r=1:KP
        k=validPkt(op(r));
        fprintf(' #%02d t=%8.3f | packet=%8.4f ms | source=%s | e2e=%8.4f ms\n', ...
            r,t(k),packet(k),char(refSource(k)),e2e(k));
    end
end

fprintf('\n[H. TRIGGER / COLLISION OBSERVATION]\n');
it=idx(need(idx));
fprintf(' need_replan cycles = %d\n',numel(it));
if ~isempty(it)
    fprintf(' first trigger: t=%.6f s | TTC=%.6f s | minDist=%g\n', ...
        t(it(1)),ttc(it(1)),minDist(it(1)));
    fprintf(' last trigger : t=%.6f s | TTC=%.6f s | minDist=%g\n', ...
        t(it(end)),ttc(it(end)),minDist(it(end)));
end

ik=idx(kernelCalled(idx));
fprintf(' kernel-called cycles = %d / %d\n',numel(ik),numel(idx));
if ~isempty(ik)
    print_stats('MEX on called cycles',mex(ik));
end

fprintf('\n[I. AUTOMATIC INTERPRETATION]\n');
[~,kmaxRel]=max(e2e(idx));
kmax=idx(kmaxRel);
maxCause=classify_cycle( ...
    live(kmax),dep(kmax),base(kmax),sensor(kmax),packet(kmax), ...
    trigger(kmax),residual(kmax));

fprintf(' worst observed cycle t=%.6f s | e2e=%.6f ms | dominant=%s\n', ...
    t(kmax),e2e(kmax),maxCause);

if nnz(massChanged(idx))>0
    depChanged=dep(idx(massChanged(idx)));
    depStable=dep(idx(~massChanged(idx)));
    medC=median(depChanged(isfinite(depChanged)));
    medS=median(depStable(isfinite(depStable)));
    if ~isempty(medC) && ~isempty(medS) && isfinite(medC) && isfinite(medS)
        fprintf(' median dependency: mL-change=%.6f ms | stable=%.6f ms\n', ...
            medC,medS);
        if medC>2*max(medS,eps)
            fprintf([' interpretation: exact estimator mL changes are strongly associated ' ...
                'with dependency-check cost.\n']);
        else
            fprintf([' interpretation: mL changes are present but are not by themselves ' ...
                'a >2x median dependency-cost explanation.\n']);
        end
    end
else
    fprintf(' interpretation: no exact flight-cycle mL changes were observed.\n');
end

fprintf('\n----------------------------------------------------------------------------------------------------\n');
fprintf('[G27-V73R6] Send sections [B] through [I].\n');
fprintf('[G27-V73R6] Do NOT change planner architecture until the dominant measured source is identified.\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.time=t;
OUT.flight_idx=idx;
OUT.names=names;
OUT.timing=X;
OUT.e2e_trigger_proxy_ms=e2e;
OUT.residual_ms=residual;
OUT.trigger_non_mex_ms=triggerNonMex;
OUT.loadmass=mL;
OUT.mass_changed=massChanged;
OUT.delta_loadmass=dmL;
OUT.need_replan=need;
OUT.trigger_ttc=ttc;
OUT.reference_source=refSource;
OUT.top_cycles=REC;
OUT.measured_timing_is_wcet=false;
OUT.complete_trajectory_modification_included=false;

assignin('base','G27V73R6_OUT',OUT);
end


function y=zero_if_nan(x)
y=x;
y(~isfinite(y))=0;
end


function c=classify_cycle(bind,dep,base,sensor,packet,trigger,residual)
v=[bind,dep,base,sensor,packet,trigger,residual];
v(~isfinite(v))=-Inf;
names={'LIVE_BINDING','DEPENDENCY','BASE_REFERENCE','SENSOR', ...
       'REFERENCE_PACKET','TRIGGER','RESIDUAL_OTHER'};
[m,ii]=max(v);
if isempty(ii) || ~isfinite(m)
    c='UNKNOWN';
else
    c=names{ii};
end
end


function print_stats(name,x)
x=x(isfinite(x));
if isempty(x)
    fprintf(' %-28s : no finite samples\n',name);
    return;
end
x=sort(x(:));
fprintf(' %-28s : p50 %.6f | p95 %.6f | p99 %.6f | max %.6f ms\n', ...
    name,pct(x,50),pct(x,95),pct(x,99),max(x));
end


function q=pct(x,p)
if isempty(x)
    q=NaN;
    return;
end
if numel(x)==1
    q=x(1);
    return;
end
r=1+(numel(x)-1)*p/100;
i=floor(r);
j=ceil(r);
if i==j
    q=x(i);
else
    q=x(i)+(r-i)*(x(j)-x(i));
end
end


function j=col(names,name)
j=find(strcmp(names,name),1);
assert(~isempty(j),'[G27-V73R6 INTERNAL] missing timing field %s',name);
end


function [x,ok]=scalarvalue(v)
x=NaN;
ok=false;

for k=1:4
    if iscell(v) && numel(v)==1
        v=v{1};
    else
        break;
    end
end

if (isnumeric(v) || islogical(v)) && isscalar(v)
    d=double(v);
    if isfinite(d)
        x=d;
        ok=true;
    end
end
end


function v=getmember(x,name)
v=[];
try
    for k=1:4
        if iscell(x) && numel(x)==1
            x=x{1};
        else
            break;
        end
    end

    if isstruct(x) && isscalar(x) && isfield(x,name)
        v=x.(name);
    elseif isobject(x) && isscalar(x) && isprop(x,name)
        v=x.(name);
    end
catch
    v=[];
end
end
