function OUT = G27V73R7R1_POSTRUN_RESIDUAL_AUDIT()
% G27V73R7_POSTRUN_RESIDUAL_AUDIT
% Fine-grained decomposition of the remaining rt_total residual.
% Measured timing is diagnostic only, not WCET proof.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' G27-V73R7R1 | RESIDUAL RUNTIME SPLIT AUDIT\n');
fprintf('====================================================================================================\n');

assert(evalin('base','exist(''logger'',''var'')')==1,'[G27-V73R7R1 ERROR] logger not found.');
L=evalin('base','logger');
D=member(L,'Data');
A=member(D,'agent'); if numel(A)>1, A=A(1); end
C=member(member(A,'reference'),'result');
if ~iscell(C), C=num2cell(C); end
C=C(:);

t=member(D,'t');
if isempty(t), t=(0:numel(C)-1)'*0.025; else, t=double(t(:)); end
N=min(numel(C),numel(t)); C=C(1:N); t=t(1:N);

F={ ...
'rt_live_binding_ms','rt_dependency_ms','rt_base_ref_ms', ...
'rt_cert_guard_ms','rt_geometry_eval_ms','rt_sensor_ms', ...
'rt_reference_packet_ms','rt_kernel_ms','rt_trigger_ms', ...
'rt_replan_status_ms','rt_output_write_ms', ...
'rt_state_trigger_write_ms','rt_state_cert_write_ms','rt_state_meta_write_ms', ...
'rt_total_ms'};

X=nan(N,numel(F)); need=false(N,1); ttc=nan(N,1);

for k=1:N
    st=member(C{k},'state');
    if isempty(st), continue; end
    for j=1:numel(F)
        [v,ok]=scalar(member(st,F{j}));
        if ok, X(k,j)=v; end
    end
    [v,ok]=scalar(member(st,'need_replan')); if ok, need(k)=logical(v); end
    [v,ok]=scalar(member(st,'trigger_ttc')); if ok, ttc(k)=v; end
end

v=@(n) X(:,col(F,n));
total=v('rt_total_ms');
idx=find(isfinite(total));
assert(~isempty(idx),'[G27-V73R7R1 ERROR] no finite rt_total_ms.');

parts={ ...
'rt_base_ref_ms', ...
'rt_cert_guard_ms', ...
'rt_geometry_eval_ms', ...
'rt_sensor_ms', ...
'rt_reference_packet_ms', ...
'rt_trigger_ms', ...
'rt_replan_status_ms', ...
'rt_output_write_ms', ...
'rt_state_trigger_write_ms', ...
'rt_state_cert_write_ms', ...
'rt_state_meta_write_ms'};

sumKnown=zeros(N,1);
for j=1:numel(parts)
    x=v(parts{j}); x(~isfinite(x))=0; sumKnown=sumKnown+x;
end
residual2=total-sumKnown;

live=v('rt_live_binding_ms'); live(~isfinite(live))=0;
dep=v('rt_dependency_ms'); dep(~isfinite(dep))=0;
e2e=live+dep+total;

fprintf('[A. SUMMARY -- DIAGNOSTIC, NOT WCET]\n');
for j=1:numel(parts)
    xj=v(parts{j});
    stats(parts{j},xj(idx));
end
stats('rt_total_ms',total(idx));
stats('residual_after_R7_split',residual2(idx));
stats('e2e_trigger_proxy_ms',e2e(idx));

fprintf('\n[B. TOP 25 E2E CYCLES]\n');
[~,ord]=sort(e2e(idx),'descend');
K=min(25,numel(ord));
REC=repmat(struct( ...
    'time',NaN, ...
    'e2e_ms',NaN, ...
    'total_ms',NaN, ...
    'residual2_ms',NaN, ...
    'cause',''),K,1);

for r=1:K
    k=idx(ord(r));
    vals=zeros(1,numel(parts)+1);
    labs=[parts {'RESIDUAL_AFTER_SPLIT'}];
    for j=1:numel(parts), vals(j)=getv(X,F,parts{j},k); end %#ok<AGROW>
    vals(end)=residual2(k);
    vals(~isfinite(vals))=-Inf;
    [~,ii]=max(vals);
    cause=labs{ii};

    fprintf([' #%02d t=%8.3f | e2e=%9.4f total=%9.4f | ' ...
        'cert=%7.4f geom=%7.4f sensor=%7.4f packet=%7.4f trig=%7.4f ' ...
        'status=%7.4f output=%7.4f stTrig=%7.4f stCert=%7.4f stMeta=%7.4f ' ...
        'res2=%9.4f | replan=%d TTC=%g | CAUSE=%s\n'], ...
        r,t(k),e2e(k),total(k), ...
        getv(X,F,'rt_cert_guard_ms',k),getv(X,F,'rt_geometry_eval_ms',k), ...
        getv(X,F,'rt_sensor_ms',k),getv(X,F,'rt_reference_packet_ms',k),getv(X,F,'rt_trigger_ms',k), ...
        getv(X,F,'rt_replan_status_ms',k),getv(X,F,'rt_output_write_ms',k), ...
        getv(X,F,'rt_state_trigger_write_ms',k),getv(X,F,'rt_state_cert_write_ms',k), ...
        getv(X,F,'rt_state_meta_write_ms',k),residual2(k),need(k),ttc(k),cause);

    REC(r)=struct('time',t(k),'e2e_ms',e2e(k),'total_ms',total(k), ...
        'residual2_ms',residual2(k),'cause',cause);
end

fprintf('\n[C. TOP 15 STATE-CERTIFICATE WRITE CYCLES]\n');
x=v('rt_state_cert_write_ms'); valid=idx(isfinite(x(idx)));
[~,o]=sort(x(valid),'descend');
for r=1:min(15,numel(o))
    k=valid(o(r));
    fprintf(' #%02d t=%8.3f | stateCert=%9.4f ms | total=%9.4f | e2e=%9.4f\n', ...
        r,t(k),x(k),total(k),e2e(k));
end

fprintf('\n[D. TOP 15 STATE-METADATA WRITE CYCLES]\n');
x=v('rt_state_meta_write_ms'); valid=idx(isfinite(x(idx)));
[~,o]=sort(x(valid),'descend');
for r=1:min(15,numel(o))
    k=valid(o(r));
    fprintf(' #%02d t=%8.3f | stateMeta=%9.4f ms | total=%9.4f | e2e=%9.4f\n', ...
        r,t(k),x(k),total(k),e2e(k));
end

fprintf('\n[E. TOP 15 CERT-GUARD / GEOMETRY CYCLES]\n');
cg=v('rt_cert_guard_ms'); gg=v('rt_geometry_eval_ms');
score=max([cg gg],[],2);
valid=idx(isfinite(score(idx)));
[~,o]=sort(score(valid),'descend');
for r=1:min(15,numel(o))
    k=valid(o(r));
    fprintf(' #%02d t=%8.3f | cert=%8.4f geom=%8.4f | total=%9.4f | e2e=%9.4f\n', ...
        r,t(k),cg(k),gg(k),total(k),e2e(k));
end

fprintf('\n----------------------------------------------------------------------------------------------------\n');
fprintf('[G27-V73R7R1] Send sections [A]-[E].\n');
fprintf('[G27-V73R7R1] No runtime architecture change is justified until this split identifies the 90-ms path.\n');
fprintf('====================================================================================================\n');

OUT=struct('time',t,'fields',{F},'timing',X,'residual2_ms',residual2, ...
    'e2e_ms',e2e,'top',REC,'measured_timing_is_wcet',false);
assignin('base','G27V73R7R1_OUT',OUT);
end

function stats(n,x)
x=x(isfinite(x));
if isempty(x)
    fprintf(' %-30s : no data\n',n); return;
end
x=sort(x(:));
fprintf(' %-30s : p50 %.6f | p95 %.6f | p99 %.6f | max %.6f ms\n', ...
    n,pct(x,50),pct(x,95),pct(x,99),max(x));
end

function q=pct(x,p)
if numel(x)==1, q=x(1); return; end
r=1+(numel(x)-1)*p/100; i=floor(r); j=ceil(r);
if i==j, q=x(i); else, q=x(i)+(r-i)*(x(j)-x(i)); end
end

function j=col(F,n)
j=find(strcmp(F,n),1); assert(~isempty(j));
end

function [x,ok]=scalar(v)
x=NaN; ok=false;
for k=1:3
    if iscell(v)&&numel(v)==1, v=v{1}; else, break; end
end
if (isnumeric(v)||islogical(v))&&isscalar(v)&&isfinite(double(v))
    x=double(v); ok=true;
end
end


function y=getv(X,F,n,k)
j=col(F,n);
y=X(k,j);
end

function v=member(x,n)
v=[];
try
    for k=1:3
        if iscell(x)&&numel(x)==1, x=x{1}; else, break; end
    end
    if isstruct(x)&&isscalar(x)&&isfield(x,n), v=x.(n);
    elseif isobject(x)&&isscalar(x)&&isprop(x,n), v=x.(n);
    end
catch
    v=[];
end
end
