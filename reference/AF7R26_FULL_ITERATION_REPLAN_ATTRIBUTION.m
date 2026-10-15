function OUT = AF7R26_FULL_ITERATION_REPLAN_ATTRIBUTION(runtime_profile,logger)
% AF7R26_FULL_ITERATION_REPLAN_ATTRIBUTION
% =========================================================================
% Read-only attribution of runtime_profile.full_iteration_s.
%
% full_iteration_s in run_flight_headless measures:
%   sensor acquisition
% + sensor
% + estimator
% + reference/replanner
% + controller
% + input_transform
% + simulation plant
% + logger
%
% It does NOT include the later runtime_profile write, time increment,
% periodic console fprintf, or post-run plotting.
%
% This audit aligns each runtime_profile sample with REPLANNING_SYSTEM's
% request-local R25 QPC telemetry and separates:
%   online-before-plant / plant / logger / small residual
% while identifying whether a generator request occurred on that cycle.
% =========================================================================

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R26 | FULL-ITERATION / REPLAN ATTRIBUTION AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | diagnosticOnly=1 | safetyLogicUnchanged=1\n');

assert(isstruct(runtime_profile) && isfield(runtime_profile,'t'), ...
    '[AF7R26] runtime_profile required.');

t=double(runtime_profile.t(:));
online=1e3*double(runtime_profile.online_before_plant_s(:));
plant=1e3*double(runtime_profile.simulation_plant_s(:));
logms=1e3*double(runtime_profile.logger_s(:));
full=1e3*double(runtime_profile.full_iteration_s(:));
phase=runtime_profile.phase(:);

N=numel(t);
assert(all([numel(online),numel(plant),numel(logms),numel(full),numel(phase)]==N), ...
    '[AF7R26] runtime_profile length mismatch.');

% Logger-side replanner telemetry.
D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells_local(member_local(member_local(A,'reference'),'result'));
NL=numel(R);

if isfield(D,'t') && numel(D.t)>=NL
    tl=double(D.t(1:NL)); tl=tl(:);
else
    tl=nan(NL,1);
end

req=series_local(R,'state.generator_request_count');
status=series_local(R,'state.generator_last_status_code');
reason=series_local(R,'state.bezier_shadow_last_reason_code');
seq=series_local(R,'state.r9_hr_request_seq');
qcb=series_local(R,'state.r9_hr_callback_wall_ms');
qep=series_local(R,'state.r9_hr_episode_wall_ms');
qsy=series_local(R,'state.r9_hr_synthesis_wall_ms');
qqp=series_local(R,'state.r9_hr_qp_wall_ms');

% Request edges in logger time.
edge=false(NL,1);
last=-Inf;
for k=1:NL
    if isfinite(req(k)) && req(k)>last
        if last~=-Inf
            edge(k)=true;
        elseif req(k)>0
            edge(k)=true;
        end
        last=req(k);
    end
end

% Align nearest logger sample. Expected exact equality under headless sim.
reqA=nan(N,1); statusA=nan(N,1); reasonA=nan(N,1);
qcbA=nan(N,1); qepA=nan(N,1); qsyA=nan(N,1); qqpA=nan(N,1);
edgeA=false(N,1); seqOK=false(N,1);
for i=1:N
    if isempty(tl) || all(~isfinite(tl)), continue; end
    [d,j]=min(abs(tl-t(i)));
    if isfinite(d) && d<=1e-9+16*eps(max(1,abs(t(i))))
        reqA(i)=req(j);
        statusA(i)=status(j);
        reasonA(i)=reason(j);
        edgeA(i)=edge(j);
        seqOK(i)=isfinite(seq(j)) && isfinite(req(j)) && seq(j)==req(j);
        if seqOK(i)
            qcbA(i)=qcb(j);
            qepA(i)=qep(j);
            qsyA(i)=qsy(j);
            qqpA(i)=qqp(j);
        end
    end
end

residual=full-online-plant-logms;

fprintf('\n[SEMANTIC CHECK]\n');
fprintf(' full iteration includes                 = online + numerical plant + logger + small bookkeeping between timers\n');
fprintf(' post-full runtime_profile write included= 0\n');
fprintf(' time increment / periodic fprintf incl. = 0\n');
fprintf(' post-run drawing included               = 0\n');

maskF=(phase=='f');
idx=find(maskF & isfinite(full));
assert(~isempty(idx),'[AF7R26] no Flight samples.');

fprintf('\n[FLIGHT SUMMARY]\n');
print_stats('online before plant',online(idx));
print_stats('numerical plant',plant(idx));
print_stats('logger',logms(idx));
print_stats('full iteration',full(idx));
print_stats('timer residual',residual(idx));

% Top cycles.
[~,ordF]=sort(full(idx),'descend');
K=min(15,numel(ordF));
fprintf('\n[TOP FULL-ITERATION CYCLES]\n');
fprintf(' #    t[s]      full    online   plant   logger residual reqEdge stat rsn replQPC epQPC synQPC qpQPC [ms]\n');
for q=1:K
    i=idx(ordF(q));
    fprintf('%2d %8.3f %9.3f %8.3f %7.3f %8.3f %8.3f   %d    %4.0f %3.0f %8.3f %6.3f %7.3f %6.3f\n', ...
        q,t(i),full(i),online(i),plant(i),logms(i),residual(i),edgeA(i), ...
        statusA(i),reasonA(i),qcbA(i),qepA(i),qsyA(i),qqpA(i));
end

% Focus windows requested by current investigation.
print_window('REPLAN WINDOW 9.5--10.75 s',9.5,10.75, ...
    t,full,online,plant,logms,residual,edgeA,statusA,reasonA,qcbA,qepA,qsyA,qqpA,maskF);
print_window('SYSTEM-SIDE WINDOW 24.5--25.5 s',24.5,25.5, ...
    t,full,online,plant,logms,residual,edgeA,statusA,reasonA,qcbA,qepA,qsyA,qqpA,maskF);

% Exact high-level classification of the 10-second feature.
I10=find(maskF & t>=9.5 & t<=10.75);
nReq10=sum(edgeA(I10));
nQpc10=sum(isfinite(qcbA(I10)));
fprintf('\n[10 s FEATURE]\n');
fprintf(' generator request edges in window       = %d\n',nReq10);
fprintf(' request-local QPC callback records       = %d\n',nQpc10);
if nQpc10>0
    v=qcbA(I10); v=v(isfinite(v));
    fprintf(' replanner callback min/med/max [ms]      = %.6g / %.6g / %.6g\n', ...
        min(v),median(v),max(v));
end
fprintf(' interpretation                           = ');
if nReq10>0 && nQpc10>0
    fprintf('REPLAN_RETRY_BURST_CONFIRMED\n');
else
    fprintf('NOT_CONFIRMED_FROM_AVAILABLE_TELEMETRY\n');
end

I25=find(maskF & t>=24.5 & t<=25.5);
fprintf('\n[25 s FEATURE]\n');
fprintf(' generator request edges in window       = %d\n',sum(edgeA(I25)));
fprintf(' request-local QPC callback records       = %d\n',sum(isfinite(qcbA(I25))));
fprintf(' interpretation                           = ');
if sum(edgeA(I25))==0
    fprintf('NOT_R9_GENERATOR_REQUEST\n');
else
    fprintf('REPLAN_ACTIVITY_PRESENT\n');
end

OUT=struct();
OUT.schema='AF7R26_FULL_ITERATION_REPLAN_ATTRIBUTION_V1';
OUT.t=t;
OUT.phase=phase;
OUT.full_ms=full;
OUT.online_ms=online;
OUT.plant_ms=plant;
OUT.logger_ms=logms;
OUT.residual_ms=residual;
OUT.request_edge=edgeA;
OUT.status=statusA;
OUT.reason=reasonA;
OUT.replan_callback_qpc_ms=qcbA;
OUT.episode_qpc_ms=qepA;
OUT.synthesis_qpc_ms=qsyA;
OUT.qp_qpc_ms=qqpA;
OUT.request_seq_aligned=seqOK;
assignin('base','AF7R26_FULL_ITERATION_REPLAN_ATTRIBUTION_OUT',OUT);

fprintf('====================================================================================================\n');
end


function print_window(name,a,b,t,full,online,plant,logms,res,edge,st,rs,qcb,qep,qsy,qqp,mask)
I=find(mask & t>=a & t<=b);
if isempty(I), return; end
[peak,ii]=max(full(I)); ip=I(ii);
fprintf('\n[%s]\n',name);
fprintf(' peak t / full [ms]                       = %.6f / %.6g\n',t(ip),peak);
fprintf(' online / plant / logger / residual [ms] = %.6g / %.6g / %.6g / %.6g\n', ...
    online(ip),plant(ip),logms(ip),res(ip));
fprintf(' request edge / status / reason           = %d / %.0f / %.0f\n', ...
    edge(ip),st(ip),rs(ip));
fprintf(' replanner QPC cb/ep/syn/qp [ms]          = %.6g / %.6g / %.6g / %.6g\n', ...
    qcb(ip),qep(ip),qsy(ip),qqp(ip));
end


function print_stats(name,v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v)
    fprintf(' %-22s min/med/p95/max = NaN / NaN / NaN / NaN ms\n',name);
else
    fprintf(' %-22s min/med/p95/max = %.6g / %.6g / %.6g / %.6g ms\n', ...
        name,min(v),median(v),pct(v,95),max(v));
end
end


function y=pct(x,p)
x=sort(double(x(:))); x=x(isfinite(x));
if isempty(x), y=NaN; return; end
u=1+(numel(x)-1)*p/100; i=floor(u); j=ceil(u);
if i==j, y=x(i); else, y=x(i)+(u-i)*(x(j)-x(i)); end
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
x=S; P=strsplit(path,'.');
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
        if (isnumeric(x)||islogical(x))&&isscalar(x), v(k)=double(x); end
    catch
    end
end
end
