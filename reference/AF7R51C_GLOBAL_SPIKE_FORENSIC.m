function OUT = AF7R51C_GLOBAL_SPIKE_FORENSIC(runtime_profile,logger)
% Read-only global spike classification from timings already recorded in the
% finished run.  Adds no online instrumentation.
%
% It answers whether the large full-iteration spikes (e.g. ~25 s, ~39 s)
% are caused by:
%   online/control/reference,
%   numerical plant,
%   logger,
%   or outer-loop residual.
%
% Request edges are aligned from the recorded generator_request_count.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R51C | GLOBAL FULL-ITERATION SPIKE FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | existingRuntimeProfileOnly=1 | addedOnlineTimers=0\n');

P=runtime_profile;
reqFields={'t','online_before_plant_s','simulation_plant_s','logger_s','full_iteration_s','phase'};
for i=1:numel(reqFields)
    assert(isfield(P,reqFields{i}),'[AF7R51C] runtime_profile.%s missing.',reqFields{i});
end
K=min(double(P.k),numel(P.t));
t=double(P.t(1:K));
online=1e3*double(P.online_before_plant_s(1:K));
plant=1e3*double(P.simulation_plant_s(1:K));
logms=1e3*double(P.logger_s(1:K));
full=1e3*double(P.full_iteration_s(1:K));
phase=P.phase(1:K);
residual=full-online-plant-logms;

% Align request count/status from logger by nearest exact simulation time.
[D_t,reqCount,status]=extract_request_series(logger);
reqAt=nan(K,1); statAt=nan(K,1);
for k=1:K
    [d,j]=min(abs(D_t-t(k)));
    if d<=1e-9*max(1,abs(t(k)))+1e-12
        reqAt(k)=reqCount(j);
        statAt(k)=status(j);
    end
end

edge=false(K,1); prev=NaN;
for k=1:K
    if isfinite(reqAt(k))
        if ~isfinite(prev)
            prev=reqAt(k);
        elseif reqAt(k)>prev
            edge(k)=true;
            prev=reqAt(k);
        end
    end
end
ff=find(isfinite(reqAt),1,'first');
if ~isempty(ff)&&reqAt(ff)>0,edge(ff)=true;end

flight=(phase=='f');
idx=find(flight & isfinite(full));
[~,ord]=sort(full(idx),'descend');
N=min(20,numel(ord));
top=idx(ord(1:N));

fprintf('\n[A. TOP FLIGHT FULL-ITERATION SPIKES]\n');
fprintf(' #   t[s]   full  online plant logger residual reqEdge req stat   dominant [ms]\n');
for q=1:N
    k=top(q);
    vals=[online(k),plant(k),logms(k),residual(k)];
    names={'ONLINE','PLANT','LOGGER','OUTER_RESIDUAL'};
    vals2=vals; vals2(~isfinite(vals2))=-Inf;
    [~,m]=max(vals2);
    fprintf('%2d %7.3f %7.2f %7.2f %5.2f %6.2f %8.2f %7d %3.0f %4.0f   %s\n', ...
        q,t(k),full(k),online(k),plant(k),logms(k),residual(k), ...
        edge(k),reqAt(k),statAt(k),names{m});
end

fprintf('\n[B. THRESHOLD COUNTS — FLIGHT]\n');
fprintf(' full >=25 ms                             = %d\n',nnz(flight & full>=25));
fprintf(' full >=40 ms                             = %d\n',nnz(flight & full>=40));
fprintf(' full >=100 ms                            = %d\n',nnz(flight & full>=100));
fprintf(' online >=20 ms                           = %d\n',nnz(flight & online>=20));
fprintf(' online >=25 ms                           = %d\n',nnz(flight & online>=25));
fprintf(' plant >=10 ms                            = %d\n',nnz(flight & plant>=10));
fprintf(' logger >=10 ms                           = %d\n',nnz(flight & logms>=10));
fprintf(' outer residual >=10 ms                   = %d\n',nnz(flight & residual>=10));

% Specifically classify the two visually obvious non-request regions.
windows={ ...
    '24.8--25.2 s',24.8,25.2; ...
    '38.8--39.3 s',38.8,39.3; ...
    '9.80--10.30 s',9.8,10.3};
fprintf('\n[C. NAMED WINDOWS]\n');
for i=1:size(windows,1)
    m=flight & t>=windows{i,2} & t<=windows{i,3};
    fprintf(' %s\n',windows{i,1});
    stats('full',full(m));
    stats('online',online(m));
    stats('plant',plant(m));
    stats('logger',logms(m));
    stats('outer residual',residual(m));
    fprintf('   request edges                         = %d\n',nnz(edge&m));
end

% Dominance of excess over the steady baseline.
base=flight & t>=8.5 & t<9.75;
baseMed=[medianfinite(online(base)),medianfinite(plant(base)), ...
         medianfinite(logms(base)),medianfinite(residual(base))];

fprintf('\n[D. TOP-SPIKE EXCESS ABOVE BASELINE]\n');
fprintf(' #   t[s] onlineEx plantEx loggerEx residualEx  dominant excess\n');
for q=1:N
    k=top(q);
    ex=[online(k),plant(k),logms(k),residual(k)]-baseMed;
    names={'ONLINE','PLANT','LOGGER','OUTER_RESIDUAL'};
    ex2=ex; ex2(~isfinite(ex2))=-Inf;
    [mx,m]=max(ex2);
    fprintf('%2d %7.3f %8.2f %7.2f %8.2f %10.2f  %-14s %7.2f\n', ...
        q,t(k),ex(1),ex(2),ex(3),ex(4),names{m},mx);
end

fprintf('\n[E. INTERPRETATION CONTRACT]\n');
fprintf(' post-run dashboard/forensic cost inside run = 0 ms\n');
fprintf(' request-edge == 0 means                    = not a new replanner request\n');
fprintf(' large full with small online/plant/logger   = outer/scheduler/system-side spike\n');
fprintf(' large full with large online                = online pipeline spike\n');
fprintf(' large online at request edge                = candidate/commit path remains relevant\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R51C_GLOBAL_SPIKE_FORENSIC_V1';
OUT.t=t; OUT.full_ms=full; OUT.online_ms=online; OUT.plant_ms=plant;
OUT.logger_ms=logms; OUT.outer_residual_ms=residual;
OUT.request_edge=edge; OUT.request_count=reqAt; OUT.status=statAt;
OUT.top_indices=top;
assignin('base','AF7R51C_GLOBAL_SPIKE_FORENSIC_OUT',OUT);
end

function [t,req,status]=extract_request_series(logger)
D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
t=double(D.t(:)); n=numel(t); req=nan(n,1); status=nan(n,1);
C={};
if isstruct(A)&&isfield(A,'reference')&&isstruct(A.reference)&&isfield(A.reference,'result')
    R=A.reference.result; if iscell(R),C=R(:);else,C=num2cell(R(:));end
end
for k=1:min(n,numel(C))
    st=get_member(C{k},'state');
    req(k)=scalar(get_member(st,'generator_request_count'));
    status(k)=scalar(get_member(st,'generator_last_status_code'));
end
end

function x=get_member(a,n)
x=[];try
if isstruct(a)&&isfield(a,n),x=a.(n);elseif isobject(a)&&isprop(a,n),x=a.(n);end
catch,end
end

function y=scalar(x)
y=NaN;if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end
end

function stats(name,x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),fprintf('   %-36s = NaN\n',name);
else,fprintf('   %-36s min/med/max = %.6g / %.6g / %.6g ms\n',name,min(x),median(x),max(x));end
end

function y=medianfinite(x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),y=NaN;else,y=median(x);end
end
