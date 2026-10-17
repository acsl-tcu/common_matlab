function OUT = AF7R50C_CLEAN_TIMING_CLOSEOUT(runtime_profile,logger,replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R50C-B | CLEAN TIMING RESULT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | runtimeProfileOuterTiming=1 | lightAuditDisabled=1\n');

D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
t=double(D.t(:)); n=numel(t);
Cr=get_result_cells(A,'reference');
req=nan(n,1); stat=nan(n,1); cb=nan(n,1); hard=nan(n,1);
for k=1:min(n,numel(Cr))
    st=get_member(Cr{k},'state');
    req(k)=v(st,'generator_request_count');
    stat(k)=v(st,'generator_last_status_code');
    cb(k)=v(st,'r9_hr_callback_wall_ms');
    hard(k)=v(st,'generator_last_cycle_critical_ms');
end

P=runtime_profile;
K=min(double(P.k),numel(P.t));
tp=double(P.t(1:K));
online=1e3*double(P.online_before_plant_s(1:K));
plant=1e3*double(P.simulation_plant_s(1:K));
loggerMs=1e3*double(P.logger_s(1:K));
full=1e3*double(P.full_iteration_s(1:K));

% request edges by increments
edge=false(n,1); prev=NaN;
for k=1:n
    if isfinite(req(k))
        if ~isfinite(prev), prev=req(k);
        elseif req(k)>prev, edge(k)=true; prev=req(k);
        end
    end
end
ff=find(isfinite(req),1,'first');
if ~isempty(ff)&&req(ff)>0,edge(ff)=true;end

Wb=tp>=8.5&tp<9.75;
Wr=tp>=9.8&tp<=10.3;
Wp=tp>=10.75&tp<=14;

fprintf('\n[A. OUTER LOOP WINDOWS]\n');
print_stats('baseline online',online(Wb));
print_stats('request online',online(Wr));
print_stats('post online',online(Wp));
print_stats('request plant',plant(Wr));
print_stats('request logger',loggerMs(Wr));
print_stats('request full iteration',full(Wr));
fprintf(' request window online >=20 ms            = %d\n',nnz(Wr & online>=20));
fprintf(' request window online >=25 ms            = %d\n',nnz(Wr & online>=25));
fprintf(' Flight online >=20 ms                    = %d\n',nnz(tp>=5 & tp<47 & online>=20));
fprintf(' Flight online >=25 ms                    = %d\n',nnz(tp>=5 & tp<47 & online>=25));

fprintf('\n[B. TRUE REQUEST EDGES]\n');
ii=find(edge);
fprintf(' # req t[s] status callbackQPC hard [ms]\n');
for q=1:numel(ii)
    k=ii(q);
    fprintf('%2d %3.0f %6.3f %6.0f %10.4f %9.4f\n', ...
        q,req(k),t(k),stat(k),cb(k),hard(k));
end
if ~isempty(ii)
    k=ii(1);
    fprintf(' first request status                     = %.0f\n',stat(k));
    fprintf(' first request callback QPC [ms]          = %.6g\n',cb(k));
    fprintf(' first request hard cycle [ms]            = %.6g\n',hard(k));
end

fprintf('\n[C. DIAGNOSTIC SWITCH CONFIRMATION]\n');
fprintf(' ENABLE_LIGHT_RUNTIME_AUDIT               = %d\n',logical(replanner.ENABLE_LIGHT_RUNTIME_AUDIT));
fprintf(' CAPTURE_REAL_TRIGGER_EVENT               = %d\n',logical(replanner.CAPTURE_REAL_TRIGGER_EVENT));
fprintf(' ENABLE_PROPOSAL_DIAGNOSTIC_MIRROR        = %d\n',logical(replanner.ENABLE_PROPOSAL_DIAGNOSTIC_MIRROR));
fprintf(' observer hard support enabled            = %d\n',logical(replanner.ENABLE_OBSERVER_HARD_SUPPORT));
fprintf(' active reference mode/version            = %s / %.0f\n', ...
    char(replanner.active_reference_mode),double(replanner.active_reference_version));
fprintf(' reference-only commit count              = %.0f\n',double(replanner.reference_only_commit_count));
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R50C_CLEAN_TIMING_CLOSEOUT_V1', ...
    't',tp,'online_ms',online,'plant_ms',plant,'logger_ms',loggerMs,'full_ms',full, ...
    'request_edge',edge,'request_count',req,'status',stat,'callback_qpc_ms',cb,'hard_ms',hard);
assignin('base','AF7R50C_CLEAN_TIMING_OUT',OUT);
end

function C=get_result_cells(A,name)
C={}; if ~isstruct(A)||~isfield(A,name),return;end
S=A.(name); if ~isstruct(S)||~isfield(S,'result'),return;end
R=S.result; if iscell(R),C=R(:);else,C=num2cell(R(:));end
end
function x=get_member(a,n)
x=[]; try
if isstruct(a)&&isfield(a,n),x=a.(n);elseif isobject(a)&&isprop(a,n),x=a.(n);end
catch,end
end
function y=v(st,n)
y=NaN; x=get_member(st,n);
if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end
end
function print_stats(name,x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),fprintf(' %-38s = NaN\n',name);
else,fprintf(' %-38s min/med/max = %.6g / %.6g / %.6g ms\n',name,min(x),median(x),max(x));end
end
