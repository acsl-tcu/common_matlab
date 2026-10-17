function OUT = AF7R53A_TIMING_CLOSEOUT(runtime_profile,logger,replanner)
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53A | PREPARED-COMMIT TIMING CLOSEOUT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | noNewOnlineTimers=1 | runtimeProfileOuterTiming=1\n');

D=logger.Data;A=D.agent;if numel(A)>1,A=A(1);end
t=double(D.t(:));n=numel(t);
Cr=get_result_cells(A,'reference');
req=nan(n,1);stat=nan(n,1);cb=nan(n,1);hard=nan(n,1);
for k=1:min(n,numel(Cr))
    st=get_member(Cr{k},'state');
    req(k)=v(st,'generator_request_count');
    stat(k)=v(st,'generator_last_status_code');
    cb(k)=v(st,'r9_hr_callback_wall_ms');
    hard(k)=v(st,'generator_last_cycle_critical_ms');
end
edge=false(n,1);prev=NaN;
for k=1:n
    if isfinite(req(k))
        if ~isfinite(prev),prev=req(k);
        elseif req(k)>prev,edge(k)=true;prev=req(k);
        end
    end
end
ff=find(isfinite(req),1,'first');
if ~isempty(ff)&&req(ff)>0,edge(ff)=true;end

P=runtime_profile;K=min(double(P.k),numel(P.t));
tp=double(P.t(1:K));
online=1e3*double(P.online_before_plant_s(1:K));
plant=1e3*double(P.simulation_plant_s(1:K));
full=1e3*double(P.full_iteration_s(1:K));
phase=P.phase(1:K);

Wr=tp>=9.8&tp<=10.3;
Wf=phase=='f';

fprintf('\n[A. SWITCH / COMMIT]\n');
fprintf(' prepared switch                         = %d\n',logical(replanner.ENABLE_AF7R53_PREPARED_COMMIT));
fprintf(' prepared commit count                   = %.0f\n',double(replanner.af7r53_prepared_commit_count));
fprintf(' legacy commit attempts                  = %.0f\n',double(replanner.af7r53_legacy_commit_count));
fprintf(' reference-only commits                  = %.0f\n',double(replanner.reference_only_commit_count));
fprintf(' active reference mode/version           = %s / %.0f\n', ...
    char(replanner.active_reference_mode),double(replanner.active_reference_version));

fprintf('\n[B. REQUEST WINDOW]\n');
stats('online',online(Wr));stats('plant',plant(Wr));stats('full',full(Wr));
fprintf(' request online >=20 ms                  = %d\n',nnz(Wr&online>=20));
fprintf(' request online >=25 ms                  = %d\n',nnz(Wr&online>=25));
fprintf(' Flight online >=20 ms                   = %d\n',nnz(Wf&online>=20));
fprintf(' Flight online >=25 ms                   = %d\n',nnz(Wf&online>=25));

fprintf('\n[C. TRUE REQUEST EDGES]\n');
ii=find(edge);
fprintf(' # req t[s] status callbackQPC hard [ms]\n');
for q=1:numel(ii)
    k=ii(q);
    fprintf('%2d %3.0f %6.3f %6.0f %10.4f %9.4f\n', ...
        q,req(k),t(k),stat(k),cb(k),hard(k));
end
if ~isempty(ii)
    fprintf(' first request status                    = %.0f\n',stat(ii(1)));
    fprintf(' first request callback QPC [ms]         = %.6g\n',cb(ii(1)));
    fprintf(' first request hard [ms]                 = %.6g\n',hard(ii(1)));
end
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R53A_TIMING_CLOSEOUT_V1','request_edge',edge, ...
    'status',stat,'callback_qpc_ms',cb,'hard_ms',hard,'online_ms',online, ...
    'prepared_switch',logical(replanner.ENABLE_AF7R53_PREPARED_COMMIT), ...
    'prepared_commit_count',double(replanner.af7r53_prepared_commit_count));
assignin('base','AF7R53A_TIMING_CLOSEOUT_OUT',OUT);
end

function stats(nm,x)
x=double(x(:));x=x(isfinite(x));
if isempty(x),fprintf(' %-38s = NaN\n',nm);
else,fprintf(' %-38s min/med/max = %.6g / %.6g / %.6g ms\n',nm,min(x),median(x),max(x));end
end
function C=get_result_cells(A,name)
C={};if ~isstruct(A)||~isfield(A,name),return;end
S=A.(name);if ~isstruct(S)||~isfield(S,'result'),return;end
R=S.result;if iscell(R),C=R(:);else,C=num2cell(R(:));end
end
function x=get_member(a,n)
x=[];try
if isstruct(a)&&isfield(a,n),x=a.(n);elseif isobject(a)&&isprop(a,n),x=a.(n);end
catch,end
end
function y=v(st,n)
y=NaN;x=get_member(st,n);
if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end
end
