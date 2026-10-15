function OUT = AF7R25A_HIGHRES_TIMING_AUDIT(logger)
% AF7R25A_HIGHRES_TIMING_AUDIT
% =========================================================================
% Read-only request-aligned high-resolution timing audit.
%
% QPC wall time is compared with the legacy tic/toc callback timing.
% Thread/process cycle deltas are reported only as relative work counters.
% Stale records are rejected by request sequence.
% =========================================================================

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R25A | HIGH-RESOLUTION REQUEST-ALIGNED TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | QPCwall=1 | threadProcessCyclesAreWorkOnly=1 | staleRejected=1\n');

assert(~isempty(which('af7r25_stamp_mex')),'[AF7R25A] timer MEX missing.');

% Pair-overhead calibration outside the measured flight.
Ncal=3000;
cal=nan(Ncal,1);
for k=1:Ncal
    a=af7r25_stamp_mex(); b=af7r25_stamp_mex();
    cal(k)=1e3*double(b(1)-a(1))/double(a(2));
end

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells_local(member_local(member_local(A,'reference'),'result'));
N=numel(R);
assert(N>0,'[AF7R25A] no reference result.');

if isfield(D,'t')&&numel(D.t)>=N
    tt=double(D.t(1:N)); tt=tt(:);
else
    tt=(0:N-1).';
end

req=series_local(R,'state.generator_request_count');
seq=series_local(R,'state.r9_hr_request_seq');
status=series_local(R,'state.generator_last_status_code');
reason=series_local(R,'state.bezier_shadow_last_reason_code');
ticCb=series_local(R,'state.generator_last_callback_ms');

qCb=series_local(R,'state.r9_hr_callback_wall_ms');
tCb=series_local(R,'state.r9_hr_callback_thread_cycles');
pCb=series_local(R,'state.r9_hr_callback_process_cycles');
qEp=series_local(R,'state.r9_hr_episode_wall_ms');
tEp=series_local(R,'state.r9_hr_episode_thread_cycles');
pEp=series_local(R,'state.r9_hr_episode_process_cycles');
qSy=series_local(R,'state.r9_hr_synthesis_wall_ms');
tSy=series_local(R,'state.r9_hr_synthesis_thread_cycles');
pSy=series_local(R,'state.r9_hr_synthesis_process_cycles');
qQp=series_local(R,'state.r9_hr_qp_wall_ms');
tQp=series_local(R,'state.r9_hr_qp_thread_cycles');
pQp=series_local(R,'state.r9_hr_qp_process_cycles');

ATT=repmat(struct('request',NaN,'t',NaN,'status',NaN,'reason',NaN, ...
    'seq_ok',false,'tic_callback_ms',NaN,'qpc_callback_ms',NaN, ...
    'episode_ms',NaN,'synthesis_ms',NaN,'qp_ms',NaN, ...
    'callback_thread_cycles',NaN,'callback_process_cycles',NaN, ...
    'episode_thread_cycles',NaN,'episode_process_cycles',NaN, ...
    'synthesis_thread_cycles',NaN,'synthesis_process_cycles',NaN, ...
    'qp_thread_cycles',NaN,'qp_process_cycles',NaN),1,0);

prev=0;
for k=1:N
    if isfinite(req(k))&&req(k)>prev
        seqok=isfinite(seq(k)) && seq(k)==req(k);
        a=struct('request',req(k),'t',tt(k),'status',status(k),'reason',reason(k), ...
            'seq_ok',seqok,'tic_callback_ms',ticCb(k), ...
            'qpc_callback_ms',pick(seqok,qCb(k)), ...
            'episode_ms',pick(seqok,qEp(k)), ...
            'synthesis_ms',pick(seqok,qSy(k)), ...
            'qp_ms',pick(seqok,qQp(k)), ...
            'callback_thread_cycles',pick(seqok,tCb(k)), ...
            'callback_process_cycles',pick(seqok,pCb(k)), ...
            'episode_thread_cycles',pick(seqok,tEp(k)), ...
            'episode_process_cycles',pick(seqok,pEp(k)), ...
            'synthesis_thread_cycles',pick(seqok,tSy(k)), ...
            'synthesis_process_cycles',pick(seqok,pSy(k)), ...
            'qp_thread_cycles',pick(seqok,tQp(k)), ...
            'qp_process_cycles',pick(seqok,pQp(k)));
        ATT(end+1)=a; %#ok<AGROW>
        prev=req(k);
    end
end

fprintf(' QPC stamp-pair min/med/p95/max [ms]     = %.6g / %.6g / %.6g / %.6g\n', ...
    min(cal),median(cal),pct(cal,95),max(cal));
fprintf('\n');
fprintf(' # req   t[s] stat rsn seq  ticCB    qpcCB   episode synthesis      QP [ms]\n');
for i=1:numel(ATT)
    a=ATT(i);
    fprintf('%2d %3.0f %7.3f %4.0f %3.0f  %d %8.3f %8.3f %9.3f %9.3f %8.3f\n', ...
        i,a.request,a.t,a.status,a.reason,a.seq_ok, ...
        a.tic_callback_ms,a.qpc_callback_ms,a.episode_ms,a.synthesis_ms,a.qp_ms);
end

I=find([ATT.status]==-20 & [ATT.seq_ok]);
fprintf('\n[-20 QPC SUMMARY]\n');
summ('callback wall',[ATT(I).qpc_callback_ms]);
summ('episode wall',[ATT(I).episode_ms]);
summ('synthesis wall',[ATT(I).synthesis_ms]);
summ('QP wall',[ATT(I).qp_ms]);

% Compare tic/toc vs QPC on same request.
if ~isempty(I)
    d=[ATT(I).tic_callback_ms]-[ATT(I).qpc_callback_ms];
    d=d(isfinite(d));
    if ~isempty(d)
        fprintf(' ticCB-QPCCB min/med/max [ms]             = %.6g / %.6g / %.6g\n', ...
            min(d),median(d),max(d));
    end
end

% Work-counter attribution at the largest QPC callback event.
if ~isempty(I)
    walls=[ATT(I).qpc_callback_ms];
    [~,jm]=max(walls);
    im=I(jm);
    fprintf('\n[MAX CALLBACK EVENT]\n');
    fprintf(' request / t / wall [ms]                 = %.0f / %.6f / %.6g\n', ...
        ATT(im).request,ATT(im).t,ATT(im).qpc_callback_ms);
    workratio('callback',ATT,I,im,'callback_thread_cycles','callback_process_cycles');
    workratio('episode',ATT,I,im,'episode_thread_cycles','episode_process_cycles');
    workratio('synthesis',ATT,I,im,'synthesis_thread_cycles','synthesis_process_cycles');
    workratio('QP',ATT,I,im,'qp_thread_cycles','qp_process_cycles');
end

stale=sum(~[ATT.seq_ok]);
fprintf('\n[INTEGRITY]\n');
fprintf(' request records                           = %d\n',numel(ATT));
fprintf(' stale/misaligned high-res records rejected= %d\n',stale);
fprintf(' cycle counters converted to time          = 0\n');
fprintf(' Windows/MATLAB hard WCET claimed          = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R25A_HIGHRES_TIMING_AUDIT_V1', ...
    'attempts',ATT,'stamp_pair_overhead_ms',cal, ...
    'deadline_indices',I,'stale_rejected',stale);
assignin('base','AF7R25A_HIGHRES_TIMING_AUDIT_OUT',OUT);
end

function v=pick(ok,x)
if ok, v=x; else, v=NaN; end
end

function workratio(name,A,I,im,tf,pf)
tv=[A(I).(tf)]; pv=[A(I).(pf)];
tm=median(tv(isfinite(tv))); pm=median(pv(isfinite(pv)));
tr=A(im).(tf)/tm; pr=A(im).(pf)/pm;
fprintf(' %-10s work ratio at max thread/process = %.6g / %.6g\n',name,tr,pr);
end

function summ(name,v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v)
    fprintf(' %-18s min/med/max [ms] = NaN / NaN / NaN\n',name);
else
    fprintf(' %-18s min/med/max [ms] = %.6g / %.6g / %.6g\n', ...
        name,min(v),median(v),max(v));
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
