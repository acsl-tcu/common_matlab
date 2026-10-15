function OUT = AF7R23A_R9_STAGE_TIMING_AUDIT(logger)
% AF7R23A_R9_STAGE_TIMING_AUDIT
% Read-only extraction of AF7R23 top-level R9 stage timings.

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R23A | R9 TOP-LEVEL STAGE TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | timingOnly=1 | safetyLogicUnchanged=1\n');

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells_local(member_local(member_local(A,'reference'),'result'));
N=numel(R);
assert(N>0,'[AF7R23A] no reference result history.');

if isfield(D,'t')&&numel(D.t)>=N
    tt=double(D.t(1:N)); tt=tt(:);
else
    tt=(0:N-1).';
end

req=series_local(R,'state.generator_request_count');
status=series_local(R,'state.generator_last_status_code');
reason=series_local(R,'state.bezier_shadow_last_reason_code');
ep=series_local(R,'state.r9_stage_episode_ms');
sy=series_local(R,'state.r9_stage_synthesis_ms');
ca=series_local(R,'state.r9_stage_candidate_loop_ms');
to=series_local(R,'state.r9_stage_total_internal_ms');
cb=series_local(R,'state.generator_last_callback_ms');

ATT=repmat(struct('t',NaN,'status',NaN,'reason',NaN, ...
    'episode_ms',NaN,'synthesis_ms',NaN,'candidate_loop_ms',NaN, ...
    'internal_ms',NaN,'callback_ms',NaN),1,0);

prev=0;
for k=1:N
    if isfinite(req(k))&&req(k)>prev
        a=struct('t',tt(k),'status',status(k),'reason',reason(k), ...
            'episode_ms',ep(k),'synthesis_ms',sy(k), ...
            'candidate_loop_ms',ca(k),'internal_ms',to(k), ...
            'callback_ms',cb(k));
        ATT(end+1)=a; %#ok<AGROW>
        prev=req(k);
    end
end

fprintf(' #   t[s]  stat rsn    episode   synthesis   candLoop   internal   callback [ms]\n');
for i=1:numel(ATT)
    a=ATT(i);
    fprintf('%2d %7.3f %5.0f %3.0f %10.4f %11.4f %10.4f %10.4f %10.4f\n', ...
        i,a.t,a.status,a.reason,a.episode_ms,a.synthesis_ms, ...
        a.candidate_loop_ms,a.internal_ms,a.callback_ms);
end

I=find([ATT.status]==-20);
fprintf('\n[-20 SUMMARY]\n');
fprintf(' count                                   = %d\n',numel(I));
if ~isempty(I)
    show('episode', [ATT(I).episode_ms]);
    show('synthesis',[ATT(I).synthesis_ms]);
    show('candidate loop',[ATT(I).candidate_loop_ms]);
    show('internal total',[ATT(I).internal_ms]);
    show('callback',[ATT(I).callback_ms]);
end

OUT=struct('schema','AF7R23A_R9_STAGE_TIMING_AUDIT_V1', ...
    'attempts',ATT,'deadline_indices',I);
assignin('base','AF7R23A_R9_STAGE_TIMING_AUDIT_OUT',OUT);
fprintf('====================================================================================================\n');
end


function show(name,v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v)
    fprintf(' %-22s min/med/max [ms] = NaN / NaN / NaN\n',name);
else
    fprintf(' %-22s min/med/max [ms] = %.6g / %.6g / %.6g\n', ...
        name,min(v),median(v),max(v));
end
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
