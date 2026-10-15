function OUT = AF7R24A_R9_SYNTHESIS_TIMING_AUDIT(logger)
% AF7R24A_R9_SYNTHESIS_TIMING_AUDIT
% Read-only true-Flight synthesis substage report.

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R24A | R9 SYNTHESIS SUBSTAGE TIMING AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | timingOnly=1 | safetyLogicUnchanged=1\n');

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end
R=result_cells_local(member_local(member_local(A,'reference'),'result'));
N=numel(R);
assert(N>0,'[AF7R24A] no reference result.');

if isfield(D,'t')&&numel(D.t)>=N
    tt=double(D.t(1:N)); tt=tt(:);
else
    tt=(0:N-1).';
end

req=series_local(R,'state.generator_request_count');
st=series_local(R,'state.generator_last_status_code');
hs=series_local(R,'state.r9_synth_hessian_ms');
co=series_local(R,'state.r9_synth_corridor_ms');
qp=series_local(R,'state.r9_synth_qp_ms');
cb=series_local(R,'state.r9_synth_candidate_build_ms');
nr=series_local(R,'state.r9_synth_row_count');
ne=series_local(R,'state.r9_synth_episode_count');
nc=series_local(R,'state.r9_synth_component_count');
ns=series_local(R,'state.r9_synth_sign_count');
syn=series_local(R,'state.r9_stage_synthesis_ms');

ATT=repmat(struct('t',NaN,'status',NaN,'hessian_ms',NaN, ...
    'corridor_ms',NaN,'qp_ms',NaN,'candidate_build_ms',NaN, ...
    'synthesis_ms',NaN,'row_count',NaN,'episode_count',NaN, ...
    'component_count',NaN,'sign_count',NaN),1,0);

prev=0;
for k=1:N
    if isfinite(req(k))&&req(k)>prev
        ATT(end+1)=struct('t',tt(k),'status',st(k), ...
            'hessian_ms',hs(k),'corridor_ms',co(k),'qp_ms',qp(k), ...
            'candidate_build_ms',cb(k),'synthesis_ms',syn(k), ...
            'row_count',nr(k),'episode_count',ne(k), ...
            'component_count',nc(k),'sign_count',ns(k)); %#ok<AGROW>
        prev=req(k);
    end
end

fprintf(' #   t[s] stat  Hesse  corridor      QP  candBuild synthesis rows epi cmp signs [ms]\n');
for i=1:numel(ATT)
    a=ATT(i);
    fprintf('%2d %7.3f %4.0f %6.3f %9.3f %7.3f %10.3f %9.3f %4.0f %3.0f %3.0f %5.0f\n', ...
        i,a.t,a.status,a.hessian_ms,a.corridor_ms,a.qp_ms, ...
        a.candidate_build_ms,a.synthesis_ms,a.row_count, ...
        a.episode_count,a.component_count,a.sign_count);
end

I=find([ATT.status]==-20);
fprintf('\n[-20 SUMMARY]\n');
summary('hessian',[ATT(I).hessian_ms]);
summary('corridor rows',[ATT(I).corridor_ms]);
summary('QP enumeration',[ATT(I).qp_ms]);
summary('candidate build',[ATT(I).candidate_build_ms]);
summary('synthesis total',[ATT(I).synthesis_ms]);

OUT=struct('schema','AF7R24A_R9_SYNTHESIS_TIMING_AUDIT_V1', ...
    'attempts',ATT,'deadline_indices',I);
assignin('base','AF7R24A_R9_SYNTHESIS_TIMING_AUDIT_OUT',OUT);
fprintf('====================================================================================================\n');
end


function summary(name,v)
v=double(v(:)); v=v(isfinite(v));
if isempty(v)
    fprintf(' %-18s min/med/max [ms] = NaN / NaN / NaN\n',name);
else
    fprintf(' %-18s min/med/max [ms] = %.6g / %.6g / %.6g\n', ...
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
