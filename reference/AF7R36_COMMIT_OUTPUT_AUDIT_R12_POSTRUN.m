function S = AF7R36_COMMIT_OUTPUT_AUDIT_R12_POSTRUN()
% AF7R36_COMMIT_OUTPUT_AUDIT_R12_POSTRUN
%
% POSTRUN ONLY -- no source modification and no rerun required.
%
% Uses the existing workspace variables:
%   logger
%   replanner_instance   (optional but strongly recommended)
%   dt                   (optional; inferred from logger time if absent)
%
% Audits the ACTUAL logger-visible reference path used by the simulation:
%   reference.result.state.xd
%   reference.result.state.active_reference_mode
%   reference.result.state.active_reference_version
%
% This distinguishes:
%   A) planner generated a moving candidate but no repaired reference commit
%   B) REPAIRED commit occurred and logger-visible reference moves
%   C) REPAIRED mode occurred but logger-visible reference is effectively static
%
% No safety acceptance is derived from sampled data.

assert(evalin('base','exist(''logger'',''var'')==1'), ...
    '[COMMIT-R12] logger is not available in the base workspace.');

logger=evalin('base','logger');
if evalin('base','exist(''replanner_instance'',''var'')==1')
    replanner=evalin('base','replanner_instance');
else
    replanner=[];
end

D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end

if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    if evalin('base','exist(''dt'',''var'')==1')
        dt=double(evalin('base','dt'));
    else
        dt=0.025;
    end
    n=result_count(A,'reference');
    t=(0:n-1)'*dt;
end

Cref=get_result_cells(A,'reference');
n=min(numel(t),numel(Cref));
t=t(1:n);

xd=nan(n,3);
mode=strings(n,1);
ver=nan(n,1);
need=nan(n,1);
src=strings(n,1);

for k=1:n
    rr=Cref{k};
    st=get_member(rr,'state');

    q=get_member(st,'xd');
    if isnumeric(q) && numel(q)>=3
        q=double(q(:));
        xd(k,:)=q(1:3).';
    end

    q=get_member(st,'active_reference_mode');
    if ischar(q) || isstring(q)
        mode(k)=string(q);
    end

    q=get_member(st,'active_reference_version');
    if isnumeric(q) && isscalar(q) && isfinite(q)
        ver(k)=double(q);
    end

    q=get_member(st,'need_replan');
    if isnumeric(q) || islogical(q)
        if isscalar(q), need(k)=double(q); end
    end

    q=get_member(st,'trigger_reference_source');
    if ischar(q) || isstring(q)
        src(k)=string(q);
    end
end

valid=all(isfinite(xd),2);
flight=(t>=5 & t<47);
rep=strcmpi(mode,'REPAIRED');
nom=strcmpi(mode,'NOMINAL');

idxRep=find(rep & valid);
idxNom=find(nom & valid);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 COMMIT / LOGGER OUTPUT AUTHORITY R12 RESULT\n');
fprintf(' postrun only | no source modification | no rerun\n');
fprintf('====================================================================================================\n');

fprintf('logged reference samples                  = %d\n',n);
fprintf('valid xd samples                          = %d\n',nnz(valid));
fprintf('NOMINAL samples                           = %d\n',nnz(nom));
fprintf('REPAIRED samples                          = %d\n',nnz(rep));
fprintf('max active reference version              = %.0f\n',safe_max(ver));

if ~isempty(idxRep)
    i0=idxRep(1); i1=idxRep(end);
    fprintf('first REPAIRED sample                      = t=%.6f s | version=%.0f\n',t(i0),ver(i0));
    fprintf('last REPAIRED sample                       = t=%.6f s | version=%.0f\n',t(i1),ver(i1));

    R=xd(idxRep,:);
    p0=R(1,:);
    d=R-p0;
    ext3=max(sqrt(sum(d.^2,2)));
    extxy=max(sqrt(sum(d(:,1:2).^2,2)));
    xrange=[min(R(:,1)),max(R(:,1))];
    yrange=[min(R(:,2)),max(R(:,2))];
    zrange=[min(R(:,3)),max(R(:,3))];

    fprintf('\n[ACTUAL LOGGER-VISIBLE REPAIRED REFERENCE]\n');
    fprintf('3D extent from first repaired sample        = %.9g m\n',ext3);
    fprintf('XY extent from first repaired sample        = %.9g m\n',extxy);
    fprintf('x range                                      = [%.9g, %.9g] m\n',xrange(1),xrange(2));
    fprintf('y range                                      = [%.9g, %.9g] m\n',yrange(1),yrange(2));
    fprintf('z range                                      = [%.9g, %.9g] m\n',zrange(1),zrange(2));
    fprintf('first repaired p                             = [% .6f % .6f % .6f]\n',R(1,:));
    fprintf('middle repaired p                            = [% .6f % .6f % .6f]\n',R(round((size(R,1)+1)/2),:));
    fprintf('last repaired p                              = [% .6f % .6f % .6f]\n',R(end,:));

    % Consecutive movement in actual logged output.
    step=sqrt(sum(diff(R,1,1).^2,2));
    fprintf('max consecutive repaired step                = %.9g m\n',safe_max(step));
    fprintf('nonzero repaired steps (>1e-9 m)             = %d / %d\n',nnz(step>1e-9),numel(step));
else
    ext3=NaN;extxy=NaN;
    fprintf('\n[ACTUAL LOGGER-VISIBLE REPAIRED REFERENCE]\n');
    fprintf('No REPAIRED samples were logged.\n');
end

% Version transition summary.
fprintf('\n[REFERENCE AUTHORITY TRANSITIONS]\n');
chg=[true; mode(2:end)~=mode(1:end-1) | ver(2:end)~=ver(1:end-1)];
I=find(chg);
for kk=1:numel(I)
    i=I(kk);
    fprintf('t=%9.4f | mode=%-8s | version=%g | xd=[% .5f % .5f % .5f]\n', ...
        t(i),char(mode(i)),ver(i),xd(i,1),xd(i,2),xd(i,3));
end

% Optional live object metadata.
fprintf('\n[FINAL REPLANNER OBJECT]\n');
if ~isempty(replanner)
    [okMode,vMode]=safe_prop(replanner,'active_reference_mode');
    [okVer,vVer]=safe_prop(replanner,'active_reference_version');
    [okCount,vCount]=safe_prop(replanner,'replan_commit_count');
    [okRO,vRO]=safe_prop(replanner,'reference_only_commit_count');
    [okStatus,vStatus]=safe_prop(replanner,'last_replan_status');

    if okMode, fprintf('active_reference_mode                      = %s\n',char(string(vMode))); end
    if okVer, fprintf('active_reference_version                   = %.0f\n',double(vVer)); end
    if okCount, fprintf('replan_commit_count                        = %.0f\n',double(vCount)); end
    if okRO, fprintf('reference_only_commit_count                = %.0f\n',double(vRO)); end
    if okStatus && isstruct(vStatus)
        if isfield(vStatus,'committed')
            fprintf('last_replan_status.committed               = %d\n',logical(vStatus.committed));
        end
        if isfield(vStatus,'reference_version')
            fprintf('last_replan_status.reference_version       = %.0f\n',double(vStatus.reference_version));
        end
        if isfield(vStatus,'commit_time')
            fprintf('last_replan_status.commit_time              = %.9g s\n',double(vStatus.commit_time));
        end
        if isfield(vStatus,'rejoin_time')
            fprintf('last_replan_status.rejoin_time              = %.9g s\n',double(vStatus.rejoin_time));
        end
        if isfield(vStatus,'safety_mode')
            fprintf('last_replan_status.safety_mode              = %s\n',char(string(vStatus.safety_mode)));
        end
    end
else
    fprintf('replanner_instance unavailable in base workspace.\n');
end

% Decision.
if isempty(idxRep)
    cls='MOVING_PLANNER_CANDIDATE_NOT_COMMITTED_TO_LOGGER_REFERENCE';
elseif isfinite(ext3) && ext3>1e-6 && size(xd(idxRep,:),1)>=2
    R=xd(idxRep,:);
    step=sqrt(sum(diff(R,1,1).^2,2));
    if any(step>1e-9)
        cls='REPAIRED_REFERENCE_IS_COMMITTED_AND_MOVING_IN_ACTUAL_LOGGER_OUTPUT';
    else
        cls='REPAIRED_MODE_LOGGED_BUT_REFERENCE_SAMPLES_ARE_STATIC';
    end
else
    cls='REPAIRED_MODE_PRESENT_BUT_EFFECTIVE_REFERENCE_MOTION_IS_NEGLIGIBLE';
end

fprintf('\n[DECISION]\n');
fprintf('classification                              = %s\n',cls);

if strcmp(cls,'REPAIRED_REFERENCE_IS_COMMITTED_AND_MOVING_IN_ACTUAL_LOGGER_OUTPUT')
    fprintf(['Interpretation: planner -> commit -> reference.result.state.xd is closed.\n' ...
             'If the plotted path still appears not to move, the remaining issue is visualization scale/projection or plant tracking, not reference generation/commit.\n']);
elseif strcmp(cls,'MOVING_PLANNER_CANDIDATE_NOT_COMMITTED_TO_LOGGER_REFERENCE')
    fprintf(['Interpretation: planner candidate exists, but the live reference authority never entered REPAIRED in the logged simulation.\n' ...
             'Next audit should target commit gating/deadline/rollback.\n']);
else
    fprintf(['Interpretation: REPAIRED authority exists but its logger-visible xd is not showing the planned motion.\n' ...
             'Next audit should target eval_fn/batch-model/version binding.\n']);
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_COMMIT_OUTPUT_AUTHORITY_R12';
S.t=t;
S.xd=xd;
S.mode=mode;
S.version=ver;
S.repaired_mask=rep;
S.classification=cls;
S.repaired_extent_3d=ext3;
S.repaired_extent_xy=extxy;
end

function n=result_count(A,name)
n=0;
if isstruct(A) && isfield(A,name)
    X=A.(name);
    if isstruct(X) && isfield(X,'result'), n=numel(X.result); end
end
end

function C=get_result_cells(A,name)
C={};
if ~isstruct(A) || ~isfield(A,name), return; end
X=A.(name);
if ~isstruct(X) || ~isfield(X,'result'), return; end
R=X.result;
if iscell(R), C=R(:); else, C=num2cell(R(:)); end
end

function x=get_member(S,name)
x=[];
try
    if isempty(S), return; end
    if isstruct(S)
        if isfield(S,name), x=S.(name); end
    elseif isobject(S)
        if isprop(S,name), x=S.(name); end
    end
catch
    x=[];
end
end

function [ok,v]=safe_prop(obj,name)
ok=false;v=[];
try
    if isobject(obj) && isprop(obj,name)
        v=obj.(name);ok=true;
    elseif isstruct(obj) && isfield(obj,name)
        v=obj.(name);ok=true;
    end
catch
    ok=false;v=[];
end
end

function v=safe_max(x)
x=x(isfinite(x));
if isempty(x), v=NaN; else, v=max(x); end
end
