function report = CBF_ANALYZE_PHYSICAL_QP_FAILURE(snapshotFile)
% CBF_ANALYZE_PHYSICAL_QP_FAILURE.m
% Standalone diagnostic of saved physical-CBF QP feasibility.
% Does NOT change HLC, trajectory, QP, filter or applied input.
% Requires Optimization Toolbox (linprog).
%
% Usage:
%   R = CBF_ANALYZE_PHYSICAL_QP_FAILURE;
%   R = CBF_ANALYZE_PHYSICAL_QP_FAILURE('CBF_physical_QP_failure.mat');
%
% Positive minimum slack for A*w <= b means those exact sampled linear
% constraints cannot all be satisfied. It does NOT prove that collision
% avoidance with an earlier switch, different auxiliary state or different
% architecture is physically impossible.
if nargin < 1 || isempty(snapshotFile)
    snapshotFile = 'CBF_physical_QP_failure.mat';
end
if ~isfile(snapshotFile)
    error('QP snapshot not found: %s', snapshotFile);
end
S = load(snapshotFile);
if ~isfield(S,'D'), error('Snapshot does not contain struct D.'); end
D = S.D;
A = D.Aineq; b = D.bineq(:);
assert(size(A,2) == 4 && size(A,1) == numel(b), 'Invalid saved QP matrices.');
assert(all(isfinite([A(:);b(:)])), 'Nonfinite QP matrices.');
n = size(A,1);
if ~isfield(D,'filterResult') || ~isfield(D.filterResult,'rows')
    error('Snapshot lacks filterResult.rows for grouping.');
end
rows = D.filterResult.rows;
nCBF = numel(rows);
assert(nCBF+8 == n, 'Unexpected constraint order; inspect snapshot before grouping.');
labels = cell(n,1);
for j=1:nCBF
    labels{j} = sprintf('CBF obs=%s %s',char(string(rows(j).obstacleID)),rows(j).component);
end
labels{nCBF+1} = 'thrust lower HOCBF';
labels{nCBF+2} = 'thrust upper HOCBF';
for k=1:3
    labels{nCBF+2+k} = sprintf('torque upper axis %d',k);
    labels{nCBF+5+k} = sprintf('torque lower axis %d',k);
end
opts = optimoptions('linprog','Display','off');
[baseSlack,baseFlag,baseW] = localLP(A,b,opts);
fprintf('\n=== CBF PHYSICAL QP CONFLICT ANALYSIS ===\n');
if isfield(D,'time'), fprintf('Time: %.5f s\n',D.time);end
fprintf('Rows: %d (%d CBF, 2 thrust, 6 torque), decision: 4\n',n,nCBF);
fprintf('All constraints: linprog flag=%d  minimum common slack=%.9g\n',baseFlag,baseSlack);
if baseFlag > 0 && baseSlack > 1e-6
    fprintf('=> Current QP constraints are mutually incompatible at this state.\n');
elseif baseFlag > 0
    fprintf('=> LP finds feasible/nearly feasible inequalities; inspect QP numerics.\n');
else
    fprintf('=> LP conclusion is inconclusive.\n');
end

fprintf('\n--- Constraints tightest at BEST common-slack LP solution ---\n');
if baseFlag > 0 && all(isfinite(baseW))
    residual = A*baseW - b;
    [~,ix] = sort(residual,'descend');
    for k=1:min(12,n)
        j=ix(k);
        fprintf('  %2d: residual=%+.8g  %-34s\n',j,residual(j),labels{j});
    end
else
    residual = nan(n,1);
end

% Remove constraint groups ONE AT A TIME to identify which relaxation
% makes the current QP feasible. This does not propose relaxing safety.
groupNames = {'all UAV CBF','all cable CBF','all load CBF',...
              'all obstacle CBF','thrust HOCBF bounds','torque limits'};
groupSets = {find(strcmp({rows.component},'uav')),...
             find(strcmp({rows.component},'cable')),...
             find(strcmp({rows.component},'load')),...
             (1:nCBF)',(nCBF+1:nCBF+2)',(nCBF+3:n)'};
obsIDs = [rows.obstacleID];
if isnumeric(obsIDs)
    for oid = unique(obsIDs)
        groupNames{end+1} = sprintf('CBF for obstacle %g',oid); %#ok<AGROW>
        groupSets{end+1} = find(obsIDs==oid); %#ok<AGROW>
    end
end
fprintf('\n--- One group removed (DIAGNOSTIC ONLY; never remove safety constraints) ---\n');
groupResults = struct('removedGroup',{},'removedRows',{},'exitflag',{},'minSlack',{});
for k=1:numel(groupNames)
    removeIdx = groupSets{k}(:);
    keep = true(n,1); keep(removeIdx)=false;
    [s,flag] = localLP(A(keep,:),b(keep),opts);
    fprintf('  %-30s  minSlack=%+.9g  flag=%d\n',groupNames{k},s,flag);
    groupResults(end+1) = struct('removedGroup',groupNames{k},...
        'removedRows',removeIdx,'exitflag',flag,'minSlack',s); %#ok<AGROW>
end

fprintf('\n--- Remove each ONE CBF row (smallest remaining slack first) ---\n');
rowSlack = nan(nCBF,1); rowFlag = nan(nCBF,1);
for j=1:nCBF
    keep=true(n,1);keep(j)=false;
    [rowSlack(j),rowFlag(j)]=localLP(A(keep,:),b(keep),opts);
end
[~,ix]=sort(rowSlack);
for k=1:min(9,nCBF)
    j=ix(k);
    fprintf('  remove row %2d %-34s  minSlack=%+.9g  flag=%d\n',...
        j,labels{j},rowSlack(j),rowFlag(j));
end
fprintf('\nInterpretation: a positive minimum slack proves ONLY the saved\n');
fprintf('sampled QP infeasible, not that earlier feasible avoidance is impossible.\n');
fprintf('Never treat the slack-LP solution as a safe control command.\n');
report=struct('snapshotFile',snapshotFile,'time',D.time,'A',A,'b',b,...
    'nCBF',nCBF,'labels',{labels},'minSlack',baseSlack,'exitflag',baseFlag,...
    'bestLPw',baseW,'bestLPResidual',residual,...
    'groupResults',groupResults,'leaveOneCBFOutSlack',rowSlack,...
    'leaveOneCBFOutExitflag',rowFlag);
end

function [minSlack,flag,w]=localLP(A,b,opts)
N=size(A,2);M=size(A,1);
minSlack=nan;flag=-999;w=nan(N,1);
if M==0,minSlack=0;flag=1;w=zeros(N,1);return;end
try
    cost=[zeros(N,1);1];
    AA=[A,-ones(M,1)];
    bb=b(:);
    lb=[-inf(N,1);0];
    [x,~,flag]=linprog(cost,AA,bb,[],[],lb,[],opts);
    if flag>0 && ~isempty(x) && all(isfinite(x))
        minSlack=x(end);w=x(1:N);
    end
catch ME
    fprintf(2,'linprog diagnostic failed: %s\n',ME.message);
end
end
