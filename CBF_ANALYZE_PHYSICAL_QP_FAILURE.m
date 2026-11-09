function report = CBF_ANALYZE_PHYSICAL_QP_FAILURE(snapshotFile)
% CBF_ANALYZE_PHYSICAL_QP_FAILURE
% READ-ONLY diagnosis of a saved physical S3-CBF QP failure.
% Does NOT alter any controller, EKF, observer, obstacle, trajectory, or QP.
% Requires Optimization Toolbox (linprog).
%
% Usage (in the directory containing CBF_physical_QP_failure.mat):
%   R = CBF_ANALYZE_PHYSICAL_QP_FAILURE;
%   R = CBF_ANALYZE_PHYSICAL_QP_FAILURE('CBF_physical_QP_failure.mat');
%
% Explanatory notes:
%   A*w <= b is the exact saved pointwise QP, with w=[fDDot;Mx;My;Mz].
%   The reported common slack is in mixed/normalized constraint units,
%   NOT physical collision distance, meters, or Newton-meters.
%   An infeasible subsystem (IIS) is deletion-minimal, not necessarily
%   the globally minimum-cardinality conflict set.
%   Farkas weights prove incompatibility of the saved inequalities if
%   y>=0, A'*y=0, and b'*y<0 are verified within numerical tolerance.
%   They do NOT prove physical collision is inevitable.

if nargin<1 || isempty(snapshotFile)
    snapshotFile='CBF_physical_QP_failure.mat';
end
if ~isfile(snapshotFile)
    error('CBF:SnapshotNotFound','QP snapshot not found: %s',snapshotFile);
end
S=load(snapshotFile);
if ~isfield(S,'D') || ~isstruct(S.D)
    error('CBF:SnapshotMalformed','Expected a struct D in snapshot.');
end
D=S.D;
needed={'Aineq','bineq','filterResult','time'};
for k=1:numel(needed)
    if ~isfield(D,needed{k})
        error('CBF:SnapshotMalformed','Snapshot missing D.%s.',needed{k});
    end
end
A=D.Aineq; b=D.bineq(:);
if size(A,2)~=4 || size(A,1)~=numel(b) || ~all(isfinite([A(:);b(:)]))
    error('CBF:SnapshotMalformed','Invalid or nonfinite saved QP matrices.');
end
if ~isfield(D.filterResult,'rows')
    error('CBF:SnapshotMalformed','Snapshot missing D.filterResult.rows.');
end
rows=D.filterResult.rows;
nCBF=numel(rows); n=size(A,1);
if ~ismember(n,[nCBF+8,nCBF+10])
    error('CBF:UnexpectedRows','Expected %d or %d rows (CBF + 8 bounds + optional yaw lock), found %d.',nCBF+8,nCBF+10,n);
end
labels=cell(n,1);
for j=1:nCBF
    labels{j}=sprintf('CBF obs=%s %s',char(string(rows(j).obstacleID)),char(rows(j).component));
end
labels{nCBF+1}='thrust lower HOCBF';
labels{nCBF+2}='thrust upper HOCBF';
for j=1:3
    labels{nCBF+2+j}=sprintf('torque upper axis %d',j);
    labels{nCBF+5+j}=sprintf('torque lower axis %d',j);
end
if n==nCBF+10
    labels{nCBF+9}='yaw lock upper: Mz<=nominal HLC';
    labels{nCBF+10}='yaw lock lower: Mz>=nominal HLC';
end
opts=optimoptions('linprog','Display','off');
tol=1e-6;
[baseSlack,baseFlag,baseW]=localLP(A,b,opts);

fprintf('\n========== PHYSICAL/S3 QP FAILURE FORENSICS ==========\n');
fprintf('Snapshot: %s | time %.6f s\n',snapshotFile,D.time);
fprintf('QP: %d rows (%d CBF + 2 thrust + 6 torque + %d yaw-lock), 4 variables [fDDot Mx My Mz]\n',n,nCBF,n-nCBF-8);
fprintf('Independent LP: exitflag=%g, min common slack=%+.12g\n',baseFlag,baseSlack);
if baseFlag>0 && baseSlack>tol
    fprintf('VERDICT: exact saved QP inequalities incompatible at this state.\n');
elseif baseFlag>0
    fprintf('VERDICT: inequalities feasible/nearly feasible; investigate QP numerics.\n');
else
    fprintf('VERDICT: LP inconclusive. No proof of infeasibility.\n');
end
fprintf('NOTE: slack is NOT a distance, and pointwise infeasibility is NOT inevitable collision.\n');

fprintf('\n--- Nominal-input and best-common-slack residuals (A*w-b) ---\n');
if isfield(D,'wNom') && numel(D.wNom)==4
    nominal=A*D.wNom(:)-b;
else
    nominal=nan(n,1);
end
if baseFlag>0 && all(isfinite(baseW))
    bestResidual=A*baseW-b;
else
    bestResidual=nan(n,1);
end
[~,order]=sort(nominal,'descend');
for k=1:min(n,12)
    j=order(k);
    fprintf('  row %2d  nominal=%+.7g  bestSlackLP=%+.7g  %s\n',...
        j,nominal(j),bestResidual(j),labels{j});
end
fprintf('Largest NOMINAL violation alone does not identify an infeasible set.\n');

% DIRECT INPUT-BOX TEST: For each CBF row find its mathematical minimum
% over the SAVED actuator admissible box. This uses the saved hard bounds,
% does NOT relax constraints or produce a flight command.
% In this physical QP the final eight rows are always thrust lower/upper,
% torque upper (three axes), and torque lower (three axes).
boxLower=[-b(nCBF+1);-b(nCBF+6:nCBF+8)];
boxUpper=[ b(nCBF+2); b(nCBF+3:nCBF+5)];
boxMinimum=nan(nCBF,1);
boxGap=nan(nCBF,1);
boxMinimizer=nan(4,nCBF);
fprintf('\n--- Direct per-CBF conflict with the saved physical input box ---\n');
fprintf('  fDDot in [%+.9g, %+.9g], Mx in [%+.9g, %+.9g], My in [%+.9g, %+.9g], Mz in [%+.9g, %+.9g]\n',...
    boxLower(1),boxUpper(1),boxLower(2),boxUpper(2),...
    boxLower(3),boxUpper(3),boxLower(4),boxUpper(4));
for j=1:nCBF
    a=A(j,:).';
    wExtreme=boxLower;
    wExtreme(a<0)=boxUpper(a<0);
    boxMinimizer(:,j)=wExtreme;
    boxMinimum(j)=a.'*wExtreme;
    boxGap(j)=boxMinimum(j)-b(j);
    if boxGap(j)>1e-8
        fprintf('  IMPOSSIBLE row %2d %-24s: min(A*w)=%+.10g > b=%+.10g, unavoidable gap=%+.10g\n',...
            j,labels{j},boxMinimum(j),b(j),boxGap(j));
        fprintf('    extremal w=%s [fDDot Mx My Mz]\n',mat2str(wExtreme.',9));
    end
end
if ~any(boxGap>1e-8)
    fprintf('  No single CBF row conflicts with the input box by itself; check multirow IIS.\n');
end
if isfield(D,'uRaw') && isfield(D,'uBase') && numel(D.uRaw)==4 && numel(D.uBase)==4
    fprintf('  HLC raw input      [f Mx My Mz]=%s\n',mat2str(D.uRaw(:).',9));
    fprintf('  HLC limited input  [f Mx My Mz]=%s\n',mat2str(D.uBase(:).',9));
    fprintf('  NOTE: raw HLC input is diagnostic, not the physical applied command.\n');
end

% Read-only counterfactual deletion tests. No safety constraints are changed
% in the flight controller. Positive remaining slack means STILL infeasible.
groupNames={'all UAV CBF','all cable CBF','all load CBF',...
            'all obstacle CBF','thrust HOCBF bounds','torque bounds'};
components={rows.component};
groupSets={find(strcmp(components,'uav')),...
           find(strcmp(components,'cable')),...
           find(strcmp(components,'load')),...
           (1:nCBF)',(nCBF+1:nCBF+2)',(nCBF+3:nCBF+8)'};
if n==nCBF+10
    groupNames{end+1}='CBF yaw-lock equality';
    groupSets{end+1}=(nCBF+9:nCBF+10)';
end
obsIDs=double([rows.obstacleID]);
for oid=unique(obsIDs)
    groupNames{end+1}=sprintf('obstacle %g (all objects)',oid); %#ok<AGROW>
    groupSets{end+1}=find(obsIDs==oid); %#ok<AGROW>
end
fprintf('\n--- Removal tests: OFFLINE DIAGNOSTIC ONLY ---\n');
groupResults=struct('removedGroup',{},'removedRows',{},'exitflag',{},'minSlack',{});
for k=1:numel(groupNames)
    gone=groupSets{k}(:); keep=true(n,1); keep(gone)=false;
    [s,ef]=localLP(A(keep,:),b(keep),opts);
    fprintf('  remove %-30s  slack=%+.9g  LPflag=%g\n',groupNames{k},s,ef);
    groupResults(end+1)=struct('removedGroup',groupNames{k},...
        'removedRows',gone,'exitflag',ef,'minSlack',s); %#ok<AGROW>
end
rowSlack=nan(nCBF,1); rowFlag=nan(nCBF,1);
for j=1:nCBF
    keep=true(n,1); keep(j)=false;
    [rowSlack(j),rowFlag(j)]=localLP(A(keep,:),b(keep),opts);
end
[~,rank]=sort(rowSlack,'ascend');
fprintf('\n--- Remove ONE hard CBF row (diagnostic, not a controller change) ---\n');
for k=1:nCBF
    j=rank(k);
    fprintf('  row %2d %-24s -> slack=%+.9g  flag=%g\n',...
        j,labels{j},rowSlack(j),rowFlag(j));
end

% Farkas theorem: if y>=0, A'*y=0, sum(y)=1 and b'*y<0,
% then no w can satisfy A*w<=b. Certificate is independent of the QP solver.
certificate=struct('exitflag',NaN,'y',nan(n,1),'bTy',NaN,...
    'equilibriumError',NaN,'simplexError',NaN,'verified',false);
fprintf('\n--- Farkas infeasibility certificate (independent of quadprog) ---\n');
try
    [y,by,ef]=linprog(b,[],[],[A.';ones(1,n)],...
        [zeros(4,1);1],zeros(n,1),[],opts);
    certificate.exitflag=ef;
    if ef>0 && ~isempty(y) && all(isfinite(y))
        certificate.y=y(:);
        certificate.bTy=dot(b,y);
        certificate.equilibriumError=norm(A.'*y,inf);
        certificate.simplexError=abs(sum(y)-1);
        certificate.verified=(all(y>=-1e-8) && ...
            certificate.equilibriumError<1e-6 && ...
            certificate.simplexError<1e-6 && certificate.bTy<-tol);
        fprintf('  b''*y=%+.10g, max|A''*y|=%.4g, |sum(y)-1|=%.4g\n',...
            certificate.bTy,certificate.equilibriumError,certificate.simplexError);
        if certificate.verified
            fprintf('  CERTIFICATE VERIFIED: 0 = y''*A*w <= y''*b < 0 is impossible.\n');
        else
            fprintf('  Certificate not verified to tolerance; treat as inconclusive.\n');
        end
        [~,idx]=sort(y,'descend');
        for k=1:n
            j=idx(k);
            if y(j)<1e-7,break;end
            fprintf('  dual weight row %2d: %.8g  %s\n',j,y(j),labels{j});
        end
    else
        fprintf('  Farkas LP unavailable or inconclusive (flag=%g).\n',ef);
    end
catch ME
    fprintf('  Farkas solver unavailable: %s\n',ME.message);
end

% Find deletion-minimal infeasible subsystems (IIS). Multiple orders help
% reveal alternative conflict sets; these are not minimum-size proofs.
iisSets={};
if baseFlag>0 && baseSlack>tol
    fprintf('\n--- Inclusion-minimal contradictory constraint groups (IIS) ---\n');
    orders={1:n,n:-1:1};
    for k=1:numel(orders)
        subset=localDeletionIIS(A,b,orders{k},opts,tol);
        [ss,ef]=localLP(A(subset,:),b(subset),opts);
        fprintf('  IIS candidate %d: %d rows; remaining slack=%+.9g (flag=%g)\n',...
            k,numel(subset),ss,ef);
        for t=1:numel(subset)
            j=subset(t);
            fprintf('    row %2d %s\n',j,labels{j});
        end
        iisSets{end+1}=subset; %#ok<AGROW>
    end
end

% At the previous accepted QP, compare the estimated body rate update to
% bounded-torque rigid-body dynamics. Only checks the ESTIMATED states.
stateJump=struct();
if isfield(D,'supervisor') && isfield(D.supervisor,'lastActiveTrace') && ...
        ~isempty(D.supervisor.lastActiveTrace)
    prev=D.supervisor.lastActiveTrace;
    if isfield(prev,'x') && isfield(prev,'uSafe') && isfield(D,'x') && ...
            isfield(D,'P') && isfield(D,'dt') && ...
            numel(prev.x)==19 && numel(D.x)==19 && numel(D.P)>=4 && ...
            numel(prev.uSafe)>=4
        x0=prev.x(:); x1=D.x(:); I=D.P(2:4);dt=D.dt;
        w0=x0(5:7);w1=x1(5:7);moment=prev.uSafe(2:4);
        wDot=(moment(:)-cross(w0,I(:).*w0))./I(:);
        wExpected=w0+dt*wDot;
        q0=x0(1:4)/norm(x0(1:4));q1=x1(1:4)/norm(x1(1:4));
        qAbsDot=min(1,abs(dot(q0,q1)));
        angleJump=2*acos(qAbsDot);
        stateJump=struct('previousTime',prev.time,'currentTime',D.time,...
            'priorMoment',moment(:),'omegaPrevious',w0,...
            'omegaCurrent',w1,'omegaEulerPrediction',wExpected,...
            'omegaResidualNorm',norm(w1-wExpected),...
            'quaternionGeodesicAngleRad',angleJump);
        fprintf('\n--- Last accepted step vs next EKF estimate (diagnostic) ---\n');
        fprintf('  time %.6f -> %.6f; dt %.6g\n',prev.time,D.time,dt);
        fprintf('  applied torque      %s Nm\n',mat2str(moment(:).',7));
        fprintf('  prior omega         %s rad/s\n',mat2str(w0.',7));
        fprintf('  estimated next omega %s rad/s\n',mat2str(w1.',7));
        fprintf('  rigid-body Euler est %s rad/s\n',mat2str(wExpected.',7));
        fprintf('  omega mismatch norm %.7g rad/s; quaternion angle difference %.7g rad\n',...
            stateJump.omegaResidualNorm,angleJump);
        fprintf('  CAUTION: compares EKF estimates with model prediction, NOT truth.\n');
        if isfield(prev,'rows') && isfield(prev,'hardMargins')
            fprintf('\n--- Previous accepted CBF margins vs current psi chain ---\n');
            for j=1:min(numel(prev.rows),nCBF)
                ridx=find(arrayfun(@(r) isequal(double(r.obstacleID),...
                    double(prev.rows(j).obstacleID)) && ...
                    strcmp(char(r.component),char(prev.rows(j).component)),rows),1);
                if isempty(ridx),continue;end
                fprintf('  %-24s margin(prev)=%+.6g psi3(prev)=%+.6g psi3(now)=%+.6g\n',...
                    labels{ridx},prev.hardMargins(j),...
                    prev.rows(j).psiChain(end),rows(ridx).psiChain(end));
            end
        end
    end
end

fprintf('\nINTERPRETATION: Do NOT use any LP/IIS result as a flight command.\n');
fprintf('No constraint, estimator, controller or QP is modified by this script.\n');
report=struct('snapshotFile',snapshotFile,'time',D.time,'A',A,'b',b,...
    'nCBF',nCBF,'labels',{labels},'minSlack',baseSlack,'exitflag',baseFlag,...
    'bestLPw',baseW,'bestLPResidual',bestResidual,'nominalResidual',nominal,...
    'inputBoxLower',boxLower,'inputBoxUpper',boxUpper,...
    'inputBoxCBFMinimum',boxMinimum,'inputBoxCBFGap',boxGap,...
    'inputBoxMinimizers',boxMinimizer,...
    'groupResults',groupResults,'leaveOneCBFOutSlack',rowSlack,...
    'leaveOneCBFOutExitflag',rowFlag,'farkas',certificate,...
    'IIS',{iisSets},'stateJump',stateJump);
end

function subset=localDeletionIIS(A,b,order,opts,tol)
% Starts with all inequalities and removes a row only if the remainder
% remains infeasible. Returns an inclusion-minimal infeasible subsystem.
n=size(A,1);keep=true(n,1);
for j=order
    trial=keep;trial(j)=false;
    [slack,ef]=localLP(A(trial,:),b(trial),opts);
    if ef>0 && slack>tol
        keep=trial;
    end
end
subset=find(keep);
end

function [minSlack,flag,w]=localLP(A,b,opts)
N=size(A,2);M=size(A,1);
minSlack=NaN;flag=-999;w=nan(N,1);
if M==0
    minSlack=0;flag=1;w=zeros(N,1);return;
end
try
    cost=[zeros(N,1);1];
    AA=[A,-ones(M,1)];
    lb=[-inf(N,1);0];
    [v,~,flag]=linprog(cost,AA,b(:),[],[],lb,[],opts);
    if flag>0 && ~isempty(v) && all(isfinite(v))
        minSlack=v(end);w=v(1:N);
    end
catch ME
    fprintf(2,'linprog diagnostic failed: %s\n',ME.message);
end
end
