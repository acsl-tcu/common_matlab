function report = CBF_BREEDEN_ZOH_TEST(action,varargin)
% CBF_BREEDEN_ZOH_TEST  Read-only sampled-data/HOCBF investigation.
% Never edits Sim, Scheme02, generator, or the MATLAB search path permanently.
%
%   CBF_BREEDEN_ZOH_TEST('selftest')
%   CBF_BREEDEN_ZOH_TEST('replay','C:\path\CBF_FOURWAY_RESULTS.zip')
%   CBF_BREEDEN_ZOH_TEST('replay',zipPath,'maxCases',40,'substeps',8)
%   CBF_BREEDEN_ZOH_TEST('snapshot',oneSnapshot,'substeps',16)
%   CBF_BREEDEN_ZOH_TEST('theory')
%
% A report ZIP is written next to THIS TEST, not into the Sim folders.
% Numerically sampled continuous trajectories are not proofs.
% A verified bound on d/dt(psi_r) over the entire reachable set is required
% for the Breeden-type inequality to constitute a safety certificate.
%
% MATLAB: expects the unchanged CBF_GENERATOR/COMMON and REAL/XY/GENERATED.
% 2026-11-22 FIXED: FOURWAY snapshot is Nx1 cell, not 1xN struct.
% Independent research test; no production hooks.

if nargin<1,action='theory';end
act=lower(char(action));
if strcmp(act,'theory')
    display_theory();report=struct('proofStatus','NOT_PROVEN');return;
end
[params,remaining]=parse_options(varargin);
restorePath=path;
pathCleanup=onCleanup(@()path(restorePath)); %#ok<NASGU>
find_runtime();
if strcmp(act,'selftest')
    a=synthetic_snapshot();
    [rows,details]=analyze_snapshot(a,params.substeps);
    check_selftest(a,rows);
    report=make_report(rows,details,'SYNTHETIC',params);
elseif strcmp(act,'snapshot')
    if isempty(remaining) || ~isstruct(remaining{1})
        error('BREEDEN:Input','snapshot mode requires a snapshot struct.');
    end
    [rows,details]=analyze_snapshot(remaining{1},params.substeps);
    report=make_report(rows,details,'USER_SNAPSHOT',params);
elseif strcmp(act,'replay')
    if isempty(remaining) || ~(ischar(remaining{1}) || isstring(remaining{1}))
        error('BREEDEN:Input','replay mode requires the original FOURWAY results ZIP.');
    end
    report=replay_zip(char(remaining{1}),params);
else
    error('BREEDEN:Action','Unknown action: %s',act);
end
print_report(report);
if params.save
    save_report(report,params);
end
end

function [params,other]=parse_options(args)
params=struct('substeps',8,'maxCases',40,'save',true);
other={};i=1;
while i<=numel(args)
    if (ischar(args{i}) || isstring(args{i})) && i<numel(args)
        name=char(args{i});
        if ismember(lower(name),{'substeps','maxcases','save'})
            value=args{i+1};
            switch lower(name)
                case 'substeps',params.substeps=value;
                case 'maxcases',params.maxCases=value;
                case 'save',params.save=logical(value);
            end
            i=i+2;continue;
        end
    end
    other{end+1}=args{i}; %#ok<AGROW>
    i=i+1;
end
validateattributes(params.substeps,{'numeric'},{'scalar','integer','>=',2,'<=',200});
validateattributes(params.maxCases,{'numeric'},{'scalar','integer','>=',1,'<=',1000});
end

function find_runtime()
needed={'CBF_COMMON_MODEL','CBF_REAL_XY_EVAL','CBF_REAL_XY_JET'};
if all(cellfun(@(s)exist(s,'file')==2,needed)),return;end
roots={pwd,fileparts(mfilename('fullpath')),fileparts(pwd)};
for r=1:numel(roots)
    root=roots{r};
    candidates={fullfile(root,'model','substance','CBF_GENERATOR'), ...
        fullfile(root,'CBF_GENERATOR'),root};
    for j=1:numel(candidates)
        g=candidates{j};
        a=fullfile(g,'COMMON');b=fullfile(g,'REAL','XY','GENERATED');
        if isfile(fullfile(a,'CBF_COMMON_MODEL.m')) && ...
                isfile(fullfile(b,'CBF_REAL_XY_EVAL.m'))
            addpath(a,b,'-begin');
            if all(cellfun(@(s)exist(s,'file')==2,needed)),return;end
        end
    end
end
error('BREEDEN:MissingGenerator', ...
    ['Cannot locate unchanged CBF_COMMON_MODEL/CBF_REAL_XY_EVAL/JET. ' ...
     'From project root, run addpath(genpath(''model/substance/CBF_GENERATOR'')) ' ...
     'then retry. No source modifications are needed.']);
end

function a=synthetic_snapshot()
q=[.98;.08;-.12;.10];q=q/norm(q);
p=[.08;-.11;-.99];p=p/norm(p);
a.x=[q;.1;-.12;.08;0;0;3;.4;-.2;.1;p;.02;.03;-.04];
a.u=[9.5;.22;-.13;.04];
a.P=struct('m',.85,'J',[.012;.013;.022], ...
    'gravity',9.81,'mL',.14,'cableL',2,'dstx',0,'dsty',0);
a.G=CBF_COMMON_GEOMETRY(a.P,struct('includeGround',false, ...
    'trimCableWithBodySpheres',false));
a.G.cableCenterFromLoad=.5;a.G.cableAxisShort=.2;
a.G.cableAxisLong=a.P.cableL/2+.02;
a.lambda=[4;4;4;4];a.dt=.025;a.time=0;
a.tiltCfg=struct('bodyEnabled',true,'cableEnabled',true, ...
   'bodyMaxDeg',45,'cableMaxDeg',60, ...
   'bodyLambda',[2;2],'cableLambda',[1;1.2;1.4;1.6]);
pa=struct('kind','ellipsoid','center',[1.4;-.9;2.6], ...
   'R',eye(3),'params',struct('axes',[.5;.8;1.2]),'obstacleName','left');
pb=pa;pb.center=[-1.6;.9;2.8];pb.obstacleName='right';
a.parts=[pa pb];
end

function check_selftest(a,rows)
assert(numel(rows)==4,'BREEDEN:Selftest', ...
    'Expected 2 obstacle barriers + body and cable tilt barriers.');
E0=CBF_REAL_XY_EVAL(a.x,[a.u(1);0;0;a.u(4)], ...
    a.P,a.G,a.parts,a.lambda,a.tiltCfg);
Ex=CBF_REAL_XY_EVAL(a.x,[a.u(1);1;0;a.u(4)], ...
    a.P,a.G,a.parts,a.lambda,a.tiltCfg);
Ey=CBF_REAL_XY_EVAL(a.x,[a.u(1);0;1;a.u(4)], ...
    a.P,a.G,a.parts,a.lambda,a.tiltCfg);
Ez=CBF_REAL_XY_EVAL(a.x,a.u,a.P,a.G,a.parts,a.lambda,a.tiltCfg);
B=[Ex.stages(5,:)-E0.stages(5,:);Ey.stages(5,:)-E0.stages(5,:)].';
err=max(abs(E0.stages(5,:).'+B*a.u(2:3)-Ez.stages(5,:).'));
assert(err<1e-7,'BREEDEN:Affine','XY affine property failed.');
f=CBF_COMMON_MODEL(a.x,a.u,a.P);
j=CBF_REAL_XY_JET(a.x,a.u,a.P);
assert(max(abs(f-j.stateTaylor(:,2)))<1e-9, ...
    'BREEDEN:Model','Model and jet RHS disagree.');
end

function report=replay_zip(zipFile,params)
if ~isfile(zipFile),error('BREEDEN:File','ZIP does not exist: %s',zipFile);end
work=tempname;mkdir(work);
cleanup=onCleanup(@()delete_temp(work)); %#ok<NASGU>
unzip(zipFile,work);
matPath=fullfile(work,'RAW_SNAPSHOTS.mat');
csvPath=fullfile(work,'PER_SAMPLE.csv');
if ~isfile(matPath) || ~isfile(csvPath)
    error('BREEDEN:Format','FOURWAY ZIP must contain RAW_SNAPSHOTS.mat and PER_SAMPLE.csv.');
end
S=readtable(csvPath);
ix=select_indices(S,params.maxCases);
if isempty(ix),error('BREEDEN:Empty','No observed obstacle snapshots.');end
mf=matfile(matPath); % v7.3: read ONE CELL at a time, never assume orientation
snapshotDims=size(mf,'snapshot');
if numel(snapshotDims)<2 || prod(snapshotDims)<max(ix)
    error('BREEDEN:SnapshotDimensions', ...
        'Snapshot array size [%s] cannot contain sample %d.', ...
        num2str(snapshotDims),max(ix));
end
% FOURWAY saves a COLUMN CELL ARRAY, e.g. 2157x1 in MATLAB.
% MATLAB/HDF5 file inspection tools may display reversed storage axes.
if snapshotDims(1)>=max(ix) && snapshotDims(2)==1
    layout='COLUMN_CELL';
elseif snapshotDims(2)>=max(ix) && snapshotDims(1)==1
    layout='ROW_CELL';
else
    error('BREEDEN:SnapshotLayout', ...
        'Unsupported snapshot layout [%s]. Expected a vector cell array.', ...
        num2str(snapshotDims));
end
fprintf('[BREEDEN] snapshot dimensions (MATLAB): [%s], layout: %s\n', ...
    num2str(snapshotDims),layout);
rows=struct([]);details=struct([]);fail=0;
failedCases=cell(0,1);fullSnapshotCache=[];
for j=1:numel(ix)
    try
        % matfile reads a 1x1 CELL; then unwrap its stored snapshot struct.
        % If a MATLAB release cannot partially read a cell, load the
        % entire cell variable ONCE as a compatibility fallback.
        if ~isempty(fullSnapshotCache)
            oneCell=fullSnapshotCache(ix(j));
        else
            try
                if strcmp(layout,'COLUMN_CELL')
                    oneCell=mf.snapshot(ix(j),1);
                else
                    oneCell=mf.snapshot(1,ix(j));
                end
            catch readError
                fprintf(['[BREEDEN] MatFile indexed-cell read unavailable ' ...
                    '(%s). Using one-time full snapshot load.\n'], ...
                    readError.identifier);
                full=load(matPath,'snapshot');
                fullSnapshotCache=full.snapshot;
                clear full
                oneCell=fullSnapshotCache(ix(j));
            end
        end
        if iscell(oneCell) && numel(oneCell)==1
            snap=oneCell{1};
        elseif isstruct(oneCell) && isscalar(oneCell)
            snap=oneCell;
        else
            error('BREEDEN:SnapshotType', ...
                'Snapshot %d has unexpected type %s, elements %d.', ...
                ix(j),class(oneCell),numel(oneCell));
        end
        if ~isstruct(snap) || ~isscalar(snap)
            error('BREEDEN:Snapshot','Sample %d must contain a scalar struct.',ix(j));
        end
        [r,d]=analyze_snapshot(snap,params.substeps);
        rows=append_struct(rows,r);details=append_struct(details,d);
    catch ME
        fail=fail+1;
        lines=cell(numel(ME.stack),1);
        for si=1:numel(ME.stack)
            lines{si}=sprintf('  at %s line %d',ME.stack(si).name,ME.stack(si).line);
        end
        failure=sprintf('case %d, sample %d: %s (%s)\n%s', ...
            j,ix(j),ME.message,ME.identifier,strjoin(lines,newline));
        failedCases{end+1,1}=failure; %#ok<AGROW>
        fprintf('[BREEDEN] FAILED %s\n',failure);
    end
    if mod(j,10)==0 || j==numel(ix)
        fprintf('[BREEDEN] replay %d/%d cases completed; errors %d\n', ...
            j,numel(ix),fail);
    end
end
report=make_report(rows,details,'FOURWAY_SNAPSHOTS',params);
if fail==numel(ix)
    report.proofStatus='NOT_PROVEN_ALL_REPLAY_CASES_FAILED';
    report.reason='All selected replay snapshots failed; inspect REPLAY_ERRORS.txt.';
    fprintf('[BREEDEN] ALL CASES FAILED. Diagnostic ZIP will contain stack traces.\n');
end
report.requestedSnapshots=numel(ix);
report.failedSnapshots=fail;
report.failedCaseDetails=failedCases;
report.snapshotDimensions=snapshotDims;
report.snapshotLayout=layout;
report.usedFullSnapshotLoad=~isempty(fullSnapshotCache);
report.selectedSampleNumbers=ix(:);
report.dataSource=zipFile;
end

function ix=select_indices(S,maxCases)
valid=find(S.parts>0);
if isempty(valid),ix=[];return;end
m=min(maxCases,numel(valid));
nH=min(ceil(m*.35),numel(valid));
nPsi=min(ceil(m*.35),numel(valid));
[~,pH]=sort(S.min_h_m(valid),'ascend');
[~,pPsi]=sort(S.min_psi3(valid),'ascend');
nEven=min(ceil(m*.3),numel(valid));
iEven=unique(round(linspace(1,numel(valid),nEven)));
iEven=iEven(:);
ix=unique([valid(pH(1:nH));valid(pPsi(1:nPsi));valid(iEven)],'stable');
if numel(ix)>m
    % Preserve the first two priority groups; truncate uniformly if needed.
    ix=ix(1:m);
elseif numel(ix)<m
    leftovers=setdiff(valid,ix,'stable');
    ix=[ix;leftovers(1:min(m-numel(ix),numel(leftovers)))];
end
ix=sort(ix(:));
end

function [rows,detail]=analyze_snapshot(a,n)
required={'x','u','P','G','lambda','dt','parts'};
for z=1:numel(required)
    if ~isfield(a,required{z})
        error('BREEDEN:Snapshot','Snapshot missing %s',required{z});
    end
end
x=double(a.x(:));u=double(a.u(:));dt=double(a.dt);
if isempty(a.parts),parts=struct([]);else,parts=a.parts;end
P=a.P;G=a.G;lambda=a.lambda(:);
if isfield(a,'tiltCfg'),tilt=a.tiltCfg;else,tilt=[];end
if isfield(a,'time'),t=double(a.time);else,t=NaN;end
assert(numel(x)==19 && numel(u)==4 && dt>0, ...
    'BREEDEN:Size','Invalid x,u,dt.');
if abs(norm(x(1:4))-1)>1e-5 || abs(norm(x(14:16))-1)>1e-5
    x=unit_project(x); % same projected manifold as Scheme02 evaluator
end
ns=n+1;tvec=linspace(0,dt,ns);
E=cell(1,ns);E{1}=CBF_REAL_XY_EVAL(x,u,P,G,parts,lambda,tilt);
states=zeros(19,ns);states(:,1)=x;
y=x;
for k=2:ns
    y=rk4_step(y,u,P,dt/n);
    states(:,k)=y;
    E{k}=CBF_REAL_XY_EVAL(y,u,P,G,parts,lambda,tilt);
end
[names,stages,lambdas]=collect_stages(E{1},lambda,tilt);
rows=struct([]);
for c=1:numel(names)
    r=numel(lambdas{c});
    vals=zeros(r+1,ns);
    for k=1:ns
        [~,s]=collect_stages(E{k},lambda,tilt);
        vals(:,k)=s{c};
    end
    predicted=zeros(r,2);
    for k=1:2
        tk=dt*k/2;
        for order=0:r-1
            predicted(order+1,k)=stage_taylor(E{1},c, ...
                stages{c},lambdas{c},order,tk);
        end
    end
    % Compare generator's Taylor-4 guard predictions with RK4 integration.
    rawLow=vals(1:r,:);
    if mod(n,2)==0
        actualAtHalf=rawLow(:,n/2+1);
    else
        % We require substeps divisible by 2 for an exact half-time sample.
        % Preserve a fair guard comparison even for odd n via interpolation.
        actualAtHalf=zeros(r,1);
        for jj=1:r
            actualAtHalf(jj)=interp1(tvec,rawLow(jj,:),dt/2);
        end
    end
    actualAtEnd=rawLow(:,end);
    guardErr=max(abs([actualAtHalf,actualAtEnd]-predicted),[],'all');
    q=vals(r,:);F=vals(r+1,:); % q=psi_{r-1}; F=psi_r
    drops=max(0,F(1)-F(2:end));
    sampledRate=max([0, drops./tvec(2:end)]);
    % This is NOT a bound on dF/dt; it is only a necessary finite-grid
    % envelope for a constant slope margin and cannot certify invariance.
    h=vals(1,:);
    row=struct('t',t,'barrier',names{c},'order',r, ...
       'dt',dt,'substeps',n,'startH',h(1),'gridMinH',min(h), ...
       'startLow',min(vals(1:r,1)), ...
       'gridMinLow',min(rawLow,[],'all'), ...
       'startPsiFinal',F(1),'gridMinPsiFinal',min(F), ...
       'sampledNecessarySlope',sampledRate, ...
       'allowableCertifiedSlope',F(1)/dt, ...
       'observedPhi4Drop',max(drops), ...
       'gridStartValid',all(vals(1:r,1)>=-1e-9), ...
       'gridAllLowNonnegative',all(rawLow>=-1e-9,'all'), ...
       'gridAllFinalNonnegative',all(F>=-1e-9), ...
       'taylorGuardGridPass',all(predicted>=-1e-9,'all'), ...
       'taylorGuardObservedPass',all([actualAtHalf,actualAtEnd]>=-1e-9,'all'), ...
       'taylorGuardMaxAbsError',guardErr, ...
       'certifiedDerivativeBound',NaN, ...
       'zohCertificate','NOT_PROVEN_NEEDS_REACHABLE_SET_BOUND');
    rows=append_struct(rows,row);
end
% Accuracy indication, NOT an encloser: compare RK4 grid 1-step vs 2 half.
xe=states(:,end);yh=x;
for i=1:2*n,yh=rk4_step(yh,u,P,dt/(2*n));end
endpointNumericalDelta=norm(xe-yh,inf);
detail=struct('t',t,'parts',numel(parts),'dt',dt,'n',n, ...
    'initialInput',u.','initialState',x.','endpointState',xe.', ...
    'endpointRK4RefinedDeltaInf',endpointNumericalDelta, ...
    'model','CBF_COMMON_MODEL: estimated 19-state dynamics', ...
    'actualAppliedInputUnknown',true, ...
    'plantIntersampleSafety','NOT_PROVEN', ...
    'recursiveFeasibility','NOT_PROVEN');
end

function x1=rk4_step(x,u,P,h)
f=@(z)CBF_COMMON_MODEL(z,u,P);
k1=f(x);k2=f(x+.5*h*k1);k3=f(x+.5*h*k2);k4=f(x+h*k3);
x1=x+h*(k1+2*k2+2*k3+k4)/6;
x1=unit_project(x1);
end

function x=unit_project(x)
% Projection is for numerical replay, not an interval/reachability proof.
q=norm(x(1:4));p=norm(x(14:16));
if q<.5 || p<.5,error('BREEDEN:Norm','Degenerate manifold state.');end
x(1:4)=x(1:4)/q;x(14:16)=x(14:16)/p;
end

function [names,stages,lambdas]=collect_stages(E,lambda,tilt)
names=cell(0,1);stages=cell(0,1);lambdas=cell(0,1);
for j=1:numel(E.names)
    names{end+1,1}=char(E.names{j}); %#ok<AGROW>
    stages{end+1,1}=E.stages(:,j); %#ok<AGROW>
    lambdas{end+1,1}=lambda(:); %#ok<AGROW>
end
if isempty(tilt),return;end
if tilt.bodyEnabled
    names{end+1,1}='bodyTilt'; %#ok<AGROW>
    stages{end+1,1}=E.tiltBodyStages(:); %#ok<AGROW>
    lambdas{end+1,1}=tilt.bodyLambda(:); %#ok<AGROW>
end
if tilt.cableEnabled
    names{end+1,1}='cableTilt'; %#ok<AGROW>
    stages{end+1,1}=E.tiltCableStages(:); %#ok<AGROW>
    lambdas{end+1,1}=tilt.cableLambda(:); %#ok<AGROW>
end
end

function predicted=stage_taylor(E,c,stages,lambda,order,t)
% Reconstruct raw h derivatives from stage recursion (same polynomial used
% in Scheme02's xy_unpack_hocbf / xy_project_stage).
% E, c reserved for consistency with collected barrier indexing.
%#ok<INUSD>
r=numel(lambda);h=zeros(r+1,1);
for k=0:r
    p=1;
    for j=1:k,p=conv(p,[lambda(j),1]);end
    if k==0,h(1)=stages(1);else
        h(k+1)=stages(k+1)-p(1:end-1)*h(1:k);
    end
end
d=zeros(1,order+1);
for j=0:order
    for q=j:r
        d(j+1)=d(j+1)+h(q+1)*t^(q-j)/factorial(q-j);
    end
end
p=1;for j=1:order,p=conv(p,[lambda(j),1]);end
predicted=p*d.';
end

function out=append_struct(out,in)
if isempty(in),return;end
if isempty(out),out=in;else,out(end+1:end+numel(in))=in;end
end

function report=make_report(rows,detail,source,opts)
report=struct();
report.kind='READ_ONLY_SAMPLED_DATA_EXPLORATION';
report.source=source;report.parameters=opts;
report.rows=rows;report.details=detail;
report.samplesAnalyzed=numel(detail);report.barrierCases=numel(rows);
report.proofStatus='NOT_PROVEN';
report.reason=['A finite RK4 grid and a Taylor approximation cannot ' ...
    'certify a 25 ms interval. Need analytically validated upper bound ' ...
    'M >= sup_{reachable x,u} max(0,-d psi_r/dt); initial lower stages ' ...
    'and future feasibility also require separate proofs.'];
if isempty(rows)
    report.initialLowerInvalid=0;report.observedFinalNegative=0;
    report.observedHNegative=0;report.polynomialMisses=0;
else
    report.initialLowerInvalid=sum(~[rows.gridStartValid]);
    report.observedFinalNegative=sum(~[rows.gridAllFinalNonnegative]);
    report.observedHNegative=sum([rows.gridMinH]<0);
    report.polynomialMisses=sum([rows.taylorGuardGridPass] & ...
        ~[rows.taylorGuardObservedPass]);
end
end

function print_report(r)
fprintf('\n[BREEDEN ZOH] %s\n',r.source);
fprintf('  sampled states:                 %d\n',r.samplesAnalyzed);
fprintf('  individual barrier cases:       %d\n',r.barrierCases);
fprintf('  invalid initial lower stages:   %d\n',r.initialLowerInvalid);
fprintf('  RK4 grid saw psi_final<0:      %d\n',r.observedFinalNegative);
fprintf('  RK4 grid saw physical h<0:     %d\n',r.observedHNegative);
fprintf('  Taylor guard midpoint/end miss: %d\n',r.polynomialMisses);
if isfield(r,'failedSnapshots')
    fprintf('  unreadable/failed snapshots:    %d\n',r.failedSnapshots);
end
if ~isempty(r.rows)
    F=[r.rows.startPsiFinal]; M=[r.rows.sampledNecessarySlope];
    good=F>=0 & isfinite(M);
    fprintf('  positive psi_final at sample:   %d/%d\n',sum(good),numel(F));
    fprintf('  sampled slope exceeds budget:  %d (diagnostic ONLY)\n', ...
        sum(good & M>[r.rows.allowableCertifiedSlope]));
end
fprintf('  Breeden intersample certificate: %s\n',r.proofStatus);
fprintf('  All slopes from RK4 are OBSERVATIONS, not verified bounds.\n');
end

function save_report(r,opts)
% Create exactly one result ZIP, outside the project sources.
outDir=fileparts(mfilename('fullpath'));
if isempty(outDir),outDir=pwd;end
name='CBF_BREEDEN_ZOH_RESULTS.zip';
tmp=tempname;mkdir(tmp);cl=onCleanup(@()delete_temp(tmp)); %#ok<NASGU>
summary=fullfile(tmp,'RESULT_SUMMARY.txt');
fid=fopen(summary,'w');
if fid<0,error('BREEDEN:FileWrite','Cannot write results.');end
fcloseGuard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'BREEDEN ZOH READ-ONLY RESEARCH TEST\n');
fprintf(fid,'source: %s\n',r.source);
fprintf(fid,'snapshots: %d, barrier cases: %d\n',r.samplesAnalyzed,r.barrierCases);
fprintf(fid,'initial invalid lower stages: %d\n',r.initialLowerInvalid);
fprintf(fid,'sampled final stage negative: %d\n',r.observedFinalNegative);
fprintf(fid,'sampled physical h negative: %d\n',r.observedHNegative);
fprintf(fid,'Taylor midpoint/endpoint false pass: %d\n',r.polynomialMisses);
if isfield(r,'failedSnapshots')
    fprintf(fid,'failed replay snapshots: %d\n',r.failedSnapshots);
end
fprintf(fid,'THEORETICAL GUARANTEE: NOT PROVEN.\n');
fprintf(fid,'No modifications are made to original sources.\n');
fprintf(fid,'No modifications are made to the original Sim, Scheme02 or generator.\n');
fprintf(fid,['REQUIRED: proven lower bound on d(psi_r)/dt ' ...
    'over the entire 25ms reachable tube, valid initial low stages, ' ...
    'and recursive admissible-input viability.\n']);
clear fcloseGuard;
if ~isempty(r.rows)
    writetable(struct2table(r.rows),fullfile(tmp,'PER_BARRIER.csv'));
end
if ~isempty(r.details)
    % Keep a small numeric summary; no raw state dump unless requested.
    d=r.details;
    dt=table([d.t].',[d.parts].',[d.dt].', ...
       [d.endpointRK4RefinedDeltaInf].', ...
       'VariableNames',{'time','parts','dt','RK4EndpointConvergenceInf'});
    writetable(dt,fullfile(tmp,'NUMERICAL_ACCURACY.csv'));
end
if isfield(r,'failedCaseDetails')
    errorPath=fullfile(tmp,'REPLAY_ERRORS.txt');
    fidErr=fopen(errorPath,'w');
    if fidErr<0,error('BREEDEN:FileWrite','Cannot write replay errors.');end
    if isempty(r.failedCaseDetails)
        fprintf(fidErr,'No snapshot replay errors.\n');
    else
        for ei=1:numel(r.failedCaseDetails)
            fprintf(fidErr,'%s\n',r.failedCaseDetails{ei});
        end
    end
    fclose(fidErr);
end
save(fullfile(tmp,'REPORT.mat'),'r','-v7');
zipPath=fullfile(outDir,name);
if isfile(zipPath),delete(zipPath);end
zip(zipPath,dir_files(tmp),tmp);
fprintf('  Results ZIP: %s\n',zipPath);
end

function f=dir_files(root)
d=dir(root);d=d(~[d.isdir]);f={d.name};
end

function delete_temp(p)
if isfolder(p),rmdir(p,'s');end
end

function display_theory()
fprintf('\n[BREEDEN ZOH THEORY / MODEL CONDITIONAL]\n');
fprintf(['For each order-r barrier, let q=psi_{r-1}(x;T_k,tauZ_k) ' ...
    'and F=psi_r(x,u_k).\n']);
fprintf(['Assume within [t_k,t_k+dt] the actual input is held ' ...
    'and the estimated dynamics are exact.\n']);
fprintf(['If initial psi_0..psi_{r-1}>=0 and along the FULL ' ...
    'reachable tube dF/dt>=-M, then F(t_k)>=M*dt\n']);
fprintf(['implies F(t)>=0 throughout the hold. Repeated comparison ' ...
    'lemmas imply h(t)>=0 on this hold.\n']);
fprintf(['M must be a rigorously justified reachable-set bound, ' ...
    'NOT fitted from RK4 samples.\n']);
fprintf(['At the next tick, re-evaluate ALL low stages with NEW T; ' ...
    'do not assume the old stages persist.\n']);
fprintf(['Multi-obstacle: all such inequalities must share ONE ' ...
    'bounded physical input. Existence for every tick is separate.\n']);
end
