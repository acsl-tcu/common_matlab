function report=CBF_BREEDEN_BOUND_STUDY(mode,varargin)
% CBF_BREEDEN_BOUND_STUDY  Read-only, parameter-dependent sampled-data study.
% Does NOT modify Sim, controller, generator, MATLAB files, or control laws.
% Required original files on MATLAB path:
%   CBF_COMMON_MODEL, CBF_REAL_XY_EVAL, CBF_REAL_XY_JET,
%   CBF_COMMON_GEOMETRY (all inside the supplied CBF_GENERATOR).
%
% Examples:
%   CBF_BREEDEN_BOUND_STUDY('selftest');
%   % Also checks replay positional ZIP argument parsing (regression).
%   CBF_BREEDEN_BOUND_STUDY('replay','CBF_FOURWAY_RESULTS.zip', ...
%          'maxCases',8,'substeps',8);
%   CBF_BREEDEN_BOUND_STUDY('replay','CBF_FOURWAY_RESULTS.zip', ...
%          'maxCases',8,'massFactors',[0.8 1 1.2], ...
%          'gainFactors',[0.8 1 1.2]);
%
% Output (next to this M file): CBF_BREEDEN_BOUND_RESULTS.zip
% CAUTION: finite-sample numerical slopes are NOT mathematically certified
% bounds. Output never claims a proof of intersample safety or viability.
if nargin<1,mode='selftest';end
% Parse the single positional replay ZIP BEFORE name/value options.
% Previously, a following 'maxCases' made the ZIP look like an option name.
[opts,file]=bound_parse_arguments(mode,varargin);
oldPath=path;restore=onCleanup(@()path(oldPath)); %#ok<NASGU>
locate_generator();
if strcmpi(mode,'selftest')
    bound_parser_selftest();
    a=example_snapshot();snapshots={a};sampleIds=1;
elseif strcmpi(mode,'replay')
    if isempty(file) || ~isfile(file),error('BOUND:Input','Valid FOURWAY ZIP required.');end
    folder=tempname;mkdir(folder);cleanup=onCleanup(@()cleanup_temp(folder)); %#ok<NASGU>
    unzip(file,folder);
    tab=readtable(fullfile(folder,'PER_SAMPLE.csv'));
    valid=find(tab.parts>0);
    if isempty(valid),error('BOUND:Empty','No obstacle samples in log.');end
    k=min(opts.maxCases,numel(valid));
    % Deterministic coverage: near geometric boundary, HOCBF boundary,
    % and uniformly spaced times. No assumption about specific flight times.
    [~,a]=sort(tab.min_h_m(valid));[~,b]=sort(tab.min_psi3(valid));
    ka=min(ceil(k/3),numel(valid));kb=min(ceil(k/3),numel(valid));
    picks=unique([valid(a(1:ka));valid(b(1:kb)); ...
        valid(unique(round(linspace(1,numel(valid),k))))],'stable');
    picks=picks(1:min(numel(picks),k));
    if numel(picks)<k
        remains=setdiff(valid,picks,'stable');
        picks=[picks;remains(1:k-numel(picks))];
    end
    picks=sort(picks);
    dat=load(fullfile(folder,'RAW_SNAPSHOTS.mat'),'snapshot');
    cellSnapshots=dat.snapshot;
    if ~iscell(cellSnapshots) || numel(cellSnapshots)<max(picks)
        error('BOUND:Data','RAW_SNAPSHOTS snapshot must be a cell vector.');
    end
    snapshots=cellSnapshots(picks);sampleIds=picks(:);
else
    error('BOUND:Mode','Use selftest or replay.');
end
rows=struct([]);failures={};
for a=1:numel(snapshots)
    try
        snap=snapshots{a};
        for mi=1:numel(opts.massFactors)
            for gi=1:numel(opts.gainFactors)
                mfac=opts.massFactors(mi);lfac=opts.gainFactors(gi);
                r=assess_case(snap,sampleIds(a),mfac,lfac,opts.substeps);
                if isempty(rows),rows=r;else,rows(end+1:end+numel(r))=r;end %#ok<AGROW>
            end
        end
    catch ME
        stack='';
        for k=1:numel(ME.stack)
            stack=[stack,sprintf(' %s:%d',ME.stack(k).name,ME.stack(k).line)]; %#ok<AGROW>
        end
        failures{end+1,1}=sprintf('sample=%d  %s  %s', ...
            sampleIds(a),ME.message,stack); %#ok<AGROW>
    end
end
report=struct('mode',char(mode),'caseCount',numel(snapshots), ...
    'barrierRows',numel(rows),'rows',rows,'errors',{failures}, ...
    'guarantee','NOT_PROVEN_NUMERICAL_DIAGNOSTIC_ONLY', ...
    'massIsEstimated',true,'parameters',opts);
if ~isempty(rows)
    fprintf('[BOUND] cases=%d barriers=%d failed=%d\n', ...
        numel(snapshots),numel(rows),numel(failures));
    fprintf('[BOUND] initial lower-stage invalid: %d\n',sum(~[rows.initialLowerOK]));
    fprintf('[BOUND] F=psi_r negative on sample grid: %d\n', ...
        sum([rows.gridMinimumF]<0));
    fprintf('[BOUND] mass and gain are read from each snapshot, then scaled.\n');
else
    fprintf('[BOUND] NO VALID CASES. Inspect error report.\n');
end
if ~isempty(failures)
    fprintf('[BOUND] First case error: %s\n',failures{1});
end
fprintf('[BOUND] CERTIFIED upper bounds: NOT AVAILABLE. Safety: NOT_PROVEN.\n');
if opts.save,save_report(report);end
end

function [opts,file]=bound_parse_arguments(mode,args)
% Single positional replay input, then strict name/value pairs. This helper
% is also exercised by selftest so the public replay syntax is covered.
opts=struct('maxCases',8,'substeps',8,'massFactors',1, ...
    'gainFactors',1,'save',true);
file='';
if ~(ischar(mode) || (isstring(mode) && isscalar(mode)))
    error('BOUND:Mode','Mode must be a character vector or scalar string.');
end
mode=lower(char(mode));
if strcmp(mode,'replay')
    if isempty(args) || ~(ischar(args{1}) || ...
            (isstring(args{1}) && isscalar(args{1})))
        error('BOUND:Input', ...
            'Replay requires a ZIP path as its first argument.');
    end
    file=char(args{1});
    if isempty(file),error('BOUND:Input','Replay ZIP path is empty.');end
    args=args(2:end);
elseif ~strcmp(mode,'selftest')
    error('BOUND:Mode','Use selftest or replay.');
end
if mod(numel(args),2)~=0
    error('BOUND:OptionPairs', ...
        'Options must be name/value pairs after the replay ZIP path.');
end
for i=1:2:numel(args)
    name=args{i};value=args{i+1};
    if ~(ischar(name) || (isstring(name) && isscalar(name)))
        error('BOUND:Option','An option name must be text.');
    end
    key=lower(char(name));
    switch key
        case 'maxcases',opts.maxCases=value;
        case 'substeps',opts.substeps=value;
        case 'massfactors',opts.massFactors=value;
        case 'gainfactors',opts.gainFactors=value;
        case 'save'
            validateattributes(value,{'numeric','logical'},{'scalar'});
            if ~(isequal(value,0) || isequal(value,1))
                error('BOUND:Option','save must be true or false.');
            end
            opts.save=logical(value);
        otherwise
            error('BOUND:Option','Unknown option: %s',key);
    end
end
validateattributes(opts.maxCases,{'numeric'},{'scalar','integer','>=',1,'<=',200});
validateattributes(opts.substeps,{'numeric'},{'scalar','integer','>=',2,'<=',100});
validateattributes(opts.massFactors,{'numeric'},{'vector','positive','finite'});
validateattributes(opts.gainFactors,{'numeric'},{'vector','positive','finite'});
end

function bound_parser_selftest()
% Regression tests for the EXACT user-facing call that failed on 2026-11-23.
[o,p]=bound_parse_arguments('replay', ...
    {'CBF_FOURWAY_RESULTS.zip','maxCases',8,'substeps',8});
assert(strcmp(p,'CBF_FOURWAY_RESULTS.zip') && ...
    o.maxCases==8 && o.substeps==8, ...
    'BOUND:ParserSelftest','Replay positional ZIP parsing failed.');
[o,p]=bound_parse_arguments('replay', ...
    {'C:\example\CBF_FOURWAY_RESULTS.zip', ...
     'massFactors',[0.8 1 1.2],'gainFactors',[0.8 1 1.2]});
assert(strcmp(p,'C:\example\CBF_FOURWAY_RESULTS.zip') && ...
    numel(o.massFactors)==3 && numel(o.gainFactors)==3, ...
    'BOUND:ParserSelftest','Replay parameter arrays parsing failed.');
[o,p]=bound_parse_arguments('selftest',{'substeps',16});
assert(isempty(p) && o.substeps==16,'BOUND:ParserSelftest', ...
    'Selftest name/value parsing failed.');
rejected=false;
try
    bound_parse_arguments('replay', ...
        {'CBF_FOURWAY_RESULTS.zip','unknownOption',123});
catch ME
    rejected=strcmp(ME.identifier,'BOUND:Option');
end
assert(rejected,'BOUND:ParserSelftest', ...
    'Unknown replay option was not rejected.');
fprintf('[BOUND] Argument parser regression: PASS (replay ZIP + options).\n');
end

function r=assess_case(snap,id,mfac,lfac,n)
required={'x','u','P','G','lambda','parts','dt'};
for i=1:numel(required)
    if ~isfield(snap,required{i}),error('BOUND:Snapshot','Missing %s',required{i});end
end
x=double(snap.x(:));u=double(snap.u(:));P=snap.P;
P.mL=double(P.mL)*mfac; % mL is a per-snapshot ESTIMATE.
lam=double(snap.lambda(:))*lfac;
if isfield(snap,'tiltCfg'),tilt=snap.tiltCfg;else,tilt=[];end
if ~isempty(tilt)
    tilt.bodyLambda=double(tilt.bodyLambda(:))*lfac;
    tilt.cableLambda=double(tilt.cableLambda(:))*lfac;
end
if isfield(snap,'time'),tm=double(snap.time);else,tm=NaN;end
G=snap.G;dt=double(snap.dt);
assert(numel(x)==19 && numel(u)==4 && dt>0,'BOUND:Dimensions','Bad sample.');
x=unit_state(x);t=(0:n)*dt/n;
eval0=CBF_REAL_XY_EVAL(x,u,P,G,snap.parts,lam,tilt);
[names,vals0]=collect_barriers(eval0,lam,tilt);
M=numel(names);L=zeros(M,n+1);H=zeros(M,n+1);
F=zeros(M,n+1);D=zeros(M,n+1);
y=x;
for j=1:n+1
    if j>1,y=rk4(y,u,P,dt/n);end
    e=CBF_REAL_XY_EVAL(y,u,P,G,snap.parts,lam,tilt);
    [namesNow,stages]=collect_barriers(e,lam,tilt);
    if ~isequal(namesNow,names),error('BOUND:BarrierSet','Barrier list changed.');end
    % Directional derivative in the fixed-input model. This is a finite
    % difference ONLY; not a mathematically verified derivative bound.
    dx=CBF_COMMON_MODEL(y,u,P);
    step=1e-6;yp=unit_state(y+step*dx);ym=unit_state(y-step*dx);
    [~,sp]=collect_barriers(CBF_REAL_XY_EVAL(yp,u,P,G,snap.parts,lam,tilt),lam,tilt);
    [~,sm]=collect_barriers(CBF_REAL_XY_EVAL(ym,u,P,G,snap.parts,lam,tilt),lam,tilt);
    for z=1:M
        v=stages{z};vp=sp{z};vm=sm{z};
        H(z,j)=v(1);L(z,j)=min(v(1:end-1));
        F(z,j)=v(end);D(z,j)=(vp(end)-vm(end))/(2*step);
    end
end
r=struct([]);
for z=1:M
    % Theorem adaptation: if PROVEN M>=sup[-d(psi_r)/dt]_+ over reachable
    % tube, then F(0)>=M*dt is sufficient for F(s)>=0 for all s in [0,dt].
    % These finite-grid numbers do NOT provide a proven M.
    obsDrop=max(0,F(z,1)-min(F(z,:)));
    empiricalM=max(0,-min(D(z,:)));
    meanSecant=max([0,(F(z,1)-F(z,2:end))./t(2:end)]);
    entry=struct('sampleIndex',id,'time',tm,'barrier',names{z}, ...
        'massEstimateKg',P.mL,'massFactor',mfac,'gainFactor',lfac, ...
        'dt',dt,'startH',H(z,1),'minimumGridH',min(H(z,:)), ...
        'initialLowerOK',L(z,1)>=-1e-9, ...
        'startFinalCBF',F(z,1),'gridMinimumF',min(F(z,:)), ...
        'gridMinimumLower',min(L(z,:)), ...
        'observedDropF',obsDrop,'empiricalNecessarySlope',meanSecant, ...
        'empiricalDerivativeSlope',empiricalM, ...
        'necessaryBudgetSlope',F(z,1)/dt, ...
        'gridSlopeTest',F(z,1)>=empiricalM*dt, ...
        'certifiedSlope',NaN,'certificate','NOT_PROVEN');
    if isempty(r),r=entry;else,r(end+1)=entry;end %#ok<AGROW>
end
end

function [names,stages]=collect_barriers(E,lam,tilt) %#ok<INUSD>
names=cell(0,1);stages=cell(0,1);
for i=1:numel(E.names)
    names{end+1,1}=char(E.names{i}); %#ok<AGROW>
    stages{end+1,1}=E.stages(:,i); %#ok<AGROW>
end
if ~isempty(tilt)
    if tilt.bodyEnabled
        names{end+1,1}='bodyTilt';stages{end+1,1}=E.tiltBodyStages(:); %#ok<AGROW>
    end
    if tilt.cableEnabled
        names{end+1,1}='cableTilt';stages{end+1,1}=E.tiltCableStages(:); %#ok<AGROW>
    end
end
end

function y=rk4(x,u,P,h)
f=@(v)CBF_COMMON_MODEL(v,u,P);
k1=f(x);k2=f(x+h*k1/2);k3=f(x+h*k2/2);k4=f(x+h*k3);
y=unit_state(x+h*(k1+2*k2+2*k3+k4)/6);
end

function x=unit_state(x)
q=norm(x(1:4));p=norm(x(14:16));
if q<0.5 || p<0.5,error('BOUND:Norm','Invalid quaternion/cable norm.');end
x(1:4)=x(1:4)/q;x(14:16)=x(14:16)/p;
end

function a=example_snapshot()
q=[.98;.08;-.12;.1];q=q/norm(q);
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
    'bodyMaxDeg',45,'cableMaxDeg',60,'bodyLambda',[2;2], ...
    'cableLambda',[1;1.2;1.4;1.6]);
pa=struct('kind','ellipsoid','center',[1.4;-.9;2.6], ...
    'R',eye(3),'params',struct('axes',[.5;.8;1.2]),'obstacleName','A');
pb=pa;pb.center=[-1.6;.9;2.8];pb.obstacleName='B';
a.parts=[pa pb];
end

function locate_generator()
needed={'CBF_COMMON_MODEL','CBF_COMMON_GEOMETRY','CBF_REAL_XY_EVAL'};
if all(cellfun(@(s)exist(s,'file')==2,needed)),return;end
roots={pwd,fileparts(mfilename('fullpath')),fileparts(pwd)};
for i=1:numel(roots)
    choices={fullfile(roots{i},'CBF_GENERATOR'), ...
        fullfile(roots{i},'model','substance','CBF_GENERATOR'),roots{i}};
    for j=1:numel(choices)
        r=choices{j};common=fullfile(r,'COMMON');gen=fullfile(r,'REAL','XY','GENERATED');
        if isfile(fullfile(common,'CBF_COMMON_MODEL.m')) && ...
                isfile(fullfile(gen,'CBF_REAL_XY_EVAL.m'))
            addpath(common,gen,'-begin');
            if all(cellfun(@(s)exist(s,'file')==2,needed)),return;end
        end
    end
end
error('BOUND:Generator','Original CBF_GENERATOR COMMON/REAL/XY/GENERATED not found. Add these two folders to MATLAB path.');
end

function save_report(report)
folder=fileparts(mfilename('fullpath'));
tmp=tempname;mkdir(tmp);guard=onCleanup(@()cleanup_temp(tmp)); %#ok<NASGU>
sumFile=fullfile(tmp,'SUMMARY.txt');fid=fopen(sumFile,'w');
if fid<0,error('BOUND:IO','Cannot write summary.');end
fprintf(fid,'Breeden bound applicability audit (READ ONLY)\n');
fprintf(fid,'Mode: %s, snapshots: %d, rows: %d\n', ...
    report.mode,report.caseCount,report.barrierRows);
fprintf(fid,'Mass is an estimated P.mL per snapshot.\n');
fprintf(fid,'Numerical RK4/grid/directional difference are NOT verified upper bounds.\n');
fprintf(fid,'THEORETICAL CERTIFICATE: NOT_PROVEN\n');
fprintf(fid,'No production source modified.\n');fclose(fid);
if ~isempty(report.rows),writetable(struct2table(report.rows),fullfile(tmp,'PER_BOUND_CANDIDATE.csv'));end
fid=fopen(fullfile(tmp,'ERRORS.txt'),'w');
if fid<0,error('BOUND:IO','Cannot write errors.');end
if isempty(report.errors),fprintf(fid,'No errors.\n');else
    for i=1:numel(report.errors),fprintf(fid,'%s\n',report.errors{i});end
end
fclose(fid);
filename=fullfile(folder,'CBF_BREEDEN_BOUND_RESULTS.zip');
if isfile(filename),delete(filename);end
files={'SUMMARY.txt','ERRORS.txt'};
if ~isempty(report.rows),files{end+1}='PER_BOUND_CANDIDATE.csv';end
zip(filename,files,tmp);
fprintf('[BOUND] results: %s\n',filename);
end

function cleanup_temp(folder)
if isfolder(folder),rmdir(folder,'s');end
end
