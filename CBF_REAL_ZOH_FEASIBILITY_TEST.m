function report = CBF_REAL_ZOH_FEASIBILITY_TEST(action,varargin)
% CBF_REAL_ZOH_FEASIBILITY_TEST  TEST-ONLY, non-actuating scheme-01 diagnosis.
% NO changes to CBF_REAL_SCHEME_01_JOINT_ZOH, HLC, DISPATCH, INIT or Sim.
%
% Install next to mainGUI.m; snapshots and reports overwrite in that folder.
% At a MATLAB debug stop at CBFReal:NoFeasibleInput, in HLC.do's workspace:
%   CBF_REAL_ZOH_FEASIBILITY_TEST('capture',obj,x,tmp,P,Param.dt,varargin{:})
% Then dbquit; run in normal command window:
%   a=CBF_REAL_ZOH_FEASIBILITY_TEST('analyze');
%   b=CBF_REAL_ZOH_FEASIBILITY_TEST('sweep');
%   CBF_REAL_ZOH_FEASIBILITY_TEST('theory');
%
% The original safety math's local helper functions are bundled VERBATIM
% at the bottom of this independent test, not installed over the original.
% 'sourcecheck' rejects version drift in those helper functions.
if nargin<1 || isempty(action),action='help';end
action=lower(char(action));
folder=fileparts(mfilename('fullpath'));
paths={fullfile(folder,'CBF_REAL_ZOH_LAST_FAILURE.mat'), ...
    fullfile(folder,'CBF_REAL_ZOH_TEST_RESULTS.mat'), ...
    fullfile(folder,'CBF_REAL_ZOH_SWEEP_RESULTS.mat'), ...
    fullfile(folder,'CBF_REAL_ZOH_THEORY_RESULTS.mat'), ...
    fullfile(folder,'CBF_REAL_ZOH_ROOTCAUSE_RESULTS.mat')};
report=[];
switch action
    case 'help'
        test_print_help(folder);
    case 'sourcecheck'
        report=test_sourcecheck(folder);
    case 'capture'
        snapshot=test_capture_from_debug(varargin{:});
        test_validate_snapshot(snapshot);
        save(paths{1},'snapshot','-v7');
        fprintf('[CBF TEST ONLY] Failure snapshot saved (overwrite): %s\n',paths{1});
        report=snapshot;
    case 'captureagent'
        snapshot=test_capture_from_agent(varargin{:});
        test_validate_snapshot(snapshot);
        save(paths{1},'snapshot','-v7');
        fprintf('[CBF TEST ONLY] Failure snapshot saved (overwrite): %s\n',paths{1});
        report=snapshot;
    case {'analyze','sweep','theory','verify','rootcause','all'}
        test_prepare_dependencies(folder);
        reportSource=test_sourcecheck(folder);
        if ~reportSource.exactHelperMatch
            error('CBFTest:SourceChanged', ['Original scheme helper functions differ ' ...
                'from this test copy. Restore the supplied original scheme, or ' ...
                'regenerate a TEST-ONLY file from the currently used scheme.']);
        end
        if ~isfile(paths{1})
            error('CBFTest:SnapshotMissing', ['No snapshot. CBF is deliberately ' ...
                'unchanged: set dbstop if error CBFReal:NoFeasibleInput, run Sim, ' ...
                'then at the HLC error line call:\n' ...
                'CBF_REAL_ZOH_FEASIBILITY_TEST(''capture'',obj,x,tmp,P,Param.dt,varargin{:})']);
        end
        S=load(paths{1},'snapshot');
        if ~isfield(S,'snapshot'),error('CBFTest:SnapshotFormat','Missing snapshot.');end
        s=S.snapshot;test_validate_snapshot(s);
        switch action
            case 'analyze'
                report=test_analyze(s);
                save(paths{2},'report','-v7');
                fprintf('[CBF TEST ONLY] Overwrote %s\n',paths{2});
            case 'sweep'
                report=test_sweep(s);
                save(paths{3},'report','-v7');
                fprintf('[CBF TEST ONLY] Overwrote %s\n',paths{3});
            case 'theory'
                report=test_theory(s);
                save(paths{4},'report','-v7');
                fprintf('[CBF TEST ONLY] Overwrote %s\n',paths{4});
            case 'verify'
                report=test_verify(s);
            case 'rootcause'
                report=test_rootcause(s);
                save(paths{5},'report','-v7');
                fprintf('[CBF TEST ONLY] Overwrote %s\n',paths{5});
            case 'all'
                a=test_analyze(s);report=a;save(paths{2},'report','-v7');
                b=test_sweep(s);report=b;save(paths{3},'report','-v7');
                c=test_theory(s);report=c;save(paths{4},'report','-v7');
                d=test_rootcause(s);report=d;save(paths{5},'report','-v7');
                report=struct('analyze',a,'sweep',b,'theory',c,'rootcause',d);
        end
    case 'clean'
        for k=1:numel(paths)
            if isfile(paths{k}),delete(paths{k});fprintf('[CBF TEST ONLY] deleted %s\n',paths{k});end
        end
    otherwise
        error('CBFTest:Action','Unknown action %s; use help.',action);
end
end

function test_print_help(folder)
fprintf(['\nCBF TEST-ONLY: original CBF is untouched.\n' ...
 '1. dbstop if error CBFReal:NoFeasibleInput\n' ...
 '2. Run original Sim. At debugger stop in HLC.do:\n' ...
 '   CBF_REAL_ZOH_FEASIBILITY_TEST(''capture'',obj,x,tmp,P,Param.dt,varargin{:})\n' ...
 '3. dbquit\n' ...
 '4. CBF_REAL_ZOH_FEASIBILITY_TEST(''verify'')\n' ...
 '5. CBF_REAL_ZOH_FEASIBILITY_TEST(''analyze'')\n' ...
 '6. CBF_REAL_ZOH_FEASIBILITY_TEST(''sweep'')\n' ...
 '7. CBF_REAL_ZOH_FEASIBILITY_TEST(''theory'')\n' ...
 'Or use captureagent(app.agent(1),0.025) if app survives a stopped run.\n' ...
 'Results are overwritten next to mainGUI.m: %s\n\n'],folder);
end

function snapshot=test_capture_from_debug(ctrl,x,uNom,Praw,dt,varargin)
% ctr is the existing HLC object at its (unmodified) error line.
if ~isobject(ctrl) || ~isprop(ctrl,'self') || ~isprop(ctrl,'result')
    error('CBFTest:Capture','Pass HLC object obj as first argument while debugging HLC.do.');
end
if ~isstruct(ctrl.result) || ~isfield(ctrl.result,'cbf')
    error('CBFTest:Capture','No CBF failure info on obj.result.cbf. Stop at the NoFeasibleInput error.');
end
failed=ctrl.result.cbf;
if ~isfield(failed,'success') || failed.success
    error('CBFTest:Capture','obj.result.cbf is not a CBF failure.');
end
snapshot=test_pack_capture(ctrl.self,x,uNom,Praw,dt,failed,varargin{:});
end

function snapshot=test_capture_from_agent(agent,dt,varargin)
% Optional post-error capture when the Sim app object remains available.
if ~isobject(agent) || ~isprop(agent,'controller') || ~isprop(agent,'estimator')
    error('CBFTest:Capture','Pass the actual agent object.');
end
r=agent.controller.result;
if ~isstruct(r) || ~isfield(r,'cbf') || ~isfield(r,'tmp') || ...
        ~isfield(r.cbf,'success') || r.cbf.success
    error('CBFTest:Capture','Agent does not hold a failed CBF result and pre-CBF nominal input.');
end
z=agent.estimator.result.state;
pL=z.pL;
if isprop(z,'pT')
    pT=z.pT;
else
    v=pL-z.p;pT=v/max(norm(v),eps);
end
x=[z.getq('compact');z.w;pL;z.vL;pT;z.wL];
Praw=[agent.parameter.get(["mass","jx","jy","jz","gravity", ...
    "loadmass","cableL"]),0,0];
snapshot=test_pack_capture(agent,x,r.tmp,Praw,dt,r.cbf,varargin{:});
end

function s=test_pack_capture(agent,x,uNom,Praw,dt,failed,varargin)
if ~isprop(agent,'sensor') || ~isfield(agent.sensor.result,'cbfObservedWorld')
    error('CBFTest:Capture','Live spherical sensor data unavailable.');
end
res=agent.sensor.result;
if ~isfield(res,'cbfSensorTime')
    error('CBFTest:Capture','Spherical sensor time unavailable.');
end
phase='';
if isfield(failed,'phase'),phase=char(failed.phase);end
if numel(varargin)>=2 && (ischar(varargin{2}) || isstring(varargin{2}))
    phase=char(varargin{2});
elseif ~isempty(varargin) && (ischar(varargin{1}) || isstring(varargin{1}))
    phase=char(varargin{1});
end
s=struct('x',double(x(:)),'uNom',double(uNom(:)), ...
    'Praw',double(Praw(:)),'dt',double(dt), ...
    'observed',res.cbfObservedWorld,'sensorTime',double(res.cbfSensorTime), ...
    'phase',phase,'failedInfo',failed, ...
    'captureTime',datestr(now,30), ...
    'captureNote','From actual failing HLC; no production code modification', ...
    'cfg',struct('cbfId',1,'dt',double(dt)));
fprintf('[CBF TEST ONLY] Captured t=%.3f phase=%s status=%s mL=%.6g Tnom=%.6g\n', ...
    s.sensorTime,phase,char(failed.status),s.Praw(6),s.uNom(1));
end

function test_validate_snapshot(s)
fields={'x','uNom','Praw','dt','observed','sensorTime','phase','failedInfo'};
for j=1:numel(fields)
    if ~isfield(s,fields{j}),error('CBFTest:SnapshotFormat','Missing %s.',fields{j});end
end
validateattributes(s.x,{'double'},{'vector','numel',19,'real','finite'});
validateattributes(s.uNom,{'double'},{'vector','numel',4,'real','finite'});
validateattributes(s.Praw,{'double'},{'vector','numel',9,'real','finite'});
validateattributes(s.dt,{'double'},{'scalar','positive','finite'});
if ~isstruct(s.observed) || ~isfield(s.observed,'parts') || ...
        ~isstruct(s.observed.parts) || ~isfield(s.observed,'ground')
    error('CBFTest:SnapshotFormat','Observed parts or ground are missing.');
end
if ~isscalar(s.sensorTime) || ~isfinite(s.sensorTime)
    error('CBFTest:SnapshotFormat','Invalid time.');
end
end

function test_prepare_dependencies(folder)
% The test lives NEXT TO mainGUI.m. Find the original CBF scheme elsewhere
% under that project root; do not modify, replace or copy it.
schemeFile=test_find_original_scheme(folder);
schemeDir=fileparts(schemeFile);
p=which('CBF_REAL_SCHEME_01_JOINT_ZOH');
if isempty(p)
    addpath(schemeDir,'-begin');
    p=which('CBF_REAL_SCHEME_01_JOINT_ZOH');
end
if ~test_samepath(p,schemeFile)
    error('CBFTest:PathShadowed', ...
        ['The active CBF scheme is not the original found under mainGUI.\n' ...
         'Active: %s\nExpected: %s\n' ...
         'Check: which CBF_REAL_SCHEME_01_JOINT_ZOH -all'],p,schemeFile);
end
if exist('CBF_COMMON_MODEL','file')==2 && ...
        exist('CBF_COMMON_GEOMETRY','file')==2 && ...
        exist('CBF_REAL_ZOH_DIAG_EVAL','file')==2
    return
end
ancestor=folder;genRoot='';
for i=1:10
    opts={fullfile(ancestor,'CBF_GENERATOR'), ...
        fullfile(ancestor,'model','substance','CBF_GENERATOR'), ...
        fullfile(ancestor,'substance','CBF_GENERATOR')};
    for j=1:numel(opts)
        if isfile(fullfile(opts{j},'COMMON','CBF_COMMON_MODEL.m'))
            genRoot=opts{j};break
        end
    end
    if ~isempty(genRoot),break;end
    parent=fileparts(ancestor);if strcmp(parent,ancestor),break;end
    ancestor=parent;
end
if isempty(genRoot)
    % Support layouts where the generator is nested under the mainGUI root.
    hits=dir(fullfile(folder,'**','CBF_COMMON_MODEL.m'));
    for j=1:numel(hits)
        guess=fileparts(hits(j).folder);
        if isfile(fullfile(guess,'COMMON','CBF_COMMON_GEOMETRY.m')) && ...
                isfile(fullfile(guess,'REAL','HOCBF_ZOH_THRUST', ...
                    'GENERATED','CBF_REAL_ZOH_DIAG_EVAL.m'))
            genRoot=guess;break
        end
    end
end
if isempty(genRoot)
    error('CBFTest:GeneratorMissing', ...
        ['Cannot locate CBF_GENERATOR under the mainGUI project. ' ...
         'Run mainGUI first to set the original MATLAB paths.']);
end
addpath(fullfile(genRoot,'COMMON'), ...
    fullfile(genRoot,'REAL','HOCBF_ZOH_THRUST','GENERATED'),'-begin');
end

function r=test_sourcecheck(folder)
origFile=test_find_original_scheme(folder);
orig=fileread(origFile);mine=fileread([mfilename('fullpath') '.m']);
pat='(?m)^function \[c,ceq\]=zoh_sampled_data_nlcon\(';
io=regexp(orig,pat,'once');it=regexp(mine,pat,'once');
if isempty(io)||isempty(it),error('CBFTest:SourceFormat','Could not locate CBF helper functions.');end
a=regexprep(orig(io:end),'\r\n?','\n');
b=regexprep(mine(it:end),'\r\n?','\n');
normSource=regexprep(orig,'\r\n?','\n');
ch=double(normSource);
checksum=mod(sum((1:numel(ch)).*ch),1000000007);
uploadedOriginal=(numel(ch)==32220 && checksum==646041683);
r=struct('originalFile',origFile,'testFile',[mfilename('fullpath') '.m'], ...
    'exactHelperMatch',strcmp(strtrim(a),strtrim(b)), ...
    'matchesUserUploadedOriginal',uploadedOriginal, ...
    'originalSourceBytes',numel(orig),'codeChecksum',checksum, ...
    'comparison','Full exact text of unchanged local helper functions');
fprintf('[CBF TEST ONLY] Source-helper match: %d, entire uploaded version match: %d (%s)\n', ...
    r.exactHelperMatch,uploadedOriginal,origFile);
if ~uploadedOriginal
    warning('CBFTest:SchemeChanged',['Main scheme differs from the uploaded ' ...
        'original baseline. Test copies original baseline defaults, so ' ...
        'VERIFY must be reviewed before interpreting the sweep.']);
end
end

function schemeFile=test_find_original_scheme(folder)
% Locate the ORIGINAL scheme under the folder holding mainGUI.m. This
% deliberately never writes to the original code tree.
name='CBF_REAL_SCHEME_01_JOINT_ZOH.m';
opts={fullfile(folder,'HLC_CBF_CONTROLLER','CBF_REAL','SCHEMES',name), ...
    fullfile(folder,'model','substance','HLC_CBF_CONTROLLER','CBF_REAL','SCHEMES',name), ...
    fullfile(folder,'model','HLC_CBF_CONTROLLER','CBF_REAL','SCHEMES',name), ...
    fullfile(folder,'CBF_REAL','SCHEMES',name)};
for i=1:numel(opts)
    if isfile(opts{i}),schemeFile=opts{i};return;end
end
hits=dir(fullfile(folder,'**',name));
if isempty(hits)
    error('CBFTest:SourceMissing', ...
        'Original scheme not found below the mainGUI folder: %s',folder);
end
if numel(hits)==1
    schemeFile=fullfile(hits(1).folder,hits(1).name);return
end
% Prefer the active scheme only if it is one of the files in THIS project.
active=which('CBF_REAL_SCHEME_01_JOINT_ZOH');
for i=1:numel(hits)
    candidate=fullfile(hits(i).folder,hits(i).name);
    if test_samepath(active,candidate),schemeFile=candidate;return;end
end
error('CBFTest:AmbiguousScheme', ...
    ['Found %d scheme copies below mainGUI; cannot select safely. ' ...
     'Set the project MATLAB path and check which ... -all.'],numel(hits));
end

function tf=test_samepath(a,b)
a=strrep(char(a),'/',filesep);b=strrep(char(b),'/',filesep);
if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end

function o=test_defaults()
o=struct('gamma',1.5,'weights',[1;1;1;1], ...
    'scales',[10;1;1;1],'algorithm','sqp', ...
    'maxIter',65,'maxEval',400,'fdStep',[1e-3;0.05;0.05;0.05], ...
    'initialGuess',[],'massMultiplier',1,'dtMultiplier',1, ...
    'geometryOffsetM',0,'fifthBoundMultiplier',1);
end

function o=test_override(o,delta)
fn=fieldnames(delta);
for j=1:numel(fn),o.(fn{j})=delta.(fn{j});end
end

function ctx=test_context(s,opt)
p=double(s.Praw(:));p(6)=p(6)*opt.massMultiplier;
if p(6)<0,error('CBFTest:Mass','Negative payload mass in test case.');end
P=struct('m',p(1),'J',p(2:4),'gravity',p(5),'mL',p(6), ...
    'cableL',p(7),'dstx',p(8),'dsty',p(9));
G=zoh_geometry(P,s.observed);
G.includeGround=G.includeGround && strcmp(char(s.phase),'f');
if opt.geometryOffsetM~=0
    G.droneSafetyRadius=G.droneSafetyRadius+opt.geometryOffsetM;
    G.loadSafetyRadius=G.loadSafetyRadius+opt.geometryOffsetM;
    G.cableAxisShort=G.cableAxisShort+opt.geometryOffsetM;
    G.cableAxisLong=G.cableAxisLong+opt.geometryOffsetM;
    if min([G.droneSafetyRadius,G.loadSafetyRadius, ...
            G.cableAxisShort,G.cableAxisLong])<=0
        error('CBFTest:Geometry','Safety shape became invalid.');
    end
end
planes=zoh_make_planes(s.x(:),s.observed.parts,P,G);
[h0,h1]=zoh_plane_initial(s.x(:),P,G,planes);
ctx=struct('x',s.x(:),'P',P,'G',G,'planes',planes, ...
    'h0',h0,'h1',h1,'dt',s.dt*opt.dtMultiplier, ...
    'massRange',[P.mL,P.mL],'opt',opt, ...
    'rho',exp(-opt.gamma*s.dt*opt.dtMultiplier));
end

function [g,branch]=test_margins(u,ctx)
if ctx.opt.fifthBoundMultiplier==1
    [g,branch]=zoh_support_interval_margins(u(:),ctx.x,ctx.P,ctx.G, ...
        ctx.planes,ctx.dt,ctx.massRange);
    return
end
[acc,~]=test_accel_bounds(u,ctx);
jet=test_taylor_bounds(u,ctx,ctx.opt.fifthBoundMultiplier);
g=acc;branch=repmat({'SUPPORT_ACCELERATION_BOUND'},numel(acc),1);
for i=1:numel(g)
    if ctx.planes(i).body~=2 && jet(i)>g(i)
        g(i)=jet(i);branch{i}='ZOH_TAYLOR4_FIFTH_BOUND_ASSUMED';
    end
end
end

function [acc,branch]=test_accel_bounds(u,ctx)
% Identical acceleration-only branch of the original condition.
x=ctx.x;P=ctx.P;G=ctx.G;planes=ctx.planes;dt=ctx.dt;
if isempty(planes),acc=zeros(0,1);branch=cell(0,1);return;end
T=abs(u(1));m=P.m;L=P.cableL;
Omega=norm(x(17:19))+(T/(m*L))*dt;
Aload=P.gravity+norm([P.dstx;P.dsty])+ ...
    (T+m*L*Omega^2)/(m+ctx.massRange(1));
pt2=T/(m*L)+Omega^2;
cf=[1 G.cableCenterFromLoad 0];
acc=zeros(numel(planes),1);
branch=repmat({'SUPPORT_ACCELERATION_BOUND'},numel(planes),1);
for j=1:numel(planes)
    k=planes(j).body;c=cf(k);
    shapeRate=0;shapeDot0=0;
    if k==2
        delta=G.cableAxisLong^2-G.cableAxisShort^2;
        n=planes(j).n;y=n.'*x(14:16);
        ydot=n.'*cross(x(17:19),x(14:16));
        shapeDot0=delta*y*ydot/sqrt(G.cableAxisShort^2+delta*y^2);
        shapeRate=sqrt(delta)*Omega;
    end
    Ac=Aload+c*L*pt2;
    lowerEnd=ctx.h0(j)+(ctx.h1(j)+shapeDot0-shapeRate)*dt-0.5*Ac*dt^2;
    acc(j)=min(ctx.h0(j),lowerEnd);
end
end

function jet=test_taylor_bounds(u,ctx,multiplier)
% Original Taylor-4 method with diagnostic multiplier on M5 ONLY.
% multiplier <1 intentionally weakens an unverified remainder envelope:
% never interpret such a case as a valid safety certificate.
planes=ctx.planes;jet=-inf(numel(planes),1);
if isempty(planes) || ~any([planes.body]~=2),return;end
Gjet=ctx.G;Gjet.includeGround=false;
J=CBF_REAL_ZOH_DIAG_EVAL(ctx.x,u(:),struct([]),ctx.P,Gjet,4);
M5=zoh_plane_fifth_bounds(ctx.x,u(:),ctx.P,ctx.G,ctx.dt);
for j=1:numel(planes)
    body=planes(j).body;
    if body==2,continue;end
    coeff=zoh_plane_series(J.stateTaylor,planes(j),ctx.P,ctx.G,4);
    if ~isfinite(M5(body)) || M5(body)<0,continue;end
    minorant=[coeff(:).' -multiplier*M5(body)/factorial(5)];
    jet(j)=zoh_poly_min(minorant,ctx.dt);
end
end

function [r,g,branch]=test_residual(u,ctx)
[g,branch]=test_margins(u,ctx);
r=g-ctx.rho*ctx.h0;
end

function [c,ceq]=test_nlcon(u,ctx)
r=test_residual(u,ctx);
c=-r(:);ceq=[];
end

function [u,info]=test_call_original(s)
agent=struct('sensor',struct('result',struct( ...
    'cbfObservedWorld',s.observed,'cbfSensorTime',s.sensorTime)));
[u,info]=CBF_REAL_SCHEME_01_JOINT_ZOH(agent,struct('cbfId',1,'dt',s.dt), ...
    s.x(:),s.uNom(:),s.Praw(:),s.dt,[],char(s.phase));
end

function result=test_verify(s)
% Original untouched CBF called on a synthetic OBSERVATION-ONLY agent.
try
    [u,info]=test_call_original(s);
    ctx=test_context(s,test_defaults());
    minr=NaN;maxDiff=NaN;
    testedU=info.candidateInput;
    if isempty(testedU) && ~isempty(info.nominalIntervalMargins)
        testedU=min(max(s.uNom(:),[0;-1;-1;-1]),[20;1;1;1]);
    end
    if ~isempty(testedU)
        [r,g]=test_residual(testedU,ctx);
        if ~isempty(r)
            minr=min(r);
            if ~isempty(info.candidateInput) && ~isempty(info.intervalLowerBounds)
                maxDiff=max(abs(info.intervalLowerBounds(:)-g(:)));
            elseif ~isempty(info.nominalIntervalMargins)
                maxDiff=max(abs(info.nominalIntervalMargins(:)-g(:)));
            end
        end
    end
    result=struct('originalStatus',info.status,'originalSuccess',info.success, ...
        'originalInput',u,'originalInfo',info, ...
        'testComputedMinResidual',minr,'testVsOriginalLowerBoundMaxDiff',maxDiff, ...
        'capturedOriginalStatus',char(s.failedInfo.status));
    fprintf('[CBF VERIFY] captured=%s | original replay=%s | lower-bound diff=%.5g m\n', ...
        result.capturedOriginalStatus,info.status,maxDiff);
    if isfinite(maxDiff) && maxDiff>1e-9
        warning('CBFTest:NumericDisagreement','Constraint copy disagrees with the original scheme.');
    end
catch ME
    result=struct('originalStatus','REPLAY_EXCEPTION','exception', ...
        [ME.identifier ': ' ME.message]);
    warning('CBFTest:VerifyFailed','Original replay failed: %s',ME.message);
end
end

function candidates=test_candidates(uNom)
lb=[0;-1;-1;-1];ub=[20;1;1;1];
n=min(max(uNom(:),lb),ub);
Ts=unique([0 5 10 15 20 n(1)]);
base=[n(2:4),zeros(3,1)];
for k=1:3
    neg=n(2:4);pos=neg;neg(k)=-1;pos(k)=1;
    base=[base,neg,pos]; %#ok<AGROW>
end
base=[base,[-1;-1;-1],[1;1;1],[-1;1;-1],[1;-1;1]];
candidates=n;
for i=1:numel(Ts)
    for j=1:size(base,2)
        candidates(:,end+1)=[Ts(i);base(:,j)]; %#ok<AGROW>
    end
end
% Small deterministic low-discrepancy sample, includes simultaneous torque changes.
for k=1:48
    q=[test_vdc(k,2);test_vdc(k,3);test_vdc(k,5);test_vdc(k,7)];
    candidates(:,end+1)=lb+(ub-lb).*q; %#ok<AGROW>
end
candidates=unique(round(candidates.',10),'rows','stable').';
end

function v=test_vdc(i,b)
v=0;den=1;
while i>0
    den=den*b;v=v+mod(i,b)/den;i=floor(i/b);
end
end

function report=test_analyze(s)
ctx=test_context(s,test_defaults());
report=struct('kind','ORIGINAL_CONSTRAINT_FIXED_STATE_TEST', ...
    'capturedTime',s.sensorTime,'phase',s.phase, ...
    'failedStatus',char(s.failedInfo.status),'originalCheck',test_verify(s), ...
    'nominal',s.uNom,'dt',s.dt,'estimatedLoadMass',s.Praw(6), ...
    'constraintNames',{{ctx.planes.name}},'h0',ctx.h0,'hdot0',ctx.h1, ...
    'gammaValues',[0.5 1 1.5 2 3], ...
    'conclusion','Numerical scan: failure to find candidate does NOT prove global infeasibility');
if any(ctx.h0 < -1e-8)
    report.initialCertificateStatus='NO_INITIAL_SEPARATION_CERTIFICATE';
    report.criticalInitialPlanes={ctx.planes(ctx.h0<-1e-8).name};
    fprintf('[CBF ANALYZE] Initial negative plane gap. No input can change h(0) at this time. Not proof of collision.\n');
    return
end
if isempty(ctx.planes)
    report.initialCertificateStatus='NO_ACTIVE_CONSTRAINTS';
    fprintf('[CBF ANALYZE] No active obstacles/ground constraints.\n');
    return
end
report.initialCertificateStatus='INITIAL_PLANES_PASSED';
candidates=test_candidates(s.uNom);
N=size(candidates,2);gammas=report.gammaValues;
mins=NaN(N,numel(gammas));
branch=cell(N,1);critical=cell(N,1);failures=cell(N,1);
allG=NaN(numel(ctx.h0),N);
for i=1:N
    try
        [g,br]=test_margins(candidates(:,i),ctx);
        allG(:,i)=g;
        for j=1:numel(gammas)
            mins(i,j)=min(g-exp(-gammas(j)*ctx.dt)*ctx.h0);
        end
        [~,which]=min(g-ctx.rho*ctx.h0);
        critical{i}=ctx.planes(which).name;
        branch{i}=br{which};
    catch ME
        failures{i}=[ME.identifier ': ' ME.message];
    end
end
report.candidates=candidates;report.minimumResidualByGamma=mins;
report.minimumLowerBounds=allG;report.critical=critical;
report.activeBranch=branch;report.exceptions=failures;
report.feasibleByGamma=mins>=-1e-10;
report.feasibleCountsByGamma=sum(report.feasibleByGamma,1);
[bestVal,ibest]=max(mins(:,3));
if ~isfinite(bestVal)
    report.diagnostic='All candidate evaluations failed or produced nonfinite margins';
    fprintf('[CBF ANALYZE] Cannot rank candidates; see report.exceptions.\n');
    return
end
report.bestInput=candidates(:,ibest);report.bestResidual=bestVal;
report.bestCriticalPlane=critical{ibest};
report.feasibleAtOriginalGamma=find(report.feasibleByGamma(:,3));
% Compare the two hard lower-bound branches, rather than only the max.
[acc,~]=test_accel_bounds(report.bestInput,ctx);
jet=test_taylor_bounds(report.bestInput,ctx,1);
report.bestAccelerationBound=acc;
report.bestTaylorBound=jet;
report.branchGapTaylorMinusAcceleration=jet-acc;
% Sensitivity of the actual nonlinear constraint at the nominal and best input.
report.nominalJacobian=test_jacobian(min(max(s.uNom(:),[0;-1;-1;-1]),[20;1;1;1]),ctx);
report.bestJacobian=test_jacobian(report.bestInput,ctx);
report.maxAbsSensitivityNominal=max(abs(report.nominalJacobian),[],1);
report.maxAbsSensitivityBest=max(abs(report.bestJacobian),[],1);
fprintf('[CBF ANALYZE] fixed state %.3f s: %d candidate inputs\n',s.sensorTime,N);
for j=1:numel(gammas)
    fprintf('  gamma %.2f: %d/%d satisfy TEST constraint\n',gammas(j),report.feasibleCountsByGamma(j),N);
end
fprintf('  BEST at gamma=1.5: min residual %+.6g m, u=[%.5g %.5g %.5g %.5g], %s\n', ...
    bestVal,report.bestInput,report.bestCriticalPlane);
fprintf('  Max abs g sensitivity [T tx ty tz]: [%.4g %.4g %.4g %.4g]\n', ...
    report.maxAbsSensitivityNominal);
if bestVal>=-1e-10
    fprintf('[CBF ANALYZE] Existence of one numerical feasible point rules out global infeasibility at this captured state, conditional on implemented bound.\n');
else
    fprintf('[CBF ANALYZE] No scanned candidate feasible. This does NOT prove there is no feasible physical input.\n');
end
end

function J=test_jacobian(u,ctx)
lb=[0;-1;-1;-1];ub=[20;1;1;1];d=[0.01;0.05;0.05;0.05];
J=NaN(numel(ctx.h0),4);
for k=1:4
    plus=u;minus=u;
    plus(k)=min(ub(k),plus(k)+d(k));
    minus(k)=max(lb(k),minus(k)-d(k));
    if plus(k)==minus(k),continue;end
    gp=test_margins(plus,ctx);gm=test_margins(minus,ctx);
    J(:,k)=(gp(:)-gm(:))/(plus(k)-minus(k));
end
end

function row=test_run_solver(s,o,name)
row=struct('name',name,'status','','accepted',false, ...
    'minResidual',NaN,'maxViolation',NaN, ...
    'input',[NaN;NaN;NaN;NaN],'exitflag',NaN, ...
    'iterations',NaN,'evaluations',NaN,'elapsedMs',NaN, ...
    'critical','','usedSolver',false,'error','', ...
    'gamma',o.gamma,'massMultiplier',o.massMultiplier, ...
    'dtMultiplier',o.dtMultiplier,'geometryOffsetM',o.geometryOffsetM, ...
    'fifthBoundMultiplier',o.fifthBoundMultiplier);
start=tic;
try
    ctx=test_context(s,o);
    if any(ctx.h0 < -1e-8)
        row.status='NO_INITIAL_SEPARATION_CERTIFICATE';row.elapsedMs=1000*toc(start);return
    end
    lb=[0;-1;-1;-1];ub=[20;1;1;1];nom=s.uNom(:);
    u0=min(max(nom,lb),ub);
    if isempty(ctx.h0)
        row.input=u0;row.status='NO_ACTIVE_CONSTRAINTS';
        row.accepted=true;row.minResidual=Inf;row.elapsedMs=1000*toc(start);return
    end
    r0=test_residual(u0,ctx);
    inBounds=all(nom>=lb & nom<=ub);
    if min(r0)>=-1e-10
        row.input=u0;row.minResidual=min(r0);row.maxViolation=max(0,-min(r0));
        if inBounds,row.status='NOMINAL_FEASIBLE';else,row.status='CLIPPED_NOMINAL_FEASIBLE';end
        row.accepted=true;row.elapsedMs=1000*toc(start);return
    end
    if exist('fmincon','file')~=2 && exist('fmincon','file')~=6
        row.status='FMINCON_MISSING';row.elapsedMs=1000*toc(start);return
    end
    row.usedSolver=true;
    guess=u0;
    if ~isempty(o.initialGuess)
        guess=min(max(o.initialGuess(:),lb),ub);
    end
    fun=@(v)sum(o.weights(:).*((v(:)-nom)./o.scales(:)).^2);
    con=@(v)test_nlcon(v,ctx);
    optim=optimoptions('fmincon','Algorithm',char(o.algorithm),'Display','off', ...
        'MaxIterations',o.maxIter,'MaxFunctionEvaluations',o.maxEval, ...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-8, ...
        'FiniteDifferenceType','central', ...
        'FiniteDifferenceStepSize',o.fdStep(:), ...
        'TypicalX',[10;0.2;0.2;0.2]);
    [sol,~,ef,out]=fmincon(fun,guess,[],[],[],[],lb,ub,con,optim);
    row.exitflag=ef;
    if isfield(out,'iterations'),row.iterations=out.iterations;end
    if isfield(out,'funcCount'),row.evaluations=out.funcCount;end
    if numel(sol)~=4 || any(~isfinite(sol))
        row.status='NONFINITE_CANDIDATE';row.elapsedMs=1000*toc(start);return
    end
    row.input=sol(:);
    r=test_residual(sol,ctx);
    [row.minResidual,ic]=min(r);
    row.critical=ctx.planes(ic).name;
    row.maxViolation=max([0;-r(:);lb-sol(:);sol(:)-ub]);
    row.accepted=row.minResidual>=-1e-10 && ...
        all(sol(:)>=lb-1e-10) && all(sol(:)<=ub+1e-10);
    if row.accepted
        row.status='NUMERIC_FEASIBLE';
    else
        row.status='NUMERIC_UNSATISFIED';
    end
catch ME
    row.status='TEST_EXCEPTION';row.error=[ME.identifier ': ' ME.message];
end
row.elapsedMs=1000*toc(start);
end

function c=test_case(name,delta)
c=struct('name',name,'delta',delta);
end

function report=test_sweep(s)
o=test_defaults();
nom=min(max(s.uNom(:),[0;-1;-1;-1]),[20;1;1;1]);
cases=struct('name',{},'delta',{});
cases(end+1)=test_case('BASELINE',struct());
for gamma=[0.5 1 2 3]
    cases(end+1)=test_case(sprintf('GAMMA_%g',gamma),struct('gamma',gamma)); %#ok<AGROW>
end
cases(end+1)=test_case('WEIGHT_THRUST_100',struct('weights',[100;1;1;1]));
cases(end+1)=test_case('WEIGHT_THRUST_0p1',struct('weights',[0.1;1;1;1]));
cases(end+1)=test_case('WEIGHT_TORQUES_10',struct('weights',[1;10;10;10]));
cases(end+1)=test_case('WEIGHT_PITCH_10',struct('weights',[1;1;10;1]));
cases(end+1)=test_case('SCALE_THRUST_5',struct('scales',[5;1;1;1]));
cases(end+1)=test_case('SCALE_THRUST_20',struct('scales',[20;1;1;1]));
cases(end+1)=test_case('SOLVER_INTERIOR_POINT',struct('algorithm','interior-point'));
cases(end+1)=test_case('SOLVER_SQP_200ITER',struct('maxIter',200,'maxEval',1600));
cases(end+1)=test_case('SOLVER_FD_FINE',struct('fdStep',[1e-4;0.01;0.01;0.01]));
cases(end+1)=test_case('SOLVER_FD_COARSE',struct('fdStep',[1e-2;0.1;0.1;0.1]));
for signval=[-1 1]
    guess=nom;guess(3)=signval;
    cases(end+1)=test_case(sprintf('START_PITCH_%+d',signval),struct('initialGuess',guess)); %#ok<AGROW>
end
for T=[0 20]
    guess=nom;guess(1)=T;
    cases(end+1)=test_case(sprintf('START_T_%g',T),struct('initialGuess',guess)); %#ok<AGROW>
end
% Physical/model/sampling/geometry perturbations are sensitivity tests,
% NOT proofs of original-system feasibility or valid controller settings.
cases(end+1)=test_case('MODEL_ML_HALF',struct('massMultiplier',0.5));
cases(end+1)=test_case('MODEL_ML_DOUBLE',struct('massMultiplier',2));
if s.Praw(6)>0
    cases(end+1)=test_case('MODEL_ML_0p14_FROM_SIM',struct('massMultiplier',0.14/s.Praw(6)));
end
cases(end+1)=test_case('DT_HALF_HYPOTHETICAL',struct('dtMultiplier',0.5));
cases(end+1)=test_case('DT_DOUBLE_HYPOTHETICAL',struct('dtMultiplier',2));
cases(end+1)=test_case('GEOMETRY_PLUS_1CM',struct('geometryOffsetM',0.01));
cases(end+1)=test_case('GEOMETRY_MINUS_1CM',struct('geometryOffsetM',-0.01));
cases(end+1)=test_case('FIFTH_BOUND_X2',struct('fifthBoundMultiplier',2));
cases(end+1)=test_case('FIFTH_BOUND_X0p5_UNCERTIFIED',struct('fifthBoundMultiplier',0.5));
rows=repmat(test_run_empty_row(),numel(cases),1);
for i=1:numel(cases)
    opts=test_override(o,cases(i).delta);
    rows(i)=test_run_solver(s,opts,cases(i).name);
    fprintf('[CBF SWEEP %2d/%2d] %-34s feasible=%d residual=%+.5g exit=%g %s\n', ...
        i,numel(cases),rows(i).name,rows(i).accepted, ...
        rows(i).minResidual,rows(i).exitflag,rows(i).status);
end
report=struct('kind','TEST_ONLY_FIXED_STATE_FMINCON_SWEEP', ...
    'sourceTime',s.sensorTime,'originalGamma',1.5, ...
    'originalFailure',char(s.failedInfo.status), ...
    'cases',cases,'rows',rows, ...
    'acceptedSameConstraints',[], ...
    'note',['One-at-a-time parameter variations: weight/solver/seed ' ...
      'keep original feasibility set; gamma/dt/mass/geometry/fifth ' ...
      'change the constraint or model, and DO NOT validate original CBF.']);
for i=2:numel(cases)
    changes=fieldnames(cases(i).delta);
    unchanged=all(ismember(changes,{'weights','scales','algorithm', ...
        'maxIter','maxEval','fdStep','initialGuess'}));
    if unchanged && rows(i).accepted
        report.acceptedSameConstraints(end+1)=i; %#ok<AGROW>
    end
end
fprintf('[CBF SWEEP] %d of %d non-baseline cases found an input under IDENTICAL original constraints.\n', ...
    numel(report.acceptedSameConstraints),numel(cases)-1);
end

function row=test_run_empty_row()
row=struct('name','','status','','accepted',false, ...
    'minResidual',NaN,'maxViolation',NaN, ...
    'input',[NaN;NaN;NaN;NaN],'exitflag',NaN, ...
    'iterations',NaN,'evaluations',NaN,'elapsedMs',NaN, ...
    'critical','','usedSolver',false,'error','', ...
    'gamma',NaN,'massMultiplier',NaN, ...
    'dtMultiplier',NaN,'geometryOffsetM',NaN, ...
    'fifthBoundMultiplier',NaN);
end

function report=test_theory(s)
% A numerical diagnosis and explicit list of theoretical proof obligations.
ctx=test_context(s,test_defaults());
nom=min(max(s.uNom(:),[0;-1;-1;-1]),[20;1;1;1]);
report=struct('kind','ZOH_CBF_THEORY_DIAGNOSTICS', ...
    'sourceTime',s.sensorTime,'phase',s.phase,'originalStatus',char(s.failedInfo.status), ...
    'mLUsed',s.Praw(6),'uNom',s.uNom,'clippedNominal',nom, ...
    'inputUpper',[20;1;1;1],'inputLower',[0;-1;-1;-1], ...
    'h0',ctx.h0,'h1',ctx.h1,'lambda',[4;4;4;4], ...
    'hardConstraint','g_j(u) >= exp(-1.5*dt)*h_j(x_k)', ...
    'hardVsDiagnostic','psi1..psi4 are diagnostic only; mixed relative degree (thrust vs torque)', ...
    'provisionalProofs',{{ ...
     'TAYLOR fifth derivative majorant must bound the actual 19-state flow on the complete interval', ...
     'fixed separating normal must be a valid conservative certificate', ...
     'one-step feasibility does not imply recursive feasibility', ...
     'estimated payload mass is currently treated as exact', ...
     'obstacles outside sensor radius are not covered by this inequality', ...
     'discrete input switching is ZOH, not a smooth thrust/torque extension'}}, ...
    'numericalMetrics',struct());
if isempty(ctx.h0)
    fprintf('[CBF THEORY] No constraints in captured state.\n');return
end
required=ctx.rho*ctx.h0;
[g,branches]=test_margins(nom,ctx);
r=g-required;
[worst,iw]=min(r);
report.numericalMetrics=struct('required',required, ...
    'nominalLowerBound',g,'nominalResidual',r, ...
    'nominalWorstResidual',worst, ...
    'criticalConstraint',ctx.planes(iw).name, ...
    'criticalCertificateBranch',branches{iw}, ...
    'initialNegativeGap',any(ctx.h0 < -1e-8), ...
    'absoluteNominalOutsideBounds',max([0; ...
        [0;-1;-1;-1]-s.uNom(:);s.uNom(:)-[20;1;1;1]]));
index=find(ctx.h1<0 & ctx.h0>0);
ttc=nan(numel(ctx.h0),1);
ttc(index)=ctx.h0(index)./(-ctx.h1(index));
report.numericalMetrics.constantVelocityPlaneTTCEstimate=ttc;
report.numericalMetrics.testJacobians=test_jacobian(nom,ctx);
report.numericalMetrics.maxJacobianAbs=max(abs(report.numericalMetrics.testJacobians),[],1);
report.numericalMetrics.torqueResponseAtMaxMinusMin=NaN(numel(ctx.h0),3);
for k=2:4
    up=nom;down=nom;up(k)=1;down(k)=-1;
    gp=test_margins(up,ctx);gm=test_margins(down,ctx);
    report.numericalMetrics.torqueResponseAtMaxMinusMin(:,k-1)=gp(:)-gm(:);
end
report.numericalMetrics.maxTorqueResponse=max(abs( ...
    report.numericalMetrics.torqueResponseAtMaxMinusMin),[],1);
report.numericalMetrics.detectedParts=numel(s.observed.parts);
report.numericalMetrics.minInitialGap=min(ctx.h0);
% Search alternative separating normals at the frozen state. A better
% positive gap can falsify the claim that the original chosen plane is
% the only possible certificate; a negative search result proves nothing.
planeProbe=struct('name',{},'originalGap',{},'bestSampledGap',{}, ...
    'improvementM',{},'foundPositiveAlternative',{});
for k=1:numel(ctx.planes)
    z=ctx.planes(k);item=struct('name',z.name,'originalGap',ctx.h0(k), ...
        'bestSampledGap',NaN,'improvementM',NaN, ...
        'foundPositiveAlternative',false);
    if strcmp(z.kind,'obstacle')
        item.bestSampledGap=test_sample_best_plane(z,ctx);
        item.improvementM=item.bestSampledGap-ctx.h0(k);
        item.foundPositiveAlternative=(ctx.h0(k)<0 && item.bestSampledGap>1e-8);
    end
    planeProbe(end+1)=item; %#ok<AGROW>
end
report.numericalMetrics.alternativePlaneSearch=planeProbe;
if any([planeProbe.foundPositiveAlternative])
    fprintf('[CBF THEORY] Positive alternative support-plane gap found where original had negative: normal construction may be a cause.\n');
end
[acc,~]=test_accel_bounds(nom,ctx);jet=test_taylor_bounds(nom,ctx,1);
report.numericalMetrics.accelOnly=acc;
report.numericalMetrics.jetOnly=jet;
report.numericalMetrics.activeJet=(jet>acc);
% Falsification-oriented check: integrate the SAME CBF_COMMON_MODEL ODE
% at fixed candidate inputs, then compare computed interval lower bounds
% against min observed frozen-normal clearance. This is NOT interval proof.
probes=nom;
for T=[0 20]
    q=nom;q(1)=T;probes(:,end+1)=q; %#ok<AGROW>
end
for axis=2:4
    for edge=[-1 1]
        q=nom;q(axis)=edge;probes(:,end+1)=q; %#ok<AGROW>
    end
end
probes=unique(probes.','rows','stable').';
checks=struct('u',{},'minIntegratedH',{},'testLowerBound',{}, ...
    'maxOverstatementM',{},'criticalConstraint',{},'integrationFailed',{});
for i=1:size(probes,2)
    q=probes(:,i);
    chk=struct('u',q,'minIntegratedH',[], ...
        'testLowerBound',[],'maxOverstatementM',NaN, ...
        'criticalConstraint','','integrationFailed','');
    try
        [gq,~]=test_margins(q,ctx);
        hsim=test_integrate_min_clearance(q,ctx,100);
        [overstatement,where]=max(gq(:)-hsim(:));
        chk.minIntegratedH=hsim;
        chk.testLowerBound=gq;
        chk.maxOverstatementM=overstatement;
        chk.criticalConstraint=ctx.planes(where).name;
    catch ME
        chk.integrationFailed=[ME.identifier ': ' ME.message];
    end
    checks(end+1)=chk; %#ok<AGROW>
end
report.numericalMetrics.integrationChecks=checks;
nums=[checks.maxOverstatementM];nums=nums(isfinite(nums));
if ~isempty(nums)
    report.numericalMetrics.maximumObservedLowerBoundOverstatementM=max(nums);
    fprintf('  RK4 100-substep consistency check: worst (claimed lower - sampled minimum) = %+.6g m\n',max(nums));
    if max(nums)>1e-5
        warning('CBFTest:Counterexample',['Potential invalid interval bound: ' ...
            'claim exceeds sampled model clearance. Inspect integrationChecks.']);
    end
end
fprintf('[CBF THEORY] t=%.3f, dt=%.4f, mL_used=%.4g, observed parts=%d\n', ...
    s.sensorTime,ctx.dt,s.Praw(6),numel(s.observed.parts));
fprintf('  Worst nominal residual %+.6g m at %s, initial h=%.6g, hdot=%.6g\n', ...
    worst,ctx.planes(iw).name,ctx.h0(iw),ctx.h1(iw));
fprintf('  Torque endpoint delta g [x y z] max abs = [%.5g %.5g %.5g] m\n', ...
    report.numericalMetrics.maxTorqueResponse);
fprintf('  Active Taylor jet at %d of %d constraints (not independently certified).\n', ...
    sum(report.numericalMetrics.activeJet),numel(ctx.h0));
fprintf(['[CBF THEORY] Interpretation: negative support-plane gap is NOT collision proof.\n' ...
 '  Negative candidate residual is NOT physical infeasibility proof.\n' ...
 '  Weights cannot change feasibility. A gamma/dt/geometry/mass/remainder\n' ...
 '  change does change the modeled constraint or its assumptions.\n']);
end

function best=test_sample_best_plane(z,ctx)
% Unit-normal search. For obstacle gap, positive value suffices to certify
% separation at this instant. No negative-result optimality guarantee.
G=ctx.G;P=ctx.P;x=ctx.x;
if z.body==1
    Qb=G.droneSafetyRadius^2*eye(3);
elseif z.body==3
    Qb=G.loadSafetyRadius^2*eye(3);
else
    pT=x(14:16);
    Qb=G.cableAxisShort^2*eye(3)+ ...
        (G.cableAxisLong^2-G.cableAxisShort^2)*(pT*pT.');
end
d=z.currentPoint-z.center;
best=zoh_plane_gap(d,z.Q,Qb,z.n);
N=600;golden=pi*(3-sqrt(5));
for i=0:N-1
    zz=1-2*(i+0.5)/N;
    rr=sqrt(max(0,1-zz^2));
    ang=golden*i;
    v=[rr*cos(ang);rr*sin(ang);zz];
    best=max(best,zoh_plane_gap(d,z.Q,Qb,v));
end
end

function minH=test_integrate_min_clearance(u,ctx,N)
% Unmodified physical RHS. Numerical values are sampled, NOT guaranteed
% extrema. A significant g > observed min is evidence of a bound defect.
state=ctx.x(:);minH=inf(numel(ctx.h0),1);
h=ctx.dt/N;
for k=0:N
    [gap,~]=zoh_plane_initial(state,ctx.P,ctx.G,ctx.planes);
    minH=min(minH,gap(:));
    if k==N,break;end
    f1=CBF_COMMON_MODEL(state,u(:),ctx.P);
    f2=CBF_COMMON_MODEL(state+0.5*h*f1,u(:),ctx.P);
    f3=CBF_COMMON_MODEL(state+0.5*h*f2,u(:),ctx.P);
    f4=CBF_COMMON_MODEL(state+h*f3,u(:),ctx.P);
    state=state+(h/6)*(f1+2*f2+2*f3+f4);
    if any(~isfinite(state)),error('CBFTest:Integration','Nonfinite integrated state.');end
end
end

% ===== TEST-ONLY extension (2026-11-20): fixed-state root-cause diagnostics.
% Original controller, generated model and the verbatim helper section below
% are never written to or replaced by this test.
function report=test_rootcause(s)
ctx=test_context(s,test_defaults());
lb=[0;-1;-1;-1];ub=[20;1;1;1];
uNom=min(max(s.uNom(:),lb),ub);
[gNom,branchNom]=test_margins(uNom,ctx);
resNom=gNom-ctx.rho*ctx.h0;
[minNom,kcrit]=min(resNom);
if isempty(kcrit)
    error('CBFTest:NoConstraints','No active constraint in saved failure.');
end
h0=ctx.h0(kcrit);h1=ctx.h1(kcrit);required=ctx.rho*h0;
if h0<=0
    warning('CBFTest:NonpositiveH','Critical h0<=0, exponential threshold interpretations are limited.');
end
names={ctx.planes.name}.';
% Scan thrust and all extreme torque corners at every thrust grid point.
% This is a deterministic finite scan, not a proof of global infeasibility.
Tgrid=linspace(0,20,41);
sgn=[-1 1]; corners=zeros(3,8);n=0;
for i=1:2,for j=1:2,for k=1:2
    n=n+1;corners(:,n)=[sgn(i);sgn(j);sgn(k)];
end,end,end
scan=nan(4,41*9+1);scan(:,1)=uNom;
c=1;
for i=1:numel(Tgrid)
    c=c+1;scan(:,c)=[Tgrid(i);uNom(2:4)];
    for j=1:8
        c=c+1;scan(:,c)=[Tgrid(i);corners(:,j)];
    end
end
scan=unique(round(scan(:,1:c).',12),'rows','stable').';
N=size(scan,2);margins=NaN(numel(ctx.h0),N);
residuals=NaN(numel(ctx.h0),N);minres=NaN(1,N);
neededGamma=NaN(1,N);failed=false(1,N);
for i=1:N
    try
        gi=test_margins(scan(:,i),ctx);
        margins(:,i)=gi(:);
        ri=gi(:)-ctx.rho*ctx.h0(:);
        residuals(:,i)=ri;
        minres(i)=min(ri);
        if all(gi(:)>0) && all(ctx.h0(:)>0)
            neededGamma(i)=max(log(ctx.h0(:)./gi(:)))/ctx.dt;
        end
    catch
        failed(i)=true;
    end
end
valid=find(isfinite(minres));
if isempty(valid)
    error('CBFTest:ScanFailed','All grid evaluations failed.');
end
[bestResidual,whichBest]=max(minres);
bestInput=scan(:,whichBest);
bestG=margins(:,whichBest);
% Critical-surface sensitivity MUST be distinguished from max sensitivity
% across unrelated constraints (which can be much larger).
J=test_jacobian(uNom,ctx);
torqueDelta=NaN(1,3);
for j=2:4
    plus=uNom;minus=uNom;plus(j)=1;minus(j)=-1;
    gp=test_margins(plus,ctx);gm=test_margins(minus,ctx);
    torqueDelta(j-1)=gp(kcrit)-gm(kcrit);
end
% Independently integrate the SAME frozen-state ODE over just one ZOH step.
% No assumption that an input is held for multiple future control steps.
probes=[uNom,bestInput,[0;uNom(2:4)],[20;uNom(2:4)]];
probes=unique(round(probes.',12),'rows','stable').';
checks=repmat(struct('u',zeros(4,1),'minH',[],'endH',[], ...
    'endHdot',[],'bound',[],'criticalBoundConservatismM',NaN, ...
    'equivalentCriticalAcceleration',NaN,'error',''),1,size(probes,2));
for i=1:size(probes,2)
    checks(i).u=probes(:,i);
    try
        [hMin,hEnd,hDotEnd]=test_rootcause_integrate(probes(:,i),ctx,200);
        g=test_margins(probes(:,i),ctx);
        checks(i).minH=hMin;checks(i).endH=hEnd;
        checks(i).endHdot=hDotEnd;checks(i).bound=g;
        checks(i).criticalBoundConservatismM=hMin(kcrit)-g(kcrit);
        checks(i).equivalentCriticalAcceleration= ...
            2*(hEnd(kcrit)-h0-h1*ctx.dt)/ctx.dt^2;
    catch ME
        checks(i).error=[ME.identifier ': ' ME.message];
    end
end
thresholdEqAccel=2*(required-h0-h1*ctx.dt)/ctx.dt^2;
if h0>0 && gNom(kcrit)>0
    gammaNeededNom=log(h0/gNom(kcrit))/ctx.dt;
else
    gammaNeededNom=NaN;
end
% If all constraints are positive, this is the sampled input's exact
% required gamma for the implemented fixed-state inequality.
positiveScan=neededGamma(isfinite(neededGamma));
if isempty(positiveScan),minGammaSampled=NaN;
else,minGammaSampled=min(positiveScan);end
report=struct('kind','TEST_ONLY_FIXED_SNAPSHOT_ROOTCAUSE', ...
    'sourceTime',s.sensorTime,'criticalName',names{kcrit}, ...
    'constraintNames',{names},'criticalIndex',kcrit, ...
    'gamma',ctx.opt.gamma,'dt',ctx.dt, ...
    'initialMargins',ctx.h0,'initialMarginRates',ctx.h1, ...
    'nominalInput',uNom,'nominalIntervalLowerBounds',gNom, ...
    'nominalCertificateBranches',{branchNom}, ...
    'nominalResiduals',resNom,'nominalMinResidual',minNom, ...
    'criticalInitialH',h0,'criticalInitialHdot',h1, ...
    'criticalContinuousCBFMargin',h1+ctx.opt.gamma*h0, ...
    'criticalRequiredEndOrMinimumH',required, ...
    'criticalNominalLowerH',gNom(kcrit), ...
    'criticalNecessaryConstantAccelerationProxy',thresholdEqAccel, ...
    'criticalNominalEquivalentAccelerationProxy', ...
        2*(gNom(kcrit)-h0-h1*ctx.dt)/ctx.dt^2, ...
    'criticalJacobianPerInput',J(kcrit,:), ...
    'criticalTorqueFullSpanDifference',torqueDelta, ...
    'nominalGammaRequirementCriticalOnly',gammaNeededNom, ...
    'bestGammaAmongGrid',minGammaSampled, ...
    'scannedInputs',scan,'scannedMargins',margins, ...
    'scannedResiduals',residuals,'scannedMinResiduals',minres, ...
    'scannedGammaThresholds',neededGamma, ...
    'scannedExceptions',failed,'scannedInputCount',N, ...
    'scannedFeasibleCount',sum(minres>=-1e-10), ...
    'bestInput',bestInput,'bestResidual',bestResidual, ...
    'bestConstraintMargins',bestG, ...
    'checks',checks, ...
    'notes',{{ ...
      'Only one failed state is available; cannot infer what the CBF did at earlier steps', ...
      'Grid sampling never establishes global infeasibility', ...
      'Equivalent acceleration is a finite-interval proxy, not instantaneous physical acceleration', ...
      'Numerical RK4 agreement tests the bound on these states/inputs, not a formal fifth-derivative proof', ...
      'Changing gamma or plant parameters changes the safety condition; it does not fix original infeasibility', ...
      'Future braking/avoiding policies and recursive feasibility are not evaluated'}});
fprintf('[CBF ROOTCAUSE] t=%.3f, critical=%s, h=%.9g, hdot=%.9g\n', ...
    s.sensorTime,names{kcrit},h0,h1);
fprintf('  instantaneous hdot+gamma*h=%.8g m/s; nominal 25ms residual=%+.9g m\n', ...
    h1+ctx.opt.gamma*h0,minNom);
fprintf('  required equivalent acceleration=%.7g m/s^2 vs nominal certificate=%.7g m/s^2\n', ...
    thresholdEqAccel,report.criticalNominalEquivalentAccelerationProxy);
fprintf('  critical input partials [T tx ty tz]=[%.7g %.7g %.7g %.7g] (m/input unit)\n', ...
    J(kcrit,:));
fprintf('  %d finite-grid input candidates, %d feasible; best residual=%+.8g m, best u=[%.5g %.5g %.5g %.5g]\n', ...
    N,report.scannedFeasibleCount,bestResidual,bestInput);
fprintf('  gamma needed at nominal for critical=%.8g 1/s; minimum among sampled full constraints=%.8g 1/s\n', ...
    gammaNeededNom,minGammaSampled);
for i=1:numel(checks)
    if isempty(checks(i).error)
        fprintf('  RK4 u=[%.5g %.5g %.5g %.5g]: critical true sampled min=%.9g, bound=%.9g, conservatism=%.3g m\n', ...
            checks(i).u,checks(i).minH(kcrit),checks(i).bound(kcrit), ...
            checks(i).criticalBoundConservatismM);
    else
        fprintf('  RK4 probe %d FAILED: %s\n',i,checks(i).error);
    end
end
fprintf('[CBF ROOTCAUSE] One-step fixed-state numerical diagnosis only; NOT a proof of full-input or future-step viability.\n');
end

function [minH,endH,endHdot]=test_rootcause_integrate(u,ctx,N)
% Same ODE as original generator; fixed ZOH input on [0, dt].
state=ctx.x(:);minH=inf(numel(ctx.h0),1);
step=ctx.dt/N;
for k=0:N
    [hk,~]=zoh_plane_initial(state,ctx.P,ctx.G,ctx.planes);
    minH=min(minH,hk(:));
    if k==N,break;end
    f1=CBF_COMMON_MODEL(state,u,ctx.P);
    f2=CBF_COMMON_MODEL(state+step*f1/2,u,ctx.P);
    f3=CBF_COMMON_MODEL(state+step*f2/2,u,ctx.P);
    f4=CBF_COMMON_MODEL(state+step*f3,u,ctx.P);
    state=state+step*(f1+2*f2+2*f3+f4)/6;
    if any(~isfinite(state)),error('CBFTest:Integration','Nonfinite state.');end
end
[endH,endHdot]=zoh_plane_initial(state,ctx.P,ctx.G,ctx.planes);
end

% ===== The following local helpers are verbatim, UNMODIFIED copies of
% the original CBF_REAL_SCHEME_01_JOINT_ZOH.m helpers.
% Exact textual match is checked by the sourcecheck action.
function [c,ceq]=zoh_sampled_data_nlcon(u,x,P,G,planes,dt,massRange,h0,rho)
% The SAME predicate is checked for the nominal, SQP candidates, and the
% applied input. The lower bound is over the entire one-step ZOH interval.
g=zoh_support_interval_margins(u,x,P,G,planes,dt,massRange);
c=rho*h0(:)-g(:);
ceq=[];
end

function [stage,derivatives,psi2Lower]=zoh_stage_evaluate(u,x,P,G,planes,dt,lambda,computePsi2Diagnostic)
% h(t): signed support-plane gap in metres (one frozen separating normal).
% psi_k = (D+lambda_k)...(D+lambda_1) h, for k=1,...,4.
% Relative degrees vary: T generally enters h''; tau enters h''''.
% None of psi2..psi4 is used as a hard constraint for mixed-input ZOH.
% The optional psi2(t) estimate may be extremely conservative for thin
% cable ellipsoids; it is diagnostic only, not an input constraint.
if nargin<8,computePsi2Diagnostic=true;end
n=numel(planes);
stage=zeros(5,n);derivatives=zeros(5,n);psi2Lower=NaN(n,1);
if n==0,return;end
Gjet=G;Gjet.includeGround=false;
J=CBF_REAL_ZOH_DIAG_EVAL(x,u(:),struct([]),P,Gjet,4);
if computePsi2Diagnostic
    M5=zoh_plane_fifth_bounds(x,u,P,G,dt);
end
for k=1:n
    coeff=zoh_plane_series(J.stateTaylor,planes(k),P,G,4);
    derivatives(:,k)=factorial(0:4).'.*coeff(:);
    % Different gains are allowed at each order; no factorial shortcuts.
    for ord=0:4
        poly=[1];
        for j=1:ord,poly=conv(poly,[lambda(j) 1]);end
        stage(ord+1,k)=poly*derivatives(1:ord+1,k);
    end
    if computePsi2Diagnostic
        % Diagnostic only: the 5th-derivative envelope may be excessively
        % conservative, especially for a narrow cable ellipsoid.
        lamSum=lambda(1)+lambda(2);
        lamProd=lambda(1)*lambda(2);
        h=derivatives(:,k);
        B5=M5(planes(k).body);
        B4=abs(h(5))+B5*dt;
        B3=abs(h(4))+abs(h(5))*dt+0.5*B5*dt^2;
        psi2ThirdBound=B5+lamSum*B4+lamProd*B3;
        c0=stage(3,k);
        c1=h(4)+lamSum*h(3)+lamProd*h(2);
        c2=(h(5)+lamSum*h(4)+lamProd*h(3))/2;
        psi2Lower(k)=zoh_poly_min( ...
            [c0 c1 c2 -psi2ThirdBound/6],dt);
    end
end
end

function zoh_report_stage_failure(sensorTime,phase,info,stage,psi2Lower,names,~)
% A certificate/optimization failure is NOT a demonstrated collision.
[bad,i]=min(info.barrierResiduals);
if isempty(i) || ~isfinite(bad)
    bad=info.combinedMinimum;
    i=find(strcmp(names,info.criticalConstraint),1);
end
if isempty(i),i=1;end
fprintf(2,['[CBF REAL ZOH] no verified sampled-data input: t=%.3f ' ...
    'phase=%s critical=%s residual=%.6g m exitflag=%.0f iterations=%.0f\n'], ...
    sensorTime,phase,names{i},bad,info.solverExitflag,info.solverIterations);
fprintf(2,['[CBF REAL ZOH] %s: h=%.6g intervalLower=%.6g ' ...
    'required=%.6g; psi1..4 DIAGNOSTIC=[%.4g %.4g %.4g %.4g], ' ...
    'psi2ZOH DIAGNOSTIC=%.4g\n'], ...
    names{i},stage(1,i),info.intervalLowerBounds(i), ...
    info.zohBarrierFactor*stage(1,i),stage(2:5,i),psi2Lower(i));
end

function planes=zoh_make_planes(x,parts,P,G)
template=struct('name','','kind','','body',0,'n',[0;0;1], ...
    'center',[NaN;NaN;NaN],'axes',[NaN;NaN;NaN],'Q',zeros(3), ...
    'currentPoint',[NaN;NaN;NaN]);
planes=repmat(template,0,1);
cf=[1 G.cableCenterFromLoad 0];kinds={'drone','cable','load'};
for j=1:numel(parts)
    [ctr,Q,ax,lab]=zoh_obstacle_geometry(parts(j),j);
    for k=1:3
        c=cf(k);cent=x(8:10)-c*P.cableL*x(14:16);
        if k==1,Qs=(G.droneSafetyRadius^2)*eye(3);
        elseif k==3,Qs=(G.loadSafetyRadius^2)*eye(3);
        else
            Qs=G.cableAxisShort^2*eye(3)+(G.cableAxisLong^2-G.cableAxisShort^2)* ...
                (x(14:16)*x(14:16).');
        end
        n=zoh_best_plane_normal(cent-ctr,Q,Qs,k~=2);
        item=template;item.kind='obstacle';item.name=[lab '/' kinds{k}];
        item.body=k;item.n=n;item.Q=Q;item.center=ctr;
        item.axes=ax;item.currentPoint=cent;
        planes(end+1,1)=item; %#ok<AGROW>
    end
end
if G.includeGround
    for k=1:3
        item=template;item.kind='ground';item.name=['ground/' kinds{k}];
        item.body=k;item.n=[0;0;1];item.center=[0;0;G.groundZ];
        item.currentPoint=x(8:10)-cf(k)*P.cableL*x(14:16);
        planes(end+1,1)=item; %#ok<AGROW>
    end
end
end

function n=zoh_best_plane_normal(d,Qo,Qb,sphere)
% Each tested normal is a valid supporting-plane certificate; the search
% quality affects conservatism, not soundness. For spherical bodies, the
% normal of the nearest Euclidean point on the ellipsoid is exact.
D=norm(d);
if D<1e-12,n=[1;0;0];return;end
n0=d/D;
n1=zoh_closest_point_normal(d,Qo);
if sphere,n=n1;return;end
[V,~]=eig(Qo);
signs=sign(V.'*d);signs(signs==0)=1;
starts=[n0 n1 V*diag(signs)];
best=-inf;n=n1;
for j=1:size(starts,2)
    v=starts(:,j);v=v/max(norm(v),eps);
    for k=1:14
        margin=zoh_plane_gap(d,Qo,Qb,v);
        if margin>best,best=margin;n=v;end
        go=Qo*v/sqrt(v.'*Qo*v);
        gb=Qb*v/sqrt(v.'*Qb*v);
        grad=d-go-gb;
        tangent=grad-v*(v.'*grad);
        tnorm=norm(tangent);
        if tnorm<1e-11,break;end
        step=0.7;
        for it=1:8
            vv=v+step*tangent;
            vv=vv/norm(vv);
            if zoh_plane_gap(d,Qo,Qb,vv)>=margin
                v=vv;break
            end
            step=step/2;
        end
        if it==8 && zoh_plane_gap(d,Qo,Qb,vv)<margin,break;end
    end
    margin=zoh_plane_gap(d,Qo,Qb,v);
    if margin>best,best=margin;n=v;end
end
end

function n=zoh_closest_point_normal(d,Q)
% Nearest point on origin-centred ellipsoid with shape matrix Q.
[V,S]=eig((Q+Q.')/2);
a2=diag(S);v=V.'*d;
if sum((v.^2)./a2)<=1
    n=d/max(norm(d),eps);return
end
lo=0;hi=max(a2);
for k=1:64
    if sum(a2.*(v.^2)./(a2+hi).^2)<1,break;end
    hi=hi*2;
end
for k=1:65
    mid=(lo+hi)/2;
    if sum(a2.*(v.^2)./(a2+mid).^2)>1
        lo=mid;
    else
        hi=mid;
    end
end
foot=V*((a2.*v)./(a2+(lo+hi)/2));
n=d-foot;
n=n/max(norm(n),eps);
end

function g=zoh_plane_gap(d,Qo,Qb,n)
g=n.'*d-sqrt(max(0,n.'*Qo*n))-sqrt(max(0,n.'*Qb*n));
end

function [h0,h1]=zoh_plane_initial(x,P,G,planes)
h0=zeros(numel(planes),1);h1=h0;
pT=x(14:16);wL=x(17:19);pTdot=cross(wL,pT);
cf=[1 G.cableCenterFromLoad 0];
for i=1:numel(planes)
    z=planes(i);k=z.body;c=cf(k);n=z.n;
    cent=x(8:10)-c*P.cableL*pT;
    vel=x(11:13)-c*P.cableL*pTdot;
    if k==2
        a=G.cableAxisShort;delta=G.cableAxisLong^2-a^2;
        y=n.'*pT;yd=n.'*pTdot;
        sb=sqrt(a^2+delta*y^2);
        sdot=delta*y*yd/sb;
    elseif k==1
        sb=G.droneSafetyRadius;sdot=0;
    else
        sb=G.loadSafetyRadius;sdot=0;
    end
    if strcmp(z.kind,'obstacle')
        obsSupport=sqrt(n.'*z.Q*n);
        h0(i)=n.'*(cent-z.center)-obsSupport-sb;
    else
        h0(i)=cent(3)-G.groundZ-sb;
    end
    h1(i)=n.'*vel-sdot;
end
end

function [g,branch]=zoh_support_interval_margins(u,x,P,G,planes,dt,massRange)
% TWO candidate-dependent lower bounds for the SAME ZOH physical input:
% A) acceleration/shape-rate envelope (independent of torque, conservative);
% B) 4th-order Taylor + 5th-derivative envelope for sphere components.
%    B is a conditional research certificate; its global majorant must
%    be validated independently before a formal invariant-set claim.
% The maximum of two proven lower bounds would also be a lower bound.
% For the central thin cable, only use A (avoid the large shape series).
% In all cases the obstacle is the ORIGINAL observed ellipsoid.
nc=numel(planes);g=zeros(nc,1);branch=cell(nc,1);
if nc==0,return;end
[h0,h1]=zoh_plane_initial(x,P,G,planes);
T=abs(u(1));m=P.m;L=P.cableL;
Omega=norm(x(17:19))+(T/(m*L))*dt;
Aload=P.gravity+norm([P.dstx;P.dsty])+ ...
    (T+m*L*Omega^2)/(m+massRange(1));
pt2=T/(m*L)+Omega^2;
cf=[1 G.cableCenterFromLoad 0];
for j=1:nc
    k=planes(j).body;c=cf(k);
    shapeRate=0;shapeDot0=0;
    if k==2
        delta=G.cableAxisLong^2-G.cableAxisShort^2;
        n=planes(j).n;
        y=n.'*x(14:16);
        ydot=n.'*cross(x(17:19),x(14:16));
        shapeDot0=delta*y*ydot/sqrt(G.cableAxisShort^2+delta*y^2);
        shapeRate=sqrt(delta)*Omega;
    end
    Ac=Aload+c*L*pt2;
    lowerEnd=h0(j)+(h1(j)+shapeDot0-shapeRate)*dt-0.5*Ac*dt^2;
    g(j)=min(h0(j),lowerEnd);
    branch{j}='SUPPORT_ACCELERATION_BOUND';
end
% The Taylor alternative is evaluated once per candidate using the
% generated 19-state ZOH physical dynamics, including all three torques.
% It is useful for the sphere components near an active safety boundary.
% Never use this variant for the thin cable shape without a verified majorant.
if any([planes.body]~=2)
    gJet=zoh_four_input_taylor_lower(u,x,P,G,planes,dt);
    for j=1:nc
        if planes(j).body==2 || ~isfinite(gJet(j)),continue;end
        if gJet(j)>g(j)
            g(j)=gJet(j);
            branch{j}='ZOH_TAYLOR4_FIFTH_BOUND_ASSUMED';
        end
    end
end
end

function g=zoh_four_input_taylor_lower(u,x,P,G,planes,dt)
% Interval lower bound h(t)>=Taylor4(t)-M5*t^5/5!, conditional on
% M5 being a verified uniform bound on |d^5 h/dt^5| over [0,dt].
% The generated ZOH jet includes T and (tauX,tauY,tauZ) directly.
nc=numel(planes);g=-inf(nc,1);
Gjet=G;Gjet.includeGround=false;
J=CBF_REAL_ZOH_DIAG_EVAL(x,u(:),struct([]),P,Gjet,4);
M5=zoh_plane_fifth_bounds(x,u(:),P,G,dt);
for j=1:nc
    body=planes(j).body;
    if body==2,continue;end
    coeff=zoh_plane_series(J.stateTaylor,planes(j),P,G,4);
    if ~isfinite(M5(body)) || M5(body)<0,continue;end
    minorant=[coeff(:).' -M5(body)/factorial(5)];
    g(j)=zoh_poly_min(minorant,dt);
end
end

function J=zoh_barrier_jacobian(u,x,P,G,planes,dt,massRange)
% Differentiates the ACTUAL nonlinear constraint g(u), NOT the 4-input
% objective.  Units: m/N for T, m/(N*m) for each torque.
% Central differences at feasible bounded inputs; no fabricated sensitivity.
if isempty(planes),J=zeros(0,4);return;end
lb=[0;-1;-1;-1];ub=[20;1;1;1];
step=[0.01;0.05;0.05;0.05];
J=nan(numel(planes),4);
for k=1:4
    up=u(:);um=u(:);
    up(k)=min(ub(k),up(k)+step(k));
    um(k)=max(lb(k),um(k)-step(k));
    if up(k)==um(k),continue;end
    gp=zoh_support_interval_margins(up,x,P,G,planes,dt,massRange);
    gm=zoh_support_interval_margins(um,x,P,G,planes,dt,massRange);
    J(:,k)=(gp(:)-gm(:))/(up(k)-um(k));
end
end

function c=zoh_plane_series(X,z,P,G,N)
cf=[1 G.cableCenterFromLoad 0];cBody=cf(z.body);n=z.n;
cent=X(8:10,1:N+1)-cBody*P.cableL*X(14:16,1:N+1);
c=(n.'*cent);
if strcmp(z.kind,'obstacle')
    c(1)=c(1)-n.'*z.center-sqrt(n.'*z.Q*n);
else
    c(1)=c(1)-G.groundZ;
end
if z.body==2
    as=G.cableAxisShort;delta=G.cableAxisLong^2-as^2;
    y=n.'*X(14:16,1:N+1);
    b=delta*conv(y,y);b=b(1:N+1);b(1)=b(1)+as^2;
    s=zeros(1,N+1);s(1)=sqrt(b(1));
    for k=1:N
        prev=0;
        for i=1:k-1,prev=prev+s(i+1)*s(k-i+1);end
        s(k+1)=(b(k+1)-prev)/(2*s(1));
    end
    c=c-s;
elseif z.body==1
    c(1)=c(1)-G.droneSafetyRadius;
else
    c(1)=c(1)-G.loadSafetyRadius;
end
end

function M=zoh_plane_fifth_bounds(x,u,P,G,dt)
% Uniform coefficient majorants over the current ZOH flow. The fifth
% derivative of each projected body centre is bounded by the positive
% Taylor-coefficient convolution of the physical 19-state RHS.
T=abs(u(1));tauN=norm(u(2:4));m=P.m;mL=P.mL;L=P.cableL;
Jmin=min(P.J);Jmax=max(P.J);
W=(sqrt(sum(P.J(:).*x(5:7).^2))+tauN*dt/sqrt(Jmin))/sqrt(Jmin);
Omega=norm(x(17:19))+(T/(m*L))*dt;
Aload=P.gravity+norm([P.dstx;P.dsty])+(T+m*L*Omega^2)/(m+mL);
Vload=norm(x(11:13))+Aload*dt;
N=5;Q=zeros(1,N+1);WB=Q;PT=Q;WC=Q;VL=Q;
Q(1)=1;WB(1)=W;PT(1)=1;WC(1)=Omega;VL(1)=Vload;
for k=0:N-1
    WB(k+2)=(Jmax/Jmin*zoh_conv_at(WB,WB,k)+ ...
        double(k==0)*tauN/Jmin)/(k+1);
    Q(k+2)=0.5*zoh_conv_at(Q,WB,k)/(k+1);
    PT(k+2)=zoh_conv_at(WC,PT,k)/(k+1);
    B3=conv(Q,Q);
    WC(k+2)=(T/(m*L))*zoh_conv_at(PT,B3,k)/(k+1);
    CV=conv(WC,PT);
    scalar=zeros(1,N+1);
    for j=0:k
        scalar(j+1)=T*zoh_conv_at(PT,B3,j)+m*L*zoh_conv_at(CV,CV,j);
    end
    al=zoh_conv_at(scalar,PT,k)/(m+mL);
    if k==0,al=al+P.gravity+norm([P.dstx;P.dsty]);end
    VL(k+2)=al/(k+1);
end
PL=zeros(1,N+1);
for k=1:N,PL(k+1)=VL(k)/k;end
% Support shape s=sqrt(as^2+delta*y^2), |y^(k)|/k! <= PT(k+1).
% Binomial series around any instant with s>=as gives an upper bound on
% the coefficient of t^5 of s. This avoids sign-unsafe sqrt recurrences.
as=G.cableAxisShort;delta=G.cableAxisLong^2-as^2;
bDelta=zeros(1,N+1);
for k=1:N,bDelta(k+1)=delta*zoh_conv_at(PT,PT,k);end
shape5=0;pow=zeros(1,N+1);pow(1)=1;
binom=1;
for j=1:N
    pow=conv(pow,bDelta);pow=pow(1:N+1);
    binom=binom*(0.5-(j-1))/j;
    shape5=shape5+abs(binom)*as^(1-2*j)*pow(N+1);
end
M=factorial(N)*[PL(N+1)+L*PT(N+1), ...
    PL(N+1)+G.cableCenterFromLoad*L*PT(N+1)+shape5, PL(N+1)];
end

function s=zoh_conv_at(a,b,k)
s=0;for j=0:k,s=s+a(j+1)*b(k-j+1);end
end

function m=zoh_poly_min(c,dt)
% Exact polynomial stationary-point minimization in real arithmetic.
% Floating-point roots are not a formal validated interval computation.
if any(~isfinite(c)),m=-inf;return;end
v=fliplr((1:numel(c)-1).*c(2:end));
r=roots(v);
r=real(r(abs(imag(r))<1e-9));
ts=[0;dt;r(r>0 & r<dt)];
m=min(polyval(fliplr(c),ts));
end

function info=zoh_critical(info,planes,i)
z=planes(i);info.criticalH0=info.currentH(i);
info.criticalHdot0=info.currentHdot(i);
if strcmp(z.kind,'obstacle')
    info.criticalObstacleCenter=z.center;
    info.criticalObstacleAxes=z.axes;
    info.criticalBodyCenterDistance=norm(z.currentPoint-z.center);
else
    info.criticalObstacleCenter=[NaN;NaN;NaN];
    info.criticalObstacleAxes=[NaN;NaN;NaN];
    info.criticalBodyCenterDistance=NaN;
end
end

function [center,Q,axes,label]=zoh_obstacle_geometry(part,index)
if ~isfield(part,'center'),error('CBFReal:Obstacle','Missing center.');end
center=part.center(:);
if numel(center)~=3 || any(~isfinite(center))
    error('CBFReal:Obstacle','Invalid center.');
end
kind='ellipsoid';if isfield(part,'kind'),kind=char(part.kind);end
switch kind
    case 'ellipsoid'
        if isfield(part,'params'),axes=part.params.axes(:);
        else,axes=part.axes(:);end
        R=part.R;
    case 'sphere'
        if isfield(part,'params'),r=part.params.radius;
        else,r=part.radius;end
        axes=repmat(r,3,1);R=eye(3);
    otherwise
        error('CBFReal:Obstacle','Unsupported kind: %s',kind);
end
if numel(axes)~=3 || any(~isfinite(axes)) || any(axes<=0) || ...
        ~isequal(size(R),[3 3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5
    error('CBFReal:Obstacle','Invalid ellipsoid parameters.');
end
Q=R*diag(axes.^2)*R.';
if isfield(part,'obstacleName'),label=char(part.obstacleName);
elseif isfield(part,'name'),label=char(part.name);
else,label=sprintf('obstacle%d',index);end
end

function G=zoh_geometry(P,observed)
G=CBF_COMMON_GEOMETRY(P,struct());
if ~isfield(observed,'ground')
    error('CBFReal:Ground','Sensor ground data missing.');
end
if isempty(observed.ground),G.includeGround=false;return;end
ground=observed.ground;
if ~strcmp(char(ground.kind),'halfspace') || ...
        norm(ground.normal(:)-[0;0;1])>1e-9 || ...
        (isfield(ground,'forbiddenSide') && ...
         ~strcmp(char(ground.forbiddenSide),'negative'))
    error('CBFReal:Ground','Only upward ground halfspace supported.');
end
margin=0;
if isfield(ground,'safetyMargin'),margin=ground.safetyMargin;end
if ~isfinite(margin) || margin<0
    error('CBFReal:Ground','Invalid margin.');
end
G.groundZ=ground.point(3)+margin;
G.includeGround=true;
end

function zoh_send_failure_to_view(sensorTime,info,args)
% Plot already-recorded LOGGER history plus this failed CBF step BEFORE
% HLC raises its existing error. This is observational only.
% If the optional plotting function is not installed, do not change control.
if numel(args)<3,return;end
if exist('CBF_REAL_DIAG_VIEW','file')~=2,return;end
try
    CBF_REAL_DIAG_VIEW('failure',args{3},sensorTime,info);
catch ME
    warning('CBFReal:HistoryPlotFailed', ...
        'CBF history plotting failed (control still stops): %s',ME.message);
end
end
