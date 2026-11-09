function out = Generate_CBF_ActuatorAware_6Methods(action,varargin)
% Generate_CBF_ActuatorAware_6Methods.m
% ONE MATLAB source: TWO control architectures x SIX geometries, PLUS
% ACTUATOR-AWARE SUPERVISION with monitoring, engagement and smooth return.
% 'supervise' is EXPERIMENTAL (not certified sampled-data safety).
% Extra experimentalContinueAfterPsiViolation=true only allows ACTIVE
% chain loss to be observed; the hard QP and actuator limits remain.
% Set experimentalStopAtGeometricBoundary=true to STOP at S h<=0,
% which is loss of a separation CERTIFICATE (not proof of collision).
% NEVER USE this trial option for safety claims or real flights.
% Optional opts.preemptivePush>0 activates a SOFT positive-margin
% anticipation objective for physical mode BEFORE the standard CBF-QP
% becomes active. ALL original safety and actuator rows remain HARD.
% This is a research comparison, NOT a proof of recursive feasibility.
% Unlike the legacy 'filter', the supervisor uses the saturated HLC
% command as non-intervention baseline; no changes to HLC or reference.
% During saturated-HLC flight, virtual control cannot alter f instantly;
% the supervisor may explicitly use physical-mode dynamic extension.
% Monitor is the physical-mode instantaneous frozen-thrust proxy and is
% NOT a proof of future safety under time-varying saturated HLC.
%
% VIRTUAL:   aux=delta(4), decision w=[nu;v2;v3;v4]
%            delta(1:3)dot=delta(2:4), delta(4)dot=nu
%            v1=-F1*[z-zd;vz-vzd]+delta(1)
%            f=HLC first-layer thrust, M=Beta2\(v234-alpha2).
% PHYSICAL:  aux=[f;fDot], decision w=[fDDot;M1;M2;M3]
%            fDot=aux(2), fDDot=w(1), and plant uses actual f=aux(1).
%            This is a second-order THRUST DYNAMIC EXTENSION; it changes
%            the original HLC thrust response even when the QP is inactive.
%            Thrust bounds are enforced using second-order HOCBF rows.
%
% Geometry: S1,S2,S3 support plane; K1,K2,K3 KKT solid-ellipsoid distance.
% All obstacles must be STATIONARY (arbitrary fixed orientation allowed).
% Uses no Symbolic Math Toolbox; derivatives via truncated Taylor jets.
% K methods require fmincon, filter requires quadprog (Optimization Toolbox).
% Numerical sample audits are NOT a rigorous safety theorem.
%
% USAGE:
%  report=Generate_CBF_ActuatorAware_6Methods();     % both modes x all methods
%  cfg=struct('modes',{{'virtual','physical'}},'methods',{{'S1','S2'}});
%  report=Generate_CBF_ActuatorAware_6Methods(cfg);
%  s=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,method,component,...
%                                   x,aux,refZ,F1,P,obs,geom,kappa);
%  result=Generate_CBF_ActuatorAware_6Methods('filter',mode,obj,x,xd,...
%                 vfNom,F1,P,aux,obsList,method,shapes,kappa,opts);
%  report=Generate_CBF_ActuatorAware_6Methods('selftest',cfg);
%  result=Generate_CBF_ActuatorAware_6Methods('supervise',requestedMode,...
%       obj,x,xd,vfNom,F1,P,uHLCraw,obstacles,method,shapes,kappa,...
%       opts,supervisorState,dt);
%  Assign result.supervisor back to a persistent HLC property.
%  diagnostics=Generate_CBF_ActuatorAware_6Methods('diagnose',mode,obj,x,xd,...
%         vfNom,F1,P,aux,obsList,method,shapes,kappa,opts);
%  On failed initial chain, CBF_initial_chain_failure.mat is saved to pwd.
%  On virtual thrust-bound failure, CBF_thrust_limit_failure.mat is saved.
%  These diagnostic snapshots DO NOT modify the safety filter.
%
% OUTPUT: generated_cbf_dual_input/virtual/S1/CBF_V_S1_single.m etc
%         generated_cbf_dual_input/physical/S1/CBF_P_S1_single.m etc
% Generated wrappers call THIS master file; keep it on MATLAB path.
%
% HLC INTEGRATION: after tmp=[uf(1);us] is computed, before saturation:
%   res=Generate_CBF_ActuatorAware_6Methods('filter',mode,obj,x,xd',vf,...
%              F1,P,cbfAux,obs,method,shapes,kappa,opts);
%   if ~res.success, error('CBF QP infeasible'); end
%   obj.result.input=res.uSafe;  % NO POST-QP SATURATION
%   cbfAux=cbfAux+Param.dt*res.auxDot; % illustrative Euler ONLY.
%
% IMPORTANT: need separate persistent aux state for each MODE; initialize
% physical aux(1) to existing f_nom, aux(2)=0; virtual delta=zeros(4,1).
% Fail-closed on invalid geometry, initial high-order barrier chain,
% singular HLC beta2, KKT failures, and infeasible QP.
% Theoretical guarantee requires recursive feasibility, smooth enough
% trajectory, initial HOCBF conditions, true geometry enclosure, and
% continuous-time/sample-data input constraints. NEVER claim these from
% one successful numerical selftest.
if nargin==0
    action='generate';
elseif isstruct(action)
    varargin=[{action},varargin];action='generate';
end
switch lower(char(action))
    case 'generate'
        if isempty(varargin),cfg=struct();else,cfg=varargin{1};end
        out=localGenerate(cfg);
    case 'evaluate'
        [h,psi,A,b,info]=localEvaluate(varargin{:});
        out=struct('h',h,'psi',psi,'Araw',A,'braw',b,'info',info);
    case 'selftest'
        if isempty(varargin),cfg=struct();else,cfg=varargin{1};end
        cfg.smokeTest=true;cfg.writeFunctions=false;
        out=localGenerate(cfg);
    case 'filter'
        out=localFilter(varargin{:});
    case 'supervise'
        out=localSupervise(varargin{:});
    case 'warmupphysicalqp'
        % NO PLANT, NO STATE MUTATION: pre-exercise both sample-bounds QPs.
        % Synthetic constraints are used solely to initialize solver paths.
        out=localWarmupPhysicalQP(varargin{:});
    case 'sourceversion'
        % Read-only fingerprint for troubleshooting stale MATLAB path/cache.
        out=struct('tag','CBF_GEN_20261015_NAV2','path',mfilename('fullpath'));
    case 'diagnose'
        % Non-controlling evaluation; returns all geometric HOCBF chains.
        % NEVER send outputs of this action to the plant as safe inputs.
        if numel(varargin) < 13 || isempty(varargin{13})
            varargin{13}=struct();
        end
        o=varargin{13};o.diagnosticOnly=true;o.requireInitialChain=false;
        varargin{13}=o;
        out=localFilter(varargin{:});
    otherwise
        error('Unknown action: %s',char(action));
end
end

function cfg=localDefaults(cfg)
if ~isfield(cfg,'modes') || isempty(cfg.modes),cfg.modes={'virtual','physical'};end
if ~isfield(cfg,'methods') || isempty(cfg.methods),cfg.methods={'S1','S2','S3','K1','K2','K3'};end
if ~isfield(cfg,'smokeTest'),cfg.smokeTest=true;end
if ~isfield(cfg,'writeFunctions'),cfg.writeFunctions=true;end
if ~isfield(cfg,'maxSeconds'),cfg.maxSeconds=360;end
if ~isfield(cfg,'outputDir'),cfg.outputDir='generated_cbf_dual_input';end
if ischar(cfg.modes),cfg.modes={cfg.modes};end
if isstring(cfg.modes),cfg.modes=cellstr(cfg.modes);end
if ischar(cfg.methods),cfg.methods={cfg.methods};end
if isstring(cfg.methods),cfg.methods=cellstr(cfg.methods);end
end

function pieces=localMethodSpecs(method)
switch upper(char(method))
    case {'S1','K1'},pieces={'single'};
    case {'S2','K2'},pieces={'sphere'};
    case {'S3','K3'},pieces={'uav','cable','load'};
    otherwise,error('Unknown geometry method: %s',char(method));
end
end

function mode=localMode(mode)
mode=lower(char(mode));
if ~any(strcmp(mode,{'virtual','physical'}))
    error('mode must be ''virtual'' or ''physical''.');
end
end

function report=localGenerate(cfg)
cfg=localDefaults(cfg);
root=fullfile(fileparts(mfilename('fullpath')),cfg.outputDir);
if cfg.writeFunctions && ~exist(root,'dir'),mkdir(root);end
T=tic; modeCol=strings(0,1);methodCol=strings(0,1);componentCol=strings(0,1);
statusCol=strings(0,1);rdCol=zeros(0,1);secondsCol=zeros(0,1);
lowCol=nan(0,1);affineCol=nan(0,1);inputCoeffCol=nan(0,1);
fprintf('[CBF-DUAL] TWO modes: virtual and physical; SIX geometry methods.\n');
fprintf('[CBF-DUAL] Source is ONE file: %s\n',mfilename('fullpath'));
fprintf('[CBF-DUAL] Output: %s\n',root);
for mm=1:numel(cfg.modes)
 mode=localMode(cfg.modes{mm});
 for j=1:numel(cfg.methods)
  method=upper(char(cfg.methods{j}));pieces=localMethodSpecs(method);
  for k=1:numel(pieces)
   name=pieces{k};tc=tic;reason='';r=localExpectedRD(mode,name,[]);
   modeCol(end+1,1)=string(mode);methodCol(end+1,1)=string(method);
   componentCol(end+1,1)=string(name);rdCol(end+1,1)=r;
   fprintf('[CBF-DUAL][+%.2fs][%s/%s/%s] START r=%d\n',toc(T),mode,method,name,r);
   maxLow=nan;maxAffine=nan;normA=nan;
   try
    if toc(T)>cfg.maxSeconds,error('Overall time budget reached (checked between evaluations).');end
    if cfg.smokeTest
     [x,aux,refZ,F1,P,obs,geom,kappa]=localExample(mode,name);
     s=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,method,name,...
           x,aux,refZ,F1,P,obs,geom,kappa);
     if ~s.info.valid,error('Smoke test failed: %s',s.info.reason);end
     maxLow=s.info.maxLowerInputDependence;
     maxAffine=s.info.maxAffineResidual;normA=norm(s.Araw);
     if strcmp(name,'sphere')
      for la=[0,1]
       geomE=geom;geomE(1)=la;
       ss=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,method,name,...
           x,aux,refZ,F1,P,obs,geomE,kappa);
       if ~ss.info.valid,error('Endpoint lambda=%g failed: %s',la,ss.info.reason);end
       fprintf('  [endpoint lambda=%g] r=%d affine=%g\n',la,ss.info.relativeDegreeAssumed,ss.info.maxAffineResidual);
      end
     end
    end
    if cfg.writeFunctions,localWriteWrapper(root,mode,method,name);end
    reason='generated+sample_audited';
    if ~cfg.smokeTest,reason='generated_UNTESTED';end
   catch ME
    reason=['FAILED: ' ME.message];
    fprintf(2,'[CBF-DUAL] %s/%s/%s %s\n',mode,method,name,reason);
   end
   statusCol(end+1,1)=string(reason);secondsCol(end+1,1)=toc(tc);
   lowCol(end+1,1)=maxLow;affineCol(end+1,1)=maxAffine;inputCoeffCol(end+1,1)=normA;
   fprintf('[CBF-DUAL][+%.2fs] END %s/%s/%s %.3fs : %s\n',toc(T),mode,method,name,toc(tc),reason);
  end
 end
end
report=table(modeCol,methodCol,componentCol,statusCol,rdCol,secondsCol,lowCol,affineCol,inputCoeffCol,...
  'VariableNames',{'mode','method','component','status','assumedRD','seconds','lowerResidual','affineResidual','inputCoefficientNorm'});
if cfg.writeFunctions,writetable(report,fullfile(root,'CBF_DUAL_generation_report.csv'));end
disp(report);
fprintf('[CBF-DUAL] Total elapsed %.3fs; numerical audits do not prove general safety.\n',toc(T));
end

function localWriteWrapper(root,mode,method,name)
if strcmp(mode,'virtual'),abbr='V';else,abbr='P';end
folder=fullfile(root,mode,method);if ~exist(folder,'dir'),mkdir(folder);end
fun=sprintf('CBF_%s_%s_%s',abbr,method,name);
file=fullfile(folder,[fun '.m']);
fid=fopen(file,'wt');if fid<0,error('Cannot write %s',file);end
closer=onCleanup(@()fclose(fid));
fprintf(fid,'function [h,psi,Araw,braw,info]=%s(x,aux,refZ,F1,P,obs,geom,kappa)\n',fun);
fprintf(fid,'%% Autogenerated wrapper; keep Generate_CBF_ActuatorAware_6Methods.m on path.\n');
fprintf(fid,'s=Generate_CBF_ActuatorAware_6Methods(''evaluate'',''%s'',''%s'',''%s'',x,aux,refZ,F1,P,obs,geom,kappa);\n',mode,method,name);
fprintf(fid,'h=s.h; psi=s.psi; Araw=s.Araw; braw=s.braw; info=s.info;\nend\n');
end

function r=localExpectedRD(mode,name,geom)
if strcmp(mode,'physical')
    r=4;  % thrust is extended twice, torque enters generically at fourth order.
elseif strcmp(name,'load') || (strcmp(name,'sphere') && ~isempty(geom) && abs(geom(1))<1e-12)
    r=6;
else
    r=4;
end
end

function [h,psi,Araw,braw,info]=localEvaluate(mode,method,name,x,aux,refZ,F1,P,obs,geom,kappa)
clock=tic;mode=localMode(mode);method=upper(char(method));name=char(name);
if ~ismember(name,localMethodSpecs(method)),error('%s not in %s',name,method);end
x=x(:);aux=aux(:);refZ=refZ(:);F1=F1(:);P=P(:);geom=geom(:);kappa=kappa(:);
r=localExpectedRD(mode,name,geom);
assert(numel(x)==19 && numel(refZ)==7 && numel(F1)==2 && numel(P)==9);
if strcmp(mode,'virtual'),assert(numel(aux)==4);else,assert(numel(aux)==2);end
assert(numel(kappa)>=r && all(isfinite(kappa(1:r))) && all(kappa(1:r)>0));
assert(all(isfinite([x;aux;refZ;F1;P;geom])));
assert(P(1)>0 && all(P(2:4)>0) && P(6)>=0 && P(7)>0);
assert(abs(norm(x(1:4))-1)<1e-4 && abs(norm(x(14:16))-1)<1e-4);
assert(abs(x(14:16).'*x(17:19))<1e-3);
[co,SO,margin]=localObstacle(obs);
[SA,cA0]=localShapeAtState(name,x,P(7),geom);
ctx=struct('cObs',co,'SO',SO,'margin',margin,'method',method);
if method(1)=='K',ctx=localKKTPrecompute(ctx,cA0,SA);end
w0=zeros(4,1);
der0=localHDerivatives(mode,method,name,[x;aux],refZ,F1,P,geom,ctx,r,w0);
poly=1;psiChain=zeros(r,1);psiChain(1)=der0(1);
for k=1:r-1
    poly=conv(poly,[kappa(k),1]);psiChain(k+1)=poly*der0(1:k+1).';
end
psi=psiChain(end);poly=conv(poly,[kappa(r),1]);
braw=poly*der0.';h=der0(1);
Araw=zeros(1,4);maxLow=0;maxCurve=0;maxMixed=0;
for i=1:4
    u=zeros(4,1);u(i)=1;
    pos=localHDerivatives(mode,method,name,[x;aux],refZ,F1,P,geom,ctx,r,u);
    neg=localHDerivatives(mode,method,name,[x;aux],refZ,F1,P,geom,ctx,r,-u);
    Araw(i)=.5*(poly*(pos-neg).');
    maxLow=max(maxLow,max(abs([pos(1:r)-der0(1:r),neg(1:r)-der0(1:r)])));
    maxCurve=max(maxCurve,abs(poly*(pos+neg-2*der0).'));
end
for u=[ones(4,1),[.37;-.22;.19;-.13]]
    val=localHDerivatives(mode,method,name,[x;aux],refZ,F1,P,geom,ctx,r,u);
    maxLow=max(maxLow,max(abs(val(1:r)-der0(1:r))));
    maxMixed=max(maxMixed,abs(poly*val.'-braw-Araw*u));
end
maxAffine=max(maxCurve,maxMixed);
tolLow=1e-5*max(1,norm(der0,inf));tolAffine=1e-5*max([1,abs(braw),norm(Araw,inf)]);
valid=all(isfinite([h;psi;braw;Araw(:)])) && maxLow<=tolLow && maxAffine<=tolAffine;
if valid,reason='sample input-affine audit passed; not a safety certificate';
else,reason='lower-order input dependence / non-affinity detected';end
if method(1)=='K',gap=ctx.separation;kr=ctx.kktResidual;jr=ctx.kktRcond;
else,gap=h;kr=nan;jr=nan;end
info=struct('valid',valid,'reason',reason,'mode',mode,'relativeDegreeAssumed',r,...
    'maxLowerInputDependence',maxLow,'maxAffineResidual',maxAffine,...
    'lowerTolerance',tolLow,'affineTolerance',tolAffine,...
    'psiChain',psiChain,'initialChainSatisfied',all(psiChain>=-1e-8),...
    'geometricGap',gap,'kktResidual',kr,'kktRcond',jr,...
    'firstInputCoefficient',Araw(1),'momentCoefficientNorm',norm(Araw(2:4)),...
    'evaluationSeconds',toc(clock));
end

function hd=localHDerivatives(mode,method,name,X0,refZ,F1,P,geom,ctx,r,u)
N=r+1; Xj=zeros(numel(X0),N);Xj(:,1)=X0(:);
for k=0:r-2
    F=localJetRHS(Xj,refZ,F1,P,u,mode);
    Xj(:,k+2)=F(:,k+1)/(k+1);
end
% Obtain position and cable direction coefficient of order r from their
% first-order kinematics; avoid computing unrelated high-order state jets.
Xj(8:10,r+1)=Xj(11:13,r)/r;
ptDot=localJCross(Xj(17:19,:),Xj(14:16,:));
Xj(14:16,r+1)=ptDot(:,r)/r;
[cA,SA,PA]=localGeometryJets(name,Xj,geom,P(7));
if method(1)=='S',hj=localSupportJet(cA,SA,ctx);
else,hj=localKKTJet(cA,PA,ctx);end
hd=zeros(1,N);
for k=0:r,hd(k+1)=hj(k+1)*factorial(k);end
end

function [c,S,Pmat]=localGeometryJets(name,X,geom,L)
N=size(X,2);pl=X(8:10,:);pt=X(14:16,:);
S=zeros(3,3,N);Pmat=zeros(3,3,N);
switch name
    case 'single'
        lambda=.5;a=geom(1);b=geom(2);
        assert(numel(geom)==2 && a>=L/2 && a>0 && b>0);
        S=localAxisShapeJet(pt,a,b);
        Pmat=localAxisShapeJet(pt,1/a,1/b);
    case 'sphere'
        assert(numel(geom)==2);
        lambda=geom(1);radius=geom(2);
        assert(lambda>=0 && lambda<=1 && radius>0);
        S(:,:,1)=radius^2*eye(3);Pmat(:,:,1)=eye(3)/radius^2;
    case 'uav'
        assert(numel(geom)==3 && all(geom>0));
        lambda=1;S(:,:,1)=diag(geom.^2);Pmat(:,:,1)=diag(1./geom.^2);
    case 'cable'
        assert(numel(geom)==2 && all(geom>0));
        lambda=.5;a=geom(1);b=geom(2);
        assert(a>=L/2);
        S=localAxisShapeJet(pt,a,b);Pmat=localAxisShapeJet(pt,1/a,1/b);
    case 'load'
        assert(numel(geom)==3 && all(geom>0));
        lambda=0;S(:,:,1)=diag(geom.^2);Pmat(:,:,1)=diag(1./geom.^2);
    otherwise,error('Unknown component.');
end
c=pl-lambda*L*pt;
end

function [S,cA]=localShapeAtState(name,x,L,geom)
X=zeros(23,1);X(1:19)=x(:);
[c,SA,~]=localGeometryJets(name,X,geom,L);
cA=c(:,1);S=SA(:,:,1);
end

function [c,SO,margin]=localObstacle(obs)
assert(isstruct(obs),'Obstacle must be ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE struct.');
assert(isfield(obs,'p_center') && isfield(obs,'ellipsoid_radii') && isfield(obs,'R_obs'));
c=obs.p_center(:);r=obs.ellipsoid_radii(:);R=obs.R_obs;
if isfield(obs,'d_margin'),margin=obs.d_margin;else,margin=0;end
assert(numel(c)==3 && numel(r)==3 && all(r>0) && margin>=0);
assert(all(isfinite([c;r;R(:);margin])));
assert(isequal(size(R),[3 3]) && norm(R.'*R-eye(3),'fro')<1e-5 && det(R)>0);
if isfield(obs,'v_center') && norm(obs.v_center)>1e-12
    error('MOVING obstacle: current CBF omits obstacle time derivatives. Fail closed.');
end
SO=R*diag(r.^2)*R.';SO=(SO+SO.')/2;
end

function hj=localSupportJet(cA,SA,ctx)
N=size(cA,2);d=-cA;d(:,1)=d(:,1)+ctx.cObs;
q=localJDot(d,d);
if q(1)<1e-10,error('Support direction undefined near coincident centers.');end
nd=localJSqrt(q);n=localJRowMul(d,localJInv(nd));
sysNorm=localJSqrt(localJetQuadratic(n,SA));
SO=zeros(3,3,N);SO(:,:,1)=ctx.SO;
obsNorm=localJSqrt(localJetQuadratic(n,SO));
hj=localJDot(n,d)-sysNorm-obsNorm;
hj(1)=hj(1)-ctx.margin;
end

function hj=localKKTJet(cA,PA,ctx)
N=size(cA,2);z=zeros(8,N);z(:,1)=ctx.z0;
for k=2:N
    ff=localKKTResidualJet(z,cA,PA,ctx);
    zk= -(ctx.Jz\ff(:,k));
    if any(~isfinite(zk)),error('Implicit KKT jet diverged.');end
    z(:,k)=zk;
end
d=z(1:3,:)-z(4:6,:);
hj=localJDot(d,d);hj(1)=hj(1)-ctx.margin^2;
end

function F=localKKTResidualJet(z,cA,PA,ctx)
N=size(cA,2);cB=zeros(3,N);cB(:,1)=ctx.cObs;
PB=zeros(3,3,N);PB(:,:,1)=ctx.PB;
eA=z(1:3,:)-cA;eB=z(4:6,:)-cB;
PAeA=localMatJetVec(PA,eA);PBeB=localMatJetVec(PB,eB);
r=z(1:3,:)-z(4:6,:);
F=[r+2*localJRowMul(PAeA,z(7,:));...
   -r+2*localJRowMul(PBeB,z(8,:));...
   localJDot(eA,PAeA);localJDot(eB,PBeB)];
F(7:8,1)=F(7:8,1)-1;
end

function out=localJetQuadratic(v,S)
out=localJDot(v,localMatJetVec(S,v));
end
function o=localMatJetVec(A,b)
N=size(b,2);o=zeros(3,N);
for i=1:3
    for j=1:3
        o(i,:)=o(i,:)+localJMul(reshape(A(i,j,:),1,N),b(j,:));
    end
end
end

function ctx=localKKTPrecompute(ctx,cA,SA)
PB=ctx.SO\eye(3);PA=SA\eye(3);
d0=ctx.cObs-cA;if norm(d0)<1e-9,error('KKT invalid at coincident centers.');end
% Closest points in SOLID ellipsoids, using a convex constrained problem.
% Boundary KKT derivative applies only on separated, both-active branch.
y0=[cA;ctx.cObs];
opt=optimoptions('fmincon','Algorithm','sqp','Display','off',...
   'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,...
   'MaxIterations',250,'MaxFunctionEvaluations',4000,...
   'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10);
[y,~,flag]=fmincon(@obj,y0,[],[],[],[],[],[],@con,opt);
if flag<=0,error('KKT closest-point solver did not converge (exitflag=%d).',flag);end
yA=y(1:3);yB=y(4:6);d=yA-yB;separation=norm(d);
if separation<1e-7,error('KKT separated branch unavailable (intersecting/tangent).');end
eA=yA-cA;eB=yB-ctx.cObs;va=PA*eA;vb=PB*eB;
la=-(d.'*va)/(2*(va.'*va));lb=(d.'*vb)/(2*(vb.'*vb));
if min(la,lb)<=1e-9,error('KKT inactive/degenerate multiplier.');end
z0=[y;la;lb];
Jz=[eye(3)+2*la*PA,-eye(3),2*va,zeros(3,1);...
     -eye(3),eye(3)+2*lb*PB,zeros(3,1),2*vb;...
     2*va.',zeros(1,3),0,0;...
     zeros(1,3),2*vb.',0,0];
if rcond(Jz)<1e-11,error('KKT Jacobian Jz nearly singular.');end
rK=[d+2*la*va;-d+2*lb*vb;eA.'*PA*eA-1;eB.'*PB*eB-1];
if norm(rK,inf)>2e-5,error('KKT stationary residual too high: %g.',norm(rK,inf));end
ctx.z0=z0;ctx.Jz=Jz;ctx.PB=PB;ctx.separation=separation;
ctx.kktResidual=norm(rK,inf);ctx.kktRcond=rcond(Jz);
function [f,g]=obj(y)
    d=y(1:3)-y(4:6);f=.5*(d.'*d);g=[d;-d];
end
function [cc,ceq,gc,gceq]=con(y)
    a=y(1:3)-cA;b=y(4:6)-ctx.cObs;
    cc=[a.'*PA*a-1;b.'*PB*b-1];ceq=[];
    gc=zeros(6,2);gc(1:3,1)=2*PA*a;gc(4:6,2)=2*PB*b;
    gceq=[];
end
end

function fjet=localJetRHS(X,ref,F1,P,u,mode)
N=size(X,2);
q=X(1:4,:); wb=X(5:7,:); vl=X(11:13,:);
pT=X(14:16,:);wL=X(17:19,:);
m=P(1);J=P(2:4);g=P(5);mL=P(6);L=P(7);
ptDot=localJCross(wL,pT);speed2=localJDot(ptDot,ptDot);
% Third body axis R(q)*e3, consistent with original E*L' attitude.
b3=zeros(3,N);
b3(1,:)=2*(localJMul(q(2,:),q(4,:))+localJMul(q(1,:),q(3,:)));
b3(2,:)=2*(localJMul(q(3,:),q(4,:))-localJMul(q(1,:),q(2,:)));
b3(3,:)=localJMul(q(1,:),q(1,:))-localJMul(q(2,:),q(2,:)) ...
        -localJMul(q(3,:),q(3,:))+localJMul(q(4,:),q(4,:));
proj=localJDot(pT,b3);
beta=localJMul(pT(3,:),proj)/(m+mL);
base=-localJConst(g,N)-m*L*localJMul(speed2,pT(3,:))/(m+mL);
zd=zeros(1,N);dz=zeros(1,N);ddz=zeros(1,N);
for k=0:N-1
    if k+1<=numel(ref),zd(k+1)=ref(k+1)/factorial(k);end
    if k+2<=numel(ref),dz(k+1)=ref(k+2)/factorial(k);end
    if k+3<=numel(ref),ddz(k+1)=ref(k+3)/factorial(k);end
end
if strcmp(mode,'virtual')
    v1=-F1(1)*(X(10,:)-zd)-F1(2)*(X(13,:)-dz)+X(20,:);
    thrust=localJDiv(ddz+v1-base,beta);
else
    thrust=X(20,:); % physical thrust is a state: f, fDot.
end
qDot=zeros(4,N);
qDot(1,:)=-localJDot(q(2:4,:),wb);
qDot(2,:)=localJMul(q(1,:),wb(1,:))-localJMul(q(4,:),wb(2,:))+localJMul(q(3,:),wb(3,:));
qDot(3,:)=localJMul(q(4,:),wb(1,:))+localJMul(q(1,:),wb(2,:))-localJMul(q(2,:),wb(3,:));
qDot(4,:)=-localJMul(q(3,:),wb(1,:))+localJMul(q(2,:),wb(2,:))+localJMul(q(1,:),wb(3,:));
qDot=qDot/2;
Jwb=bsxfun(@times,wb,J(:));
wbdot=-bsxfun(@rdivide,localJCross(wb,Jwb),J(:));
wbdot(:,1)=wbdot(:,1)+u(2:4)./J(:);
forceDir=localJRowMul(b3,thrust);
wLdot=localJCross(-pT,forceDir)/(m*L);
accScale=(localJDot(pT,forceDir)-m*L*speed2)/(m+mL);
vlDot=localJRowMul(pT,accScale);
vlDot(:,1)=vlDot(:,1)+[P(8);P(9);-g];
fjet=zeros(size(X,1),N);
fjet(1:4,:)=qDot;
fjet(5:7,:)=wbdot;
fjet(8:10,:)=vl;
fjet(11:13,:)=vlDot;
fjet(14:16,:)=ptDot;
fjet(17:19,:)=wLdot;
if strcmp(mode,'virtual')
    fjet(20:22,:)=X(21:23,:);
    fjet(23,1)=u(1);
else
    fjet(20,:)=X(21,:); % fDot = eta
    fjet(21,1)=u(1);  % etaDot = fDDot = nuF
end
end


%% NUMERIC TAYLOR POLYNOMIAL HELPERS
function c=localJConst(a,N)
c=zeros(1,N);c(1)=a;
end
function c=localJMul(a,b)
N=numel(a);c=conv(a,b);c=c(1:N);
end
function c=localJInv(a)
N=numel(a);
if ~(isfinite(a(1)) && abs(a(1))>=1e-10)
    error('Singular jet denominator at current state.');
end
c=zeros(1,N);c(1)=1/a(1);
for k=2:N
    c(k)=-(a(2:k)*flip(c(1:k-1).'))/a(1);
end
end
function c=localJDiv(a,b)
c=localJMul(a,localJInv(b));
end
function c=localJDot(a,b)
N=size(a,2);c=zeros(1,N);
for i=1:size(a,1),c=c+localJMul(a(i,:),b(i,:));end
end
function c=localJRowMul(a,b)
N=size(a,2);c=zeros(size(a,1),N);
for i=1:size(a,1),c(i,:)=localJMul(a(i,:),b);end
end
function c=localJCross(a,b)
N=size(a,2);c=zeros(3,N);
c(1,:)=localJMul(a(2,:),b(3,:))-localJMul(a(3,:),b(2,:));
c(2,:)=localJMul(a(3,:),b(1,:))-localJMul(a(1,:),b(3,:));
c(3,:)=localJMul(a(1,:),b(2,:))-localJMul(a(2,:),b(1,:));
end


function c=localJSqrt(a)
N=numel(a);
if a(1)<=1e-15 || ~isfinite(a(1)),error('Jet square root nonpositive/singular.');end
c=zeros(1,N);c(1)=sqrt(a(1));
for k=2:N
    s=0;
    for j=2:k-1,s=s+c(j)*c(k-j+1);end
    c(k)=(a(k)-s)/(2*c(1));
end
end
function S=localAxisShapeJet(pt,a,b)
N=size(pt,2);S=zeros(3,3,N);S(:,:,1)=b^2*eye(3);
for i=1:3
 for j=1:3
  temp=reshape(S(i,j,:),1,N)+(a^2-b^2)*localJMul(pt(i,:),pt(j,:));
  S(i,j,:)=reshape(temp,1,1,N);
 end
end
end

%% ========== Runtime assembly and QP (both modes, same source) ==========
function out=localFilter(mode,obj,x,xd,vfNom,F1,P,aux,obsList,method,shapes,kappa,opts)
if nargin<13 || isempty(opts),opts=struct();end
mode=localMode(mode);method=upper(char(method));
assert(numel(x)==19 && numel(xd)==28 && numel(vfNom)==6 && numel(P)==9 && numel(F1)==2);
assert(isstruct(obsList));
if strcmp(mode,'virtual'),assert(numel(aux)==4);else,assert(numel(aux)==2);end
if ~isfield(opts,'requireInitialChain'),opts.requireInitialChain=true;end
if ~isfield(opts,'uLimits'),opts.uLimits=[0;20;1;1;1];end
if ~isfield(opts,'qpFeasTol'),opts.qpFeasTol=1e-7;end
if ~isfield(opts,'diagnosticOnly'),opts.diagnosticOnly=false;end
if ~isfield(opts,'saveInitialFailure'),opts.saveInitialFailure=true;end
if ~isfield(opts,'diagnosticFile'),opts.diagnosticFile='CBF_initial_chain_failure.mat';end
clock=tic; A=zeros(0,4);b=zeros(0,1);
rows=struct('obstacleID',{},'component',{},'h',{},'psi',{},...
 'relativeDegree',{},'evaluationSeconds',{},'geomGap',{},...
 'psiChain',{},'firstViolatedIndex',{},'initialChainSatisfied',{});
refZ=xd([3,7,11,15,19,23,27]);
parts=localMethodSpecs(method);
for io=1:numel(obsList)
    if strcmp(method(2),'2')
        sl=localRequiredField(shapes,'spheres');
        assert(size(sl,2)==2 && size(sl,1)>=1,'shapes.spheres must be Nx2 [lambda radius]');
        kinds=repmat({'sphere'},1,size(sl,1));gs=cell(1,size(sl,1));
        for il=1:size(sl,1),gs{il}=sl(il,:).';end
    else
        kinds=parts;gs=cell(1,numel(parts));
        for il=1:numel(parts),gs{il}=localRequiredField(shapes,parts{il});end
    end
    for ip=1:numel(kinds)
        name=kinds{ip};geom=gs{ip};
        res=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,method,name,...
            x,aux,refZ,F1,P,obsList(io),geom,kappa);
        if ~res.info.valid
            error('CBF %s/%s/%s invalid derivative coefficient: %s',mode,method,name,res.info.reason);
        end
        if opts.requireInitialChain && ~res.info.initialChainSatisfied
            if isfield(obsList(io),'id'),obsID=obsList(io).id;else,obsID=io;end
            localReportInitialFailure(mode,method,name,obsID,io,...
                x,xd,vfNom,refZ,F1,P,aux,obsList,shapes,geom,kappa,res,opts);
            firstBad=find(res.info.psiChain < -1e-8,1,'first');
            error('CBF:InvalidInitialChain', ...
                ['CBF %s/%s/%s obs=%g: initial HOCBF psi_%d=%.8g < 0 ' ...
                 '(h=%.8g). See CBF initial-chain diagnostics above.'], ...
                mode,method,name,double(obsID),firstBad-1,...
                res.info.psiChain(firstBad),res.h);
        end
        sc=max([1,abs(res.braw),norm(res.Araw,inf)]);
        A(end+1,:)=res.Araw/sc; %#ok<AGROW>
        b(end+1,1)=res.braw/sc; %#ok<AGROW>
        if isfield(obsList(io),'id'),id=obsList(io).id;else,id=io;end
        firstBad=find(res.info.psiChain < -1e-8,1,'first');
        if isempty(firstBad),firstBad=0;else,firstBad=firstBad-1;end
        rows(end+1)=struct('obstacleID',id,'component',name,...
           'h',res.h,'psi',res.psi,'relativeDegree',res.info.relativeDegreeAssumed,...
           'evaluationSeconds',res.info.evaluationSeconds,'geomGap',res.info.geometricGap,...
           'psiChain',res.info.psiChain,'firstViolatedIndex',firstBad,...
           'initialChainSatisfied',res.info.initialChainSatisfied); %#ok<AGROW>
    end
end
if opts.diagnosticOnly
    % Diagnostic only: no QP, no auxiliary-state integration, no output command.
    out=struct('success',false,'diagnosticOnly',true,'mode',mode,...
        'method',method,'rows',rows,'allInitialChainsSatisfied',...
        all([rows.initialChainSatisfied]),'totalSeconds',toc(clock));
    fprintf('[CBF-DIAG] %s/%s: %d rows, all chains satisfied=%d, %.3f s\n',...
        mode,method,numel(rows),out.allInitialChainsSatisfied,out.totalSeconds);
    for ir=1:numel(rows)
        fprintf('  obs=%g %-7s h=% .7g firstBadPsi=%d psi=%s\n',...
            double(rows(ir).obstacleID),rows(ir).component,...
            rows(ir).h,rows(ir).firstViolatedIndex,...
            mat2str(rows(ir).psiChain.',5));
    end
    return
end
if strcmp(mode,'virtual')
    map=localVirtualMap(obj,x,xd,vfNom,F1,P,aux,A,b,opts);
    % Diagnostic before optimization: compare the nominal HLC input with
    % the virtual-map thrust, and test whether nominal virtual inputs
    % already satisfy the CBF rows. This does NOT change any constraints.
    if ~map.ok || (isfield(opts,'debugCBFTrace') && opts.debugCBFTrace)
        nCBF=size(A,1);
        hMin=NaN;
        if ~isempty(rows),hMin=min([rows.h]);end
        if nCBF>0
            % map.Aineq starts with the CBF rows, followed by 6 torque rows.
            nominalMargin=map.bineq(1:nCBF)-map.Aineq(1:nCBF,:)*map.wNom;
            minNominalMargin=min(nominalMargin);
            nominalViolations=nnz(nominalMargin < -opts.qpFeasTol);
        else
            minNominalMargin=NaN;
            nominalViolations=0;
        end
        fprintf(2,['[CBF-PRE-QP] virtual/%s | min(h)=%.9g m | ' ...
            'nominal CBF margin(min)=%.9g | violated CBF rows=%d/%d\n'],...
            method,hMin,minNominalMargin,nominalViolations,nCBF);
        fprintf(2,['[CBF-PRE-QP] raw HLC thrust=%.9g N | ' ...
            'current virtual-map thrust=%.9g N | delta0=%.9g | ' ...
            'thrust-limits=[%.9g, %.9g] N\n'],...
            map.nominalU(1),map.thrust,aux(1),opts.uLimits(1),opts.uLimits(2));
        nominalM=map.torqueT*map.wNom+map.torqueOffset;
        maxNominalMoment=max(abs(nominalM));
        momentWithinLimits=all(abs(nominalM)<=opts.uLimits(3:5)+opts.qpFeasTol);
        fprintf(2,['[CBF-PRE-QP] nominal torque max(abs)=%.9g Nm | ' ...
            'torque inside limits=%d | QP NOT YET EXECUTED\n'],...
            maxNominalMoment,momentWithinLimits);
    end
    if ~map.ok
        localReportThrustLimit(obj,x,xd,vfNom,F1,P,aux,map,opts,mode,method);
    end
else
    map=localPhysicalMap(obj,x,xd,vfNom,P,aux,A,b,opts);
end
sol=localQP(map,opts);
out=struct('success',false,'mode',mode,'method',method,'uSafe',[],...
 'auxDot',[],'w',[],'map',map,'qp',sol,'rows',rows,'rawA',A,'rawb',b,...
 'totalSeconds',toc(clock),'nominalU',map.nominalU);
if sol.success
    w=sol.w;
    if strcmp(mode,'virtual')
        vfSafe=localDeltaJet(vfNom,F1,aux,w(1));
        out.uSafe=[map.thrust;map.torqueT*w+map.torqueOffset];
        out.auxDot=[aux(2:4);w(1)];
        out.vfSafe=vfSafe;
    else
        out.uSafe=[aux(1);w(2:4)];
        out.auxDot=[aux(2);w(1)];
        out.vfSafe=[]; % physical safety filter does not preserve first-layer feedback.
    end
    out.w=w;out.success=true;
else
    warning('CBF QP unsuccessful: %s. Do not treat nominal input as safe fallback.',sol.reason);
end
end

function localReportThrustLimit(obj,x,xd,vfNom,F1,P,aux,map,opts,mode,method)
% Diagnostic only. Do NOT clamp the command or alter the QP/auxiliary state.
lim=opts.uLimits(:);
ref=reshape(xd,1,[]);
vfZero=reshape(vfNom,1,[]);vfZero(1)=0;
vfOne=vfZero;vfOne(1)=1;
ufZero=obj.Uf_SuspendedLoadxyDst(x,ref,vfZero,reshape(P,1,[]));
ufOne=obj.Uf_SuspendedLoadxyDst(x,ref,vfOne,reshape(P,1,[]));
fZero=ufZero(1);
fSlope=ufOne(1)-fZero;
v1Current=vfNom(1)+aux(1);
v1Feasible=[nan;nan];
deltaFeasible=[nan;nan];
if isfinite(fSlope) && abs(fSlope)>1e-12
    v1Feasible=sort(([lim(1);lim(2)]-fZero)/fSlope);
    deltaFeasible=v1Feasible-vfNom(1);
end
fprintf(2,'\n[CBF-THRUST FAILURE] virtual/%s | thrust bound failed; QP NOT attempted\n',method);
fprintf(2,'  proposed thrust f = %.12g N, permitted [%.12g, %.12g] N, nominal HLC f = %.12g N\n',...
    map.thrust,lim(1),lim(2),map.nominalU(1));
fprintf(2,'  v1_nom=%.12g, delta0=%.12g, effective v1=%.12g\n',...
    vfNom(1),aux(1),v1Current);
fprintf(2,'  affine thrust relation: f = %.12g + (%.12g)*v1\n',fZero,fSlope);
fprintf(2,'  v1 interval for thrust limits = [%.12g, %.12g]\n',v1Feasible(1),v1Feasible(2));
fprintf(2,'  delta0 interval (if modified) = [%.12g, %.12g]; DIAGNOSTIC ONLY\n',deltaFeasible(1),deltaFeasible(2));
fprintf(2,'  pLz=%.9g, vLz=%.9g, ref z=%.9g, ref ddz=%.9g, F1=%s\n',...
    x(10),x(13),ref(3),ref(11),mat2str(F1(:).',6));
fprintf(2,'  mass=%.9g, loadmass=%.9g, cableL=%.9g, pT=%s, q=%s\n',...
    P(1),P(6),P(7),mat2str(x(14:16).',6),mat2str(x(1:4).',6));
fprintf(2,'  The decision nu changes delta0 only through its integrator chain.\n');
fprintf(2,'  NO automatic saturation, no unsafe fallback, and no claim of QP infeasibility.\n');
if ~isfield(opts,'saveThrustFailure') || opts.saveThrustFailure
    if isfield(opts,'thrustFailureFile') && ~isempty(opts.thrustFailureFile)
        file=opts.thrustFailureFile;
    else
        file='CBF_thrust_limit_failure.mat';
    end
    CBF_THRUST=struct('mode',mode,'method',method,'x',x,'xd',xd,...
        'vfNom',vfNom,'F1',F1,'P',P,'aux',aux,'uLimits',lim,...
        'proposedThrust',map.thrust,'nominalThrust',map.nominalU(1),...
        'fAtV1Zero',fZero,'fPerUnitV1',fSlope,...
        'v1Current',v1Current,'v1Feasible',v1Feasible,...
        'delta0Feasible',deltaFeasible);
    try
        save(file,'CBF_THRUST');
        fprintf(2,'  Snapshot: %s\n\n',file);
    catch ME
        fprintf(2,'  Snapshot save failed: %s\n\n',ME.message);
    end
end
end

function localReportInitialFailure(mode,method,name,obsID,obsIndex,...
    x,xd,vfNom,refZ,F1,P,aux,obsList,shapes,geom,kappa,res,opts)
% A failed HOCBF chain must not be silently ignored. Save a reproducible
% single-state snapshot and report which *specific* chain condition failed.
chain=res.info.psiChain(:);
firstBad=find(chain < -1e-8,1,'first');
fprintf(2,'\n[CBF-INIT FAILURE] mode=%s method=%s component=%s obstacle id=%g (#%d)\n',...
    mode,method,name,double(obsID),obsIndex);
fprintf(2,'  h=psi_0=% .10g | geometricGap=% .10g | first failure psi_%d=% .10g\n',...
    res.h,res.info.geometricGap,firstBad-1,chain(firstBad));
for j=1:numel(chain)
    fprintf(2,'  psi_%d = % .11e  %s\n',j-1,chain(j),...
        localChainStatus(chain(j)));
end
fprintf(2,'  pL=%s  vL=%s  pT=%s  kappa=%s\n',...
    mat2str(x(8:10).',6),mat2str(x(11:13).',6),...
    mat2str(x(14:16).',6),mat2str(kappa(:).',6));
fprintf(2,'  This is a high-order *initial-set* violation, not a quadprog failure.\n');
if opts.saveInitialFailure
    CBF_INIT=struct('mode',mode,'method',method,'component',name,...
        'obstacleID',obsID,'obstacleIndex',obsIndex,...
        'x',x,'xd',xd,'vfNom',vfNom,'refZ',refZ,...
        'F1',F1,'P',P,'aux',aux,'obsList',obsList,...
        'obs',obsList(obsIndex),'shapes',shapes,'geom',geom,...
        'kappa',kappa,'h',res.h,...
        'psiChain',chain,'firstViolatedIndex',firstBad-1,...
        'evaluation',res,'reason','CBF initial HOCBF chain violated');
    try
        save(opts.diagnosticFile,'CBF_INIT');
        fprintf(2,'  Diagnostic snapshot saved to %s\n',opts.diagnosticFile);
    catch ME
        fprintf(2,'  WARNING: diagnostic snapshot not saved: %s\n',ME.message);
    end
end
end

function status=localChainStatus(value)
if value >= -1e-8, status='OK';else,status='VIOLATED';end
end

function value=localRequiredField(s,name)
if ~isfield(s,name),error('shapes.%s missing',name);end
value=s.(name);
end

function vfSafe=localDeltaJet(vfNom,F1,delta,nu)
assert(numel(vfNom)==6 && numel(F1)==2 && numel(delta)==4);
K=F1(:).';delta=delta(:);d=[delta;nu];
Ac=[0 1;0 0]-[0;1]*K;B=[0;1];vfSafe=reshape(vfNom,1,[]);
for k=0:4
    dv=d(k+1);
    for j=0:k-1,dv=dv-K*(Ac^(k-1-j))*B*d(j+1);end
    vfSafe(k+1)=vfSafe(k+1)+dv;
end
if size(vfNom,1)==6,vfSafe=vfSafe.';end
end

function map=localVirtualMap(obj,x,xd,vfNom,F1,P,delta,Araw,braw,opts)
if ~isfield(opts,'rcondMin'),opts.rcondMin=1e-9;end
if ~isfield(opts,'auxOmega'),opts.auxOmega=1;end
if ~isfield(opts,'uLimits'),opts.uLimits=[0;20;1;1;1];end
xd=reshape(xd,1,[]);P=reshape(P,1,[]);delta=delta(:);lim=opts.uLimits(:);
vf0=localDeltaJet(vfNom,F1,delta,0);
vf1=localDeltaJet(vfNom,F1,delta,1);
vfm=localDeltaJet(vfNom,F1,delta,-1);
B0=obj.Beta2_SuspendedLoadxyDst(x,xd,vf0,P);
B1=obj.Beta2_SuspendedLoadxyDst(x,xd,vf1,P);
Bm=obj.Beta2_SuspendedLoadxyDst(x,xd,vfm,P);
if any(~isfinite(B0(:))) || rcond(B0)<opts.rcondMin,error('HLC beta2 singular');end
if norm(B1-B0,'fro')>1e-7*(1+norm(B0,'fro')) || norm(Bm-B0,'fro')>1e-7*(1+norm(B0,'fro'))
    error('beta2 depends on nu: virtual conversion is not affine.');
end
alpha0=-obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vf0,zeros(3,1),P);
alpha1=-obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vf1,zeros(3,1),P);
alphaM=-obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vfm,zeros(3,1),P);
alpha0=alpha0(:);alpha1=alpha1(:);alphaM=alphaM(:);
alphaNu=(alpha1-alphaM)/2;
if any(~isfinite([alpha0;alpha1;alphaM])) || norm(alpha1+alphaM-2*alpha0)>1e-5*(1+norm(alpha0))
    error('alpha2 nonlinear in nu or numerically ill-conditioned');
end
D=B0\eye(3);T=[-D*alphaNu,D];c=-D*alpha0;
Av=Araw(:,2:4)*D;
Avirtual=[Araw(:,1)-Av*alphaNu,Av];
bvirtual=braw-Av*alpha0;
Mmax=lim(3:5);
map.Aineq=[-Avirtual;T;-T];
map.bineq=[bvirtual;Mmax-c;Mmax+c];
vs=obj.Vs_SuspendedLoadxyDst(x,xd,vf0,P,obj.param.F2,obj.param.F3,obj.param.F4);
omega=opts.auxOmega;
nuNom=-[omega^4,4*omega^3,6*omega^2,4*omega]*delta;
map.wNom=[nuNom;vs(:)];
uf=obj.Uf_SuspendedLoadxyDst(x,xd,vf0,P);
map.thrust=uf(1);
ufNom=obj.Uf_SuspendedLoadxyDst(x,xd,vfNom,P);
Bnom=obj.Beta2_SuspendedLoadxyDst(x,xd,vfNom,P);
vsNom=obj.Vs_SuspendedLoadxyDst(x,xd,vfNom,P,obj.param.F2,obj.param.F3,obj.param.F4);
Mnom=Bnom\obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vfNom,vsNom(:),P);
map.nominalU=[ufNom(1);Mnom(:)];
map.torqueT=T;map.torqueOffset=c;
map.ok=isfinite(map.thrust) && map.thrust>=lim(1) && map.thrust<=lim(2);
map.reason=sprintf('current commanded thrust %.9g N outside [%.9g,%.9g] N; nu cannot change f instantly',map.thrust,lim(1),lim(2));
if ~all(isfinite([map.Aineq(:);map.bineq(:);map.wNom(:);map.nominalU(:)])),error('Nonfinite virtual QP map');end
end

function map=localPhysicalMap(obj,x,xd,vfNom,P,aux,Araw,braw,opts)
if ~isfield(opts,'uLimits'),opts.uLimits=[0;20;1;1;1];end
if ~isfield(opts,'physicalThrustGains'),opts.physicalThrustGains=[4;4];end
if ~isfield(opts,'physicalOmega'),opts.physicalOmega=3;end
lim=opts.uLimits(:);thrustGains=opts.physicalThrustGains(:);
assert(numel(thrustGains)==2 && all(thrustGains>0));
xd=reshape(xd,1,[]);P=reshape(P,1,[]);vfNom=reshape(vfNom,1,[]);
f=aux(1);eta=aux(2);
% IMPORTANT: the flight supervisor already computed the original HLC
% command in HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT/do. Use THAT saturated
% command directly; recalculating Vf/Vs/Beta2/V2_alpha2 inside the
% safety filter is redundant, adds latency, and may introduce inconsistent
% argument shapes. No nominal controller law or CBF inequality is changed.
if isfield(opts,'nominalActuatorU') && ~isempty(opts.nominalActuatorU)
    base=opts.nominalActuatorU(:);
    assert(numel(base)==4 && all(isfinite(base)), ...
        'CBF:NominalInputShape','nominalActuatorU must be a finite 4-vector.');
    map.nominalU=base;
    Mnom=base(2:4);
else
    % Preserve standalone/direct 'filter' support when the already-computed
    % nominal actuator command has not been supplied by a supervisor.
    uf=obj.Uf_SuspendedLoadxyDst(x,xd,vfNom,P);
    B=obj.Beta2_SuspendedLoadxyDst(x,xd,vfNom,P);
    vs=obj.Vs_SuspendedLoadxyDst(x,xd,vfNom,P,obj.param.F2,obj.param.F3,obj.param.F4);
    if any(~isfinite(B(:))) || rcond(B)<1e-9
        error('CBF:NominalBetaSingular','HLC nominal beta2 singular');
    end
    Mnom=B\obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vfNom,vs(:),P);
    map.nominalU=[uf(1);Mnom(:)];
end
% Delta physical input w=[fDDot;M]. The current thrust f is NOT a QP variable.
nuFnom=opts.physicalOmega^2*(map.nominalU(1)-f)-2*opts.physicalOmega*eta;
map.wNom=[nuFnom;Mnom(:)];
% Lower thrust HOCBF: nuF + (g1+g2)*eta + g1*g2*(f-fMin)>=0
% Upper thrust HOCBF: -nuF -(g1+g2)*eta+g1*g2*(fMax-f)>=0
% Matrix inequalities Aineq*w<=bineq.
assert(size(Araw,2)==4 && numel(braw)==size(Araw,1), ...
    'CBF:PhysicalMapDimensions', ...
    'Expected Araw as N-by-4 and braw as N-by-1; got Araw=%s, braw=%s.', ...
    mat2str(size(Araw)),mat2str(size(braw)));
braw=braw(:);
map.Aineq=[-Araw; -1 0 0 0; 1 0 0 0;...
          zeros(3,1),eye(3); zeros(3,1),-eye(3)];
map.bineq=[braw;sum(thrustGains)*eta+prod(thrustGains)*(f-lim(1));...
                -sum(thrustGains)*eta+prod(thrustGains)*(lim(2)-f);lim(3:5);lim(3:5)];
% Optional FLIGHT EXPERIMENT: CBF may change fDDot, Mx, My but NOT Mz.
% Encode Mz=M_z,nominal (the original SATURATED HLC yaw command) as
% TWO HARD inequalities inside BOTH ordinary and anticipatory QPs.
% Never post-edit the QP command: it would invalidate the safety check.
% Keep the 9 hard CBF inequalities and the existing 8 actuator rows intact.
map.yawLocked=false;
if isfield(opts,'lockNominalYaw') && logical(opts.lockNominalYaw)
    assert(isfield(opts,'nominalActuatorU') && numel(opts.nominalActuatorU)==4, ...
        'CBF:YawLockRequiresNominal', ...
        'lockNominalYaw needs the 4-element saturated original HLC input.');
    yawHLC=opts.nominalActuatorU(4);
    map.Aineq=[map.Aineq; 0 0 0 1; 0 0 0 -1];
    map.bineq=[map.bineq; yawHLC; -yawHLC];
    map.yawLocked=true;
    map.yawNominal=yawHLC;
end
map.ok= f>=lim(1) && f<=lim(2) && ...
        eta+thrustGains(1)*(f-lim(1))>=-1e-8 && ...
       -eta+thrustGains(1)*(lim(2)-f)>=-1e-8;
map.reason='physical thrust bound or first HOCBF chain invalid';
map.thrust=f;map.torqueT=[];map.torqueOffset=[];
if ~all(isfinite([map.Aineq(:);map.bineq(:);map.wNom(:);map.nominalU(:)])),error('Nonfinite physical QP map');end
end

function sol=localQP(map,opts)
if ~isfield(opts,'virtualScales'),opts.virtualScales=[1;1;1;1];end
if ~isfield(opts,'physicalScales'),opts.physicalScales=[10;1;1;1];end
if ~isfield(opts,'qpFeasTol'),opts.qpFeasTol=1e-7;end
sol=struct('success',false,'reason','QP not solved','w',nan(4,1),'exitflag',-999,'fval',nan);
if ~map.ok,sol.reason=map.reason;return;end
if isempty(map.torqueT),sc=opts.physicalScales(:);else,sc=opts.virtualScales(:);end
if numel(sc)~=4 || any(sc<=0),error('QP scales must be positive 4-vector');end
[H,q]=localQPTrackingObjective(map,opts,sc);
opt=optimoptions('quadprog','Display','off');
[w,fv,ef]=quadprog(H,q,map.Aineq,map.bineq,[],[],[],[],[],opt);
sol.exitflag=ef;sol.fval=fv;
if ef>0 && all(isfinite(w)) && all(map.Aineq*w<=map.bineq+opts.qpFeasTol)
    sol.success=true;sol.reason='sample QP feasible; NOT a global safety proof';sol.w=w;
else
    sol.reason='quadprog failed or sampled inequalities violated';
end
end

function [x,aux,refZ,F1,P,obs,geom,kappa]=localExample(mode,name)
roll=.13;pitch=-.17;yaw=.23;
q=[cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);...
 sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);...
 cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);...
 cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2)];
pt=[.14;-.10;-.9849];pt=pt/norm(pt);
wl=[.06;-.08;.03];wl=wl-pt*(pt.'*wl);
x=[q;.11;-.06;.04;.1;.1;2;.3;-.1;.05;pt;wl];
if strcmp(mode,'virtual'),aux=zeros(4,1);else,aux=[15;0.05];end
refZ=[2;0;0;0;0;0;0];F1=[5;4];
P=[1.3;.020;.019;.035;9.81;.4;1.1;0;0];
obs=struct('p_center',[3;2;4],'ellipsoid_radii',[.6;.5;.8],...
    'R_obs',eye(3),'d_margin',.2,'v_center',zeros(3,1));
switch name
    case 'single',geom=[1.1;.28];
    case 'sphere',geom=[.5;.25];
    case 'cable',geom=[.65;.12];
    case 'uav',geom=[.28;.22;.20];
    case 'load',geom=[.28;.22;.20];
end
kappa=[2;2;2;2;2;2];
end


%% =======================================================================
%  Actuator-aware safety supervisor (prototype, requires viability audit)
%  requestedMode = 'virtual' or 'physical'. One source file for all modes.
%  State must be carried between control steps, NEVER reset every call.
%  Modes: MONITOR -> ACTIVE -> MONITOR, with hysteresis and output-matching.
%  IMPORTANT: monitoring is a pointwise physical-input HOCBF proxy, not a
%  future-trajectory certificate. Do not claim formal collision avoidance.
%% =======================================================================
function out=localSupervise(requestedMode,obj,x,xd,vfNom,F1,P,uRaw,obsList,method,shapes,kappa,opts,st,dt)
clock=tic;
requestedMode=localMode(requestedMode);method=upper(char(method));
if nargin<13 || isempty(opts),opts=struct();end
if nargin<14 || isempty(st),st=struct();end
assert(isfinite(dt) && dt>0,'dt must be positive');
assert(numel(uRaw)==4 && all(isfinite(uRaw)),'HLC input must be finite');
assert(isstruct(obsList) && ~isempty(obsList),'Obstacle list required');
assert(numel(x)==19 && numel(xd)==28 && numel(vfNom)==6);
if ~isfield(opts,'uLimits'),opts.uLimits=[0;20;1;1;1];end
if ~isfield(opts,'onMargin'),opts.onMargin=0;end
if ~isfield(opts,'offMargin'),opts.offMargin=0.01;end
if ~isfield(opts,'releaseHoldSteps'),opts.releaseHoldSteps=5;end
if ~isfield(opts,'releaseForceTol'),opts.releaseForceTol=0.10;end
if ~isfield(opts,'releaseMomentTol'),opts.releaseMomentTol=0.02;end
if ~isfield(opts,'releaseAuxTol'),opts.releaseAuxTol=0.05;end
if ~isfield(opts,'allowPhysicalFallback'),opts.allowPhysicalFallback=true;end
if ~isfield(opts,'monitorOnly'),opts.monitorOnly=false;end
if ~isfield(opts,'qpFeasTol'),opts.qpFeasTol=1e-7;end
if ~isfield(opts,'rampLimitFddot'),opts.rampLimitFddot=Inf;end
if ~isfield(opts,'rampLimitFdot'),opts.rampLimitFdot=Inf;end
if ~isfield(opts,'rampLimitMoment'),opts.rampLimitMoment=Inf;end
if ~isfield(opts,'emitEvents'),opts.emitEvents=true;end
if ~isfield(opts,'preemptivePush'),opts.preemptivePush=0;end
if ~isfield(opts,'preemptiveWeight'),opts.preemptiveWeight=100;end
if ~isfield(opts,'preemptiveMaxRows'),opts.preemptiveMaxRows=3;end
% GENERIC anticipatory safety-filter heuristic (NO change to reference/HLC).
% Uses trend of EACH normalized physical CBF margin, independent of number,
% location or order of obstacles. Not a prediction of future HOCBF states.
if ~isfield(opts,'navigationLeadSeconds'),opts.navigationLeadSeconds=0;end
if ~isfield(opts,'navigationMinClosingRate'),opts.navigationMinClosingRate=0.002;end
if ~isfield(opts,'navigationRateBlend'),opts.navigationRateBlend=0.65;end
if ~isfield(opts,'navigationMaxPush'),opts.navigationMaxPush=opts.preemptivePush;end
if ~isfield(opts,'navigationConfirmSteps'),opts.navigationConfirmSteps=1;end
if ~isfield(opts,'smoothInputWeight'),opts.smoothInputWeight=0;end
if ~isfield(opts,'failOnGeometricCertificateLoss'),opts.failOnGeometricCertificateLoss=false;end
assert(isfinite(opts.navigationLeadSeconds) && opts.navigationLeadSeconds>=0);
assert(isfinite(opts.navigationMinClosingRate) && opts.navigationMinClosingRate>=0);
assert(isfinite(opts.navigationRateBlend) && opts.navigationRateBlend>=0 && opts.navigationRateBlend<=1);
assert(isfinite(opts.navigationMaxPush) && opts.navigationMaxPush>=opts.preemptivePush);
assert(isfinite(opts.navigationConfirmSteps) && opts.navigationConfirmSteps>=1 && ...
    opts.navigationConfirmSteps==floor(opts.navigationConfirmSteps));
assert(isfinite(opts.smoothInputWeight) && opts.smoothInputWeight>=0);
% EXPERIMENTAL, OFF by default: after VALID engagement, allow psi_j<0 to
% continue through the HARD CBF QP. This does NOT restore HOCBF guarantees.
if ~isfield(opts,'experimentalContinueAfterPsiViolation'),opts.experimentalContinueAfterPsiViolation=false;end
if ~isfield(opts,'experimentalStopAtGeometricBoundary'),opts.experimentalStopAtGeometricBoundary=true;end
if ~isfield(opts,'experimentalTraceFile'),opts.experimentalTraceFile='CBF_chain_recovery_trial.csv';end
assert(isfinite(opts.preemptivePush) && opts.preemptivePush>=0, ...
    'preemptivePush must be a finite nonnegative normalized margin');
assert(isfinite(opts.preemptiveWeight) && opts.preemptiveWeight>0, ...
    'preemptiveWeight must be positive');
assert(isfinite(opts.preemptiveMaxRows) && opts.preemptiveMaxRows>=1 && ...
    opts.preemptiveMaxRows==floor(opts.preemptiveMaxRows), ...
    'preemptiveMaxRows must be a positive integer');
lim=opts.uLimits(:);
assert(numel(lim)==5 && lim(1)<lim(2) && all(lim(3:5)>0));
assert(opts.onMargin<opts.offMargin,'onMargin must be below offMargin');
assert(isfinite(opts.releaseHoldSteps) && opts.releaseHoldSteps>=1);
uRaw=uRaw(:);
uBase=[min(max(uRaw(1),lim(1)),lim(2));...
       min(max(uRaw(2:4),-lim(3:5)),lim(3:5))];
if ~isfield(st,'active'),st.active=false;end
if ~isfield(st,'mode'),st.mode='';end
if ~isfield(st,'aux'),st.aux=[];end
if ~isfield(st,'releaseCount'),st.releaseCount=0;end
if ~isfield(st,'lastU'),st.lastU=uBase;end
if ~isfield(st,'step'),st.step=0;end
if ~isfield(st,'experimentalChainWasBad'),st.experimentalChainWasBad=false;end
if ~isfield(st,'experimentalBadSamples'),st.experimentalBadSamples=0;end
if ~isfield(st,'experimentalRecoveries'),st.experimentalRecoveries=0;end
if ~isfield(st,'experimentalMinPsi'),st.experimentalMinPsi=Inf;end
if ~isfield(st,'experimentalMinH'),st.experimentalMinH=Inf;end
if ~isfield(st,'experimentalFirstViolationTime'),st.experimentalFirstViolationTime=NaN;end
if ~isfield(st,'experimentalCsvInitialized'),st.experimentalCsvInitialized=false;end
st.step=st.step+1;

% Monitor with actual saturated thrust NOW, frozen during the derivative
% chain, and actual saturated torques. It is not the time-varying HLC law.
try
    monitor=localMonitorSaturatedPhysical(x,xd,F1,P,uBase,obsList,method,shapes,kappa);
catch ME
    % Read-only failure snapshot: diagnose WHICH kinematic/model assertion
    % failed instead of hiding the original exception behind MonitorInvalid.
    % Never project/renormalize an EKF state inside the CBF filter: doing so
    % would silently change the model and could invalidate safety claims.
    if isprop(obj,'cbfTime'),failureTime=obj.cbfTime;else,failureTime=NaN;end
    xDiagnostic=x(:);
    qNorm=NaN;pTNorm=NaN;pTDotWL=NaN;
    if numel(xDiagnostic)>=19
        qNorm=norm(xDiagnostic(1:4));
        pTNorm=norm(xDiagnostic(14:16));
        pTDotWL=dot(xDiagnostic(14:16),xDiagnostic(17:19));
    end
    fprintf(2,'\n[CBF-MON-FAIL] t=%.5f original=%s: %s\n', ...
        failureTime,ME.identifier,ME.message);
    fprintf(2,['[CBF-MON-FAIL] norm(q)=%.12g (target 1, tol 1e-4); ' ...
        'norm(pT)=%.12g (target 1, tol 1e-4); ' ...
        'pT''*wL=%.12g (tol 1e-3)\n'],qNorm,pTNorm,pTDotWL);
    fprintf(2,'[CBF-MON-FAIL] x=%s, P=%s, uBase=%s\n', ...
        mat2str(size(x)),mat2str(size(P)),mat2str(uBase(:).',6));
    for ist=1:min(numel(ME.stack),12)
        fprintf(2,'[CBF-MON-STACK] %s, line %d\n', ...
            ME.stack(ist).name,ME.stack(ist).line);
    end
    % If the simulation plant state is accessible, capture ground truth
    % separately for comparison with the 19-element estimated CBF state.
    % Read only: never overwrite the plant, estimator or state manager.
    plantTruth=struct();
    try
        plantState=obj.self.plant.state;
        truthNames={'q','w','p','pL','pT','vL','wL'};
        for it=1:numel(truthNames)
            nm=truthNames{it};
            if isprop(plantState,nm)
                plantTruth.(nm)=plantState.(nm);
            end
        end
    catch truthError
        plantTruth.captureError=truthError.message;
    end
    CBF_MON_FAIL=struct('time',failureTime,'x',x(:),'xd',xd(:), ...
        'vfNom',vfNom(:),'F1',F1(:),'P',P(:),'uRaw',uRaw(:), ...
        'uBase',uBase(:),'obstacles',obsList,'method',method, ...
        'shapes',shapes,'kappa',kappa(:),'opts',opts, ...
        'supervisor',st,'dt',dt,'originalID',ME.identifier, ...
        'originalMessage',ME.message,'originalStack',ME.stack, ...
        'qNorm',qNorm,'pTNorm',pTNorm,'pTDotWL',pTDotWL, ...
        'plantTruth',plantTruth);
    try
        save('CBF_monitor_invalid.mat','CBF_MON_FAIL');
        fprintf(2,'[CBF-MON-FAIL] Snapshot saved: CBF_monitor_invalid.mat\n');
    catch saveError
        fprintf(2,'[CBF-MON-FAIL] Snapshot save failed: %s\n',saveError.message);
    end
    error('CBF:MonitorInvalid', ...
        'Actuator-aware monitor invalid at t=%.5f: [%s] %s. Check CBF_monitor_invalid.mat.', ...
        failureTime,ME.identifier,ME.message);
end
sigma=monitor.minNormalizedMargin;
% Update a time-to-margin heuristic using the actual measured change
% of each barrier row. No assumptions about the obstacle arrangement or
% the nominal trajectory are required. Only supervisor-side memory changes.
[nav,st]=localCBFMarginTrend(monitor,st,opts,dt);
% Fail closed if the geometry separation certificate is lost. This
% does not imply that a physical collision has already taken place.
if opts.failOnGeometricCertificateLoss && monitor.minH <= 0
    error('CBF:GeometryCertificateLost', ...
        'CBF S-geometry certificate hMin=%g <=0. Refusing uncertified command.', ...
        monitor.minH);
end
% Strict geometry *certificate* guard for the research continuation.
% S geometry h<=0 means the chosen separating-plane certificate is LOST;
% it does NOT prove actual intersection of the two solid ellipsoids.
if opts.experimentalContinueAfterPsiViolation && ...
        opts.experimentalStopAtGeometricBoundary && monitor.minH<=0
    if isprop(obj,'cbfTime'),tt=obj.cbfTime;else,tt=NaN;end
    fprintf(2,['\n[CBF-EXP] GEOMETRY-CERTIFICATE STOP t=%.5f: ' ...
        'S-method minimum h=%.8g <=0; NOT a confirmed solid-body collision.\n'], ...
        tt,monitor.minH);
    CBF_EXP_STOP=struct('time',tt,'x',x,'xd',xd,'monitor',monitor,...
        'supervisor',st,'reason','S geometry separation margin nonpositive');
    try
        save('CBF_experimental_geometry_stop.mat','CBF_EXP_STOP');
    catch saveErr
        fprintf(2,'[CBF-EXP] Geometry-stop snapshot save failed: %s\n',saveErr.message);
    end
    error('CBF:ExperimentalGeometryGuard', ...
        'Experimental trial stopped: min separation certificate h=%.8g <=0',monitor.minH);
end
% Engaging only on the terminal inequality may be too late if a lower
% order HOCBF chain condition has already become negative. Treat that
% as a fail-closed trigger rather than silently accept the baseline.
needIntervene = (sigma<=opts.onMargin) || nav.risk || ~monitor.allInitialChainsSatisfied;
status='MONITOR'; reason='saturated HLC accepted by instantaneous CBF monitor';
releasing=false; engaged=false;
if ~st.active && needIntervene
    if opts.monitorOnly
        status='TRIGGER_DIAGNOSTIC';reason='monitorOnly: would intervene; output is NOT certified safe';
    else
        if ~monitor.allInitialChainsSatisfied
            idx=find(~[monitor.rows.initialChainSatisfied],1,'first');
            row=monitor.rows(idx);
            error('CBF:TooLateToEngage',...
                ['Physical HOCBF chain already violated at activation: ' ...
                 'obstacle=%g component=%s, h=%g. Engagement not certified.'],...
                double(row.obstacleID),row.component,row.h);
        end
        % Virtual dynamic extension cannot alter THRUST immediately.
        % Saturation of MOMENTS alone is handled by virtual-map QP bounds
        % and must not force a physical-mode fallback or a false stop.
        thrustSaturated=(abs(uRaw(1)-uBase(1))>opts.qpFeasTol);
        if strcmp(requestedMode,'virtual') && thrustSaturated
            if ~opts.allowPhysicalFallback
                error('CBF:VirtualSaturated',...
                    'Virtual extension cannot correct saturated thrust instantly. Enable physical fallback or redesign.');
            end
            st.mode='physical';
            reason='virtual requested, physical fallback because HLC is saturated';
        else
            st.mode=requestedMode;
            if sigma<=0
                reason='baseline violates instantaneous physical CBF inequality';
            else
                reason='early activation: current/trend-based positive-margin supervision; hard CBF unchanged';
            end
        end
        if strcmp(st.mode,'virtual')
            st.aux=zeros(4,1);
        else
            st.aux=[uBase(1);0]; % exactly match actuator thrust at entry
        end
        st.active=true;st.releaseCount=0;status='ENGAGE';engaged=true;
        % Discard the previous engagement trace; start a new chain audit.
        st.lastActiveTrace=[];
        st.experimentalChainWasBad=false;
        st.experimentalBadSamples=0;st.experimentalRecoveries=0;
        st.experimentalMinPsi=Inf;st.experimentalMinH=Inf;
        st.experimentalFirstViolationTime=NaN;
        st.experimentalCsvInitialized=false;
        if opts.emitEvents
            fprintf('[CBF-SUP] ENGAGE %s/%s | sigma=%+.4g | hmin=%.4g | %s\n',...
                 st.mode,method,sigma,monitor.minH,reason);
            if opts.navigationLeadSeconds>0
                fprintf('[CBF-NAV] predictedMin=%+.4g lead=%.3gs minTimeToTrigger=%.3gs (trend heuristic)\n', ...
                    nav.minPredicted,opts.navigationLeadSeconds,nav.minTimeToTrigger);
            end
        end
    end
end

if st.active && strcmp(st.mode,'virtual')
    % Dynamic-extended v1 cannot respect a new saturation instantaneously.
    % Transition to physical branch at last APPLIED f; the transition is
    % NOT a certified hybrid switch and is explicitly flagged.
    vf0=localDeltaJet(vfNom,F1,st.aux,0);
    uf0=obj.Uf_SuspendedLoadxyDst(x,xd(:).',vf0(:).',P(:).');
    if uf0(1)<lim(1)-opts.qpFeasTol || uf0(1)>lim(2)+opts.qpFeasTol
        if ~opts.allowPhysicalFallback
            error('CBF:VirtualSaturatedWhileActive',...
                'Virtual state would command thrust outside bounds.');
        end
        st.mode='physical';
        st.aux=[st.lastU(1);0];
        st.releaseCount=0;
        reason='active virtual thrust invalid: switch to physical from last applied thrust';
        status='SWITCH_PHYSICAL_UNCERTIFIED';
        if opts.emitEvents,fprintf(2,'[CBF-SUP] %s\n',reason);end
    end
end

if ~st.active
    st.lastU=uBase;
    out=localSupervisorResult(true,uBase,uBase,uRaw,monitor,st,status,reason,clock);
    out.intervened=false;
    out.navigation=nav;
    out.supervisor=st;
    return;
end

fopts=opts;
% Only the RUNTIME high-order chain test is relaxed IN THIS EXPERIMENT.
% The engagement check above remains strict; A*w<=b still remains HARD.
fopts.requireInitialChain=~opts.experimentalContinueAfterPsiViolation;
fopts.nominalActuatorU=uBase;
fopts.monitorOnly=false;
if opts.smoothInputWeight>0 && isfield(st,'lastActiveTrace') && ...
        isstruct(st.lastActiveTrace) && isfield(st.lastActiveTrace,'w') && ...
        numel(st.lastActiveTrace.w)==4
    fopts.smoothPreviousW=st.lastActiveTrace.w(:);
end
% Physical state f is shared with QP; add sampled-data actuator feasibility.
try
    res=localFilter(st.mode,obj,x,xd,vfNom,F1,P,st.aux,obsList,...
                    method,shapes,kappa,fopts);
catch ME
    % Diagnostic only: compare the LAST accepted physical HOCBF chain
    % against the current chain loss. Do not alter the control command.
    if strcmp(ME.identifier,'CBF:InvalidInitialChain')
        try
            localReportActiveChainTransition(obj,st,x,xd,uBase,monitor,opts,ME);
        catch diagError
            fprintf(2,'[CBF-CHAIN] Additional diagnostic failed: %s\n',diagError.message);
        end
    end
    error('CBF:ActiveFilterInvalid','Active %s/%s filter invalid: %s',st.mode,method,ME.message);
end
% Observe loss/recovery of lower-order HOCBF-chain conditions.
% Negative psi is NOT geometric contact. But the forward-invariance
% theorem no longer applies after this event, even if the QP solves.
expBadRows=[];expPsiMin=NaN;expPsiLastMin=NaN;expMinH=NaN;
if opts.experimentalContinueAfterPsiViolation
    expBadRows=find(~[res.rows.initialChainSatisfied]);
    psiCells={res.rows.psiChain};
    expPsiMin=min(cellfun(@(v)min(v(:)),psiCells));
    expPsiLastMin=min(cellfun(@(v)v(end),psiCells));
    expMinH=min([res.rows.h]);
    st.experimentalMinPsi=min(st.experimentalMinPsi,expPsiMin);
    st.experimentalMinH=min(st.experimentalMinH,expMinH);
    if isprop(obj,'cbfTime'),tt=obj.cbfTime;else,tt=NaN;end
    if ~isempty(expBadRows)
        st.experimentalBadSamples=st.experimentalBadSamples+1;
        if ~st.experimentalChainWasBad
            j=expBadRows(1);psi=res.rows(j).psiChain(:);
            firstBad=find(psi < -1e-8,1,'first');
            if isempty(firstBad),firstBad=1;end
            st.experimentalChainWasBad=true;
            if isnan(st.experimentalFirstViolationTime)
                st.experimentalFirstViolationTime=tt;
            end
            fprintf(2,['\n[CBF-EXP] CHAIN VIOLATION: CONTINUING QP t=%.5f ' ...
                'obs=%g %s psi_%d=%+.7g, h=%.7g. ' ...
                'CBF SAFETY GUARANTEE LOST.\n'],tt,...
                double(res.rows(j).obstacleID),res.rows(j).component,...
                firstBad-1,psi(firstBad),res.rows(j).h);
            CBF_EXP_FIRST_LOSS=struct('time',tt,'x',x,'xd',xd,...
                'aux',st.aux,'rows',res.rows,'monitor',monitor,'supervisor',st);
            try
                save('CBF_experimental_first_chain_loss.mat','CBF_EXP_FIRST_LOSS');
            catch saveErr
                fprintf(2,'[CBF-EXP] Chain-loss snapshot save failed: %s\n',saveErr.message);
            end
        end
    elseif st.experimentalChainWasBad
        st.experimentalChainWasBad=false;
        st.experimentalRecoveries=st.experimentalRecoveries+1;
        fprintf(2,['[CBF-EXP] CHAIN RECOVERED t=%.5f ' ...
            'all %d chains nonnegative again; guarantee NOT retroactively restored.\n'],...
            tt,numel(res.rows));
    end
end
if ~res.success
    % Diagnostic only: store the exact failed QP and check whether its
    % linear inequalities admit ANY solution. Never modify the command.
    localReportActiveQPFailure(obj,x,xd,vfNom,F1,P,uRaw,uBase,...
        obsList,method,shapes,kappa,opts,st,dt,monitor,res);
    error('CBF:ActiveQPFailed','CBF %s/%s QP failure: %s',st.mode,method,res.qp.reason);
end
% Apply sampled actuator constraints inside the QP; external clipping is
% forbidden. If extra input bounds are requested we solve a new QP.
if strcmp(st.mode,'physical')
    sampleOpts=opts;
    sampleOpts.preemptiveRows=[];
    if sigma>0 && opts.preemptivePush>0
        % Select every impending obstacle/shape by a row-local trend,
        % rather than by obstacle number, position or particular path.
        % ALL 9 original HOCBF inequalities remain hard.
        margins=[monitor.rows.normalizedMargin].';
        candidates=find((nav.predicted(:)<=opts.onMargin | margins<=opts.onMargin) & margins>=0);
        [~,ord]=sort(nav.predicted(candidates),'ascend');
        candidates=candidates(ord);
        sampleOpts.preemptiveRows=candidates(1:min(numel(candidates),opts.preemptiveMaxRows));
        if ~isempty(sampleOpts.preemptiveRows)
            worst=min(nav.predicted(sampleOpts.preemptiveRows));
            urgency=max(0,min(1,(opts.onMargin-worst)/max(abs(opts.onMargin),0.1)));
            sampleOpts.preemptivePush=opts.preemptivePush + ...
                (opts.navigationMaxPush-opts.preemptivePush)*urgency;
        end
    end
    if isfield(fopts,'smoothPreviousW')
        sampleOpts.smoothPreviousW=fopts.smoothPreviousW;
    end
    [res,success,why]=localPhysicalSampleBounds(res,st.aux,dt,lim,sampleOpts);
    if ~success,error('CBF:SampleBounds','Physical sample-bound QP failed: %s',why);end
    f=st.aux(1);eta=st.aux(2);nu=res.w(1);
    nextAux=[f+dt*eta+.5*dt^2*nu;eta+dt*nu];
    uSafe=[f;res.w(2:4)];
else
    nu=res.w(1);d=st.aux(:);
    nextAux=[d(1)+dt*d(2)+dt^2*d(3)/2+dt^3*d(4)/6+dt^4*nu/24;...
             d(2)+dt*d(3)+dt^2*d(4)/2+dt^3*nu/6;...
             d(3)+dt*d(4)+dt^2*nu/2;d(4)+dt*nu];
    uSafe=res.uSafe(:);
end
if any(~isfinite(uSafe)) || any(uSafe(1)<lim(1)-1e-7) || ...
        uSafe(1)>lim(2)+1e-7 || any(abs(uSafe(2:4))>lim(3:5)+1e-7)
    error('CBF:OutputOutOfBounds','Safety filter produced actuator-invalid output');
end
% Append compact diagnostics: do not store a growing trace inside the
% supervisor (which the flight logger may copy on every control step).
if opts.experimentalContinueAfterPsiViolation
    if isprop(obj,'cbfTime'),tt=obj.cbfTime;else,tt=NaN;end
    if ~st.experimentalCsvInitialized
        fid=fopen(opts.experimentalTraceFile,'w');
        if fid>=0
            fprintf(fid,['time_s,hmin_active,psi_min,psi_last_min,bad_chains,' ...
                'sigma_monitor,hard_qp_min_margin,force_N,forceDot_Nps,' ...
                'force_base_N,input_diff_inf\n']);
            fclose(fid);st.experimentalCsvInitialized=true;
        else
            warning('CBF:ExperimentalTraceWrite','Cannot open experimental trace file');
        end
    end
    if st.experimentalCsvInitialized
        fid=fopen(opts.experimentalTraceFile,'a');
        if fid>=0
            minQPmargin=min(res.map.bineq-res.map.Aineq*res.w(:));
            fprintf(fid,'%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n',...
                tt,expMinH,expPsiMin,expPsiLastMin,numel(expBadRows),...
                sigma,minQPmargin,uSafe(1),st.aux(2),uBase(1),norm(uSafe-uBase,inf));
            fclose(fid);
        end
    end
end
% Do not release until the BASELINE itself is acceptable again, and the
% command is close enough that release will not make a large jump.
match=(abs(uSafe(1)-uBase(1))<=opts.releaseForceTol) && ...
      all(abs(uSafe(2:4)-uBase(2:4))<=opts.releaseMomentTol);
if strcmp(st.mode,'virtual')
    auxNear=norm(nextAux,inf)<=opts.releaseAuxTol;
else
    auxNear=(abs(nextAux(1)-uBase(1))<=opts.releaseForceTol && ...
             abs(nextAux(2))<=opts.releaseAuxTol);
end
% In the experiment, do not release to baseline while ANY active-chain
% condition is still negative, even if the frozen-thrust proxy is positive.
chainReleaseOK=(~opts.experimentalContinueAfterPsiViolation) || isempty(expBadRows);
if sigma>=opts.offMargin && ~nav.risk && match && auxNear && chainReleaseOK
    st.releaseCount=st.releaseCount+1;
else
    st.releaseCount=0;
end
% Retain exactly one previously accepted sampled QP for chain-loss audit.
% At this instant res.rows refer to x(k), aux(k), while nextAux is aux(k+1).
% The QP margins are computed for the ACTUALLY SELECTED w, not wNom.
trace=struct();
if isprop(obj,'cbfTime'),trace.time=obj.cbfTime;else,trace.time=NaN;end
trace.x=x(:);trace.auxBefore=st.aux(:);trace.auxAfter=nextAux(:);
trace.uSafe=uSafe(:);trace.uBase=uBase(:);trace.w=res.w(:);
trace.rows=res.rows;
nCBF=numel(res.rows);
trace.hardMargins=res.map.bineq(1:nCBF)-res.map.Aineq(1:nCBF,:)*res.w(:);
st.lastActiveTrace=trace;
st.aux=nextAux;
if st.releaseCount>=opts.releaseHoldSteps
    % Release command change is explicitly bounded by match thresholds.
    % Hybrid switch is not a robust viability/closed-loop stability proof.
    st.active=false;st.aux=[];st.mode='';st.releaseCount=0;
    status='RELEASE';releasing=true;
    uSafe=uBase;
    if opts.emitEvents
        fprintf('[CBF-SUP] RELEASE to saturated HLC | sigma=%+.4g | hmin=%.4g\n',sigma,monitor.minH);
    end
elseif ~engaged && ~strcmp(status,'SWITCH_PHYSICAL_UNCERTIFIED')
    status='ACTIVE';
end
st.lastU=uSafe;
out=localSupervisorResult(true,uSafe,uBase,uRaw,monitor,st,status,reason,clock);
out.intervened=norm(uSafe-uBase,inf)>1e-9;
out.engaged=engaged;out.released=releasing;
out.navigation=nav;
out.activeQP=res.qp;out.w=res.w;out.activeMode=res.mode;
out.activeRows=res.rows;out.auxDot=res.auxDot;
if opts.experimentalContinueAfterPsiViolation
    out.experimental=struct('uncertified',true,'chainOK',isempty(expBadRows),...
        'badChains',numel(expBadRows),'minPsi',expPsiMin,...
        'minPsiLast',expPsiLastMin,'hMinActive',expMinH,...
        'badSamples',st.experimentalBadSamples,...
        'recoveryCount',st.experimentalRecoveries);
end
if isfield(res,'preemptive'),out.preemptive=res.preemptive;end
out.supervisor=st;
end

function [nav,st]=localCBFMarginTrend(monitor,st,opts,dt)
% Observer-only lead-time *heuristic*: NOT a forward reachable set,
% sampled-data certificate, or proof of obstacle clearance.
rows=monitor.rows;
n=numel(rows);margins=reshape([rows.normalizedMargin],[],1);
keys=cell(n,1);baseKeys=cell(n,1);
for i=1:n
    % Distinguish repeated sub-geometries (e.g. S2/K2 ball samples) as
    % well as arbitrary obstacle IDs, without any scene-specific indices.
    baseKeys{i}=sprintf('%.12g|%s',double(rows(i).obstacleID),char(rows(i).component));
    keys{i}=sprintf('%s#%d',baseKeys{i},sum(strcmp(baseKeys(1:i),baseKeys{i})));
end
closing=zeros(n,1);
if isfield(st,'navKeys') && isfield(st,'navMargins') && ...
        isfield(st,'navClosing') && iscell(st.navKeys)
    for i=1:n
        j=find(strcmp(keys{i},st.navKeys),1,'first');
        if ~isempty(j) && j<=numel(st.navMargins) && j<=numel(st.navClosing)
            observed=(st.navMargins(j)-margins(i))/dt;
            % SIGNED smoothing prevents one-sided amplification of noise.
            % Only the predicted trend and soft objective are affected;
            % this is never used as a replacement for a HARD HOCBF row.
            filtered=opts.navigationRateBlend*observed + ...
                (1-opts.navigationRateBlend)*max(0,st.navClosing(j));
            if filtered >= opts.navigationMinClosingRate
                closing(i)=filtered;
            end
        end
    end
end
predicted=margins-opts.navigationLeadSeconds*closing;
triggerTime=inf(n,1);
ind=closing>opts.navigationMinClosingRate;
triggerTime(ind)=max(0,margins(ind)-opts.onMargin)./closing(ind);
if any(margins<=opts.onMargin),triggerTime(margins<=opts.onMargin)=0;end
rawRisk=opts.navigationLeadSeconds>0 && any(triggerTime<=opts.navigationLeadSeconds);
if ~isfield(st,'navRiskCount'),st.navRiskCount=0;end
if rawRisk
    st.navRiskCount=min(opts.navigationConfirmSteps,st.navRiskCount+1);
else
    st.navRiskCount=0;
end
risk=rawRisk && st.navRiskCount>=opts.navigationConfirmSteps;
nav=struct('enabled',opts.navigationLeadSeconds>0,'risk',risk, ...
    'rawRisk',rawRisk,'confirmedSteps',st.navRiskCount, ...
    'current',margins,'closingRate',closing,'predicted',predicted, ...
    'minPredicted',min(predicted),'timeToTrigger',triggerTime, ...
    'minTimeToTrigger',min(triggerTime), ...
    'note','Instantaneous margin trend extrapolation only; uncertified');
st.navKeys=keys;st.navMargins=margins;st.navClosing=closing;
end

function [H,q]=localQPTrackingObjective(map,opts,sc)
% Smooth the PHYSICAL QP command with a previous feasible input as a
% secondary objective. No slack or change to HARD CBF/actuator constraints.
base=diag(1./sc(:).^2);
H=base;q=-base*map.wNom(:);
if isempty(map.torqueT) && isfield(opts,'smoothInputWeight') && ...
        opts.smoothInputWeight>0 && isfield(opts,'smoothPreviousW') && ...
        numel(opts.smoothPreviousW)==4 && all(isfinite(opts.smoothPreviousW(:)))
    smoother=opts.smoothInputWeight*base;
    H=H+smoother;
    q=q-smoother*opts.smoothPreviousW(:);
end
end

function monitor=localMonitorSaturatedPhysical(x,xd,F1,P,uBase,obsList,method,shapes,kappa)
clock=tic;rows=struct('obstacleID',{},'component',{},'h',{},'psiChain',{},...
  'initialChainSatisfied',{},'margin',{},'normalizedMargin',{},...
  'relativeDegree',{});
refZ=xd([3,7,11,15,19,23,27]);
aux=[uBase(1);0];w=[0;uBase(2:4)];
parts=localMethodSpecs(method);
for io=1:numel(obsList)
    if method(2)=='2'
        ss=localRequiredField(shapes,'spheres');
        assert(size(ss,2)==2 && size(ss,1)>=1,'spheres Nx2 required');
        names=repmat({'sphere'},1,size(ss,1));
        geoms=cell(1,size(ss,1));
        for j=1:size(ss,1),geoms{j}=ss(j,:).';end
    else
        names=parts;geoms=cell(size(parts));
        for j=1:numel(parts),geoms{j}=localRequiredField(shapes,parts{j});end
    end
    for j=1:numel(names)
        try
            ev=Generate_CBF_ActuatorAware_6Methods('evaluate','physical',method,names{j},...
                     x,aux,refZ,F1,P,obsList(io),geoms{j},kappa);
            if ~ev.info.valid,error('CBF:MonitorDerivativeInvalid', ...
                    'invalid physical monitor CBF: %s',ev.info.reason);end
        catch ME
            if isfield(obsList(io),'id'),oid=double(obsList(io).id);else,oid=io;end
            fprintf(2,'[CBF-MON-ROW] FAILED obs=%g component=%s: [%s] %s\n', ...
                oid,names{j},ME.identifier,ME.message);
            rethrow(ME);
        end
        % This is a pointwise frozen-thrust proxy. f_dot=f_ddot=0 NOW.
        margin=ev.braw+ev.Araw*w;
        scale=max(1,abs(ev.braw)+sum(abs(ev.Araw).*max(1,abs(w)).'));
        normalizedMargin=margin/scale;
        if isfield(obsList(io),'id'),oid=obsList(io).id;else,oid=io;end
        rows(end+1)=struct('obstacleID',oid,'component',names{j},...
            'h',ev.h,'psiChain',ev.info.psiChain(:),...
            'initialChainSatisfied',ev.info.initialChainSatisfied,...
            'margin',margin,'normalizedMargin',normalizedMargin,...
            'relativeDegree',ev.info.relativeDegreeAssumed); %#ok<AGROW>
    end
end
monitor=struct('rows',rows,'minH',min([rows.h]),...
    'minNormalizedMargin',min([rows.normalizedMargin]),...
    'violations',nnz([rows.normalizedMargin]<0),...
    'allInitialChainsSatisfied',all([rows.initialChainSatisfied]),...
    'evaluationSeconds',toc(clock),'model',...
    'physical f state frozen at saturated HLC command, fDot=0; instantaneous test only');
end

function localReportActiveQPFailure(obj,x,xd,vfNom,F1,P,uRaw,uBase,...
    obsList,method,shapes,kappa,opts,st,dt,monitor,res)
% Diagnostic only. Does not relax any constraints or change the applied input.
qpMap=res.map; tol=opts.qpFeasTol;
if isprop(obj,'cbfTime'),t=obj.cbfTime;else,t=NaN;end
fprintf(2,'\n[CBF-QP-FAIL] t=%.5f physical/%s | QP exitflag=%g | %s\n',...
    t,method,res.qp.exitflag,res.qp.reason);
fprintf(2,'  hMin=%+.7g, sigmaMonitor=%+.7g, chainOK=%d, f=%.8g, fDot=%.8g\n',...
    monitor.minH,monitor.minNormalizedMargin,monitor.allInitialChainsSatisfied,...
    st.aux(1),st.aux(2));
A=qpMap.Aineq;b=qpMap.bineq;wnom=qpMap.wNom(:);
nominalViolation=A*wnom-b;
[maxNominal,nominalRow]=max(nominalViolation);
nCBF=numel(res.rows);
fprintf(2,'  QP: %d constraints (%d CBF), 4 decision variables, max nominal residual=%+.8g at row %d\n',...
    size(A,1),nCBF,maxNominal,nominalRow);
[sortedResidual,idx]=sort(nominalViolation,'descend');
for i=1:min(8,numel(idx))
    j=idx(i);
    if j<=nCBF
        label=sprintf('CBF obs=%g %s',double(res.rows(j).obstacleID),res.rows(j).component);
    elseif j==nCBF+1
        label='thrust lower HOCBF';
    elseif j==nCBF+2
        label='thrust upper HOCBF';
    elseif j<=nCBF+8
        label=sprintf('torque bound index %d',j-(nCBF+2));
    elseif j==nCBF+9
        label='CBF yaw lock upper (HLC Mz)';
    elseif j==nCBF+10
        label='CBF yaw lock lower (HLC Mz)';
    else
        label=sprintf('auxiliary bound %d',j-(nCBF+10));
    end
    fprintf(2,'    row %2d %-26s A*wNom-b=%+.10g\n',j,label,sortedResidual(i));
end
% An independent LP minimizes a common constraint slack; this is just a
% diagnostic feasibility check, NOT an input and NOT a safety controller.
lp=struct('exitflag',NaN,'slack',NaN,'message','not attempted','w',nan(4,1));
try
    n=size(A,1);
    lpA=[A,-ones(n,1)]; lpB=b;
    cost=[zeros(4,1);1]; lb=[-inf(4,1);0];
    lo=optimoptions('linprog','Display','off');
    [wslack,~,ef,lpOut]=linprog(cost,lpA,lpB,[],[],lb,[],lo);
    lp.exitflag=ef;
    if ~isempty(wslack) && all(isfinite(wslack))
        lp.slack=wslack(5);lp.w=wslack(1:4);
    end
    if isstruct(lpOut) && isfield(lpOut,'message'),lp.message=lpOut.message;end
    fprintf(2,'  Independent LP: exitflag=%g, minimum common slack=%+.9g\n',ef,lp.slack);
    if ef>0 && lp.slack>max(1e-6,10*tol)
        fprintf(2,'  -> Physical QP inequalities are incompatible (within LP tolerances).\n');
    elseif ef>0
        fprintf(2,'  -> LP finds nearly feasible inequalities: investigate quadprog scaling/numerics.\n');
    else
        fprintf(2,'  -> LP inconclusive: inspect snapshot and numerical conditioning.\n');
    end
catch err
    lp.message=err.message;
    fprintf(2,'  Independent LP unavailable/inconclusive: %s\n',err.message);
end
D=struct('time',t,'x',x,'xd',xd,'vfNom',vfNom,'F1',F1,'P',P,...
    'uRaw',uRaw,'uBase',uBase,'obsList',obsList,'method',method,...
    'shapes',shapes,'kappa',kappa,'opts',opts,'supervisor',st,'dt',dt,...
    'monitor',monitor,'filterResult',res,'Aineq',A,'bineq',b,...
    'wNom',wnom,'nominalResidual',nominalViolation,'independentLP',lp);
try
    save('CBF_physical_QP_failure.mat','D');
    fprintf(2,'  Saved CBF_physical_QP_failure.mat in current MATLAB directory.\n');
catch err
    fprintf(2,'  Snapshot save failed: %s\n',err.message);
end
end

function [res,ok,reason]=localPhysicalSampleBounds(res,aux,dt,lim,opts)
ok=false;reason='';f=aux(1);eta=aux(2);
map=res.map;
% Quadratic Bezier control-point condition guarantees f(tau) stays inside
% [fmin,fmax] for tau in [0,dt] when fDDot is held constant.
if f < lim(1)-1e-8 || f>lim(2)+1e-8 || ...
   f+dt*eta/2 < lim(1)-1e-8 || f+dt*eta/2>lim(2)+1e-8
    reason='sampled thrust Bezier initial/middle control point is out of limits';return;
end
row=[.5*dt^2 0 0 0];
val=f+dt*eta;
A=[map.Aineq;row;-row];
b=[map.bineq;lim(2)-val;val-lim(1)];
if isfinite(opts.rampLimitFddot)
    A=[A;1 0 0 0;-1 0 0 0];
    b=[b;opts.rampLimitFddot;opts.rampLimitFddot];
end
if isfinite(opts.rampLimitFdot)
    A=[A;dt 0 0 0;-dt 0 0 0];
    b=[b;opts.rampLimitFdot-eta;opts.rampLimitFdot+eta];
end
if isfinite(opts.rampLimitMoment)
    for j=1:3
        ej=zeros(1,4);ej(j+1)=1;
        A=[A;ej;-ej]; %#ok<AGROW>
        b=[b;res.map.nominalU(j+1)+dt*opts.rampLimitMoment;...
           -res.map.nominalU(j+1)+dt*opts.rampLimitMoment]; %#ok<AGROW>
    end
end
map.Aineq=A;map.bineq=b;
% IMPORTANT: preemptive optimization adds only SOFT positive-margin targets.
% The original 9 hard CBF inequalities, thrust and torque bounds, and
% sampled thrust bounds remain HARD with no safety slack.
[sol,anticipation]=localQPWithSoftAnticipation(map,opts,numel(res.rows));
res.preemptive=anticipation;
if ~sol.success,reason=sol.reason;return;end
w=sol.w;
% Inspect the tightened constraints explicitly.
if any(A*w>b+opts.qpFeasTol)
    reason='sampled actuator constraints not met';return;
end
res.w=w;res.qp=sol;res.uSafe=[f;w(2:4)];res.map=map;
ok=true;reason='sampled actuator constraints solved';
end

function [sol,anticipationDiag]=localQPWithSoftAnticipation(map,opts,nCBF)
% Optional experimental anticipation, NOT a viability / collision
% certificate. At a positive monitoring margin it encourages increasing
% the critical HARD-CBF margins BEFORE they cross zero.
%
% Hard inequalities: A*w<=b (unchanged; 4 physical decision variables).
% Soft targets:      A_i*w - s_i <= b_i - target_i; s_i>=0.
% Objective: ||w-wNom||_H^2/2 + rho*||s||^2/2.
% Because slacks are ONLY in the extra targets, they CANNOT relax CBF,
% thrust, torque or sampled actuator bounds.
anticipationDiag=struct('enabled',false,'rowIndices',[],'targetMargin',[],...
    'achievedMargin',[],'slack',[],'maxSlack',0,'push',0);
if ~isfield(opts,'preemptiveRows') || isempty(opts.preemptiveRows) || ...
        ~isfield(opts,'preemptivePush') || opts.preemptivePush<=0
    sol=localQP(map,opts);return;
end
idx=opts.preemptiveRows(:);
assert(all(idx>=1 & idx<=nCBF & idx==round(idx)), ...
    'preemptiveRows must be CBF indices');
idx=unique(idx,'stable');
assert(numel(idx)>=1);
if ~isfield(opts,'physicalScales'),opts.physicalScales=[10;1;1;1];end
if ~isfield(opts,'qpFeasTol'),opts.qpFeasTol=1e-7;end
scale=opts.physicalScales(:);
assert(numel(scale)==4 && all(scale>0));
A=map.Aineq;b=map.bineq(:);wn=map.wNom(:);
if ~map.ok
    sol=localQP(map,opts);return;
end
G=A(idx,:);
nominalMargin=b(idx)-G*wn;
target=max(0,nominalMargin)+opts.preemptivePush;
k=numel(idx);
[H4,q4]=localQPTrackingObjective(map,opts,scale);
Haug=blkdiag(H4,opts.preemptiveWeight*eye(k));
qaug=[q4;zeros(k,1)];
Aaug=[A zeros(size(A,1),k);G -eye(k)];
baug=[b;b(idx)-target];
lb=[-inf(4,1);zeros(k,1)];
opt=optimoptions('quadprog','Display','off');
[z,fv,ef]=quadprog(Haug,qaug,Aaug,baug,[],[],lb,[],[],opt);
sol=struct('success',false,'reason','anticipatory QP unsuccessful',...
    'w',nan(4,1),'exitflag',ef,'fval',fv);
if ef>0 && ~isempty(z) && all(isfinite(z)) && ...
        all(Aaug*z<=baug+opts.qpFeasTol)
    sol.success=true;
    sol.w=z(1:4);
    sol.reason='hard CBF feasible, soft-anticipation experiment solved';
    anticipationDiag.enabled=true;
    anticipationDiag.rowIndices=idx;
    anticipationDiag.targetMargin=target;
    anticipationDiag.achievedMargin=b(idx)-G*sol.w;
    anticipationDiag.slack=z(5:end);
    anticipationDiag.maxSlack=max(anticipationDiag.slack);
    anticipationDiag.push=opts.preemptivePush;
else
    % Do not let experimental anticipation make an otherwise feasible
    % HARD-constrained QP fail. Use the ordinary hard-constrained QP.
    % This is NOT permission to fall back after a hard-QP failure.
    sol=localQP(map,opts);
    anticipationDiag.enabled=false;
end
end

function out=localSupervisorResult(success,uSafe,uBase,uRaw,monitor,st,status,reason,clock)
out=struct('success',success,'status',status,'reason',reason,...
    'uSafe',uSafe,'rawHLC',uRaw,'nominalU',uBase,'uBase',uBase,...
    'monitor',monitor,'rows',monitor.rows,'supervisor',st,...
    'intervened',false,'engaged',false,'released',false,...
    'activeMode',st.mode,'w',[],'qp',struct('reason',status),...
    'totalSeconds',toc(clock));
end

function localReportActiveChainTransition(obj,st,x,xd,uBase,monitor,opts,ME)
% A runtime chain loss is NOT an initial-condition failure at engagement.
% Save the previous successful sample alongside current failure data.
if isprop(obj,'cbfTime'),t=obj.cbfTime;else,t=NaN;end
if isfield(opts,'diagnosticFile') && ~isempty(opts.diagnosticFile)
    sourceFile=opts.diagnosticFile;
else
    sourceFile='CBF_initial_chain_failure.mat';
end
current=[];
if exist(sourceFile,'file')==2
    src=load(sourceFile,'CBF_INIT');
    if isfield(src,'CBF_INIT'),current=src.CBF_INIT;end
end
prev=[];
if isfield(st,'lastActiveTrace') && ~isempty(st.lastActiveTrace)
    prev=st.lastActiveTrace;
end
fprintf(2,'\n[CBF-CHAIN-LOSS] t=%.5f | active physical filter, NOT engagement initial failure\n',t);
fprintf(2,'  current auxiliary f=%.8g N, fDot=%.8g N/s; base f=%.8g N\n',...
    st.aux(1),st.aux(2),uBase(1));
if ~isempty(current)
    fprintf(2,'  current obs=%g component=%s psi=%s\n',...
        double(current.obstacleID),current.component,mat2str(current.psiChain(:).',7));
end
if ~isempty(prev)
    fprintf(2,'  previous accepted QP t=%.5f: f=%.8g fDot=%.8g, next f=%.8g fDot=%.8g\n',...
        prev.time,prev.auxBefore(1),prev.auxBefore(2),prev.auxAfter(1),prev.auxAfter(2));
    fprintf(2,'  previous applied input %s; min hard QP CBF margin=%+.8g\n',...
        mat2str(prev.uSafe.',7),min(prev.hardMargins));
    if ~isempty(current)
        idx=find(arrayfun(@(r)isequal(double(r.obstacleID),double(current.obstacleID)) && ...
            strcmp(r.component,current.component),prev.rows),1,'first');
        if ~isempty(idx)
            fprintf(2,'  SAME row previous psi=%s; prior selected-QP margin=%+.8g\n',...
                mat2str(prev.rows(idx).psiChain(:).',7),prev.hardMargins(idx));
        else
            fprintf(2,'  Failed row was not present in prior trace.\n');
        end
    end
else
    fprintf(2,'  No previously accepted QP trace available.\n');
end
fprintf(2,'  Monitor sigma=%.6g is a frozen-thrust PROXY; it is not the active chain.\n',...
    monitor.minNormalizedMargin);
CBF_ACTIVE_CHAIN_LOSS=struct('time',t,'previous',prev,'current',current,...
    'x',x,'xd',xd,'aux',st.aux,'uBase',uBase,'monitor',monitor,...
    'errorIdentifier',ME.identifier,'errorMessage',ME.message);
try
    save('CBF_active_chain_loss.mat','CBF_ACTIVE_CHAIN_LOSS');
    fprintf(2,'  Saved CBF_active_chain_loss.mat (current + preceding accepted QP).\n');
catch saveError
    fprintf(2,'  Could not save active-chain diagnostic: %s\n',saveError.message);
end
end


function report=localWarmupPhysicalQP(dt)
% TEST/STARTUP ACTION ONLY: exercise both localPhysicalSampleBounds and
% localQPWithSoftAnticipation before the first flight-phase ACTIVE call.
% NO actual CBF state, input, plant, log or supervisor is touched.
if nargin<1 || isempty(dt),dt=0.025;end
assert(isfinite(dt) && dt>0 && isscalar(dt),'dt must be positive');
nCBF=9;
% Simple feasible problem in w=[fDDot;Mx;My;Mz] with 9 dummy CBF rows.
A=[zeros(nCBF,4);eye(4);-eye(4)];
b=[ones(nCBF,1);5*ones(4,1);5*ones(4,1)];
map=struct('ok',true,'Aineq',A,'bineq',b,'wNom',zeros(4,1), ...
    'nominalU',[8;0;0;0],'torqueT',[]);
base=struct('map',map,'rows',repmat(struct('h',1),1,nCBF));
opts=struct('qpFeasTol',1e-7,'rampLimitFddot',Inf, ...
    'rampLimitFdot',Inf,'rampLimitMoment',Inf, ...
    'physicalScales',[10;1;1;1], ...
    'preemptivePush',0.03,'preemptiveWeight',100, ...
    'preemptiveRows',[]);
T=tic;
[r0,ok0,why0]=localPhysicalSampleBounds(base,[8;0],dt,[0;20;1;1;1],opts);
if ~ok0,error('CBF:QPPrewarm','Standard sampled QP prewarm failed: %s',why0);end
opts.preemptiveRows=[1;4;5];
[r1,ok1,why1]=localPhysicalSampleBounds(base,[8;0],dt,[0;20;1;1;1],opts);
if ~ok1 || ~r1.preemptive.enabled
    error('CBF:QPPrewarm','Soft anticipatory sampled QP prewarm failed: %s',why1);
end
% Also compile/exercise the optional previous-input smooth QP branch.
opts.smoothInputWeight=0.2;
opts.smoothPreviousW=[0;0.1;-0.1;0];
[r2,ok2,why2]=localPhysicalSampleBounds(base,[8;0],dt,[0;20;1;1;1],opts);
if ~ok2,error('CBF:QPPrewarm','Smoothed anticipatory QP prewarm failed: %s',why2);end
report=struct('success',ok0 && ok1 && ok2,'seconds',toc(T), ...
    'standardExitflag',r0.qp.exitflag,'softExitflag',r1.qp.exitflag, ...
    'softDimension',4+numel(opts.preemptiveRows), ...
    'warning','Synthetic QP warmup is not a feasibility or safety proof');
end
