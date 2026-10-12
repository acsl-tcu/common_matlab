function out = Generate_CBF_DualInput_6Methods(action,varargin)
% Generate_CBF_DualInput_6Methods.m
% ONE managed MATLAB source for TWO control architectures x SIX geometries.
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
%  report=Generate_CBF_DualInput_6Methods();     % both modes x all methods
%  cfg=struct('modes',{{'virtual','physical'}},'methods',{{'S1','S2'}});
%  report=Generate_CBF_DualInput_6Methods(cfg);
%  s=Generate_CBF_DualInput_6Methods('evaluate',mode,method,component,...
%                                   x,aux,refZ,F1,P,obs,geom,kappa);
%  result=Generate_CBF_DualInput_6Methods('filter',mode,obj,x,xd,...
%                 vfNom,F1,P,aux,obsList,method,shapes,kappa,opts);
%  report=Generate_CBF_DualInput_6Methods('selftest',cfg);
%
% OUTPUT: generated_cbf_dual_input/virtual/S1/CBF_V_S1_single.m etc
%         generated_cbf_dual_input/physical/S1/CBF_P_S1_single.m etc
% Generated wrappers call THIS master file; keep it on MATLAB path.
%
% HLC INTEGRATION: after tmp=[uf(1);us] is computed, before saturation:
%   res=Generate_CBF_DualInput_6Methods('filter',mode,obj,x,xd',vf,...
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
     s=Generate_CBF_DualInput_6Methods('evaluate',mode,method,name,...
           x,aux,refZ,F1,P,obs,geom,kappa);
     if ~s.info.valid,error('Smoke test failed: %s',s.info.reason);end
     maxLow=s.info.maxLowerInputDependence;
     maxAffine=s.info.maxAffineResidual;normA=norm(s.Araw);
     if strcmp(name,'sphere')
      for la=[0,1]
       geomE=geom;geomE(1)=la;
       ss=Generate_CBF_DualInput_6Methods('evaluate',mode,method,name,...
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
fprintf(fid,'%% Autogenerated wrapper; keep Generate_CBF_DualInput_6Methods.m on path.\n');
fprintf(fid,'s=Generate_CBF_DualInput_6Methods(''evaluate'',''%s'',''%s'',''%s'',x,aux,refZ,F1,P,obs,geom,kappa);\n',mode,method,name);
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
clock=tic; A=zeros(0,4);b=zeros(0,1);
rows=struct('obstacleID',{},'component',{},'h',{},'psi',{},...
 'relativeDegree',{},'evaluationSeconds',{},'geomGap',{});
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
        res=Generate_CBF_DualInput_6Methods('evaluate',mode,method,name,...
            x,aux,refZ,F1,P,obsList(io),geom,kappa);
        if ~res.info.valid
            error('CBF %s/%s/%s invalid derivative coefficient: %s',mode,method,name,res.info.reason);
        end
        if opts.requireInitialChain && ~res.info.initialChainSatisfied
            error('CBF %s/%s/%s initial HOCBF chain violated. No certificate.',mode,method,name);
        end
        sc=max([1,abs(res.braw),norm(res.Araw,inf)]);
        A(end+1,:)=res.Araw/sc; %#ok<AGROW>
        b(end+1,1)=res.braw/sc; %#ok<AGROW>
        if isfield(obsList(io),'id'),id=obsList(io).id;else,id=io;end
        rows(end+1)=struct('obstacleID',id,'component',name,...
           'h',res.h,'psi',res.psi,'relativeDegree',res.info.relativeDegreeAssumed,...
           'evaluationSeconds',res.info.evaluationSeconds,'geomGap',res.info.geometricGap); %#ok<AGROW>
    end
end
if strcmp(mode,'virtual')
    map=localVirtualMap(obj,x,xd,vfNom,F1,P,aux,A,b,opts);
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
map.reason='current thrust out of bounds; nu cannot change f instantly';
if ~all(isfinite([map.Aineq(:);map.bineq(:);map.wNom(:);map.nominalU(:)])),error('Nonfinite virtual QP map');end
end

function map=localPhysicalMap(obj,x,xd,vfNom,P,aux,Araw,braw,opts)
if ~isfield(opts,'uLimits'),opts.uLimits=[0;20;1;1;1];end
if ~isfield(opts,'physicalThrustGains'),opts.physicalThrustGains=[4;4];end
if ~isfield(opts,'physicalOmega'),opts.physicalOmega=3;end
lim=opts.uLimits(:);thrustGains=opts.physicalThrustGains(:);
assert(numel(thrustGains)==2 && all(thrustGains>0));
xd=reshape(xd,1,[]);P=reshape(P,1,[]);
f=aux(1);eta=aux(2);
uf=obj.Uf_SuspendedLoadxyDst(x,xd,vfNom,P);
B=obj.Beta2_SuspendedLoadxyDst(x,xd,vfNom,P);
vs=obj.Vs_SuspendedLoadxyDst(x,xd,vfNom,P,obj.param.F2,obj.param.F3,obj.param.F4);
if any(~isfinite(B(:))) || rcond(B)<1e-9,error('HLC nominal beta2 singular');end
Mnom=B\obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vfNom,vs(:),P);
map.nominalU=[uf(1);Mnom(:)];
% Delta physical input w=[fDDot;M]. The current thrust f is NOT a QP variable.
nuFnom=opts.physicalOmega^2*(uf(1)-f)-2*opts.physicalOmega*eta;
map.wNom=[nuFnom;Mnom(:)];
% Lower thrust HOCBF: nuF + (g1+g2)*eta + g1*g2*(f-fMin)>=0
% Upper thrust HOCBF: -nuF -(g1+g2)*eta+g1*g2*(fMax-f)>=0
% Matrix inequalities Aineq*w<=bineq.
map.Aineq=[-Araw; -1 0 0 0; 1 0 0 0;...
          zeros(3,1),eye(3); zeros(3,1),-eye(3)];
map.bineq=[braw;sum(thrustGains)*eta+prod(thrustGains)*(f-lim(1));...
                -sum(thrustGains)*eta+prod(thrustGains)*(lim(2)-f);lim(3:5);lim(3:5)];
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
H=diag(1./sc.^2);q=-H*map.wNom;
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
