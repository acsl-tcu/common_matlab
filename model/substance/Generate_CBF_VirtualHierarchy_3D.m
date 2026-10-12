function out = Generate_CBF_VirtualHierarchy_3D(action,varargin)
% Generate_CBF_VirtualHierarchy_3D
% One managed MATLAB source. No Symbolic Toolbox, no expanded Lie derivatives.
% Numeric truncated Taylor jets for h^(0)...h^(r), r=4 or 6.
%
% Default: generation + smoke tests of seven geometric primitives.
%   report = Generate_CBF_VirtualHierarchy_3D();
%   report = Generate_CBF_VirtualHierarchy_3D(cfg);
%   s = Generate_CBF_VirtualHierarchy_3D( ...
%      'evaluate','load',x,delta,refZ,F1,P,obsVec,geom,kappa);
%   [h,psi,Araw,braw,info]=deal(s.h,s.psi,s.Araw,s.braw,s.info);
%   map = Generate_CBF_VirtualHierarchy_3D( ...
%      'map',obj,x,xd,vfNom,F1,P,delta,Araw,braw);
%   sol = Generate_CBF_VirtualHierarchy_3D('solve',map,opts);
%   result = Generate_CBF_VirtualHierarchy_3D( ...
%      'filter',obj,x,xd,vfNom,F1,P,delta,Araw,braw,opts);
%   obsVec = Generate_CBF_VirtualHierarchy_3D('obstacle',obsItem);
%
% Araw * [nu; M1; M2; M3] + braw >= 0, w = [nu;v2;v3;v4]
% Only ONE simultaneous 4-input QP. No z-first or xy-first heuristic.
% delta=[delta0;delta1;delta2;delta3], delta3dot=nu.
% The first-layer thrust f is computed from v1 = -F1*[z-zd;dz-dzd]+delta0.
% Araw is evaluated in this exact extended physical system, before HLC
% torque conversion; conversion reuses the supplied Beta2 and V2_alpha2.
%
% GEOMETRY: single, sphere (0<lambda<1), sphere_load (lambda=0),
% sphere_uav (lambda=1), uav, cable, load. Fixed obstacle only.
% Ellipsoid barrier h = d' * ((1+1/betaM)*SA + (1+betaM)*SO)\d - 1.
% Obstacle: [cx cy cz S11 S22 S33 S12 S13 S23].
% No symbolic theorem of relative degree is claimed; expected r=4 except
% sphere_load/load r=6 is independently audited NUMERICALLY at run time.
% CBF lower-order initial conditions, input bounds, model uncertainties,
% sampled-data guarantees, and recursive QP feasibility remain necessary.
%
% Designed for MATLAB R2024b. MATLAB was not available in the authoring
% environment: RUN 'selftest' before using on a physical plant.

if nargin==0
    action='generate';
elseif isstruct(action)
    varargin=[{action},varargin]; action='generate';
end
switch lower(char(action))
    case 'generate'
        if isempty(varargin),cfg=struct();else,cfg=varargin{1};end
        out=localGenerate(cfg);
    case 'selftest'
        out=localSelfTest();
    case 'evaluate'
        out=cell(1,5);
        [out{:}]=localEvaluate(varargin{:});
        out=struct('h',out{1},'psi',out{2},'Araw',out{3},'braw',out{4},'info',out{5});
    case 'obstacle'
        out=localObstacle(varargin{:});
    case 'jet'
        out=localDeltaJet(varargin{:});
    case 'map'
        out=localQPMap(varargin{:});
    case 'solve'
        out=localSolve(varargin{:});
    case 'filter'
        if numel(varargin)<9,error('filter needs obj,x,xd,vfNom,F1,P,delta,Araw,braw');end
        if numel(varargin)<10,opts=struct();else,opts=varargin{10};end
        mp=localQPMap(varargin{1:9},opts);
        sol=localSolve(mp,opts);
        out=struct('success',false,'sol',sol,'map',mp,'uSafe',[],...
                   'vfSafe',[],'deltaDot',[]);
        if sol.success
            nu=sol.w(1);
            vfSafe=localDeltaJet(varargin{4},varargin{5},varargin{7},nu);
            out.success=true;
            out.vfSafe=vfSafe;
            out.uSafe=[mp.thrust;mp.torqueT*sol.w+mp.torqueOffset];
            aux=varargin{7};aux=aux(:);
            out.deltaDot=[aux(2:4);nu];
        end
    otherwise
        error('Unknown action: %s',char(action));
end
end

%% GENERATION: write small generated wrappers, not huge symbolic expressions.
function report=localGenerate(cfg)
if ~isfield(cfg,'models') || isempty(cfg.models)
    cfg.models={'single','sphere','sphere_load','sphere_uav','uav','cable','load'};
end
if ischar(cfg.models),cfg.models={cfg.models};end
if isstring(cfg.models),cfg.models=cellstr(cfg.models);end
if ~isfield(cfg,'outputDir'),cfg.outputDir='generated_cbf_virtual_3d_jet';end
if ~isfield(cfg,'runSelfTest'),cfg.runSelfTest=true;end
if ~isfield(cfg,'maxGenerationSeconds'),cfg.maxGenerationSeconds=120;end
outDir=fullfile(fileparts(mfilename('fullpath')),cfg.outputDir);
if ~exist(outDir,'dir'),mkdir(outDir);end
names={'single','sphere','sphere_load','sphere_uav','uav','cable','load'};
t0=tic;component=strings(numel(names),1);status=strings(numel(names),1);
seconds=nan(numel(names),1);rd=nan(numel(names),1);
maxResidual=nan(numel(names),1);nuCoefficient=nan(numel(names),1);momentCoefficientNorm=nan(numel(names),1);
fprintf('[JET-CBF] Generating lightweight numeric Taylor-jet wrappers; NO symbolic expansion.\n');
fprintf('[JET-CBF] Output: %s\n',outDir);
for k=1:numel(names)
    name=names{k};component(k)=string(name);
    if ~ismember(name,cfg.models),status(k)="skipped";continue;end
    if toc(t0)>cfg.maxGenerationSeconds
        status(k:end)="not_run: elapsed budget";break;
    end
    ti=tic;
    if strcmp(name,'load') || strcmp(name,'sphere_load'),r=6;else,r=4;end
    rd(k)=r;
    fprintf('[JET-CBF][+%.2fs][%s] START r=%d\n',toc(t0),name,r);
    try
        file=fullfile(outDir,['CBF_V_' name '.m']);
        fid=fopen(file,'w');
        if fid<0,error('Cannot write generated wrapper: %s',file);end
        clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
        fprintf(fid,'function [h,psi,Araw,braw,info] = CBF_V_%s(x,delta,refZ,F1,P,obsVec,geom,kappa)\n',name);
        fprintf(fid,'%% Generated numeric Taylor-jet wrapper; requires Generate_CBF_VirtualHierarchy_3D.m on path.\n');
        fprintf(fid,'s = Generate_CBF_VirtualHierarchy_3D(''evaluate'',''%s'',x,delta,refZ,F1,P,obsVec,geom,kappa);\n',name);
        fprintf(fid,'h=s.h; psi=s.psi; Araw=s.Araw; braw=s.braw; info=s.info;\n');
        fprintf(fid,'end\n');
        clear clean
        if cfg.runSelfTest
            [args,~]=localExample(name);
            s=Generate_CBF_VirtualHierarchy_3D('evaluate',name,args{:});
            maxResidual(k)=s.info.maxLowerInputDependence;
            nuCoefficient(k)=s.Araw(1);
            momentCoefficientNorm(k)=norm(s.Araw(2:4));
            if ~s.info.valid
                status(k)="audit_failed: "+string(s.info.reason);
                if exist(file,'file'),delete(file);end
                fprintf('[JET-CBF][%s] AUDIT FAILED: %s\n',name,s.info.reason);
            else
                status(k)="generated+sample_audited";
            end
        else
            status(k)="generated_not_audited";
        end
    catch ME
        status(k)="error: "+string(ME.message);
        fprintf(2,'[JET-CBF][%s] ERROR: %s\n',name,ME.message);
    end
    seconds(k)=toc(ti);
    fprintf('[JET-CBF][+%.2fs][%s] END %.3fs : %s\n',toc(t0),name,seconds(k),char(status(k)));
end
report=table(component,status,rd,seconds,maxResidual,nuCoefficient,momentCoefficientNorm);
writetable(report,fullfile(outDir,'CBF_JET_report.csv'));
disp(report);
fprintf('[JET-CBF] DONE +%.3fs. No symbolic size guard needed.\n',toc(t0));
end

%% NUMERIC HOCBF with truncated Taylor algebra
function [h,psi,Araw,braw,info]=localEvaluate(name,x,delta,refZ,F1,P,obsVec,geom,kappa)
timeStart=tic;
name=char(name);
if any(strcmp(name,{'load','sphere_load'})),r=6;
elseif any(strcmp(name,{'single','sphere','sphere_uav','uav','cable'})),r=4;
else,error('Unknown shape: %s',name);end
x=x(:);delta=delta(:);refZ=refZ(:);F1=F1(:);P=P(:);obsVec=obsVec(:);geom=geom(:);kappa=kappa(:);
assert(numel(x)==19 && numel(delta)==4 && numel(refZ)==7 && numel(F1)==2 && numel(P)==9);
assert(numel(obsVec)==9 && numel(kappa)>=r && all(isfinite(kappa)) && all(kappa(1:r)>0));
assert(all(isfinite([x;delta;refZ;F1;P;obsVec;geom])));
assert(P(1)>0 && P(2)>0 && P(3)>0 && P(4)>0 && P(6)>=0 && P(7)>0);
assert(geom(end)>0 && all(geom(1:end-1)>0));
X0=[x;delta];
der0=localHDerivatives(name,X0,refZ,F1,P,obsVec,geom,r,zeros(4,1));
chain=zeros(r,1);
cp=1; chain(1)=der0(1);
for k=1:r-1
    cp=conv(cp,[kappa(k),1]);
    chain(k+1)=cp*der0(1:k+1).';
end
psi=chain(end);
c=conv(cp,[kappa(r),1]);
braw=c*der0.';
h=der0(1);
Araw=zeros(1,4); maxLow=0; maxCurv=0;
for j=1:4
    uj=zeros(4,1);uj(j)=1;
    plus=localHDerivatives(name,X0,refZ,F1,P,obsVec,geom,r,uj);
    minus=localHDerivatives(name,X0,refZ,F1,P,obsVec,geom,r,-uj);
    Araw(j)=0.5*(c*(plus-minus).');
    lowDifference=max(abs([plus(1:r)-der0(1:r); minus(1:r)-der0(1:r)]),[],'all');
    maxLow=max(maxLow,lowDifference);
    maxCurv=max(maxCurv,abs(c*(plus+minus-2*der0).'));
end
%% Also test combined inputs to expose cross-input nonlinearities.
maxMixed=0;
for wTest=[ones(4,1),[.37;-.22;.19;-.13]]
    combined=localHDerivatives(name,X0,refZ,F1,P,obsVec,geom,r,wTest);
    maxLow=max(maxLow,max(abs(combined(1:r)-der0(1:r))));
    maxMixed=max(maxMixed,abs(c*combined.'-braw-Araw*wTest));
end
% Test is local numeric, NOT a symbolic certificate of uniform RD.
scale=max(1,norm(der0,inf));
tolLow=1e-5*scale;tolCurv=1e-5*max([1,abs(braw),norm(Araw,inf)]);
valid=all(isfinite([h;psi;braw;Araw(:)])) && maxLow<=tolLow && maxCurv<=tolCurv && maxMixed<=tolCurv;
if ~valid,reason='lower-order input dependence / non-affine output / numerical singularity';
else,reason='local Taylor-jet input-affinity audit passed';end
info=struct('valid',valid,'reason',reason,'relativeDegreeAssumed',r,...
            'maxLowerInputDependence',maxLow,'maxAffineCurvature',maxCurv,...
            'maxMixedInputResidual',maxMixed,'psiChain',chain,...
            'evaluationSeconds',toc(timeStart),...
            'lowerTolerance',tolLow,'affineTolerance',tolCurv,...
            'hDerivativesAtZeroInput',der0(:),...
            'nuCoefficient',Araw(1),'momentCoefficients',Araw(2:4));
end

function hd=localHDerivatives(name,X0,refZ,F1,P,obsVec,geom,r,u)
% x(t) represented as ordinary Taylor coefficients x^(k)/k!
% Solve x' = f(x,u,t) coefficient-wise. No time stepping or finite dt.
N=r+1;Xj=zeros(23,N);Xj(:,1)=X0(:);
for k=0:r-2
    F=localJetRHS(Xj,refZ,F1,P,u);
    Xj(:,k+2)=F(:,k+1)/(k+1);
end
% The last coefficient of h uses only the geometric centers pL and pT;
% their derivatives depend on vL and wL at order r-1. Avoid requiring
% reference derivative zd^(r+1) for unrelated dynamics.
k=r-1;
Xj(8:10,r+1)=Xj(11:13,r)/r;
ptDot=localJCross(Xj(17:19,:),Xj(14:16,:));
Xj(14:16,r+1)=ptDot(:,r)/r;
hJet=localGeometryJet(name,Xj,obsVec,geom,P(7));
hd=hJet(:).';
for k=0:r,hd(k+1)=hd(k+1)*factorial(k);end
end

function fjet=localJetRHS(X,ref,F1,P,u)
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
v1=-F1(1)*(X(10,:)-zd)-F1(2)*(X(13,:)-dz)+X(20,:);
thrust=localJDiv(ddz+v1-base,beta);
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
fjet=zeros(23,N);
fjet(1:4,:)=qDot;
fjet(5:7,:)=wbdot;
fjet(8:10,:)=vl;
fjet(11:13,:)=vlDot;
fjet(14:16,:)=ptDot;
fjet(17:19,:)=wLdot;
fjet(20:22,:)=X(21:23,:);
fjet(23,1)=u(1);
end

function hj=localGeometryJet(name,X,obs,geom,L)
N=size(X,2);pl=X(8:10,:);pt=X(14:16,:);
S=zeros(3,3,N);
switch name
    case 'single'
        lambda=.5; a=geom(1); b=geom(2);
        S=localAxisShapeJet(pt,a,b);
    case 'sphere'
        lambda=geom(1);S(:,:,1)=geom(2)^2*eye(3);
        if lambda<=0 || lambda>=1,error('Generic sphere requires 0<lambda<1');end
    case 'sphere_load'
        lambda=0;S(:,:,1)=geom(1)^2*eye(3);
    case 'sphere_uav'
        lambda=1;S(:,:,1)=geom(1)^2*eye(3);
    case 'uav'
        lambda=1;S(:,:,1)=diag(geom(1:3).^2);
    case 'cable'
        lambda=.5;S=localAxisShapeJet(pt,geom(1),geom(2));
    case 'load'
        lambda=0;S(:,:,1)=diag(geom(1:3).^2);
    otherwise
        error('Unsupported model: %s',name);
end
center=pl-lambda*L*pt;
d=center;d(:,1)=d(:,1)-obs(1:3);
SO=[obs(4) obs(7) obs(8);obs(7) obs(5) obs(9);obs(8) obs(9) obs(6)];
if min(eig(SO))<=0,error('Obstacle matrix must be positive definite.');end
betaM=geom(end);
Ssum=(1+1/betaM)*S;
Ssum(:,:,1)=Ssum(:,:,1)+(1+betaM)*SO;
S0=Ssum(:,:,1);
if rcond(S0)<1e-12,error('Collision shape sum nearly singular.');end
% Solve S(t) y(t)=d(t) by order-wise coefficient matching.
y=zeros(3,N);
for k=1:N
    rhs=d(:,k);
    for j=2:k
        rhs=rhs-Ssum(:,:,j)*y(:,k-j+1);
    end
    y(:,k)=S0\rhs;
end
hj=-localJConst(1,N);
for i=1:3,hj=hj+localJMul(d(i,:),y(i,:));end
end
function S=localAxisShapeJet(pt,a,b)
N=size(pt,2);S=zeros(3,3,N);
S(:,:,1)=b^2*eye(3);
for i=1:3
    for j=1:3
        S(i,j,:)=reshape(squeeze(S(i,j,:)).'+(a^2-b^2)*localJMul(pt(i,:),pt(j,:)),1,1,N);
    end
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

%% OLD HLC MAPPING AND QP (all kept in the SAME managed source)
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
function obsVec=localObstacle(item,betaMargin)
if nargin<2 || isempty(betaMargin),betaMargin=1;end
c=item.p_center(:);rad=item.ellipsoid_radii(:);R=item.R_obs;
assert(all(rad>0) && numel(rad)==3 && isequal(size(R),[3 3]) && betaMargin>0);
assert(norm(R'*R-eye(3),'fro')<1e-6 && det(R)>0);
S=R*diag(rad.^2)*R.';
if isfield(item,'d_margin') && item.d_margin>0
    m=item.d_margin;
    S=(1+1/betaMargin)*S+(1+betaMargin)*m^2*eye(3);
end
S=(S+S.')/2;
obsVec=[c.',S(1,1),S(2,2),S(3,3),S(1,2),S(1,3),S(2,3)];
end
function map=localQPMap(obj,x,xd,vfNom,F1,P,delta,Araw,braw,opts)
if nargin<10 || isempty(opts),opts=struct();end
if ~isfield(opts,'actuatorLimits'),opts.actuatorLimits=[0 20 1 1 1];end
if ~isfield(opts,'rcondMin'),opts.rcondMin=1e-9;end
if ~isfield(opts,'auxOmega'),opts.auxOmega=1;end
assert(numel(x)==19 && numel(xd)==28 && numel(P)==9 && numel(delta)==4);
xd=reshape(xd,1,[]);P=reshape(P,1,[]);delta=delta(:);
if isempty(Araw),Araw=zeros(0,4);end
assert(size(Araw,2)==4 && size(Araw,1)==numel(braw));braw=braw(:);
vf0=localDeltaJet(vfNom,F1,delta,0);
vf1=localDeltaJet(vfNom,F1,delta,1);
vfm=localDeltaJet(vfNom,F1,delta,-1);
B0=obj.Beta2_SuspendedLoadxyDst(x,xd,vf0,P);
B1=obj.Beta2_SuspendedLoadxyDst(x,xd,vf1,P);
Bm=obj.Beta2_SuspendedLoadxyDst(x,xd,vfm,P);
if any(~isfinite(B0(:))) || rcond(B0)<opts.rcondMin,error('beta2 is singular.');end
if norm(B1-B0,'fro')>1e-7*(1+norm(B0,'fro')) || norm(Bm-B0,'fro')>1e-7*(1+norm(B0,'fro'))
    error('beta2 depends on nu; affine conversion is invalid.');
end
Z=zeros(3,1);
a0=-obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vf0,Z,P);
a1=-obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vf1,Z,P);
am=-obj.V2_alpha2_SuspendedLoadxyDst(x,xd,vfm,Z,P);
a0=a0(:);a1=a1(:);am=am(:);
aNu=(a1-am)/2;
if norm(a1+am-2*a0)>1e-5*(1+norm(a0)) || any(~isfinite([a0;a1;am]))
    error('alpha2 nonlinear in nu or numerically unstable.');
end
D=B0\eye(3);T=[-D*aNu,D];c=-D*a0;
Av=Araw(:,2:4)*D;
map.Avirtual=[Araw(:,1)-Av*aNu,Av];
map.bvirtual=braw-Av*a0;
lim=opts.actuatorLimits(:);mmax=lim(3:5);
map.Aineq=[-map.Avirtual;T;-T];
map.bineq=[map.bvirtual;mmax-c;mmax+c];
vNom=obj.Vs_SuspendedLoadxyDst(x,xd,vf0,P,obj.param.F2,obj.param.F3,obj.param.F4);
omega=opts.auxOmega;
nuNom=-[omega^4 4*omega^3 6*omega^2 4*omega]*delta;
map.wNom=[nuNom;vNom(:)];
map.torqueT=T;map.torqueOffset=c;map.beta2=B0;map.vfAtNu0=vf0;
uf=obj.Uf_SuspendedLoadxyDst(x,xd,vf0,P);
map.thrust=uf(1);
map.thrustInBounds=isfinite(map.thrust) && map.thrust>=lim(1) && map.thrust<=lim(2);
assert(all(isfinite([map.Aineq(:);map.bineq(:);map.wNom(:)])));
end
function sol=localSolve(map,opts)
if nargin<2 || isempty(opts),opts=struct();end
if ~isfield(opts,'translationScale'),opts.translationScale=1;end
if ~isfield(opts,'yawScale'),opts.yawScale=1;end
if ~isfield(opts,'feasTolerance'),opts.feasTolerance=1e-7;end
s=[opts.translationScale;opts.translationScale;opts.translationScale;opts.yawScale];
H=diag(1./s.^2);q=-H*map.wNom;
sol=struct('success',false,'w',nan(4,1),'fval',nan,'exitflag',-999,'reason','no solution');
if ~map.thrustInBounds
    sol.reason='Current thrust out of range; nu cannot change it instantly.';return;
end
op=optimoptions('quadprog','Display','off');
[w,val,flag]=quadprog(H,q,map.Aineq,map.bineq,[],[],[],[],[],op);
sol.exitflag=flag;sol.fval=val;
if flag>0 && all(isfinite(w)) && all(map.Aineq*w<=map.bineq+opts.feasTolerance)
    sol.success=true;sol.w=w;sol.reason='QP feasible at sample; not a full safety certificate';
end
end

function summary=localSelfTest()
names={'single','sphere','sphere_load','sphere_uav','uav','cable','load'};
summary=struct();
for i=1:numel(names)
    [args,~]=localExample(names{i});
    ticTest=tic;
    [h,psi,A,b,info]=localEvaluate(names{i},args{:}); %#ok<ASGLU>
    summary.(names{i})=struct('valid',info.valid,'seconds',toc(ticTest),...
        'h',h,'Araw',A,'braw',b,'info',info);
    fprintf('[JET-CBF][SELFTEST] %-12s time=%6.3fs valid=%d, r=%d, low=%g, curvature=%g\n',...
        names{i},summary.(names{i}).seconds,info.valid,info.relativeDegreeAssumed,...
        info.maxLowerInputDependence,info.maxAffineCurvature);
end
end
function [args,note]=localExample(name)
roll=.13;pitch=-.17;yaw=.23;
q=[cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
   sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
   cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
   cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2)];
pt=[.14;-.10;-.9849];pt=pt/norm(pt);
wl=[.06;-.08;.03];wl=wl-pt*(pt.'*wl);
x=[q;.11;-.06;.04;.1;.1;2; .3;-.1;.05;pt;wl];
delta=zeros(4,1);
refZ=[2;0;0;0;0;0;0];F1=[5;4];
P=[1.3;.020;.019;.035;9.81;.4;1.1;0;0];
obs=[.8;.9;2;.8^2;.55^2;1.1^2;0;0;0];
switch name
    case 'single',geom=[1.1;.28;1];
    case 'sphere',geom=[.45;.25;1];
    case {'sphere_load','sphere_uav'},geom=[.24;1];
    case 'cable',geom=[.65;.12;1];
    otherwise,geom=[.28;.22;.20;1];
end
kappa=[2;2;2;2;2;2];
args={x,delta,refZ,F1,P,obs,geom,kappa};
note='local numerical audit only; not a proof';
end
