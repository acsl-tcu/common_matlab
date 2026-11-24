function out=CBF_REAL_D2T_SAMPLED_EVAL_SOURCE(x,Tkm1,Tkm2,v,tauZ,P,parts,geom,dt,rkSteps,lambda,reserve,previousStages)
% ID3: FOUR-STAGE DISCRETE HOCBF for the physical-input-ZOH plant.
% NOT the continuous-Tddot HOCBF. One candidate physical input held for
% ONE interval only. No assumption on future control commands.
% v=[Delta2T_N; tauX; tauY], T_k=2*Tkm1-Tkm2+Delta2T_N.
% Historical barrier states are measured at prior sample; do not replace
% them by a future rollout. For each obstacle, define
%   psi(0,k)=h(x_k),
%   phi(j,k)=(phi(j-1,k)-phi(j-1,k-1))+alpha(j)*phi(j-1,k),
%   alpha(j)=dt*lambda(j), phi(j,k)=dt^j*psi(j,k).
% Using scaled stages avoids catastrophic 1/dt^4 cancellation.
% Enforce phi(4,k+1)>=0 and within-period geometric clearance.
% If phi(0..3,k)>=0 and the map is exact, phi(4,k+1)>=0 implies
% phi(0..3,k+1)>=0 (nested forward invariance at sampling instants).
% This implementation uses RK4 and a non-validated interval bound, so
% it does NOT certify the exact map, intersample safety or feasibility.
validateattributes(x,{'numeric'},{'vector','numel',19,'real','finite'});
validateattributes(v,{'numeric'},{'vector','numel',3,'real','finite'});
validateattributes(lambda,{'numeric'},{'vector','numel',4,'real','positive','finite'});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
validateattributes(reserve,{'numeric'},{'scalar','nonnegative','finite'});
validateattributes(Tkm1,{'numeric'},{'scalar','real','finite'});
validateattributes(Tkm2,{'numeric'},{'scalar','real','finite'});
if nargin<13,previousStages=[];end
x=double(x(:));v=double(v(:));lambda=double(lambda(:));
alpha=lambda*dt;
T=2*Tkm1-Tkm2+v(1);
u=[T;v(2:3);tauZ];
if T<-1e-9 || T>20+1e-9 || any(abs(u(2:4))>1+1e-9)
    error('CBFD2T:InputBox','Virtual input maps outside original physical box');
end
[h0,hdot0,names]=sampled_clearance(x,parts,geom,P.cableL);
n=numel(parts);
% Fresh-obstacle bootstrap: constant-approach-speed *history* for the
% first observation. This is a model initialization, not past evidence.
bootstrap=isempty(previousStages);
if bootstrap
    hp=h0-dt*hdot0;
    a1=alpha(1);a2=alpha(2);a3=alpha(3);
    previousStages=[hp.'; ...
        (dt*hdot0+a1*hp).'; ...
        (dt*(a1+a2)*hdot0+a1*a2*hp).'; ...
        (dt*(a1*a2+a1*a3+a2*a3)*hdot0+a1*a2*a3*hp).'];
end
if ~isequal(size(previousStages),[4,n]) || any(~isfinite(previousStages(:)))
    error('CBFD2T:StageMemory','Previous stages must be finite 4-by-n');
end
cur=zeros(4,n);cur(1,:)=h0.';
for j=2:4
    cur(j,:)=(cur(j-1,:)-previousStages(j-1,:))+ ...
        alpha(j-1)*cur(j-1,:);
end
pred=CBF_REAL_D2T_ZOH_EVAL(x,u,P,parts,geom,dt,rkSteps);
[h1,hdot1]=sampled_clearance(pred.xEnd,parts,geom,P.cableL);
next=zeros(5,n);next(1,:)=h1.';
for j=2:5
    next(j,:)=(next(j-1,:)-cur(j-1,:))+ ...
        alpha(j-1)*next(j-1,:);
end
% Both constraints use SAME three inputs. Relative magnitudes differ;
% scale QP rows separately, not the physics or chosen lambda values.
res=[next(5,:).';pred.intervalLower-reserve];
out=struct('physicalInput',u,'discreteSecondDifference',v(1)/dt^2, ...
    'delta2ThrustN',v(1),'input',v,'h0',h0,'hEnd',h1, ...
    'hDot0',hdot0,'hDotEnd',hdot1, ...
    'currentStages',cur,'nextStages',next, ...
    'previousStagesUsed',previousStages,'bootstrapUsed',bootstrap, ...
    'intervalLower',pred.intervalLower,'residual',res, ...
    'names',{names},'xEnd',pred.xEnd,'slopeUpper',pred.slopeUpper, ...
    'sampledDynamics','19-state physical ZOH + thrust history', ...
    'safetyCertified',false,'gainCountUsed',4, ...
    'stageConvention','phi_i=dt^i*psi_i [m]', 'alpha',alpha);
end

function [h,hdot,names]=sampled_clearance(x,parts,geom,L)
% Same 3D centre-normal supporting-plane geometry as XY obstacle HOCBF.
C=x(8:10)-0.5*L*x(14:16);
Cdot=x(11:13)-0.5*L*cross(x(17:19),x(14:16));
pT=x(14:16);pTdot=cross(x(17:19),pT);
nPart=numel(parts);h=zeros(nPart,1);hdot=zeros(nPart,1);
names=cell(nPart,1);
for j=1:nPart
    part=parts(j);d=C-double(part.center(:));D=norm(d);
    if D<1e-10,error('CBFD2T:CentreCoincidence','Centre-normal undefined');end
    n=d/D;ndot=(Cdot-n*(n.'*Cdot))/D;
    if strcmp(char(part.kind),'sphere')
        if isfield(part,'params') && ~isempty(part.params)
            r=double(part.params.radius);
        else,r=double(part.radius);end
        Q=(r*r)*eye(3);
    elseif strcmp(char(part.kind),'ellipsoid')
        if isfield(part,'params') && ~isempty(part.params)
            ax=double(part.params.axes(:));
        else,ax=double(part.axes(:));end
        if isfield(part,'R'),R=double(part.R);else,R=eye(3);end
        Q=R*diag(ax.^2)*R.';
    else,error('CBFD2T:ObstacleType','Unsupported obstacle type');end
    a=n.'*pT;ad=ndot.'*pT+n.'*pTdot;
    sCable=sqrt(geom.b^2+(geom.a^2-geom.b^2)*a^2);
    sObs=sqrt(n.'*Q*n);
    h(j)=D-sCable-sObs;
    hdot(j)=n.'*Cdot-((geom.a^2-geom.b^2)*a*ad)/sCable ...
        -(ndot.'*Q*n)/sObs;
    if isfield(part,'obstacleName'),names{j}=char(part.obstacleName);
    elseif isfield(part,'name'),names{j}=char(part.name);
    else,names{j}=sprintf('obs%d',j);end
end
end
