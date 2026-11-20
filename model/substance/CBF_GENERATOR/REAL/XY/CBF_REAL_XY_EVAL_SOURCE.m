function out = CBF_REAL_XY_EVAL_SOURCE(x,u,P,G,parts,lambda)
% CBF_REAL_XY_EVAL_SOURCE: 3-D direct support of two actual ellipsoids.
% No XY projection; no extra Minkowski outer ellipsoid.
% n(x)=(pCable-cObs)/norm(pCable-cObs) is a STATE-DEPENDENT unit normal.
% h= n'*d - sqrt(n'*Qc*n) - sqrt(n'*Qobs*n), with rotating Qc(pT).
% Both n(x) and Qc(pT) are differentiated through order four.
% h>=0 is sufficient for disjoint ellipsoids; h<0 is inconclusive.
% This particular normal need not maximize support separation.
% All derivatives assume piecewise-constant physical T and tauZ over a
% ZOH interval, without extending either input into the state.
validateattributes(x,{'double'},{'vector','numel',19,'finite','real'});
validateattributes(u,{'double'},{'vector','numel',4,'finite','real'});
validateattributes(lambda,{'double'},{'vector','numel',4,'positive','finite'});
if ~isstruct(parts), error('CBFXY:Parts','parts must be a struct array');end
if ~isfield(G,'cableAxisLong') || ~isfield(G,'cableAxisShort') || ...
        ~isfield(G,'cableCenterFromLoad')
    error('CBFXY:Geometry','Missing cable ellipsoid axes/centre');
end
as=G.cableAxisShort;al=G.cableAxisLong;
if ~isfinite(as) || ~isfinite(al) || as<=0 || al<as
    error('CBFXY:Geometry','Invalid cable semi-axes');
end
nParts=numel(parts);
out=struct('names',{cell(nParts,1)},'stages',zeros(5,nParts), ...
    'hTaylor',zeros(5,nParts),'hDerivatives',zeros(5,nParts), ...
    'hDistance',zeros(nParts,1),'normal',zeros(3,nParts), ...
    'bodyCentre',zeros(3,1),'cableShape',zeros(3));
if nParts==0,return;end
jet=CBF_REAL_XY_JET(x,u,P);
X=jet.stateTaylor;
pT=X(14:16,:);
C=X(8:10,:)-G.cableCenterFromLoad*P.cableL*pT;
Qc=zeros(3,3,5);Qc(:,:,1)=as^2*eye(3);
delta=al^2-as^2;
for k=0:4
    for j=0:k
        Qc(:,:,k+1)=Qc(:,:,k+1)+delta*pT(:,j+1)*pT(:,k-j+1).';
    end
end
out.bodyCentre=C(:,1);out.cableShape=Qc(:,:,1);
for obstacle=1:nParts
    [centre,Qobs,label]=xy_obstacle_shape(parts(obstacle),obstacle);
    D=C; D(:,1)=D(:,1)-centre;
    % Direction normal depends on the entire 3-D relative position.
    r2=xy_jet_dot(D,D);
    if r2(1)<1e-12
        error('CBFXY:CoincidentCenters', ...
            'Cable/obstacle centres coincide; unit normal is undefined');
    end
    radius=xy_jet_sqrt(r2);
    invRadius=xy_jet_inverse(radius);
    N=xy_jet_vector_product(D,invRadius);
    cableProjection=xy_jet_quadratic(N,Qc);
    obsJet=zeros(3,3,5);obsJet(:,:,1)=Qobs;
    obsProjection=xy_jet_quadratic(N,obsJet);
    sc=xy_jet_sqrt(cableProjection);
    so=xy_jet_sqrt(obsProjection);
    h=xy_jet_dot(N,D)-sc-so;
    deriv=factorial(0:4)'.*h(:);
    if any(~isfinite(deriv)),error('CBFXY:Numerics','Nonfinite HOCBF jet');end
    out.names{obstacle}=[label '/cable'];
    out.hTaylor(:,obstacle)=h(:);
    out.hDerivatives(:,obstacle)=deriv;
    out.hDistance(obstacle)=h(1);
    out.normal(:,obstacle)=N(:,1);
    for order=0:4
        poly=1;
        for k=1:order,poly=conv(poly,[lambda(k) 1]);end
        out.stages(order+1,obstacle)=poly*deriv(1:order+1);
    end
end
end

function s=xy_jet_dot(A,B)
s=zeros(1,5);
for k=0:4
    for j=0:k,s(k+1)=s(k+1)+A(:,j+1).'*B(:,k-j+1);end
end
end

function Y=xy_jet_vector_product(V,s)
Y=zeros(3,5);
for k=0:4
    for j=0:k,Y(:,k+1)=Y(:,k+1)+V(:,j+1)*s(k-j+1);end
end
end

function s=xy_jet_quadratic(N,Q)
V=zeros(3,5);
for k=0:4
    for j=0:k,V(:,k+1)=V(:,k+1)+Q(:,:,j+1)*N(:,k-j+1);end
end
s=xy_jet_dot(N,V);
end

function s=xy_jet_sqrt(a)
if a(1)<=0 || ~isfinite(a(1))
    error('CBFXY:Support','Ellipsoid support is not differentiable at this state');
end
s=zeros(1,5);s(1)=sqrt(a(1));
for k=1:4
    crossTerms=0;
    for j=1:k-1,crossTerms=crossTerms+s(j+1)*s(k-j+1);end
    s(k+1)=(a(k+1)-crossTerms)/(2*s(1));
end
end

function invs=xy_jet_inverse(s)
invs=zeros(1,5);invs(1)=1/s(1);
for k=1:4
    acc=0;
    for j=1:k,acc=acc+s(j+1)*invs(k-j+1);end
    invs(k+1)=-acc/s(1);
end
end

function [centre,Q,label]=xy_obstacle_shape(part,j)
if ~isfield(part,'center'),error('CBFXY:Obstacle','Missing centre');end
centre=double(part.center(:));
if numel(centre)~=3 || any(~isfinite(centre))
    error('CBFXY:Obstacle','Invalid 3-D obstacle centre');
end
kind='ellipsoid';if isfield(part,'kind'),kind=char(part.kind);end
switch kind
    case 'ellipsoid'
        if isfield(part,'params'),axes=part.params.axes(:);
        else,axes=part.axes(:);end
        R=double(part.R);
    case 'sphere'
        if isfield(part,'params'),r=part.params.radius;
        else,r=part.radius;end
        axes=[r;r;r];R=eye(3);
    otherwise
        error('CBFXY:Obstacle','Unsupported obstacle kind %s',kind);
end
axes=double(axes(:));
if numel(axes)~=3 || any(~isfinite(axes)) || any(axes<=0) || ...
        ~isequal(size(R),[3 3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5 || det(R)<0
    error('CBFXY:Obstacle','Invalid 3-D ellipsoid axes or rotation');
end
Q=R*diag(axes.^2)*R.';
if isfield(part,'obstacleName'),label=char(part.obstacleName);
elseif isfield(part,'name'),label=char(part.name);
else,label=sprintf('obstacle%d',j);end
end
