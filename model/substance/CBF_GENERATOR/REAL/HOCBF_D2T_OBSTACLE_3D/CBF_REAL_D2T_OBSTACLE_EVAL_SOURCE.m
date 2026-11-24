function out = CBF_REAL_D2T_OBSTACLE_EVAL_SOURCE(y,v,tauZ,P,G,parts,lambda)
% 3D obstacle separation ONLY: no body tilt, cable tilt, or ground rows.
% y=[19-state physical x;T;dT], v=[ddT;tauX;tauY]; tauZ fixed.
% G.collisionMode='load_sphere' (default), 'drone_sphere' or 'cable_ellipsoid'.
% Last mode retains the original XY obstacle/cable collision geometry; it is
% OPTIONAL, never adds an independent cable-tilt constraint.
% A state-dependent centre-to-centre normal gives a SUFFICIENT (not necessary)
% ellipsoid separation condition, singular for coincident centres.
validateattributes(y,{'double'},{'vector','numel',21,'finite','real'});
validateattributes(v,{'double'},{'vector','numel',3,'finite','real'});
validateattributes(tauZ,{'double'},{'scalar','finite','real'});
validateattributes(lambda,{'double'},{'vector','numel',4,'positive','finite'});
if ~isstruct(parts),error('CBFD2T:Obstacles','parts must be a struct array');end
y=y(:);v=v(:);lambda=lambda(:);
jet=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),v(1),[v(2:3);tauZ],P);
X=jet.stateTaylor;S=5;
mode='load_sphere';
if isfield(G,'collisionMode'),mode=char(G.collisionMode);end
Q=zeros(3,3,S);
switch mode
    case {'load_sphere','drone_sphere'}
        if ~isfield(G,'radius'),error('CBFD2T:Radius','G.radius required');end
        r=G.radius;
        validateattributes(r,{'double'},{'scalar','positive','finite'});
        C=X(8:10,:);
        if strcmp(mode,'drone_sphere'),C=C-P.cableL*X(14:16,:);end
        Q(:,:,1)=r^2*eye(3);
    case 'cable_ellipsoid'
        if ~all(isfield(G,{'cableAxisShort','cableAxisLong','cableCenterFromLoad'}))
            error('CBFD2T:CableGeometry','Specify ellipsoid axes and centre fraction');
        end
        as=G.cableAxisShort;al=G.cableAxisLong;
        validateattributes(as,{'double'},{'scalar','positive','finite'});
        validateattributes(al,{'double'},{'scalar','finite','>=',as});
        C=X(8:10,:)-G.cableCenterFromLoad*P.cableL*X(14:16,:);
        Q(:,:,1)=as^2*eye(3);delta=al^2-as^2;
        pT=X(14:16,:);
        for k=0:4
            for j=0:k
                Q(:,:,k+1)=Q(:,:,k+1)+delta*pT(:,j+1)*pT(:,k-j+1).';
            end
        end
    otherwise
        error('CBFD2T:Mode','Unsupported collision mode %s',mode);
end
num=numel(parts);
out=struct('names',{cell(num,1)},'collisionMode',mode, ...
    'hTaylor',zeros(5,num),'hDerivatives',zeros(5,num), ...
    'stages',zeros(5,num),'normal',zeros(3,num), ...
    'centre',C(:,1),'shape',Q(:,:,1),'input',v, ...
    'thrust',[y(20);y(21);v(1)]);
for j=1:num
    [obsC,Qobs,label]=obstacle_shape(parts(j),j);
    D=C;D(:,1)=D(:,1)-obsC;
    r2=jetdot(D,D);
    if r2(1)<1e-12
        error('CBFD2T:CoincidentCentres','Obstacle and protected centres coincide');
    end
    normal=jetvector(D,jetinverse(jetsqrt(r2)));
    QobsSeries=zeros(3,3,S);QobsSeries(:,:,1)=Qobs;
    sc=jetsqrt(jetquad(normal,Q));
    so=jetsqrt(jetquad(normal,QobsSeries));
    h=jetdot(normal,D)-sc-so;
    d=factorial(0:4)'.*h(:);
    if any(~isfinite(d)),error('CBFD2T:Numerics','Nonfinite barrier jet');end
    out.names{j}=label;
    out.normal(:,j)=normal(:,1);
    out.hTaylor(:,j)=h(:);
    out.hDerivatives(:,j)=d;
    for order=0:4
        poly=1;
        for k=1:order,poly=conv(poly,[lambda(k),1]);end
        out.stages(order+1,j)=poly*d(1:order+1);
    end
end
end

function [c,Q,label]=obstacle_shape(part,j)
if ~isfield(part,'center'),error('CBFD2T:ObstacleCentre','centre missing');end
c=double(part.center(:));
validateattributes(c,{'double'},{'vector','numel',3,'finite','real'});
kind='ellipsoid';if isfield(part,'kind'),kind=char(part.kind);end
switch kind
    case 'sphere'
        if isfield(part,'params'),a=part.params.radius;else,a=part.radius;end
        a=repmat(a,3,1);R=eye(3);
    case 'ellipsoid'
        if isfield(part,'params'),a=part.params.axes;else,a=part.axes;end
        if isfield(part,'R'),R=double(part.R);else,R=eye(3);end
    otherwise
        error('CBFD2T:ObstacleType','Unknown obstacle kind %s',kind);
end
a=double(a(:));
validateattributes(a,{'double'},{'vector','numel',3,'positive','finite'});
if ~isequal(size(R),[3,3]) || any(~isfinite(R(:))) || ...
        norm(R.'*R-eye(3),'fro')>1e-5 || det(R)<0
    error('CBFD2T:ObstacleRotation','Invalid obstacle rotation');
end
Q=R*diag(a.^2)*R.';
label=sprintf('obstacle%d',j);
if isfield(part,'obstacleName'),label=char(part.obstacleName);
elseif isfield(part,'name'),label=char(part.name);end
end
function s=jetdot(A,B)
s=zeros(1,5);
for k=0:4
    for j=0:k,s(k+1)=s(k+1)+A(:,j+1).'*B(:,k-j+1);end
end
end
function V=jetvector(A,b)
V=zeros(3,5);
for k=0:4
    for j=0:k,V(:,k+1)=V(:,k+1)+A(:,j+1)*b(k-j+1);end
end
end
function s=jetquad(n,Q)
V=zeros(3,5);
for k=0:4
    for j=0:k,V(:,k+1)=V(:,k+1)+Q(:,:,j+1)*n(:,k-j+1);end
end
s=jetdot(n,V);
end
function r=jetsqrt(s)
if s(1)<=0 || ~isfinite(s(1)),error('CBFD2T:Support','Invalid square root');end
r=zeros(1,5);r(1)=sqrt(s(1));
for k=1:4
    crossTerms=0;
    for j=1:k-1,crossTerms=crossTerms+r(j+1)*r(k-j+1);end
    r(k+1)=(s(k+1)-crossTerms)/(2*r(1));
end
end
function b=jetinverse(a)
if abs(a(1))<1e-12,error('CBFD2T:Inverse','Inverse singular');end
b=zeros(1,5);b(1)=1/a(1);
for k=1:4
    acc=0;
    for j=1:k,acc=acc+a(j+1)*b(k-j+1);end
    b(k+1)=-acc/a(1);
end
end
