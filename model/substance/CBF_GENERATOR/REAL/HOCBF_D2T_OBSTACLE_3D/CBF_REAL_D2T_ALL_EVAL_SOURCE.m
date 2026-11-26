function out=CBF_REAL_D2T_ALL_EVAL_SOURCE(y,w,tauZ,P,G,parts,opt,plan)
% REV60: parameterized CT HOCBF evaluator; REV58 mathematical core.
% plan is selected once at the control tick. PW eta and FIXED normal are
% held parameters on the interval. SHAPE and PW normals vary with state.
% All stages are CT HOCBF jets of that particular parameterized function.
% A*w+b is the last stage (direct input sensitivities); no SD certificate.
% Required opts: mode, spheres, extras, groundEnabled, tensionEnabled,
% obsLambda, bodyLambda, cableLambda, groundLambda, tensionLambda,
% bodyDeg, cableDeg, groundZ, tensionMin. G sphere radii supplied by caller.
if nargin<8,plan=[];end
y=double(y(:));w=double(w(:));
validateattributes(y,{'double'},{'numel',21,'finite','real'});
validateattributes(w,{'double'},{'numel',3,'finite','real'});
validateattributes(opt.obsLambda,{'numeric'},{'numel',4,'positive','finite'});
mode=char(opt.mode);assert(ismember(mode,{'CENTER','SHAPE','PW','FIXED_SHAPE','SELECTIVE'}),'REV55:Mode');
J=CBF_REAL_D2T_JET(y(1:19),y(20),y(21),w(1),[w(2:3);tauZ],P);X=J.extendedTaylor;
p=X(14:16,:);q=y(1:4);a=q(1);b=q(2);c=q(3);d=q(4);
R=[a*a+b*b-c*c-d*d,2*(b*c-a*d),2*(b*d+a*c);2*(b*c+a*d),a*a-b*b+c*c-d*d,2*(c*d-a*b);2*(b*d-a*c),2*(c*d+a*b),a*a-b*b-c*c+d*d];
e=R(:,3);Db=[zeros(3,1),-R(:,2)/P.J(1),R(:,1)/P.J(2)];
Dp=zeros(3,3);DL=Dp;for j=1:3
    Dp(:,j)=cross(-(double(j==1)*cross(p(:,1),e)+y(20)*cross(p(:,1),Db(:,j)))/(P.m*P.cableL),p(:,1));
    DL(:,j)=p(:,1)*(double(j==1)*dot(p(:,1),e)+y(20)*dot(p(:,1),Db(:,j)))/(P.m+P.mL);
end
fractions=G.cableCenterFromLoad;short=G.cableAxisShort;long=G.cableAxisLong;types={'CABLE'};
if opt.droneSphereEnabled
    fractions=[fractions,1];short=[short,G.droneSafetyRadius];long=[long,G.droneSafetyRadius];types=[types,{'DRONE_SPHERE'}];
end
if opt.loadSphereEnabled
    fractions=[fractions,0];short=[short,G.loadSafetyRadius];long=[long,G.loadSafetyRadius];types=[types,{'LOAD_SPHERE'}];
end
assert(all(isfinite([short,long,fractions]))&&all(short>0)&&all(long>=short),'REV55:Geometry');
nr=numel(parts)*numel(types);fresh=isempty(plan);
if fresh,plan=struct('mode',mode,'eta',nan(nr,1),'normal',nan(3,nr));else,assert(strcmp(plan.mode,mode)&&numel(plan.eta)==nr&&isequal(size(plan.normal),[3,nr]),'REV55:Plan');end
st={};names={};A=zeros(0,3);normals=zeros(3,nr);hT={};k=0;
for m=1:numel(types)
    C=X(8:10,:)-fractions(m)*P.cableL*p;DC=DL-fractions(m)*P.cableL*Dp;
    delta=long(m)^2-short(m)^2;Q=zeros(3,3,5);Q(:,:,1)=short(m)^2*eye(3);
    for i=0:4,for j=0:i,Q(:,:,i+1)=Q(:,:,i+1)+delta*p(:,j+1)*p(:,i-j+1).';end,end
    for j=1:numel(parts)
        k=k+1;[oc,Qo,label]=obstacle_shape(parts(j),j);D=C;D(:,1)=D(:,1)-oc;D0=D(:,1);
        if D0.'*D0<1e-12,error('REV55:Coincident','Centres too close for legacy normal domain');end
        if fresh
            switch mode
            case 'CENTER',plan.eta(k)=0;
            case {'SHAPE','FIXED_SHAPE','SELECTIVE'},plan.eta(k)=.5;
            case 'PW'
                % Scalar search only; positivity of actual gap is tested separately.
                f=@(eta)-eta*(1-eta)*(D0.'*(((1-eta)*Q(:,:,1)+eta*Qo)\D0));
                plan.eta(k)=fminbnd(f,0,1,optimset('Display','off','TolX',1e-8));
            end
        end
        eta=plan.eta(k);
        if fresh&&strcmp(mode,'FIXED_SHAPE')
            v0=((1-eta)*Q(:,:,1)+eta*Qo)\D0;plan.normal(:,k)=v0/norm(v0);
        end
        [h,aa,n,ss]=plane_row58(D,Q,Qo,p,delta,DC,Dp,mode,eta,plan.normal(:,k),opt.obsLambda);
        if fresh&&strcmp(mode,'SELECTIVE')&&any(ss(1:end-1)<0)
            f=@(et)-et*(1-et)*(D0.'*(((1-et)*Q(:,:,1)+et*Qo)\D0));
            ep=fminbnd(f,0,1,optimset('Display','off','TolX',1e-8));
            [hp,ap,np,sp]=plane_row58(D,Q,Qo,p,delta,DC,Dp,mode,ep,plan.normal(:,k),opt.obsLambda);
            if all(sp(1:end-1)>=0),eta=ep;h=hp;aa=ap;n=np;ss=sp;end
        end
        if fresh,plan.eta(k)=eta;plan.normal(:,k)=n;end
        A(end+1,:)=aa;st{end+1}=ss;hT{end+1}=h;names{end+1}=sprintf('%s_%d_%s',types{m},j,label);normals(:,k)=n;
    end
end
if opt.extras
    qb=zeros(1,5);sg=[1,-1,-1,1];for j=1:4,qb=qb+sg(j)*mul_series49(X(j,:),X(j,:));end
    qb(1)=qb(1)-cosd(opt.bodyDeg);hc=-p(3,:);hc(1)=hc(1)-cosd(opt.cableDeg);
    if opt.bodyEnabled
        st=[st,{stages(qb,opt.bodyLambda)}];hT=[hT,{qb}];names=[names,{'BODY_TILT'}];A=[A;Db(3,:)];
    end
    if opt.cableEnabled
        st=[st,{stages(hc,opt.cableLambda)}];hT=[hT,{hc}];names=[names,{'CABLE_TILT'}];A=[A;-Dp(3,:)];
    end
    if opt.groundEnabled
        hg=X(10,:);hg(1)=hg(1)-opt.groundZ;hq=hg-P.cableL*p(3,:);
        st=[st,{stages(hg,opt.groundLambda),stages(hq,opt.groundLambda)}];hT=[hT,{hg,hq}];names=[names,{'GROUND_LOAD','GROUND_BODY'}];A=[A;DL(3,:);DL(3,:)-P.cableL*Dp(3,:)];
    end
    if opt.tensionEnabled
        [hh,BB]=extra_point51(X,P,struct('groundEnabled',false,'tensionEnabled',true,'tensionMin',opt.tensionMin),DL,Dp,Db);
        st=[st,{stages(hh{1},opt.tensionLambda)}];hT=[hT,hh];names=[names,{'TENSION_MIN'}];A=[A;BB];
    end
end
last=zeros(numel(st),1);low=[];hs=last;for i=1:numel(st),last(i)=st{i}(end);hs(i)=st{i}(1);low=[low;st{i}(1:end-1)];end
[tension,~]=extra_point51(X,P,struct('groundEnabled',false,'tensionEnabled',true,'tensionMin',0),DL,Dp,Db);
out=struct('tensionN',tension{1}(1),'A',A,'b',last-A*w,'last',last,'lower',low,'h',hs,'stages',{st},'hTaylor',{hT},'names',{names},'plan',plan,'normal',normals,'certifiedSampledData',false);
assert(all(isfinite([A(:);out.b;low(:);normals(:)])),'REV55:Nonfinite');
end
function [h,aa,n,ss]=plane_row58(D,Q,Qo,p,delta,DC,Dp,mode,eta,fixedNormal,lambda)
D0=D(:,1);
        M=Q.*(1-eta);M(:,:,1)=M(:,:,1)+eta*Qo;
        if strcmp(mode,'CENTER'),M=zeros(3,3,5);M(:,:,1)=eye(3);end
        if strcmp(mode,'FIXED_SHAPE')
            N=zeros(3,5);N(:,1)=fixedNormal;
        else
            if strcmp(mode,'CENTER'),V=D;else
                V=zeros(3,5);
                for i=0:4
                    t=D(:,i+1);for l=1:i,t=t-M(:,:,l+1)*V(:,i-l+1);end
                    V(:,i+1)=M(:,:,1)\t;
                end
            end
            vn=norm(V(:,1));assert(vn>0&&isfinite(vn),'REV55:Normal');
            N=jetvector(V,jetinverse(jetsqrt(jetdot(V,V))));
        end
        n=N(:,1);QoS=zeros(3,3,5);QoS(:,:,1)=Qo;
        sc=jetsqrt(jetquad(N,Q));so=jetsqrt(jetquad(N,QoS));
        h=jetdot(N,D)-sc-so;
        % Differential of the exact support gap, including moving normal.
        pn=p(:,1).'*n;gD=n;gp=-delta*pn*n/sc(1);
        if ~strcmp(mode,'FIXED_SHAPE')
            gN=D0-Q(:,:,1)*n/sc(1)-Qo*n/so(1);
            av=M(:,:,1).'\((eye(3)-n*n.')*gN/vn);
            gD=gD+av;
            if ~strcmp(mode,'CENTER'),gp=gp-(1-eta)*delta*((p(:,1).'*V(:,1))*av+(av.'*p(:,1))*V(:,1));end
        end
        aa=gD.'*DC+gp.'*Dp;ss=stages(h,lambda);
end

function s=stages(h,l)
l=l(:);assert(ismember(numel(l),[2,4])&&all(isfinite(l))&&all(l>0),'REV55:Gains');d=h(:).*factorial((0:4)');s=zeros(numel(l)+1,1);s(1)=d(1);pol=1;for i=1:numel(l),pol=conv(pol,[l(i),1]);s(i+1)=pol*d(1:i+1);end
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
function c=mul_series49(a,b)
c=zeros(size(a));for k=1:size(a,2),c(:,k)=sum(a(:,1:k).*b(:,k:-1:1),2);end
end
function [h,B]=extra_point51(X,P,o,DL,Dp,Db)
% Ordinary Taylor coefficients. Ground positions z_L and z_Q=z_L-L*p_z.
% Tension N=-mL*radial. Relative degree 4/4/2 in the extended system.
h={};B=zeros(0,3);if ~o.groundEnabled&&~o.tensionEnabled,return;end;N=size(X,2);p=X(14:16,:);T=X(20,:);
if o.groundEnabled
    a=X(10,:);a(1)=a(1)-o.groundZ;c=a-P.cableL*p(3,:);h=[h,{a,c}];
    if nargin>=6,B=[DL(3,:);DL(3,:)-P.cableL*Dp(3,:)];end
end
if o.tensionEnabled
    q=X(1:4,:);a=q(1,:);b=q(2,:);c=q(3,:);d=q(4,:);
    e=[2*(mul_series49(b,d)+mul_series49(a,c));2*(mul_series49(c,d)-mul_series49(a,b));mul_series49(a,a)-mul_series49(b,b)-mul_series49(c,c)+mul_series49(d,d)];
    wl=X(17:19,:);v=zeros(3,N);for i=1:3,j=mod(i,3)+1;k=mod(i+1,3)+1;v(i,:)=mul_series49(wl(j,:),p(k,:))-mul_series49(wl(k,:),p(j,:));end
    pe=zeros(1,N);vv=pe;for i=1:3,pe=pe+mul_series49(p(i,:),e(i,:));vv=vv+mul_series49(v(i,:),v(i,:));end
    f=-P.mL/(P.m+P.mL)*(mul_series49(T,pe)-P.m*P.cableL*vv);f(1)=f(1)-o.tensionMin;h=[h,{f}];
    if nargin>=6,B=[B;-P.mL/(P.m+P.mL)*[dot(p(:,1),e(:,1)),T(1)*p(:,1).'*Db(:,2:3)]];end
end
end


