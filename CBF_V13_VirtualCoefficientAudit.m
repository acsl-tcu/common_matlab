function D=CBF_V13_VirtualCoefficientAudit(shape,xv,s,ob)
%CBF_V13_VIRTUALCOEFFICIENTAUDIT Independent test-only audit of mixed 4/6 rows.
% No input clipping, soft slacks, or safety claims. Uses corrected V12 jets.
nu0=zeros(4,1);
[H0,labels,orders]=CBF_V12_VirtualRows(shape,xv,nu0,s,ob);
n=numel(labels);A=zeros(n,4);b=zeros(n,1);
psiMin=zeros(n,1);early=zeros(n,4);curvature=zeros(n,4);
for j=1:4
    e=zeros(4,1);e(j)=1;
    Hp=CBF_V12_VirtualRows(shape,xv,e,s,ob);
    Hm=CBF_V12_VirtualRows(shape,xv,-e,s,ob);
    for i=1:n
        r=orders(i);
        A(i,j)=factorial(r)*(Hp(i,r+1)-Hm(i,r+1))/2;
        curvature(i,j)=factorial(r)*abs(Hp(i,r+1)+Hm(i,r+1)-2*H0(i,r+1));
        early(i,j)=max(abs((Hp(i,1:r)-Hm(i,1:r)).*factorial(0:r-1)/2));
    end
end
for i=1:n
    r=orders(i);k=s.cbf.hocbfGain;
    if strncmp(labels{i},'ground_',7) && isfield(s.cbf,'groundGain') && ...
            ~isempty(s.cbf.groundGain),k=s.cbf.groundGain;end
    f=H0(i,1:r+1).*factorial(0:r);
    F=f(r+1);
    for j=0:r-1
        F=F+nchoosek(r,j)*k^(r-j)*f(j+1);
    end
    b(i)=-F;
    P=zeros(r,1);
    for j=0:r-1
        for m=0:j
            P(j+1)=P(j+1)+nchoosek(j,m)*k^(j-m)*f(m+1);
        end
    end
    psiMin(i)=min(P);
end
D=struct('H0',H0(:,1),'labels',{labels},'orders',orders,'A',A,'b',b,...
    'psiMin',psiMin,'early',early,'curvature',curvature,...
    'maxEarly',max(early(:)),'maxCurvature',max(curvature(:)));
end
