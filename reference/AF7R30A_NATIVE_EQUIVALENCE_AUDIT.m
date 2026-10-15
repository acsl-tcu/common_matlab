function OUT = AF7R30A_NATIVE_EQUIVALENCE_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R30A | NATIVE SUPPORT/HULL EQUIVALENCE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] implementationEquivalence=1 | sampledSafetyAcceptance=0 | ieee754OutwardClaim=0\n');

assert(exist('af7r30_support_hull_mex','file')==3,'[AF7R30A] MEX missing.');

angs=[0,0.2,-0.7,1.1];
maxPoint=0; maxSeg=0; hullMismatch=0; ntest=0;
for ia=1:numel(angs)
    th=angs(ia);
    R=[cos(th),-sin(th),0;sin(th),cos(th),0;0,0,1];
    c=[0.3;-0.4;0.8]+0.1*ia;
    a=[0.8;1.3;2.1]+0.05*ia;
    rho=0.02*ia;
    for ix=1:8
        x=[-2+0.7*ix; 1.5-0.2*ix; -0.6+0.31*ix];
        [mm,~]=af7r30_support_hull_mex(0,x,c,R,a,rho);
        mr=point_ref(x,c,R,a,rho);
        maxPoint=max(maxPoint,abs(mm-mr));

        p0=x; p1=x+[0.4-0.03*ix;-0.2+0.02*ix;0.1+0.01*ix];
        [ms,~]=af7r30_support_hull_mex(1,p0,p1,c,R,a,rho);
        sr=segment_ref(p0,p1,c,R,a,rho);
        maxSeg=max(maxSeg,abs(ms-sr));
        ntest=ntest+1;
    end
end

SETS={ ...
    {[0;1;2],[1;0;1]}, ...
    {[0;0;1;2],[1;2;1;1]}, ...
    {[-2;-1;0;1;2],[0;1;0.2;1.3;0.1]}, ...
    {[0;1;2;3;4],[1;1;1;1;1]}};
for k=1:numel(SETS)
    S=SETS{k}{1}; B=SETS{k}{2};
    km=af7r30_support_hull_mex(2,S,B);
    kr=hull_ref(S,B);
    hullMismatch=max(hullMismatch,~isequal(double(km(:)),double(kr(:))));
end

scale=max([1,maxPoint,maxSeg]);
tol=512*eps(scale);
closed=(maxPoint<=tol)&&(maxSeg<=tol)&&(hullMismatch==0);

fprintf(' deterministic geometry cases            = %d\n',ntest);
fprintf(' max point margin |MEX-MATLAB|           = %.17g\n',maxPoint);
fprintf(' max segment margin |MEX-MATLAB|         = %.17g\n',maxSeg);
fprintf(' upper-hull index mismatch               = %d\n',hullMismatch);
fprintf(' audit tolerance                         = %.17g\n',tol);
fprintf(' IEEE754 outward closure claimed         = 0\n');
fprintf(' equivalence closed                      = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R30A_NATIVE_EQUIVALENCE_AUDIT_V1', ...
    'max_point_abs_diff',maxPoint,'max_segment_abs_diff',maxSeg, ...
    'hull_mismatch',hullMismatch,'tolerance',tol,'closed',closed);
assignin('base','AF7R30A_NATIVE_EQUIVALENCE_AUDIT_OUT',OUT);
end

function m=point_ref(x,c,R,a,rho)
y=R.'*(x-c);
N=[R*(y./(a.^2)),x-c];
m=-inf;
for k=1:size(N,2)
    n=N(:,k); nn=norm(n);
    if nn>0
        h=dot(n,c)+norm(a.*(R.'*n));
        m=max(m,(dot(n,x)-h-rho*nn)/nn);
    end
end
end

function m=segment_ref(p0,p1,c,R,a,rho)
z0=(R.'*(p0-c))./a; z1=(R.'*(p1-c))./a;
s=z1-z0; ss=dot(s,s);
if ss<=0, lam=0; else, lam=max(0,min(1,-dot(z0,s)/ss)); end
xs=p0+lam*(p1-p0);
Y=[xs,p0,p1]; m=-inf;
for j=1:3
    y=R.'*(Y(:,j)-c);
    N=[R*(y./(a.^2)),Y(:,j)-c];
    for k=1:2
        n=N(:,k); nn=norm(n);
        if nn>0
            h=dot(n,c)+norm(a.*(R.'*n));
            pr=min(dot(n,p0),dot(n,p1));
            m=max(m,(pr-h-rho*nn)/nn);
        end
    end
end
end

function keep=hull_ref(S,B)
S=double(S(:)); B=double(B(:));
[S,ord]=sort(S); B=B(ord);
Su=[]; Bu=[]; Iu=[];
k=1;
while k<=numel(S)
    ss=S(k); j=k; bb=B(k); src=k;
    while j<numel(S)&&S(j+1)==ss
        j=j+1;
        if B(j)>bb, bb=B(j); src=j; end
    end
    Su(end+1,1)=ss; %#ok<AGROW>
    Bu(end+1,1)=bb; %#ok<AGROW>
    Iu(end+1,1)=ord(src); %#ok<AGROW>
    k=j+1;
end
H=zeros(numel(Su),1); nh=0;
for k=1:numel(Su)
    while nh>=2
        i1=H(nh-1); i2=H(nh); i3=k;
        dx1=Su(i2)-Su(i1); dy1=Bu(i2)-Bu(i1);
        dx2=Su(i3)-Su(i2); dy2=Bu(i3)-Bu(i2);
        t1=dx1*dy2; t2=dy1*dx2; cr=t1-t2;
        scale=max([1,abs(t1),abs(t2),abs(dx1),abs(dx2),abs(dy1),abs(dy2)]);
        guard=128*eps(scale);
        if cr>guard || cr==0, nh=nh-1; else, break; end
    end
    nh=nh+1; H(nh)=k;
end
keep=sort(Iu(H(1:nh)));
end
