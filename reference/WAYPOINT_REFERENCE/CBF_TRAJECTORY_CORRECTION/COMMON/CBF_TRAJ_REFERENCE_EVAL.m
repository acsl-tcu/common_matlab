function jet=CBF_TRAJ_REFERENCE_EVAL(method,s,H,base28,baseEval)
% Reconstructed five local BASE reference methods from REV08..12 notes.
% All match actual current post-SLOAD 28-vector exactly. No approximation
% of the correction: caller adds the exact analytic correction jet.
validateattributes(method,{'numeric'},{'scalar','integer','>=',1,'<=',5});
validateattributes(s,{'numeric'},{'scalar','finite','nonnegative','<=',H});
if s==0,jet=base28(:);return;end
j0=reshape(base28,4,7);
if method==1,jet=baseEval(s);jet=jet(:);return;end
% Taylor at current time in normalized coordinates.
a=zeros(4,7);
for j=0:6,a(:,j+1)=j0(:,j+1)*H^j/factorial(j);end
if method==2,jet=reshape(CBF_TRAJ_POLY_JET(a,s,H),28,1);return;end
j1=reshape(baseEval(H),4,7);
switch method
    case 3
        a(:,8)=(j1(:,7)-j0(:,7))*H^6/factorial(7);
        jet=CBF_TRAJ_POLY_JET(a,s,H);
    case {4,5}
        prep=stable_prepare(base28,baseEval,H);
        jet=stable_eval(method,s,H,prep);
end
jet=jet(:);
end

function prep=stable_prepare(b,f,H)
% Stable residual formulation. Q(u)=(f(Hu)-Taylor6(Hu))/H^6.
% R(:,k+1)=Q^(k)(1), computed from sixth derivative differences,
% NOT by subtracting nearly equal large position endpoint values.
% Exact integral identity, evaluated by Gauss quadrature; endpoints checked.
persistent nodes weights
if isempty(nodes)
    n=16;k=(1:n-1)';q=k./sqrt(4*k.^2-1);[V,D]=eig(diag(q,1)+diag(q,-1));[nodes,idx]=sort(diag(D));weights=2*V(1,idx).^2;
end
j0=reshape(b,4,7);j1=reshape(f(H),4,7);R=zeros(4,7);
R(:,7)=j1(:,7)-j0(:,7);
for half=1:2
    for i=1:numel(nodes)
        u=(half-1)/2+(1+nodes(i))/4;w=weights(i)/4;
        z=reshape(f(H*u),4,7);delta=z(:,7)-j0(:,7);
        for k=0:5,R(:,k+1)=R(:,k+1)+w*(1-u)^(5-k)/factorial(5-k)*delta;end
    end
end
% Hermite degree 13: first 7 Bernstein coefficients zero; last 7
% satisfy right endpoint derivatives. Flatblend: right residual Taylor
% polynomial of degree 6 multiplied by flatstep of degree 13.
B=zeros(4,14);B(:,8:14)=right_coeff(R,13);
T=right_coeff(R,6);F=[zeros(1,7),ones(1,7)];C=zeros(4,20);
for k=0:19
    for i=max(0,k-6):min(13,k)
        j=k-i;C(:,k+1)=C(:,k+1)+F(i+1)*T(:,j+1)*nchoosek(13,i)*nchoosek(6,j)/nchoosek(19,k);
    end
end
% Precompute derivative coefficients in normalized u, never in seconds.
prep=struct('j0',j0,'hermite',{bern_derivatives(B)},'blend',{bern_derivatives(C)});
end
function B=right_coeff(R,n)
B=zeros(size(R,1),7);
for k=0:6
    for j=0:k,B(:,7-k)=B(:,7-k)+(-1)^j*nchoosek(k,j)*R(:,j+1)/prod(n-j+1:n);end
end
end
function D=bern_derivatives(B)
D=cell(1,7);D{1}=B;
for j=1:6,D{j+1}=(size(D{j},2)-1)*diff(D{j},1,2);end
end
function z=stable_eval(m,s,H,prep)
% No endpoint snapping. Evaluate the same polynomial at s=0, H and inside.
u=s/H;if m==4,D=prep.hermite;else,D=prep.blend;end
z=zeros(4,7);
for j=0:6
    v=zeros(4,1);
    for k=6:-1:j,v=v*s+prep.j0(:,k+1)/factorial(k-j);end
    B=D{j+1};n=size(B,2)-1;w=zeros(n+1,1);
    for k=0:n,w(k+1)=nchoosek(n,k)*u^k*(1-u)^(n-k);end
    z(:,j+1)=v+H^(6-j)*(B*w);
end
z=z(:);
end

