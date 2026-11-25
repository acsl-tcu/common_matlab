function jet=CBF_TRAJ_HERMITE_JET(j0,j1,s,H)
% Degree-13 Hermite interpolant in a factored endpoint basis. The endpoint
% basis avoids a poorly conditioned monomial solve and terminal cancellation.
if s==0,jet=j0;return;end
if s==H,jet=j1;return;end
rev=(-1).^(0:6);
jet=leftJet(j0,s,H)+leftJet(j1.*rev,H-s,H).*rev;
end
function jet=leftJet(j,s,H)
% (1-u)^7 * trunc_6( Taylor_6(j,u)/(1-u)^7 ).
a=zeros(size(j,1),7);u=s/H;
for n=0:6
    for k=0:n
        a(:,n+1)=a(:,n+1)+j(:,k+1)*H^k/factorial(k)*nchoosek(6+n-k,n-k);
    end
end
p=CBF_TRAJ_POLY_JET(a,s,H);f=zeros(1,7);jet=zeros(size(j));
for k=0:6,f(k+1)=(-1)^k*prod(8-k:7)*(1-u)^(7-k)/H^k;end
for n=0:6
    for k=0:n,jet(:,n+1)=jet(:,n+1)+nchoosek(n,k)*f(k+1)*p(:,n-k+1);end
end
end
