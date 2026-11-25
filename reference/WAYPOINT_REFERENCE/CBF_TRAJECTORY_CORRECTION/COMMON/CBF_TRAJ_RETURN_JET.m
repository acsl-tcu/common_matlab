function jet=CBF_TRAJ_RETURN_JET(j0,s,omega)
% Single stable order-seven analytic RETURN: (D+omega)^7 c=0.
% Exact 0..6 derivatives at s=0; not used to filter CARRY.
if s==0,jet=j0;return;end
z=j0;
a=zeros(3,7);
for n=0:6
    for j=0:n
        a(:,n+1)=a(:,n+1)+z(:,j+1)*omega^(n-j)/(factorial(j)*factorial(n-j));
    end
end
p=CBF_TRAJ_POLY_JET(a,s,1);jet=zeros(3,7);
for j=0:6
    for l=0:j
        jet(:,j+1)=jet(:,j+1)+nchoosek(j,l)*(-omega)^(j-l)*p(:,l+1);
    end
end
jet=exp(-omega*s)*jet;
end
