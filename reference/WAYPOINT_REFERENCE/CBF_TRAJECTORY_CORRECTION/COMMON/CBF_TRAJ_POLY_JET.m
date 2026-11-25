function jet=CBF_TRAJ_POLY_JET(a,s,H)
% a(:,n+1) coefficient of normalized time (s/H)^n, derivatives in seconds.
jet=zeros(size(a,1),7);z=s/H;N=size(a,2)-1;
for j=0:min(6,N)
    v=zeros(size(a,1),1);
    for n=N:-1:j
        v=v*z+a(:,n+1)*prod(n-j+1:n);
    end
    jet(:,j+1)=v/H^j;
end
end
