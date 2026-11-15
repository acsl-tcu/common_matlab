function signedProxy=wp_ellipsoid_ellipsoid_margin(cA,RA,aA,cB,RB,aB)
% EXACT intersection sign for two solid ellipsoids (numerical tolerance).
% Solve min_{||u||<=1} ||M u + v||^2 (trust-region subproblem), where
% x=cA+RA*diag(aA)*u, and second ellipsoid is ||RB'*(x-cB)./aB||<=1.
% signedProxy = (sqrt(minObj)-1)*min(aB) has units of metres but is NOT
% an exact Euclidean signed distance. Use only its sign for collision.
aA=aA(:);aB=aB(:);
M=diag(1./aB)*RB'*RA*diag(aA);
v=diag(1./aB)*RB'*(cA(:)-cB(:));
Q=M'*M;b=M'*v;
u=-M\v;
if norm(u)>1
    lamL=0;lamU=max(1,norm(b));
    for k=1:90
        u=-(Q+lamU*eye(3))\b;
        if norm(u)<=1, break;end
        lamU=2*lamU;
    end
    for k=1:60
        lam=(lamL+lamU)/2;
        u=-(Q+lam*eye(3))\b;
        if norm(u)>1,lamL=lam;else,lamU=lam;end
    end
    u=-(Q+lamU*eye(3))\b;
end
f=norm(M*u+v);
signedProxy=(f-1)*min(aB);
end
