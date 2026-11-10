function out = wp_compose_scalar(base, inner)
%WP_COMPOSE_SCALAR Derivatives 0..6 of base(inner(t)) (Faa di Bruno).
% base: [s(tau); s'(tau); ... s^(6)(tau)]
% inner: [tau(t); tau'(t); ... tau^(6)(t)]
arguments
    base (7,1) double
    inner (7,1) double
end
out=zeros(7,1); out(1)=base(1);
B=zeros(7,7); B(1,1)=1;
for n=1:6
    for k=1:n
        for j=1:n-k+1
            B(n+1,k+1)=B(n+1,k+1) + ...
                nchoosek(n-1,j-1)*inner(j+1)*B(n-j+1,k);
        end
        out(n+1)=out(n+1)+base(k+1)*B(n+1,k+1);
    end
end
end
