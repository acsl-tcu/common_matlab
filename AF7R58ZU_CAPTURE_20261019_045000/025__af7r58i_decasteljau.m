function y = af7r58i_decasteljau(C,u)
C=double(C);
u=double(u);
assert(ismatrix(C)&&size(C,2)>=1&&isscalar(u)&&isfinite(u));
while size(C,2)>1
    C=(1-u)*C(:,1:end-1)+u*C(:,2:end);
end
y=C(:,1);
end