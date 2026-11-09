function [Q,C]=TRAJECTORY_C9_BERNSTEIN_SUBINTERVAL_CONTROLS(P,u0,u1)
% Exact-in-real-arithmetic restriction of a degree-23 Bernstein curve.
% P: d-by-24 global controls; [u0,u1] a subinterval of [0,1].
% Q=P*C is the SAME physical polynomial on that entire subinterval,
% reparameterized onto [0,1]. C depends only on subinterval endpoints.
% No extrapolation or approximate samples. Floats are not directed-round.
assert(isnumeric(P)&&isreal(P)&&size(P,2)==24&& ...
    all(isfinite(P(:)))&&isnumeric(u0)&&isnumeric(u1)&& ...
    isreal(u0)&&isreal(u1)&&isscalar(u0)&&isscalar(u1)&& ...
    isfinite(u0)&&isfinite(u1)&&0<=u0&&u0<u1&&u1<=1, ...
    'TRAJECTORY_C9_SUBINTERVAL:Domain', ...
    'Require finite 24-control curve with 0<=u0<u1<=1.');
persistent last0 last1 lastC
if ~isempty(lastC)&&isequal(last0,u0)&&isequal(last1,u1)
    C=lastC;
else
    C=eye(24);
    if u0>0
        [~,C]=split_at(C,u0);
    end
    if u1<1
        local=(u1-u0)/(1-u0);
        [C,~]=split_at(C,local);
    end
    last0=u0;last1=u1;lastC=C;
end
Q=double(P)*C;
end

function [left,right]=split_at(B,u)
% de Casteljau on all independent basis vectors at once.
n=size(B,2)-1;
levels=B;left=zeros(size(B));right=zeros(size(B));
left(:,1)=levels(:,1);right(:,end)=levels(:,end);
for k=1:n
    levels=(1-u)*levels(:,1:end-1)+u*levels(:,2:end);
    left(:,k+1)=levels(:,1);
    right(:,end-k)=levels(:,end);
end
end
