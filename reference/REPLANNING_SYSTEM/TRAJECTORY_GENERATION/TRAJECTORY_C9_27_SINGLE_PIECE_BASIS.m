function [P,dP_dT,Jfree]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(J0,J1,X,T)
%TRAJECTORY_C9_27_SINGLE_PIECE_BASIS Degree 27, one piece, endpoint C9.
%  J0,J1 : 3x10 physical payload derivatives of orders 0..9.
%  X     : 3x8 free Bernstein controls; T : positive physical duration.
%  P     : 3x28 controls. dP_dT fixes J0,J1,X.
%  Jfree : sparse 84x24 free-control Jacobian.
%
% Preserves UAV position C7 under ideal taut flatness pU=pL-L*q,
% q=(g-pL'')/||g-pL''|| with strictly nonzero w=g-pL''.
% If horizontal UAV velocity is nonzero, yaw=atan2(vUy,vUx) is C6.
% Merely constructing this representation is NOT a yaw/nondegeneracy,
% collision, actuation, tracking, tension, or runtime certificate.
%
% Degree-23 C9 trajectories embed into degree-27 via four exact degree
% elevations. This is SINGLE polynomial (not multiple stages, no CBF/AP).
% The existing 13D/23-degree oracle and HLC proof CANNOT directly take P.
n=27;r=9;
assert(isa(J0,'double')&&isequal(size(J0),[3 r+1])&&isreal(J0)&& ...
    all(isfinite(J0(:)))&&isa(J1,'double')&& ...
    isequal(size(J1),[3 r+1])&&isreal(J1)&&all(isfinite(J1(:)))&& ...
    isa(X,'double')&&isequal(size(X),[3 8])&&isreal(X)&& ...
    all(isfinite(X(:)))&&isa(T,'double')&&isscalar(T)&&isreal(T)&& ...
    isfinite(T)&&T>0,'C9_27:Input', ...
    'Finite 3x10 endpoint jets, 3x8 controls and positive T required.');
persistent A
if isempty(A)
    A=zeros(r+1);
    fall=1;
    for k=0:r
        if k>0,fall=fall*(n-k+1);end
        for j=k:r,A(j+1,k+1)=nchoosek(j,k)/fall;end
    end
end
powers=T.^(0:r);
if any(~isfinite(powers)),error('C9_27:DurationPower','Overflow.');end
M=bsxfun(@times,A,powers);
ME=bsxfun(@times,M,(-1).^(0:r));
P=zeros(3,n+1);
P(:,1:10)=J0*M.';
P(:,19:28)=fliplr(J1*ME.');
P(:,11:18)=X;
if nargout>1
    deriv=[0,(1:r).*T.^(0:r-1)];
    MD=bsxfun(@times,A,deriv);
    dP_dT=zeros(3,n+1);
    dP_dT(:,1:10)=J0*MD.';
    dP_dT(:,19:28)=fliplr(J1*bsxfun(@times,MD,(-1).^(0:r)).');
else
    dP_dT=[];
end
if nargout>2
    rows=zeros(1,24);
    for j=1:8,rows(3*j-2:3*j)=3*(j+9)+(1:3);end
    Jfree=sparse(rows,1:24,ones(1,24),84,24);
else
    Jfree=[];
end
assert(all(isfinite(P(:)))&& ...
    (isempty(dP_dT)||all(isfinite(dP_dT(:)))), ...
    'C9_27:Nonfinite','Nonfinite physical curve/sensitivity.');
end
