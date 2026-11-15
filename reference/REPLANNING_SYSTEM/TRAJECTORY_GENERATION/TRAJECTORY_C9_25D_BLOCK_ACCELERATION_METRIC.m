function M=TRAJECTORY_C9_25D_BLOCK_ACCELERATION_METRIC(acceleration_control_J,T)
%TRAJECTORY_C9_25D_BLOCK_ACCELERATION_METRIC Exact acceleration L2 metric.
% Degree-27 C9 has 8 free Bernstein controls per spatial axis, plus T.
% Their acceleration Jacobian has the SAME 26x8 basis on all three axes;
% only the 25th (duration) column couples the axes. Therefore obtain the
% 25x25 triangular metric factor from ONE flight-independent 26x8 QR and
% three 26-vector projections instead of a fresh 78x25 QR every QP.
%
% With W=diag(Gauss26 weights), B=E25*secondDiff(free27), K=27*26/T^1.5,
% Z_a=K*sqrt(W)*B, zT_a=sqrt(T)*sqrt(W)*E25*dA_a/dT.
% A QR B=Q8*R8 gives U(I_a,I_a)=K*R8,
% U(I_a,25)=Q8'*zT_a, U(25,25)=norm((I-Q8*Q8')*zT,2).
% Thus U'*U=integral_0^T (d acceleration/dx)'(d acceleration/dx)dt.
% No penalty regularization, no fitted physical constant or deadline.
% This is a SEARCH METRIC only, never a collision/HLC proof.
M=struct('valid',false,'status','UNKNOWN_25D_BLOCK_METRIC_NOT_EVALUATED', ...
    'U',NaN(25),'H',NaN(25),'quadrature_nodes',[], ...
    'quadrature_weights',[],'factorization_residual',NaN, ...
    'rank',0,'elapsed_ms',NaN,'method','BLOCK_GAUSS26_QR8');
tic0=tic;
if ~(isnumeric(acceleration_control_J)&&isreal(acceleration_control_J)&& ...
        isequal(size(acceleration_control_J),[3 26 25])&& ...
        all(isfinite(acceleration_control_J(:))) && ...
        isnumeric(T)&&isreal(T)&&isscalar(T)&&isfinite(T)&&T>0)
    M.status='UNKNOWN_25D_BLOCK_METRIC_INVALID_INPUT';
    M.elapsed_ms=1000*toc(tic0);return
end
persistent E nodes weights Q8 R8 template
if isempty(E)
    order=26; k=(1:order-1).';
    off=0.5*k./sqrt(4*k.^2-1);
    Jacobi=diag(0.5*ones(order,1))+diag(off,1)+diag(off,-1);
    [V,D]=eig(Jacobi);
    [nodes,ix]=sort(diag(D));
    weights=(V(1,ix).').^2;
    E=zeros(26,26);
    for i=1:26
        u=nodes(i);
        E(i,1)=(1-u)^25;
        for j=1:25
            E(i,j+1)=E(i,j)*(26-j)*u/(j*(1-u));
        end
    end
    template=zeros(26,8);
    for j=1:8
        bj=zeros(1,28);bj(j+10)=1;
        template(:,j)=diff(bj,2,2).';
    end
    weighted=bsxfun(@times,E*template,sqrt(weights));
    [Q8,R8]=qr(weighted,0);
    R8=triu(R8);
end
% Verify that the tensor is an instance of the exact 25D C9 basis;
% otherwise block factorization is not valid (never guess independence).
scale_acc=27*26/T^2;
for j=1:8
    col=scale_acc*template(:,j);
    for a=1:3
        ix=3*(j-1)+a;
        C=reshape(acceleration_control_J(:,:,ix),3,26);
        if ~isequal(C(a,:).',col) || ...
                any(C(setdiff(1:3,a),:)~=0,'all')
            % The canonical 25D tangent constructs these entries using
            % precisely this T and Bernstein second-difference operation.
            % Reject a different tensor; do not approximate or tune it.
            M.status='UNKNOWN_25D_BLOCK_METRIC_NONCANONICAL_JACOBIAN';
            M.elapsed_ms=1000*toc(tic0);return
        end
    end
end
factor=scale_acc*sqrt(T);
U=zeros(25,25);
weightedT=zeros(26,3);
for axis=1:3
    idx=axis:3:24;
    U(idx,idx)=factor*R8;
    dT=reshape(acceleration_control_J(axis,:,25),26,1);
    zT=sqrt(T)*sqrt(weights).*(E*dT);
    weightedT(:,axis)=zT;
    U(idx,25)=Q8.'*zT;
end
residual=weightedT-Q8*(Q8.'*weightedT);
U(25,25)=norm(residual,'fro');
if ~(all(isfinite(U(:)))&&all(abs(diag(U))>0))
    M.status='UNKNOWN_25D_BLOCK_METRIC_NUMERICALLY_SINGULAR';
    M.elapsed_ms=1000*toc(tic0);return
end
M.U=U;
M.H=U.'*U;
M.rank=25;
M.valid=true;
M.status='EXACT_GAUSS26_BLOCK_QR8_25D_ACCELERATION_METRIC__LOCAL_OBJECTIVE';
M.quadrature_nodes=nodes;
M.quadrature_weights=weights;
% Residual in the ORIGINAL weighted acceleration space, without a 78x25 QR.
Zt=weightedT;
M.factorization_residual=abs(sum(Zt(:).^2)-sum(U(:,25).^2))/ ...
    max(1,sum(Zt(:).^2));
M.elapsed_ms=1000*toc(tic0);
end
