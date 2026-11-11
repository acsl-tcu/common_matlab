function M=TRAJECTORY_C9_25D_EXACT_ACCELERATION_QR(acceleration_control_J,T)
%TRAJECTORY_C9_25D_EXACT_ACCELERATION_QR  Exact-in-real-arithmetic metric.
% A degree-27 load piece has degree-25 acceleration controls. The product
% of two degree-25 acceleration polynomials has degree at most 50.
% 26-point Gauss-Legendre quadrature integrates this polynomial EXACTLY.
% Thus ||sqrt(T)*Q*J||_F^2 = integral_0^T ||d pL''||^2 dt.
% QR directly on the weighted evaluation matrix avoids constructing and
% Cholesky-factorizing a 26x26 Bernstein Gram of condition ~1e14.
% No objective regularizer, penalty, physics guess or tuning weight.
% Only degree-dependent nodes/weights are cached, never Flight parameters.
M=struct('valid',false,'status','UNKNOWN_25D_METRIC_NOT_EVALUATED', ...
    'U',NaN(25),'H',NaN(25),'quadrature_nodes',[], ...
    'quadrature_weights',[],'factorization_residual',NaN, ...
    'rank',0,'elapsed_ms',NaN);
clock=tic;
if ~(isnumeric(acceleration_control_J)&&isreal(acceleration_control_J)&& ...
        isequal(size(acceleration_control_J),[3 26 25])&& ...
        all(isfinite(acceleration_control_J(:)))&&isnumeric(T)&& ...
        isreal(T)&&isscalar(T)&&isfinite(T)&&T>0)
    M.status='UNKNOWN_25D_METRIC_INVALID_JACOBIAN_OR_DURATION';
    M.elapsed_ms=1000*toc(clock);return
end
persistent E nodes weights
if isempty(E)
    % Shift the 26-node Gauss-Legendre Jacobi matrix from [-1,1] to [0,1].
    order=26;k=(1:order-1).';
    off=0.5*k./sqrt(4*k.^2-1);
    J=diag(0.5*ones(order,1))+diag(off,1)+diag(off,-1);
    [V,D]=eig(J);
    [nodes,perm]=sort(diag(D));
    weights=(V(1,perm).').^2;
    % Stable Bernstein evaluation at nodes, without binomial overflow.
    E=zeros(26,26);
    for i=1:order
        u=nodes(i);
        E(i,1)=(1-u)^25;
        for j=1:25
            E(i,j+1)=E(i,j)*(26-j)*u/(j*(1-u));
        end
    end
end
% The single 78x25 weighted Jacobian accounts for all three body axes.
Z=zeros(3*26,25);
scale=sqrt(T)*sqrt(weights(:));
for a=1:3
    Ca=reshape(acceleration_control_J(a,:,:),26,25);
    Z((a-1)*26+(1:26),:)=bsxfun(@times,E*Ca,scale);
end
[~,U]=qr(Z,0);
U=triu(U);
if ~(all(isfinite(U(:)))&&all(abs(diag(U))>0))
    M.status='UNKNOWN_25D_METRIC_RANK_DEFICIENT';
    M.elapsed_ms=1000*toc(clock);return
end
M.valid=true;
M.status='EXACT_GAUSS26_QR_25D_ACCELERATION_METRIC__LOCAL_OBJECTIVE';
M.U=U;
M.H=U.'*U;
M.rank=25;
M.quadrature_nodes=nodes;
M.quadrature_weights=weights;
M.factorization_residual=norm(Z.'*Z-M.H,'fro')/max(1,norm(M.H,'fro'));
M.elapsed_ms=1000*toc(clock);
end
