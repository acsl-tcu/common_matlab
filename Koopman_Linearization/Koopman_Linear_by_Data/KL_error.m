
function output = KL_error(E,dU,E_next,flg)

tic

%% Error system identification
% e[k+1] = A*e[k] + B*delta_u[k]
%
% E      : e[k]        (26 x N)
% dU     : delta_u[k]  (4 x N)
% E_next : e[k+1]      (26 x N)

%% jigenn kakunin 
N = size(E,2);

assert(size(dU,2) == N, ...
    'EとdUのデータ数が一致しません');

assert(size(E_next,2) == N, ...
    'EとE_nextのデータ数が一致しません');

assert(size(E,1) == size(E_next,1), ...
    '誤差の次元が一致しません');

assert(all(isfinite([E(:);dU(:);E_next(:)])), ...
    'データにNaNまたはInfが含まれています');

numX = size(E,1);
numU = size(dU,1);

%% Identification of A and B
if flg.weight

    % 誤差状態の重み付け
    Q = blkdiag(flg.weight_Qisobe, ...
        eye(numX-size(flg.weight_Qisobe,1)));

    assert(isequal(size(Q),[numX numX]), ...
        'Qのサイズが不正です');

    % 変換後の誤差モデルを同定
    Xw = Q*E;
    Yw = Q*E_next;

    Omega = [Xw; dU];

    M = Yw * pinv(Omega);

    output.A = M(:,1:numX);
    output.B = M(:,numX+1:numX+numU);
    output.Q = Q;

else

    Omega = [E; dU];

    M = E_next * pinv(Omega);

    output.A = M(:,1:numX);
    output.B = M(:,numX+1:numX+numU);

end

%% Identification accuracy
if flg.weight
    Ehat_next = output.A*Xw + output.B*dU;
    residual = Yw-Ehat_next;
else
    Ehat_next = output.A*E + output.B*dU;
    residual = E_next-Ehat_next;
end

output.RMSE = sqrt(mean(residual(:).^2));

%% Output matrix
if flg.weight
    output.C = Q \ eye(numX);
else
    output.C = eye(numX);
end

fprintf('Error model RMSE : %.6g\n',output.RMSE);
fprintf('A size : %d x %d\n',size(output.A));
fprintf('B size : %d x %d\n',size(output.B));

toc

end
