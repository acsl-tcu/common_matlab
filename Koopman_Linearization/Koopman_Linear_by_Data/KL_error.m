function output = KL_error(X,U,Y,Xnom,Ynom,F,flg)

tic


%%観測量でリフト
N = size(X,2);

for i = 1:N
    Xplift(:,i) = F(X(:,i));
    Xnlift(:,i) = F(Xnom(:,i));

    Yplift(:,i) = F(Y(:,i));
    Ynlift(:,1) = F(Znom(:,i));

end

%% errore calc
dXlift = Xplift - Xnlift;
dYlift = Yplift - Ynlift;
dU = U - Unon;
dX = X - Xnom;

numX = size(dXlift,1);
niimU = size(dU,1);

%%identifi A B
if flg.weight
    Q = blkdiag(flg.weight_Qisobe, eye(numZ-size(flg.weight_Qisobe,1)));

    dXlift = Q*dXlift;
    dYlift = Q*dYlift;

    Omega = [dXlift; dU];

    G = Omega*Omega';
    V = dYlift*Omega';

    M = V*pinv(G);

    output.A = M(:,1:numX);
    output.B = M(:,numX+1:numX+numU);
    output.C = dX*pinv(dXlift);
    output.Q = Q;

else
    Omega = [dXlift; dU];

    G = Omega * Omega';
    V = dYlift * Omega';

    M = V * pinv(G);

    output.A = M(:, 1:numX); % size(.A) = (numX, numX)
    output.B = M(1:numX, numX+1:numX+numU); % size(.B) = (numX, numU)
    output.C = dX*pinv(dXlift); % C: Z->X の厳密な求め方 pinv: Moore-Penrose疑似逆行列  size(.C) = (size(X), numX)
end
toc
