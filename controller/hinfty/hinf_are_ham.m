function X = hinf_are_ham(A, R, Q)
% 元: ctow_are_ham   
% ハミルトン行列によるリカッチ方程式  A'X + XA + X R X + Q = 0 の安定化解（なければ []）
%   H = [A R; -Q -A'] の安定固有値 (Re<0) の固有空間 [V1; V2] から X = V2/V1
n = size(A,1);
H = [A, R; -Q, -A']; %ハミルトン行列の作成 
[V, D] = eig(H); %行列の固有値と固有ベクトルを算出
d = diag(D); %固有値を取り出す
tol = 1e-9*max(1, norm(H,1)); 
idx = real(d) < -tol; %実部が負のものを保管
if nnz(idx) ~= n          % 虚軸上の固有値がある / 安定固有値が n 個ない → 解なし
    X = []; return
end
V1 = V(1:n, idx); %Aのサイズで分ける
V2 = V(n+1:end, idx);
if rcond(V1) < 1e-12      % V1 が特異 → 安定化解なし
    X = []; return
end
X = real(V2/V1); %解を作成
X = (X + X')/2; %平均にして対称に
% 検証: 残差が小さく、X が半正定
res = A'*X + X*A + X*R*X + Q; % リカッチ方程式に代入
if norm(res,1) > 1e-6*(1 + norm(X,1)) || min(eig(X)) < -1e-8
    X = [];
end
end