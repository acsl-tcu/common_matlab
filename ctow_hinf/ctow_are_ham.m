function X = ctow_are_ham(A, Rm, Qm)
%CTOW_ARE_HAM  A'X + XA + X*Rm*X + Qm = 0 の安定化解（Rm は不定でもよい）
%   ハミルトン行列の安定固有空間から求める。解がなければ [] を返す。
n = size(A,1);
H = [A Rm; -Qm -A'];
ev = eig(H);
if min(abs(real(ev))) < 1e-9 * max(1, max(abs(ev)))
    X = []; return;                           % 虚軸上の固有値
end
[U, T] = schur(H, 'real');
[U, T] = ordschur(U, T, 'lhp');
if sum(real(ordeig(T)) < 0) ~= n
    X = []; return;
end
U1 = U(1:n, 1:n); U2 = U(n+1:end, 1:n);
if cond(U1) > 1e12
    X = []; return;
end
X = U2 / U1; X = (X + X')/2;
if min(eig(X)) < -1e-8, X = []; end
end
