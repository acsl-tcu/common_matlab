function gain = sampled_lqr_gain(A,B,Q,R,dt)
% 入力を零次ホールドする連続時間プラントの離散LQRゲイン。
% lqrdがなければ、運動方程式と評価関数を離散化してRiccati反復で解く。
% 計算方法：https://www.mathworks.com/help/control/ref/lqrd.html
arguments
    A (:,:) double {mustBeReal,mustBeFinite}
    B (:,:) double {mustBeReal,mustBeFinite}
    Q (:,:) double {mustBeReal,mustBeFinite}
    R (:,:) double {mustBeReal,mustBeFinite}
    dt (1,1) double {mustBePositive,mustBeFinite}
end

if exist('lqrd','file') ~= 0 && license('test','Control_Toolbox')
    gain = lqrd(A,B,Q,R,dt);
    return;
end

n = size(A,1);
m = size(B,2);
assert(isequal(size(A),[n,n]) && size(B,1)==n ...
    && isequal(size(Q),[n,n]) && isequal(size(R),[m,m]), ...
    'sampled_lqr_gain:InvalidSize','Incompatible dynamics or cost dimensions.');

% u_dot=0を追加し、Van Loanの行列指数で1区間の評価関数を求める。
% 零次ホールドによって生じる、状態と入力の交差項も含める。
dynamics = [A,B;zeros(m,n+m)];
weights = blkdiag(Q,R);
augmented_size = n+m;
transition = expm([-dynamics',weights; ...
    zeros(augmented_size),dynamics]*dt);
flow = transition(augmented_size+1:end,augmented_size+1:end);
cost = flow'*transition(1:augmented_size,augmented_size+1:end);
cost = (cost+cost')/2;
Ad = flow(1:n,1:n);
Bd = flow(1:n,n+1:end);
Qd = cost(1:n,1:n);
Nd = cost(1:n,n+1:end);
Rd = cost(n+1:end,n+1:end);

P = zeros(n);
for iteration = 1:10000
    cross = Ad'*P*Bd+Nd;
    next_P = Qd+Ad'*P*Ad-cross*((Rd+Bd'*P*Bd)\cross');
    next_P = (next_P+next_P')/2;
    converged = norm(next_P-P,'fro') <= 1e-12*max(1,norm(next_P,'fro'));
    P = next_P;
    if converged
        gain = (Rd+Bd'*P*Bd)\(Bd'*P*Ad+Nd');
        assert(all(isfinite(gain(:))) && all(abs(eig(Ad-Bd*gain))<1), ...
            'sampled_lqr_gain:UnstableSolution','LQR solution is not stabilizing.');
        return;
    end
end
error('sampled_lqr_gain:NoConvergence', ...
    'Riccati iteration did not converge for the supplied plant and cost.');
end
