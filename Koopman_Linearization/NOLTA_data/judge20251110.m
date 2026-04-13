clear; clc;

% === 修改这里为你的文件名 ===
load('2026-01-30_exp_ob26_code00_randompp', 'est');  %ob25_1 have 1 ob25_2 not have 1

A = est.A;
B = est.B;
C = est.C;

dt = 0.025;
nz = size(A,1);  % 应该是 25
nu = size(B,2);  % 应该是 4
idx1 = 16;       % 常数项 1 的索引（你已确认）

% 1) 精确规范“常数1”维：行 = 单位保持，列 = 偏置注入
A(idx1,:) = 0;             % 常数行清零
A(idx1,idx1) = 1;          % 自己保持 1
B(idx1,:)   = 0;           % 常数维不受输入影响（更干净）
% 其它列 A(:,idx1) 保留（这就是偏置向所有观测的注入），但可做极小截断：
thr = 1e-10; 
A(abs(A) < thr) = 0;       % 清理极小数，避免数值噪声

% 2) 选择矩阵：前12维是基础状态（6位置+6速度）
S = [eye(12), zeros(12,nz-12)];

% 3) 钳制“位置更新”为标准积分器：x_{k+1} = x_k + dt*v_k
A(1:6,1:6)    = eye(6);
A(1:6,7:12)   = dt*eye(6);
A(1:6,13:end) = 0;         % 切断 ϕ -> 位置 的直接耦合（很关键）

% 4) 温和抑制“ϕ -> 速度”的过强耦合，避免噪声放大
A(7:12,13:end) = 0.75 * A(7:12,13:end);  % 可在(0.7~0.9)间调

% 5) 轻度谱收缩（若最大特征值太靠近1）
[U,T] = schur(A,'real'); 
rho = max(abs(eig(T)));
if rho > 0.995
    T = T * (0.995/rho);
    A = U*T/U;
end

Cproj = [];
AB = B;
for i = 0:(nz-1)
    Cproj = [Cproj, S*AB];
    AB = A*AB;
end
rk = rank(Cproj);    % 期望 >= 12

% --- 光谱与阻尼
eigA = eig(A);
rhoA = max(abs(eigA));    % 期望 < 1，越小阻尼越强

% --- 观测块耦合强度（有助于判断 ϕ->速度 是否过强）
Axphi_norm = norm(A(7:12,13:end),'fro');
Bv_norm    = norm(B(7:12,:),'fro');  % 输入对速度的“施力能力”