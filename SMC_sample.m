clear; 
close all; 
clc;

% サンプリング周期と時間ベクトル
dt = 0.01;
t = 0:dt:10;

% プラントダイナミクス
a = 1.0; b = 1.0;
f = @(x) [ x(2);
           a*sin(x(1)) ];
B = [0;
     b];
F = @(x,u) f(x) + B*u;

% 初期状態
x0 = [5.0; 0.0];

% 切換平面パラメータ
p = [1.0; 2.0];

% 切換平面と符号関数
S   = @(x) p'*x;
alpha = 10;
sig = @(Sval) tanh(alpha*Sval);

% SMCゲイン
K = 2.0;

% 状態・入力の保存用変数
xsmc = zeros(2, length(t));
usmc = zeros(1, length(t));
xsmc(:,1) = x0;

% ===== シミュレーションループ =====
for k = 1:length(t)-1
    % 制御入力
    usmc(k) = -(p'*B)\(p'*f(xsmc(:,k))) - K*sig(S(xsmc(:,k)));
    
    % ---- Runge-Kutta法を展開 ----
    k1 = F(xsmc(:,k), usmc(k));
    k2 = F(xsmc(:,k) + k1*dt/2, usmc(k));
    k3 = F(xsmc(:,k) + k2*dt/2, usmc(k));
    k4 = F(xsmc(:,k) + k3*dt, usmc(k));
    xsmc(:,k+1) = xsmc(:,k) + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
end
usmc(end) = -(p'*B)\(p'*f(xsmc(:,end))) - K*sig(S(xsmc(:,end)));

% ===== プロット =====
figure;
subplot(3,1,1);
plot(t, xsmc(1,:));
ylabel("state x1"); grid on; title("time series");

subplot(3,1,2);
plot(t, xsmc(2,:));
ylabel("state x2"); grid on;

subplot(3,1,3);
plot(t, usmc);
xlabel("time"); ylabel("input"); grid on;

figure;
xx = -1:0.1:5;
plot(xx, -p(1)/p(2)*xx, 'k--', 'DisplayName','sliding surface'); hold on;
plot(xsmc(1,:), xsmc(2,:), 'DisplayName','state trajectory');
xlabel("state x1"); ylabel("state x2");
legend; title("state space"); grid on;