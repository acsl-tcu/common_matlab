% syms w1 w2 w3 phi theta psi v1 v2 v3 p1 p2 p3 ex ey ez real
% w = [w1,w2,w3];
% v = [v1,v2,v3];
% p = [p1,p2,p3];
% q = [phi,theta,psi];
% e = [ex,ey,ez];
% what= [0,-w3,w2;w3,0,-w1;-w2,w1,0];
% Rzyx = [ cos(psi)*cos(theta), ...
%           cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi), ...
%           cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
% 
%           sin(psi)*cos(theta), ...
%           sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), ...
%           sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
% 
%           -sin(theta), ...
%           cos(theta)*sin(phi), ...
%           cos(theta)*cos(phi) ];
% T_inv = [ 1,  sin(phi) * tan(theta),  cos(phi) * tan(theta);
%           0,  cos(phi),              -sin(phi);
%           0,  sin(phi) / cos(theta),  cos(phi) / cos(theta) ];
% 
% qdot = T_inv * w';
% Ib = diag([2.24, 2.99, 4.80]) * 1e-3;
% 
% f =[v';
%     qdot;
%     -9.81*e';
%     -inv(Ib)*what*Ib*w'];
%% 定义符号
syms w1 w2 w3 phi theta psi v1 v2 v3 ex ey ez g real
syms I1 I2 I3 real % 惯性主矩

% 向量定义
w = [w1; w2; w3];        % 体坐标系角速度
v = [v1; v2; v3];        % 体坐标系线速度
q = [phi; theta; psi];   % 欧拉角
e = [ex; ey; ez];        % 重力方向向量（一般取 [0; 0; 1]）

% 斜对称矩阵 (Skew-symmetric)
what = [  0, -w3,  w2;
          w3,  0, -w1;
         -w2,  w1,  0 ];

% 惯性张量
Ib = diag([I1, I2, I3]);

% ZYX 旋转矩阵 (Body -> Inertial)
Rzyx = [ cos(psi)*cos(theta),  cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi),  cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
         sin(psi)*cos(theta),  sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi),  sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
        -sin(theta),           cos(theta)*sin(phi),                               cos(theta)*cos(phi) ];

%% === 四行分块矩阵 F(x) 定义 ===

% 第1行块：体速度
F1 = v;

% 第2行块：欧拉角导数（使用标准转换矩阵）
T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi),           -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
F2 = simplify(T * w);

% 第3行块：重力项（在机体系）
F3 = -g * e;  % 若 e = [0; 0; 1], 即为 [0; 0; -g]

% 第4行块：角动量项（陀螺效应）
F4 = -Ib \ (what * Ib * w);

% 组合成总向量函数
F = [F1; F2; F3; F4];
simplify(F)