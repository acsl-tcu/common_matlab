function z = quaternions_all(x)
% F(x)およびG(x)の全ての項を観測量とする
%   Z = quartanionParameter(X)
%   X : [位置P; クォータニオンq or オイラー角 Q; 速度V; 角速度W]を持つ状態量
%   Z : [X(クォータニオンを含まない); (クォータニオン); (クォータニオンの2乗); (クォータニオンの3乗) ]

% 状態がクォータニオンを用いた13次元の場合
%　xがq;p;v;wの順番の場合
if size(x,1) == 13
    q = x(1:4);                           % クォータニオン
    eul = quat2eul(q');                   % MATLABの関数（ZYX順）
    Q1 = eul(3); Q2 = eul(2); Q3 = eul(1);% roll, pitch, yaw
    P1 = x(5,1);
    P2 = x(6,1);
    P3 = x(7,1);
    V1 = x(8,1);
    V2 = x(9,1);
    V3 = x(10,1);
    W1 = x(11,1);
    W2 = x(12,1);
    W3 = x(13,1);
    u1 = 0;
    u2 = 0;
    u3 = 0;
    u4 = 0;
% 状態がオイラー角を用いた12次元の場合
elseif size(x,1) == 12 || size(x,1) == 16
    P1 = x(1,1);
    P2 = x(2,1);
    P3 = x(3,1);
    Q1 = x(4,1); % roll
    Q2 = x(5,1); % pitch
    Q3 = x(6,1); % yaw
    V1 = x(7,1);
    V2 = x(8,1);
    V3 = x(9,1);
    W1 = x(10,1);
    W2 = x(11,1);
    W3 = x(12,1);
    u1 = 0; %x(13,1);
    u2 = 0; %x(14,1);
    u3 = 0; %x(15,1);
    u4 = 0; %x(16,1);
end

    % q0-q3 : 与えたオイラー角から求めたクォータニオン
    % eul2quat,quaternion はsingleかdouble型にしか使え無くて関数ハンドルを設定した時にエラーをはいた 残念
    % q0 = cos(Q1/2)*cos(Q2/2)*cos(Q3/2)+sin(Q1/2)*sin(Q2/2)*sin(Q3/2);
    % q1 = sin(Q1/2)*cos(Q2/2)*cos(Q3/2)-cos(Q1/2)*sin(Q2/2)*sin(Q3/2);
    % q2 = cos(Q1/2)*sin(Q2/2)*cos(Q3/2)+sin(Q1/2)*cos(Q2/2)*sin(Q3/2);
    % q3 = cos(Q1/2)*cos(Q2/2)*sin(Q3/2)-sin(Q1/2)*sin(Q2/2)*cos(Q3/2);
% end

%回転行列の一部(修正前)
% R13 = ( 2.*(cos(Q2/2).*cos(Q1/2).*cos(Q3/2) + sin(Q2/2).*sin(Q1/2).*sin(Q3/2)).*(cos(Q1/2).*cos(Q3/2).*sin(Q2/2) + cos(Q2/2).*sin(Q1/2).*sin(Q3/2)) + 2.*(cos(Q2/2).*cos(Q1/2).*sin(Q3/2) - cos(Q3/2).*sin(Q2/2).*sin(Q1/2)).*(cos(Q2/2).*cos(Q3/2).*sin(Q1/2) - cos(Q1/2).*sin(Q2/2).*sin(Q3/2)));
% R23 = (-2.*(cos(Q2/2).*cos(Q1/2).*cos(Q3/2) + sin(Q2/2).*sin(Q1/2).*sin(Q3/2)).*(cos(Q2/2).*cos(Q3/2).*sin(Q1/2) - cos(Q1/2).*sin(Q2/2).*sin(Q3/2)) - 2.*(cos(Q1/2).*cos(Q3/2).*sin(Q2/2) + cos(Q2/2).*sin(Q1/2).*sin(Q3/2)).*(cos(Q2/2).*cos(Q1/2).*sin(Q3/2) - cos(Q3/2).*sin(Q2/2).*sin(Q1/2)));
% R33 = (cos(Q2).*cos(Q1));

%回転行列の一部(クォータニオンから算出)
% R13 = 2.*(q1.*q3 + q0.*q2);
% R23 = 2.*(q2.*q3 - q0.*q1);
% R33 = 1 - 2.*(q1.^2 + q2.^2);

%回転行列の一部(オイラー角から算出)
R13 = sin(Q1)*sin(Q3)+cos(Q1)*sin(Q2)*cos(Q3);
R23 = -sin(Q1)*cos(Q3)+cos(Q1)*sin(Q2)*sin(Q3);
R33 = cos(Q1)*cos(Q2);

R13_l = sin(Q1)*sin(Q3);
R13_r = cos(Q1)*sin(Q2)*cos(Q3);
R23_l = sin(Q1)*cos(Q3);
R23_r = cos(Q1)*sin(Q2)*sin(Q3);

% common_z = [P1;
%             P2;
%             P3;
%             Q1;
%             Q2;
%             Q3;
%             V1;
%             V2;
%             V3;
%             W1;
%             W2;
%             W3;
%             R13_l;
%             R13_r;
%             R23_l;
%             R23_r;
%             R33;
%             1];

common_z = [P1;
            P2;
            P3;
            Q1;
            Q2;
            Q3;
            V1;
            V2;
            V3;
            W1;
            W2;
            W3;
            R13;
            R23;
            R33;
            % ];
            1];

%% 磯部先輩観測量 code = 00
isobe_z = [W1*W2;
            W2*W3;
            W3*W1;
            W2*cos(Q1);
            W3*sin(Q1);
            W1*cos(Q2)/cos(Q1);
            W2*sin(Q1)/cos(Q2);
            W3*cos(Q1)/cos(Q2);
            W2*sin(Q1)*sin(Q2)/cos(Q1);
            W3*cos(Q1)*sin(Q2)/cos(Q1) %26番目
            ];

%% KMEC観測量（テイラー展開の微分項）
%G(x)に出てくるRの微分項
Rdot  =[sin(Q2);
        cos(Q2);%sin,cos
        sin(Q1)*sin(Q2);
        sin(Q2)*sin(Q3);
        sin(Q3)*sin(Q1);%ssの組
        cos(Q1)*cos(Q2);
        cos(Q2)*cos(Q3);
        cos(Q3)*cos(Q1);%ccの組
        sin(Q1)*cos(Q2);%35番目
        sin(Q1)*cos(Q3);
        sin(Q2)*cos(Q1);
        sin(Q2)*cos(Q3);
        sin(Q3)*cos(Q1);
        sin(Q3)*cos(Q2);%scの組40番目
        sin(Q1)*sin(Q2)*sin(Q3);%sssの組
        sin(Q1)*sin(Q2)*cos(Q3);
        sin(Q1)*sin(Q3)*cos(Q2);
        sin(Q2)*sin(Q3)*cos(Q1);%sscの組
        sin(Q1)*cos(Q2)*cos(Q3);
        sin(Q2)*cos(Q1)*cos(Q3);
        sin(Q3)*cos(Q1)*cos(Q2);%sccの組
        cos(Q1)*cos(Q2)*cos(Q3)%cccの組
        ];
%F(x)に出てくる微分項
qdotdot=[    %qdotの一階微分の項
             cos(Q1)*tan(Q2);
             sin(Q1)*tan(Q2);
             W2*sin(Q1);
             W3*cos(Q1);
             cos(Q1);
             sin(Q1);
             W2*cos(Q1)*tan(Q2);
             W3*sin(Q1)*tan(Q2);
             cos(Q1)/cos(Q2);
             sin(Q1)/cos(Q2);
             W3*sin(Q1)/cos(Q2);
             W2*cos(Q1)/cos(Q2);
             %qdotの二階微分の項
             W2*sin(Q2)*tan(Q1);
             W3*cos(Q2)*tan(Q1);
             W3*cos(Q1)/cos(Q2);
             W2*sin(Q1)/cos(Q2);


             
             ];

%% まとめ
z = [common_z; isobe_z]; % 00
% z = [common_z; isobe_z;Rdot;qdotdot]; % 01

end

