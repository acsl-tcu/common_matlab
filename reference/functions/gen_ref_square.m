function ref = gen_ref_square(param)
arguments
    param.T = 7.0   % 1辺にかける時間
end

T = param.T;

% 四角形の頂点
p1 = [0;0;1];
p2 = [1;0;1];
p3 = [1;1;1];
p4 = [0;1;1];

% minimum jerk
s = @(tau) 10*tau.^3 - 15*tau.^4 + 6*tau.^5;

ref = @(t) square_ref(t, T, p1, p2, p3, p4, s);

end


function r = square_ref(t, T, p1, p2, p3, p4, s)

if t < T
    % 1辺目
    tau = t/T;
    p = p1 + (p2-p1)*s(tau);

elseif t < 2*T
    % 2辺目
    tau = (t-T)/T;
    p = p2 + (p3-p2)*s(tau);

elseif t < 3*T
    % 3辺目
    tau = (t-2*T)/T;
    p = p3 + (p4-p3)*s(tau);

elseif t < 4*T
    % 4辺目
    tau = (t-3*T)/T;
    p = p4 + (p1-p4)*s(tau);

else
    % 一周したら開始位置で停止
    p = p1;
end

r = [p; 0];

end