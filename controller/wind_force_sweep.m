
function F = wind_force_sweep(t)
% 時間変動する外力外乱 [N]

A = 0.2;           % 外力振幅 [N]
f_start = 0.1;     % 開始周波数 [Hz]
f_end = 2;         % 終了周波数 [Hz]
T_sweep = 40;
t_start = 7;
T_ramp = 2;

F = zeros(3,1);

if t < t_start || t >= t_start + T_sweep
    return;
end

tau = t - t_start;

% 線形周波数スイープ
k = (f_end-f_start)/T_sweep;

phase = 2*pi*(f_start*tau + 0.5*k*tau^2);

% 滑らかな立ち上げ・立ち下げ
if tau < T_ramp

    envelope = 0.5*(1-cos(pi*tau/T_ramp));

elseif tau > T_sweep-T_ramp

    tau_end = T_sweep-tau;
    envelope = 0.5*(1-cos(pi*tau_end/T_ramp));

else

    envelope = 1;

end

% z方向の外力
F(3) = A*envelope*cos(phase);

end
