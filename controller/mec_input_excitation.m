function input = mec_input_excitation(t,profile)
% 飛行開始からの時刻tにおける4軸の入力励振。区間外は0とする。
input = zeros(4,1);
if t<=0 || t>=profile.duration
    return;
end
ramp = min([t,profile.duration-t,profile.ramp_time])/profile.ramp_time;
envelope = 0.5*(1-cos(pi*ramp));
% 本数で割ることで、各軸の絶対値を指定振幅以下に保つ。
input = profile.amplitude.*envelope.*mean( ...
    sin(2*pi*profile.frequency*t+profile.phase),2);
end
