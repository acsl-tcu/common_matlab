function [h, orderHint, labels] = cbf_actual_input_limits(x,settings)
%CBF_ACTUAL_INPUT_LIMITS 拡張状態T, tauの上下限から安全関数を作る。
% 実入力QPの決定変数が [T^(4);tau^(2)] なので、入力限界は
% そのままQPの箱制約にはできず、高次の状態安全条件が必要。
% orderHint はT:4次, tau:2次で、この積分器単体について成立。
if numel(x)~=29,error('CBF:StateLength','実入力状態29要素を指定してください。');end
T=x(20);tau=x(24:26);u=settings.input;
h=[T-u.thrustMin;u.thrustMax-T; tau-u.torqueMin(:);u.torqueMax(:)-tau];
orderHint=[4;4;2*ones(6,1)];
labels={'thrust_min';'thrust_max';'tau_x_min';'tau_y_min';'tau_z_min'; ...
        'tau_x_max';'tau_y_max';'tau_z_max'};
end
