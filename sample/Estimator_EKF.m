function Estimator = Estimator_EKF(agent,dt,model,output,opts)
% output ：出力のリスト　例 ["p","q"]
% var : 各出力に対するセンサーの観測ノイズ
%% estimator class demo
% estimator property をEstimator classのインスタンス配列として定義
% すべての機体で同一設定
arguments
    agent
    dt
    model
    output = {"p","q"}
    opts.B = []
    opts.P = []
    opts.Q = []
    opts.R = diag([1e-5*ones(1,3), 1e-8*ones(1,3)]);
end
Estimator.model = model;
% p = length(model.state.get(output)); % number of output
%dt = Estimator.model.dt;
n = Estimator.model.dim(1);% 状態数
% 出力方程式の拡張線形化行列(JacobianH)の生成
% output で登録された出力
if class(output)=="function_handle"
    Estimator.JacobianH = output;
else
    Estimator.JacobianH= @(x) [x.p;x.q];
end

Estimator.sensor_param = {"p","q"}; % parameter for sensor_func
Estimator.sensor_func = @(self,param) [self.sensor.result.state.p;self.sensor.result.state.getq(3)]; % function to get sensor value: sometimes some conversion will be done
Estimator.output_func = @(state,param) param*state; % output function
Estimator.output_param = Estimator.JacobianH(0,0); % parameter for output_func

% P, Q, R, B生成
if isempty(opts.P) % 初期共分散行列
    Estimator.P = eye(n);
else
    Estimator.P = opts.P;
end
if isempty(opts.Q)
    Estimator.Q = 1E-3*diag([1E3,1E3,1E3,1E5,1E5,1E5]);%eye(6)*1E3;%*7.058E-5;%diag(ones(n,1))*1e-7;%eye(6)*7.058E-5;%.*[50;50;50;1E04;1E04;1E04];%1.0e-1; % システムノイズ（Modelクラス由来）
else
    Estimator.Q = opts.Q;
end
Estimator.R = opts.R;
if isempty(opts.B)
    Estimator.B = [eye(6)*dt^2;eye(6)*dt]; % システムノイズが加わるチャンネル
else
    Estimator.B = opts.B;
end

% modelによってEKFのパラメータを調整
Estimator.list=output;
end

function mat = zeroone(row,col,idx)
if idx == 0
    mat = zeros(row,col);
elseif row== col
    mat = eye(row);
else
    error("ACSL : invalid size");
end
end
