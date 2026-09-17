function Sensor = Sensor_Motive(opts)
arguments
    opts.rigid_id
    opts.initial_yaw_angle
    opts.motive
    opts.state_list
    opts.q_type
    opts.output_func
end
rigid_id = opts.rigid_id;
initial_yaw_angle = opts.initial_yaw_angle;
motive = opts.motive;
state_list = opts.state_list;
q_type = opts.q_type;
output_func = opts.output_func;
%% sensor class demo : constructor
% sensor property をSensor classのインスタンス配列として定義
% rpos : RnagePos_sim
Sensor.Flag = struct('Noise',0,'Occlusion', 0); % '1' : Active, '0' : none
Sensor.ObjFeature=4;
Sensor.LocalX     = [ 0.075, -0.075,  0.015;  -0.075, -0.075, -0.015;-0.075,  0.075,  0.015;0.075,  0.075, -0.015];
Sensor.LPF_T=10;
Sensor.initial_yaw_angle = initial_yaw_angle;
% X, Y. Z
Sensor.rigid_id=rigid_id;
Sensor.motive = motive;
if nargin < 4 || isempty(state_list)
    Sensor.state_list = ["p","q"];
else
    Sensor.state_list = state_list;
end
if nargin < 5 || isempty(q_type)
    Sensor.q_type = "3";
else
    Sensor.q_type = q_type;
end
if nargin >= 6
    Sensor.output_func = output_func;
else
    Sensor.output_func = [];
end
end
