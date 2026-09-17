function Sensor = Sensor_VRigidBody(source_name, rigid_id, output, q_type, output_func, output_param)
%% virtual rigid-body sensor parameter
arguments
    source_name = "motive"
    rigid_id = 1
    output = ["p","q"]
    q_type = "3"
    output_func = []
    output_param = []
end
Sensor.source_name = source_name;
Sensor.rigid_id = rigid_id;
Sensor.output = output;
Sensor.q_type = q_type;
Sensor.output_func = output_func;
Sensor.output_param = output_param;
end
