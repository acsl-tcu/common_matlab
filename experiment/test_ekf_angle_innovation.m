% 観測角度が±πをまたいでも、姿勢補正が小さな角度になることを検証する。
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));
dt = 0.025;
angle_offset = 0.01;
for direction = [-1,1]
    agent = DRONE;
    agent.parameter = DRONE_PARAM("DIATONE");
    initial = struct('p',zeros(3,1),'q',[0;0;direction*(pi-angle_offset)], ...
        'v',zeros(3,1),'w',zeros(3,1));
    model = MODEL_CLASS(agent,Model_EulerAngle(dt,initial,1));
    observer = EKF(agent,Estimator_EKF(agent,dt,model,["p","q"]));
    agent.sensor.result = struct('output', ...
        [zeros(5,1);-direction*(pi-angle_offset)]);
    agent.controller.result = struct('input',[agent.parameter.mass*9.81;0;0;0]);
    before = observer.result.state.q(3);
    result = observer.do(struct('t',dt,'dt',dt),'f',[],[],agent,1);
    correction = result.state.q(3)-before;
    assert(abs(correction)<3*angle_offset,'Attitude innovation jumped by 2*pi.');
    assert(direction*correction>angle_offset,'Attitude correction has wrong direction.');
end
fprintf('EKF: measurements crossing +/-pi produce small corrections.\n');
