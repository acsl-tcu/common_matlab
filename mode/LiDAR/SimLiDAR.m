N = 1; % the number of agents
ts = 0;
dt = 0.025;
te = 10;
time = TIME(ts,dt,te,N);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");
motive = Connector_Natnet_sim(dt);              % 3rd arg is a flag for noise (1 : active )

%env = stlread('3F.stl');
  a = 1;
  b = 2;
  c = 3;
  Points = [4 0 0]+[-a -b -c;a -b -c;a b -c; -a b -c;-a -b c;a -b c;a b c; -a b c]; 
  Tri  = [1,4,3;1,3,2;5 6 7;5 7 8;1 5 8;1 8 4;1 2 6;1 6 5; 2 3 7;2 7 6;3 4 8;3 8 7];
  env = triangulation(Tri,Points);

initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_Quat13(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"])));
%agent.sensor.lidar = LiDAR3D_SIM(agent,Sensor_LiDAR3D(1, 'env', env, 'theta_range', pi / 2, 'phi_range', -pi:0.1:pi, 'noise', 3.0E-2, 'seed', 3)); % 2D lidar
agent.sensor.set_function_class("lidar", LiDAR3D_SIM(agent,Sensor_LiDAR3D(1, 'env', env, 'R0', Rodrigues([0,1,0],pi/6),'p0',[1;0;1],'theta_range', pi/2, 'phi_range', 0, 'noise', 0, 'seed', 0)));
agent.sensor.set_function_class("motive", MOTIVE(agent, motive));
agent.sensor.do = @sensor_do;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",5,"center",[0;0;1],"radius",[2,2,0.5]}}));
agent.controller.set_function_class("hlc", HLC(agent,Controller_HL(dt)));

function result = sensor_do(varargin)
sensor = varargin{5}.sensor;
result = sensor.lidar.do(varargin);
result = merge_result(result,sensor.motive.do(varargin));
varargin{5}.sensor.result = result;
end

function in_prog(app)
app.agent.show(["sensor", "lidar"], "ax", app.UIAxes,"k",app.time.k,"logger",app.logger, "param",struct("fLocal", false,"fField",true));
end
function post(app)
app.agent.animation(app.logger,"ax", app.UIAxes,"self",app.agent, "target", 1, "opt_plot", ["sensor", "lidar"], "param",struct("fLocal", false,"fField",true));
show_animation(app);
end
function show_animation(app)
% 協調吊り下げ（複数ドローン＋牽引物）のアニメーションを生成する。
% Nは機体数、牽引物はN+1
if app.logger.k <= 1
    return
end
  mov = DRAW_DRONE_MOTION(app.logger, "self", app.agent, "target", 1,...
          "lims", [ -5 5;  -5 5;  -3 5 ]);
  mov.animation(app.logger, "self", app.agent, "target", 1, "Motive_ref", 1);
end
function v = build_display_vector(agent, time)
% コンソール表示用の文字列を作る。
% 参照位置、推定位置、入力、推定質量を並べる。
idx = 1;
if ~isprop(agent(idx).reference.result.state, "xd")
    v = [];
    return
end
xd = agent(idx).reference.result.state.xd;
if isfield(agent(idx).estimator.result, "state")
    p = agent(idx).estimator.result.state.p;
else
    p = [NaN; NaN; NaN];
end
u = agent(idx).controller.result.input;
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]",agent(idx).cha, time.t, xd(1:3)', p', u');
end
