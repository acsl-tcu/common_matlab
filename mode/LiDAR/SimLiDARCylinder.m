ts = 0;
dt = 0.025;
te = 12;
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 0, [], []);
logger.display_func = @(agent, time) build_display_vector(agent, time);
logger.display_on = true;
fprintf("表示物\nref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n");
motive = Connector_Natnet_sim(dt);              % 3rd arg is a flag for noise (1 : active )

% Cylinder tunnel: diameter 800 mm, axis aligned with x.
tunnelRadius = 0.4;
tunnelLength = 6.0;
tunnelSegments = 96;
env = make_cylinder_tunnel(tunnelRadius, tunnelLength, tunnelSegments);

initial_state.p = [0.5; 0; 0];
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = MODEL_CLASS(agent, Model_Quat13(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator.set_function_class("ekf", EKF(agent, Estimator_EKF(agent, dt, MODEL_CLASS(agent, Model_EulerAngle(dt, initial_state, 1)), ["p", "q"])));
agent.sensor.set_function_class("lidar", LiDAR3D_SIM(agent, Sensor_LiDAR3D(1, ...
  'env', env, ...
  'R0', eye(3), ...
  'p0', [0; 0; 0], ...
  'theta_range', pi / 2 + deg2rad([-15, -13, -11, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9, 11, 13, 15]), ...
  'phi_range', -pi:deg2rad(2):pi, ...
  'radius', 2.0, ...
  'noise', 0.002, ...
  'seed', 0)));
agent.sensor.set_function_class("motive", MOTIVE(agent, motive));
agent.sensor.do = @sensor_do;
agent.reference.set_function_class("timevarying", TIME_VARYING_REFERENCE(agent, {"gen_ref_p2p_back_and_forth", {"p0", [0.5; 0; 0], "p1", [5.0; 0; 0], "t_go", 4.0, "t_hold", 1.0, "t_back", 4.0}, 4}));
agent.controller.set_function_class("hlc", HLC(agent, Controller_HL(dt)));

function result = sensor_do(varargin)
sensor = varargin{5}.sensor;
result = sensor.lidar.do(varargin);
result = merge_result(result, sensor.motive.do(varargin));
varargin{5}.sensor.result = result;
end

function in_prog(app)
app.agent.show(["sensor", "lidar"], "ax", app.UIAxes, "k", app.time.k, "logger", app.logger, "param", struct("fLocal", false, "fField", true));
end

function post(app)
app.agent.animation(app.logger, "ax", app.UIAxes, "self", app.agent, "target", 1, "opt_plot", ["sensor", "lidar"], "param", struct("fLocal", false, "fField", true));
show_animation(app);
end

function show_animation(app)
if app.logger.k <= 1
    return
end
mov = DRAW_DRONE_MOTION(app.logger, "self", app.agent, "target", 1, ...
        "lims", [-0.5 6.5; -0.8 0.8; -0.8 0.8]);
mov.animation(app.logger, "self", app.agent, "target", 1, "Motive_ref", 1);
end

function v = build_display_vector(agent, time)
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
v = sprintf("%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]", agent(idx).cha, time.t, xd(1:3)', p', u');
end

function TR = make_cylinder_tunnel(radius, lengthX, nSeg)
angles = linspace(0, 2 * pi, nSeg + 1);
angles(end) = [];
x0 = 0;
x1 = lengthX;

ring0 = [x0 * ones(nSeg, 1), radius * cos(angles(:)), radius * sin(angles(:))];
ring1 = [x1 * ones(nSeg, 1), radius * cos(angles(:)), radius * sin(angles(:))];
Points = [ring0; ring1];

Tri = zeros(2 * nSeg, 3);
for segIdx = 1:nSeg
    nextIdx = mod(segIdx, nSeg) + 1;
    p00 = segIdx;
    p01 = nextIdx;
    p10 = nSeg + segIdx;
    p11 = nSeg + nextIdx;

    % Wind triangles so face normals point inward to the tunnel axis.
    Tri(2 * segIdx - 1, :) = [p00, p11, p10];
    Tri(2 * segIdx, :) = [p00, p01, p11];
end

TR = triangulation(Tri, Points);
end
