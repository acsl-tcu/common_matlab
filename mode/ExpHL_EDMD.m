% ExpHL_EDMD
% Real-flight HL / EDMD-MEC data collection entry.
%
% Set these variables in MATLAB before running this file:
%   EXPHL_EDMD_CASE = "nominal";      % "nominal", "mismatch", "training", "compensated"
%   EXPHL_EDMD_REF  = "circle";       % "square", "circle", "figure8", "saddle", "heart", "star"
%
% Recommended collection order:
%   1) nominal      : measured inertia, no excitation, no compensation
%   2) mismatch     : old/large inertia, no excitation, no compensation
%   3) training     : old/large inertia, structured excitation, no compensation
%   4) compensated  : old/large inertia, EDMD-MEC on, no excitation
%
% Optional overrides:
%   EXPHL_EDMD_MODEL_FILE
%   EXPHL_EDMD_SERIAL_PORT
%   EXPHL_EDMD_MOTIVE_IP
%   EXPHL_EDMD_RIGID_ID
%   EXPHL_EDMD_TE
%   EXPHL_EDMD_HOVER
%   EXPHL_EDMD_REF_PARAM
%   EXPHL_EDMD_PARAM_MEASURED
%   EXPHL_EDMD_PARAM_MISMATCH

clc

case_name = lower(get_base_string("EXPHL_EDMD_CASE", "nominal"));
ref_name = lower(get_base_string("EXPHL_EDMD_REF", "circle"));
model_file = get_base_string("EXPHL_EDMD_MODEL_FILE", ...
    "C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\edmd_residual_model_hl_plant.mat");
serial_port = get_base_string("EXPHL_EDMD_SERIAL_PORT", "COM3");
motive_ip = get_base_string("EXPHL_EDMD_MOTIVE_IP", "192.168.100.59");
rigid_id = get_base_double("EXPHL_EDMD_RIGID_ID", 1);
hover = get_base_vector("EXPHL_EDMD_HOVER", [0 0 1.0]);
ref_param = get_base_struct("EXPHL_EDMD_REF_PARAM", struct());

ts = 0;
dt = 0.025;
te = get_base_double("EXPHL_EDMD_TE", 360);
time = TIME(ts, dt, te);
in_prog_func = @(app) in_prog(app);
post_func = @(app) post(app);
logger = LOGGER(1, size(ts:dt:te, 2), 1, [], []);

fprintf("\n===== ExpHL_EDMD configuration =====\n");
fprintf("case      : %s\n", case_name);
fprintf("reference : %s\n", ref_name);
fprintf("serial    : %s\n", serial_port);
fprintf("motive ip : %s\n", motive_ip);
fprintf("rigid id  : %d\n", rigid_id);
fprintf("te        : %.1f s\n", te);

motive = Connector_Natnet(motive_ip);
motive.getData([], []);
rigid_ids = rigid_id;
sstate = motive.result.rigid(rigid_ids);
initial_state.p = sstate.p;
initial_state.q = sstate.q;
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = DRONE_EXP_MODEL(agent, Model_Drone_Exp(dt, initial_state, "serial", serial_port));
agent.parameter = DRONE_PARAM("DIATONE");

param_measured = get_base_struct("EXPHL_EDMD_PARAM_MEASURED", default_measured_param());
param_mismatch = get_base_struct("EXPHL_EDMD_PARAM_MISMATCH", default_mismatch_param(agent.parameter));

switch case_name
    case "nominal"
        apply_param_override(agent.parameter, param_measured);
        controller_param = make_hl_edmd_param(dt, "nominal", false, false, model_file);

    case "mismatch"
        apply_param_override(agent.parameter, param_mismatch);
        controller_param = make_hl_edmd_param(dt, "mismatch", false, false, model_file);

    case "training"
        apply_param_override(agent.parameter, param_mismatch);
        controller_param = make_hl_edmd_param(dt, "training", true, false, model_file);

    case "compensated"
        if ~isfile(model_file)
            error("EDMD model file does not exist: %s", model_file);
        end
        apply_param_override(agent.parameter, param_mismatch);
        controller_param = make_hl_edmd_param(dt, "compensated", false, true, model_file);

    otherwise
        error("Unknown EXPHL_EDMD_CASE: %s", case_name);
end

fprintf("controller mass/J: m=%.5f, J=[%.6g %.6g %.6g]\n", ...
    agent.parameter.mass, agent.parameter.jx, agent.parameter.jy, agent.parameter.jz);
fprintf("excitation enabled: %d\n", controller_param.edmd_disturbance_enable);
fprintf("compensation mode : %d\n", controller_param.residual.mode);
fprintf("compensation on   : %d\n", controller_param.residual.apply_compensation);
fprintf("data rule         : plant actual = estimator/mocap state; nominal = A_nom*z + B_nom*u_nom\n");
fprintf("logged fields     : u_nominal, delta_u_training, delta_u_comp, input_actual, edmd_x_actual, edmd_z_nom_next\n");

agent.estimator = EKF(agent, Estimator_EKF(agent, dt, ...
    MODEL_CLASS(agent, Model_EulerAngle(dt, initial_state, 1)), ["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1, 0, motive));
agent.input_transform = THRUST2THROTTLE_DRONE(agent, InputTransform_Thrust2Throttle_drone());
agent.reference.time_var = make_reference(agent, ref_name, hover, ref_param);

agent.controller.hlc = HLC_EDMD_MEC_EXP(agent, controller_param);
agent.controller.result.input = [0; 0; 0; 0];

run("ExpBase");

agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = "hlc";

function controller_param = make_hl_edmd_param(dt, case_name, excitation_on, compensation_on, model_file)
controller_param = Controller_HL(dt);
controller_param.experiment.case = case_name;
controller_param.explore.enable = double(excitation_on);
controller_param.explore.flight_only = 1;
controller_param.explore.mode = "structured_multisine";
controller_param.explore.start_time = 2.0;
controller_param.explore.ramp_time = 3.0;

% Training excitation: avoid thrust excitation and excite attitude channels.
controller_param.explore.u1_amp = [0.0; 0.0];
controller_param.explore.u1_freq = [0.35; 0.90];
controller_param.explore.u1_phase = [0.20; 1.10];
controller_param.explore.u2_amp = [0.010; 0.005];
controller_param.explore.u2_freq = [0.55; 1.25];
controller_param.explore.u2_phase = [0.40; 1.30];
controller_param.explore.u3_amp = [0.010; 0.005];
controller_param.explore.u3_freq = [0.70; 1.55];
controller_param.explore.u3_phase = [0.90; 0.30];
controller_param.explore.u4_amp = [0.0010; 0.0005];
controller_param.explore.u4_freq = [0.45; 1.05];
controller_param.explore.u4_phase = [0.50; 1.40];
controller_param.explore.du_cap = [0.0; 0.018; 0.018; 0.003];

controller_param.edmd_disturbance_enable = double(excitation_on);
controller_param.residual.mode = 0;
controller_param.residual.model_file = model_file;
controller_param.residual.state_source = "estimator";
controller_param.residual.apply_compensation = 0;
controller_param.residual.alpha = 0.30;
controller_param.residual.beta = 0.20;
controller_param.residual.du_max = [0.0; 0.012; 0.012; 0.003];
controller_param.residual.active_channels = [0; 1; 1; 1];
controller_param.residual.pinv_damping = 1e-1;
controller_param.residual.q_scale = 50.0;
controller_param.residual.r_scale = 0.1;
controller_param.residual.use_reference = 1;
controller_param.residual.max_pos_err = 1.5;
controller_param.residual.max_ang_err = 0.35;
controller_param.residual.max_vel_err = 2.5;
controller_param.residual.max_w_err = 3.0;

if compensation_on
    controller_param.residual.mode = 2;
    controller_param.residual.apply_compensation = 1;
end
end

function ref = make_reference(agent, ref_name, hover, ref_param)
switch ref_name
    case "square"
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_hl_square", { ...
            "hover", hover, ...
            "side", ref_value(ref_param, "side", 1.0), ...
            "period", ref_value(ref_param, "period", 16), ...
            "hold_start", ref_value(ref_param, "hold_start", 2), ...
            "hold_end", ref_value(ref_param, "hold_end", 2)}});
    case "circle"
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_circle", { ...
            "freq", ref_value(ref_param, "period", 12), ...
            "init", hover, ...
            "radius", ref_value(ref_param, "radius", 1.0), ...
            "phase", ref_value(ref_param, "phase", 0)}, "HL"});
    case "demo"
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_hl_demo_multi", { ...
            "hover", hover, ...
            "loops", ref_value(ref_param, "loops", 3), ...
            "square_size", ref_value(ref_param, "square_size", 1.0), ...
            "figure8_size", ref_value(ref_param, "figure8_size", [0.85 1.70 0.0]), ...
            "saddle_size", ref_value(ref_param, "saddle_size", [0.55 0.55 0.30]), ...
            "circle_radius", ref_value(ref_param, "circle_radius", 1.0), ...
            "square_period", ref_value(ref_param, "square_period", 18), ...
            "figure8_period", ref_value(ref_param, "figure8_period", 18), ...
            "saddle_period", ref_value(ref_param, "saddle_period", 18), ...
            "circle_period", ref_value(ref_param, "circle_period", 18), ...
            "hold_start", ref_value(ref_param, "hold_start", 4), ...
            "hold_between", ref_value(ref_param, "hold_between", 3), ...
            "hold_end", ref_value(ref_param, "hold_end", 60), ...
            "transition_time", ref_value(ref_param, "transition_time", 5)}});
    case {"figure8", "lemniscate"}
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_figure8", { ...
            "freq", ref_value(ref_param, "period", 12), ...
            "orig", hover, ...
            "size", ref_value(ref_param, "size", [1 1 0]), ...
            "phase", ref_value(ref_param, "phase", 0)}});
    case "saddle"
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_saddle", { ...
            "freq", ref_value(ref_param, "period", 12), ...
            "orig", hover(:), ...
            "size", ref_value(ref_param, "size", [0.8 0.8 0.25])}, "HL"});
    case "heart"
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_heart", { ...
            "freq", ref_value(ref_param, "period", 12), ...
            "orig", hover, ...
            "size", ref_value(ref_param, "size", [0.8 0.8 0]), ...
            "phase", ref_value(ref_param, "phase", -pi/2)}});
    case "star"
        ref = TIME_VARYING_REFERENCE(agent, {"gen_ref_star", { ...
            "freq", ref_value(ref_param, "period", 12), ...
            "orig", hover, ...
            "size", ref_value(ref_param, "size", [0.8 0.8 0]), ...
            "phase", ref_value(ref_param, "phase", pi/2)}});
    otherwise
        error("Unknown EXPHL_EDMD_REF: %s", ref_name);
end
end

function value = ref_value(ref_param, name, default_value)
field = char(name);
if isstruct(ref_param) && isfield(ref_param, field) && ~isempty(ref_param.(field))
    value = ref_param.(field);
else
    value = default_value;
end
end

function p = default_measured_param()
% Replace these with the latest measured real-airframe values if needed.
p.mass = 0.7478;
p.jx = 1.8e-3;
p.jy = 3.6e-3;
p.jz = 4.5e-3;
end

function p = default_mismatch_param(agent_param)
% Old large-inertia controller setting used as the mismatch condition.
p.mass = agent_param.mass;
p.jx = 0.06;
p.jy = 0.06;
p.jz = 0.06;
end

function apply_param_override(parameter_obj, p)
names = string(fieldnames(p));
keep = ismember(names, ["mass", "jx", "jy", "jz"]);
names = names(keep);
if isempty(names)
    return;
end
v = struct();
for i = 1:numel(names)
    v.(names(i)) = p.(names(i));
end
parameter_obj.set(names, v);
end

function value = get_base_string(name, default_value)
if evalin("base", "exist('" + name + "', 'var')")
    value = string(evalin("base", name));
else
    value = string(default_value);
end
end

function value = get_base_double(name, default_value)
if evalin("base", "exist('" + name + "', 'var')")
    value = double(evalin("base", name));
else
    value = double(default_value);
end
end

function value = get_base_vector(name, default_value)
if evalin("base", "exist('" + name + "', 'var')")
    value = double(evalin("base", name));
else
    value = double(default_value);
end
value = reshape(value, 1, []);
end

function value = get_base_struct(name, default_value)
if evalin("base", "exist('" + name + "', 'var')")
    value = evalin("base", name);
else
    value = default_value;
end
end

function post(app)
app.logger.plot({1, "p", "er"}, "ax", app.UIAxes, "phase", "tfl");
app.logger.plot({1, "q", "e"}, "fig_num", 4, "phase", "tfl");
app.logger.plot({1, "input", ""}, "fig_num", 1, "phase", "tfl");
app.logger.plot({1, "v", "er"}, "fig_num", 2, "phase", "tfl");
app.logger.plot({1, "inner_input", ""}, "fig_num", 3, "phase", "tfl");
app.logger.plot({1, "p1-p2-p3", "er"}, "fig_num", 6, "phase", "tfl", "color", 0);
app.logger.plot({1, "controller.result.hlc", ""}, "fig_num", 5, "phase", "f");
end

function in_prog(app)
app.TextArea.Text = "estimator : " + app.agent(1).estimator.result.state.get();
end
