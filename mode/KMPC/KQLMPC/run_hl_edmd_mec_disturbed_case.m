function results = run_hl_edmd_mec_disturbed_case(opts)
% run_hl_edmd_mec_disturbed_case
% Validate HLC EDMD-MEC with:
%   1) plant model mismatch (mass / inertia)
%   2) dynamic additive input disturbance deltau
% The disturbance is always kept in the loop and is added after the
% compensated controller output, so the validation remains physically
% consistent with an actuator-side disturbance.

if nargin < 1 || ~isstruct(opts)
    opts = struct();
end

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));
cd(repo_root);

opts = set_default_local(opts, 'dt', 0.025);
opts = set_default_local(opts, 'total_time', 120);
opts = set_default_local(opts, 'takeoff_duration', 6.0);
opts = set_default_local(opts, 'landing_duration', 4.0);
opts = set_default_local(opts, 'arming_steps', 1);
opts = set_default_local(opts, 'silent_output', true);
opts = set_default_local(opts, 'save_log', true);
opts = set_default_local(opts, 'save_separate', false);
opts = set_default_local(opts, 'save_prefix', 'HL_EDMD_DIST');
opts = set_default_local(opts, 'data_dir', fullfile(repo_root, 'Data', 'Sim_data'));
opts = set_default_local(opts, 'model_file', fullfile(script_dir, 'edmd_residual_model_hl_plant.mat'));
opts = set_default_local(opts, 'state_source', 'plant');
opts = set_default_local(opts, 'plant_mass', 0.71);
opts = set_default_local(opts, 'plant_jx', 0.003);
opts = set_default_local(opts, 'plant_jy', 0.006);
opts = set_default_local(opts, 'plant_jz', 0.01);
opts = set_default_local(opts, 'reference_kind', 'spline');
opts = set_default_local(opts, 'reference_seed', 1001);
opts = set_default_local(opts, 'spline_point', 12);
opts = set_default_local(opts, 'spline_point_dt', 5.0);
opts = set_default_local(opts, 'saddle_freq', 9.0);
opts = set_default_local(opts, 'saddle_orig', [0; 0; 0.6]);
opts = set_default_local(opts, 'saddle_size', [1.0, 1.0, 0.0]);
opts = set_default_local(opts, 'figure8_freq', 20.0);
opts = set_default_local(opts, 'figure8_orig', [0, 0, 0.6]);
opts = set_default_local(opts, 'figure8_size', [1.0, 1.0, 0.0]);
opts = set_default_local(opts, 'heart_freq', 20.0);
opts = set_default_local(opts, 'heart_orig', [0, 0, 0.6]);
opts = set_default_local(opts, 'heart_size', [1.0, 1.0, 0.0]);
opts = set_default_local(opts, 'star_freq', 20.0);
opts = set_default_local(opts, 'star_orig', [0, 0, 0.6]);
opts = set_default_local(opts, 'star_size', [1.0, 1.0, 0.0]);

schedule = resolve_schedule(opts.dt, opts.total_time, opts.arming_steps, ...
    opts.takeoff_duration, opts.landing_duration);

dt = opts.dt;
ts = 0;
te = schedule.total_duration;
time = TIME(ts, dt, te);
logger = LOGGER(1, schedule.sample_count, 0, [], []);

initial_state.p = arranged_position([0, 0], 1, 1, 0);
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];

agent = DRONE;
agent.plant = MODEL_CLASS(agent, Model_EulerAngle(dt, initial_state, 1));
agent.plant.param(1) = opts.plant_mass;
agent.plant.param(6) = opts.plant_jx;
agent.plant.param(7) = opts.plant_jy;
agent.plant.param(8) = opts.plant_jz;

agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent, dt, ...
    MODEL_CLASS(agent, Model_EulerAngle(dt, initial_state, 1)), ["p", "q"]));
agent.sensor = DIRECT_SENSOR(agent, 0.0);
agent.reference.time_var = build_reference(agent, opts);

ctl_param = execute_quietly(@() Controller_HL_EDMD_MEC(dt, opts.model_file, opts.state_source, 1), opts.silent_output);
ctl_param.explore.enable = 0;
inner_ctl = HLC_EDMD_MEC(agent, ctl_param);

% Keep the dynamic input disturbance active during compensation.
agent.controller.hlc_edmd_dist = HLC_INPUT_ERROR_WRAPPER(inner_ctl, build_input_error_param(opts));

run("SimBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc_edmd_dist";
agent.cha_allocation.f.controller = ["hlc_edmd_dist"];

run_phase_sequence(agent, time, logger, schedule, opts.silent_output);

results = struct();
results.options = opts;
results.schedule = schedule;
results.time = time;
results.logger = logger;

if opts.save_log
    logger.save(opts.save_prefix, "separate", opts.save_separate);
    results.log_path = find_latest_log(opts.data_dir, opts.save_prefix);
    fprintf('Saved log: %s\n', results.log_path);
else
    results.log_path = '';
end

assignin('base', 'hl_edmd_mec_disturbed_results', results);
end

function cfg = build_input_error_param(opts)
cfg = struct();
cfg.enable = 1;
cfg.flight_only = 1;
cfg.mode = 'structured_multisine';
cfg.start_time = 0.0;
cfg.ramp_time = 1.0;
cfg.u1_amp = [0.35; 0.15];
cfg.u1_freq = [0.35; 0.90];
cfg.u1_phase = [0.20; 1.10];
cfg.u2_amp = [0.010; 0.005];
cfg.u2_freq = [0.55; 1.25];
cfg.u2_phase = [0.40; 1.30];
cfg.u3_amp = [0.010; 0.005];
cfg.u3_freq = [0.70; 1.55];
cfg.u3_phase = [0.90; 0.30];
cfg.u4_amp = [0.0010; 0.0005];
cfg.u4_freq = [0.45; 1.05];
cfg.u4_phase = [0.50; 1.40];
cfg.du_cap = [0.55; 0.018; 0.018; 0.003];
cfg.dt = opts.dt;

if isfield(opts, 'input_error') && isstruct(opts.input_error)
    fns = fieldnames(opts.input_error);
    for i = 1:numel(fns)
        cfg.(fns{i}) = opts.input_error.(fns{i});
    end
end
end

function ref_obj = build_reference(agent, opts)
switch lower(char(opts.reference_kind))
    case 'spline'
        ref_obj = TIME_VARYING_REFERENCE(agent, { ...
            "gen_ref_spline_edmd", ...
            {"point", opts.spline_point, ...
             "order", 9, ...
             "point_dt", opts.spline_point_dt, ...
             "ManualSetting", 0, ...
             "check", 0, ...
             "seed", opts.reference_seed}});
    case 'saddle'
        ref_obj = TIME_VARYING_REFERENCE(agent, { ...
            "gen_ref_saddle", ...
            {"freq", opts.saddle_freq, ...
             "orig", opts.saddle_orig(:)', ...
             "size", opts.saddle_size(:)'}, ...
            "HL"});
    case 'figure8'
        ref_obj = TIME_VARYING_REFERENCE(agent, { ...
            "gen_ref_figure8", ...
            {"freq", opts.figure8_freq, ...
             "orig", opts.figure8_orig(:)', ...
             "size", opts.figure8_size(:)', ...
             "phase", 0}});
    case 'heart'
        ref_obj = TIME_VARYING_REFERENCE(agent, { ...
            "gen_ref_heart", ...
            {"freq", opts.heart_freq, ...
             "orig", opts.heart_orig(:)', ...
             "size", opts.heart_size(:)', ...
             "phase", -pi/2}});
    case 'star'
        ref_obj = TIME_VARYING_REFERENCE(agent, { ...
            "gen_ref_star", ...
            {"freq", opts.star_freq, ...
             "orig", opts.star_orig(:)', ...
             "size", opts.star_size(:)', ...
             "phase", pi/2}});
    case 'p2p'
        wp = struct("f", [0; 0; 0.6], ...
                    "g", [0; -1; 0.6], ...
                    "h", [0; 1; 0.6], ...
                    "j", [1; 0; 0.6], ...
                    "k", [1; 0; 0.3], ...
                    "z", [0; 0; 0.6]);
        ref_obj = MPC_POINT_REFERENCE(agent, {wp, 6});
    otherwise
        error('Unsupported reference_kind: %s', opts.reference_kind);
end
end

function schedule = resolve_schedule(dt, total_time, arming_steps, takeoff_duration, landing_duration)
flight_duration = max(5.0, total_time - takeoff_duration - landing_duration);

schedule = struct();
schedule.arming_steps = max(1, round(arming_steps));
schedule.takeoff_steps = max(1, round(takeoff_duration / dt));
schedule.flight_steps = max(1, round(flight_duration / dt));
schedule.landing_steps = max(1, round(landing_duration / dt));
schedule.takeoff_duration = schedule.takeoff_steps * dt;
schedule.flight_duration = schedule.flight_steps * dt;
schedule.landing_duration = schedule.landing_steps * dt;
schedule.total_duration = schedule.takeoff_duration + schedule.flight_duration + schedule.landing_duration;
schedule.sample_count = schedule.arming_steps + schedule.takeoff_steps + schedule.flight_steps + schedule.landing_steps;
end

function run_phase_sequence(agent, time, logger, schedule, silent_output)
for i = 1:length(agent)
    agent(i).cha_allocation = expand_cha_allocation(agent(i));
end

run_phase_steps(agent, time, logger, 'a', schedule.arming_steps, false, silent_output);
run_phase_steps(agent, time, logger, 't', schedule.takeoff_steps, true, silent_output);
run_phase_steps(agent, time, logger, 'f', schedule.flight_steps, true, silent_output);
run_phase_steps(agent, time, logger, 'l', schedule.landing_steps, true, silent_output);
end

function run_phase_steps(agent, time, logger, phase_char, step_count, advance_time, silent_output)
for k = 1:step_count
    do_calculation_step(agent, time, logger, phase_char, silent_output);
    if advance_time
        time.t = time.t + time.dt;
    end
end
end

function do_calculation_step(agent, time, logger, phase_char, silent_output)
for i = 1:length(agent)
    do_allocated_prop(agent, i, "sensor", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "estimator", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "reference", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "controller", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "input_transform", time, logger, phase_char, silent_output);
    do_allocated_prop(agent, i, "plant", time, logger, phase_char, silent_output);
end
logger.logging(time, phase_char, agent, []);
time.k = logger.k;
end

function do_allocated_prop(agent, idx, prop, time, logger, phase_char, silent_output)
entry = agent(idx).cha_allocation.(phase_char).(prop);
if isempty(entry)
    call_fn = @() agent(idx).(prop).do(time, phase_char, logger, [], agent, idx);
    res = execute_prop_call(call_fn, silent_output);
else
    call_fn = @() agent(idx).(prop).(entry(1)).do(time, phase_char, logger, [], agent, idx);
    res = execute_prop_call(call_fn, silent_output);
    for j = 2:length(entry)
        call_fn = @() agent(idx).(prop).(entry(j)).do(time, phase_char, logger, [], agent, idx);
        res = merge_result(res, execute_prop_call(call_fn, silent_output));
    end
end
agent(idx).(prop).result = res;
end

function res = execute_prop_call(call_fn, silent_output)
if silent_output
    tmp_res = [];
    evalc('tmp_res = call_fn();');
    res = tmp_res;
else
    res = call_fn();
end
end

function out = execute_quietly(call_fn, silent_output)
if silent_output
    tmp_out = [];
    evalc('tmp_out = call_fn();');
    out = tmp_out;
else
    out = call_fn();
end
end

function AL = expand_cha_allocation(agent)
AL = struct();
for cha = ['a', 't', 'f', 'l']
    AL.(cha) = struct();
    for prop = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"]
        AL.(cha).(prop) = [];
    end
end

if isprop(agent, "cha_allocation")
    al = agent.cha_allocation;
    for prop = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"]
        if isfield(al, prop)
            for cha = ['a', 't', 'f', 'l']
                AL.(cha).(prop) = al.(prop);
            end
        end
    end

    for cha = ['a', 't', 'f', 'l']
        if isfield(al, cha)
            for prop = ["sensor", "estimator", "controller", "reference", "input_transform", "plant"]
                if isfield(al.(cha), prop)
                    AL.(cha).(prop) = al.(cha).(prop);
                end
            end
        end
    end
end
end

function log_path = find_latest_log(data_dir, prefix)
pat = sprintf('%s_Log(*.mat', prefix);
files = dir(fullfile(data_dir, pat));
if isempty(files)
    error('No saved log matched prefix: %s', prefix);
end
[~, idx] = max([files.datenum]);
log_path = fullfile(files(idx).folder, files(idx).name);
end

function s = set_default_local(s, key, value)
if ~isfield(s, key) || isempty(s.(key))
    s.(key) = value;
end
end
