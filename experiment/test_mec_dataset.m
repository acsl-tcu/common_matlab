% 4入力励振の再現性、予測の時刻対応、学習データの隣接関係を検証する。
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));
setting = mec_dataset_settings();
setting.enabled = true;
before = rng;
profile = make_mec_excitation(7,setting);
assert(isequal(before,rng),'励振の生成で他の乱数列を変更しています。');
assert(isequal(profile,make_mec_excitation(7,setting)));
other_profile = make_mec_excitation(8,setting);
assert(~isequal(profile.phase,other_profile.phase));
sample_times = 0:0.025:setting.flight_time;
inputs = cell2mat(arrayfun(@(t) mec_input_excitation(t,profile),sample_times,'UniformOutput',false));
assert(rank(inputs)==4 && all(max(abs(inputs),[],2)<=setting.amplitude));
assert(all(inputs(:,1)==0) && all(inputs(:,end)==0));
assert(all(mec_input_excitation(-1,profile)==0));

% 推定状態とノミナル入力を同じ時刻から取得していることを確認する。
agent = DRONE;
agent.parameter = DRONE_PARAM("DIATONE");
initial = struct('p',[0;0;1],'q',zeros(3,1),'v',zeros(3,1),'w',zeros(3,1));
model = MODEL_CLASS(agent,Model_EulerAngle(0.025,initial,1));
agent.estimator.result = struct('state',state_copy(model.state));
param = struct('dataset',setting,'trajectory_seed',7);
controller = MECKC(agent,param);
time = TIME(0,0.025,1);
hover_input = [agent.parameter.mass*9.81;0;0;0];
agent.controller.result = struct('input',hover_input);
first = controller.do(time,'f',[],[],agent,1);
assert(~first.e_valid && all(first.e==0));
for sample_index = 1:3
    previous = controller.result;
    previous_x = agent.estimator.result.state.get();
    actual_x = previous_x+time.dt*roll_pitch_yaw_thrust_torque_physical_parameter_model( ...
        previous_x,previous.input_plant,controller.param.P);
    agent.estimator.result.state.set_state(actual_x);
    time.t = time.t+time.dt;
    agent.controller.result.input = hover_input+[0.1*sample_index;0;0;0];
    current = controller.do(time,'f',[],[],agent,1);
    expected_nominal = previous_x+time.dt*roll_pitch_yaw_thrust_torque_physical_parameter_model( ...
        previous_x,previous.input_nominal,controller.param.P);
    assert(norm(current.x_nominal-expected_nominal)<1e-12);
    expected_error = quaternions_all(expected_nominal)-quaternions_all(actual_x);
    assert(norm(current.e-expected_error)<1e-12);
    assert(norm(current.delta_u-(current.input_plant-current.input_nominal))<1e-12);
end

% 励振OFFでも、誤差の計算とノミナル入力は残ることを確認する。
off_param = param;
off_param.dataset.excitation_enabled = false;
off_controller = MECKC(agent,off_param);
off_controller.do(time,'f',[],[],agent,1);
time.t = time.t+time.dt;
off_result = off_controller.do(time,'f',[],[],agent,1);
assert(off_result.e_valid && all(off_result.excitation==0));
assert(all(off_result.delta_u_mec==0) && all(off_result.delta_u==0));
assert(isequal(off_result.input,agent.controller.result.input));
% 通常モードでも、明示指定で補償だけを止められる。
normal_param = param;
normal_param.dataset.enabled = false;
normal_param.use_compensation = false;
normal_controller = MECKC(agent,normal_param);
assert(~normal_controller.collect_dataset && ~normal_controller.use_compensation);
assert(all(normal_controller.gain(:)==0));

% 空回し、時間の飛び、着陸への切替を含む人工ログから正しい組だけを抜く。
logger = LOGGER(1,12,0,[],[]);
logger.k = 12;
logger.Data.t = [0;0;(1:7)'*0.025;0.3;0.325;0.35];
logger.Data.phase = ['f';'f';repmat('f',8,1);'l';'l'];
for sample_index = 1:12
    r = current;
    r.e = (1:26)'*sample_index;
    r.e(16) = 0;
    r.e_valid = sample_index>2;
    r.input_nominal = hover_input;
    r.excitation = mec_input_excitation(sample_index,profile);
    r.delta_u_mec = zeros(4,1);
    r.input_plant = r.input_nominal+r.excitation;
    r.delta_u = r.input_plant-r.input_nominal;
    logger.Data.agent.controller.result{sample_index} = r;
end
metadata = struct('dt',0.025,'error_definition','nominal_input_one_step_v1');
data = build_mec_error_dataset(logger,metadata);
assert(isequal(data.sample_index,3:8));
assert(isequal(size(data.E),[26,6]) && isequal(size(data.dU),[4,6]));
assert(isequal(data.E_next(1,:),data.E(1,:)+1));
assert(norm(data.dU-(data.input_plant-data.input_nominal),'fro')<1e-12);
assert(data.diagnostics.input_rank==4);
fprintf('MECデータ収集：4入力の再現性・時刻対応・連続データ抽出の検証に成功しました。\n');
