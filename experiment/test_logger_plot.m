% 文字配列・文字列の指定、階層パス、3次元描画の回帰テスト。
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(repo_root));
agent = DRONE;
state = STATE_CLASS(struct('state_list',["p","q","v","w"], ...
    'num_list',[3,3,3,3]));
state.set_state(struct('p',zeros(3,1),'q',zeros(3,1), ...
    'v',zeros(3,1),'w',zeros(3,1)));
agent.plant = struct('state',state);
agent.sensor.result = struct('state',state_copy(state));
agent.estimator.result = struct('state',state_copy(state));
agent.reference.result = struct('state',state_copy(state));
agent.controller.result = struct('input',zeros(4,1),'delta_u',zeros(4,1), ...
    'z_p_forward',zeros(13,1),'z_n_forward',zeros(13,1));
logger = LOGGER(1,3,0,[],[]);
time = TIME(0,0.025,0.05);
for sample_index = 1:3
    time.t = (sample_index-1)*time.dt;
    agent.estimator.result.state.p = [sample_index;2*sample_index;3*sample_index];
    agent.reference.result.state.p = agent.estimator.result.state.p+10;
    agent.controller.result.input = sample_index*ones(4,1);
    logger.logging(time,'f',agent,[]);
end

fig = figure('Visible','off');
cleanup = onCleanup(@() close(fig));
ax = axes(fig);
logger.plot({1,'p','er'},'ax',ax,'color',0);
lines = findobj(ax,'Type','line');
assert(numel(lines)==6,'Expected three estimated and three reference lines.');
expected_estimate = [1,2,3];
expected_reference = expected_estimate+10;
assert(any(arrayfun(@(line) isequal(line.YData,expected_estimate),lines)));
assert(any(arrayfun(@(line) isequal(line.YData,expected_reference),lines)));

cla(ax);
logger.plot({1,"p","er"},'ax',ax,'color',0);
assert(numel(findobj(ax,'Type','line'))==6);
cla(ax);
logger.plot({1,'controller.result.input',''},'ax',ax,'color',0);
assert(numel(findobj(ax,'Type','line'))==4);
cla(ax);
logger.plot({{1,'controller.result.z_p_forward',''}, ...
    {1,'controller.result.z_n_forward','s'}},'ax',ax,'color',0);
assert(numel(findobj(ax,'Type','line'))==26);
cla(ax);
logger.plot({1,'p1-p2-p3','er'},'ax',ax,'color',0);
assert(numel(findobj(ax,'Type','line'))==2);
fprintf('LOGGER: char/string, result paths, overlays and 3D plots passed.\n');
clear cleanup;
