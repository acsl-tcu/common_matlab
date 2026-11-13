function run_auto_trials(app, nTrial, baseName, opts)
% Run reproducible simulation trials without the interactive UI loop.
arguments
    app
    nTrial (1,1) double {mustBeInteger,mustBePositive}
    baseName string = "SimHL"
    opts.firstSeed (1,1) double {mustBeInteger,mustBeNonnegative} = 1
    opts.phaseSeconds (1,5) double {mustBeNonnegative} = [3, 7, 40, 10, 2]
end

if app.fExp
    error('SimExp:AutoTrialsSimulationOnly', 'run_auto_trials is for simulation modes only.');
end

baseName = char(baseName);
phases = ['a', 't', 'f', 'l', 's'];
for trial = 1:nTrial
    seed = opts.firstSeed + trial - 1;
    app.initial_setting.trajectory_seed = seed;
    rng(seed, 'twister');
    app.reset_app();
    fprintf('\n====================================\n');
    fprintf(' Trial %d / %d\n', trial, nTrial);
    fprintf('====================================\n');
    fprintf('Trajectory seed = %d\n', seed);
    fprintf('ref:[px, py, pz]  est:[px, py, pz]  U:[T, tx, ty, tz]\n\n');

    % Match start_app's one-step initialization for each active phase.
    for phase = ['a', 't', 'f', 'l']
        app.cha = phase;
        if ~isempty(app.motive)
            app.motive.getData(app.agent);
        end
        evalc('app.do_calculation();');
        print_trial_state(app);
    end
    app.isReady = true;
    app.fStart = 1;
    app.time.t = app.time.ts;
    app.cha0 = 's';

    % The pasted trial schedule lasts longer than SimHL's manual 50 s limit.
    app.time.te = max(app.time.te, app.time.ts + sum(opts.phaseSeconds(2:end)));
    app.TimeSlider.Limits = [app.time.ts, app.time.te];

    diverged = false;
    diverge_time = NaN;
    for phaseIndex = 1:numel(phases)
        phase = phases(phaseIndex);
        app.cha = phase;
        if phase == 's'
            app.StopProp();
        end
        stepCount = round(opts.phaseSeconds(phaseIndex) / app.time.dt);
        for step = 1:stepCount
            if ~isvalid(app) || ~app.fStart
                return
            end
            if phase ~= 's' && ~isempty(app.motive)
                app.motive.getData(app.agent);
            end
            evalc('app.do_calculation();');
            print_trial_state(app);
            if phase == 'f' || phase == 'l'
                input = app.agent(1).controller.result.input(:);
                if any(~isfinite(input)) || any(abs(input) >= 20)
                    diverged = true;
                    diverge_time = app.time.t;
                    break
                end
            end
            if phase == 'a'
                app.time.t = app.time.ts;
            else
                app.time.t = app.time.t + app.time.dt;
            end
            if mod(step, 20) == 0
                drawnow limitrate
            end
        end
        app.cha0 = phase;
        if diverged
            break
        end
    end

    app.StopProp();
    app.stop_app();
    trialName = baseName;
    if string(app.initial_setting.mode) == "SimHL"
        trialName = simhl_trial_name(app, baseName);
    end
    if diverged
        fileName = sprintf('%s_%03d_seed%d_DIVERGED_t%.3f', trialName, trial, seed, diverge_time);
        fileName = strrep(fileName, '.', 'p');
    else
        fileName = sprintf('%s_%03d_seed%d', trialName, trial, seed);
    end
    app.data_file_name = fileName;
    app.logger.Data.trajectory_seed = seed;
    app.logger.save(fileName);
    fprintf('Saved %s\n', fileName);
end
end

function fileName = simhl_trial_name(app, baseName)
setting = app.initial_setting;
trajectory = erase(string(setting.simhl_trajectory), "gen_ref_");
plantParam = app.agent(1).plant.param;
nominalParam = setting.simhl_nominal_plant_param;
if numel(plantParam) ~= numel(nominalParam)
    error('SimExp:PlantParameterCount', 'SimHL plant and nominal parameter counts differ.');
end

fileName = char(trajectory);
changed = find(abs(plantParam(:) - nominalParam(:)) > 1e-12);
for index = reshape(changed, 1, [])
    value = strrep(sprintf('%.15g', plantParam(index)), '.', 'p');
    value = strrep(value, '-', 'm');
    fileName = sprintf('%s_ME%d_%s', fileName, index, value);
end
fileName = sprintf('%s_%s', fileName, baseName);
end

function print_trial_state(app)
agent = app.agent(1);
reference = agent.reference.result.state.xd(1:3);
position = agent.estimator.result.state.p(1:3);
input = agent.controller.result.input(1:4);
fprintf('%c %.3f : R [%7.3f,%7.3f,%7.3f] : P [%7.3f,%7.3f,%7.3f] : U [%7.3f,%7.3f,%7.3f,%7.3f]\n', ...
    char(app.cha), app.time.t, reference, position, input);
end
