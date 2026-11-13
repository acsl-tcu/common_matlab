function collect_hl_error_data(app, outputDir)
% Collect HLC identification logs with and without plant error and excitation.
if app.fExp || string(app.initial_setting.mode) ~= "SimHL"
    error('SimExp:InvalidMode', 'Select SimHL simulation mode.');
end
if ~isfolder(outputDir), mkdir(outputDir); end
cases = [0 0; 0 1; 1 0; 1 1];
for caseIndex = 1:size(cases, 1)
    rng(100 + caseIndex, 'twister');
    app.reset_app();
    plant = app.agent(1).plant;
    if ~cases(caseIndex, 1)
        plant.param = app.initial_setting.simhl_nominal_plant_param;
    end
    hlc = app.agent(1).controller.hlc;
    hlc.use_mec = false;
    hlc.param.explore.enable = logical(cases(caseIndex, 2));
    hlc.param.apply_identification_excitation = logical(cases(caseIndex, 2));
    for phase = ['a', 't', 'f', 'l']
        app.cha = phase;
        app.motive.getData(app.agent);
        evalc('app.do_calculation();');
    end
    app.isReady = true;
    app.fStart = 1;
    app.time.t = app.time.ts;
    app.cha0 = 's';
    phases = ['a', 't', 'f', 'l'];
    seconds = [3, 7, 25, 8];
    for phaseIndex = 1:numel(phases)
        app.cha = phases(phaseIndex);
        for step = 1:round(seconds(phaseIndex) / app.time.dt)
            app.motive.getData(app.agent);
            evalc('app.do_calculation();');
            state = app.agent(1).plant.state;
            if any(~isfinite([state.p(:); state.v(:); state.q(:); state.w(:)])) || norm(state.p) > 20
                error('SimExp:DataDiverged', 'Case %d diverged at t=%.3f.', caseIndex, app.time.t);
            end
            if app.cha ~= 'a', app.time.t = app.time.t + app.time.dt; end
        end
        app.cha0 = app.cha;
    end
    log.Data = app.logger.Data;
    log.collection = struct('model_error', logical(cases(caseIndex, 1)), ...
        'excitation', logical(cases(caseIndex, 2)), 'plant_param', plant.param, ...
        'nominal_param', app.initial_setting.simhl_nominal_plant_param, ...
        'dt', app.time.dt, 'trajectory', app.initial_setting.simhl_trajectory, ...
        'seed', 100 + caseIndex);
    filename = fullfile(outputDir, sprintf('hl_error_me%d_exc%d.mat', cases(caseIndex, 1), cases(caseIndex, 2)));
    save(filename, 'log', '-v7.3');
    fprintf('Saved %s (%d samples)\n', filename, app.logger.k);
end
end
