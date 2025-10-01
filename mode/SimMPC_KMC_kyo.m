ts = 0; % initial time
dt = 0.025; % sampling period
te = 50; % terminal time
time = TIME(ts,dt,te); % instance of time class
post_func = @(app) post(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1, 0); % [x, y], 1, 1, z
initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
mmatflag = 0;
 model_file = '2025-10-01_exp_renew_code00_randomppnor';
 % model_file = '2025-07-15_Exp_Kyo_code00_randompp2';
%%
agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.reference.bezier = BEZIER_REFERENCE(agent,{[0,0,0.6]},time);
 % agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",8,"orig",[0;0;0.6],"size",[0,0,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});
agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_spline_kyo",{"point",10,"order",9,"point_dt",5,"ManualSetting",0,"check",1}});
 %2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
 agent.controller.kmpc = MPC_CONTROLLER_KMC_kyo_guiexperiment(agent,Controller_MPC_KMC_kyo(dt,model_file,agent)); %最適化手法：QP

run("SimBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
agent.cha_allocation.f.controller = ["kmpc"];
function post(app)
app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "input", ""},"fig_num", 2,"xrange",[app.time.ts,app.time.te]);
app.logger.plot({1, "v", "er"}, "fig_num", 3,"xrange",[app.time.ts,app.time.te]);
 % app.logger.plot({1, "input", ""},"fig_num", 4,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "controller.result.input_kmpc", ""}, "fig_num", 4);
% app.logger.plot({1, "controller.result.hlc", ""},{1, "controller.result.kmpc", ""},"fig_num", 4,"phase","f");
end
function est =import_vars_from_mfile(mfile)
   
    fid = fopen(mfile, 'r');
    if fid == -1
        error('Cannot open file: %s', mfile);
    end

    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line) && ~isempty(strtrim(line)) && ~startsWith(strtrim(line), '%')
            try
                evalin('base', line); 
            catch ME
                warning('Skipped line: %s\nReason: %s\n', line, ME.message);
            end
        end
    end

    fclose(fid);
    fprintf('Imported variables from %s into workspace.\n', mfile);

     try
        est = evalin('base', 'est');
    catch
        error('Variable ''est'' was not defined in the file.');
    end
end
