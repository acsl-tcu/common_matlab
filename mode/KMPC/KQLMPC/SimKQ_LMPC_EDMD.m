ts = 0; % initial time
dt = 0.025; % sampling period
te = 60; % terminal time
compare_mode0_log_path = '';
assignin('base', 'compare_mode0_log_path', compare_mode0_log_path);
% instance of time class
post_func = @(app) post(app); % function working at the "draw button" pushed.
motive = Connector_Natnet_sim(dt); % imitation of Motive camera (motion capture system)
logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]); % instance of LOOGER class for data logging
initial_state.p = arranged_position([0, 0], 1, 1,0); % [x, y], 1, 1, z
% initial_state.p = arranged_position([0, 0], 1, 1,0.6); % [x, y], 1, 1, z

initial_state.q = [1; 0; 0; 0];
initial_state.v = [0; 0; 0];
initial_state.w = [0; 0; 0];
  % stable_point = [0.4886;0.3456;0.5981;
  %                0.0117;-0.0035;-0.0528;
  %               -0.0150;0.0018;-0.0001;
  %               -0.0141;0.0040;-0.0050]; 
%%
% for j = 1:120
%  fprintf('Initializing... N:%d \n', j);
%  clear logger agent
 time = TIME(ts,dt,te); 
 logger = LOGGER(1, size(ts:dt:te, 2), 0, [],[]);
 agent = DRONE;
agent.plant = MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1));
 % agent.plant.param(1) = 0.73; % ５％増->0.7875, ５％減->0.7125
% agent.plant.param(1) = 1.0;

% agent.plant.param(6) = 0.24; % (1;1;1)P2PでのNNMECを入れたときの限界値
% agent.plant.param(7) = 0.24;
% agent.plant.param(6) = 0.18; % x3
% agent.plant.param(7) = 0.18; % (1;1;1)P2Pでの限界値
% agent.plant.param(6) = 0.15;
% agent.plant.param(7) = 0.15;
% 
% agent.plant.param(6) = 0.12; % x2
% agent.plant.param(7) = 0.12; % x2.5

% agent.plant.param(10:13) = [0.003, 0.003, 0.003, 0.003];
% agent.plant.param(4) = 0.07;
% agent.plant.param(10) = 0.003;
agent.parameter = DRONE_PARAM("DIATONE");
agent.estimator = EKF(agent, Estimator_EKF(agent,dt,MODEL_CLASS(agent,Model_EulerAngle(dt, initial_state, 1)),["p", "q"]));
agent.sensor = MOTIVE(agent, Sensor_Motive(1,0, motive));
% agent.sensor = DIRECT_SENSOR(agent, 0.0);
% traj_index = mod(floor((j-1)/20), 6) + 1;
% switch traj_index
    % case 1       
      % agent.reference = TIME_VARYING_REFERENCE(agent, {"gen_ref_saddle", {"freq",7,"orig",[0;0;0.6],"size",[1,1,0]}, "HL"});
    % case 2       
        % agent.reference = TIME_VARYING_REFERENCE(agent, {"gen_ref_spline", {"point",12,"order",9,"point_dt",5,"ManualSetting",0,"check",1}});
    % case 3  
      % agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_figure8", {"freq",20,"orig",[0 0 0.6],"size",[1 1 0],"phase",0}});
    % case 4       
    %     agent.reference = MPC_POINT_REFERENCE(agent, {struct("f", [0;0;0.6], "g", [0;-1.2;0.6], "h",[-0.5;1;0.6],"j",[1;0;0.6],"k",[1;0;0.3],"z",[0;0;0.6]), 6});
    % case 5  
    %     agent.reference = TIME_VARYING_REFERENCE(agent, {"gen_ref_heart", {"freq",20,"orig",[0 0 0.6],"size",[1 1 0],"phase",-pi/2}});
    % case 6 
    %       agent.reference = TIME_VARYING_REFERENCE(agent, {"gen_ref_saddle", {"freq",7,"orig",[0;0;0.6],"size",[0,0,0]}, "HL"});
% end
        agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_saddle",{"freq",9,"orig",[0;0;0.6],"size",[1,1,0]},"HL"});%{"Case_study_trajectory",{[0,0,0.6]},"HL"});
% agent.reference.time_var= TIME_VARYING_REFERENCE(agent,{"gen_ref_spline",{"point",12,"order",9,"point_dt",5,"ManualSetting",0,"check",1}});
% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd-0.5], "x",center...
                                                                % , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+0.5], "n",center), 7.5}); % P2P
% agent.reference.time_var = MPC_POINT_REFERENCE(agent, {struct("f", [0;0;0.6], "g", [0;-1;0.6], "h",[-1;-1;0.6],"j",[-1;1;0.6],"k",[1;1;0.6],"z",[1;-1;0.6],"v",[-1;-1;0.6]), 6}); % P2P
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_heart", {"freq",20,"orig",[0 0 0.6],"size",[1 1 0],"phase",-pi/2}});
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent, {"gen_ref_star", {"freq", 20, "orig", [0 0 0.6], "size", [1 1 0], "phase", pi/2}});
% 8字

% agent.reference.time_varying = MY_POINT_REFERENCE(agent, {struct("f", center, "g", [1;0;takeoff_zd], "h",center, "j",[0;1;takeoff_zd], "k",center, "z",[0;0;takeoff_zd-0.5], "x",center...
                                                                % , "c",[-1;-1;takeoff_zd], "v",center, "b",[1;-1;takeoff_zd+0.5], "n",center), 7.5}); % P2P
% agent.reference.time_var = TIME_VARYING_REFERENCE(agent,{"gen_ref_ptp", {"freq", 20}});
% agent.reference.time_var = RANDOM_POINT_REFERENCE(agent,{[0;0;0.6],[0;0;0.6],6,5}); % P2P
%2つのコントローラの設定---------------------------------------------------------------------------------------------------
agent.controller.hlc = HLC(agent,Controller_HL(dt));
agent.controller.kqlmpc = KQ_LMPC_EDMD_CONTROLLER(agent,Controller_KQ_LMPC_EDMD(dt,agent)); %最適化手法：QP
run("SimBase");
agent.cha_allocation.reference = "time_var";
agent.cha_allocation.controller = "hlc";
% agent.cha_allocation.f.controller = ["kqlmpc","hlc"];
agent.cha_allocation.f.controller = ["kqlmpc"];
% timeidx = 60/dt;
% for i = 1:timeidx       
%         tic
%         agent(1).sensor.do(time, 'f');
%         agent(1).estimator.do(time, 'f');
%         agent(1).reference.do(time, 'f');
%         agent(1).controller.do(time, 'f');
%         agent(1).plant.do(time, 'f');
%         logger.logging(time, 'f', agent);
%         time.t = time.t + time.dt;
% 
%         all = toc;
% 
% end
%      logger.save(strcat('KQLMPC_', num2str(j)));
%  end
function post(app)
% baseline_path = '';
% try
%     baseline_path = evalin('base', 'compare_mode0_log_path');
% catch
% end
% 
% line_w = 3.0;
% current_run = extract_run_data_from_logger(app.logger, 1);
% baseline_run = [];
% has_baseline = false;
% 
% if ischar(baseline_path) || isstring(baseline_path)
%     baseline_path = char(baseline_path);
%     if ~isempty(strtrim(baseline_path)) && isfile(baseline_path)
%         try
%             [~,~,ext] = fileparts(baseline_path);
%             if strcmpi(ext, '.mat')
%                 tmp = load(baseline_path);
%                 if isfield(tmp, 'log')
%                     baseline_run = extract_run_data_from_log_struct(tmp.log, 1);
%                 else
%                     baseline_logger = LOGGER(baseline_path);
%                     baseline_run = extract_run_data_from_logger(baseline_logger, 1);
%                 end
%             else
%                 baseline_logger = LOGGER(baseline_path);
%                 baseline_run = extract_run_data_from_logger(baseline_logger, 1);
%             end
%             has_baseline = true;
%         catch ME
%             warning('Failed to load baseline log: %s', ME.message);
%         end
%     end
% end
% 
% fig = figure(100); clf(fig)
% tiledlayout(2,2,"TileSpacing","compact","Padding","compact")
% 
% nexttile
% hold on; grid on;
% if has_baseline
%     plot(baseline_run.t, baseline_run.pos_err_norm, 'LineWidth', line_w, ...
%         'Color', [0.20 0.45 0.85], 'DisplayName', 'mode 0');
% end
% plot(current_run.t, current_run.pos_err_norm, 'LineWidth', line_w, ...
%     'Color', [0.85 0.20 0.20], 'DisplayName', sprintf('mode %d', current_run.mode));
% xlim([app.time.ts,app.time.te]);
% xlabel('Time [s]'); ylabel('||p-p_{ref}|| [m]');
% title('Position Error Norm')
% legend('Location','best')
% 
% nexttile
% hold on; grid on;
% if has_baseline
%     plot(baseline_run.t, baseline_run.u(:,1), 'LineWidth', line_w, ...
%         'Color', [0.20 0.45 0.85], 'DisplayName', 'mode 0 U1');
% end
% plot(current_run.t, current_run.u(:,1), 'LineWidth', line_w, ...
%     'Color', [0.85 0.20 0.20], 'DisplayName', sprintf('mode %d U1', current_run.mode));
% xlim([app.time.ts,app.time.te]);
% xlabel('Time [s]'); ylabel('U1 [N]');
% title('Thrust Input U1')
% legend('Location','best')
% 
% nexttile
% hold on; grid on;
% if has_baseline
%     plot(baseline_run.t, baseline_run.vel_err_norm, 'LineWidth', line_w, ...
%         'Color', [0.20 0.45 0.85], 'DisplayName', 'mode 0');
% end
% plot(current_run.t, current_run.vel_err_norm, 'LineWidth', line_w, ...
%     'Color', [0.85 0.20 0.20], 'DisplayName', sprintf('mode %d', current_run.mode));
% xlim([app.time.ts,app.time.te]);
% xlabel('Time [s]'); ylabel('||v-v_{ref}|| [m/s]');
% title('Velocity Error Norm')
% legend('Location','best')
% 
% nexttile
% hold on; grid on; axis equal;
% if has_baseline
%     plot(baseline_run.p(:,1), baseline_run.p(:,2), 'LineWidth', line_w, ...
%         'Color', [0.20 0.45 0.85], 'DisplayName', 'mode 0');
% end
% plot(current_run.ref(:,1), current_run.ref(:,2), '--', 'LineWidth', 2.5, ...
%     'Color', [0.15 0.15 0.15], 'DisplayName', 'reference');
% plot(current_run.p(:,1), current_run.p(:,2), 'LineWidth', line_w, ...
%     'Color', [0.85 0.20 0.20], 'DisplayName', sprintf('mode %d', current_run.mode));
% xlabel('x [m]'); ylabel('y [m]');
% title('XY Trajectory')
% legend('Location','best')
% 
% rmse_current = compute_rmse_metrics(current_run);
% if has_baseline
%     rmse_baseline = compute_rmse_metrics(baseline_run);
% else
%     rmse_baseline = [];
% end
% 
% fig_tbl = figure(101); clf(fig_tbl)
% set(fig_tbl, 'Name', 'RMSE Summary')
% 
% row_names = {'Position RMSE'; 'Velocity RMSE'; 'X RMSE'; 'Y RMSE'; 'Z RMSE'};
% if has_baseline
%     table_data = [
%         rmse_baseline.pos_rmse, rmse_current.pos_rmse;
%         rmse_baseline.vel_rmse, rmse_current.vel_rmse;
%         rmse_baseline.pos_xyz_rmse(1), rmse_current.pos_xyz_rmse(1);
%         rmse_baseline.pos_xyz_rmse(2), rmse_current.pos_xyz_rmse(2);
%         rmse_baseline.pos_xyz_rmse(3), rmse_current.pos_xyz_rmse(3)];
%     col_names = {'mode 0', sprintf('mode %d', current_run.mode)};
% else
%     table_data = [
%         rmse_current.pos_rmse;
%         rmse_current.vel_rmse;
%         rmse_current.pos_xyz_rmse(1);
%         rmse_current.pos_xyz_rmse(2);
%         rmse_current.pos_xyz_rmse(3)];
%     col_names = {sprintf('mode %d', current_run.mode)};
% end
% 
% uitable(fig_tbl, ...
%     'Data', table_data, ...
%     'ColumnName', col_names, ...
%     'RowName', row_names, ...
%     'Units', 'normalized', ...
%     'Position', [0.05 0.08 0.90 0.84], ...
%     'FontSize', 12);q
% 
% disp(array2table(table_data, 'VariableNames', matlab.lang.makeValidName(col_names), 'RowNames', row_names));
% return
% app.logger.plot({1, "p", "er"},"ax",app.UIAxes,"xrange",[app.time.ts,app.time.te],"linewidth", 2.5, ...
%     "fontsize", 14);
% % app.logger.plot({1, "inner_input", ""}, "fig_num", 1,"xrange",[app.time.ts,app.time.te]);
% % app.logger.plot({1, "v", "e"},"ax",app.UIAxes3,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "input", ""},"fig_num", 2,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
%     "fontsize", 14);
% app.logger.plot({1, "v", "er"}, "fig_num", 3,"xrange",[app.time.ts,app.time.te], "linewidth", 2.5, ...
%     "fontsize", 14);
%  app.logger.plot({1, "p1-p2-p3", "er"},"fig_num", 4,"phase",'tfl', "color",0, "linewidth", 2.5, ...
%     "fontsize", 14);
fig = figure(100); clf
tiledlayout(2,2,"TileSpacing","compact","Padding","compact")

%% 1 — p error
nexttile
app.logger.plot({1, "p", "er"}, ...
    "ax", gca, ...
    "xrange",[app.time.ts,app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Position Error")

%% 2 — input
nexttile
app.logger.plot({1, "input", ""}, ...
    "ax", gca, ...
    "xrange",[app.time.ts,app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Control Input")

%% 3 — velocity error
nexttile
app.logger.plot({1, "v", "er"}, ...
    "ax", gca, ...
    "xrange",[app.time.ts,app.time.te], ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Velocity Error")

%% 4 — phase plot
nexttile
app.logger.plot({1, "p1-p2-p3", "er"}, ...
    "ax", gca, ...
    "phase",'tfl', ...
    "color",0, ...
    "linewidth", 2.5, ...
    "fontsize", 14);
title("Phase Plot")
% app.logger.plot({1, "inner_input", ""},"ax",app.UIAxes6,"xrange",[app.time.ts,app.time.te]);
% app.logger.plot({1, "controller.result.delta_u_edmd", ""}, "fig_num", 4,"phase","f");
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
