
classdef SimExp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure matlab.ui.Figure
        GridLayout matlab.ui.container.GridLayout
        LeftPanel matlab.ui.container.Panel
        FilenameoptionEditField matlab.ui.control.EditField
        FilenameoptionEditFieldLabel matlab.ui.control.Label
        SavedataButton matlab.ui.control.Button
        DrawgraphButton matlab.ui.control.Button
        LampLabel matlab.ui.control.Label
        Lamp matlab.ui.control.Lamp
        StartButton matlab.ui.control.Button
        ReloadButton matlab.ui.control.Button
        RightPanel matlab.ui.container.Panel
        TimeSlider matlab.ui.control.Slider
        TimeSliderLabel matlab.ui.control.Label
        CenterPanel matlab.ui.container.Panel
        UIAxes matlab.ui.control.UIAxes
        TextArea matlab.ui.control.TextArea
        TextAreaLabel matlab.ui.control.Label
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    properties (Access = private)
        fStart = 0;
        fInit = 0;
        post = @(app) [];
        in_prog = @(app) [];
        dummy_class = @(class) struct("do", @(varargin)[], "result", class.result);
        isReady = false;
    end

    properties (Access = public)
        data_file_name = [];
        takeoff_ref = [];
        landing_ref = [];
        flight_sensor = [];
        flight_reference = [];
        flight_estimator = [];
        flight_controller = [];
        flight_input_transform = [];
        fDebug
        PInterval
        fExp = 0;
        mode = [];
        logger
        env = [];
        time
        cha = "s";
        cha0 = "s";
        agent = [];
        motive = [];
        initial_setting = [];
        update_timer;
        N = 1;
        t0 = 0;
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Callback function
        function startupFcn(app, Setting)
            % start up function : set modes list and load the first mode
            app.fExp = Setting.fExp;
            app.fDebug = Setting.fDebug;
            app.PInterval = Setting.PInterval;
            app.initial_setting = Setting;
            app.reset_app();
            % appearance
            app.LampLabel.HorizontalAlignment = 'left';
            app.TextArea.Value = {char("Mode: " + Setting.mode)};
            % app.UIFigure.WindowState = 'maximized';
        end

        % Button pushed function: DrawgraphButton
        function draw_data(app, event)
            app.clear_axes;

            if isempty(app.post)
                app.logger.plot({1:app.N, "p", "er"}, {1:app.N, "q", "er"}, {1:app.N, "input", ""}, "FH", app.UIAxes, "xrange", [app.time.ts, app.time.t]);
            else
                app.post(app);
            end

        end

        % Value changing function: TimeSlider
        function set_current_time(app, event)
            app.time.k = find(app.logger.Data.t >= event.Value, 1);

            if isempty(app.time.k)
                app.time.k = app.logger.k;
            end

            app.time.t = app.logger.Data.t(app.time.k);
            app.draw_data;
        end

        % Button pushed function: SavedataButton
        function save_data(app, event)
            app.logger.save(app.data_file_name);
        end

        % Value changed function: FilenameoptionEditField
        function set_file_name(app, event)
            app.data_file_name = app.FilenameoptionEditField.Value;
        end

        % Button pushed function: ReloadButton
        function reload_mode(app, event)
            app.reset_app();
        end

    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'on');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [94 294 1100 494];
            app.UIFigure.Name = 'MATLAB App';

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {210, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create ReloadButton
            app.ReloadButton = uibutton(app.LeftPanel, 'push');
            app.ReloadButton.ButtonPushedFcn = createCallbackFcn(app, @reload_mode, true);
            app.ReloadButton.Position = [7 460 100 23];
            app.ReloadButton.Text = 'Reload';

            % Create StartButton
            app.StartButton = uibutton(app.LeftPanel, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @start_app, true);
            app.StartButton.Position = [7 431 100 23];
            app.StartButton.Text = 'Start';

            % Create Lamp
            app.Lamp = uilamp(app.LeftPanel);
            app.Lamp.Position = [7 393 28 28];

            % Create LampLabel
            app.LampLabel = uilabel(app.LeftPanel);
            app.LampLabel.HorizontalAlignment = 'right';
            app.LampLabel.Position = [37 393 183 28];
            app.LampLabel.Text = 'Lamp';

            % Create DrawgraphButton
            app.DrawgraphButton = uibutton(app.LeftPanel, 'push');
            app.DrawgraphButton.ButtonPushedFcn = createCallbackFcn(app, @draw_data, true);
            app.DrawgraphButton.Position = [7 350 100 23];
            app.DrawgraphButton.Text = 'Draw graph';

            % Create SavedataButton
            app.SavedataButton = uibutton(app.LeftPanel, 'push');
            app.SavedataButton.ButtonPushedFcn = createCallbackFcn(app, @save_data, true);
            app.SavedataButton.Position = [7 315 100 23];
            app.SavedataButton.Text = 'Save data';

            % Create FilenameoptionEditFieldLabel
            app.FilenameoptionEditFieldLabel = uilabel(app.LeftPanel);
            app.FilenameoptionEditFieldLabel.HorizontalAlignment = 'right';
            app.FilenameoptionEditFieldLabel.Position = [7 280 102 22];
            app.FilenameoptionEditFieldLabel.Text = 'File name (option)';

            % Create FilenameoptionEditField
            app.FilenameoptionEditField = uieditfield(app.LeftPanel, 'text');
            app.FilenameoptionEditField.ValueChangedFcn = createCallbackFcn(app, @set_file_name, true);
            app.FilenameoptionEditField.Position = [7 258 193 22];

            % Create TextAreaLabel
            app.TextAreaLabel = uilabel(app.LeftPanel);
            app.TextAreaLabel.HorizontalAlignment = 'right';
            app.TextAreaLabel.Position = [7 230 55 22];
            app.TextAreaLabel.Text = 'Static Info';

            % Create TextArea
            app.TextArea = uitextarea(app.LeftPanel);
            app.TextArea.Position = [7 150 193 80];

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;
            app.RightPanel.AutoResizeChildren = 'off';

            % Create UIAxes
            app.UIAxes = uiaxes(app.RightPanel);
            title(app.UIAxes, 'Title')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')

            % Create TimeSliderLabel
            app.TimeSliderLabel = uilabel(app.RightPanel);
            app.TimeSliderLabel.HorizontalAlignment = 'right';
            app.TimeSliderLabel.Text = 'Time';

            % Create TimeSlider
            app.TimeSlider = uislider(app.RightPanel);
            app.TimeSlider.ValueChangingFcn = createCallbackFcn(app, @set_current_time, true);

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
            % disp(app.RightPanel.Position);
            % pause(1);
            % disp(app.RightPanel.Position);
            % app.TimeSliderLabel.Position = [7 454 31 22];
            % app.TimeSlider.Position = [7 1 app.RightPanel.Position(3) app.RightPanel.Position(4)-44];
            % app.UIAxes.Position = [10, app.RightPanel.Position(4)/3, app.RightPanel.Position(3)*3/5, app.RightPanel.Position(4)*3/5];
        end

    end

    % App creation and deletion
    methods (Access = public)

        function mySizeChangedFcn(app, ~) % ウインドウサイズ変更イベントハンドラ関数
            pause(0.5);
            position = app.RightPanel.Position; % [Left, Bottom, Width, Height];
            app.TimeSliderLabel.Position = [7, position(4) - 22, 50, 22];
            app.TimeSlider.Position = [70, position(4) - 12, 284, app.TimeSlider.Position(4)];
            app.UIAxes.Position = [10, position(4) / 3, position(3) * 3/5, position(4) * 2/3 - 70];
        end

        % Construct app
        function app = SimExp(varargin)

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % resize
            set(app.UIFigure, 'SizeChangedFcn', @(src, event) app.mySizeChangedFcn(src));

            % Execute the startup function
            runStartupFcn(app, @(app)startupFcn(app, varargin{:}))

            if nargout == 0
                clear app
            end

        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
        function run_auto_trials(app, nTrial, baseName)

            arguments
                app
                nTrial (1,1) double {mustBeInteger,mustBePositive}
                baseName string = "SimMEC"
            end

            for trial = 1:nTrial

                % 発散判定用フラグ
                diverged = false;
                diverge_time = NaN;
                diverge_input = [];

                fprintf('\n');
                fprintf('====================================\n');
                fprintf(' Trial %d / %d\n', trial, nTrial);
                fprintf('====================================\n');

                % =============================================================
                % ★ 今回のtrialで使うseedを決定
                % =============================================================
                seed = trial;

                % ★ resetする「前」に設定へ渡す
                app.initial_setting.trajectory_seed = seed;

                fprintf('Trajectory seed = %d\n', seed);

                % =============================================================
                % ★ このseedを使ってシミュレーション全体を再構築
                % =============================================================
                app.reset_app();

                % =============================================================
                % Startを押した直後の初期化処理
                % =============================================================
                for cha = ['a','t','f','l']
                    app.fStart = 1;
                    app.cha = cha;
                    app.loop();
                end

                app.isReady = true;
                app.fStart = 1;

                app.time.t = app.time.ts;
                app.cha0 = "s";

                    %% ============================================================
                % Startを押した直後に行われる初期化処理
                %
                % start_app.m では isReady=false の状態で
                % a,t,f,l を1回ずつ loop() に通している。
                % ============================================================



                %% ============================================================
                % a : Arming 3秒
                %
                % 元のloopと同じく、a中はtime.tを進めない。
                % ただし3秒/dt回だけ計算する。
                % =============================================================
                app.cha = 'a';

                N_a = round(3 / app.time.dt);

                for k = 1:N_a

                    app.do_calculation();

                    % 元のloop.mと同じ仕様
                    app.time.t = app.time.ts;

                end

                app.cha0 = 'a';

                fprintf('a finished : t = %.3f\n', app.time.t);


                %% ============================================================
                % t : Take-off 7秒
                % =============================================================
                app.cha = 't';

                N_t = round(7 / app.time.dt);

                for k = 1:N_t

                    app.do_calculation();

                    app.time.t = app.time.t + app.time.dt;

                end

                app.cha0 = 't';

                fprintf('t finished : t = %.3f\n', app.time.t);


                %% ============================================================
                % f : Flight 40秒
                % =============================================================
                app.cha = 'f';

                N_f = round(40 / app.time.dt);

                for k = 1:N_f

                    % この時刻の制御・プラント計算
                    app.do_calculation();

                    % if k == 1
                    %     fprintf("\n===== Flight first step =====\n");
                    %     fprintf("time = %.6f\n", app.time.t);
                    %
                    %     disp("reference.result.state:")
                    %     disp(app.agent(1).reference.result.state)
                    %
                    %     disp("controller input:")
                    %     disp(app.agent(1).controller.result.input)
                    %
                    %     % keyboard   % ★ここで一時停止
                    % end

                    % 発散判定
                    if any(abs(app.agent(1).controller.result.input(:)) >= 20)

                        diverged = true;
                        diverge_time = app.time.t;
                        diverge_input = app.agent(1).controller.result.input;

                        fprintf('\n*** DIVERGENCE DETECTED ***\n');
                        fprintf('Trial = %d\n', trial);
                        fprintf('Time  = %.3f s\n', diverge_time);
                        fprintf('Input = ');
                        fprintf('%.6f ', diverge_input);
                        fprintf('\n');

                        break;   % ← fのforループを抜ける
                    end

                    % 問題なければ次の25 msへ
                    app.time.t = app.time.t + app.time.dt;

                end


                % =============================================================
                % ★ fで発散していたら、この試行をここで完全終了
                % =============================================================
                if diverged

                    % まずプラントを停止
                    app.StopProp();

                    % SimExp側も停止状態にする
                    app.stop_app();

                    % 発散結果として保存
                    filename = sprintf( ...
                        '%s_%03d_DIVERGED_t%.3f', ...
                        baseName, trial, diverge_time);

                    % 小数点をファイル名用に変換
                    filename = strrep(filename, '.', 'p');

                    app.data_file_name = filename;
                    app.logger.save(app.data_file_name);

                    fprintf('Diverged data saved : %s\n', filename);

                    % ★ l以降を全部飛ばして、次のtrialへ
                    continue;

                end


                % =============================================================
                % ここから下は正常な場合だけ来る
                % =============================================================

                app.cha0 = 'f';

                %% l : Landing 10秒
                app.cha = 'l';

                ...
                    app.cha0 = 'f';

                fprintf('f finished : t = %.3f\n', app.time.t);


                %% ============================================================
                % l : Landing 10秒
                % =============================================================

                app.cha = 'l';

                N_l = round(10 / app.time.dt);

                for k = 1:N_l

                    % この時刻の制御・プラント計算
                    app.do_calculation();

                    % ==========================================
                    % ★ Landing中も発散判定
                    % ==========================================
                    if any(abs(app.agent(1).controller.result.input(:)) >= 20)

                        diverged = true;
                        diverge_time = app.time.t;
                        diverge_input = app.agent(1).controller.result.input;

                        fprintf('\n*** DIVERGENCE DETECTED ***\n');
                        fprintf('Trial = %d\n', trial);
                        fprintf('Phase = Landing\n');
                        fprintf('Time  = %.3f s\n', diverge_time);
                        fprintf('Input = ');
                        fprintf('%.6f ', diverge_input);
                        fprintf('\n');

                        break;
                    end

                    % 正常なら時間を進める
                    app.time.t = app.time.t + app.time.dt;
                end


                %% ============================================================
                % ★ Landing中に発散していた場合
                % ============================================================

                if diverged

                    % プラント停止
                    app.StopProp();

                    % SimExp側も停止
                    app.stop_app();

                    % 発散結果として保存
                    filename = sprintf( ...
                        '%s_%03d_DIVERGED_LANDING_t%.3f', ...
                        baseName, trial, diverge_time);

                    filename = strrep(filename, '.', 'p');

                    app.data_file_name = filename;
                    app.logger.save(app.data_file_name);

                    fprintf('Diverged data saved : %s\n', filename);

                    % ★ s,qなどには行かず次のtrialへ
                    continue;
                end


                % Landingが正常終了したときだけここへ来る
                app.cha0 = 'l';

                fprintf('l finished : t = %.3f\n', app.time.t);


                %% ============================================================
                % s : Stop
                % =============================================================
                app.cha = 's';

                % 通常loopと同じくpropeller停止
                app.StopProp();

                N_s = round(2 / app.time.dt);

                for k = 1:N_s

                    app.do_calculation();

                    app.time.t = app.time.t + app.time.dt;

                end

                app.cha0 = 's';

                fprintf('s finished : t = %.3f\n', app.time.t);


                %% ============================================================
                % q : Quit
                % =============================================================
                app.cha = 'q';

                app.StopProp();

                % 通常終了処理
                app.stop_app();

                fprintf('q : trial finished\n');


                %% ============================================================
                % 保存
                % =============================================================

                filename = sprintf('%s_%03d', baseName, trial);
                app.data_file_name = filename;

                % Save data ボタンのコールバックをそのまま実行
                app.save_data([]);

                fprintf('saved : %s\n', filename);

            end

            fprintf('\n');
            fprintf('====================================\n');
            fprintf(' All %d trials finished\n', nTrial);
            fprintf('====================================\n');

        end
    end

end
