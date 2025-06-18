classdef SimExp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        GridLayout                    matlab.ui.container.GridLayout
        LeftPanel                     matlab.ui.container.Panel
        FilenameoptionEditField       matlab.ui.control.EditField
        FilenameoptionEditFieldLabel  matlab.ui.control.Label
        SavedataButton                matlab.ui.control.Button
        DrawgraphButton               matlab.ui.control.Button
        LampLabel                     matlab.ui.control.Label
        Lamp                          matlab.ui.control.Lamp
        StartButton                   matlab.ui.control.Button
        ReloadButton                  matlab.ui.control.Button
        RightPanel                    matlab.ui.container.Panel
        TimeSlider                    matlab.ui.control.Slider
        TimeSliderLabel               matlab.ui.control.Label
        CenterPanel                   matlab.ui.container.Panel
        UIAxes                        matlab.ui.control.UIAxes
        TextArea       matlab.ui.control.TextArea
        TextAreaLabel  matlab.ui.control.Label
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end


    properties (Access = private)
        fStart = 0;
        fInit  = 0;
        post = @(app) [];
        in_prog = @(app) [];
        dummy_class = @(class) struct("do",@(varargin)[],"result",class.result);
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
            app.UIFigure.WindowState = 'maximized';
        end

        % Button pushed function: DrawgraphButton
        function draw_data(app, event)
            app.clear_axes;
            if isempty(app.post)
                app.logger.plot({1:app.N, "p", "er"},{1:app.N, "q", "er"},{1:app.N, "input", ""},"FH",app.UIAxes,"xrange",[app.time.ts,app.time.t]);
            else
                app.post(app);
            end
        end

        % Value changing function: TimeSlider
        function set_current_time(app, event)
            app.time.k = find(app.logger.Data.t>=event.Value,1);
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
            app.TimeSliderLabel.Position = [7,position(4)-22, 50, 22];
            app.TimeSlider.Position = [70,position(4)-12, 284, app.TimeSlider.Position(4)];           
            app.UIAxes.Position = [10, position(4)/3, position(3)*3/5, position(4)*2/3-70];
        end

        % Construct app
        function app = SimExp(varargin)

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % resize 
            set(app.UIFigure,'SizeChangedFcn',@(src, event) app.mySizeChangedFcn(src));

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
    end
end