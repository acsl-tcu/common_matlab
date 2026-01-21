function reset_app(app)
% Reset for next trial
app.stop_app();
Setting = app.initial_setting;
if Setting.fExp
    for i = 1:length(app.agent); delete(app.agent(i).plant.connector.serial); end
end
run(Setting.mode);
mode_file = which(Setting.mode);
if ~isempty(mode_file)
    mode_dir = fileparts(mode_file);
    local_template = fullfile(mode_dir, "template");
    if exist(local_template, "dir")
        addpath(local_template, "-begin");
    end
    addpath(mode_dir, "-begin");
end
Setting.agent = agent;
Setting.logger = logger;
Setting.time = time;
if exist("motive","var"); Setting.motive = motive;end
if exist("env","var"); Setting.env = env;end
if exist("in_prog_func","var"); Setting.in_prog_func = in_prog_func;end
if exist("post_func","var"); Setting.post_func = post_func;end

app.initial_setting = Setting;
app.set_mode();
app.time.t = app.time.ts;
app.time.k = 1;
app.isReady = false;
% app.StartButton.Text = "Start";
app.LampLabel.Text = ""; % clear emergency text
end
