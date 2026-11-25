function set_mode(app)
Setting = app.initial_setting;
app.time = Setting.time;

if isfield(Setting,"do_calculation")
  app.do_calculation = @(A) Setting.do_calculation(A);
else
  app.agent = Setting.agent;
  app.N = length(Setting.agent);

  app.logger = Setting.logger;
  for i = 1:app.N
    app.agent(i).cha_allocation = app.set_cha_allocation(Setting.agent(i));
  end

  if isfield(Setting,"env"); app.env = Setting.env; end
  if isfield(Setting,"in_prog_func"); app.in_prog = Setting.in_prog_func;     end
  if isfield(Setting,"post_func"); app.post = Setting.post_func;      end
end
app.TimeSlider.Value = 0;
app.TimeSlider.Limits = [Setting.time.ts Setting.time.te];
app.TimeSlider.MajorTicks = Setting.time.ts:(Setting.time.te-Setting.time.ts)/5:Setting.time.te;
if isfield(Setting,"motive"); app.motive = Setting.motive; end
if isfield(Setting,"cha_schedule") && ~Setting.fExp
  app.cha_schedule = Setting.cha_schedule;
else
  app.cha_schedule = {};
end
app.clear_axes;
app.TimeSliderLabel.Text = ["time : " + app.time.t];
end