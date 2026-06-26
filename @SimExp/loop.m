
function loop(app)
pause(0.1);
if app.fExp % experiment
  app.LampLabel.Text = ["== EXPERIMENT ==","Start! Press key code" + app.cha,"s : stop", "a : arming", "t : take-off", "f : fligh", "l : landing"];
end
  app.TimeSliderLabel.Text = string(round(app.time.t, 1));%["..."];
  if app.isReady
    uiwait(app.UIFigure);
    app.t0 = app.time.ts;
    start(app.update_timer);
  end
  i=1;
  while app.fStart 
    app.tmp(i,end) = app.time.t;
    tStart = tic;
    t2 = tic;
    drawnow
    app.tmp(i,2) = toc(t2);
    if app.cha0 ~= app.cha % exec once per another key press
      switch app.cha
        case {'q',' '}
          app.StopProp;          
          app.LampLabel.Text = "Quit the trial";
          app.Lamp.Color = [0 0 0];
          break;
        case 's'
          app.StopProp;
          app.LampLabel.Text = "Stop propellas";
          app.Lamp.Color = [0 1 0];
        case 'f'
          if strcmp(app.cha0,'t') % "flight" is allowed after "take-off"
            app.LampLabel.Text = "Flight";
          end
        case 'l'
          app.LampLabel.Text = "Landing";
        case 't'
          if strcmp(app.cha0,'a') % "take-off" is allowed after "arming"
            app.LampLabel.Text = "Take-off";
          end
        case 'a'
          app.LampLabel.Text = "Arming";
          app.Lamp.Color = [1 0 0];
      end
      app.cha0 = app.cha;
    end
    t3=tic;
    if ~isempty(app.motive)
      app.motive.getData(app.agent);            
    end
    app.tmp(i,3)=toc(t3);
    app.TimeSlider.Value = app.time.t;
    t4=tic;
    app.do_calculation(i);
    app.tmp(i,4)=toc(t4);
    if app.fExp
        app.time.dt = toc(tStart);
    end
    app.tmp(i,1)=app.time.dt;
    if contains('flts',app.cha)
        app.time.t = app.time.t + app.time.dt;   
    else
        app.time.t = 0;
    end
    if ~app.isReady || app.time.t >= app.time.te
      break
    end
    i=i+1;
  end
app.stop_app;
end
