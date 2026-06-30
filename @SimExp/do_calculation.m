function do_calculation(app)
% profile on
tStartTotal = tic;
for i = 1:app.N
    app.agent(i).cha = app.cha;
    app.do_prop(app.agent(i),"sensor",i);
    app.do_prop(app.agent(i),"estimator",i);
    app.do_prop(app.agent(i),"reference",i);
    app.do_prop(app.agent(i),"controller",i);
    app.do_prop(app.agent(i),"input_transform",i);
    app.do_prop(app.agent(i),"plant",i);

    app.agent(i).controller.result.controllertime_total = toc(tStartTotal);
end
app.logger.logging(app.time, app.cha, app.agent,[]);
app.time.k = app.logger.k;
% profile viewer
end
