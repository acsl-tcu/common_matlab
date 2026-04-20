function do_calculation(app)
% profile on
for i = 1:app.N
    app.agent(i).cha = app.cha;
    app.do_prop(app.agent(i),"sensor",i);
    app.do_prop(app.agent(i),"estimator",i);
    app.do_prop(app.agent(i),"reference",i);
    app.do_prop(app.agent(i),"controller",i);
    app.do_prop(app.agent(i),"input_transform",i);
    app.do_prop(app.agent(i),"plant",i);
end
app.logger.logging(app.time, app.cha, app.agent,[]);
app.time.k = app.logger.k;
% profile viewer
end
