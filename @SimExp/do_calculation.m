function do_calculation(app)
% profile on
do_dt = zeros(6,app.N);
for i = 1:app.N
    app.agent(i).cha = app.cha;
    t1=tic; app.do_prop(app.agent(i),"sensor",i); do_dt(1,i)=toc(t1);
    t2=tic; app.do_prop(app.agent(i),"estimator",i); do_dt(2,i)=toc(t2);
    t3=tic; app.do_prop(app.agent(i),"reference",i); do_dt(3,i)=toc(t3);
    t4=tic; app.do_prop(app.agent(i),"controller",i); do_dt(4,i)=toc(t4);
    t5=tic; app.do_prop(app.agent(i),"input_transform",i); do_dt(5,i)=toc(t5);
    t6=tic; app.do_prop(app.agent(i),"plant",i); do_dt(6,i)=toc(t6);
end
t7=tic; app.logger.logging(app.time, app.cha, app.agent,[]); log_dt=toc(t7);
app.time.k = app.logger.k;
app.time.store_calc_time4do_calculation(do_dt, log_dt);
% profile viewer
end
