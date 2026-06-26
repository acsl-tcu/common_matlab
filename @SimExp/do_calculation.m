function do_calculation(app,ind)
% profile on
for i = 1:app.N
    app.agent(i).cha = app.cha;
    t5=tic; app.do_prop(app.agent(i),"sensor",i); app.tmp(ind,5)=toc(t5);
    t6=tic; app.do_prop(app.agent(i),"estimator",i); app.tmp(ind,6)=toc(t6);
    t7=tic; app.do_prop(app.agent(i),"reference",i); app.tmp(ind,7)=toc(t7);
    t8=tic; app.do_prop(app.agent(i),"controller",i); app.tmp(ind,8)=toc(t8);
    t9=tic; app.do_prop(app.agent(i),"input_transform",i); app.tmp(ind,9)=toc(t9);
    t10=tic; app.do_prop(app.agent(i),"plant",i); app.tmp(ind,10)=toc(t10);
end
t11=tic; app.logger.logging(app.time, app.cha, app.agent,[]); app.tmp(ind,11)=toc(t11);
app.time.k = app.logger.k;
% profile viewer
end
