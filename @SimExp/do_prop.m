function do_prop(app,agent,prop,i)
if ~app.isReady && strcmp(prop,"plant")
    return
else
    if ~app.isReady; fCha = "0"; else fCha = app.cha; end
    if ~contains("ftla", app.cha) || strcmp(app.cha,"")
        app.cha = "s";
        return
    else
        list=agent.cha_allocation.(app.cha).(prop);
        if isempty(list)
            res = agent.(prop).do(app.time,fCha,app.logger,app.env,app.agent,i);
        else
            res = agent.(prop).(list(1)).do(app.time,fCha,app.logger,app.env,app.agent,i);
            for i = 2:length(list)
                res = merge_result(res,agent.(prop).(list(i)).do(app.time,fCha,app.logger,app.env,app.agent,i));
            end
        end
        agent.(prop).result = res;
    end
end
end