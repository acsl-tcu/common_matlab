function do_prop(app,agent,prop,i)
if ~app.isReady && strcmp(prop,"plant")
    return
else
    if ~app.isReady 
        fCha = "0";
        app.time.t = 0;
    else fCha = app.cha; end
    if ~contains("ftla", app.cha) || strcmp(app.cha,"")
        app.cha = "s";
        return
    else
        list=agent.cha_allocation.(app.cha).(prop);
        if isempty(list)
            agent.(prop).result = agent.(prop).do(app.time,fCha,app.logger,app.env,app.agent,i);
        else
            agent.(prop).result = agent.(prop).(list(1)).do(app.time,fCha,app.logger,app.env,app.agent,i);
            for j = 2:length(list)
                agent.(prop).result = merge_result(agent.(prop).result,agent.(prop).(list(j)).do(app.time,fCha,app.logger,app.env,app.agent,i));
            end
        end    
    end
end
end