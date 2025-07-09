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
            List = cell(1,length(list));
            res = agent.(prop).(list(1)).do(app.time,fCha,app.logger,app.env,app.agent,i);
            List{1} = res;
            for j = 2:length(list)
                if strcmp(app.cha,'f')
                app.cha;
            end
                res = merge_result(res,agent.(prop).(list(j)).do(app.time,fCha,app.logger,app.env,app.agent,i));
                List{j} = res;
            end
        end
        agent.(prop).result = res;
        if ~isempty(list)
            if strcmp(app.cha,'f')
                app.cha;
            end
            for i = 1:length(list)
                agent.(prop).result.(list(i)) = List{i};
            end
        end
    end
end
end