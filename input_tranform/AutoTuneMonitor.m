classdef AutoTuneMonitor < handle
    properties
        fig
        text
    end
    methods
        function obj = AutoTuneMonitor()
            obj.fig = figure('Name','AutoTune Monitor','NumberTitle','off',...
                'Position',[100 100 300 180],'Color',[0.15 0.15 0.15]);
            obj.text = uicontrol('Style','text','Parent',obj.fig,...
                'Units','normalized','Position',[0.05 0.2 0.9 0.75],...
                'BackgroundColor',[0.15 0.15 0.15],'ForegroundColor','w',...
                'FontSize',12,'HorizontalAlignment','left',...
                'String','AutoTune initialized...');
        end

        function update(obj,msg)
            if isvalid(obj.fig)
                set(obj.text,'String',msg);
                drawnow limitrate;
            end
        end

        function close(obj)
            if isvalid(obj.fig)
                close(obj.fig);
            end
        end
    end
end