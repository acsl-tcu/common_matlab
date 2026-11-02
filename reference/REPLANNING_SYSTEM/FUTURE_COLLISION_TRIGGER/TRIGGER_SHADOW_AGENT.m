classdef TRIGGER_SHADOW_AGENT < handle
    % Isolated container for *unmodified* HLC_SUSPENDED_LOAD.do().
    % Never contains or writes the Flight's actual agent.
    properties
        estimator
        reference
        parameter
    end
    methods
        function obj=TRIGGER_SHADOW_AGENT(state,ref,params)
            obj.estimator=struct('result',struct('state',state));
            obj.reference=struct('result',struct('state',ref));
            obj.parameter=params;
        end
    end
end
