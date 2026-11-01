classdef SPIKE_OFFLINE_AGENT < handle
    % Private offline mock only; never substitutes for the real Flight agent.
    properties
        estimator
        reference
    end
    methods
        function obj=SPIKE_OFFLINE_AGENT(ref)
            obj.estimator=struct('result',struct('state',struct('p',zeros(3,1))));
            obj.reference=struct('result',ref);
        end
        function set_sample(obj,p,ref)
            obj.estimator.result.state.p=double(p(:));
            obj.reference.result=ref;
        end
    end
end
