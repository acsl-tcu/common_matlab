classdef WP_TRIGGER_PARAMETER_VIEW
    %WP_TRIGGER_PARAMETER_VIEW Immutable read-only numeric parameter snapshot.
    % Constructed with the CURRENT EKF-estimated mL and actual model's other
    % parameters. No setter, no plant access, no modification of agent.
    properties (Access=private)
        Values
    end
    methods
        function obj=WP_TRIGGER_PARAMETER_VIEW(values)
            assert(isstruct(values)&&isfield(values,'loadmass'));
            obj.Values=values;
        end
        function v=get(obj,names,varargin) %#ok<INUSD>
            if nargin<2 || (isscalar(string(names)) && strcmpi(string(names),'all'))
                order={'mass','Lx','Ly','lx','ly','jx','jy','jz','gravity', ...
                    'km1','km2','km3','km4','k1','k2','k3','k4','rotor_r', ...
                    'Length','loadmass','cableL'};
                names=string(order);
            end
            names=string(names);
            v=zeros(1,numel(names));
            for k=1:numel(names)
                key=char(names(k));
                if ~isfield(obj.Values,key)
                    error('LightTrigger:ParameterMissing','Missing source model parameter %s',key);
                end
                v(k)=obj.Values.(key);
            end
        end
    end
end
