classdef TRIGGER_SHADOW_PARAMETER < handle
    % Read-only, flight-bound view of the existing HLC parameter getter.
    % The HLC's original mass source is preserved; no replacement controller.
    properties (SetAccess=private)
        names
        values
    end
    methods
        function obj=TRIGGER_SHADOW_PARAMETER(names,values)
            obj.names=string(names(:));
            obj.values=double(values(:));
            assert(numel(obj.names)==numel(obj.values) && ...
                all(isfinite(obj.values)) && ...
                numel(unique(obj.names))==numel(obj.names), ...
                'TRIGGER_SHADOW_PARAMETER:Binding','Invalid existing parameter map.');
        end
        function v=get(obj,keys,varargin)
            % Precisely the row-vector convention of PARAMETER_CLASS.get.
            assert(isstring(keys)||ischar(keys), ...
                'TRIGGER_SHADOW_PARAMETER:Names','Parameter names must be strings.');
            if ~isempty(varargin)
                assert(numel(varargin)==1 && strcmp(char(varargin{1}),'row'), ...
                    'TRIGGER_SHADOW_PARAMETER:Mode','Only live HLC row mode is supported.');
            end
            keys=string(keys);
            assert(~any(keys=="all"), ...
                'TRIGGER_SHADOW_PARAMETER:All', ...
                'HLC must request the named physical parameters, not all.');
            v=zeros(1,numel(keys));
            for k=1:numel(keys)
                j=find(obj.names==keys(k),1);
                assert(~isempty(j),'TRIGGER_SHADOW_PARAMETER:Missing', ...
                    'Live HLC parameter is unavailable: %s',char(keys(k)));
                v(k)=obj.values(j);
            end
        end
    end
end
