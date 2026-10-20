classdef (Abstract) PARAMETER_CLASS < matlab.mixin.SetGetExactNames& dynamicprops
    % Model parameter class

    properties
        parameter % 制御モデル用パラメータ : 値ベクトル
        parameter_order % 物理パラメータの順序
        parameter_raw
        type
    end
    properties (Dependent)
        parameter_name % deprecated alias for parameter_order
    end

    methods
        function obj = PARAMETER_CLASS(name,type,param)
            arguments
                name %
                type = "row"; % row : 列ベクトル or struct : 構造体
                param = [];
            end
            obj.type = type;
            if ~isempty(param)
                fn = fieldnames(param);
                fn(fn=="type") = [];
                fn(fn=="parameter") = [];
                fn(fn=="parameter_order") = [];
                fn(fn=="parameter_name") = [];
                fn(fn=="additional") = [];  
                if isfield(param, "parameter_order") && ~isempty(param.parameter_order)
                    obj.parameter_order = string(param.parameter_order);
                elseif isfield(param, "parameter_name") && ~isempty(param.parameter_name)
                    obj.parameter_order = string(param.parameter_name);
                else
                    obj.parameter_order = string(fn);
                end
                for i = 1:length(fn)
                    obj.(fn{i}) = param.(fn{i});
                end
                obj.parameter_raw = param;
            end
            if ~isempty(param.additional) % propertyに無いパラメータを設定する場合
                fn = fieldnames(param.additional);
                obj.parameter_order = [obj.parameter_order; string(fn)];
                for i = 1:length(fn)
                    addprop(obj,fn{i});
                    obj.(fn{i}) = param.additional.(fn{i});
                end
            end            
            obj.update_parameter();
        end
        function v = get.parameter_name(obj)
            v = obj.parameter_order;
        end
        function set.parameter_name(obj, v)
            obj.parameter_order = v;
        end
    end
    methods
        function v = get(obj,p,type)
            arguments
                obj
                p = "all";
                type = obj.type;
            end
            if isstring(p) && isempty(p)
                if strcmp(type, "row")
                    v = [];
                else
                    v = struct();
                end
                return
            end
            if strcmp(p,"all")
              if strcmp(type, "row")
                v = obj.parameter;
              else
                v = obj.parameter_raw;
              end
            else
                for i = 1:length(p)
                    if strcmp(type,"row")
                        if p(i) == ""
                            val = 0;
                        else
                            val = obj.(p(i));
                        end
                        if ismatrix(val)
                            val = reshape(val,[1,numel(val)]);
                        end
                        if exist("v","var")
                            v = [v,val];
                        else
                            v = val;
                        end
                    else
                        if p(i) == ""
                            error("ACSL: empty parameter_order entries are not supported for struct output.");
                        end
                        v.(p(i))= obj.(p(i));
                    end
                end
            end
        end
        function set(obj,p,v)
            % v is struct with field p(:)
            for i = p
                if i == ""
                    continue
                end
                obj.(i) = v.(i);
            end
            obj.update_parameter();
        end
        function update_parameter(obj)
          obj.parameter=[];
            for i = 1:length(obj.parameter_order)
                if obj.parameter_order(i) == ""
                    obj.parameter = [obj.parameter, 0];
                elseif isprop(obj,obj.parameter_order(i))
                    % if strcmp(obj.type,"row")
                        val = obj.(obj.parameter_order(i));
                        if size(val,1) > 1
                            val = reshape(val,[1,numel(val)]);
                        end
                        obj.parameter=[obj.parameter, val];
                    % else
                    %     obj.parameter.(obj.parameter_order(i)) = obj.(obj.parameter_order(i));
                    % end
                else % propertyに無いパラメータ
                    error("ACSL : this is not a parameter.");
                end
            end
        end
    end
end
