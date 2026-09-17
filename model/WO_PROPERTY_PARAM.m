classdef WO_PROPERTY_PARAM < matlab.mixin.SetGetExactNames
    % プロパティの無い（parameterにベクトルとして格納）パラメータクラス

    properties
        parameter % 制御モデル用パラメータ
        parameter_order % 物理パラメータの順序
        model_error % モデル誤差 : 制御対象の真値 - 制御モデル用パラメータ
    end

    methods
        function obj = WO_PROPERTY_PARAM(name,param)
            arguments
                name % DIATONE
                param.parameter_order = [];
                param.model_error = [];
            end
        if isfield(param, "parameter_name") && isempty(param.parameter_order)
            param.parameter_order = param.parameter_name;
        end
        if isempty(param.parameter_order)
            obj.parameter_order = string(properties(obj)');
            obj.parameter_order(strcmp(obj.parameter_order,"parameter")) = [];
            obj.parameter_order(strcmp(obj.parameter_order,"parameter_order")) = [];
            obj.parameter_order(strcmp(obj.parameter_order,"model_error")) = [];
        else
            obj.parameter_order = param.parameter_order;
        end
        for i = length(obj.parameter_order):-1:1
            obj.parameter(i)=obj.(obj.parameter_order(i));
            obj.model_error(i) = 0;
        end
        if ~isempty(param.model_error)
            obj.model_error = param.model_error;
        end
        end        
    end
    methods
        function v = get(obj,p,plant)
            arguments
                obj
                p = "all";
                plant = "model"
            end
            if strcmp(plant,"plant") % 制御対象の真値 : 制御モデル(parameter) + モデル誤差(model_error)
                if strcmp(p,"all") % 非推奨
                    v = obj.parameter + obj.model_error;
                else
                    for i = length(p):-1:1
                        v(i) = obj.(p(i)) + obj.model_error(strcmp(obj.parameter_order,p(i)));
                    end
                end
            else % 制御モデルで想定している値
                if strcmp(p,"all") % 非推奨
                    v = obj.parameter;
                else
                    for i = length(p):-1:1
                        v(i) = obj.(p(i));
                    end
                end
            end
        end
        function set_model_error(obj,p,v)
            for i = length(p):-1:1
                obj.model_error(strcmp(obj.parameter_order,p(i))) = v(i);
            end
        end
    end
end
