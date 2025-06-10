classdef DRONE_PARAM < handle %PARAMETER_CLASS
    % ドローンの物理パラメータ管理用クラス
    % 以下のconfigurationはclass_description.pptxも参照すること．
    % T = [T1;T2;T3;T4];                  % Thrust force ：正がzb 向き
    % 前：ｘ軸，　左：y軸，　上：ｚ軸
    % motor configuration
    % T1 : 右後，T2：右前，T3：左後，T4：左前（x-y平面の象限順）
    % T2, T3 の回転方向は軸 zb,  T1, T4 : -zb      [1,0,0,1] で 正のyaw回転
    % tau = [(Ly - ly)*(T3+T4)-ly*(T1+T2); lx*(T1+T3)-(Lx-lx)*(T2+T4); km1*T1-km2*T2-km3*T3+km4*T4]; % Torque for body

    properties
        mass % DIATONE
        Lx
        Ly
        lx
        ly
        jx
        jy
        jz
        gravity
        km1
        km2
        km3
        km4
        k1
        k2
        k3
        k4
        rotor_r
        % T = k*w^2
        % T : thrust , w : angular velocity of rotor
        % M = km * T = km* k * w^2
        % M : zb moment  ：そのため普通の意味でのロータ定数とは違う
    end
    properties
        parameter % 制御モデル用パラメータ : 値ベクトル
        parameter_name % 物理パラメータの名前
        parameter_raw
        type
    end
    methods
        function obj = DRONE_PARAM(name,type,param)
            arguments
                name % DIATONE
                type = "row";
                param.mass = 0.7354;%0.5884;
                param.Lx = 0.16;
                param.Ly = 0.16;
                param.lx = 0.16/2;%0.05;
                param.ly = 0.16/2;%0.05;
                param.jx = 0.06;
                param.jy = 0.06;
                param.jz = 0.06;
                param.gravity = 9.81;
                param.km1 = 0.0301; % ロータ定数
                param.km2 = 0.0301; % ロータ定数
                param.km3 = 0.0301; % ロータ定数
                param.km4 = 0.0301; % ロータ定数
                param.k1 = 0.000008;          % 推力定数
                param.k2 = 0.000008;          % 推力定数
                param.k3 = 0.000008;          % 推力定数
                param.k4 = 0.000008;          % 推力定数
                param.rotor_r = 0.0392;
                % param.additional = []; % プロパティに無いパラメータを追加する場合
            end
            obj.type = type;
            if ~isempty(param)
                fn = fieldnames(param);
                % i = find(fn=="type",1); if ~isempty(i); fn(i) = [];end
                % i = find(fn=="parameter",1); if ~isempty(i); fn(i) = [];end
                % i = find(fn=="parameter_name",1); if ~isempty(i); fn(i) = [];end
                % i = find(fn=="additional",1); if ~isempty(i); fn(i) = [];end
                %obj.parameter_name = strings(1,length(fn));
                obj.parameter = zeros(1,length(fn));
                for i = 1:length(fn)
                    obj.(fn{i}) = param.(fn{i});
                    %   obj.parameter_name(i) = string(fn{i});
                    obj.parameter(i) = param.(fn{i});
                end
                obj.parameter_raw = param;
            end
            % if ~isempty(param.additional) % propertyに無いパラメータを設定する場合
            %     fn = fieldnames(param.additional);
            %     obj.parameter_name = [obj.parameter_name; string(fn)];
            %     for i = 1:length(fn)
            %         addprop(obj,fn{i});
            %         obj.(fn{i}) = param.additional.(fn{i});
            %     end
            % end
            obj.update_parameter();

            % obj = obj@PARAMETER_CLASS(name,type,param);
        end
    end
    methods
        function v = get(obj,p,type)
            arguments
                obj
                p = "all";
                type = obj.type;
            end
            if strcmp(p,"all")
                if strcmp(type, "row")
                    v = obj.parameter;
                else
                    v = obj.parameter_raw;
                end
            else
                % for i = 1:length(p)
                %     if strcmp(type,"row")
                %         val = obj.(string(p{i}));
                %         if ismatrix(val)
                %             val = reshape(val,[1,numel(val)]);
                %         end
                %         % if exist("v","var")
                %         %     v = [v,val];
                %         % else
                %             v = val;
                %         % end
                %     else
                %         v.(p{i})= obj.(p{i});
                %     end
                % end
                v= obj.(p);
            end
        end
        function set(obj,p,v)
            % v is struct with field p(:)
            for i = p
                obj.(i) = v.(i);
            end
            obj.update_parameter();
        end
        function update_parameter(obj)
            fn = fieldnames(obj.parameter_raw);
            obj.parameter = zeros(1,length(fn));
            for i = 1:length(fn)
                obj.parameter(i) = obj.(fn{i});
            end
        end
    end
end