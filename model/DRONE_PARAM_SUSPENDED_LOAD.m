classdef DRONE_PARAM_SUSPENDED_LOAD < PARAMETER_CLASS
    % ドローンの物理パラメータ管理用クラス（吊り荷モデル用）
    % 物理パラメータの順序は generateModel_SuspendedLoad.m の physicalParam と一致させること。

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
        Length
        loadmass
        cableL
    end

    methods
        function obj = DRONE_PARAM_SUSPENDED_LOAD(name, type, param)
            arguments
                name % DIATONE
                type = "row"
                param.mass = 0.754
                param.Lx = 0.195
                param.Ly = 0.195
                param.lx = 0.195/2
                param.ly = 0.195/2
                param.jx = 0.02237568
                param.jy = 0.02985236
                param.jz = 0.0480374
                param.gravity = 9.81
                param.km1 = 0.0301
                param.km2 = 0.0301
                param.km3 = 0.0301
                param.km4 = 0.0301
                param.k1 = 0.000008
                param.k2 = 0.000008
                param.k3 = 0.000008
                param.k4 = 0.000008
                param.rotor_r = 0.0392
                param.Length = 0.075
                param.loadmass = 0.0556
                param.cableL = 0.46
                param.additional = []
            end

            param_ordered = struct();
            param_ordered.mass = param.mass;
            param_ordered.Lx = param.Lx;
            param_ordered.Ly = param.Ly;
            param_ordered.lx = param.lx;
            param_ordered.ly = param.ly;
            param_ordered.jx = param.jx;
            param_ordered.jy = param.jy;
            param_ordered.jz = param.jz;
            param_ordered.gravity = param.gravity;
            param_ordered.km1 = param.km1;
            param_ordered.km2 = param.km2;
            param_ordered.km3 = param.km3;
            param_ordered.km4 = param.km4;
            param_ordered.k1 = param.k1;
            param_ordered.k2 = param.k2;
            param_ordered.k3 = param.k3;
            param_ordered.k4 = param.k4;
            param_ordered.rotor_r = param.rotor_r;
            param_ordered.Length = param.Length;
            param_ordered.loadmass = param.loadmass;
            param_ordered.cableL = param.cableL;
            param_ordered.additional = param.additional;

            obj = obj@PARAMETER_CLASS(name, type, param_ordered);
        end

        function v = get(obj, p, mode)
            arguments
                obj
                p = "all"
                mode = "row"
            end
            if strcmp(mode, "plant") || strcmp(mode, "model")
                mode = "row";
            end
            v = get@PARAMETER_CLASS(obj, p, mode);
        end

        function set(obj, p, v)
            % 制御モデルの値を設定する関数（従来互換）
            if isstruct(v)
                set@PARAMETER_CLASS(obj, p, v);
                return
            end
            for i = length(p):-1:1
                obj.(p(i)) = v(i);
            end
            obj.update_parameter();
        end

        function info = describe(obj)
            % パラメータの順序と現在値を一覧表示する。
            info = table((1:numel(obj.parameter_order))', obj.parameter_order, obj.parameter', ...
                "VariableNames", {"Index", "Name", "Value"});
            if nargout == 0
                disp(info);
            end
        end
    end
end
