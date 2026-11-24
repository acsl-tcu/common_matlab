classdef Ideal_MEC < handle
    %Ideal_MEC
    %   クアッドコプター用の理想的なモデル誤差補償器(MEC)
    %   プラントの物理パラメータ（質量・慣性モーメント）が既知であるとして，
    %   ノミナルモデルと同じ加速度・角加速度をプラントに生じさせる補償入力を毎時刻計算する．
    %   NN-MECの性能上限（理想補償器）との比較用．controllerのカスケードとして使う．
    %
    %   [理想補償入力]  入力 u = [f; tau]（総推力; 各軸トルク）
    %     並進:  f_p   = (m_p/m_n) * f_n
    %     回転:  tau_p = J_p*J_n^{-1}*(tau_n - w x (J_n w)) + w x (J_p w)
    %     Delta_u = u_p - u_n
    %   J_p = c*J_n（全軸同じ比率）のときジャイロ項は打ち消し合い，
    %   Delta_u* = ((I+Lambda)^{-1} - I) u_n （安定性解析ドキュメント 5.1）と一致する．
    %   "gyro",false にすると，ジャイロ項を無視した 5.1 の式そのものを使う．
    %
    %   [Inputs]
    %    self:        ドローンのagent（ノミナル値は self.parameter から取得）
    %    plant_param: プラントの物理パラメータ．次のどちらか
    %                 - 数値ベクトル: プラントモデルのパラメータ順序（resolve_parameter_order）
    %                   または self.parameter.parameter_order の並び（例: agent.plant.param）．
    %                   要素数が一致する順序を探し，パラメータ名の一致で mass, jx, jy, jz を取り出す
    %                 - struct: フィールド mass, jx, jy, jz（省略したものはノミナル値）
    %    opts.gyro:   true(既定) ジャイロ項まで含めた厳密な補償 / false 5.1 の式
    %
    %   [使用例]
    %    agent.controller.set_function_class("mec", Ideal_MEC(agent, agent.plant.param));
    %    agent.set_cha_allocation_for_all("controller", ["nominal","mec"])
    %
    %   2026/11 作成

    properties
        self
        result
        nominal_param   % ノミナル値 struct (mass, J)
        plant_param     % プラント値 struct (mass, J)
        fGyro           % ジャイロ項を補償に含めるか
    end

    methods
        function obj = Ideal_MEC(self, plant_param, opts)
            arguments
                self % agent
                plant_param
                opts.gyro (1,1) logical = true
            end
            obj.self = self;
            obj.fGyro = opts.gyro;

            % ノミナル値
            nominal = self.parameter.get(["mass","jx","jy","jz"]);
            obj.nominal_param.mass = nominal(1);
            obj.nominal_param.J = diag(nominal(2:4));

            % プラント値
            obj.plant_param = obj.parse_plant_param(plant_param, nominal);

            % データ関連の初期化
            est_name = self.estimator.name;
            input_dim = self.estimator.(est_name).model.dim(2);
            if input_dim ~= 4
                error("Ideal_MEC: 入力は [f; tau] の4次元を想定しています (input_dim = %d)", input_dim);
            end
            obj.result.nominal_input = zeros(input_dim,1);
            obj.result.delta_input = zeros(input_dim,1);
            obj.result.input = zeros(input_dim,1);

            fprintf("Ideal_MEC: m_p/m_n = %.4f,  J_p/J_n = [%.4f, %.4f, %.4f],  gyro = %d\n", ...
                obj.plant_param.mass/obj.nominal_param.mass, ...
                diag(obj.plant_param.J)./diag(obj.nominal_param.J), obj.fGyro);
        end % function Ideal_MEC

        function result = do(obj, varargin)
            u_n = varargin{5}.controller.nominal.result.input; % ノミナル入力 [f; tau]
            x = obj.self.estimator.result.state.get;          % 推定値 [p; q; v; w]（ε=0: 推定値=プラント値）
            w = x(10:12);

            m_n = obj.nominal_param.mass;  J_n = obj.nominal_param.J;
            m_p = obj.plant_param.mass;    J_p = obj.plant_param.J;

            f_n = u_n(1);
            tau_n = u_n(2:4);

            % 並進: f_p/m_p = f_n/m_n
            f_p = (m_p/m_n)*f_n;

            % 回転: J_p^{-1}(tau_p - w x J_p w) = J_n^{-1}(tau_n - w x J_n w)
            if obj.fGyro
                tau_p = J_p*(J_n\(tau_n - cross(w, J_n*w))) + cross(w, J_p*w);
            else
                tau_p = J_p*(J_n\tau_n); % ((I+Lambda)^{-1} - I) u_n に相当
            end

            obj.result.nominal_input = u_n;
            obj.result.delta_input = [f_p; tau_p] - u_n;
            obj.result.input = u_n + obj.result.delta_input;
            result = obj.result;
        end % function do
    end

    methods (Access = private)
        function p = parse_plant_param(obj, plant_param, nominal)
            % プラントパラメータを struct (mass, J) に整理する
            if isstruct(plant_param)
                names = ["mass","jx","jy","jz"];
                val = nominal;
                for idx = 1:numel(names)
                    if isfield(plant_param, names(idx))
                        val(idx) = plant_param.(names(idx));
                    end
                end
            elseif isnumeric(plant_param)
                % 数値ベクトルの各要素に対応するパラメータ名を取得し，名前の一致で取り出す
                order = obj.resolve_plant_order(numel(plant_param));
                names = ["mass","jx","jy","jz"];
                val = zeros(1,4);
                for idx = 1:numel(names)
                    pos = find(order == names(idx), 1);
                    if isempty(pos)
                        error("Ideal_MEC: plant_param のパラメータ順序に ""%s"" がありません (順序: %s)", ...
                            names(idx), strjoin(order, ", "));
                    end
                    val(idx) = plant_param(pos);
                end
            else
                error("Ideal_MEC: plant_param は数値ベクトルか struct で与えてください");
            end
            p.mass = val(1);
            p.J = diag(val(2:4));
        end

        function order = resolve_plant_order(obj, n)
            % 長さ n の数値パラメータベクトルに対応するパラメータ名の並びを求める
            %   1. プラントモデル(self.plant)の実体ファイルに記載の順序（resolve_parameter_order）
            %   2. agent.parameter の parameter_order
            % の順に，要素数が一致するものを採用する
            candidates = {};
            if isprop(obj.self, "plant") && ~isempty(obj.self.plant) && ~isempty(obj.self.plant.method)
                method = obj.self.plant.method;
                if isa(method, "function_handle")
                    method = func2str(method);
                end
                candidates{end+1} = resolve_parameter_order(string(method));
            end
            candidates{end+1} = string(obj.self.parameter.parameter_order);

            for idx = 1:numel(candidates)
                order = reshape(string(candidates{idx}), 1, []);
                if numel(order) == n
                    return
                end
            end
            error("Ideal_MEC: plant_param の長さ(%d)に一致するパラメータ順序が見つかりません", n);
        end
    end
end
