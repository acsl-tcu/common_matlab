classdef HLC_EDMD_MEC_EXP < HLC_EDMD_MEC
    % HLC_EDMD_MECを実験ログ取得用に拡張したクラス
    % 制御則自体は変更せず，EDMD残差学習に必要なログ情報だけを追加する

    methods
        function obj = HLC_EDMD_MEC_EXP(self, param)
            % 親クラスを初期化し，ログ用の公称EDMDモデルを読み込む
            obj@HLC_EDMD_MEC(self, param);
            obj.load_nominal_model_for_logging(param);
        end

        function result = do(obj, varargin)
            % 親クラスの制御計算を行い，実験用ログ項目を追加する
            result = do@HLC_EDMD_MEC(obj, varargin{:});
            result = obj.add_experiment_logging_fields(result);
            obj.result = result;
        end
    end

    methods (Access = private)
        function load_nominal_model_for_logging(obj, param)
            % ログ計算に必要な公称EDMDモデルを読み込む
            if obj.residual.loaded
                return;
            end
            % すでにモデルが読み込まれている場合は何もしない

            if ~isfield(param, 'residual') || ...
                    ~isfield(param.residual, 'model_file') || ...
                    isempty(param.residual.model_file)
                return;
            end
            % モデルファイルが指定されていない場合は何もしない

            model_file = char(string(param.residual.model_file));
            if ~isfile(model_file)
                return;
            end
            % 指定されたMATファイルが存在しない場合は何もしない

            try
                mdl = load(model_file, 'results_edmd');
                if isfield(mdl, 'results_edmd')
                    re = mdl.results_edmd;
                    if isfield(re, 'A_nom'), obj.residual.A_nom = re.A_nom; end
                    if isfield(re, 'B_nom'), obj.residual.B_nom = re.B_nom; end
                    if isfield(re, 'A_err'), obj.residual.A_err = re.A_err; end
                    if isfield(re, 'B_err'), obj.residual.B_err = re.B_err; end
                    obj.residual.loaded = ~isempty(obj.residual.A_nom) && ~isempty(obj.residual.B_nom);
                end
                % results_edmd から公称モデルと残差モデルを取得する
            catch ME
                warning('HLC_EDMD_MEC_EXP:modelLoadForLogging', ...
                    'Nominal EDMD logging model load failed: %s', ME.message);
            end
        end

        function result = add_experiment_logging_fields(obj, result)
            % EDMD残差学習に必要な情報をresultへ追加する
            u_nom = obj.get_result_vector(result, 'u_nom', zeros(4, 1));
            du_total = obj.get_result_vector(result, 'deltau', zeros(4, 1));
            du_training = obj.get_result_vector(result, 'delta_u_explore', zeros(4, 1));
            du_comp = obj.get_result_vector(result, 'delta_u_edmd', zeros(4, 1));
            % 公称入力，総差分入力，励振入力，補償入力を取得する

            result.u_nominal = u_nom;
            result.edmd_u_nominal = u_nom;
            result.delta_u = du_total;
            result.delta_u_total = du_total;
            result.delta_u_training = du_training;
            result.delta_u_comp = du_comp;
            result.input_actual = obj.get_result_vector(result, 'input', zeros(4, 1));
            % 学習データとして使いやすい名前で入力情報を保存する

            result.edmd_x_actual = [];
            result.edmd_x_reference = [];
            result.edmd_z_current = [];
            result.edmd_z_reference = [];
            result.edmd_z_nom_next = [];
            result.edmd_target_residual = [];
            result.edmd_nominal_prediction_available = false;
            result.edmd_state_source = obj.residual.state_source;
            % EDMD学習用ログ項目を初期化する

            try
                x_actual = obj.get_residual_state();
                x_reference = obj.get_reference_state();
                z_current = obj.klift_edmd_residual([x_actual; u_nom(:)]);
                z_reference = obj.klift_edmd_residual([x_reference; u_nom(:)]);
                % 現在状態と参照状態をlifted stateに変換する

                result.edmd_x_actual = x_actual;
                result.edmd_x_reference = x_reference;
                result.edmd_z_current = z_current;
                result.edmd_z_reference = z_reference;

                if obj.residual.loaded && ~isempty(obj.residual.A_nom) && ~isempty(obj.residual.B_nom)
                    z_nom_next = obj.residual.A_nom * z_current + obj.residual.B_nom * u_nom(:);
                    result.edmd_z_nom_next = z_nom_next;
                    result.edmd_target_residual = z_reference - z_nom_next;
                    result.edmd_nominal_prediction_available = true;
                end
                % 公称モデルによる1ステップ予測と，学習対象の残差を保存する
            catch ME
                result.edmd_logging_error = ME.message;
            end

            if isfield(obj.param, 'experiment') && isfield(obj.param.experiment, 'case')
                result.experiment_case = obj.param.experiment.case;
            else
                result.experiment_case = "";
            end
            % 実験ケース名があれば保存する

            try
                result.edmd_param_m_j = [obj.self.parameter.mass; ...
                                         obj.self.parameter.jx; ...
                                         obj.self.parameter.jy; ...
                                         obj.self.parameter.jz];
            catch
                result.edmd_param_m_j = [];
            end
            % 機体質量と慣性パラメータをログに保存する
        end

        function value = get_result_vector(~, result, field_name, default_value)
            % result内の指定フィールドを取得し，存在しない場合はデフォルト値を返す
            if isfield(result, field_name) && ~isempty(result.(field_name))
                value = result.(field_name);
            else
                value = default_value;
            end
        end
    end
end