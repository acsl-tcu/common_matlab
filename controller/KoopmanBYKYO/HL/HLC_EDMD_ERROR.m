classdef HLC_EDMD_ERROR < handle
    % 直前の制御段の入力に，EDMDで学習した閉ループ誤差補償を加える。
    properties
        self
        result
        comp
    end

    methods
        function obj = HLC_EDMD_ERROR(self, param)
            obj.self = self;
            obj.result.input = zeros(4, 1);
            obj.comp = obj.initialize_compensator(param);
        end

        function result = do(obj, varargin)
            upstream = obj.self.controller.result;
            if ~isstruct(upstream) || ~isfield(upstream, 'input')
                error('HLC_EDMD_ERROR:MissingInput', 'Run the nominal controller before the EDMD compensator.');
            end
            [delta, diag] = obj.compute_error_compensation();
            result = upstream;
            input = HLC.limit_input(upstream.input + delta);
            result.deltau = delta;
            result.delta_u_edmd = delta;
            result.delta_u_thrust = [delta(1); 0; 0; 0];
            result.delta_u_torque = [0; delta(2:4)];
            result.u_raw = input;
            result.input = input;
            result.hlc = input;
            result.pre_u = input;
            result.residual_loaded = obj.comp.loaded;
            result.comp_e = diag.e;
            result.comp_zv = diag.zv;
            result.comp_za = diag.za;
            result.comp_du1_raw = diag.du1_raw;
            result.comp_dutau_raw = diag.dutau_raw;
            obj.result = result;
        end

        function [delta, diag] = compute_error_compensation(obj)
            delta = zeros(4, 1);
            diag = struct('e', zeros(12,1), 'zv', zeros(3,1), 'za', zeros(12,1), ...
                'du1_raw', 0, 'dutau_raw', zeros(3,1));
            if ~obj.comp.loaded
                return
            end

            state = obj.self.estimator.result.state;
            if strcmpi(obj.comp.state_source, 'plant')
                state = obj.self.plant.state;
            end
            ref = obj.self.reference.result.state;
            attitude = zeros(3, 1);
            if isprop(ref, 'q') && ~isempty(ref.q)
                attitude = ref.getq('euler');
            elseif isprop(ref, 'xd') && numel(ref.xd) >= 4
                attitude(3) = ref.xd(4);
            end
            rate = zeros(3, 1);
            if isprop(ref, 'w') && ~isempty(ref.w)
                rate = ref.w;
            end

            eq = mod(state.getq('euler') - attitude + pi, 2*pi) - pi;
            ew = state.w - rate;
            e = [state.p - ref.p; eq; state.v - ref.v; ew];
            diag.e = e;
            diag.zv = [e(3); e(9); e(3) * e(9)];
            diag.za = [eq; ew; sin(eq); eq .* ew];
            diag.du1_raw = -obj.comp.Kz * diag.zv;
            diag.dutau_raw = -obj.comp.Ka * diag.za;

            % フィルタには制限前の値を保持し，出力だけに倍率と上限を適用する。
            obj.comp.du1_prev = (1 - obj.comp.beta_z) * obj.comp.du1_prev + obj.comp.beta_z * diag.du1_raw;
            obj.comp.dutau_prev = (1 - obj.comp.beta_att) * obj.comp.dutau_prev + obj.comp.beta_att * diag.dutau_raw;
            delta = obj.comp.alpha * [obj.comp.du1_prev; obj.comp.dutau_prev];
            delta = max(-obj.comp.du_max(:), min(obj.comp.du_max(:), delta));
        end
    end

    methods (Static)
        function file = default_model_file()
            repo = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
            file = fullfile(repo, 'Data', 'Sim_data', 'hl_edmd_error_20261112_204555', ...
                'hl_edmd_error_model_flight.mat');
        end
    end

    methods (Static, Access = private)
        function comp = initialize_compensator(param)
            comp = struct('loaded', false, 'model_file', HLC_EDMD_ERROR.default_model_file(), ...
                'state_source', 'estimator', 'alpha', 1, 'du_max', [0.6; 0.12; 0.12; 0.12], ...
                'beta_z', 0.25, 'beta_att', 0.20, 'du1_prev', 0, 'dutau_prev', zeros(3,1), ...
                'Az', [], 'Bz', [], 'Aa', [], 'Ba', [], 'Kz', [], 'Ka', []);
            if isfield(param, 'comp')
                for name = string(fieldnames(param.comp))'
                    comp.(name) = param.comp.(name);
                end
            end
            if isempty(comp.model_file) || strlength(string(comp.model_file)) == 0
                return
            end
            try
                data = load(comp.model_file, 'results_hl_edmd_error');
                for name = ["Az", "Bz", "Aa", "Ba", "Kz", "Ka"]
                    comp.(name) = data.results_hl_edmd_error.(name);
                end
                comp.loaded = true;
            catch exception
                comp.loaded = false;
                warning('HLC_EDMD_ERROR:ModelLoad', 'EDMD model load failed: %s', exception.message);
            end
        end
    end
end
