classdef HLC < handle
    % 階層型線形化による公称入力と，同定に使う任意の励振入力
    properties
        self
        param
        result
        excite_counter = 0
    end

    properties (Constant)
        parameter_name = ["mass","Lx","Ly","lx","ly","jx","jy","jz","gravity","km1","km2","km3","km4","k1","k2","k3","k4"]
    end

    methods
        function obj = HLC(self, param)
            obj.self = self;
            obj.param = param;
            obj.param.P = self.parameter.get(obj.parameter_name);
            obj.result.input = zeros(4, 1);
        end

        function result = do(obj, time, phase, varargin)
            [uNom, x, xd] = obj.nominal_input(time);
            cfg = obj.get_explore_config();
            excitation = obj.generate_identification_excitation(uNom, phase, time, cfg);
            if ~isfield(obj.param, 'apply_identification_excitation') || ~obj.param.apply_identification_excitation
                excitation = 0;
            end
            input = HLC.limit_input(uNom + excitation);
            result = struct('input', input, 'u_nom', uNom, 'deltau', excitation, ...
                'hlc', input, 'pre_u', input, 'u_raw', input, 'hlc_x', x, 'hlc_xd', xd, ...
                'u_explore', excitation, 'explore_active', obj.get_explore_active_flag(phase), ...
                'explore_mode', cfg.mode);
            obj.result = result;
            if isfield(obj.param, 'enable_show') && obj.param.enable_show
                fprintf('HLC (%s) p = [%g %g %g], reference = [%g %g %g]\n', ...
                    phase, obj.self.estimator.result.state.p, obj.self.reference.result.state.p);
            end
        end

        function du = generate_identification_excitation(obj, uNom, phase, time, cfg)
            if nargin < 5
                cfg = obj.get_explore_config();
            end
            du = zeros(4, 1);
            if ~cfg.enable || (cfg.flight_only && ~obj.get_explore_active_flag(phase))
                return
            end
            tau = max(0, obj.get_time_value(time) - cfg.start_time);
            if strcmpi(cfg.mode, 'legacy_relative_random')
                du = cfg.legacy_scale .* abs(uNom) .* (2 * rand(4, 1) - 1);
            else
                for channel = 1:4
                    prefix = "u" + channel;
                    amp = cfg.(prefix + "_amp");
                    freq = cfg.(prefix + "_freq");
                    phi = cfg.(prefix + "_phase");
                    du(channel) = sum(amp(:) .* sin(2 * pi * freq(:) * tau + phi(:)));
                end
                du = min(1, tau / max(cfg.ramp_time, 1e-6)) * du;
            end
            du = max(-cfg.du_cap(:), min(cfg.du_cap(:), du));
        end

        function cfg = get_explore_config(obj)
            cfg = struct('enable', 1, 'flight_only', 1, 'mode', 'structured_multisine', ...
                'start_time', 0, 'ramp_time', 1, 'legacy_scale', 0.2 * ones(4,1), ...
                'u1_amp', [0.35; 0.15], 'u1_freq', [0.35; 0.90], 'u1_phase', [0.20; 1.10], ...
                'u2_amp', [0.010; 0.005], 'u2_freq', [0.55; 1.25], 'u2_phase', [0.40; 1.30], ...
                'u3_amp', [0.010; 0.005], 'u3_freq', [0.70; 1.55], 'u3_phase', [0.90; 0.30], ...
                'u4_amp', [0.0010; 0.0005], 'u4_freq', [0.45; 1.05], 'u4_phase', [0.50; 1.40], ...
                'du_cap', [0.55; 0.018; 0.018; 0.003]);
            if isfield(obj.param, 'explore')
                for name = string(fieldnames(obj.param.explore))'
                    cfg.(name) = obj.param.explore.(name);
                end
            end
        end

        function t = get_time_value(obj, time)
            if nargin >= 2 && isstruct(time) && isfield(time, 't')
                t = double(time.t);
            else
                t = obj.excite_counter;
                obj.excite_counter = obj.excite_counter + 1;
                if isfield(obj.param, 'dt') && ~isempty(obj.param.dt)
                    t = t * obj.param.dt;
                end
            end
        end
    end

    methods (Static)
        function input = limit_input(input)
            input = max([0; -1; -1; -1], min([10; 1; 1; 1], input));
        end

        function active = get_explore_active_flag(phase)
            active = ~(isstring(phase) || ischar(phase)) || startsWith(phase, 'f', 'IgnoreCase', true);
        end
    end

    methods (Access = private)
        function [uNom, x, xd] = nominal_input(obj, time)
            state = obj.self.estimator.result.state;
            xd = obj.self.reference.result.state.xd(:);
            xd(end+1:20, 1) = 0;

            % 現在状態と参照の各並進微分を目標yaw基準に揃える。
            rotation = RodriguesQuaternion(Eul2Quat([0; 0; xd(4)]))';
            x = [R2q(rotation * state.getq("rotmat")); rotation * state.p; rotation * state.v; state.w];
            for first = 1:4:17
                rows = first:first+2;
                xd(rows) = rotation * xd(rows);
            end
            xd(4) = 0;

            p = obj.param;
            if isfield(time, 'dt') && time.dt <= p.dt
                vf = Vfd(time.dt, x, xd', p.P, p.F1);
                vs = Vsd(time.dt, x, xd', vf, p.P, p.F2, p.F3, p.F4);
            else
                vf = Vf(x, xd', p.P, p.F1);
                vs = Vs(x, xd', vf, p.P, p.F2, p.F3, p.F4);
            end
            uNom = HLC.limit_input(Uf(x, xd', vf, p.P) + Us(x, xd', vf, vs', p.P));
        end
    end
end
