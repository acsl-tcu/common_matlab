classdef HLC_INPUT_ERROR_WRAPPER < handle
    % Apply the same additive input error to different controllers.

    properties
        inner
        param
        result
        excite_counter = 0;
    end

    methods
        function obj = HLC_INPUT_ERROR_WRAPPER(inner_controller, param)
            obj.inner = inner_controller;
            obj.param = obj.get_default_param();
            if nargin >= 2 && isstruct(param)
                fns = fieldnames(param);
                for i = 1:numel(fns)
                    obj.param.(fns{i}) = param.(fns{i});
                end
            end
            obj.result = struct();
            if isprop(inner_controller, 'result')
                obj.result = inner_controller.result;
            end
        end

        function result = do(obj, varargin)
            base_res = obj.inner.do(varargin{:});
            phase = varargin{2};
            du = obj.generate_input_error(phase, varargin{1});

            u_pre = reshape(double(base_res.input(1:4)), 4, 1);
            u_out = u_pre + du;
            u_out = [max(0, min(10, u_out(1))); ...
                     max(-1, min(1, u_out(2))); ...
                     max(-1, min(1, u_out(3))); ...
                     max(-1, min(1, u_out(4)))];

            result = base_res;
            result.input_before_error = u_pre;
            result.u_error = du;
            result.delta_u_error = du;
            result.input_after_error = u_out;
            result.error_wrapper_active = any(abs(du) > 0);
            result.input = u_out;
            result.u_raw = u_out;
            result.hlc = u_out;
            result.pre_u = u_out;

            obj.result = result;
        end

        function cfg = get_default_param(~)
            cfg = struct();
            cfg.enable = 1;
            cfg.flight_only = 1;
            cfg.mode = 'structured_multisine';
            cfg.start_time = 0.0;
            cfg.ramp_time = 1.0;
            cfg.u1_amp = [0.35; 0.15];
            cfg.u1_freq = [0.35; 0.90];
            cfg.u1_phase = [0.20; 1.10];
            cfg.u2_amp = [0.010; 0.005];
            cfg.u2_freq = [0.55; 1.25];
            cfg.u2_phase = [0.40; 1.30];
            cfg.u3_amp = [0.010; 0.005];
            cfg.u3_freq = [0.70; 1.55];
            cfg.u3_phase = [0.90; 0.30];
            cfg.u4_amp = [0.0010; 0.0005];
            cfg.u4_freq = [0.45; 1.05];
            cfg.u4_phase = [0.50; 1.40];
            cfg.du_cap = [0.55; 0.018; 0.018; 0.003];
            cfg.dt = 0.025;
        end

        function du = generate_input_error(obj, phase, time_info)
            cfg = obj.param;
            du = zeros(4, 1);

            if ~cfg.enable
                return;
            end
            if cfg.flight_only && ~obj.is_active_phase(phase)
                return;
            end

            t = obj.get_time_value(time_info);
            tau = max(0, t - cfg.start_time);
            ramp = min(1.0, tau / max(cfg.ramp_time, 1e-6));

            switch lower(char(string(cfg.mode)))
                case 'structured_multisine'
                    du(1) = sum(cfg.u1_amp(:) .* sin(2 * pi * cfg.u1_freq(:) * tau + cfg.u1_phase(:)));
                    du(2) = sum(cfg.u2_amp(:) .* sin(2 * pi * cfg.u2_freq(:) * tau + cfg.u2_phase(:)));
                    du(3) = sum(cfg.u3_amp(:) .* sin(2 * pi * cfg.u3_freq(:) * tau + cfg.u3_phase(:)));
                    du(4) = sum(cfg.u4_amp(:) .* sin(2 * pi * cfg.u4_freq(:) * tau + cfg.u4_phase(:)));
                    du = ramp * du;
                otherwise
                    du = zeros(4, 1);
            end

            du = max(min(du, cfg.du_cap(:)), -cfg.du_cap(:));
        end

        function t = get_time_value(obj, time_info)
            if nargin >= 2 && isstruct(time_info) && isfield(time_info, 't')
                t = double(time_info.t);
                return;
            end

            obj.excite_counter = obj.excite_counter + 1;
            t = (obj.excite_counter - 1) * obj.param.dt;
        end

        function active = is_active_phase(~, phase)
            if isstring(phase) || ischar(phase)
                phase_str = lower(char(phase));
                active = ~isempty(phase_str) && phase_str(1) == 'f';
            else
                active = true;
            end
        end
    end
end
