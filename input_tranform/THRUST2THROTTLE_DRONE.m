classdef THRUST2THROTTLE_DRONE < handle
% Calculate throttle level from desired thrust forces to send via transmitter
% Do 1 step simulation wrt the model to derive a desired angular velocity wd and thrust force Fd.
% To follow wd and Fd minor feedback is designed as a P control.
% throttle level = K (sum(u) - hover_thrust_force) + throttle_offset
% roll,pitch,yaw throttle = K (w - wd) + w_offset
% This calculation is conducted at do_plant method in ABSTRACT_SYSTEM
properties
    self % Drone class instance
    result % derived throttle level
    param % offsets and gains
    flight_phase % flight phase input to figure handle FH
    hover_thrust_force % hovering時のthrust force
    state
end

methods

    function obj = THRUST2THROTTLE_DRONE(self, param)
        obj.self = self;
        obj.param = param;
        obj.param.roll_offset = self.plant.arming_msg(1);
        obj.param.pitch_offset = self.plant.arming_msg(2);
        obj.param.yaw_offset = self.plant.arming_msg(4);
        obj.param.P = self.parameter.get();
        obj.flight_phase = 's';
        P = self.parameter.get;
        obj.hover_thrust_force = P(1) * P(9);
        obj.state = state_copy(self.estimator.result.state);
    end

    function u = do(obj, varargin)
        %% u = [uroll, upitch, uthr, uyaw]
        % [Input] varargin : time, cha, logger, env, agent, i

        cha = varargin{2};
        input = varargin{5}(varargin{6}).controller.result.input;
        if (cha ~= 'q' && cha ~= 's' && cha ~= 'a' && cha ~= 'f' && cha ~= 'l' && cha ~= 't')
            cha = obj.flight_phase;
        end

        obj.flight_phase = cha;

        if cha == 't' || cha == 'f' || cha == 'l'
            wh = obj.self.estimator.result.state.w; % estimated state
            obj.self.estimator.(obj.self.estimator.name(1)).model.do(varargin{:}); % one step prediction using current input
            whn = obj.self.estimator.(obj.self.estimator.name(1)).model.state.w; % predicted state
            obj.self.estimator.(obj.self.estimator.name(1)).model.state.set_state(obj.self.estimator.result.state.get); % restore estimator.model
            % if cha == 'f'
                gain = obj.param.gain;
                th_offset = obj.select_th_offset(cha);
            % else
            %     gain = obj.param.gain_tl;
            %     th_offset = obj.param.th_offset_tl;
            % end

            T_thr = input(1); % thrust, torque input 

            uroll = gain(1) * (whn(1) - wh(1));
            upitch = gain(2) * (whn(2) - wh(2));

            % apply gain to (thrust - hovering_thrust)
            uthr = max(0, gain(4) * (T_thr - obj.hover_thrust_force) + th_offset); 
            uyaw = gain(3) * (whn(3) - wh(3));
            uroll = sign(uroll) * min(abs(uroll), 500) + obj.param.roll_offset;
            upitch = sign(upitch) * min(abs(upitch), 500) + obj.param.pitch_offset;
            uyaw = -sign(uyaw) * min(abs(uyaw), 300) + obj.param.yaw_offset; % Need minus : positive rotation is clockwise in betaflight
            obj.result = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000]; % CH8 = 1000 required for autonomous flight 
        else
            obj.result = [obj.param.roll_offset, obj.param.pitch_offset, 0, obj.param.yaw_offset, 1000, 0, 0, 0];
        end

        u = obj.result;
    end

end

methods (Access = private)
    function th_offset = select_th_offset(obj, cha)
        th_offset = obj.get_param("th_offset", 0);
        if obj.get_param("fGroundEffect", 0)
            if cha == 't' || cha == 'l'
                z = obj.get_altitude();
                z_low = obj.get_param("ge_z_low", 0);
                z_high = obj.get_param("ge_z_high", 0);
                if z_high <= z_low
                    alpha = 1;
                else
                    alpha = min(max((z - z_low) / (z_high - z_low), 0), 1);
                end
                th_low = obj.get_param("ge_th_offset_low", obj.get_param("th_offset_tl", th_offset));
                th_high = obj.get_param("ge_th_offset_high", th_offset);
                th_offset = th_low + (th_high - th_low) * alpha;
            end
        end
    end

    function z = get_altitude(obj)
        z = 0;
        if isprop(obj.self, "estimator") && isfield(obj.self.estimator, "result")
            if isprop(obj.self.estimator.result, "state") && isprop(obj.self.estimator.result.state, "p")
                p = obj.self.estimator.result.state.p;
                if numel(p) >= 3
                    z = p(3);
                    return
                end
            end
        end
        if isprop(obj.self, "reference") && isfield(obj.self.reference, "result")
            if isprop(obj.self.reference.result, "state") && isprop(obj.self.reference.result.state, "p")
                p = obj.self.reference.result.state.p;
                if numel(p) >= 3
                    z = p(3);
                    return
                end
            end
        end
    end

    function value = get_param(obj, name, fallback)
        if isstruct(obj.param) && isfield(obj.param, name)
            value = obj.param.(name);
        else
            value = fallback;
        end
    end
end

end
