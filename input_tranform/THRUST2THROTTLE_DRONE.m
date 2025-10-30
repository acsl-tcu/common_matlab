% classdef THRUST2THROTTLE_DRONE < handle
% % Calculate throttle level from desired thrust forces to send via transmitter
% % Do 1 step simulation wrt the model to derive a desired angular velocity wd and thrust force Fd.
% % To follow wd and Fd minor feedback is designed as a P control.
% % throttle level = K (sum(u) - hover_thrust_force) + throttle_offset
% % roll,pitch,yaw throttle = K (w - wd) + w_offset
% % This calculation is conducted at do_plant method in ABSTRACT_SYSTEM
% properties
%     self % Drone class instance
%     result % derived throttle level
%     param % offsets and gains
%     flight_phase % flight phase input to figure handle FH
%     hover_thrust_force % hovering時のthrust force
%     state
% end
% 
% methods
% 
%     function obj = THRUST2THROTTLE_DRONE(self, param)
%         obj.self = self;
%         obj.param = param;
%         obj.param.roll_offset = self.plant.arming_msg(1);
%         obj.param.pitch_offset = self.plant.arming_msg(2);
%         obj.param.yaw_offset = self.plant.arming_msg(4);
%         obj.param.P = self.parameter.get();
%         obj.flight_phase = 's';
%         P = self.parameter.get;
%         obj.hover_thrust_force = P(1) * P(9);
%         obj.state = state_copy(self.estimator.result.state);
%     end
% 
%     function u = do(obj, varargin)
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         persistent prev_dwh
%             if isempty(prev_dwh)
%                 prev_dwh=zeros(3,1);
%             end
%             alpha=0.2;
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %% u = [uroll, upitch, uthr, uyaw]
%         % [Input] varargin : time, cha, logger, env, agent, i
% 
%         cha = varargin{2};
%         input = varargin{5}(varargin{6}).controller.result.input;
%         if (cha ~= 'q' && cha ~= 's' && cha ~= 'a' && cha ~= 'f' && cha ~= 'l' && cha ~= 't')
%             cha = obj.flight_phase;
%         end
% 
%         obj.flight_phase = cha;
% 
%         if cha == 't' || cha == 'f' || cha == 'l'
%             wh = obj.self.estimator.result.state.w; % estimated state
%             obj.self.estimator.model.do(varargin{:}); % one step prediction using current input
%             whn = obj.self.estimator.model.state.w; % predicted state
%             obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get); % restore estimator.model
%             % if cha == 'f'
%                 gain = obj.param.gain;
%                 th_offset = obj.param.th_offset;
%             % else
%             %     gain = obj.param.gain_tl;
%             %     th_offset = obj.param.th_offset_tl;
%             % end
% 
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             dwh=whn-wh;
%             dwh=alpha*dwh+(1-alpha)*prev_dwh;
%             prev_dwh=dwh;
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%             T_thr = input(1); % thrust, torque input 
% 
%             uroll = gain(1) * (whn(1) - wh(1));
%             upitch = gain(2) * (whn(2) - wh(2));
% 
%             % apply gain to (thrust - hovering_thrust)
%             uthr = max(0, gain(4) * (T_thr - obj.hover_thrust_force) + th_offset); 
%             uyaw = gain(3) * (whn(3) - wh(3));
% 
%             %% === 出力スムージング用ローパスフィルタ ===
%             persistent prev_u
%             alpha_u = 0.3; % 0.2～0.5の範囲で調整
%             if isempty(prev_u)
%                 prev_u = zeros(4,1);
%             end
%             u_current = [uroll; upitch; uthr; uyaw];
%             u_filtered = alpha_u * u_current + (1 - alpha_u) * prev_u;
%             prev_u = u_filtered;
% 
%             uroll = u_filtered(1);
%             upitch = u_filtered(2);
%             uthr = u_filtered(3);
%             uyaw = u_filtered(4);
% 
%             %% === 制限とオフセット ===
%             uroll = sign(uroll) * min(abs(uroll), 500) + obj.param.roll_offset;
%             upitch = sign(upitch) * min(abs(upitch), 500) + obj.param.pitch_offset;
%             uyaw = -sign(uyaw) * min(abs(uyaw), 300) + obj.param.yaw_offset; % Need minus : positive rotation is clockwise in betaflight
%             obj.result = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000]; % CH8 = 1000 required for autonomous flight 
%         else
%             obj.result = [obj.param.roll_offset, obj.param.pitch_offset, 0, obj.param.yaw_offset, 1000, 0, 0, 0];
%         end
% 
%         u = obj.result;
%     end
% 
% end
% 
% end


classdef THRUST2THROTTLE_DRONE < handle
% THRUST2THROTTLE_DRONE with integrated AutoTune monitor
properties
    self
    result
    param
    flight_phase
    hover_thrust_force
    state

    % --- AutoTune properties ---
    autotune_mode      % 0:off, 1:offset tuning, 2:gain tuning (set from param.mode)
    autotune_handle    % GUI handle
    best_param         % best found parameters and score
    tune_counter       % internal counter
    last_gui_update_t  % last time GUI updated
    log_buf            % circular buffer for recent values
    eval_window_sec = 1.0; % evaluation window (seconds)
end

methods
    function obj = THRUST2THROTTLE_DRONE(self, param)
        obj.self = self;
        obj.param = param;
        % keep original offset init from arming_msg
        obj.param.roll_offset = self.plant.arming_msg(1);
        obj.param.pitch_offset = self.plant.arming_msg(2);
        obj.param.yaw_offset = self.plant.arming_msg(4);
        obj.param.P = self.parameter.get();
        obj.flight_phase = 's';
        P = self.parameter.get;
        obj.hover_thrust_force = P(1) * P(9);
        obj.state = state_copy(self.estimator.result.state);

        % --- autotune init ---
        if isfield(param,'mode')
            obj.autotune_mode = param.mode;
        else
            obj.autotune_mode = 0;
        end
        if obj.autotune_mode > 0
            obj.autotune_handle = AutoTuneMonitor();
        else
            obj.autotune_handle = [];
        end
        obj.best_param = obj.param;
        obj.best_param.score = inf;
        obj.tune_counter = 0;
        obj.last_gui_update_t = -inf;
        % log buffer: store recent time, ang vel, pred ang vel, throttle
        buflen = ceil(2 / 0.025); % ~2s buffer
        obj.log_buf = struct('t', nan(1,buflen),'w', nan(3,buflen),'wn', nan(3,buflen),'uthr', nan(1,buflen),'idx',1,'len',buflen);
    end

    function u = do(obj, varargin)
        % [Input] varargin : time, cha, logger, env, agent, i
        % return u = [uroll, upitch, uthr, uyaw, ...]
        cha = varargin{2};
        input = varargin{5}(varargin{6}).controller.result.input;
        if (cha ~= 'q' && cha ~= 's' && cha ~= 'a' && cha ~= 'f' && cha ~= 'l' && cha ~= 't')
            cha = obj.flight_phase;
        end
        obj.flight_phase = cha;

        % --- normal prediction and control parts (kept from original) ---
        persistent prev_dwh prev_u
        if isempty(prev_dwh), prev_dwh=zeros(3,1); end
        if isempty(prev_u), prev_u=zeros(4,1); end
        alpha=0.2;
        alpha_u = 0.3;

        if cha == 't' || cha == 'f' || cha == 'l'
            wh = obj.self.estimator.result.state.w; % estimated angular vel (3x1)
            % one-step prediction
            obj.self.estimator.model.do(varargin{:});
            whn = obj.self.estimator.model.state.w;
            % restore model state
            obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get);

            % choose param depending on phase (takeoff uses _tl)
            if cha == 't' || cha == 'l'
                gain = obj.param.gain_tl;
                th_offset = obj.param.th_offset_tl;
            else
                gain = obj.param.gain;
                th_offset = obj.param.th_offset;
            end

            % dwh smoothing
            dwh=whn-wh;
            dwh=alpha*dwh+(1-alpha)*prev_dwh;
            prev_dwh=dwh;

            T_thr = input(1); % thrust command (model input)
            % control law (same as your original)
            uroll = gain(1) * (whn(1) - wh(1));
            upitch = gain(2) * (whn(2) - wh(2));
            uthr = max(0, gain(4) * (T_thr - obj.hover_thrust_force) + th_offset);
            uyaw = gain(3) * (whn(3) - wh(3));

            u_current = [uroll; upitch; uthr; uyaw];
            u_filtered = alpha_u * u_current + (1 - alpha_u) * prev_u;
            prev_u = u_filtered;

            uroll = u_filtered(1);
            upitch = u_filtered(2);
            uthr = u_filtered(3);
            uyaw = u_filtered(4);

            % apply limits & offsets (original)
            uroll = sign(uroll) * min(abs(uroll), 500) + obj.param.roll_offset;
            upitch = sign(upitch) * min(abs(upitch), 500) + obj.param.pitch_offset;
            uyaw = -sign(uyaw) * min(abs(uyaw), 300) + obj.param.yaw_offset;
            obj.result = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000];
        else
            obj.result = [obj.param.roll_offset, obj.param.pitch_offset, 0, obj.param.yaw_offset, 1000, 0, 0, 0];
            wh = obj.self.estimator.result.state.w;
            whn = wh;
        end

        u = obj.result; % return final result

        % -------------- AutoTune: data collection --------------
        if obj.autotune_mode > 0 && (cha == 't' || cha == 'f')
            tnow = varargin{1}.t;
            % record into circular buffer
            idx = obj.log_buf.idx;
            obj.log_buf.t(idx) = tnow;
            obj.log_buf.w(:,idx) = wh(:);
            obj.log_buf.wn(:,idx) = whn(:);
            obj.log_buf.uthr(idx) = u_filtered(3);
            obj.log_buf.idx = mod(idx, obj.log_buf.len) + 1;

            % evaluate every eval_window_sec interval
            if tnow - obj.last_gui_update_t >= obj.eval_window_sec
                % extract valid window (last eval_window_sec)
                [tvec,wvec,wnvec,uthrvec] = obj.getRecentWindow(obj.eval_window_sec);
                score = obj.evaluate_stability(wnvec, wvec);
                % perform tuning step based on mode (conservative updates)
                switch obj.autotune_mode
                    case 1 % offset tuning
                        % target height from TAKEOFF_REFERENCE if available
                        try
                            target_z = obj.self.reference.takeoff.zd;
                        catch
                            target_z = 0.5; % fallback
                        end
                        % measure height average in recent window
                        try
                            p_hist = obj.self.estimator.result.state.p; % current estimate
                            h_now = p_hist(3);
                        catch
                            h_now = nan;
                        end
                        % Use the recent uthr -> if height below target, increase offset
                        if ~isnan(h_now)
                            % simple proportional correction
                            err_h = h_now - target_z;
                            % small conservative step
                            step_off = -1.5 * err_h; % negative err_h -> increase offset
                            % apply to takeoff or flight offset depending on phase
                            if cha == 't'
                                obj.param.th_offset_tl = obj.param.th_offset_tl + step_off;
                                obj.param.th_offset_tl = min(max(obj.param.th_offset_tl, 100), 700);
                            else
                                obj.param.th_offset = obj.param.th_offset + step_off;
                                obj.param.th_offset = min(max(obj.param.th_offset, 100), 700);
                            end
                        end
                        % update best if improved
                        if score < obj.best_param.score
                            obj.best_param = obj.param;
                            obj.best_param.score = score;
                        end

                    case 2 % gain tuning (per-axis hill-climb style)
                        % For each axis attempt a small +delta test, keep if score improves
                        deltas = [4;4;4;1]; % conservative step sizes for [roll,pitch,yaw,thrust]
                        % evaluate baseline score if not set
                        baseline_score = obj.best_param.score;
                        if isinf(baseline_score)
                            baseline_score = score;
                            obj.best_param.score = baseline_score;
                        end
                        % loop through axes
                        % do not change all at once; change one axis at a time (sequential)
                        persistent axis_idx;
                        if isempty(axis_idx), axis_idx = 1; end
                        i = axis_idx;
                        % create trial copy
                        if cha == 't'
                            trial = obj.param.gain_tl;
                        else
                            trial = obj.param.gain;
                        end
                        trial(i) = trial(i) + deltas(i);
                        % apply trial temporarily for a short period by setting param (will be used in next steps)
                        if cha == 't'
                            obj.param.gain_tl = trial;
                        else
                            obj.param.gain = trial;
                        end
                        % prepare to observe improvement in next windows: we will accept only if next measured score improves.
                        % For simplicity, we mark axis_idx to the next and on next evaluation compare
                        % So store a persistent buffer of pending trial baseline
                        persistent pending;
                        if isempty(pending)
                            pending = struct('waiting',false,'baseline',[], 'axis',0,'trialVal',[]);
                        end
                        if ~pending.waiting
                            % start waiting for trial result
                            pending.waiting = true;
                            pending.baseline = baseline_score;
                            pending.axis = i;
                            if cha == 't'
                                pending.trialVal = obj.param.gain_tl(i);
                            else
                                pending.trialVal = obj.param.gain(i);
                            end
                        else
                            % we are in waiting state: compare current score with baseline
                            if score < pending.baseline - 1e-6 % improved
                                % keep trial value and update best score
                                obj.best_param = obj.param;
                                obj.best_param.score = score;
                            else
                                % revert trial
                                if cha == 't'
                                    % revert by subtracting delta
                                    obj.param.gain_tl(pending.axis) = obj.param.gain_tl(pending.axis) - deltas(pending.axis);
                                else
                                    obj.param.gain(pending.axis) = obj.param.gain(pending.axis) - deltas(pending.axis);
                                end
                            end
                            % clear pending and move to next axis
                            pending.waiting = false;
                            axis_idx = axis_idx + 1;
                            if axis_idx > 4, axis_idx = 1; end
                        end
                end

                % GUI update
                obj.updateMonitor(score);

                % timestamp
                obj.last_gui_update_t = tnow;
            end
        end

        % show final best values when flight stops (we do not auto-save to file per your request)
        if (obj.flight_phase == 's' || obj.flight_phase == 'q') && ~isinf(obj.best_param.score)
            % display final results once
            persistent shown_final;
            if isempty(shown_final) || ~shown_final
                fprintf('\n==== AutoTune final best parameters ====\n');
                disp(obj.best_param);
                shown_final = true;
            end
            % also update GUI to reflect final best
            obj.updateMonitor(obj.best_param.score);
        end

    end

    function s = evaluate_stability(obj, wn, w)
        % wn, w are 3xN arrays (or 3x1). compute stability + vibration measure
        % Make safe for single column
        if isempty(wn) || isempty(w)
            s = inf; return;
        end
        % ensure 3xN
        wn = double(wn); w = double(w);
        if size(wn,2) == 1
            % trivial: small score based on squared diff
            stability = sum((wn - w).^2);
            vibration = sum(var([wn w],0,2));
            s = stability + 0.5 * vibration;
            return;
        end
        % normal case
        stability = sum(mean((wn - w).^2,2));
        vibration = sum(var([w; wn],0,2));
        s = stability + 0.5 * vibration;
    end

    function updateMonitor(obj, score)
        if isempty(obj.autotune_handle), return; end
        h = obj.autotune_handle;
        try
            gain = obj.param.gain;
            gain_tl = obj.param.gain_tl;
            off = obj.param.th_offset;
            off_tl = obj.param.th_offset_tl;
        catch
            gain = [nan;nan;nan;nan];
            gain_tl = gain;
            off = nan;
            off_tl = nan;
        end
        best_score = obj.best_param.score;
        s = sprintf(['AutoTune Mode: %d   Phase: %s\n\n' ...
            'Gain    : [%.1f  %.1f  %.1f  %.1f]\n' ...
            'Gain_tl : [%.1f  %.1f  %.1f  %.1f]\n' ...
            'Offset  : %.1f\n' ...
            'Offset_tl: %.1f\n\n' ...
            'Current Score: %.4f\nBest Score   : %.4f\n' ...
            '(updates every %.1f s)'], ...
            obj.autotune_mode, obj.flight_phase, ...
            gain(1),gain(2),gain(3),gain(4), ...
            gain_tl(1),gain_tl(2),gain_tl(3),gain_tl(4), ...
            off, off_tl, score, best_score, obj.eval_window_sec);
        set(h.text,'String',s);
        drawnow limitrate;
    end

    function [tvec,wvec,wnvec,uthrvec] = getRecentWindow(obj, window_s)
        % collect last window_s seconds from circular buffer
        buf = obj.log_buf;
        if all(isnan(buf.t))
            tvec = []; wvec = []; wnvec = []; uthrvec = [];
            return;
        end
        % find valid indices (non-NaN)
        valid = ~isnan(buf.t);
        t_all = buf.t(valid);
        w_all = buf.w(:,valid);
        wn_all = buf.wn(:,valid);
        uthr_all = buf.uthr(valid);
        if isempty(t_all)
            tvec = []; wvec = []; wnvec = []; uthrvec = []; return;
        end
        tcut = t_all >= (t_all(end) - window_s);
        tvec = t_all(tcut);
        wvec = w_all(:,tcut);
        wnvec = wn_all(:,tcut);
        uthrvec = uthr_all(tcut);
    end
end
end
