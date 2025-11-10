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
%             T_thr = input(1); % thrust, torque input 
% 
%             uroll = gain(1) * (whn(1) - wh(1));
%             upitch = gain(2) * (whn(2) - wh(2));
% 
%             % apply gain to (thrust - hovering_thrust)
%             uthr = max(0, gain(4) * (T_thr - obj.hover_thrust_force) + th_offset); 
%             uyaw = gain(3) * (whn(3) - wh(3));
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
% THRUST2THROTTLE_DRONE
% 元の制御処理を維持しつつ、リアルタイムのAutoTune (offset/gain) を統合
%
% 使用法:
%  - InputTransform_Thrust2Throttle_drone の戻り値 param.mode を
%      0: autotune OFF (通常)
%      1: offset tuning (gain fixed = 0, offsets start 0 -> increase)
%      2: gain tuning   (offset fixed manually, gains start 0 -> increase)
%  - GUI AutoTuneMonitor が存在すれば自動で呼ぶ（任意）
%

properties
    self
    result
    param
    flight_phase
    hover_thrust_force
    state

    % AutoTune
    autotune_mode      % 0:off, 1:offset tuning, 2:gain tuning
    autotune_handle    % GUI handle (AutoTuneMonitor)
    best_param         % struct of best params found + score
    tune_counter       % internal counter
    last_gui_update_t  % last time GUI updated
    log_buf            % circular buffer for recent values
    eval_window_sec = 1.0; % evaluate every 1 s

    % tuning internals
    offset_step = 5;            % increment step for offset (throttle units)
    gain_step = [10;10;10;1];   % increment step for gains [roll;pitch;yaw;thrust]
    % maxima (use your provided "current" values + margin)
    max_gain_tl = [600;600;600;40];
    max_gain =    [600;600;600;40];
    max_offset_tl = 335;
    max_offset =    335;

    % safety / stability
    best_score = Inf;
    stable_counter = 0;
    stable_threshold = 5; % number of consecutive windows for auto-lock
    kill_flag = false;
    last_score = Inf;
end

methods

    function obj = THRUST2THROTTLE_DRONE(self, param)
        % constructor: keep original param, initialize autotune state
        obj.self = self;
        obj.param = param;
        % keep original offsets from arming msg if exist
        if isprop(self.plant,'arming_msg')
            obj.param.roll_offset = self.plant.arming_msg(1);
            obj.param.pitch_offset = self.plant.arming_msg(2);
            obj.param.yaw_offset = self.plant.arming_msg(4);
        else
            if ~isfield(obj.param,'roll_offset'), obj.param.roll_offset = 1500; end
            if ~isfield(obj.param,'pitch_offset'), obj.param.pitch_offset = 1500; end
            if ~isfield(obj.param,'yaw_offset'), obj.param.yaw_offset = 1500; end
        end
        if ~isfield(obj.param,'P'), obj.param.P = self.parameter.get(); end

        obj.flight_phase = 's';
        P = obj.param.P;
        obj.hover_thrust_force = P(1) * P(9);
        obj.state = state_copy(self.estimator.result.state);

        % autotune init
        if isfield(param,'mode'), obj.autotune_mode = param.mode; else obj.autotune_mode = 0; end
        if obj.autotune_mode > 0
            try
                obj.autotune_handle = AutoTuneMonitor(); % optional GUI
            catch
                obj.autotune_handle = [];
            end
        else
            obj.autotune_handle = [];
        end

        obj.best_param = obj.param;
        obj.best_param.score = Inf;
        obj.tune_counter = 0;
        obj.last_gui_update_t = -inf;
        buflen = ceil(3 / 0.025); % circular buffer of ~3s
        obj.log_buf = struct('t', nan(1,buflen), 'w', nan(3,buflen), 'wn', nan(3,buflen), 'uthr', nan(1,buflen), 'idx',1, 'len', buflen);

        % If autotune mode requests zero-start behavior, override starting values:
        if obj.autotune_mode == 1
            % offset tuning: gains fixed to zero, offsets start at 0
            obj.param.gain = [100;100;100;10];
            obj.param.gain_tl = [100;100;100;10];
            obj.param.th_offset = 0;
            obj.param.th_offset_tl = 0;
        elseif obj.autotune_mode == 2
            % gain tuning: gains start at zero, offsets expected to be set manually
            obj.param.gain = [100;100;100;10];
            obj.param.gain_tl = [100;100;100;10];
            % param.th_offset / th_offset_tl should be set by user before run
        end
    end


    function u = do(obj, varargin)
        % [Input] varargin : time, cha, logger, env, agent, i
        % produce u = [uroll, upitch, uthr, uyaw, ...]      
        % AutoTune用カウンタ管理
        if ~isfield(obj, 'tune_counter') || isempty(obj.tune_counter)
            obj.tune_counter = 0;
        else
            obj.tune_counter = obj.tune_counter + 1;
        end
        cha = varargin{2};
        input = varargin{5}(varargin{6}).controller.result.input;
        if (cha ~= 'q' && cha ~= 's' && cha ~= 'a' && cha ~= 'f' && cha ~= 'l' && cha ~= 't')
            cha = obj.flight_phase;
        end
        obj.flight_phase = cha;

        persistent prev_dwh prev_u
        if isempty(prev_dwh), prev_dwh = zeros(3,1); end
        if isempty(prev_u), prev_u = zeros(4,1); end
        alpha = 0.2; alpha_u = 0.3;

        if cha == 't' || cha == 'f' || cha == 'l'
            % --- normal prediction and control ---
            wh = obj.self.estimator.result.state.w; % estimated angular vel
            % one-step prediction
            obj.self.estimator.model.do(varargin{:});
            whn = obj.self.estimator.model.state.w;
            % restore model state
            obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get);

            % select param depending on phase
            if cha == 't' || cha == 'l'
                gain = obj.param.gain_tl;
                th_offset = obj.param.th_offset_tl;
            else
                gain = obj.param.gain;
                th_offset = obj.param.th_offset;
            end

            % ensure gains/offsets are numeric vectors
            if numel(gain) ~= 4, gain = reshape(gain,4,1); end

            % smoothing of dwh
            dwh = whn - wh;
            dwh = alpha * dwh + (1 - alpha) * prev_dwh;
            prev_dwh = dwh;

            T_thr = input(1); % thrust command from controller/model

            % control law (same as original)
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

            % apply limits & offsets
            uroll = sign(uroll) * min(abs(uroll), 500) + obj.param.roll_offset;
            upitch = sign(upitch) * min(abs(upitch), 500) + obj.param.pitch_offset;
            uyaw = -sign(uyaw) * min(abs(uyaw), 300) + obj.param.yaw_offset;

            obj.result = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000];
        else
            % standby / non-flying phases
            obj.result = [obj.param.roll_offset, obj.param.pitch_offset, 0, obj.param.yaw_offset, 1000, 0, 0, 0];
            wh = obj.self.estimator.result.state.w;
            whn = wh;
        end

        u = obj.result;

        % -------------- AutoTune: data collection & tuning --------------
        if obj.kill_flag
            % If killed, force safe outputs (e.g. throttle 0)
            u(3) = 0;
            return;
        end

        if obj.autotune_mode > 0 && (cha == 't' || cha == 'f')
            tnow = varargin{1}.t;
            % store in circular buffer
            idx = obj.log_buf.idx;
            obj.log_buf.t(idx) = tnow;
            obj.log_buf.w(:,idx) = wh(:);
            obj.log_buf.wn(:,idx) = whn(:);
            obj.log_buf.uthr(idx) = u_filtered(3);
            obj.log_buf.idx = mod(idx, obj.log_buf.len) + 1;

            % evaluation every eval_window_sec
            if tnow - obj.last_gui_update_t >= obj.eval_window_sec
                [tvec,wvec,wnvec,uthrvec] = obj.getRecentWindow(obj.eval_window_sec);
                % compute stability score from wn and w
                score = obj.evaluate_stability(wnvec, wvec);

                % safety checks based on current estimated attitude and altitude
                % attitude (roll,pitch) from estimator result quaternion/angles
                try
                    p_est = obj.self.estimator.result.state.p; % position
                    q_est = obj.self.estimator.result.state.q; % quaternion or euler (depends)
                    % best effort: if q is 3-vector it's likely Euler angles; if 4, treat as quaternion
                    if numel(q_est) == 3
                        roll_deg = rad2deg(q_est(1));
                        pitch_deg = rad2deg(q_est(2));
                    else
                        % convert quaternion to Euler (approx) if needed
                        qw = q_est(1); qx=q_est(2); qy=q_est(3); qz=q_est(4);
                        % standard conversion (assuming [w x y z])
                        sinr_cosp = 2*(qw*qx + qy*qz);
                        cosr_cosp = 1 - 2*(qx*qx + qy*qy);
                        roll = atan2(sinr_cosp, cosr_cosp);
                        sinp = 2*(qw*qy - qz*qx);
                        if abs(sinp) >= 1
                            pitch = sign(sinp)*pi/2;
                        else
                            pitch = asin(sinp);
                        end
                        roll_deg = rad2deg(roll);
                        pitch_deg = rad2deg(pitch);
                    end
                    alt = p_est(3);
                catch
                    roll_deg = 0; pitch_deg = 0; alt = NaN;
                end

                % Safety: attitude
                if abs(roll_deg) > 30 || abs(pitch_deg) > 30
                    obj.kill('姿勢異常 (Roll/Pitch ±30°超)');
                    return;
                end
                % Safety: altitude bounds (0.1 - 2.0 m)
                if ~isnan(alt) && (alt < 0.1 || alt > 2.0)
                    obj.kill('高度範囲外 (0.1〜2.0 m)');
                    return;
                end
                % Safety: score jump (instability)
                if ~isinf(obj.last_score) && (score - obj.last_score) > max(1.0, abs(obj.last_score)*1.0)
                    % if score increased a lot (1.0 absolute or 100% relative)
                    obj.kill('スコア急増（不安定）');
                    return;
                end

                % ------------------ TUNING LOGIC ------------------
                if obj.autotune_mode == 1
                % OFFSET TUNING:
                % gains remain zero; offset increases gradually,
                % 主にオフセットを0から少しずつ上げていき、機体が浮き始める値を探す

                % --- 現在高度の取得 ---
                try
                    alt = obj.self.estimator.result.state.position(3);
                catch
                    alt = NaN;
                end

                % --- 段階的オフセット上昇 ---
                % まずは静止状態でも一定周期で少しずつ上昇
                if obj.param.th_offset < obj.max_offset
                    % 初期は小さく、だんだん大きく
                    if obj.param.th_offset < 50
                        obj.param.th_offset = obj.param.th_offset + 10;   % 最初はゆっくり(初期２)　%最初大きく（変更）
                    elseif obj.param.th_offset < 200
                        obj.param.th_offset = obj.param.th_offset + 5;   % 中盤は普通に（初期５）　%途中から普通に（変更）
                    % else
                    %     obj.param.th_offset = obj.param.th_offset + 5;  % ある程度上がってきたら速めに（初期10）
                    end
                end

                % --- 高度に応じた微調整 ---
                % 少しでも浮き始めたら微調整モードに移行
                if ~isnan(alt) && alt > 0.1 %初期設定値0.05
                    target_z = 0.5;  % 目標高度（例: 0.5 m）
                    err_h = target_z - alt;
                    corr = 2.0 * err_h;  % 下がっていればオフセット増加，上がりすぎなら減少
                    corr = max(min(corr, 5), -5);  % 補正量を制限
                    obj.param.th_offset = obj.param.th_offset + corr;
                end

                % --- 上限制限 & 同期 ---
                obj.param.th_offset = min(max(obj.param.th_offset, 0), obj.max_offset);
                obj.param.th_offset_tl = obj.param.th_offset;
               
                
                %一番最初のもの
                % % if obj.autotune_mode == 1
                % %     % OFFSET TUNING:
                % %     % gains remain zero; offset increases gradually,
                % %     % but we apply small proportional correction using altitude error as well.
                % %     % Primary: step-wise increase; secondary: small correction by altitude error.
                % %     % Step increase
                % %     if obj.param.th_offset < obj.max_offset
                % %         obj.param.th_offset = min(obj.max_offset, obj.param.th_offset + obj.offset_step);
                % %         obj.param.th_offset_tl = obj.param.th_offset; % keep tl synced for takeoff phase
                % %     end
                % %     % small altitude-based fine tuning:
                % %     try
                % %         target_z = obj.self.reference.takeoff.zd; % TAKEOFF_REFERENCE provides zd
                % %     catch
                % %         target_z = 0.5;
                % %     end
                % %     % use current altitude deviation to slightly adjust offset
                % %     if ~isnan(alt)
                % %         err_h = (alt - target_z);
                % %         % small corrective term
                % %         corr = -0.5 * err_h; % if below target (err negative) -> increase offset
                % %         if abs(corr) > 0.1
                % %             corr = sign(corr) * 0.1;
                % %         end
                % %         obj.param.th_offset = min(max(obj.param.th_offset + corr, 0), obj.max_offset);
                % %         obj.param.th_offset_tl = obj.param.th_offset;
                % %     end
                    % % % % % 追加したもの（必要ない？）
                    % % % % % % 安全対策: 高度安定までチューニング停止 & 初期緩和ステップ
                    % % % % % alt = obj.state.position(3);
                    % % % % % if isnan(alt) || alt < 0.05 %
                    % % % % % % 高度が低すぎるときはチューニングスキップ
                    % % % % % return;
                    % % % % % end
                    % % % % % 
                    % % % % % if obj.tune_counter < 5
                    % % % % %     obj.param.th_offset = obj.param.th_offset + 1; % 初期は小刻み
                    % % % % % else
                    % % % % %     obj.param.th_offset = obj.param.th_offset + obj.offset_step;
                    % % % % % end
                    % % % % % 
                    % % % % % % 上限制限
                    % % % % % obj.param.th_offset = min(obj.param.th_offset, obj.max_offset);
                    % % % % % obj.param.th_offset_tl = obj.param.th_offset;

                elseif obj.autotune_mode == 2
                    % GAIN TUNING:
                    % offsets assumed fixed (manually set before starting)
                    % gains start at zero and are increased axis-by-axis in a conservative hill-climb
                    % Use persistent pending trial to require observation period
                    persistent axis_idx pending
                    if isempty(axis_idx), axis_idx = 1; end
                    if isempty(pending)
                        pending = struct('waiting',false,'baseline',[], 'axis',0,'trialVal',[]);
                    end

                    % Determine target gain vector reference depending on phase
                    if cha == 't'
                        gain_vec = obj.param.gain_tl;
                        max_vec = obj.max_gain_tl;
                    else
                        gain_vec = obj.param.gain;
                        max_vec = obj.max_gain;
                    end

                    % baseline init
                    baseline_score = obj.best_param.score;
                    if isinf(baseline_score)
                        baseline_score = score;
                        obj.best_param.score = baseline_score;
                    end

                    if ~pending.waiting
                        % create trial: increase axis axis_idx by step
                        i = axis_idx;
                        trial = gain_vec;
                        trial(i) = min(max_vec(i), trial(i) + obj.gain_step(i));
                        % apply trial immediately (will be observed in next window)
                        if cha == 't'
                            obj.param.gain_tl = trial;
                        else
                            obj.param.gain = trial;
                        end
                        pending.waiting = true;
                        pending.baseline = baseline_score;
                        pending.axis = i;
                        pending.trialVal = trial(i);
                    else
                        % observe
                        if score < pending.baseline - 1e-6
                            % improvement -> keep trial and update best
                            obj.best_param = obj.param;
                            obj.best_param.score = score;
                        else
                            % no improvement -> revert
                            if cha == 't'
                                obj.param.gain_tl(pending.axis) = max(0, obj.param.gain_tl(pending.axis) - obj.gain_step(pending.axis));
                            else
                                obj.param.gain(pending.axis) = max(0, obj.param.gain(pending.axis) - obj.gain_step(pending.axis));
                            end
                        end
                        % move to next axis
                        pending.waiting = false;
                        axis_idx = axis_idx + 1;
                        if axis_idx > 4, axis_idx = 1; end
                    end
                end

                % update best score if improved by any tuning step
                if score < obj.best_param.score
                    obj.best_param = obj.param;
                    obj.best_param.score = score;
                end

                % ---------- stability detection & auto-lock ----------
                % ---- 追加ここから ----
                % モータが動いていない時はスコア更新をスキップ
                if isfield(obj.result, 'input')
                    thr_mean = mean(obj.result.input(4,:));
                    if thr_mean < 50
                    return;
                    end
                end

                if isfinite(obj.best_param.score)
                    % if recent scores stable (small stddev), increment stable_counter
                    if isfinite(obj.last_score)
                        if abs(score - obj.last_score) < 0.02 * max(1, abs(obj.last_score))
                            obj.stable_counter = obj.stable_counter + 1;
                        else
                            obj.stable_counter = 0;
                        end
                    end
                    if obj.stable_counter >= obj.stable_threshold
                        % lock best param: freeze tuning and display
                        obj.autotune_mode = 0; % turn off autotune (auto-lock)
                        obj.showFinalAndLock();
                        return;
                    end
                end

                % update GUI
                obj.updateMonitor(score);

                % update timestamps / last_score
                obj.last_gui_update_t = tnow;
                obj.last_score = score;
            end
        end % end autotune block

        % Display final best values on transition to stop/standby if exists
        if (obj.flight_phase == 's' || obj.flight_phase == 'q') && ~isinf(obj.best_param.score)
            persistent shown_final;
            if isempty(shown_final) || ~shown_final
                fprintf('\n==== AutoTune final best parameters ====\n');
                disp(obj.best_param);
                shown_final = true;
            end
            obj.updateMonitor(obj.best_param.score);
        end

    end % end do()

    function s = evaluate_stability(obj, wn, w)
        % compute score: stability (mean squared error of predicted vs estimated angular vel)
        % plus vibration measure from variance
        if isempty(wn) || isempty(w)
            s = Inf; return;
        end
        % wn, w are 3xN arrays possibly
        try
            if size(wn,2) == 1
                stability = sum((wn - w).^2);
                vibration = sum(var([wn w],0,2));
            else
                stability = sum(mean((wn - w).^2,2));
                vibration = sum(var([w; wn],0,2));
            end
            s = stability + 0.5 * vibration;
        catch
            s = Inf;
        end
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
            off = nan; off_tl = nan;
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
        try
            set(h.text,'String',s);
            drawnow limitrate;
        catch
            % ignore GUI errors
        end
    end

    function [tvec,wvec,wnvec,uthrvec] = getRecentWindow(obj, window_s)
        buf = obj.log_buf;
        if all(isnan(buf.t))
            tvec = []; wvec = []; wnvec = []; uthrvec = [];
            return;
        end
        valid = ~isnan(buf.t);
        t_all = buf.t(valid);
        w_all = buf.w(:,valid);
        wn_all = buf.wn(:,valid);
        uthr_all = buf.uthr(valid);
        if isempty(t_all)
            tvec = []; wvec = []; wnvec = []; uthrvec = [];
            return;
        end
        tcut = t_all >= (t_all(end) - window_s);
        tvec = t_all(tcut);
        wvec = w_all(:,tcut);
        wnvec = wn_all(:,tcut);
        uthrvec = uthr_all(tcut);
    end

    function kill(obj, reason)
        % hard kill: immediately stop motors (set throttle outputs to safe)
        obj.kill_flag = true;
        % attempt to set outputs to safe values by writing to result (will be used by caller)
        try
            safe_u = [obj.param.roll_offset, obj.param.pitch_offset, 0, obj.param.yaw_offset, 1000, 0, 0, 0];
            obj.result = safe_u;
        catch
        end
        % notify
        fprintf('\n*** AUTO-TUNE KILLED: %s ***\n', reason);
        fprintf('Best param found (so far):\n');
        try
            disp(obj.best_param);
        catch
        end
        % update GUI status
        if ~isempty(obj.autotune_handle)
            try
                msg = sprintf('AutoTune STOPPED\nReason: %s\nBestOffset=%.1f\nBestGain=[%.1f %.1f %.1f %.1f]', ...
                    reason, obj.best_param.th_offset, obj.best_param.gain(1), obj.best_param.gain(2), obj.best_param.gain(3), obj.best_param.gain(4));
                set(obj.autotune_handle.text,'String',msg);
                drawnow;
            catch
            end
        end
    end

    function showFinalAndLock(obj)
        % called when stability detected -> lock current best params
        obj.autotune_mode = 0; % stop tuning
        fprintf('\n=== AutoTune auto-lock achieved ===\n');
        try
            disp(obj.best_param);
        catch
        end
        if ~isempty(obj.autotune_handle)
            try
                msg = sprintf('AutoTune LOCKED\nBestOffset=%.1f\nBestGain=[%.1f %.1f %.1f %.1f]\nScore=%.4f', ...
                    obj.best_param.th_offset, obj.best_param.gain(1), obj.best_param.gain(2), obj.best_param.gain(3), obj.best_param.gain(4), obj.best_param.score);
                set(obj.autotune_handle.text,'String',msg);
                drawnow;
            catch
            end
        end
    end

end % methods

end % classdef