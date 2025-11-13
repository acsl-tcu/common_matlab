% function u_trans_param = InputTransform_Thrust2Throttle_drone(varargin)
%     % input transformation from thrust force to throttle level for
%     % drone Prop. input
% 
%     %% transmitter system
%     u_trans_param.gain_tl =[600;600;600;40]; % gain : [roll pitch yaw throttle]' %不明[850;850;600;600] 4s[700;700;600;400] 複数機[700;700;600;200] 発掘[800;800;800;400]
%     u_trans_param.th_offset_tl = 335;         % offset 3s[1021] 4s[900]　発掘[926]
%     u_trans_param.gain = [400;400;400;40];
%     u_trans_param.th_offset = 335;
%     % 手動切替: 1 = オフセット探索, 2 = ゲイン探索, 0 = autotune off
%     % 飛行前にここを変更してください（または呼び出し側で上書き）
%     % u_trans_param.mode = 1;
% 
%     % u_trans_param.gain_SuspendedLoad =[500;500;500;100]; % gain : [roll pitch yaw throttle]' %不明[850;850;600;600] 4s[700;700;600;400] 複数機[700;700;600;200] 発掘[800;800;800;400]
%     % u_trans_param.th_offset_SuspendedLoad = 450;         % offset 3s[1021] 4s[900]　発掘[926]
% end

function u_trans_param = InputTransform_Thrust2Throttle_drone(varargin)
% AutoTune対応 InputTransform（THRUST2互換版）
% mode: 1=Offset tuning, 2=Gain tuning
% 引数: cha (char), state (構造体, tを含む), monitor (AutoTuneMonitor), mode (1 or 2)

% ========================
% 初期値
% ========================
OFFSET_MIN = 0; OFFSET_MAX = 400;
GAIN_RPY_MIN = 10; GAIN_RPY_MAX = 600;
GAIN_THR_MIN = 10; GAIN_THR_MAX = 400;

u_trans_param.gain = [400;400;400;40];
u_trans_param.th_offset = 0;

% 引数取得
if nargin >= 4
    cha = varargin{1};
    state = varargin{2};
    monitor = varargin{3};
    mode = varargin{4};
else
    cha = 'unknown';
    state = struct('p',[0;0;0],'q',[0;0;0;1],'v',[0;0;0],'w',[0;0;0],'t',0);
    monitor = AutoTuneMonitor();
    mode = 1;
end

% ========================
% Persistent変数
% ========================
persistent tuning_started best_offset best_gain best_score th_offset gain_rpy gain_thr last_gui_update_t stable_counter

if isempty(tuning_started)
    tuning_started = false;
    best_offset = u_trans_param.th_offset;
    best_gain = u_trans_param.gain;
    best_score = inf;
    th_offset = u_trans_param.th_offset;
    gain_rpy = 50;
    gain_thr = 20;
    last_gui_update_t = 0;
    stable_counter = 0;
end

% ========================
% Takeoffフェーズでのみチューニング開始
% ========================
if ~tuning_started && strcmpi(cha,'t')
    tuning_started=true;
    if ~isempty(monitor)
        monitor.update('Takeoff detected: AutoTune started');
        drawnow;
    end
end

if tuning_started && strcmpi(cha,'t')
    tnow = state.t; % 時刻取得

    % ========================
    % 現在スコアの計算
    % ========================
    score = evaluate_score(state);

    % ========================
    % Offsetチューニング
    % ========================
    if mode == 1
        u_trans_param.gain = [50;50;50;20];  % 小さいゲイン
        % 高度補正: 少しでも浮き始めたら微調整
        alt = state.p(3);
        target_z = 0.5;
        err_h = target_z - alt;
        corr = 2.0 * err_h;
        corr = max(min(corr, 1), -1);
        th_offset = min(max(th_offset + 10 + corr, OFFSET_MIN), OFFSET_MAX);
        u_trans_param.th_offset = th_offset;

        if score < best_score
            best_score = score;
            best_offset = th_offset;
        end

    % ========================
    % Gainチューニング
    % ========================
    elseif mode == 2
        u_trans_param.th_offset = best_offset;  % Offsetは固定
        u_trans_param.gain = [gain_rpy; gain_rpy; gain_rpy; gain_thr];

        if score < best_score
            best_score = score;
            best_gain = u_trans_param.gain;
        end

        % 増加ステップ
        gain_rpy = min(gain_rpy + 5, GAIN_RPY_MAX);
        gain_thr = min(gain_thr + 1, GAIN_THR_MAX);
        u_trans_param.gain = best_gain;
    end

    % ========================
    % 安定判定・自動ロック
    % ========================
    if abs(state.q(1))>pi/6 || abs(state.q(2))>pi/6
        if ~isempty(monitor)
            monitor.update('AutoTune killed: Roll/Pitch ±30°超過');
        end
        tuning_started=false;
        return;
    end
    if alt<0.1 || alt>2.0
        if ~isempty(monitor)
            monitor.update('AutoTune killed: 高度範囲外');
        end
        tuning_started=false;
        return;
    end

    % 安定スコア判定
    if ~isempty(best_score)
        if abs(score - best_score) < 0.02 * max(1,abs(best_score))
            stable_counter = stable_counter + 1;
        else
            stable_counter = 0;
        end
        if stable_counter >= 5
            tuning_started=false;
            if ~isempty(monitor)
                msg = sprintf('AutoTune LOCKED\nOffset=%.1f\nGain=[%.1f %.1f %.1f %.1f]\nScore=%.4f', ...
                    best_offset, best_gain(1), best_gain(2), best_gain(3), best_gain(4), best_score);
                monitor.update(msg);
                drawnow;
            end
            return;
        end
    end

    % ========================
    % GUI更新（1秒ごと）
    % ========================
    if ~isempty(monitor) && tnow - last_gui_update_t >= 1.0
        msg = sprintf(['AutoTune Mode: %d\nPhase: %s\n\n' ...
                       'Offset  : %.1f\nGain    : [%.1f %.1f %.1f %.1f]\n\n' ...
                       'Current Score: %.4f\nBest Score   : %.4f'], ...
                       mode, cha, ...
                       u_trans_param.th_offset, ...
                       u_trans_param.gain(1), u_trans_param.gain(2), ...
                       u_trans_param.gain(3), u_trans_param.gain(4), ...
                       score, best_score);
        monitor.update(msg);
        drawnow limitrate;
        last_gui_update_t = tnow;
    end
end

% ========================
% スコア評価関数
% ========================
function score = evaluate_score(state)
q_rp = state.q(1:2);
w = state.w;
p = state.p;
score = 0.5*sum(q_rp.^2) + 0.3*sum(w.^2) + 0.2*sum(p.^2);
end

end
