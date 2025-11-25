classdef INPUTTRANSFORM_AUTOTUNE < handle
% INPUTTRANSFORM_AUTOTUNE
% スロットルオフセット (mode=1) とゲイン (mode=2) の自動調整を行う入力変換クラス
% 使用例：
%   autot = INPUTTRANSFORM_AUTOTUNE(agent, mode)
% mode = 0: 自動調整オフ
% mode = 1: スロットルオフセット自動調整
% mode = 2: ゲイン自動調整
%
% agent は estimator / parameter / controller を含む構造体
% THRUST2THROTTLE_DRONE と同じ形の CH ベクトル (1x8) を返す
%
% 並列使用例：
%   agent.input_transform.origin = THRUST2THROTTLE_DRONE(...);
%   agent.input_transform.autotune = INPUTTRANSFORM_AUTOTUNE(agent, 1);
%   agent.cha_allocation.input_transform = ["origin","autotune"];

properties
    self
    mode        % 自動調整モード（0:off, 1:offset, 2:gain）
    monitor         % AutoTuneMonitor のインスタンス（任意）
    result
    % result_ch=zeros(1,8);
    result_ch=struct("roll",[],"pitch",[],"thrust",[],"yaw",[],"aux1",[],"aux2",[],"aux3",[],"aux4",[]);
    result_autotune=struct("gain",[],"offset",[],"score",[],"mode",[]);
    last_score = [];
    % ladt_score=Inf;


    % ------------------------------------------------------------
    % 自動調整対象のパラメータ（origin のものを継承する場合がある）
    % ------------------------------------------------------------
    th_offset = 0         % 通常時スロットルオフセット
    th_offset_tl = 0      % 離陸/着陸時のスロットルオフセット（takeoff/landing）
    gain = [100;100;100;10] % 通常時のゲイン [roll pitch yaw thrust]
    gain_tl = [100;100;100;10] % 離着陸時のゲイン

    % ------------------------------------------------------------
    % 自動調整パラメータ
    % ------------------------------------------------------------
    offset_step = 5;          % mode1: 1秒あたり何ステップ offset を増やすか
    offset_interval = 1.0;    % offset を更新する間隔（秒）
    offset_max = 350;         % 安全のための最大オフセット制限

    gain_step = [10;10;10;1];   % mode2: ゲイン調整ステップ量
    gain_max = [600;600;600;40];% ゲインの最大値

    % ------------------------------------------------------------
    % 内部状態管理
    % ------------------------------------------------------------
    last_t = [];              % 前回ループ時の時間
    time_accum = 0;           % offset/gain 更新タイミング用の蓄積時間

    % ------------------------------------------------------------
    % スコアリング用リングバッファ
    % ------------------------------------------------------------
    log_buf                     % ログ（t, w, wn, uthr 等）
    eval_window_sec = 1.0;      % スコア算出に用いる時間窓

    % ------------------------------------------------------------
    % ベストパラメータ記録
    % ------------------------------------------------------------
    best_param                  % 最も良いスコアのパラメータセット
    best_score = Inf            % 現在のベストスコア

    % ------------------------------------------------------------
    % 安定判定・自動ロック
    % ------------------------------------------------------------
    stable_counter = 0          % 安定判定カウンタ
    stable_threshold = 5        % 一定回数安定していたらパラメータ確定

    % ------------------------------------------------------------
    % 安全用キルスイッチ
    % ------------------------------------------------------------
    kill_flag = false           % true の場合即座にスロットルを 0 にする

    % ------------------------------------------------------------
    % スムージングフィルタの内部状態
    % ------------------------------------------------------------
    prev_u = zeros(4,1)         % 前回出力値
end


methods
    function obj = INPUTTRANSFORM_AUTOTUNE(self, mode)
        % コンストラクタ
        % mode が指定されていればセット
        if nargin >= 2
            obj.mode = mode;
        else
            obj.mode = 0;
        end
        obj.self = self;

        % ------------------------------------------------------------
        % origin のパラメータが存在すればコピーする（初期値継承）
        % ------------------------------------------------------------
        try
            if isfield(self.input_transform,'origin') && ...
               isprop(self.input_transform.origin,'param')

                p = self.input_transform.origin.param;
                if isfield(p,'th_offset'),    obj.th_offset = p.th_offset; end
                if isfield(p,'th_offset_tl'), obj.th_offset_tl = p.th_offset_tl; end
                if isfield(p,'gain'),         obj.gain = p.gain; end
                if isfield(p,'gain_tl'),      obj.gain_tl = p.gain_tl; end
            end
        catch
            % 存在しなくても問題なし。デフォルト値のまま
        end

        % ------------------------------------------------------------
        % mode > 0 の場合、モニターを作成（表示用）
        % ------------------------------------------------------------
        if obj.mode > 0
            try
                obj.monitor = AutoTuneMonitor();
            catch
                obj.monitor = [];
            end
        else
            obj.monitor = [];
        end

        % ------------------------------------------------------------
        % リングバッファの初期化（3秒分）
        % ------------------------------------------------------------
        dt_guess = 0.025;                    % サンプル時間推定値
        buflen = ceil(3 / dt_guess);         % 3秒分の長さ
        obj.log_buf = struct( ...
            't', nan(1,buflen), ...
            'w', nan(3,buflen), ...
            'wn', nan(3,buflen), ...
            'uthr', nan(1,buflen), ...
            'idx',1, ...
            'len', buflen);

        % ------------------------------------------------------------
        % ベストパラメータ初期化
        % ------------------------------------------------------------
        obj.best_param.th_offset = obj.th_offset;
        obj.best_param.gain = obj.gain;
        obj.best_score = Inf;
        obj.result_autotune = struct("gain", [], "offset", [], "score", [], "mode", []);
    end


    function u = do(obj, varargin)
        % do(): THRUST2 と同様のインタフェースで1x8ベクトルを返す
        % (time, cha, logger, env, agent_array, i) の形式を想定

        t_struct = varargin{1}; % 時間構造体
        cha = varargin{2};      % flight phase (q/s/a/f/l/t)

        % ------------------------------------------------------------
        % cha が未知の場合 origin の flight_phase を参照
        % ------------------------------------------------------------
        if (cha ~= 'q' && cha ~= 's' && cha ~= 'a' ...
         && cha ~= 'f' && cha ~= 'l' && cha ~= 't')

            try
                if isfield(obj.self.input_transform,'origin') && ...
                   isprop(obj.self.input_transform.origin,'flight_phase')

                    cha = obj.self.input_transform.origin.flight_phase;
                else
                    cha = 's';
                end
            catch
                cha = 's';
            end
        end

        % ------------------------------------------------------------
        % コントローラの算出した入力値を取得（失敗時はゼロ）
        % ------------------------------------------------------------
        try
            input = obj.self.controller.result.input;
        catch
            input = [0;0;0;0];
        end

        % ------------------------------------------------------------
        % ホバースラスト計算（P(1) = mass, P(9) = gravity）
        % ------------------------------------------------------------
        try
            P = obj.self.parameter.get();
            hover_thrust_force = P(1) * P(9);
        catch
            hover_thrust_force = 0;
        end

        % ------------------------------------------------------------
        % 推定角速度 wh と 1ステップ予測値 whn を取得
        % ------------------------------------------------------------
        try
            wh = obj.self.estimator.result.state.w; % 推定角速度

            % モデルで1ステップ未来を予測（内部状態維持のため保存→復元）
            obj.self.estimator.model.do(varargin{:});
            whn = obj.self.estimator.model.state.w;

            % 内部状態を復元
            obj.self.estimator.model.state.set_state( ...
                obj.self.estimator.result.state.get );

        catch
            wh = zeros(3,1);
            whn = zeros(3,1);
        end

        % ------------------------------------------------------------
        % 飛行フェーズに応じて TL 用ゲイン or 通常ゲインを選択
        % ------------------------------------------------------------
        if cha == 't' || cha == 'l'
            g = obj.gain_tl;
            offset = obj.th_offset_tl;
        else
            g = obj.gain;
            offset = obj.th_offset;
        end

        % shape を揃える
        if numel(g) ~= 4
            g = reshape(g,4,1);
        end

        % ------------------------------------------------------------
        % P制御（THRUST2 と同じロジック）
        % ------------------------------------------------------------
        T_thr = input(1);

        % 角速度誤差に基づく P 制御
        uroll  = g(1) * (whn(1) - wh(1));
        upitch = g(2) * (whn(2) - wh(2));
        uyaw   = g(3) * (whn(3) - wh(3));

        % スロットル：ホバーフォースとの差分 × ゲイン + オフセット
        uthr = max(0, g(4) * (T_thr - hover_thrust_force) + offset);

        % ------------------------------------------------------------
        % スムージングフィルタ（THRUST2 と同じ）
        % ------------------------------------------------------------
        alpha_u = 0.3;
        u_current = [uroll; upitch; uthr; uyaw];
        u_filtered = alpha_u * u_current + (1 - alpha_u) * obj.prev_u;
        obj.prev_u = u_filtered;

        uroll = u_filtered(1);
        upitch = u_filtered(2);
        uthr  = u_filtered(3);
        uyaw  = u_filtered(4);

        % ------------------------------------------------------------
        % arming offset を加算（実機出力に変換）
        % ------------------------------------------------------------
        try
            ro = obj.self.plant.arming_msg(1);
            po = obj.self.plant.arming_msg(2);
            yo = obj.self.plant.arming_msg(4);
        catch
            ro = 1500; po = 1500; yo = 1500;
        end

        % ±最大値を制限しつつオフセット加算
        uroll  = sign(uroll ) * min(abs(uroll ), 500) + ro;
        upitch = sign(upitch) * min(abs(upitch), 500) + po;
        uyaw   = -sign(uyaw ) * min(abs(uyaw ), 300) + yo;

        % ------------------------------------------------------------
        % 最終 CH ベクトル（THRUST2 と同じ形式）
        % ------------------------------------------------------------
        % obj.result_ch = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000];
        obj.result_ch =struct("roll",uroll,"pitch",upitch,"thrust",uthr,"yaw",uyaw,"aux1",1000,"aux2",0,"aux3",0,"aux4",1000);
        u = obj.result_ch;

        % ------------------------------------------------------------
        % ---------- ここから AutoTune モード専用処理 ----------
        % ------------------------------------------------------------

        % kill_flag が立っていたらスロットルを強制ゼロ
        if obj.kill_flag
            u(3) = 0;
            return;
        end

        % 自動調整は takeoff と flight 中のみ実行
        if obj.mode > 0 && (cha == 't' || cha == 'f')

            % 現在時刻を取得
            try
                tnow = t_struct.t;
            catch
                tnow = posixtime(datetime("now")); % fallback
            end

                       % --- ログバッファへ記録（循環バッファ方式） ---
            idx = obj.log_buf.idx;
            obj.log_buf.t(idx) = tnow;        % 時刻
            obj.log_buf.w(:,idx) = wh(:);     % 実モータ角速度
            obj.log_buf.wn(:,idx) = whn(:);   % 推定/正規化角速度
            obj.log_buf.uthr(idx) = u_filtered(3); % フィルタ済みスロットル入力
            obj.log_buf.idx = mod(idx, obj.log_buf.len) + 1; % インデックスを循環させる

            % --- 一定周期で評価を実行 ---
            if isempty(obj.last_t)
                obj.last_t = tnow; % 初回は基準時刻をセット
            end

            if tnow - obj.last_t >= obj.eval_window_sec
                % 直近 eval_window_sec 秒間のログ取得
                % [tvec,wvec,wnvec,uthrvec] = obj.getRecentWindow(obj.eval_window_sec);
                [~,wvec,wnvec,~] = obj.getRecentWindow(obj.eval_window_sec);

                % スコア（安定度）評価
                % score = obj.evaluate_stability(wnvec, wvec);
                score = obj.evaluate_stability(wnvec, wvec);
                % --- 安全チェック（姿勢・高度） ---
                try
                    p_est = obj.self.estimator.result.state.p; % 高度などの推定位置
                    q_est = obj.self.estimator.result.state.q; % 姿勢（Euler or Quaternion）
                    
                    % Euler 3成分 or クォータニオンの処理分岐
                    if numel(q_est) == 3
                        roll_deg = rad2deg(q_est(1));
                        pitch_deg = rad2deg(q_est(2));
                    else
                        % --- Quaternion → Euler変換 ---
                        qw=q_est(1); qx=q_est(2); qy=q_est(3); qz=q_est(4);
                        % roll
                        sinr_cosp = 2*(qw*qx + qy*qz);
                        cosr_cosp = 1 - 2*(qx*qx + qy*qy);
                        roll = atan2(sinr_cosp, cosr_cosp);
                        % pitch
                        sinp = 2*(qw*qy - qz*qx);
                        if abs(sinp) >= 1
                            pitch = sign(sinp)*pi/2;
                        else
                            pitch = asin(sinp);
                        end
                        roll_deg = rad2deg(roll);
                        pitch_deg = rad2deg(pitch);
                    end
                    alt = p_est(3); % 高度[m]
                catch
                    % 推定値取得失敗時はデフォルト
                    roll_deg = 0; pitch_deg = 0; alt = NaN;
                end

                % --- 姿勢安全チェック ---
                if abs(roll_deg) > 20 || abs(pitch_deg) > 20
                    obj.kill('姿勢異常 (Roll/Pitch ±20°超)');
                    return;
                end

                % --- 高度安全チェック ---
                if ~isnan(alt) && (alt < 0.1 || alt > 1.0)
                    obj.kill('高度範囲外 (0.1〜1.0 m)');
                    return;
                end

                % --- スコア急増（不安定化）の検出 ---
                if ~isinf(obj.best_score) && (score - obj.best_score) > max(1.0, abs(obj.best_score)*1.0)
                    obj.kill('スコア急増（不安定）');
                    return;
                end

                % ==============================
                %    オートチューニング本体
                % ==============================
                if obj.mode == 1
                    % ========= MODE 1: オフセット自動取得 =========
                    % 指定間隔ごとに offset を少しずつ増加させる
                    dt = tnow - obj.last_t;
                    if dt < 0, dt = 0; end
                    obj.time_accum = obj.time_accum + dt;

                    % offset_interval 秒たまったら offset を増加
                    while obj.time_accum >= obj.offset_interval
                        obj.time_accum = obj.time_accum - obj.offset_interval;
                        if obj.th_offset < obj.offset_max
                            obj.th_offset = min(obj.offset_max, obj.th_offset + obj.offset_step);
                            obj.th_offset_tl = obj.th_offset; % low-passed version 更新
                        end
                    end

                    % --- 高度に応じて微調整（浮き始めたら補正） ---
                    if ~isnan(alt) && alt > 0.05
                        target_z = 0.4;                 % 目標 0.4m
                        err_h = target_z - alt;         % 高度誤差
                        corr = 0.5 * err_h;             % 緩めの補正
                        corr = max(min(corr, 1), -1);   % -1〜1に制限
                        obj.th_offset = ...
                            min(max(obj.th_offset + corr, 0), obj.offset_max);
                        obj.th_offset_tl = obj.th_offset;
                    end

                elseif obj.mode == 2
                    % ========= MODE 2: ゲイン自動取得 (ヒルクライム) =========

                    % 状態持ち越し用（persistent 変数）
                    persistent axis_idx pending
                    if isempty(axis_idx), axis_idx = 1; end
                    if isempty(pending)
                        % 1軸ずつ改善を試すための管理構造体
                        pending = struct('waiting', false, 'baseline', [], ...
                                         'axis', 0, 'trialVal', []);
                    end

                    % thrust-line モードかどうかで使用ゲインを変更
                    if cha == 't'
                        gain_vec = obj.gain_tl;
                        max_vec = obj.gain_max;
                    else
                        gain_vec = obj.gain;
                        max_vec = obj.gain_max;
                    end

                    % ベースラインスコアの設定
                    baseline_score = obj.best_score;
                    if isinf(baseline_score)
                        baseline_score = score;
                        obj.best_score = baseline_score;
                    end

                    % ---- 新しい軸の試行を開始 ----
                    if ~pending.waiting
                        i = axis_idx;                  % 今チューニング中の軸
                        trial = gain_vec;              % 現在のゲインをコピー
                        step = min(obj.gain_step(i), max_vec(i)-trial(i)); % ステップ量
                        trial(i) = trial(i) + step;    % 1軸だけ増加

                        % 反映
                        if cha == 't'
                            obj.gain_tl = trial;
                        else
                            obj.gain = trial;
                        end

                        % 試行中フラグを立てる
                        pending.waiting = true;
                        pending.baseline = baseline_score;
                        pending.axis = i;
                        pending.trialVal = trial(i);

                    else
                        % ---- 観察期間：改善したかどうか評価 ----
                        if score < pending.baseline - 1e-6
                            % 改善 → 採用
                            obj.best_score = score;
                            obj.best_param.th_offset = obj.th_offset;
                            if cha == 't'
                                obj.best_param.gain = obj.gain_tl;
                            else
                                obj.best_param.gain = obj.gain;
                            end
                            % obj.best_param.gain = (cha=='t') - obj.gain_tl : obj.gain;
                        else
                            % 改善しない → 元に戻す
                            if cha == 't'
                                obj.gain_tl(pending.axis) = ...
                                    max(0, obj.gain_tl(pending.axis) - obj.gain_step(pending.axis));
                            else
                                obj.gain(pending.axis) = ...
                                    max(0, obj.gain(pending.axis) - obj.gain_step(pending.axis));
                            end
                        end

                        % 次の軸へ進む
                        pending.waiting = false;
                        axis_idx = axis_idx + 1;
                        if axis_idx > 4, axis_idx = 1; end
                    end
                end


                % update best score/bookkeeping
                % --- 現在の score がこれまでの best_score を更新した場合、
                %     ベスト値として記録しておく（オフセットとゲインも保存）
                if score < obj.best_score
                    obj.best_score = score;
                    obj.best_param.th_offset = obj.th_offset;
                    obj.best_param.gain = obj.gain;
                end

                % stability detection
                % --- 安定判定：スコアの変動が十分小さい状態が続くと mode=0 にロックする ---
                if isfinite(obj.best_score)
                    if isfinite(obj.best_score) && ~isempty(obj.best_score)
                        % small change check using last_score stored in object
                        % 前回スコアとの変化が 2% 未満なら「安定した」とカウント
                        if isfield(obj, 'last_score') && ~isempty(obj.last_score)
                            if abs(score - obj.last_score) < 0.02 * max(1, abs(obj.last_score))
                                obj.stable_counter = obj.stable_counter + 1;
                            else
                                % 変化が大きい場合は安定カウンタをリセット
                                obj.stable_counter = 0;
                            end
                        end
                    end

                    % stable_counter が規定値に達したら自動ロック
                    if obj.stable_counter >= obj.stable_threshold
                        obj.mode = 0; % lock (調整終了)
                        obj.showFinalAndLock();
                    % ===============================
                    % AutoTune 成功 → 最終パラメータを result_autotune に保存
                    % ===============================

                    obj.result_autotune = struct( ...
                        "gain",       obj.gain, ...       % チューニング結果のゲイン
                        "offset",     obj.th_offset, ...  % チューニング結果のオフセット
                        "score",      obj.best_score, ... % 最良スコア
                        "mode",       obj.mode ...        % 実行したモード
                    );
                       
                        % モニタにもロック状態を表示
                        if ~isempty(obj.monitor)
                            obj.monitor.update('AutoTune: LOCKED (stable)');
                        end
                    end
                end

                % gui update
                % --- GUI/モニターの表示更新（ゲイン・オフセット・スコア等） ---
                obj.updateMonitor(score);

                % last_t, last_score update
                % --- 次回の評価のために時刻とスコアをp記録 ---
                obj.last_t = tnow;
                obj.last_score = score;
            end
        end

        % If flight ended, print final best once
        % --- 飛行終了時(s or q) に final best を 1 回だけ表示 ---
        try
            if (cha == 's' || cha == 'q') && isfinite(obj.best_score)
                persistent shown;
                if isempty(shown) || ~shown
                    fprintf('\n==== AutoTune final best parameters ====\n');
                    disp(obj.best_param);
                    shown = true;
                end
            end
        catch
        end
    end


    function [tvec,wvec,wnvec,uthrvec] = getRecentWindow(obj, window_s)
        % --- 最新 window_s 秒間のログを取り出す ---
        buf = obj.log_buf;

        % ログが空なら空配列を返す
        if all(isnan(buf.t))
            tvec = []; wvec = []; wnvec = []; uthrvec = [];
            return;
        end

        % 有効データのみ抽出
        valid = ~isnan(buf.t);
        t_all = buf.t(valid);
        w_all = buf.w(:,valid);
        wn_all = buf.wn(:,valid);
        uthr_all = buf.uthr(valid);

        % データが空なら終了
        if isempty(t_all)
            tvec = []; wvec = []; wnvec = []; uthrvec = []; return;
        end

        % 最新 window_s 秒分のデータを抽出
        tcut = t_all >= (t_all(end) - window_s);
        tvec = t_all(tcut);
        wvec = w_all(:,tcut);
        wnvec = wn_all(:,tcut);
        uthrvec = uthr_all(tcut);
    end


    % function s = evaluate_stability(obj, wn, w)
      function s = evaluate_stability(~, wn, w)
        % --- 安定度スコアを計算：目標 vs 実測の差 + 振動成分 ---
        if isempty(wn) || isempty(w)
            s = Inf; return;
        end
        try
            % 1サンプルの場合と複数サンプルの場合で処理を分ける
            if size(wn,2) == 1
                % 二乗誤差
                stability = sum((wn - w).^2);
                % 振動レベル（分散）
                vibration = sum(var([wn w],0,2));
            else
                stability = sum(mean((wn - w).^2,2));
                vibration = sum(var([w; wn],0,2));
            end

            % スコア：小さいほど良い
            s = stability + 0.5 * vibration;

        catch
            s = Inf;
        end
    end


    function updateMonitor(obj, score)
        % --- モニター表示更新（オフセット・ゲイン・スコアなど） ---
        if ~isempty(obj.monitor), return; end

        try
            g = obj.gain;
            g_tl = obj.gain_tl;
            off = obj.th_offset;
            off_tl = obj.th_offset_tl;
        catch
            % 取得に失敗した場合は NaN を入れる
            g = [nan;nan;nan;nan];
            g_tl = g;
            off = nan; off_tl = nan;
        end

        b_score = obj.best_score;

        % 表示フォーマットにまとめる
        s = sprintf(['AutoTune Mode: %d   Phase: %s\n\n' ...
            'Gain    : [%.1f  %.1f  %.1f  %.1f]\n' ...
            'Gain_tl : [%.1f  %.1f  %.1f  %.1f]\n' ...
            'Offset  : %.1f\n' ...
            'Offset_tl: %.1f\n\n' ...
            'Current Score: %.4f\nBest Score   : %.4f\n' ...
            '(eval every %.1f s)'], ...
            obj.mode, 'autotune', ...
            g(1),g(2),g(3),g(4), ...
            g_tl(1),g_tl(2),g_tl(3),g_tl(4), ...
            off, off_tl, score, b_score, obj.eval_window_sec);

        % モニターへ送る
        try
            obj.monitor.update(s);
        catch
        end
    end


    function kill(obj, reason)
        % --- 緊急停止処理（姿勢異常・高度異常・不安定など） ---
        obj.kill_flag = true;

        % throttle = 0 になる安全な出力をセット
        try
            ro = obj.self.plant.arming_msg(1);
            po = obj.self.plant.arming_msg(2);
            yo = obj.self.plant.arming_msg(4);
        catch
            % arming_msg が取得できない場合のフォールバック
            ro = 1500; po = 1500; yo = 1500;
        end

        % obj.result_ch = [ro, po, 0, yo, 1000, 0, 0, 0];
        obj.result_ch =struct("roll",ro,"pitch",po,"thrust",0,"yaw",yo,"aux1",1000,"aux2",0,"aux3",0,"aux4",1000);

        fprintf('\n*** AUTOTUNE KILLED: %s ***\n', reason);

        % モニターにも kill 状態を表示
        if ~isempty(obj.monitor)
            obj.monitor.update(sprintf('AUTOTUNE KILLED: %s\nBest off=%.1f\nBest gain=[%.1f %.1f %.1f %.1f]', ...
                reason, obj.best_param.th_offset, ...
                obj.best_param.gain(1), obj.best_param.gain(2), obj.best_param.gain(3), obj.best_param.gain(4)));
        end
    end


    function showFinalAndLock(obj)
        % --- 安定してロックした場合の最終表示 ---
        fprintf('\n=== AutoTune auto-lock achieved ===\n');
        try
            disp(obj.best_param);
        catch
        end

        % GUI モニターにも最終結果を表示
        if ~isempty(obj.monitor)
            obj.monitor.update(sprintf( ...
                'AutoTune LOCKED\nBestOffset=%.1f\nBestGain=[%.1f %.1f %.1f %.1f]\nScore=%.4f', ...
                obj.best_param.th_offset, obj.best_param.gain(1), obj.best_param.gain(2), ...
                obj.best_param.gain(3), obj.best_param.gain(4), obj.best_score));
        end
    end
end
end