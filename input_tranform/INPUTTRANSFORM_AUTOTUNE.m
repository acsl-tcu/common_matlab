classdef INPUTTRANSFORM_AUTOTUNE < handle
properties
    self            % ドローン本体 (agent)
    mode            % 0:無効, 1:オフセット取得, 2:ゲイン取得

    monitor         % GUI 表示用モニタ（スコア・ゲイン・オフセット）
    result_ch       % 出力結果を格納する構造体

    % -------------------------
    % 調整するパラメータ
    % -------------------------
    th_offset = 0                   % スロットルオフセット
    gain = [100;100;100;10]         % ゲイン（roll,pitch,yaw,thrust）

    % -------------------------
    % 調整ステップ量（1回の更新幅）
    % -------------------------
    offset_step = 5
    gain_step = [10;10;10;1]        % ゲインをどれだけ増やすか
    gain_max = [600;600;600;40]     % ゲイン上限

    % ---- オフセット自動取得（mode1）用 ----
    time_accum = 0;          % 時間積算（初期ゼロ）
    offset_interval = 0.15;  % offset を増加させる間隔（秒）
    offset_max = 450;        % 安全上限（モーターが急上昇しない値）

    % -------------------------
    % ログバッファ（最近の角速度などを保存）
    % -------------------------
    log_buf
    last_t = []                     % 前回評価した時間
    best_score = Inf                % これまでの最良スコア
    last_score = []                 % 最後に計算したスコア
    
    offset_fixed = false           % ベスト offset を固定したかどうか
    offset_lock_threshold = 0.005  % offset 固定判断のためのスコア閾値

    axis_idx = 1                  % 1～4（Roll, Pitch, Yaw, Throttle）
    waiting = false               % trial 評価中か
    baseline = inf                % 比較スコア
    trialVal = []                 % 試行中ゲイン値
    best_param = struct('th_offset',0,'gain',zeros(4,1))     % ベスト offset / gain の保存場所
end


methods
    % ============================================================
    % コンストラクタ 
    % ============================================================
    function obj = INPUTTRANSFORM_AUTOTUNE(self, mode)
        if nargin<2, mode=0; end % modeが指定されなければ0（無効）
        obj.self = self;
        obj.mode = mode;

        % origin側のゲイン・オフセットを初期値に採用
        try
            if isfield(self.input_transform,'origin') ...
               && isprop(self.input_transform.origin,'param')

                p = self.input_transform.origin.param;

                if isfield(p,'th_offset'), obj.th_offset = p.th_offset; end
                if isfield(p,'gain'), obj.gain = p.gain; end
            end
        catch
        end

        % モニター生成
        if obj.mode>0
            try
                obj.monitor = AutoTuneMonitor();
            catch
                obj.monitor = [];
            end
        end

        % ログバッファを3秒分確保（25ms ≒ 40Hz を想定）
        dt_guess = 0.025;
        buflen = ceil(3/dt_guess);
        obj.log_buf = struct(...
            't',nan(1,buflen), ...
            'w',nan(3,buflen), ...
            'wn',nan(3,buflen), ...
            'uthr',nan(1,buflen), ...
            'idx',1, ...
            'len',buflen );
    end


    % ============================================================
    % メイン処理（入力変換 + 自動調整）
    % ============================================================
    function u = do(obj, t_struct, cha)

        % cha が誤っていた場合の対策
        if ~ismember(cha,{'q','s','a','f','l','t'})
            cha='s';
        end

        % コントローラの出力 (T, Roll, Pitch, Yaw)
        try
            input = obj.self.controller.result.input;
        catch
            input = [0;0;0;0];
        end

        % ホバースラスト推定
        try
            P = obj.self.parameter.get();
            hover_thrust_force = P(1)*P(9);
        catch
            hover_thrust_force = 0;
        end

        % 現在状態とモデルの更新計算
        try
            wh = obj.self.estimator.result.state.w;
            obj.self.estimator.model.do(t_struct, cha, obj.self.controller, obj.self.estimator, obj.self, 1);
            whn = obj.self.estimator.model.state.w;

            % モデル状態を元に戻す（シミュレーション干渉防止）
            obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get);
        catch
            wh=zeros(3,1);
            whn=zeros(3,1);
        end

        % ==============================
        % 入力変換 (P制御)
        % ==============================
        g = obj.gain;
        offset = obj.th_offset;

        T_thr = input(1);

        % roll, pitch, yaw の変換（角速度差をP制御）
        uroll  = g(1)*(whn(1)-wh(1));
        upitch = g(2)*(whn(2)-wh(2));
        uyaw   = g(3)*(whn(3)-wh(3));

        % thrust（推力→スロットル変換）
        uthr   = max(0, g(4)*(T_thr-hover_thrust_force) + offset);

        % 結果を構造体として格納
        obj.result_ch = struct( ...
            'roll',uroll, ...
            'pitch',upitch, ...
            'thrust',uthr, ...
            'yaw',uyaw, ...
            'aux1',1000,'aux2',0,'aux3',0,'aux4',1000 );
        u = obj.result_ch;


        % ============================================================
        % === 自動調整（autotune）処理： 't' フェーズのみ ===
        % ============================================================
        if obj.mode>0 && cha=='t'

            % 現在時刻の取得
            try
                tnow = t_struct.t;
            catch
                tnow = posixtime(datetime("now"));
            end

            % -------------------------
            % ログ更新
            % -------------------------
            idx = obj.log_buf.idx;
            obj.log_buf.t(idx)   = tnow;% 時刻
            obj.log_buf.w(:,idx) = wh(:);% 実モータ角速度
            obj.log_buf.wn(:,idx)= whn(:); % 推定/正規化角速度
            obj.log_buf.uthr(idx)= uthr; % フィルタ済みスロットル入力
            obj.log_buf.idx = mod(idx,obj.log_buf.len)+1; % インデックスを循環させる

            % 初回は評価しない
            if isempty(obj.last_t)
                obj.last_t = tnow;
            end

            % 1秒ごとに評価（評価間隔）
            if tnow - obj.last_t >= 1.0

                % 最近1秒のデータを取得
                [~,wvec,wnvec,~] = obj.getRecentWindow(1.0);

                % スコアを計算（揺れ + 不安定性）
                score = obj.evaluate_stability(wnvec, wvec);


                % ============================
                %   MODE 1 & MODE 2 (改良版)
                % ============================

                % 時間差分（自動取得は t フェーズのみ）
                dt = max(0, tnow - obj.last_t);
                obj.time_accum = obj.time_accum + dt;
                obj.last_t = tnow;

                % ----------------------------
                % MODE 1: オフセット自動取得
                % ----------------------------
                if obj.mode == 1

                    % すでにベスト値が決まったら固定
                if obj.offset_fixed
                    obj.th_offset = obj.best_param.th_offset;
                    return;
                end
            
                % 一定間隔で offset を微増
                while obj.time_accum >= obj.offset_interval
                    obj.time_accum = obj.time_accum - obj.offset_interval;
                    obj.th_offset = min(obj.offset_max, obj.th_offset + obj.offset_step);
                end

                % --- スコアによるベスト判定（最重要） ---
                if score < obj.best_score
                    obj.best_score = score;
                    obj.best_param.th_offset = obj.th_offset;
                end
            
                % --- ベスト値が決まったら固定 ---
                % offset が上限に近づいてスコア改善が止まったら
                if obj.th_offset >= obj.offset_max || ...
                   abs(score - obj.best_score) < obj.offset_lock_threshold
                    obj.offset_fixed = true;
                    obj.th_offset = obj.best_param.th_offset;
                    obj.mode = 0;     % mode1 終了
                end
            
                return;
            end


            % ----------------------------
            % MODE 2: ゲイン自動取得 (ヒルクライム)
            % ----------------------------
            if obj.mode == 2
            
                % 軸インデックス初期化
                if isempty(obj.axis_idx), obj.axis_idx = 1; end
                i = obj.axis_idx;
            

                gain_vec = obj.gain;

            
                % --- 新規試行開始 ---
                if ~obj.waiting
                    trial = gain_vec;
            
                    % 上限超えないようにステップ計算
                    step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
                    trial(i) = trial(i) + step;

                    % 反映

                        obj.gain = trial;
            
                    % 保持
                    obj.waiting = true;
                    obj.baseline = obj.best_score;
                    obj.trialVal = trial(i);
            
                    return;
            
                % --- 評価フェーズ ---
                else
                    if score < obj.baseline - 1e-6     % 改善した
                        obj.best_score = score;
                        obj.best_param.gain = gain_vec;
                    else                                % 改善なし → 元に戻す

                            obj.gain(i) = max(0, obj.gain(i) - obj.gain_step(i));
                        
                    end
            
                    % 次の軸へ進む
                    obj.waiting = false;
                    obj.axis_idx = obj.axis_idx + 1;
                    if obj.axis_idx > 4
                        obj.axis_idx = 1;
                    end
            
                    return;
                end
            end
            
            
            % ----------------------------
            % MODE 0: 何もしない（GUI表示のみ）
            % ----------------------------
            % best_score を常に更新
            if score < obj.best_score
                obj.best_score = score;
                obj.best_param.th_offset = obj.th_offset;
                obj.best_param.gain = obj.gain;
            end



                % =======================
                % GUI更新
                % =======================
                if ~isempty(obj.monitor)
                    s = sprintf( ...
                        'Mode:%d Gain:[%.1f %.1f %.1f %.1f] Offset:%.1f  Score:%.4f Best:%.4f', ...
                        obj.mode, g(1),g(2),g(3),g(4), obj.th_offset, score, obj.best_score);
                    obj.monitor.update(s);
                end

                obj.last_t = tnow;
                obj.last_score = score;
            end
        end
    end


    % ============================================================
    % ログから最近 window_s 秒のデータを取得
    % ============================================================
    function [tvec,wvec,wnvec,uthrvec] = getRecentWindow(obj, window_s)
        buf=obj.log_buf;

        % 有効データのみ取得
        valid = ~isnan(buf.t);

        t_all = buf.t(valid);
        w_all = buf.w(:,valid);
        wn_all= buf.wn(:,valid);
        uthr_all = buf.uthr(valid);

        if isempty(t_all)
            tvec=[]; wvec=[]; wnvec=[]; uthrvec=[];
            return;
        end

        % 時間でフィルタ
        tcut = t_all >= (t_all(end)-window_s);

        tvec = t_all(tcut);
        wvec = w_all(:,tcut);
        wnvec= wn_all(:,tcut);
        uthrvec = uthr_all(tcut);
    end


    % ============================================================
    % スコア計算（揺れ + 不安定性）
    % ============================================================
    function s = evaluate_stability(~, wn, w)
        if isempty(wn) || isempty(w)
            s = Inf;
            return;
        end

        % モデルとの差の大きさ（不安定性）
        stability = sum( mean( (wn - w).^2 , 2) );

        % ローパス前後の揺れ量
        vibration = sum( var([w; wn],0,2) );

        % 合計（重み0.5）
        s = stability + 0.5*vibration;
    end
end
end
