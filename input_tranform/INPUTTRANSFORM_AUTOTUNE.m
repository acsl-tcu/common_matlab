% classdef INPUTTRANSFORM_AUTOTUNE < handle
% properties
%     self            % ドローン本体 (agent)
%     mode            % 0:無効, 1:オフセット取得, 2:ゲイン取得
% 
%     monitor         % GUI 表示用モニタ（スコア・ゲイン・オフセット）
%     result       % 出力結果を格納する構造体
%     -------------------------
%     調整するパラメータ
%     -------------------------
%     th_offset = 0                   % スロットルオフセット
%     gain = [100;100;100;10]         % ゲイン（roll,pitch,yaw,thrust）
% 
%     -------------------------
%     調整ステップ量（1回の更新幅）
%     -------------------------
%     offset_step = 5
%     gain_step = [10;10;10;1]        % ゲインをどれだけ増やすか
%     gain_max = [600;600;600;40]     % ゲイン上限
% 
%     ---- オフセット自動取得（mode1）用 ----
%     time_accum = 0;          % 時間積算（初期ゼロ）
%     offset_interval = 0.15;  % offset を増加させる間隔（秒）
%     offset_max = 450;        % 安全上限（モーターが急上昇しない値）
% 
%     -------------------------
%     ログバッファ（最近の角速度などを保存）
%     -------------------------
%     log_buf
%     last_t = []                     % 前回評価した時間
%     best_score = Inf                % これまでの最良スコア
%     last_score = []                 % 最後に計算したスコア
% 
%     offset_fixed = false           % ベスト offset を固定したかどうか
%     offset_lock_threshold = 0.005  % offset 固定判断のためのスコア閾値
% 
%     axis_idx = 1                  % 1～4（Roll, Pitch, Yaw, Throttle）
%     waiting = false               % trial 評価中か
%     baseline = inf                % 比較スコア
%     trialVal = []                 % 試行中ゲイン値
%     best_param = struct('th_offset',0,'gain',zeros(4,1))     % ベスト offset / gain の保存場所
% end
% 
% 
% methods
%     ============================================================
%     コンストラクタ 
%     ============================================================
%     function obj = INPUTTRANSFORM_AUTOTUNE(self, mode)
%         if nargin<2, mode=0; end % modeが指定されなければ0（無効）
%         obj.self = self;
%         obj.mode = mode;
% 
%         origin側のゲイン・オフセットを初期値に採用
%         try
%             if isfield(self.input_transform,'origin') ...
%                && isprop(self.input_transform.origin,'param')
% 
%                 p = self.input_transform.origin.param;
% 
%                 if isfield(p,'th_offset'), obj.th_offset = p.th_offset; end
%                 if isfield(p,'gain'), obj.gain = p.gain; end
%             end
%         catch
%         end
% 
%         モニター生成
%         if obj.mode>0
%             try
%                 obj.monitor = AutoTuneMonitor();
%             catch
%                 obj.monitor = [];
%             end
%         end
% 
%         ログバッファを3秒分確保（25ms ≒ 40Hz を想定）
%         dt_guess = 0.025;
%         buflen = ceil(3/dt_guess);
%         obj.log_buf = struct(...
%             't',nan(1,buflen), ...
%             'w',nan(3,buflen), ...
%             'wn',nan(3,buflen), ...
%             'uthr',nan(1,buflen), ...
%             'idx',1, ...
%             'len',buflen );
%     end
% 
% 
%         ============================================================
%         メイン処理（入力変換 + 自動調整）
%         ============================================================
%         function  u = do(obj, varargin)
%         ============================================================
%           do() : INPUTTRANSFORM_AUTOTUNE のメイン処理
%           ・外部互換性維持のため varargin を使用
%           ・内部では t_struct, cha の2引数に統一
%         ============================================================
% 
%         % ------------------------------------------------------------
%         引数の取り出し（最重要：ここで互換性確保）
%         ------------------------------------------------------------
%         許可された最小構成：
%           do(obj, t_struct)
%           do(obj, t_struct, cha)
%           do(obj, t_struct, cha, 追加引数...)
% 
%         cha は使うのは第2引数のみ、他は無視する。
% 
%         if nargin < 2
%             error("do(): t_struct が指定されていません");
%         else
%             t_struct = varargin{1};
%         end
% 
%         if nargin < 3
%             cha = 's';      % デフォルト
%         else
%             cha = varargin{2};
%         end
%         cha が誤っていた場合の対策
%         if ~ismember(cha,{'q','s','a','f','l','t'})
%             cha='s';
%         end
% 
%         コントローラの出力 (T, Roll, Pitch, Yaw)
%         try
%             input = obj.self.controller.result.input;
%         catch
%             input = [0;0;0;0];
%         end
% 
%         ホバースラスト推定
%         try
%             P = obj.self.parameter.get();
%             hover_thrust_force = P(1)*P(9);
%         catch
%             hover_thrust_force = 0;
%         end
% 
%         現在状態とモデルの更新計算
%         try
%             wh = obj.self.estimator.result.state.w;
%             obj.self.estimator.model.do(t_struct, cha, obj.self.controller, obj.self.estimator, obj.self, 1);
%             whn = obj.self.estimator.model.state.w;
% 
%             モデル状態を元に戻す（シミュレーション干渉防止）
%             obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get);
%         catch
%             wh=zeros(3,1);
%             whn=zeros(3,1);
%         end
% 
%         ==============================
%         入力変換 (P制御)
%         ==============================
%         g = obj.gain;
%         offset = obj.th_offset;
% 
%         T_thr = input(1);
% 
%         roll, pitch, yaw の変換（角速度差をP制御）
%         uroll  = g(1)*(whn(1)-wh(1));
%         upitch = g(2)*(whn(2)-wh(2));
%         uyaw   = g(3)*(whn(3)-wh(3));
% 
%         thrust（推力→スロットル変換）
%         uthr   = max(0, g(4)*(T_thr-hover_thrust_force) + offset);
% 
%         THRUST2と同じ内容の変換
%         uroll = sign(uroll) * min(abs(uroll), 500) + obj.self.roll_offset;
%         upitch = sign(upitch) * min(abs(upitch), 500) + obj.self.pitch_offset;
%         uyaw = -sign(uyaw) * min(abs(uyaw), 300) + obj.self.yaw_offset; 
% 
%         結果を構造体として格納
%         obj.result = struct( ...
%             'roll_offset',uroll, ...
%             'pitch_offset',upitch, ...
%             'thrust_offset',uthr, ...
%             'yaw_offset',uyaw, ...
%             'aux1',1000,'aux2',0,'aux3',0,'aux4',1000 );
%         u = obj.result;
% 
% 
%         ============================================================
%         === 自動調整（autotune）処理： 't' フェーズのみ ===
%         ============================================================
%         if obj.mode>0 && cha=='t'
% 
%             現在時刻の取得
%             try
%                 tnow = t_struct.t;
%             catch
%                 tnow = posixtime(datetime("now"));
%             end
% 
%             -------------------------
%             ログ更新
%             -------------------------
%             idx = obj.log_buf.idx;
%             obj.log_buf.t(idx)   = tnow;% 時刻
%             obj.log_buf.w(:,idx) = wh(:);% 実モータ角速度
%             obj.log_buf.wn(:,idx)= whn(:); % 推定/正規化角速度
%             obj.log_buf.uthr(idx)= uthr; % フィルタ済みスロットル入力
%             obj.log_buf.idx = mod(idx,obj.log_buf.len)+1; % インデックスを循環させる
% 
%             初回は評価しない
%             if isempty(obj.last_t)
%                 obj.last_t = tnow;
%             end
% 
%             1秒ごとに評価（評価間隔）
%             if tnow - obj.last_t >= 1.0
% 
%                 最近1秒のデータを取得
%                 [~,wvec,wnvec,~] = obj.getRecentWindow(1.0);
% 
%                 % スコアを計算（揺れ + 不安定性）
%                 score = obj.evaluate_stability(wnvec, wvec);
%                 pos     = obj.self.sensor.result.state.p;
%                 pos_ref = obj.self.reference.result.state.p;
% 
%                 score = obj.evaluate_stability(wnvec, wvec, pos, pos_ref);
% 
%                 ============================
%                   MODE 1 & MODE 2 (改良版)
%                 ============================
% 
%                 時間差分（自動取得は t フェーズのみ）
%                 dt = max(0, tnow - obj.last_t);
%                 obj.time_accum = obj.time_accum + dt;
%                 obj.last_t = tnow;
% 
%                 ----------------------------
%                 MODE 1: オフセット自動取得
%                 ----------------------------
%                 if obj.mode == 1
% 
%                     すでにベスト値が決まったら固定
%                 if obj.offset_fixed
%                     obj.th_offset = obj.best_param.th_offset;
%                     return;
%                 end
% 
%                 一定間隔で offset を微増
%                 while obj.time_accum >= obj.offset_interval
%                     obj.time_accum = obj.time_accum - obj.offset_interval;
%                     obj.th_offset = min(obj.offset_max, obj.th_offset + obj.offset_step);
%                 end
% 
%                 --- スコアによるベスト判定（最重要） ---
%                 if score < obj.best_score
%                     obj.best_score = score;
%                     obj.best_param.th_offset = obj.th_offset;
%                 end
% 
%                 --- ベスト値が決まったら固定 ---
%                 offset が上限に近づいてスコア改善が止まったら
%                 if obj.th_offset >= obj.offset_max || ...
%                    abs(score - obj.best_score) < obj.offset_lock_threshold
%                     obj.offset_fixed = true;
%                     obj.th_offset = obj.best_param.th_offset;
%                     obj.mode = 0;     % mode1 終了
%                 end
% 
%                 return;
%                 end
% 
% 
%             ----------------------------
%             MODE 2: ゲイン自動取得 (ヒルクライム)
%             ----------------------------
%             if obj.mode == 2
% 
%                 軸インデックス初期化
%                 if isempty(obj.axis_idx), obj.axis_idx = 1; end
%                 i = obj.axis_idx;
% 
% 
%                 gain_vec = obj.gain;
% 
% 
%                 --- 新規試行開始 ---
%                 if ~obj.waiting
%                     trial = gain_vec;
% 
%                     上限超えないようにステップ計算
%                     step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
%                     trial(i) = trial(i) + step;
% 
%                     反映
% 
%                         obj.gain = trial;
% 
%                     保持
%                     obj.waiting = true;
%                     obj.baseline = obj.best_score;
%                     obj.trialVal = trial(i);
% 
%                     return;
% 
%                 --- 評価フェーズ ---
%                 else
%                     if score < obj.baseline - 1e-6     % 改善した
%                         obj.best_score = score;
%                         obj.best_param.gain = gain_vec;
%                     else                                % 改善なし → 元に戻す
% 
%                             obj.gain(i) = max(0, obj.gain(i) - obj.gain_step(i));
% 
%                     end
% 
%                     次の軸へ進む
%                     obj.waiting = false;
%                     obj.axis_idx = obj.axis_idx + 1;
%                     if obj.axis_idx > 4
%                         obj.axis_idx = 1;
%                     end
% 
%                     return;
%                 end
%             end
% 
% 
%             ----------------------------
%             MODE 0: 何もしない（GUI表示のみ）
%             ----------------------------
%             best_score を常に更新
%             if score < obj.best_score
%                 obj.best_score = score;
%                 obj.best_param.th_offset = obj.th_offset;
%                 obj.best_param.gain = obj.gain;
%             end
% 
% 
% 
%                 =======================
%                 GUI更新
%                 =======================
%                 if ~isempty(obj.monitor)
%                     s = sprintf( ...
%                         'Mode:%d Gain:[%.1f %.1f %.1f %.1f] Offset:%.1f  Score:%.4f Best:%.4f', ...
%                         obj.mode, g(1),g(2),g(3),g(4), obj.th_offset, score, obj.best_score);
%                     obj.monitor.update(s);
%                 end
% 
%                 obj.last_t = tnow;
%                 obj.last_score = score;
%             end
%         end
%     end
% 
% 
%     ============================================================
%     ログから最近 window_s 秒のデータを取得
%     ============================================================
%     function [tvec,wvec,wnvec,uthrvec] = getRecentWindow(obj, window_s)
%         buf=obj.log_buf;
% 
%         有効データのみ取得
%         valid = ~isnan(buf.t);
% 
%         t_all = buf.t(valid);
%         w_all = buf.w(:,valid);
%         wn_all= buf.wn(:,valid);
%         uthr_all = buf.uthr(valid);
% 
%         if isempty(t_all)
%             tvec=[]; wvec=[]; wnvec=[]; uthrvec=[];
%             return;
%         end
% 
%         時間でフィルタ
%         tcut = t_all >= (t_all(end)-window_s);
% 
%         tvec = t_all(tcut);
%         wvec = w_all(:,tcut);
%         wnvec= wn_all(:,tcut);
%         uthrvec = uthr_all(tcut);
%     end
% 
% 
%     ============================================================
%     スコア計算（揺れ + 不安定性）
%     ============================================================
%     function s = evaluate_stability(~, wn, w)
%         if isempty(wn) || isempty(w)
%             s = Inf;
%             return;
%         end
% 
%         % モデルとの差の大きさ（不安定性）
%         stability = sum( mean( (wn - w).^2 , 2) );
% 
%         % ローパス前後の揺れ量
%         vibration = sum( var([w; wn],0,2) );
% 
%         % 合計（重み0.5）
%         s = stability + 0.5*vibration;
%     end
% 
% 
% 
%     function s = evaluate_stability(~, wnvec, wvec, pos, pos_ref)
%     wn  = LPF後データ（NxM）
%     w   = LPF前データ（NxM）
%     pos = 現在位置  [x; y; z]
%     pos_ref = 目標位置 [x_ref; y_ref; z_ref]
%     pos=obj.self.estimator.result.state.p;
%     pos_ref=obj.self.reference.result.state.p;
% 
% 
%     データが足りないときは無限大（最悪評価）
%     if isempty(wnvec) || isempty(wvec) || isempty(pos) || isempty(pos_ref)
%         s = Inf;
%         return;
%     end
% 
% 
%     --------------------------
%     1. モデルとの差の大きさ（あなたの従来ロジックそのまま）
%     --------------------------
%     stability = sum( mean( (wnvec - wvec).^2 , 2) );
% 
%     --------------------------
%     2. ローパス前後の揺れ量（あなたの従来ロジック）
%     --------------------------
%     vibration = sum( var([wvec; wnvec], 0, 2) );
% 
%     --------------------------
%     3. 位置誤差（今回追加する部分） 
%     --------------------------
%     e_pos = pos_ref - pos;
%     pos_err = sum( e_pos.^2 );  % 二乗誤差
% 
%     --- 初期状態の "地面停止で最高点" を防ぐ処理 ---
%     高さが低すぎると正しく評価できないので無効化
%     if pos(3) < 0.1  
%         pos_err = 1000;  % 大きめの罰則（0 だと地面状態の方が優位になる）
%     end
% 
% 
%     --------------------------
%     4. 総合スコア
%         （重みは必要に応じて変更可能）
%     --------------------------
%     s = stability + 0.5*vibration + 2.0*pos_err;
% end
% 
% end
% end

classdef INPUTTRANSFORM_AUTOTUNE < handle
% INPUTTRANSFORM_AUTOTUNE
% - THRUST2 相当の推力→スロットル変換を内包
% - mode 0: 通常（変換のみ）
%        1: オフセット自動取得 (takeoff 't' 時のみ)
%        2: ゲイン自動取得   (takeoff 't' 時のみ)
% - do(obj,varargin) で外部互換を維持（THRUST2 と同じ呼び出し方で OK）
% - 戻り値は THRUST2 と同じ数値配列 [uroll, upitch, uthr, uyaw, aux1, aux2, aux3, aux4]
%
% 使い方:
% agent.input_transform = INPUTTRANSFORM_AUTOTUNE(agent, mode);
% u = agent.input_transform.do(t_struct, cha, logger, env, agent_list, i);

properties
    % -------------------------
    % 基本参照
    % -------------------------
    self            % drone / agent
    mode            % 0:off, 1:offset autotune, 2:gain autotune

    monitor         % GUI 表示用オブジェクト（任意）
    % 出力（数値配列互換性）および構造体版（デバッグ）
    result          % 数値配列（互換）
    result_ch       % 構造体版 {roll,pitch,thrust,yaw,aux1..aux4}

    % -------------------------
    % パラメータ（変換用）
    % -------------------------
    th_offset = 0                   % スロットルオフセット
    gain = [100;100;100;10]         % [roll,pitch,yaw,thrust]

    % -------------------------
    % autotune 関連
    % -------------------------
    offset_step = 5.0 %1.0
    offset_interval = 0.5 %0.15
    offset_max = 350

    gain_step = [10;10;10;1]
    gain_max = [600;600;600;40]

    time_accum = 0
    last_t = []
    eval_window_sec = 1.0           % スコア評価窓（秒）

    % ログバッファ
    log_buf

    % bookkeeping
    best_score = Inf
    best_param = struct('th_offset',0,'gain',zeros(4,1))
    last_score = []

    offset_fixed = false
    offset_lock_threshold = 0.005

    % gain autotune state
    axis_idx = 1
    waiting = false
    baseline = Inf
    trialVal = []

    % hover thrust (初期化時にセット)
    hover_thrust_force = 0

end

methods
    %% ---------------------------
    % コンストラクタ
    %% ---------------------------
    function obj = INPUTTRANSFORM_AUTOTUNE(self, mode)
        if nargin < 2, mode = 0; end
        obj.self = self;
        obj.mode = mode;

        % monitor 作成（存在すれば）
        if obj.mode > 0
            try
                obj.monitor = AutoTuneMonitor();
            catch
                obj.monitor = [];
            end
        else
            obj.monitor = [];
        end

        % origin (THRUST2) にパラメータがあれば初期値を引き継ぐ
        try
            if isfield(self, 'input_transform') && isfield(self.input_transform, 'origin') && isprop(self.input_transform.origin,'param')
                p = self.input_transform.origin.param;
                if isfield(p,'th_offset'), obj.th_offset = p.th_offset; end
                if isfield(p,'gain'), obj.gain = p.gain; end
            end
        catch
            % ignore
        end

        % hover thrust をパラメータから計算（THRUST2 と同じ）
        try
            P = self.parameter.get();
            obj.hover_thrust_force = P(1) * P(9);
        catch
            obj.hover_thrust_force = 0;
        end

        % ログバッファ初期化（3秒分程度）
        dt_guess = 0.025;
        buflen = ceil(3 / dt_guess);
        obj.log_buf = struct( ...
            't', nan(1,buflen), ...
            'w', nan(3,buflen), ...
            'wn', nan(3,buflen), ...
            'uthr', nan(1,buflen), ...
            'idx', 1, ...
            'len', buflen );
        
        % 初期出力
        obj.result = [500,500,0,500,1000,0,0,1000];
        obj.result_ch = struct('roll',500,'pitch',500,'thrust',0,'yaw',500,'aux1',1000,'aux2',0,'aux3',0,'aux4',1000);
    end


    %% ---------------------------
    % do(): main entry (互換のため varargin)
    %% ---------------------------
    function u = do(obj, varargin)
        % varargin expected: (t_struct, cha, logger, env, agent_list, i)
        % but accept flexible forms to remain compatible
        if nargin < 2
            error('INPUTTRANSFORM_AUTOTUNE.do requires at least t_struct');
        end
        t_struct = varargin{1};
        if numel(varargin) >= 2, cha = varargin{2}; 
        else 
            cha = 's'; 
        end

        % cha 保護
        if ~ismember(cha,{'q','s','a','f','l','t'}), cha = 's'; end

        % controller input の取得（呼び出しが controller 配列を渡す形に対応）
        % input = [0;0;0;0];
        try
            if numel(varargin) >= 5 && numel(varargin{5}) >= 1 && numel(varargin) >=6
                % common calling pattern: varargin{5}(varargin{6}).controller.result.input
                input = varargin{5}(varargin{6}).controller.result.input;
            else
                % fallback: try agent reference inside obj.self
                input = obj.self.controller.result.input;
            end
        catch
            input = [0;0;0;0];
        end

        % flight phase logic: if invalid, keep previous (we do not store flight_phase prop here)
        % for predict we need estimator etc.

        % -------------------------
        % prediction (same as THRUST2)
        % -------------------------
        try
            if cha == 't' || cha == 'f' || cha == 'l'
                wh = obj.self.estimator.result.state.w;
                % call model.do with same args to predict one step (use same signature)
                try
                    obj.self.estimator.model.do(varargin{:});
                catch
                    % sometimes different call signature; try a minimal call:
                    try
                        obj.self.estimator.model.do(t_struct, cha, obj.self.controller, obj.self.estimator, obj.self, 1);
                    catch
                        % give up prediction
                    end
                end
                whn = obj.self.estimator.model.state.w;
                % restore model state
                try
                    obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get);
                catch
                end
            else
                wh = zeros(3,1);
                whn = zeros(3,1);
            end
        catch
            wh = zeros(3,1);
            whn = zeros(3,1);
        end

        % -------------------------
        % THRUST2-like control calculation
        % -------------------------
        % choose gain/offset (we do not split gain_tl etc; unified gain used)
        g = obj.gain;
        offset = obj.th_offset;

        T_thr = input(1);

        % P control on angular velocity error
        uroll  = g(1) * (whn(1) - wh(1));
        upitch = g(2) * (whn(2) - wh(2));
        uyaw   = g(3) * (whn(3) - wh(3));

        % apply throttle conversion (hover compensation)
        uthr = max(0, g(4) * (T_thr - obj.hover_thrust_force) + offset);

        % get arming offsets (fallback values used if not available)
        try
            ro = obj.self.plant.arming_msg(1);
            po = obj.self.plant.arming_msg(2);
            yo = obj.self.plant.arming_msg(4);
        catch
            ro = 500; po = 500; yo = 500;
        end

        % saturate and add offsets (same limits as THRUST2)
        uroll  = sign(uroll) * min(abs(uroll), 500) + ro;
        upitch = sign(upitch) * min(abs(upitch), 500) + po;
        uyaw   = -sign(uyaw) * min(abs(uyaw), 300) + yo; % note sign inversion as in THRUST2

        % compose numeric output (same format as THRUST2)
        if cha == 't' || cha == 'f' || cha == 'l'
            numeric_out = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000];
        else
            numeric_out = [ro, po, 0, yo, 1000, 0, 0, 0];
        end

        % store both numeric and struct forms
        obj.result = numeric_out;
        obj.result_ch = struct('roll',numeric_out(1),'pitch',numeric_out(2),'thrust',numeric_out(3),'yaw',numeric_out(4), ...
                               'aux1',numeric_out(5),'aux2',numeric_out(6),'aux3',numeric_out(7),'aux4',numeric_out(8));

        % set return value for compatibility
        u = obj.result;

        % -------------------------
        % Autotune部分（takeoff 't' のみ実行）
        % -------------------------
        if obj.mode > 0 && cha == 't'
            % current time
            try
                tnow = t_struct.t;
            catch
                tnow = posixtime(datetime("now"));
            end

            % log update (circular buffer)
            idx = obj.log_buf.idx;
            obj.log_buf.t(idx)   = tnow;
            obj.log_buf.w(:,idx) = wh(:);
            obj.log_buf.wn(:,idx)= whn(:);
            obj.log_buf.uthr(idx)= numeric_out(3); % filtered throttle if you have filter (we don't here)
            obj.log_buf.idx = mod(idx, obj.log_buf.len) + 1;

            % if first time init last_t
            if isempty(obj.last_t)
                obj.last_t = tnow;
            end

            % evaluate every eval_window_sec (1.0s default)
            if tnow - obj.last_t >= obj.eval_window_sec
                % get recent window
                [~, wvec, wnvec, ~] = obj.getRecentWindow(obj.eval_window_sec);

                % retrieve positions for scoring (if available)
                try
                    pos = obj.self.estimator.result.state.p;        % current position [x;y;z]
                catch
                    pos = [NaN;NaN;NaN];
                end
                try
                    pos_ref = obj.self.reference.result.state.p;    % desired position
                catch
                    pos_ref = [NaN;NaN;NaN];
                end

                % compute score (uses position term)
                score = obj.evaluate_stability(wnvec, wvec, pos, pos_ref);

                % ---------- MODE 1: offset autotune ----------
                if obj.mode == 1
                    % if fixed already, keep fixed (no more autotune)
                    if obj.offset_fixed
                        % nothing to do except display
                    else
                        % accumulate time and increment offset every offset_interval
                        dt = max(0, tnow - obj.last_t);
                        obj.time_accum = obj.time_accum + dt;
                        while obj.time_accum >= obj.offset_interval
                            obj.time_accum = obj.time_accum - obj.offset_interval;
                            obj.th_offset = min(obj.offset_max, obj.th_offset + obj.offset_step);
                        end

                        % update best by score
                        if score < obj.best_score
                            obj.best_score = score;
                            obj.best_param.th_offset = obj.th_offset;
                        end

                        % decide lock: either at max offset or small improvement plateau
                        if obj.th_offset >= obj.offset_max || abs(score - obj.best_score) < obj.offset_lock_threshold
                            obj.offset_fixed = true;
                            obj.th_offset = obj.best_param.th_offset;
                            obj.mode = 0; % finish tuning
                        end
                    end
                end

                % ---------- MODE 2: gain autotune (hill-climb) ----------
                if obj.mode == 2
                    % ensure axis index initialized
                    if isempty(obj.axis_idx), obj.axis_idx = 1; end
                    i = obj.axis_idx;

                    if ~obj.waiting
                        % start trial: bump one axis by step (bounded by gain_max)
                        trial = obj.gain;
                        step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
                        trial(i) = trial(i) + step;

                        % apply trial gains for next evaluation
                        obj.gain = trial;

                        % set waiting state and baseline
                        obj.waiting = true;
                        obj.baseline = obj.best_score;
                        obj.trialVal = trial(i);
                    else
                        % evaluation phase: compare with baseline
                        if score < obj.baseline - 1e-6
                            % improvement: accept (best_score and best_param)
                            obj.best_score = score;
                            obj.best_param.gain = obj.gain;
                        else
                            % no improvement: revert the bump
                            obj.gain(i) = max(0, obj.gain(i) - obj.gain_step(i));
                        end

                        % proceed to next axis
                        obj.waiting = false;
                        obj.axis_idx = obj.axis_idx + 1;
                        if obj.axis_idx > 4, obj.axis_idx = 1; end
                    end
                end

                % update best_score bookkeeping if changed externally
                if score < obj.best_score
                    obj.best_score = score;
                    obj.best_param.th_offset = obj.th_offset;
                    obj.best_param.gain = obj.gain;
                end

                % GUI monitor update
                if ~isempty(obj.monitor)
                    try
                        s = sprintf('Mode:%d Gain:[%.1f %.1f %.1f %.1f] Offset:%.1f Score:%.4f Best:%.4f', ...
                            obj.mode, obj.gain(1),obj.gain(2),obj.gain(3),obj.gain(4), obj.th_offset, score, obj.best_score);
                        obj.monitor.update(s);
                    catch
                        % ignore monitor errors
                    end
                end

                % save last score/time
                obj.last_score = score;
                obj.last_t = tnow;
            end
        end % end autotune
    end % end do


    %% ---------------------------
    % getRecentWindow: ログバッファから最新 window_s 秒分を抽出
    %% ---------------------------
    function [tvec, wvec, wnvec, uthrvec] = getRecentWindow(obj, window_s)
        buf = obj.log_buf;

        % 有効データだけ
        valid = ~isnan(buf.t);
        t_all = buf.t(valid);
        w_all = buf.w(:,valid);
        wn_all = buf.wn(:,valid);
        uthr_all = buf.uthr(valid);

        if isempty(t_all)
            tvec = []; wvec = []; wnvec = []; uthrvec = [];
            return;
        end

        % 最新 window_s 秒分を抽出
        tcut = t_all >= (t_all(end) - window_s);
        tvec = t_all(tcut);
        wvec = w_all(:,tcut);
        wnvec = wn_all(:,tcut);
        uthrvec = uthr_all(tcut);
    end


    %% ---------------------------
    % evaluate_stability: モデル差 + 揺れ + 位置誤差 を統合したスコア
    %% ---------------------------
    function s = evaluate_stability(~, wnvec, wvec, pos, pos_ref)
        % wnvec, wvec: (3 x N) matrices
        % pos, pos_ref: 3x1 vectors
        if isempty(wnvec) || isempty(wvec) || isempty(pos) || isempty(pos_ref)
            s = Inf; return;
        end

        % model mismatch
        stability = sum(mean((wnvec - wvec).^2, 2));
        % vibration
        vibration = sum(var([wvec; wnvec], 0, 2));
        % pos error (squared)
        e_pos = pos_ref - pos;
        pos_err = sum(e_pos.^2);
        % avoid rewarding ground-resting: if too low, penalize
        if pos(3) < 0.1
            pos_err = 1000;
        end

        % weighted sum
        s = stability + 0.5 * vibration + 2.0 * pos_err;
    end
end
end
