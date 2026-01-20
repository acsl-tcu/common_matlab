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
% agent.input_transform = INPUTTRANSFORM_AUTOTUNE(agent, param);
% u = agent.input_transform.do(t_struct, cha, logger, env, agent_list, i);

%mode=1,mode=2でそれぞれコメントイン、アウトしなければならない箇所あり（これやらないとエラー、または暴走する可能性あり）
%1,最初のpropertiesでmode選択＋初期値等設定
%2,function内のth_offsetに関する部分(mode=1ではth_offset=obj.th_offset,mode=2ではth_offset=obj.param.th_offset、232行あたり)
%3.ベストスコア管理部分：mode1とmode2で分けてある（上記同様、595行あたり）
%4.モニター表示部分(660行あたり)

properties
    % -------------------------
    % 基本参照
    % -------------------------
    self            % drone / agent
    mode=1          % 0:off, 1:offset autotune, 2:gain autotune     %0は現状飛行不可能なので使わない
    monitor         % GUI 表示用オブジェクト（任意）
    % 出力（数値配列互換性）および構造体版（デバッグ）
    result          % 数値配列（互換）
    result_ch       % 構造体版 {roll,pitch,thrust,yaw,aux1..aux4}
    param
    flight_phase
    state
    % -------------------------
    % パラメータ（変換用）
    % -------------------------
    th_offset = 300                    % スロットルオフセット
    gain = [100;100;100;10]        % [roll,pitch,yaw,thrust][250;250;250;25]
    % th_offset = 336                %現在使用スロットルオフセット
    % gain = [400;400;400;40]　　　　 %現在使用ゲイン
    % -------------------------
    % autotune 関連
    % -------------------------
    offset_step = 5 %1.0　刻み値
    offset_interval = 1 %0.15
    step_change_offset = 590;  % 330に達したら
    offset_step_fine = 1;      % 1刻みにする
    offset_max = 700
    gain_step = [10;10;10;1]        % ゲインをどれだけ増やすか
    gain_max = [410;410;410;45]     % ゲイン上限
    time_accum = 0                  % 時間積算（初期ゼロ）
    last_t = []                     % offset を増加させる間隔（秒）
    eval_window_sec = 1.0           % スコア評価窓（秒）
    % ログバッファ
    log_buf
    % bookkeeping
    best_score = Inf% これまでの最良スコア
    best_param = struct('th_offset',0,'gain',zeros(4,1))% ベスト offset / gain の保存場所
    last_score = [] % 最後に計算したスコア
    offset_fixed = false% ベスト offset を固定したかどうか
    axis_idx = 1 % 1～4（Roll, Pitch, Yaw, Throttle）
    waiting = false% trial 評価中か
    baseline = Inf % 比較スコア
    trialVal = [] % 試行中ゲイン値
    % hover thrust (初期化時にセット)
    hover_thrust_force = 0
    % ----- AUTOTUNE mode1 (offset) 用 -----
    tuning_start_time = [];     % チューニング開始時刻
    smooth_score = [];          % スムージングしたスコア
    last_improve_time = [];     % 最後に改善があった時間

    trial_start_time
    eval_window_s=1.0;
    worsen_count
    enable_stop_height=0.4;
    extra_drop_threshold=0.001;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prev_offset%エラー等出たら消してみる
score_buffer
cooldown_s
cooldown_done
eval_start_time
score_drop_threshold=5;
mode2_start_time=NaN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%mode2（変更後）で使用する%%%%%%%%%%
tune_stage
no_improve_thr
no_improve_rpyaw
trial_delta
no_improve_count
%%%%%%%%%%%
end

methods
    %% ---------------------------
    % コンストラクタ
    % INPUTTRANSFORM_AUTOTUNE(self, param)
    %   - ドローン本体 self を受け取り、自動ゲイン調整機能の準備を行う
    %   - オフセット・ゲインの初期値を設定し、ログ・モニタを準備する
    %% ---------------------------
    function obj = INPUTTRANSFORM_AUTOTUNE(self, param)
        % --- 1) ドローン本体ハンドルの保存 ---
        obj.self = self;
        % --- 2) 基本パラメータの保存 ---
        obj.param = param;
        % --- 3) アーミング時のオフセットを利用して roll/pitch/yaw オフセット初期化 ---
        obj.param.roll_offset  = self.plant.arming_msg(1);
        obj.param.pitch_offset = self.plant.arming_msg(2);
        obj.param.yaw_offset   = self.plant.arming_msg(4);
        % --- 4) パラメータ（力学パラメータなど）を取得して格納 ---
        %     例：モーター推力定数、質量など
        obj.param.P = self.parameter.get();
        % --- 5) フライトフェーズ（q,s,a,f,l,t）を初期化 ---
        obj.flight_phase = 's';
        % --- 6) ホバリング時の必要推力（force）を計算 ---
        %     P(1): thrust coefficient,  P(9): motor speed at hover など
        P = self.parameter.get;
        obj.hover_thrust_force = P(1) * P(9);
        % --- 7) 推定器の現在の状態をコピー（state オブジェクトの複製）---
        obj.state = state_copy(self.estimator.result.state);
        % --- 8) オートチューンモニタ（GUI）があれば生成 ---
        %     mode>0 の時だけ表示のために monitor を用意
        if obj.mode > 0
            try
                obj.monitor = AutoTuneMonitor();   % GUI用
            catch
                obj.monitor = [];                  % GUIがない場合でも動くように
            end
        else
            obj.monitor = [];
        end
        % --- 9) 再度 hover thrust を計算（安全のため try/catch）---
        %     上の計算と同じだが、例外対応のため再記述されている
        try
            P = self.parameter.get();
            obj.hover_thrust_force = P(1) * P(9);
        catch
            obj.hover_thrust_force = 0;  % パラメータ取得に失敗した場合のfallback
        end
        % --- 10) ログバッファの生成（3秒分を確保）---
        %     t: 時刻ログ
        %     w : 推定角速度
        %     wn: 予測角速度
        %     uthr : thrust 入力
        %     idx: 現在書き込む位置
        dt_guess = 0.025;            % 推定サンプリング周期（40Hz）
        buflen = ceil(3 / dt_guess); % 約3秒分
        obj.log_buf = struct('t',nan(1,buflen),'w',nan(3,buflen),'wn',nan(3,buflen),'uthr',nan(1,buflen),'idx',1,'len', buflen );

        % --- 11) 初期出力（Roll,Pitch,Yaw=500 で安全な中央付近）---
        %     thrust=0 で完全停止、aux は自動飛行用の必須値
        obj.result = [500,500,0,500,1000,0,0,1000];

        % --- 12) 構造体形式の出力（同じ値）---
        obj.result_ch = struct('roll',500,'pitch',500,'thrust',0,'yaw',500,'aux1',1000,'aux2',0,'aux3',0,'aux4',1000 );
    end
    %% ---------------------------
    % do(): main entry (互換のため varargin)
    %% ---------------------------
    function u = do(obj, varargin)
    % DO()
    % INPUTTRANSFORM_AUTOTUNE の主処理
    % thrust2 と autotune を統合した制御入力生成関数
    %
    % 期待引数:
    %   (t_struct, cha, logger, env, agent_list, i)
    %   ただし過去互換のため柔軟な引数処理になっている
    % ---------------------------------------------------------
    % 1. t_struct の取り出し（必須）
    % ---------------------------------------------------------
        if nargin < 2
            error('INPUTTRANSFORM_AUTOTUNE.do requires at least t_struct');
        end
        t_struct = varargin{1};
        % cha（フライトフェーズ型番: q,s,a,f,l,t）を取得
        if numel(varargin) >= 2, cha = varargin{2}; 
        else 
            cha = 's'; % デフォルトは "s"（静止・安全状態）
        end
        % cha の安全確認（正しくない文字が来たら強制的に s にする）
        if ~ismember(cha,{'q','s','a','f','l','t'}), cha = 's'; end
        % controller input の取得（呼び出しが controller 配列を渡す形に対応）
        % input = [0;0;0;0];
        try
            if numel(varargin) >= 5 && numel(varargin{5}) >= 1 && numel(varargin) >=6
                 % agent_list(i).controller.result.input を取りに行くケース（多い）
                input = varargin{5}(varargin{6}).controller.result.input;
            else
                % fallback: obj.self から直接取得
                input = obj.self.controller.result.input;
            end
        catch
            % 取得失敗時は [0;0;0;0] を使う（安全策）
            input = [0;0;0;0];
        end
        % ---------------------------------------------------------
        % 3. 予測モデル（estimator.model）の一歩先予測
        %    THRUST2 と同じ処理
        % ---------------------------------------------------------
        try
            if cha == 't' || cha == 'f' || cha == 'l'
                % 現在の角速度 w
                wh = obj.self.estimator.result.state.w;
                    % ---- 予測ステップの実行 ----
                try
                    % 通常の呼び出し（THRUST2 と同じ引数構成）
                    obj.self.estimator.model.do(varargin{:});
                catch
                     % 互換性確保：引数が合わないときに最小セットで再呼び出し
                    try
                        obj.self.estimator.model.do(t_struct, cha, obj.self.controller, obj.self.estimator, obj.self, 1);
                    catch
                        % 最終的に予測ができなかった場合は無視して続行
                    end
                end
            % 予測後の角速度 w_next
                whn = obj.self.estimator.model.state.w;

            % ---- 予測前の state に戻す（モデル内部書き換えを戻す）----
                try
                    obj.self.estimator.model.state.set_state(obj.self.estimator.result.state.get);
                catch
                end
            else
                % フライトフェーズ s,q,a では予測を行わない
                wh = zeros(3,1);
                whn = zeros(3,1);
            end
        catch
            % 予測処理全体が失敗しても安全に落とす
            wh = zeros(3,1);
            whn = zeros(3,1);
        end
        % ---------------------------------------------------------
        % 4. THRUST2 互換の制御計算を実行
        % ---------------------------------------------------------
        % gain と throttle offset パラメータ
        g = obj.gain;
        offset=obj.th_offset;%mode1用
        % offset = obj.param.th_offset;%mode2用
        % thrust コマンド（外部 LQR/MPC の出力）
        T_thr = input(1);
        % ---- P制御（角速度制御）----
        uroll  = g(1) * (whn(1) - wh(1));
        upitch = g(2) * (whn(2) - wh(2));
        uyaw   = g(3) * (whn(3) - wh(3));
        % ---- thrust 計算（ホバリング推力補正 + オフセット）----
        uthr = max(0, g(4) * (T_thr - obj.hover_thrust_force) + offset);
        % ---------------------------------------------------------
        % 5. アーミング時のスティック中央値（roll/pitch/yaw オフセット）
        % ---------------------------------------------------------
        try
            ro = obj.self.plant.arming_msg(1);
            po = obj.self.plant.arming_msg(2);
            yo = obj.self.plant.arming_msg(4);
        catch
            ro = 500; po = 500; yo = 500;        % fallback（安全）
        end
        % ---------------------------------------------------------
        % 6. 各チャンネル saturate（THRUST2 と同じ制限）
        % ---------------------------------------------------------
        uroll  = sign(uroll) * min(abs(uroll), 500) + ro;
        upitch = sign(upitch) * min(abs(upitch), 500) + po;
        % yaw は符号が逆（THRUST2 の仕様に合わせている）
        uyaw   = -sign(uyaw) * min(abs(uyaw), 300) + yo;
        % ---------------------------------------------------------
        % 7. 出力を組み立て（aux チャンネル含む）
        %    cha によって thrust を出すかどうかを決める
        % ---------------------------------------------------------
        if cha == 't' || cha == 'f' || cha == 'l'
            % thrust 有効（通常飛行）
            numeric_out = [uroll, upitch, uthr, uyaw, 1000, 0, 0, 1000];
        else
            % thrust 無効（安全モード）
            numeric_out = [ro, po, 0, yo, 1000, 0, 0, 0];
        end
        % ---------------------------------------------------------
        % 8. 結果を obj.result として保存（構造体版も作成）
        % ---------------------------------------------------------
        obj.result = numeric_out;
        obj.result_ch = struct('roll',numeric_out(1),'pitch',numeric_out(2),'thrust',numeric_out(3),'yaw',numeric_out(4), ...
                               'aux1',numeric_out(5),'aux2',numeric_out(6),'aux3',numeric_out(7),'aux4',numeric_out(8));
       % 最終的な返り値
        u = obj.result;
        % -------------------------
        % Autotune部分（takeoff 't' のみ実行）
        % -------------------------
        if obj.mode > 0 && cha == 't'
            % ==== 現在時刻の取得 ====
            try
                tnow = t_struct.t; % 通常フライトループではここから取得
            catch
                tnow = posixtime(datetime("now"));% 例外時はシステム時刻
            end
            % ==== ログバッファ更新（リングバッファ） ====
            idx = obj.log_buf.idx;
            obj.log_buf.t(idx)   = tnow;
            obj.log_buf.w(:,idx) = wh(:);% 実角速度
            obj.log_buf.wn(:,idx)= whn(:); % 予測角速度（モデルから）
            obj.log_buf.uthr(idx)= numeric_out(3); % 出力スロットル(フィルタ実装時はそこで)
            obj.log_buf.idx = mod(idx, obj.log_buf.len) + 1;
            % ==== 初回のみ last_t を初期化 ====
            if isempty(obj.last_t)
                obj.last_t = tnow;
            end
            % ----------------------------------------
            % 一定間隔（eval_window_sec）ごとに評価実行
            % ----------------------------------------
            if tnow - obj.last_t >= obj.eval_window_sec
                 % 直近 eval_window_sec 秒のデータを取得
                [~, wvec, wnvec, ~] = obj.getRecentWindow(obj.eval_window_sec);
                % --- 位置情報取得 ---
                    pos = obj.self.estimator.result.state.p;         % 現在位置[x;y;z]
                    pos_ref = obj.self.reference.result.state.p;    % 目標位置
                    vel = obj.self.estimator.result.state.v;
                % ==== スコア評価（振動 + 安定性 + 位置誤差） ====
                score = obj.evaluate_stability(wnvec, wvec, pos, pos_ref,obj.mode, obj.axis_idx,vel);

                % ========================================
                %  mode = 1 : 自動オフセット取得
                % ========================================
                if obj.mode == 1
                    %----悪化回数カウンタの初期化-----
                    if isempty(obj.worsen_count)
                        obj.worsen_count = 0;
                    end
                    dt = max(0, tnow - obj.last_t);
                    % ---- 1. チューニング開始からの経過時間を計測 ----
                    if isempty(obj.tuning_start_time)
                        obj.tuning_start_time = tnow;
                    end
                    % elapsed = tnow - obj.tuning_start_time;
                    % ---- 2. offset を増加 ----
                    obj.time_accum = obj.time_accum + dt;
                    stop_rising = false;
                    while obj.time_accum >= obj.offset_interval
                        obj.time_accum = obj.time_accum - obj.offset_interval;
                        % 増加前の値を保存（悪化時に戻す用）
                        prev_offset = obj.th_offset;
                        % obj.th_offset = min(obj.offset_max, obj.th_offset + obj.offset_step);%下の細かい調整をする場合はこの行はコメント

                        %--------------- 細かく調整したい場合はこの部分を使って調整する------------
                        % --- 追加：オフセットが指定値を超えたら刻みを変更 ---
                        step_now = obj.offset_step;  % 通常の上昇幅
                        if ~isempty(obj.step_change_offset) && ~isempty(obj.offset_step_fine)
                            if obj.th_offset >= obj.step_change_offset
                                step_now = obj.offset_step_fine; % 以降は細かい刻み
                            end
                        end
                        obj.th_offset = min(obj.offset_max, obj.th_offset + step_now);
                        %---------------------------------------------------------------------

                        % ---- 増加直後の評価（悪化したら戻す） ----
                           score_diff=score-obj.last_score;
                            % if score_diff > obj.best_score + obj.score_drop_threshold
                            if score_diff > obj.score_drop_threshold
                                % 直前のオフセットに戻す
                                obj.th_offset = prev_offset;
                                stop_rising = false;   % 今回は採用しなかった]
                                break;
                            end
                            %高度~m以上なら小さい悪化でも終了
                            height=pos(3);
                            stop_gate=(height >= obj.enable_stop_height);
                            if stop_gate && (score_diff > obj.extra_drop_threshold)
                                obj.th_offset=prev_offset;
                                stop_rising=true;
                                break;
                            end
                    end

                    if stop_rising
                        obj.mode=0;
                        obj.th_offset=obj.best_param.th_offset;
                        return;
                    end
                    % ---- 3. 十分な評価時間を確保 ----
                    eval_window = 1.0;   % 最低1秒観測する
                    if (tnow - obj.last_improve_time) < eval_window
                        obj.last_score=score;
                        return;
                    end
                    % ---- 4. スコアの平滑化（ノイズに強くする） ----
                    % alpha = 0.25;  % 平滑化率
                    alpha = 0.1; % 平滑化率
                    obj.smooth_score = alpha*score + (1-alpha)*obj.smooth_score;
                    worsen_limit=2;%連続悪化で終了（２回）
                    worsen_eps=1e-6;%微小差無視
                    % ---- 5. 改善したら best_score 更新 ----
                    if obj.smooth_score < obj.best_score
                       obj.best_score = obj.smooth_score;
                       obj.best_param.th_offset = obj.th_offset;
                       obj.last_improve_time = tnow; % 最後に改善した時刻
                       %改善したので悪化カウンタをリセット
                       obj.worsen_count=0;
                    else
                        %改善していない
                        if obj.smooth_score > obj.best_score+worsen_eps
                            obj.worsen_count=obj.worsen_count+1;
                        end
                        %悪化が続いたら終了してbestに戻す
                        if obj.worsen_count>=worsen_limit
                            obj.th_offset=obj.best_param.th_offset;
                            obj.mode=0;
                            return;
                        end
                    end
                    obj.last_score=score;
                    % ---- 6. ロック判定 ----
                    % 改善が一定時間途絶えたら終了
                    height=pos(3);        %高度取得
                    threshold_height=0.2; %一定高度まで評価しない
                        if height>threshold_height && obj.th_offset >= obj.offset_max %|| abs(score - obj.best_score) < obj.offset_lock_threshold)
                            obj.offset_fixed = true;
                            obj.th_offset = obj.best_param.th_offset;
                            obj.mode = 0; % finish tuning
                        end
                end
                % =====================================================
                %                 MODE 2: ゲイン自動調整
                % =====================================================
                if obj.mode == 2

                    %================ 初期化 ================
                    if isnan(obj.mode2_start_time)
                        obj.mode2_start_time = tnow;
                    end
                    startup_hold = 6.0;
                    if (tnow - obj.mode2_start_time) < startup_hold
                        return;
                    end

                    if isempty(obj.score_buffer)
                        obj.score_buffer = [];
                    end
                    if isempty(obj.waiting)
                        obj.waiting = false;
                    end
                    if isempty(obj.axis_idx)
                        obj.axis_idx=4;
                    end

                    %--- 追加：調整ステージ ---
                    % tune_stage=1: throttleのみ → tune_stage=2: roll/pitch & yaw
                    if ~isprop(obj,'tune_stage') || isempty(obj.tune_stage)
                        obj.tune_stage = 1;
                    end

                    %--- 追加：改善なし回数（ステージ別） ---
                    if ~isprop(obj,'no_improve_thr') || isempty(obj.no_improve_thr)
                        obj.no_improve_thr = 0;
                    end
                    if ~isprop(obj,'no_improve_rpyaw') || isempty(obj.no_improve_rpyaw)
                        obj.no_improve_rpyaw = 0;
                    end


                    %================ スコアバッファ更新 ================
                    buffer_len = 50;
                    if obj.waiting && obj.cooldown_done
                        obj.score_buffer(end+1) = score;
                        if length(obj.score_buffer) > buffer_len
                            obj.score_buffer = obj.score_buffer(end-buffer_len+1:end);
                        end
                    end

                    %================ 終了条件パラメータ ================
                    thr_no_improve_limit   = 3;  % throttleは「改善が止まったら次へ」判定（調整可） 初期設定4
                    rpyaw_no_improve_limit = 4;  % roll/pitch & yaw は「改善なし2回で終了」

                    %====================================================
                    %  waiting=false : 新しい試験を開始
                    %====================================================
                    if ~obj.waiting

                        trial = obj.gain;
                        obj.trial_delta=zeros(size(obj.gain));

                        %----------- Stage 1 : Throttle -----------
                        if obj.tune_stage == 1
                            obj.axis_idx=4;
                            i = 4; % throttle
                            step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));

                            % もう上げられないなら stage2へ
                            if step < 1e-12
                                obj.tune_stage = 2;
                                obj.axis_idx = 1;
                                % obj.waiting = false;%必要か不明
                                % obj.score_buffer = [];%必要か不明
                                return;
                            end

                            trial(i) = trial(i) + step;
                            obj.trial_delta(i)=step;
                        end

                        %----------- Stage 2 : Roll/Pitch + Yaw -----------
                        if obj.tune_stage ==2
                            trial=obj.gain;

                                % Phase A : Roll + Pitch 同時
                                if obj.axis_idx ==1 
                                    step_r = min(obj.gain_step(1), obj.gain_max(1) - trial(1));
                                    step_p = min(obj.gain_step(2), obj.gain_max(2) - trial(2));
                                    if step_r < 1e-12 && step_p < 1e-12
                                        % roll/pitch これ以上無理なら yawへ
                                        obj.axis_idx = 3;
                                        % obj.waiting = false;%必要か不明
                                        % obj.score_buffer = [];%必要か不明
                                        return;
                                    end
                                    if step_r > 1e-12, trial(1) = trial(1) + step_r; obj.trial_delta(1)=step_r; end
                                    if step_p > 1e-12, trial(2) = trial(2) + step_p; obj.trial_delta(2)=step_p; end
                                    i = 1; % 記録用（trialValなど）

                                % Phase B : Yaw
                                elseif obj.axis_idx==3
                                    i = 3;
                                    step_y = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
                                    if step_y < 1e-12
                                        % yaw これ以上無理なら roll/pitchへ戻す
                                        obj.axis_idx = 1;
                                        % obj.waiting = false;%必要か不明
                                        % obj.score_buffer = [];%必要か不明
                                        return;
                                    end
                                    trial(i) = trial(i) + step_y;
                                    obj.trial_delta(i)=step_y;

                                else
                                    obj.axis_idx = 1;
                                    return;
                                end

                        end

                        %---- 試験ゲイン適用 ----
                        obj.gain = trial;
                        obj.waiting = true;

                        obj.score_buffer = [];

                        % baseline は「直近の最良」
                        obj.baseline = obj.best_score;

                        obj.trial_start_time = tnow;
                        obj.trialVal = obj.gain(i);

                        obj.cooldown_s = 0.4;
                        obj.cooldown_done = false;

                        return;
                    end

                    %====================================================
                    % waiting=true : 評価中
                    %====================================================
                    elapsed = tnow - obj.trial_start_time;

                    % クールダウン → 評価開始時刻を切る
                    if ~obj.cooldown_done
                        if elapsed < obj.cooldown_s
                            return;
                        else
                            obj.cooldown_done = true;
                            obj.eval_start_time = tnow;
                            return;
                        end
                    end

                    % 評価ウィンドウ待ち
                    eval_elapsed = tnow - obj.eval_start_time;
                    if eval_elapsed < obj.eval_window_s
                        return;
                    end

                    % スコア評価（平均化）
                    current_score = mean(obj.score_buffer);

                    % 改善判定
                    improved = (current_score < obj.baseline - 1e-6);

                    if improved
                        obj.best_score = current_score;
                        obj.best_param.gain = obj.gain;

                        % 改善が出たら、そのステージの「改善なし回数」をリセット
                        if obj.tune_stage == 1
                            obj.no_improve_thr = 0;
                        else
                            obj.no_improve_rpyaw = 0;
                        end

                    else
                        % 改善なし → 戻す
                        if obj.tune_stage == 1
                            % throttleは1段戻す（もしくはbestに戻すでもOK）
                            obj.gain(4) = max(0, obj.gain(4) - obj.gain_step(4));
                            obj.no_improve_thr = obj.no_improve_thr + 1;
                        else
                            if obj.axis_idx == 1
                                obj.gain(1) = max(0, obj.gain(1) - obj.gain_step(1));
                                obj.gain(2) = max(0, obj.gain(2) - obj.gain_step(2));
                            else
                                obj.gain(3) = obj.best_param.gain(3);
                            end
                            obj.no_improve_rpyaw = obj.no_improve_rpyaw + 1;
                        end
                        % if ~isempty(obj.trial_delta)
                        %     obj.gain=obj.gain-obj.trial_delta;
                        %     obj.gain=max(obj.gain,0);
                        % end
                        % if obj.tune_stage==1
                        %     obj.no_improve_thr=obj.no_improve_thr+1;
                        % else
                        %     obj.no_improve_rpyaw=obj.no_improve_rpyaw+1;
                        % end
                    end

                    % 評価終了 → 次試験へ
                    obj.waiting = false;

                    %----------- ステージ遷移・終了判定 -----------
                    if obj.tune_stage == 1
                        % throttle: 改善が止まったら stage2へ
                        if obj.no_improve_thr >= thr_no_improve_limit
                            obj.tune_stage = 2;
                            obj.axis_idx = 1;
                            obj.score_buffer = [];  % 切り替え時にスコアをクリア（推奨）
                        end
                    else
                        % roll/pitch & yaw: 交互に回す
                        if obj.axis_idx == 1
                            obj.axis_idx = 1;
                        else
                            obj.axis_idx = 3;
                        end


                        % 改善なし2回で終了
                        if obj.no_improve_rpyaw >= rpyaw_no_improve_limit
                            obj.mode = 0; % tuning 完了
                        end
                    end

                end
                %mode2以前までのものーーーーーーーーーーーーーーーーーーーーー
                % if obj.mode == 2
                %     % axis_idx 未初期化なら 1 (=roll)
                %     if isempty(obj.axis_idx) 
                %         obj.axis_idx = 1; 
                %     end
                %     if isnan(obj.mode2_start_time)
                %        obj.mode2_start_time=tnow;
                %     end
                %     startup_hold=6.0;
                %     if (tnow-obj.mode2_start_time)<startup_hold
                %         return;
                %     end
                %         if isempty(obj.no_improve_count)
                %             obj.no_improve_count = 0;   % 改善が途絶えた回数
                %         end
                %         if isempty(obj.score_buffer)
                %             obj.score_buffer = [];      % スコアバッファ初期化
                %         end
                % 
                %         % ========= スコアバッファに追加（最大 N 件） =========
                %         buffer_len = 50;   % 50 サンプル分のスコア平均
                %         obj.score_buffer(end+1) = score;
                %         if length(obj.score_buffer) > buffer_len
                %             obj.score_buffer = obj.score_buffer(end-buffer_len+1:end);
                %         end
                %     i = obj.axis_idx;
                %     if ~obj.waiting
                %         % ---- 試験開始：1軸だけゲインを増やす ----
                %         trial = obj.gain;
                %         % step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
                %                 switch obj.axis_idx
                %                     % ---------- Phase 1 : Roll + Pitch ----------
                %                     case 1
                %                             step_r = min(obj.gain_step(1), obj.gain_max(1) - trial(1));
                %                             step_p = min(obj.gain_step(2), obj.gain_max(2) - trial(2));
                %                             if step_r< 1e-12 && step_p< 1e-12
                %                                 obj.axis_idx=2;
                %                                 return;
                %                             end
                %                             if step_r > 1e-12, trial(1)=trial(1)+step_r;end
                %                             if step_p > 1e-12, trial(2)=trial(2)+step_r;end
                %                             i=1;
                %                     % ---------- Phase 2 : Yaw ----------
                %                     case 2
                %                         i = 3;
                %                         step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
                %                         if step < 1e-12
                %                             obj.axis_idx=3;
                %                             return;
                %                         end
                %                         trial(i) = trial(i) + step;
                %                     % ---------- Phase 3 : Throttle ----------
                %                     case 3
                %                         i = 4;
                %                         step = min(obj.gain_step(i), obj.gain_max(i) - trial(i));
                %                         if step < 1e-12
                %                             obj.axis_idx=1;
                %                             return;
                %                         end
                %                         trial(i) = trial(i) + step;
                %                     otherwise
                %                         obj.axis_idx=1;
                %                         return;
                %                 end
                %         % % もう上げられない軸はスキップ
                %         % if step < 1e-12
                %         %     obj.axis_idx = obj.axis_idx + 1;
                %         %     if obj.axis_idx > 4, obj.axis_idx = 1; end
                %         %     return;
                %         % end
                % 
                %         % trial(i) = trial(i) + step;
                % 
                %         obj.gain = trial;% 試験ゲイン適用
                %         obj.waiting = true;% 評価待ち状態へ
                %         obj.baseline = obj.best_score;% 比較用ベースライン
                %         obj.trial_start_time = tnow;   % 現在時刻記録
                %         obj.trialVal = trial(i);
                % 
                %         % ---- クールダウン時間（安定させる） ----
                %         obj.cooldown_s = 0.4;  % 0.4秒程度が妥当
                %         obj.cooldown_done = false;
                % 
                %     else
                %         % -----------------------------------------
                %         % waiting=true : 評価中
                %         % window_s 秒のデータが溜まるまで待つ
                %         % -----------------------------------------
                %         elapsed = tnow - obj.trial_start_time;
                %         % ---- (1) クールダウン中 → まだ評価しない ----
                %         if ~obj.cooldown_done
                %             if elapsed < obj.eval_window_s
                %                 % 評価にはまだ早い
                %                 return;
                %             else
                %                 obj.cooldown_done = true;
                %                 obj.eval_start_time = tnow; % 評価開始時刻
                %                 return;
                %             end
                %         end
                %         % ---- (2) 評価ウィンドウがまだ短い ----
                %         eval_elapsed = tnow - obj.eval_start_time;
                %         if eval_elapsed < obj.eval_window_s
                %             return;
                %         end
                %         % スコア評価（平均化）
                %         current_score = mean(obj.score_buffer);
                %         % ---- 試験評価中：baseline と比較 ----
                %         if score < obj.baseline - 1e-6
                %             % 改善 → 採用
                %             % obj.best_score = score;
                %             obj.best_score = current_score;
                %             obj.best_param.gain = obj.gain;
                %             % 改善があったのでカウンタリセット
                %             obj.no_improve_count = 0;
                %         else
                %             % 改善なし → 元に戻す
                %             if obj.axis_idx==1
                %                 obj.gain(1)=max(0,obj.gain(1)-obj.gain_step(1));
                %                 obj.gain(2)=max(0,obj.gain(2)-obj.gain_step(2));
                %             % obj.gain(i) = max(0, obj.gain(i) - obj.gain_step(i));
                %             else
                %             obj.gain(i) = obj.best_param.gain(i);
                %             end
                %             % 改善が無かった回数増加
                %             obj.no_improve_count = obj.no_improve_count + 1;
                %         end
                %         % 次の軸へ
                %         obj.waiting = false;
                %         obj.axis_idx = obj.axis_idx + 1;
                %         if obj.axis_idx > 3, obj.axis_idx = 1; end
                %         % ---- (4) 改善が一定回数なければ終了 ----
                %         if obj.no_improve_count >= 16   % 4軸 × 4サイクル = 16回で収束判定
                %             obj.mode = 0;  % tuning 完了
                %         end
                %     end
                % end

                % --- ベストスコア管理（他の処理で更新された場合も拾う） ---
                if score < obj.best_score
                    obj.best_score = score;
                    obj.best_param.th_offset = obj.th_offset;%mode1用
                    % obj.best_param.th_offset = obj.param.th_offset;%mode2用
                    obj.best_param.gain = obj.gain;
                end
                % ==== GUI モニター更新 ====
                if ~isempty(obj.monitor)
                    try
                        %mode1用
                        s = sprintf(['Mode:%d\n' 'Gain:[%.1f %.1f %.1f %.1f]\n' 'Offset:%.1f\n' 'Score:%.4f   (Best:%.4f)\n' 'BestOffset:%d\n' 'result_th:%.2f\n' 'param_th:%.2f\n'], ...
                            obj.mode, obj.gain(1),obj.gain(2),obj.gain(3),obj.gain(4), obj.th_offset, score, obj.best_score, obj.best_param.th_offset, obj.result(3),obj.param.th_offset);
                        %mode2用
                        % s = sprintf(['Mode:%d\n' 'Gain:[%.1f %.1f %.1f %.1f]\n' 'Offset:%.1f\n' 'Score:%.4f   (Best:%.4f)\n' 'BestGain:[%.1f %.1f %.1f %.1f]\n' 'stage:%d\n' 'throttle改善なし:%d\n' 'roll/pitch,yaw改善なし:%d\n'], ...
                        %     obj.mode, obj.gain(1),obj.gain(2),obj.gain(3),obj.gain(4), obj.th_offset, score, obj.best_score, obj.best_param.gain, obj.tune_stage,obj.no_improve_thr,obj.no_improve_rpyaw);
                        obj.monitor.update(s);
                    catch
                         % GUIエラーは無視
                    end
                end
                 % ==== 評価実行時刻・スコアを保存 ====
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
        % -------- 有効データの抽出 --------
        % NaN でない部分だけを取り出す（リングバッファ）
        valid = ~isnan(buf.t);
        t_all = buf.t(valid); % すべての時刻
        w_all = buf.w(:,valid);% 実測角速度 w
        wn_all = buf.wn(:,valid);% モデル予測角速度 wn
        uthr_all = buf.uthr(valid);% スロットル値
        % データが何もなければ空で返す
        if isempty(t_all)
            tvec = []; wvec = []; wnvec = []; uthrvec = [];
            return;
        end

        % -------- 最新 window_s 秒分のデータを切り出し --------
        % 最新時刻 t_all(end) から window_s 秒遡った時刻以上のデータを取得
        tcut = t_all >= (t_all(end) - window_s);
        tvec = t_all(tcut);
        wvec = w_all(:,tcut);
        wnvec = wn_all(:,tcut);
        uthrvec = uthr_all(tcut);
    end
    %% ---------------------------
    % evaluate_stability:
    % 数値が小さいほど良い（安定している）
    %% ---------------------------
    function s = evaluate_stability(~,wnvec, wvec, pos, pos_ref, mode, axis,vel)
    % wnvec, wvec: 各軸の角速度ログ（3×N）
    % pos, pos_ref: 現在位置と目標位置（3×1）
    % データが欠けている場合は評価不可 → 無限大 (極端に悪いスコア)
        if isempty(wnvec) || isempty(wvec) || isempty(pos) || isempty(pos_ref) ||isempty(mode) || isempty(axis) ||isempty(vel)
            s = Inf; return;
        end
   %modeによるスコア評価の切替（自動）
   % ===========mide1:オフセット調整=============
switch mode
    % =====================================================
    % MODE 1 : 推力オフセット（th_offset）取得用の評価 → 高度方向（z）のみを見る
    % =====================================================
    case 1
        % 現在高度と目標高度
        z     = pos(3);
        z_ref = pos_ref(3);

        % ---- パラメータ設定 ----
        z_min  = 0.2;    % 最低評価高度 [m]
        k_low  = 200;    % 低高度時の強い罰則
        k_over = 5.0;    % 目標高度を超えたときの罰則重み

        % ---------------------------------------------
        % (1) 低高度判定 離陸直後や地面付近で「安定している」と誤認しないための処理
        % ---------------------------------------------
        if z < z_min
            % 高度が低すぎる場合は強制的に悪いスコア → オフセット確定を防ぐ
            s = k_low + (z_ref - z)^2;
            return;
        end
        % ---------------------------------------------
        % (2) 通常時の高度誤差評価
        % ---------------------------------------------
        z_err = z_ref - z;
        s = z_err^2;   % 目標高度との二乗誤差
        % ---------------------------------------------
        % (3) 目標高度を超えた場合の追加ペナルティ
        %     → 上がりすぎるオフセットを防ぐ
        % ---------------------------------------------
        if z > z_ref
            s = s + k_over * (z - z_ref)^2;
        end
    % =====================================================
    % MODE 2 : ゲイン調整用の評価 → 調整軸ごとに評価指標を切り替える
    % =====================================================
    case 2
    % =====================================================
    % MODE2 : ゲイン調整
    %  - roll/pitch : モデル誤差 + 振動 + ピーク + 入力荒さ + 高度罰則
    %  - yaw        : 振動 + 角速度エネルギ + ピーク + 入力荒さ(弱) + 高度罰則
    %  - throttle   : 高度誤差 + 低高度ペナルティ（従来通り）
    % =====================================================

    % ---------- (追加) 高度罰則（soft gate） ----------
    % 高度が目標帯域に入っていない時は、姿勢評価が採用されにくいようにスコアを増やす
    z     = pos(3);
    z_ref = pos_ref(3);

    z_min  = 0.20;   % [m] これより低いと強い罰則（地面付近の誤判定防止）
    z_band = 0.10;   % [m] 目標高度の許容帯（±10cmの外は罰則を増やす）

    ez = z_ref - z;
    z_err = ez^2;

    if z < z_min
        lambda = 50;      % 強罰則
    elseif abs(ez) > z_band
        lambda = 10;      % 中罰則
    else
        lambda = 1;       % 通常
    end

    Jz = lambda * z_err;  % (追加) 姿勢評価に足す高度罰則項

    % ---------- (追加) 重み（調整しやすいように分離） ----------
    w_model = 1.0;
    w_vib   = 0.5;
    w_peak  = 0.2;
    %位置と速度
    % px=pos(1);
    % py=pos(2);
    % vx=vel(1);
    % vy=vel(2);

        switch axis
            % -------------------------------------------------
            % roll / pitch ゲイン調整
            % ・モデル追従性
            % ・振動の少なさ
            % -------------------------------------------------
            case {1,2}   % roll, pitch
                idx = [1 2];  % roll, pitch 成分
                % ---- モデル誤差 ----
                % 実機角速度とモデル予測角速度の差
                err = wnvec(idx,:) - wvec(idx,:);
                model_err = mean(err(:).^2);
                % ---- 振動評価 ----
                % 角速度の分散（揺れの大きさ）
                vib = mean(var(wvec(idx,:), 0, 2));
                % % % % % ---- (追加) ピーク評価 ----
                % % % % % ホバリングでも差が出やすい（過渡が荒いと悪化）
                peak = max(abs(wvec(idx,:)), [], 'all');
                %位置高周波成分＋相対速度
                % px_hp=px-movmean(px,round(1/dt_guess));
                % py_hp=py-movmean(py,round(1/dt_guess));
                % J=mean(1.0*(vx.^2+vy.^2)+0.2*(px_hp.^2+py_hp.^2));
                % ---- 合成スコア ----
                s = w_model*model_err + w_vib*vib + w_peak*peak + Jz;
            % -------------------------------------------------
            % yaw ゲイン調整
            % ・ヨー方向の荒れ（回転の滑らかさ）
            % -------------------------------------------------
            case 3   % yaw
                yaw_rate = wvec(3,:);     % ヨー角速度
                % ---- 振動成分 ----
                vib = var(yaw_rate);
                % ---- 回転の荒さ ----
                % 大きな角速度が続くとペナルティ
                yaw_rate_penalty = mean(yaw_rate.^2);
                % ---- (追加) ピーク ----
                peak = max(abs(yaw_rate));
                %追加項目
                % px_hp=px-movmean(px,round(1/dt_guess));
                % py_hp=py-movmean(py,round(1/dt_guess));
                % r_hp=yaw_rate-movmean(yaw_rate,round(1/dt_guess));
                % J=mean(1.0*(vx.^2+vy.^2)+0.2*(px_hp.^2+py_hp.^2)+0.5*(r_hp.^2));
                % ---- 合成スコア ----
                s = vib + 0.3*yaw_rate_penalty + 0.2*peak + Jz;
            % -------------------------------------------------
            % throttle ゲイン調整
            % ・高度追従性能のみを見る
            % -------------------------------------------------
            case 4   % throttle
                % 現在高度と目標高度
                z     = pos(3);
                z_ref = pos_ref(3);
                % ---- 高度誤差 ----
                e_z   = z_ref - z;
                z_err = e_z^2;
                % % % % ---- 低高度ペナルティ ----
                % 地面付近で安定と誤認しないため
                low_alt_penalty = exp(-5*z) * 50;
                % ---- 合成スコア ----
                s = z_err + low_alt_penalty;
        end
end
    end
end
end