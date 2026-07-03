classdef MOTIVE < handle
    % Motive用クラス：登録されたエージェントの位置と姿勢がわかる
    %  sensor.motive = MOTIVE(self, ~)
    %       self : agent
    %  rigid_id : [1,2,3...] : 指定したrigid bodyのp, qをresult.state(i)に登録
    %  state_list : [["p","q"],"p",...] : 指定したrigid_body の指定したプロパティをoutputベクトルに登録

    properties
        result
        state
        self
        old_time
        rigid_id % rigid body indices
        motive
        initq
        state_list
        q_type
        output_func

        bias % 定常偏差補正(hovering bias correction)関連をまとめた構造体
             %   .enable      : true: 補正機能を有効化
             %   .q           : 現在適用中の補正quaternion（q_true = q_reported * bias.q）
             %   .q_target    : ホバリング検出時に推定された補正quaternionの目標値
             %   .tau         : bias.qをbias.q_targetへ追従させる時定数[s]（大きいほど緩やか）
             %   .ref_time    : referenceが一定とみなす継続時間しきい値 [s]
             %   .att_time    : 姿勢(roll,pitch)が一定とみなす継続時間しきい値 [s]
             %   .sigma_deg   : 姿勢変動の許容幅 σ [deg]
             %   .dt          : 制御周期[s]（バッファサイズ算出用）
             %   .ref_buffer  : [t, ref...] を格納する固定長循環バッファ（未使用行はNaN）
             %   .ref_cap     : ref_bufferの確保サイズ（行数）
             %   .ref_count   : ref_bufferの有効サンプル数（capを超えない）
             %   .ref_idx     : ref_bufferの次回書き込み位置
             %   .att_buffer  : [t, roll, pitch] を格納する固定長循環バッファ（未使用行はNaN）
             %   .att_cap     : att_bufferの確保サイズ（行数）
             %   .att_count   : att_bufferの有効サンプル数（capを超えない）
             %   .att_idx     : att_bufferの次回書き込み位置
             %   .is_hovering : ホバリング検出フラグ
             %   .calibrated  : bias.q_targetの算出が既に一度実行済みかどうか
             %                  （trueになった後は再度ホバリングを検出してもq_targetを再計算しない）
             %   .last_t      : 前回update_bias呼び出し時刻（dt計算用）
    end

    methods

        function obj = MOTIVE(self, motive, dt, args)
            arguments
                self
                motive % required
                dt
                args.rigid_id = 1;
                args.q_type = 3;
                args.output_func = [];
                args.state_list = {["p","q"]}; % outputを構成する状態リスト {["p","q"],"p"}：rigid_id1からp,q、 rigid_id2からpを取りoutputを構成する
                args.initial_yaw_angle = 0;
                args.initq = [];
                % --- 定常偏差補正のパラメータ ---
                args.bias_enable = true;
                args.bias_tau = 1.0;       % [s] 補正を反映する時定数（緩やかさ）
                args.bias_ref_time = 2.0;  % [s] referenceが一定（完全一致）とみなす時間
                args.bias_att_time = 2.0;  % [s] 姿勢が一定とみなす時間
                args.bias_sigma_deg = 0.01; % [deg] 姿勢変動の許容幅 σ
            end

            %%% Output equation %%%
            obj.self = self;
            obj.motive = motive;
            obj.rigid_id = args.rigid_id;
            obj.q_type = string(args.q_type);
            obj.output_func = args.output_func;
            if length(args.state_list) < length(args.rigid_id)
                error("MOTIVE SENSOR: number of state_list is too short.");
            else
                if iscell(args.state_list)
                    obj.state_list = args.state_list;
                else
                    obj.state_list = {args.state_list};
                end
                if isempty(args.initq)
                    obj.initq = quaternion(Eul2Quat([0;0;args.initial_yaw_angle])');
                else
                    obj.initq = args.initq;
                end
                for i = 1:length(obj.rigid_id)
                    list = obj.state_list{i};
                    obj.result.state(i) = STATE_CLASS(struct('state_list', list, "num_list", obj.get_num_list(list)));
                end
            end

            % --- 定常偏差補正の初期化 ---
            obj.bias.enable = args.bias_enable;
            obj.bias.tau = args.bias_tau;
            obj.bias.ref_time = args.bias_ref_time;
            obj.bias.att_time = args.bias_att_time;
            obj.bias.sigma_deg = args.bias_sigma_deg;
            obj.bias.dt = dt;
            obj.bias.q = quaternion(1, 0, 0, 0);        % 恒等回転（補正なし）で初期化
            obj.bias.q_target = obj.bias.q;
            obj.bias.is_hovering = false;
            obj.bias.calibrated = false;
            obj.bias.last_t = [];

            % 循環バッファのサイズを確保しておく（dtより速いループでも溢れないよう余裕(x2)を持たせる）
            n_ref = max(2, ceil(obj.bias.ref_time * 2 / dt) + 1);
            n_att = max(2, ceil(obj.bias.att_time * 2 / dt) + 1);
            obj.bias.ref_cap = n_ref;
            obj.bias.att_cap = n_att;
            obj.bias.ref_buffer = nan(n_ref, 5); % 列: [t, x_d, y_d, z_d, yaw_d]
            obj.bias.att_buffer = nan(n_att, 3); % 列: [t, roll, pitch]
            obj.bias.ref_count = 0;
            obj.bias.att_count = 0;
            obj.bias.ref_idx = 1;
            obj.bias.att_idx = 1;
        end
        function result = do(obj, varargin)
            % varargin = {{TIME}, {cha}, {LOGGER}, {env}, {agent}, {1}}

            data = obj.motive.result;
            q = zeros(4, 1);
            output = [];

            % --- 定常偏差補正に使うref, tを取得（存在しなければ今回はスキップ） ---
            t_now = varargin{1}.t;
            if obj.bias.enable
                ref_now = [];
                try
                    if ~isempty(obj.self.reference) && ~isempty(obj.self.reference.result)
                        ref_now = obj.self.reference.result.state.xd(1:4); % [x_d;y_d;z_d;yaw_d]
                    end
                catch
                    ref_now = [];
                end
                has_bias_input = ~isempty(ref_now);
            else
                has_bias_input = false;
            end

            for i = 1:length(obj.rigid_id)
                % for i = length(obj.rigid_id):-1:1
                id = obj.rigid_id(i);
                list = obj.state_list{i};
                for j = list
                    if any(j == "q")
                        tmpq = quaternion(data.rigid(id).q');
                        tmpq = conj(obj.initq) * tmpq;

                        % --- 定常偏差補正（ホバリング検出→緩やかにbias反映） ---
                        if obj.bias.enable
                            if has_bias_input
                                obj.update_bias(ref_now, t_now, tmpq);
                            end
                            tmpq = tmpq * obj.bias.q; % q_true = q_reported * bias.q
                        end

                        [q(1) q(2) q(3) q(4)] = parts(tmpq);
                        obj.result.state(i).set_state("q", q);
                        if obj.q_type == "3"
                            output = [output;Quat2Eul(q)];
                        else
                            output = [output;q];
                        end
                    end
                    if any(j == "p")
                        obj.result.state(i).set_state("p", data.rigid(id).p);
                        output = [output;data.rigid(id).p];
                    end
                end
            end
            if ~isempty(obj.output_func)
                output = obj.output_func(obj,data);
            end
            % obj.result.rigid = data.rigid;
            % obj.result.feature = data.marker;
            % obj.result.feature_num = data.marker_num;
            % obj.result.local_feature = data.local_marker{id};
            % obj.result.on_feature_num = data.local_marker_nums(id);
            % obj.result.dt = data.time - obj.old_time;
            obj.result.output = output;
            % obj.old_time = data.time;
            result = obj.result;
        end % function do

        function show(obj, varargin)

            if isempty(obj.result)
                disp("do measure first.");
            end

        end

        function update_bias(obj, ref, t, q_meas)
            % 定常偏差補正のためのホバリング検出＆bias更新
            % 【入力】ref    : reference値 [x_d;y_d;z_d;yaw_d]（一定性判定・yaw基準に使用）
            %        t      : 現在時刻[s]
            %        q_meas : 現在の姿勢quaternion
            %
            % 原因：Vicon上で剛体を定義した瞬間の姿勢を無条件に基準(0)として
            %       扱ってしまうため、キャリブレーション時に機体が水平でなかった分だけ
            %       恒常的なroll/pitchオフセットが乗る。このオフセットは
            %       「reportされる姿勢 q_reported(t) = q_true(t) * bias.q」
            %       という形（世界座標基準で固定されたbias.q、右から作用）で表せる。
            %       （bias.qはyawに依存せず一定 -> 一度求めれば全方位で有効）
            %
            % 条件：
            %   ・reference [x_d;y_d;z_d;yaw_d] がbias.ref_time秒以上、完全に一致
            %   ・roll,pitchがbias.att_time秒以上、bias.sigma_deg以内で一定
            % を満たした瞬間の実測quaternionから、
            % 「その瞬間roll=pitch=0、yaw=yaw_d（現在のyawではなくreferenceのyaw）」
            % となるようなbias.qを逆算し、bias.tauの時定数でbias.q_targetへ緩やかに追従させる（slerp）。

            [q1,q2,q3,q4] = parts(q_meas);
            eul = Quat2Eul([q1,q2,q3,q4]'); % [roll; pitch; yaw]
            roll = eul(1);
            pitch = eul(2);

            % --- refバッファへ循環書き込み ---
            obj.bias.ref_buffer(obj.bias.ref_idx, :) = [t, ref(:)'];
            obj.bias.ref_idx = mod(obj.bias.ref_idx, obj.bias.ref_cap) + 1;
            obj.bias.ref_count = min(obj.bias.ref_count + 1, obj.bias.ref_cap);

            % --- attバッファへ循環書き込み ---
            obj.bias.att_buffer(obj.bias.att_idx, :) = [t, roll, pitch];
            obj.bias.att_idx = mod(obj.bias.att_idx, obj.bias.att_cap) + 1;
            obj.bias.att_count = min(obj.bias.att_count + 1, obj.bias.att_cap);

            % referenceは許容幅を設けず、完全に一致しているか(tol=0)で判定する
            ref_const = obj.check_window_constant(obj.bias.ref_buffer, obj.bias.ref_count, obj.bias.ref_time, 0);
            att_const = obj.check_window_constant(obj.bias.att_buffer, obj.bias.att_count, obj.bias.att_time, deg2rad(obj.bias.sigma_deg));

            if ref_const && att_const
                if ~obj.bias.is_hovering
                    obj.bias.is_hovering = true;
                    if ~obj.bias.calibrated
                        % ホバリング開始を検知（初回のみ）：この瞬間のq_measを使ってbiasを逆算
                        % q_true = q_reported * bias.q が roll=pitch=0, yaw=yaw_d(理想姿勢)となるように
                        % bias.q = conj(q_reported) * q_zero_rp(yaw_d)
                        yaw_d = ref(4); % reference yaw（現在yawではなく目標yaw）
                        q_zero_rp = quaternion(Eul2Quat([0; 0; yaw_d])');
                        obj.bias.q_target = conj(q_meas) * q_zero_rp;
                        obj.bias.calibrated = true;
                    end
                end
            else
                obj.bias.is_hovering = false;
            end

            % --- 緩やかな追従：bias.qをbias.q_targetへslerpで近づける ---
            if isempty(obj.bias.last_t)
                dt = 0;
            else
                dt = t - obj.bias.last_t;
            end
            obj.bias.last_t = t;

            if dt > 0 && obj.bias.tau > 0
                alpha = min(1, dt / obj.bias.tau);
                obj.bias.q = slerp(obj.bias.q, obj.bias.q_target, alpha);
            end
        end %function update_bias

    end
    methods (Access = private)

        function tf = check_window_constant(obj, buf, valid_count, win, tol)
            % buf         : 循環バッファ本体 [t, v1, v2, ...]（未使用行はNaN）
            % valid_count : 有効サンプル数（capを超えない。書き込みは循環するため
            %               行の並び順＝時系列順とは限らない）
            % win         : 判定に用いる時間窓[s]
            % tol         : 許容変動幅（スカラー、または各列に対応する行ベクトル/列ベクトル）
            %
            % 循環バッファのため「先頭行が最古」という前提を置かず、min/maxで評価する。
            if valid_count == 0
                tf = false;
                return;
            end
            rows = buf(1:valid_count, :);
            t_newest = max(rows(:, 1));
            t_oldest = min(rows(:, 1));
            if t_newest - t_oldest < win
                tf = false; % バッファ全体でもまだwin秒分のデータが溜まっていない
                return;
            end
            % 直近win秒分のデータのみを対象に変動をチェック
            mask = rows(:, 1) >= t_newest - win;
            vals = rows(mask, 2:end);
            tf = all(max(vals, [], 1) - min(vals, [], 1) <= tol(:)');
        end

        function num_list = get_num_list(obj, list)
            num_list = zeros(1, numel(list));
            for k = 1:numel(list)
                name = list(k);
                if name == "p"
                    num_list(k) = 3;
                elseif name == "q"
                    num_list(k) = 4;
                else
                    error("MOTIVE:UnsupportedState", "Unsupported state entry: %s", name);
                end
            end
        end
    end

end %classdef MOTIVE