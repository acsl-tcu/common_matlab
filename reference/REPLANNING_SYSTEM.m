classdef REPLANNING_SYSTEM < handle
    % REPLANNING_SYSTEM  障害物読込と模擬センサの統括だけを行う。
    %
    % 役割:
    %   (1) 既存の OBSTACLE_INFORMATION.definitions() を Flight 毎に1回読む。
    %   (2) 模擬センサへ検知半径・ログフラグ・障害物の原定義を渡す。
    %   (3) 各時刻の現在EKF機体位置から検知結果を作り、Sim LOGGER へ渡す。
    %
    % このクラスは軌道/制御/回避/予測/安全証明を実装しない。
    % OBSTACLE_INFORMATION と OBSTACLE_SHAPES は変更せず、読み取るだけ。
    %
    % 実Flightでの使用例:
    %   replanner = REPLANNING_SYSTEM(options);  % Flight前に1回
    %   ...  % 同じtickでMOTIVE→EKF→通常の参照→HLC→plant
    %   replanner.update(time, agent);          % Sim LOGGER.logging の直前
    %   logger.logging(time, cha, agent, []);   % 既存Simの標準記録
    %
    % updateがEKFの位置取得・模擬センサ・reference.resultへの格納を統括する。
    % runは個別のセンサや障害物ログを操作しない。Sim LOGGERは変更しない。
    % HLCや通常参照の出力は変更しない。

    properties (SetAccess=private)
        obstacles
        load_count=0
        frame_count=0
        load_time_ms=NaN
        total_frame_time_ms=0
        last_logged_time_s=NaN
        SENSOR_RANGE_M=20
        last_sensor_result=struct() % ログOFFでも最新の検知結果は保持
        preflight_warmup_ms=NaN % Flight/Armingより前の準備時間（周期時間に算入しない）
        preflight_warmup_ready=false
        preflight_warmup_distance_checks=0
    end

    properties
        ENABLE_ALL_LOG=true
        LOG_OBSTACLES=true
        ENABLE_SENSOR=true
        LOG_SENSOR=true
        LOG_REPLANNER=true % update()単独の計測・Sim LOGGERへの記録
    end

    properties (Access=private)
        sensor
        static_frame
        all_static=false
        replanner_timing_count=0
        replanner_timing_total_ms=0
        replanner_timing_max_ms=0
        % 一時診断：最初の20回のupdateを工程別計測。恒久実装には残さない。
        diagnostic_stage_ms=nan(20,7)
        diagnostic_time_s=nan(20,1)
        diagnostic_total_ms=nan(20,1)
        diagnostic_logged_ms=nan(20,1)
        diagnostic_count=0
    end

    methods
        function obj=REPLANNING_SYSTEM(options)
            if nargin<1,options=struct();end
            if ~isstruct(options)||~isscalar(options)
                error('REPLANNING_SYSTEM:Options','設定はスカラー構造体で指定してください。');
            end
            allowed={'ENABLE_ALL_LOG','LOG_OBSTACLES','ENABLE_SENSOR', ...
                'LOG_SENSOR','LOG_REPLANNER','SENSOR_RANGE_M'};
            names=fieldnames(options);
            for k=1:numel(names)
                field=names{k};value=options.(field);
                if ~ismember(field,allowed)
                    error('REPLANNING_SYSTEM:UnknownOption','未知の設定項目です: %s',field);
                end
                if strcmp(field,'SENSOR_RANGE_M')
                    REPLANNING_SYSTEM.check_range(value);
                    obj.SENSOR_RANGE_M=double(value);
                else
                    REPLANNING_SYSTEM.check_flag(value,field);
                    obj.(field)=logical(value);
                end
            end

            stopwatch=tic;
            obj.obstacles=OBSTACLE_INFORMATION.definitions();
            OBSTACLE_INFORMATION.validate_list(obj.obstacles);
            obj.load_count=1;
            obj.load_time_ms=1000*toc(stopwatch);
            obj.all_static=all(arrayfun(@(o) strcmp(o.motion,'static'),obj.obstacles));
            if obj.all_static
                obj.static_frame=struct('time_s',0,'obstacles',obj.obstacles);
            end

            % センサの準備は一度だけ。形状定義側へ手を入れない。
            obj.sensor=SIMULATED_OBSTACLE_SENSOR(obj.obstacles,obj.SENSOR_RANGE_M, ...
                obj.ENABLE_ALL_LOG&&obj.LOG_SENSOR);
            % 初回実行の関数準備を、Arming/Takeoff/Flightの前に済ませる。
            % 本番センサのscan_count/ID履歴/LOGGERは一切進めない。
            if obj.ENABLE_SENSOR
                obj.preflight_sensor_warmup();
            end
        end

        function yes=logging_enabled(obj)
            yes=obj.ENABLE_ALL_LOG&&obj.LOG_OBSTACLES;
        end

        function frame=log_frame(obj,time_s)
            % 既存Flightの障害物元定義ログを互換維持。センサ結果は混入しない。
            REPLANNING_SYSTEM.check_time(time_s);
            if ~obj.logging_enabled(),frame=[];return;end
            stopwatch=tic;
            if obj.all_static
                frame=obj.static_frame;
                frame.time_s=double(time_s);
            else
                frame=OBSTACLE_INFORMATION.log_frame(time_s);
            end
            obj.frame_count=obj.frame_count+1;
            obj.last_logged_time_s=double(time_s);
            obj.total_frame_time_ms=obj.total_frame_time_ms+1000*toc(stopwatch);
        end

        function update(obj,time,agent)
            % Flightからの唯一の周期呼出し口。
            %
            % 前提：通常のSimフローで同tickのEKFが更新済みであり、
            %       参照のdo()も終了してreference.resultが確定している。
            % 保存：Simのlogger.loggingがreference.resultを記録する。
            %       このメソッドで独自の保存やLOGGER.loggingは呼ばない。
            %
            % ENABLE_SENSORとLOG_SENSORは独立：ログを切っても検知は行う。
            % ここからリプランナーの責務だけを計測する。
            % 計測値は同じtickのreference.resultに保存し、標準LOGGERが回収する。
            % 一時診断：20回のみ工程境界でtic/tocを取得。
            % この計測自体に負荷があるため工程内訳は厳密な無侵襲測定ではない。
            diagnose=obj.diagnostic_count<20;
            if diagnose
                diagnostic_total_clock=tic;
                diagnostic_section_clock=tic;
                diagnostic_parts=nan(1,7);
            end
            measure_replanner=obj.ENABLE_ALL_LOG && obj.LOG_REPLANNER;
            if measure_replanner, update_clock=tic; end
            time_s=REPLANNING_SYSTEM.require_sim_time(time);
            if ~(isobject(agent)&&isscalar(agent)&&isprop(agent,'reference'))
                error('REPLANNING_SYSTEM:Agent', ...
                    'SimのDRONE agentを渡してください。');
            end

            % 参照結果へ追加するだけ。既存のxdや制御値は上書きしない。
            reference_result=agent.reference.result;
            if ~(isstruct(reference_result)&&isscalar(reference_result))
                error('REPLANNING_SYSTEM:ReferenceResult', ...
                    'agent.reference.resultがスカラー構造体ではありません。');
            end

            if diagnose
                diagnostic_parts(1)=1000*toc(diagnostic_section_clock);
                diagnostic_section_clock=tic;
            end
            sensor_result=[];
            if obj.ENABLE_SENSOR
                p_now=REPLANNING_SYSTEM.require_ekf_uav_position(agent);
                if diagnose
                    diagnostic_parts(2)=1000*toc(diagnostic_section_clock);
                    diagnostic_section_clock=tic;
                end
                sensor_result=obj.observe_sensor(time_s,p_now);
                obj.last_sensor_result=sensor_result;
            else
                obj.last_sensor_result=struct();
                if diagnose
                    diagnostic_parts(2)=1000*toc(diagnostic_section_clock);
                    diagnostic_section_clock=tic;
                end
            end
            if diagnose
                diagnostic_parts(3)=1000*toc(diagnostic_section_clock);
                diagnostic_section_clock=tic;
            end

            % ログONなら既存値を上書き。OFFに切り替えた場合だけ消す。
            % 毎tickのrmfield→再追加は不要なので実行しない。
            if obj.logging_enabled()
                reference_result.obstacle_information=obj.log_frame(time_s);
            elseif isfield(reference_result,'obstacle_information')
                reference_result=rmfield(reference_result,'obstacle_information');
            end
            if diagnose
                diagnostic_parts(4)=1000*toc(diagnostic_section_clock);
                diagnostic_section_clock=tic;
            end
            if obj.ENABLE_SENSOR && obj.ENABLE_ALL_LOG && obj.LOG_SENSOR
                reference_result.simulated_obstacle_sensor=sensor_result;
            elseif isfield(reference_result,'simulated_obstacle_sensor')
                reference_result=rmfield(reference_result,'simulated_obstacle_sensor');
            end
            if diagnose
                diagnostic_parts(5)=1000*toc(diagnostic_section_clock);
                diagnostic_section_clock=tic;
            end
            % センサ時間はこのupdate時間に含まれるため、二重加算しない。
            % 末尾の結果プロパティ代入は計測範囲外（小さな固定オーバーヘッド）。
            if measure_replanner
                update_ms=1000*toc(update_clock);
                reference_result.replanning_time_ms=update_ms;
                obj.replanner_timing_count=obj.replanner_timing_count+1;
                obj.replanner_timing_total_ms=obj.replanner_timing_total_ms+update_ms;
                obj.replanner_timing_max_ms=max(obj.replanner_timing_max_ms,update_ms);
            elseif isfield(reference_result,'replanning_time_ms')
                reference_result=rmfield(reference_result,'replanning_time_ms');
            end
            if diagnose
                diagnostic_parts(6)=1000*toc(diagnostic_section_clock);
                diagnostic_section_clock=tic;
            end
            agent.reference.result=reference_result;
            if diagnose
                diagnostic_parts(7)=1000*toc(diagnostic_section_clock);
                diagnostic_total_ms=1000*toc(diagnostic_total_clock);
                diagnostic_record=obj.diagnostic_count+1;
                obj.diagnostic_stage_ms(diagnostic_record,:)=diagnostic_parts;
                obj.diagnostic_time_s(diagnostic_record)=time_s;
                obj.diagnostic_total_ms(diagnostic_record)=diagnostic_total_ms;
                if measure_replanner
                    obj.diagnostic_logged_ms(diagnostic_record)=update_ms;
                end
                obj.diagnostic_count=diagnostic_record;
            end
        end

        function d=get_diagnostic_update_timing(obj)
            % 診断用出力。通常のログ形式・Flightの入力には影響しない。
            k=obj.diagnostic_count;
            d=struct('time_s',obj.diagnostic_time_s(1:k), ...
                'stage_names',{{'time_and_reference','ekf_position', ...
                   'sensor_and_last_result','obstacle_frame', ...
                   'sensor_field_write','time_accounting','reference_write'}}, ...
                'stage_ms',obj.diagnostic_stage_ms(1:k,:), ...
                'recorded_update_ms',obj.diagnostic_logged_ms(1:k), ...
                'inclusive_update_ms',obj.diagnostic_total_ms(1:k));
        end

        function frame=observe_sensor(obj,time_s,current_ekf_uav_position)
            % EKF機体位置を必須入力とする。未指定なら黙って公称値を使わない。
            if ~obj.ENABLE_SENSOR
                error('REPLANNING_SYSTEM:SensorDisabled', ...
                    'ENABLE_SENSOR=false です。検知処理は無効です。');
            end
            if nargin<3
                error('REPLANNING_SYSTEM:MissingEKF', ...
                    '現在時刻のEKF機体位置を第3引数へ渡してください。');
            end
            % 設定が変わった周期だけ検証・設定する。
            desired_log=obj.ENABLE_ALL_LOG && obj.LOG_SENSOR;
            if obj.sensor.LOG_SENSOR ~= desired_log
                obj.sensor.set_log(desired_log);
            end
            frame=obj.sensor.observe(time_s,current_ekf_uav_position);
        end

        function set_sensor_range(obj,range_m)
            % Flight途中も外部から値を更新可能。変更はセンサへ即座に反映。
            REPLANNING_SYSTEM.check_range(range_m);
            obj.sensor.set_range(range_m);
            obj.SENSOR_RANGE_M=double(range_m);
        end

        function status=get_status(obj)
            % 初期読込と元障害物ログの既存フィールドは変更しない。
            status=struct();
            status.source=which('OBSTACLE_INFORMATION');
            status.count=numel(obj.obstacles);
            status.ids=[obj.obstacles.id];
            shapes=[obj.obstacles.geometry];
            status.shapes={shapes.type};
            status.all_static=obj.all_static;
            status.definition_load_count=obj.load_count;
            status.definition_load_ms=obj.load_time_ms;
            status.ENABLE_ALL_LOG=obj.ENABLE_ALL_LOG;
            status.LOG_OBSTACLES=obj.LOG_OBSTACLES;
            status.frame_count=obj.frame_count;
            status.total_frame_time_ms=obj.total_frame_time_ms;
            status.last_logged_time_s=obj.last_logged_time_s;
            status.ENABLE_SENSOR=obj.ENABLE_SENSOR;
            status.LOG_SENSOR=obj.LOG_SENSOR;
            status.LOG_REPLANNER=obj.LOG_REPLANNER;
            status.replanner_timing_count=obj.replanner_timing_count;
            status.replanner_timing_total_ms=obj.replanner_timing_total_ms;
            status.replanner_timing_max_ms=obj.replanner_timing_max_ms;
            status.SENSOR_RANGE_M=obj.SENSOR_RANGE_M;
            status.preflight_warmup_ready=obj.preflight_warmup_ready;
            status.preflight_warmup_ms=obj.preflight_warmup_ms;
            status.preflight_warmup_distance_checks=obj.preflight_warmup_distance_checks;
            status.sensor_status=obj.sensor.get_status();
        end
    end

    methods (Access=private)
        function preflight_sensor_warmup(obj)
            % センサの初回実行コストをArming/Takeoff/Flightに持ち込まないための準備。
            % 既存障害物定義だけを使う。幾何判定式・検知結果・ログ形式は変更しない。
            %
            % 本番センサをobserveすると初回のnewly_detected_idsが消えるため、
            % 必ず独立した一時センサを使い、Flight用obj.sensorには触れない。
            % この処理はREPLANNING_SYSTEMのコンストラクタ内（Arming前）で一度だけ実行。
            preflight_clock=tic;
            probe=SIMULATED_OBSTACLE_SENSOR(obj.obstacles,obj.SENSOR_RANGE_M,true);
            shapes=OBSTACLE_GEOMETRY_QUERY.prepare(obj.obstacles);
            count=0;
            for k=1:numel(shapes)
                s=shapes(k);
                t=s.reference_time_s;
                c=OBSTACLE_GEOMETRY_QUERY.center_at(s,t);
                % 元形状に対する内部/外部の問い合わせを初回に通す。
                % 距離は判定結果に使わず、コード経路の事前実行だけが目的。
                OBSTACLE_GEOMETRY_QUERY.distance(c,s,t,c);
                count=count+1;
                % 外側の問い合わせも通す。判定ロジックは一切変更しない。
                outside=c+s.rotation*[s.bounding_radius_m+obj.SENSOR_RANGE_M/2;0;0];
                OBSTACLE_GEOMETRY_QUERY.distance(outside,s,t,c);
                count=count+1;
            end
            if isempty(shapes)
                t=0;
                p=[0;0;0];
                far=[obj.SENSOR_RANGE_M+1;0;0];
            else
                t=shapes(1).reference_time_s;
                p=OBSTACLE_GEOMETRY_QUERY.center_at(shapes(1),t);
                far=p+[shapes(1).bounding_radius_m+obj.SENSOR_RANGE_M+1;0;0];
            end
            % センサの観測、IDイベント、時間計測あり/なしの双方の経路を事前実行。
            % probeの状態は破棄され、実Flightのscan_countは0のまま。
            probe.observe(t,p);
            probe.observe(t,far);
            probe.set_log(false);
            probe.observe(t,p);
            obj.preflight_warmup_distance_checks=count;
            obj.preflight_warmup_ms=1000*toc(preflight_clock);
            obj.preflight_warmup_ready=true;
        end
    end

    methods (Static,Access=private)
        function time_s=require_sim_time(time)
            % SimのTIMEオブジェクト。テストでは同形のstructも可。
            if isobject(time)&&isscalar(time)&&isprop(time,'t')
                time_s=time.t;
            elseif isstruct(time)&&isscalar(time)&&isfield(time,'t')
                time_s=time.t;
            else
                error('REPLANNING_SYSTEM:TimeSource', ...
                    '現在のSim TIMEオブジェクト（time.t）を渡してください。');
            end
            REPLANNING_SYSTEM.check_time(time_s);
            time_s=double(time_s);
        end

        function p=require_ekf_uav_position(agent)
            % 公称位置・plant真値・前tickへのフォールバックは禁止。
            if ~(isobject(agent)&&isscalar(agent)&&isprop(agent,'estimator'))
                error('REPLANNING_SYSTEM:Estimator', ...
                    'agent.estimatorがありません。');
            end
            output=agent.estimator.result;
            if ~(isstruct(output)&&isscalar(output)&&isfield(output,'state'))
                error('REPLANNING_SYSTEM:EKFUnavailable', ...
                    '現在tickのEKF推定状態stateが取得できません。');
            end
            state=output.state;
            if isobject(state)&&isscalar(state)&&isprop(state,'p')
                p=state.p;
            elseif isstruct(state)&&isscalar(state)&&isfield(state,'p')
                p=state.p;
            else
                error('REPLANNING_SYSTEM:EKFUnavailable', ...
                    'EKF状態に機体位置pがありません。');
            end
            if ~(isnumeric(p)&&isreal(p)&&numel(p)==3&&all(isfinite(p(:))))
                error('REPLANNING_SYSTEM:EKFInvalid', ...
                    'EKFの機体位置pは有限の3次元実ベクトルでなければなりません。');
            end
            p=double(p(:));
        end

        function check_time(t)
            if ~isnumeric(t)||~isscalar(t)||~isreal(t)||~isfinite(t)
                error('REPLANNING_SYSTEM:Time','時刻は有限の実数を指定してください。');
            end
        end
        function check_range(r)
            if ~isnumeric(r)||~isscalar(r)||~isreal(r)||~isfinite(r)||r<=0
                error('REPLANNING_SYSTEM:Range','検知半径は正の有限な実数 [m] です。');
            end
        end
        function check_flag(v,name)
            valid=islogical(v)&&isscalar(v);
            valid=valid || (isnumeric(v)&&isscalar(v)&&isreal(v) ...
                && isfinite(v)&&(v==0||v==1));
            if ~valid
                error('REPLANNING_SYSTEM:InvalidFlag', ...
                    '%s は true/false または 0/1 としてください。',name);
            end
        end
    end
end
