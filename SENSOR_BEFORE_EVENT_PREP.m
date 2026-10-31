classdef SENSOR_BEFORE_EVENT_PREP < handle
    % SIMULATED_OBSTACLE_SENSOR  機体中心の球形領域に入った障害物を発見する。
    %
    % 【このクラスの責務】
    %   - 現在の EKF 機体位置と、設定された検知半径 [m] を受け取る。
    %   - 障害物の「元の占有形状」と検知球の交差を調べる。
    %   - 検知した ID、数、追加/離脱 ID と、元障害物情報を返す。
    %   - LOG_SENSOR=true 時は毎回の処理時間を結果に記録する。
    %
    % 【このクラスではしないこと】
    %   - 遮蔽、ノイズ、視野角、将来衝突予測、障害物の安全マージン付加
    %   - 検知済み障害物を永久保存すること（後段の障害物記憶の担当）
    %   - 目標軌道・HLC・plant の変更、独立した MAT/TXT 保存
    %
    % 【幾何判定】
    %   機体中心p、障害物占有領域Oについて min_{x in O}||p-x|| <= range。
    %   全形状共通の問い合わせは OBSTACLE_GEOMETRY_QUERY に委譲する。
    %   遠方の障害物だけは安全な外接球下界で早期除外する。
    %
    % 例:
    %   sensor = SENSOR_BEFORE_EVENT_PREP(obstacles,20,true);
    %   frame = sensor.observe(time_s,ekf_uav_position);
    %   % frame は Sim LOGGER へ渡せる通常の MATLAB 構造体。

    properties (SetAccess=private)
        range_m
        obstacle_count
        last_result = struct()
        scan_count = 0
        timing_count = 0
        total_scan_time_ms = 0
        max_scan_time_ms = 0
        last_scan_time_ms = NaN
        geometry_prepare_ms = NaN
    end

    properties
        LOG_SENSOR = false
    end

    properties (Access=private)
        obstacles
        prepared_geometry
        previous_detected_mask = []  % 固定ID順で新規/離脱判定（setdiffを使わない）
        all_stationary = false
        cached_visible = []          % 検知集合が同じ静止障害物だけ、元構造体を再利用
        cached_ids = []              % IDsも同じ集合なら再生成しない
    end

    methods
        function obj=SENSOR_BEFORE_EVENT_PREP(obstacles,range_m,log_enabled)
            if nargin<3, log_enabled=false; end
            obj.validate_flag(log_enabled,'LOG_SENSOR');
            obj.LOG_SENSOR=logical(log_enabled);
            obj.set_range(range_m);
            startup=tic;
            obj.prepared_geometry=OBSTACLE_GEOMETRY_QUERY.prepare(obstacles);
            obj.geometry_prepare_ms=1000*toc(startup);
            obj.obstacles=obstacles;
            obj.obstacle_count=numel(obstacles);
            obj.previous_detected_mask=false(1,obj.obstacle_count);
            % 形状・姿勢・速度などの元定義はFlight中不変。
            % 静止物体の場合は位置を毎周期計算せず、検知集合が変わる時だけ
            % LOGGERへ渡す構造体配列を再構成する（検知そのものは毎tick再計算）。
            obj.all_stationary=all([obj.prepared_geometry.is_stationary]);
            obj.cached_visible=obj.obstacles(false(1,obj.obstacle_count));
            obj.cached_ids=[];
        end

        function set_range(obj,range_m)
            % 外部設定は必ず統括クラス経由で指定する。
            if ~(isnumeric(range_m)&&isscalar(range_m)&&isreal(range_m) ...
                    && isfinite(range_m)&&range_m>0)
                error('SIMULATED_OBSTACLE_SENSOR:Range', ...
                    '検知距離は正の有限なスカラー [m] で指定してください。');
            end
            obj.range_m=double(range_m);
        end

        function set_log(obj,enabled)
            obj.validate_flag(enabled,'LOG_SENSOR');
            obj.LOG_SENSOR=logical(enabled);
        end

        function result=observe(obj,time_s,p_uav_ekf)
            % Flight各時刻に呼ぶ。EKF位置を入力せずには実行できない。
            % 出力は「今回検知した障害物のみ」を後段に渡す。
            % 毎周期の全障害物を後段へ渡す処理は行わない。
            if ~(isnumeric(time_s)&&isscalar(time_s)&&isreal(time_s)&&isfinite(time_s))
                error('SIMULATED_OBSTACLE_SENSOR:Time','観測時刻が不正です。');
            end
            if ~(isnumeric(p_uav_ekf)&&isreal(p_uav_ekf)&&numel(p_uav_ekf)==3 ...
                    && all(isfinite(p_uav_ekf(:))))
                error('SIMULATED_OBSTACLE_SENSOR:EKF', ...
                    '現在時刻の EKF 機体位置 [x;y;z] を明示的に渡してください。');
            end
            p=double(p_uav_ekf(:));
            if obj.LOG_SENSOR,clock=tic;end

            n=obj.obstacle_count;
            detected=false(1,n);
            distances=nan(1,n);
            candidate_count=0;
            % すべての障害物で共通の値を一度だけ計算する。
            % 数値境界ガードの式と距離判定の意味は変更しない。
            uav_norm=norm(p);
            for k=1:n
                shape=obj.prepared_geometry(k);
                if shape.is_stationary
                    center=shape.center0;
                    center_norm=shape.center_norm_m;
                else
                    center=OBSTACLE_GEOMETRY_QUERY.center_at(shape,time_s);
                    center_norm=norm(center);
                end
                if any(~isfinite(center))
                    error('SIMULATED_OBSTACLE_SENSOR:Motion', ...
                        '障害物ID %d の位置が有限ではありません。',shape.id);
                end
                % 外接球は早期「除外」だけ。これで検知確定とはしない。
                lower_bound=norm(p-center)-shape.bounding_radius_m;
                numerical_guard=64*eps(max([1, uav_norm, center_norm, obj.range_m]));
                if lower_bound>obj.range_m+numerical_guard
                    continue
                end
                candidate_count=candidate_count+1;
                % 同じ障害物中心を幾何クエリに渡す（再計算しない）。
                d=OBSTACLE_GEOMETRY_QUERY.distance(p,shape,time_s,center);
                distances(k)=d;
                detected(k)= d <= obj.range_m+numerical_guard;
            end

            % 検知・距離は毎周期、新規/離脱IDも毎周期確定させる。
            % 静止障害物で前周期と検知集合が同じときだけ、元形状を含む
            % LOGGER用構造体を再利用する。障害物数によらず事前列挙はしない。
            same_mask=isequal(detected,obj.previous_detected_mask);
            unchanged=obj.all_stationary && same_mask;
            if unchanged
                ids=obj.cached_ids;
                visible=obj.cached_visible;
            else
                ids=[obj.obstacles(detected).id];
                visible=obj.obstacles(detected);
                % 動的障害物は従来どおり各tickの真の位置を出力する。
                % 静止障害物でも初回は同じ式で位置を確定してからキャッシュ。
                for k=1:numel(visible)
                    o=visible(k);
                    visible(k).position=o.initial_position + ...
                        (double(time_s)-o.reference_time_s)*o.velocity;
                end
                if obj.all_stationary
                    obj.cached_ids=ids;
                    obj.cached_visible=visible;
                end
            end
            % 同じ検知集合ならイベントは空。不要な構造体インデックスをしない。
            % 集合が変わったときは従来の論理マスク式・ID順をそのまま使う。
            if same_mask
                appeared=[];
                disappeared=[];
            else
                appeared=[obj.obstacles(detected & ~obj.previous_detected_mask).id];
                disappeared=[obj.obstacles(~detected & obj.previous_detected_mask).id];
            end
            result=struct( ...
                'time_s',double(time_s), ...
                'uav_position_ekf_m',p, ...
                'sensor_range_m',obj.range_m, ...
                'detected_count',numel(ids), ...
                'detected_ids',ids, ...
                'newly_detected_ids',appeared, ...
                'left_range_ids',disappeared, ...
                'detected_obstacles',visible, ...
                'detected_distances_m',distances(detected), ...
                'candidates_checked',candidate_count, ...
                'obstacles_screened',n, ...
                'elapsed_ms',NaN, ...
                'time_measured',obj.LOG_SENSOR, ...
                'occlusion_enabled',false, ...
                'geometry_model','ORIGINAL_SOLID_DISTANCE_NO_APPROXIMATION');

            obj.previous_detected_mask=detected;
            obj.scan_count=obj.scan_count+1;
            if obj.LOG_SENSOR
                measured_ms=1000*toc(clock);
                result.elapsed_ms=measured_ms;
                obj.timing_count=obj.timing_count+1;
                obj.total_scan_time_ms=obj.total_scan_time_ms+measured_ms;
                obj.max_scan_time_ms=max(obj.max_scan_time_ms,measured_ms);
                obj.last_scan_time_ms=measured_ms;
            end
            obj.last_result=result;
        end

        function s=get_status(obj)
            s=struct('range_m',obj.range_m, ...
                'obstacles_available',obj.obstacle_count, ...
                'scan_count',obj.scan_count, ...
                'timing_count',obj.timing_count, ...
                'time_total_ms',obj.total_scan_time_ms, ...
                'time_max_ms',obj.max_scan_time_ms, ...
                'time_last_ms',obj.last_scan_time_ms, ...
                'geometry_prepare_ms',obj.geometry_prepare_ms, ...
                'LOG_SENSOR',obj.LOG_SENSOR);
            if obj.timing_count==0
                s.time_mean_ms=NaN;
            else
                s.time_mean_ms=obj.total_scan_time_ms/obj.timing_count;
            end
            if isempty(fieldnames(obj.last_result))
                s.last_detected_ids=[];
            else
                s.last_detected_ids=obj.last_result.detected_ids;
            end
        end
    end

    methods (Static,Access=private)
        function validate_flag(value,name)
            valid=islogical(value)&&isscalar(value);
            valid=valid || (isnumeric(value)&&isscalar(value)&&isreal(value) ...
                && isfinite(value)&&(value==0||value==1));
            if ~valid
                error('SIMULATED_OBSTACLE_SENSOR:Flag', ...
                    '%s は true/false または 0/1 を指定してください。',name);
            end
        end
    end
end
