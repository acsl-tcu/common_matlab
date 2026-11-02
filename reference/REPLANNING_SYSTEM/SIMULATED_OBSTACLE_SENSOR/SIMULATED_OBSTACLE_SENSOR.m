classdef SIMULATED_OBSTACLE_SENSOR < handle
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
    %   sensor = SIMULATED_OBSTACLE_SENSOR(obstacles,20,true);
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
        native_event_pack_available = false  % ID-only MEX ready before first observe()
        native_event_pack_build_status = 'NOT_CHECKED'
        native_event_pack_preload_status = 'NOT_ATTEMPTED'
    end

    properties
        LOG_SENSOR = false
    end

    properties (Access=private)
        native_event_pack_enabled = false  % 可否確認後のみ変更可能。ログの中身は不変。
        obstacles
        prepared_geometry
        previous_detected_mask = []  % 固定ID順で新規/離脱判定（setdiffを使わない）
        all_stationary = false
        all_obstacle_ids = []        % 元定義IDを初期化時に一度だけ確定
        static_log_obstacles = []    % 元形状のログを初期化時に確定（静止時のみ）
        result_template = struct() % 出力フィールド構造のみ初期化時に確定（検知結果は含めない）
        empty_visible_obstacles = [] % 同一structスキーマの空集合。検知前のデータは含めない
    end

    methods
        function obj=SIMULATED_OBSTACLE_SENSOR(obstacles,range_m,log_enabled)
            if nargin<3, log_enabled=false; end
            obj.validate_flag(log_enabled,'LOG_SENSOR');
            obj.LOG_SENSOR=logical(log_enabled);
            obj.set_range(range_m);
            startup=tic;
            obj.prepared_geometry=OBSTACLE_GEOMETRY_QUERY.prepare(obstacles);
            obj.geometry_prepare_ms=1000*toc(startup);
            % Build only during sensor initialization (before Flight), never in observe().
            % A matching prebuilt MEX in this source folder is reused as-is.
            [obj.native_event_pack_available,obj.native_event_pack_build_status]= ...
                SIMULATED_OBSTACLE_SENSOR.ensure_native_event_pack();
            obj.native_event_pack_enabled=obj.native_event_pack_available;
            % Load the ID-only MEX code BEFORE the first sensor cycle. The
            % empty arrays contain NO obstacle IDs, positions or detections.
            % This is NOT a sensor.scan/observe and cannot modify any sensor
            % history, detection mask, actual EKF state, or world knowledge.
            % The MEX source is unchanged; this never recompiles it.
            if obj.native_event_pack_available
                try
                    [cold_now,cold_new,cold_left]= ...
                        SENSOR_EVENT_IDS_NOHEAP_NATIVE( ...
                            zeros(1,0),false(1,0),false(1,0));
                    assert(isempty(cold_now)&&isempty(cold_new)&& ...
                        isempty(cold_left), ...
                        'SIMULATED_OBSTACLE_SENSOR:NativePreload', ...
                        'ID-only preload must have empty outputs.');
                    obj.native_event_pack_preload_status= ...
                        'CODE_ONLY_LOADED__ZERO_IDS_NO_SENSOR_OBSERVATION';
                catch ME
                    % The already supported MATLAB ID-event path retains
                    % the same exact detection semantics. Make it explicit.
                    obj.native_event_pack_enabled=false;
                    obj.native_event_pack_preload_status= ...
                        ['MEX_PRELOAD_FAILED__MATLAB_ID_PATH__' ME.identifier];
                end
            else
                obj.native_event_pack_preload_status= ...
                    'NATIVE_ID_MEX_UNAVAILABLE__MATLAB_ID_PATH';
            end
            obj.obstacles=obstacles;
            obj.obstacle_count=numel(obstacles);
            obj.previous_detected_mask=false(1,obj.obstacle_count);
            % 形状・姿勢・速度などの元定義はFlight中不変。
            % 静止物体の元構造体は最初に準備する。検知対象の選択は
            % 毎tick同一のコード経路に統一し、イベント周期だけに現れる
            % 別のキャッシュ更新/遅延実行分岐を作らない。
            obj.all_stationary=all([obj.prepared_geometry.is_stationary]);
            obj.all_obstacle_ids=[obj.obstacles.id];
            if obj.all_stationary
                % 静止時、各障害物のログ構造体（位置を含む）は時間非依存。
                % 動的なposition再計算は行わない。
                % 2^N通りの集合は事前生成しない（障害物数に依存しない）。
                obj.static_log_obstacles=obj.obstacles;
                for k=1:obj.obstacle_count
                    obj.static_log_obstacles(k).position= ...
                        obj.obstacles(k).initial_position;
                end
            end
            % 空集合も元定義のフィールドを厳密に維持する。
            % 空になった時にstruct配列へ論理インデックスする処理を省く。
            if obj.all_stationary
                obj.empty_visible_obstacles=obj.static_log_obstacles(false(1,obj.obstacle_count));
            else
                obj.empty_visible_obstacles=obj.obstacles(false(1,obj.obstacle_count));
            end
            % オンラインで検知していない元障害物は埋め込まない空の結果型。
            % field順序・形状は元センサと一致する。
            obj.result_template=struct( ...
                'time_s',NaN, ...
                'uav_position_ekf_m',zeros(3,1), ...
                'sensor_range_m',obj.range_m, ...
                'detected_count',0, ...
                'detected_ids',[], ...
                'newly_detected_ids',[], ...
                'left_range_ids',[], ...
                'detected_obstacles',obj.empty_visible_obstacles, ...
                'detected_distances_m',[], ...
                'candidates_checked',0, ...
                'obstacles_screened',obj.obstacle_count, ...
                'elapsed_ms',NaN, ...
                'time_measured',obj.LOG_SENSOR, ...
                'occlusion_enabled',false, ...
                'geometry_model','ORIGINAL_SOLID_DISTANCE_NO_APPROXIMATION');
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

        function empty_schema=empty_observed_obstacle_schema(obj)
            % Pure schema transfer. The array is EMPTY (numel==0); this
            % neither scans nor exposes any real or unseen obstacle data.
            % Allows memory to allocate a compatible result layout before
            % the very first genuinely observed obstacle is encountered.
            empty_schema=obj.empty_visible_obstacles;
            assert(isstruct(empty_schema)&&isempty(empty_schema), ...
                'SIMULATED_OBSTACLE_SENSOR:SchemaNonempty', ...
                'Only an empty observed-obstacle schema may be exported.');
        end

        function shapes=observed_prepared_only(obj,observed_ids)
            % Reveal ONLY the geometry belonging to the latest REAL scan.
            % The sensor already prepared its shape-distance oracle before
            % Flight for sensing. No true geometry of unseen obstacles is
            % exposed, no scan is issued, and no observation is fabricated.
            assert(isnumeric(observed_ids)&&isreal(observed_ids) && ...
                isvector(observed_ids)&&all(isfinite(observed_ids(:))) && ...
                all(observed_ids(:)==fix(observed_ids(:))) && ...
                numel(unique(observed_ids))==numel(observed_ids), ...
                'SIMULATED_OBSTACLE_SENSOR:ObservedIDs', ...
                'Only unique real observed obstacle IDs are accepted.');
            ids=reshape(double(observed_ids),1,[]);
            if isempty(ids)
                shapes=obj.prepared_geometry([]);
                return
            end
            [present,indices]=ismember(ids,obj.all_obstacle_ids);
            assert(all(present)&&all(obj.previous_detected_mask(indices)), ...
                'SIMULATED_OBSTACLE_SENSOR:UnseenGeometry', ...
                'Cannot expose prepared geometry for an unseen obstacle.');
            shapes=obj.prepared_geometry(indices);
        end

        function set_native_event_pack(obj,enabled)
            % 診断・A/B比較用。MEXが存在しないときは明示的に停止する。
            obj.validate_flag(enabled,'native_event_pack_enabled');
            if logical(enabled) && ~obj.native_event_pack_available
                error('SIMULATED_OBSTACLE_SENSOR:MexMissing', ...
                    'SENSOR_EVENT_IDS_NOHEAP_NATIVE MEXが利用できません。');
            end
            obj.native_event_pack_enabled=logical(enabled);
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

            % 毎周期の検知判定は上で完了済み。この周期の検知集合のみ返す。
            % 集合不変の周期は新規/離脱IDが必ず空：MEX呼出しで毎周期
            % 3本のmxArrayを確保する必要がないため、その経路を省く。
            % 集合変化時の既存MEXはそのまま使用し、ID順・型・空配列を維持。
            % 障害物構造体の選択と距離判定は変更しない。
            mask_changed=~isequal(detected,obj.previous_detected_mask);
            if ~mask_changed
                ids=obj.all_obstacle_ids(detected);
                if isempty(ids), ids=[]; end
                appeared=[];
                disappeared=[];
            elseif obj.native_event_pack_enabled
                [ids,appeared,disappeared]=SENSOR_EVENT_IDS_NOHEAP_NATIVE( ...
                    obj.all_obstacle_ids,detected,obj.previous_detected_mask);
            else
                ids=obj.all_obstacle_ids(detected);
                if isempty(ids), ids=[]; end
                entered_mask=detected & ~obj.previous_detected_mask;
                exited_mask=~detected & obj.previous_detected_mask;
                appeared=obj.all_obstacle_ids(entered_mask);
                disappeared=obj.all_obstacle_ids(exited_mask);
                if isempty(appeared), appeared=[]; end
                if isempty(disappeared), disappeared=[]; end
            end
            if ~any(detected)
                visible=obj.empty_visible_obstacles;
            elseif obj.all_stationary
                visible=obj.static_log_obstacles(detected);
            else
                visible=obj.obstacles(detected);
                for k=1:numel(visible)
                    o=visible(k);
                    visible(k).position=o.initial_position + ...
                        (double(time_s)-o.reference_time_s)*o.velocity;
                end
            end
            % 検証済み高速化: スカラーstructのフィールド構造を初期化時に確定。
            % 元データ・検知マスク・ログの意味を変えず、構築手順だけ変更する。
            result=obj.result_template;
            result.time_s=double(time_s);
            result.uav_position_ekf_m=p;
            result.sensor_range_m=obj.range_m;
            result.detected_count=numel(ids);
            result.detected_ids=ids;
            result.newly_detected_ids=appeared;
            result.left_range_ids=disappeared;
            result.detected_obstacles=visible;
            % 空配列の寸法も含め、元のMATLAB論理インデックスの結果を維持。
            % 特に障害物1個が未検知のときに固定1x0を強制しない。
            result.detected_distances_m=distances(detected);
            result.candidates_checked=candidate_count;
            result.time_measured=obj.LOG_SENSOR;
            % obstacles_screened/elapsed_ms/occlusion_enabled/geometry_model は
            % result_templateの不変フィールド。毎tick再設定しない。

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
                'LOG_SENSOR',obj.LOG_SENSOR, ...
                'native_event_pack_available',obj.native_event_pack_available, ...
                'native_event_pack_enabled',obj.native_event_pack_enabled, ...
                'native_event_pack_build_status',obj.native_event_pack_build_status, ...
                'native_event_pack_preload_status',obj.native_event_pack_preload_status);
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

    methods (Static)
        function [available,status]=ensure_native_event_pack()
            % Compile a missing ID-only MEX next to this sensor class, exactly once.
            % This is called by the constructor, NOT by observe()/Flight ticks.
            % If the native backend cannot be prepared, the sensor retains its
            % validated MATLAB path. No obstacle definition is disclosed here.
            persistent failed_binary
            sensor_dir=fileparts(mfilename('fullpath'));
            binary=fullfile(sensor_dir,['SENSOR_EVENT_IDS_NOHEAP_NATIVE.' mexext]);
            source=fullfile(sensor_dir,'SENSOR_EVENT_IDS_NOHEAP_NATIVE.cpp');
            available=false;
            status='MEX_MISSING';
            if ~isfile(binary)
                if ~isfile(source)
                    status='SOURCE_MISSING_MATLAB_FALLBACK';
                    warning('SIMULATED_OBSTACLE_SENSOR:NativeSourceMissing', ...
                        'MEXとC++ソースがありません。MATLAB処理を使用します: %s',source);
                    return
                end
                % Do not retry failed compilation in every temporary sensor
                % created during the same preflight. clear classes resets it.
                if isequal(failed_binary,binary)
                    status='PREVIOUS_BUILD_FAILED_MATLAB_FALLBACK';
                    return
                end
                try
                    mex('-R2018a','-outdir',sensor_dir,source);
                catch ME
                    failed_binary=binary;
                    status='BUILD_FAILED_MATLAB_FALLBACK';
                    warning('SIMULATED_OBSTACLE_SENSOR:NativeBuildFailed', ...
                        'MEXの自動ビルドに失敗。検知はMATLAB経路で継続します: %s',ME.message);
                    return
                end
                status='BUILT_AUTOMATICALLY';
            else
                status='REUSED_EXISTING';
            end
            rehash
            % Never call an identically named native module from another folder.
            resolved=which('SENSOR_EVENT_IDS_NOHEAP_NATIVE');
            if ~strcmpi(resolved,binary)
                status='SHADOWED_MEX_MATLAB_FALLBACK';
                warning('SIMULATED_OBSTACLE_SENSOR:NativeShadowed', ...
                    '同名MEXのパスが一致しません。MATLAB経路を使用します。期待: %s / 実際: %s', ...
                    binary,resolved);
                return
            end
            if ~isfile(binary) || exist('SENSOR_EVENT_IDS_NOHEAP_NATIVE','file')~=3
                status='MEX_NOT_LOADABLE_MATLAB_FALLBACK';
                warning('SIMULATED_OBSTACLE_SENSOR:NativeNotLoadable', ...
                    'MEXを認識できません。MATLAB経路を使用します: %s',binary);
                return
            end
            available=true;
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
