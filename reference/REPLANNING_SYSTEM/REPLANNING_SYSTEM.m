classdef REPLANNING_SYSTEM < handle
    % REPLANNING_SYSTEM  障害物定義を読み込み、Sim LOGGER へ渡すだけの統括クラス。
    %
    % 現段階で実行する仕事:
    %   1. OBSTACLE_INFORMATION.definitions() を Flight ごとに一度読む。
    %   2. 障害物の ID・元形状・位置・姿勢・速度をそのまま保持する。
    %   3. Sim LOGGER 用の値型構造体を、必要な時刻に返す。
    %
    % センサ、遮蔽、トリガ、衝突、経路生成、認証、マージン、近似は扱わない。
    % コントローラや目標軌道を包まず、変更もしない。
    %
    % 使い方:
    %   options = struct('ENABLE_ALL_LOG',true,'LOG_OBSTACLES',true);
    %   replanner = REPLANNING_SYSTEM(options);
    %   frame = replanner.log_frame(time.t);
    %   agent.reference.result.obstacle_information = frame;
    %   logger.logging(time,cha,agent,[]);  % 従来の Sim LOGGER で保存される

    properties (SetAccess = private)
        obstacles           % 障害物の原定義（Flight 中は変更しない）
        load_count = 0      % 原定義の読込回数
        frame_count = 0     % LOGGER へ渡すために作ったフレーム数
        load_time_ms = NaN
        total_frame_time_ms = 0
        last_logged_time_s = NaN
    end

    properties
        ENABLE_ALL_LOG = true
        LOG_OBSTACLES = true
    end

    properties (Access = private)
        static_frame        % 静止障害物なら形状情報を毎回作り直さない
        all_static = false
    end

    methods
        function obj = REPLANNING_SYSTEM(options)
            if nargin < 1
                options = struct();
            end
            if ~isstruct(options) || ~isscalar(options)
                error('REPLANNING_SYSTEM:Options', ...
                    '設定はスカラー構造体で指定してください。');
            end
            allowed = {'ENABLE_ALL_LOG','LOG_OBSTACLES'};
            names = fieldnames(options);
            for i = 1:numel(names)
                if ~ismember(names{i},allowed)
                    error('REPLANNING_SYSTEM:UnknownOption', ...
                        '現在の障害物読込専用クラスには設定 %s はありません。',names{i});
                end
                value = options.(names{i});
                if ~(islogical(value) && isscalar(value)) && ...
                        ~(isnumeric(value) && isscalar(value) && isreal(value) && ...
                          isfinite(value) && (value == 0 || value == 1))
                    error('REPLANNING_SYSTEM:InvalidFlag', ...
                        '%s は true/false (または 0/1) としてください。',names{i});
                end
                obj.(names{i}) = logical(value);
            end

            % 元定義の唯一の読み込み先。旧 ENVIRONMENT_* 関数は参照しない。
            stopwatch = tic;
            obj.obstacles = OBSTACLE_INFORMATION.definitions();
            OBSTACLE_INFORMATION.validate_list(obj.obstacles);
            obj.load_count = 1;
            obj.load_time_ms = 1000*toc(stopwatch);

            % 今の静止障害物だけの実験では、毎周期の再定義・形状検証を省く。
            obj.all_static = all(arrayfun(@(o) strcmp(o.motion,'static'),obj.obstacles));
            if obj.all_static
                obj.static_frame = struct('time_s',0,'obstacles',obj.obstacles);
            end
        end

        function yes = logging_enabled(obj)
            yes = obj.ENABLE_ALL_LOG && obj.LOG_OBSTACLES;
        end

        function frame = log_frame(obj,time_s)
            % 時刻ごとの障害物情報を値型の構造体として返す。
            % 保存担当は Sim の LOGGER。ここではディスクへ書かない。
            if ~isnumeric(time_s) || ~isscalar(time_s) || ...
                    ~isreal(time_s) || ~isfinite(time_s)
                error('REPLANNING_SYSTEM:Time', ...
                    '記録時刻は有限の実数スカラーで指定してください。');
            end
            if ~obj.logging_enabled()
                frame = [];
                return;
            end
            stopwatch = tic;
            if obj.all_static
                frame = obj.static_frame;
                frame.time_s = double(time_s);
            else
                % 将来の移動障害物では OBSTACLE_INFORMATION 側の時刻計算を使う。
                frame = OBSTACLE_INFORMATION.log_frame(time_s);
            end
            obj.frame_count = obj.frame_count+1;
            obj.last_logged_time_s = double(time_s);
            obj.total_frame_time_ms = obj.total_frame_time_ms + 1000*toc(stopwatch);
        end

        function status = get_status(obj)
            % Flight 後の検証用。数値結果のみ返し、独自形式で保存しない。
            status = struct();
            status.source = which('OBSTACLE_INFORMATION');
            status.count = numel(obj.obstacles);
            status.ids = [obj.obstacles.id];
            shapes = [obj.obstacles.geometry];
            status.shapes = {shapes.type};
            status.all_static = obj.all_static;
            status.definition_load_count = obj.load_count;
            status.definition_load_ms = obj.load_time_ms;
            status.ENABLE_ALL_LOG = obj.ENABLE_ALL_LOG;
            status.LOG_OBSTACLES = obj.LOG_OBSTACLES;
            status.frame_count = obj.frame_count;
            status.total_frame_time_ms = obj.total_frame_time_ms;
            status.last_logged_time_s = obj.last_logged_time_s;
        end
    end
end
