function report = CHECK_OBSTACLE_FLIGHT_LOG(logger,replanner_instance)
% CHECK_OBSTACLE_FLIGHT_LOG  Sim LOGGER中の障害物定義ログを全時刻照合。
%
% Flight後、同じMATLABワークスペースで実行：
%   report = CHECK_OBSTACLE_FLIGHT_LOG(logger,replanner_instance);
%
% Sim LOGGERが保存したログを再読込した場合も使用可能：
%   saved_logger = LOGGER('Data/Sim_data/保存されたMATファイル');
%   report = CHECK_OBSTACLE_FLIGHT_LOG(saved_logger);
%
% 確認内容:
%   * REPLANNING_SYSTEMは障害物定義を1回だけ読んだか
%   * LOGGERの全時刻に欠測なく情報が入ったか
%   * 時刻・障害物数・ID・名前・形状・寸法・位置・姿勢・速度・移動方式
%   * 現在のOBSTACLE_INFORMATIONの定義と一致するか
%   * 定義にマージンや楕円近似を勝手に追加していないか
%
% 診断のTXT/MATだけを、この検証ファイルと同じ場所に保存する。
% センサ・トリガ・衝突の安全性は検証対象外。

    if nargin < 2
        replanner_instance = [];
    end
    if isempty(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
        error('CHECK_OBSTACLE_FLIGHT_LOG:Logger', ...
            '通常の Sim LOGGER オブジェクトを渡してください。');
    end

    expected = OBSTACLE_INFORMATION.definitions();
    OBSTACLE_INFORMATION.validate_list(expected);
    record_count = double(logger.k);
    error_messages = {};
    matched_records = 0;
    missing_records = 0;
    shape_mismatches = 0;
    time_mismatches = 0;
    identifier_mismatches = 0;
    position_mismatches = 0;
    source_changed = false;
    last_checked_time = NaN;

    if record_count < 1
        error_messages{end+1} = 'LOGGERにデータが1件もありません。'; %#ok<AGROW>
    end
    if ~isfield(logger.Data,'agent') || numel(logger.Data.agent)<1 || ...
            ~isfield(logger.Data.agent(1),'reference') || ...
            ~isfield(logger.Data.agent(1).reference,'result')
        error('CHECK_OBSTACLE_FLIGHT_LOG:MissingReferenceLog', ...
            'LOGGER.Data.agent(1).reference.result がありません。');
    end
    reference_logs = logger.Data.agent(1).reference.result;
    if numel(reference_logs) < record_count || numel(logger.Data.t)<record_count
        error('CHECK_OBSTACLE_FLIGHT_LOG:IncompleteLogger', ...
            'LOGGERのkと実際の記録配列の長さが一致しません。');
    end

    forbidden = {'d_margin','r_obs_margin','ellipsoid_radii', ...
                 'Q_obs','r_obs','trigger','visible','detected'};

    for k = 1:record_count
        stored_reference = reference_logs{k};
        if ~isstruct(stored_reference) || ...
                ~isfield(stored_reference,'obstacle_information')
            missing_records = missing_records+1;
            if numel(error_messages) < 12
                error_messages{end+1} = sprintf('k=%d: reference.result.obstacle_information がありません。',k); %#ok<AGROW>
            end
            continue
        end
        frame = stored_reference.obstacle_information;
        if ~isstruct(frame) || ~isscalar(frame) || ...
                ~isfield(frame,'time_s') || ~isfield(frame,'obstacles')
            missing_records = missing_records+1;
            if numel(error_messages) < 12
                error_messages{end+1} = sprintf('k=%d: 障害物ログ形式が不正です。',k); %#ok<AGROW>
            end
            continue
        end

        t = double(logger.Data.t(k));
        last_checked_time = t;
        if ~isscalar(frame.time_s) || ~isnumeric(frame.time_s) || ...
                ~isfinite(frame.time_s) || abs(double(frame.time_s)-t) > ...
                1e-9*max(1,abs(t))
            time_mismatches = time_mismatches+1;
            if numel(error_messages) < 12
                error_messages{end+1} = sprintf('k=%d: LOGGER時刻と障害物時刻が不一致。',k); %#ok<AGROW>
            end
        end

        actual = frame.obstacles;
        if ~isstruct(actual) || numel(actual) ~= numel(expected)
            shape_mismatches = shape_mismatches+1;
            if numel(error_messages) < 12
                error_messages{end+1} = sprintf('k=%d: 障害物の個数が不一致。',k); %#ok<AGROW>
            end
            continue
        end

        this_record_ok = true;
        for j = 1:numel(expected)
            real_obstacle = actual(j);
            definition = expected(j);
            required = {'id','name','geometry','initial_position','position', ...
                'rotation','velocity','reference_time_s','motion'};
            if ~all(isfield(real_obstacle,required)) || ...
                    any(isfield(real_obstacle,forbidden))
                shape_mismatches = shape_mismatches+1;
                this_record_ok = false;
                if numel(error_messages) < 12
                    error_messages{end+1} = sprintf('k=%d, obstacle=%d: 必須情報不足または余計な近似/マージン。',k,j); %#ok<AGROW>
                end
                continue
            end
            if ~isequal(real_obstacle.id,definition.id)
                identifier_mismatches = identifier_mismatches+1;
                this_record_ok = false;
            end
            % 形状以外の新しい生成値は許さず、元の情報をそのまま記録する。
            if ~isequaln(real_obstacle.name,definition.name) || ...
                    ~isequaln(real_obstacle.geometry,definition.geometry) || ...
                    ~isequaln(real_obstacle.initial_position,definition.initial_position) || ...
                    ~isequaln(real_obstacle.rotation,definition.rotation) || ...
                    ~isequaln(real_obstacle.velocity,definition.velocity) || ...
                    ~isequaln(real_obstacle.reference_time_s,definition.reference_time_s) || ...
                    ~isequaln(real_obstacle.motion,definition.motion)
                shape_mismatches = shape_mismatches+1;
                this_record_ok = false;
            end
            p_expected = definition.initial_position + ...
                (t-definition.reference_time_s)*definition.velocity;
            if ~isnumeric(real_obstacle.position) || ...
                    ~isequal(size(real_obstacle.position),[3 1]) || ...
                    any(~isfinite(real_obstacle.position(:))) || ...
                    norm(double(real_obstacle.position)-p_expected,inf) > ...
                    1e-9*max(1,norm(p_expected,inf))
                position_mismatches = position_mismatches+1;
                this_record_ok = false;
            end
        end
        if this_record_ok && ...
                isfinite(frame.time_s) && ...
                abs(double(frame.time_s)-t)<=1e-9*max(1,abs(t))
            matched_records = matched_records+1;
        end
    end

    % サンプル時刻でも、OBSTACLE_INFORMATION自身の時刻関数と照合する。
    sample_indices = unique([1,ceil(record_count/2),record_count]);
    for k = sample_indices
        if k < 1 || k > numel(reference_logs),continue;end
        item = reference_logs{k};
        if isstruct(item) && isfield(item,'obstacle_information') && ...
                isstruct(item.obstacle_information) && ...
                isfield(item.obstacle_information,'obstacles')
            source_at_time = OBSTACLE_INFORMATION.at_time(logger.Data.t(k));
            if ~isequaln(item.obstacle_information.obstacles,source_at_time)
                source_changed = true;
                if numel(error_messages)<12
                    error_messages{end+1} = sprintf('k=%d: OBSTACLE_INFORMATION.at_time と不一致。',k); %#ok<AGROW>
                end
            end
        end
    end

    module_status = struct();
    load_count_ok = true;
    if ~isempty(replanner_instance)
        if ~isa(replanner_instance,'REPLANNING_SYSTEM')
            error('CHECK_OBSTACLE_FLIGHT_LOG:Module', ...
                '第2引数はREPLANNING_SYSTEMオブジェクトです。');
        end
        module_status = replanner_instance.get_status();
        load_count_ok = module_status.definition_load_count==1 && ...
            module_status.frame_count==record_count && ...
            isequaln(replanner_instance.obstacles,expected);
        if ~load_count_ok
            error_messages{end+1} = '本体の初回読込数、ログ生成数、または定義スナップショットが不一致。'; %#ok<AGROW>
        end
    end

    report = struct();
    report.obstacle_count = numel(expected);
    report.obstacle_ids = [expected.id];
    g = [expected.geometry];
    report.shape_types = {g.type};
    report.logger_records = record_count;
    report.matched_records = matched_records;
    report.missing_records = missing_records;
    report.time_mismatches = time_mismatches;
    report.identifier_mismatches = identifier_mismatches;
    report.shape_mismatches = shape_mismatches;
    report.position_mismatches = position_mismatches;
    report.source_changed = source_changed;
    report.load_count_ok = load_count_ok;
    report.module_status = module_status;
    report.last_time_s = last_checked_time;
    report.errors = error_messages;
    report.passed = record_count>0 && matched_records==record_count && ...
        missing_records==0 && time_mismatches==0 && ...
        identifier_mismatches==0 && shape_mismatches==0 && ...
        position_mismatches==0 && ~source_changed && load_count_ok;
    if report.passed
        report.status = 'PASS_OBSTACLE_DEFINITIONS_AND_SIM_LOGGER';
    else
        report.status = 'FAIL_OBSTACLE_DEFINITION_OR_LOG_RECORD';
    end

    % 実行位置に依存せず、診断ファイルの横に結果だけ保存する。
    output_dir = fileparts(mfilename('fullpath'));
    mat_file = fullfile(output_dir,'OBSTACLE_FLIGHT_AUDIT.mat');
    txt_file = fullfile(output_dir,'OBSTACLE_FLIGHT_AUDIT.txt');
    save(mat_file,'report');
    fid = fopen(txt_file,'w');
    if fid < 0
        error('CHECK_OBSTACLE_FLIGHT_LOG:Write', ...
            '検証結果TXTを保存できません: %s',txt_file);
    end
    cleanup_file = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',report.status);
    fprintf(fid,'obstacles=%d | ids=%s\n',report.obstacle_count,mat2str(report.obstacle_ids));
    fprintf(fid,'types=%s\n',strjoin(report.shape_types,', '));
    fprintf(fid,'logger_records=%d | matched=%d | missing=%d\n', ...
        record_count,matched_records,missing_records);
    fprintf(fid,'time_mismatch=%d | id_mismatch=%d | shape_mismatch=%d | position_mismatch=%d\n', ...
        time_mismatches,identifier_mismatches,shape_mismatches,position_mismatches);
    fprintf(fid,'source_changed=%d | load_count_ok=%d | last_time_s=%.9g\n', ...
        source_changed,load_count_ok,last_checked_time);
    if ~isempty(replanner_instance)
        fprintf(fid,'definition_load_count=%d | generated_frame_count=%d | definition_load_ms=%.6f\n', ...
            module_status.definition_load_count,module_status.frame_count,module_status.definition_load_ms);
    end
    for j = 1:numel(error_messages)
        fprintf(fid,'ERROR %d: %s\n',j,error_messages{j});
    end
    fprintf('%s\n',report.status);
    fprintf('障害物 %d 件 (IDs %s) | LOGGER %d件 / 正常 %d件 / 欠測 %d件\n', ...
        report.obstacle_count,mat2str(report.obstacle_ids), ...
        record_count,matched_records,missing_records);
    fprintf('診断結果: %s\n',txt_file);
end
