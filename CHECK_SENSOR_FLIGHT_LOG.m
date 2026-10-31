function report = CHECK_SENSOR_FLIGHT_LOG(logger, replanner_instance)
% CHECK_SENSOR_FLIGHT_LOG  センサ実Flightログの一時検証用（本番ソースではない）。
%
% 手順：run_flight_headless を実行後、MATLABのワークスペースで
%   report = CHECK_SENSOR_FLIGHT_LOG(logger, replanner_instance);
%
% 確認すること：
%   - 全周期に、時刻が一致した模擬センサの結果が保存されているか
%   - EKFの現時刻位置がセンサ入力に使われているか
%   - 検知障害物数とID/実体情報が一致するか
%   - 新規/離脱IDの差分が正しいか
%   - 距離・検知半径・候補数・実測処理時間に欠落がないか
%   - 障害物定義はFlightごとに1回しか読まれていないか
%   - 検知ログがSim LOGGER内に残っているか
%
% 障害物定義/センサ/REPLANNING_SYSTEM/HLC/EKF/plantは変更しない。
% 結果は同じディレクトリの SENSOR_FLIGHT_AUDIT.mat/.txt に保存。
% 検証後、このスクリプトは削除してよい。

if nargin < 2
    error('CHECK_SENSOR_FLIGHT_LOG:Arguments', ...
        'logger と replanner_instance の両方を渡してください。');
end
if ~isobject(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_SENSOR_FLIGHT_LOG:Logger', 'SimのLOGGERオブジェクトを渡してください。');
end
if ~isobject(replanner_instance) || ~ismethod(replanner_instance,'get_status')
    error('CHECK_SENSOR_FLIGHT_LOG:Replanner', ...
        'Flightで使った REPLANNING_SYSTEM オブジェクトを渡してください。');
end

D = logger.Data;
n = double(logger.k);
if ~isstruct(D) || ~isfield(D,'t') || ~isfield(D,'agent') || n<1
    error('CHECK_SENSOR_FLIGHT_LOG:Data','LOGGERに有効なFlightデータがありません。');
end
if numel(D.t)<n
    error('CHECK_SENSOR_FLIGHT_LOG:Data','LOGGERの時刻配列が不完全です。');
end
if ~isstruct(D.agent(1)) || ~isfield(D.agent(1),'reference') || ...
        ~isfield(D.agent(1).reference,'result') || ...
        ~isfield(D.agent(1),'estimator') || ...
        ~isfield(D.agent(1).estimator,'result')
    error('CHECK_SENSOR_FLIGHT_LOG:Data', ...
        'reference.result / estimator.result がありません。');
end
R = as_cells(D.agent(1).reference.result);
E = as_cells(D.agent(1).estimator.result);
time_s = double(D.t(1:n));
time_s = time_s(:);
count = nan(n,1);
elapsed_ms = nan(n,1);
range_m = nan(n,1);
detected_ids = cell(n,1);
new_ids = cell(n,1);
left_ids = cell(n,1);
candidate_count = nan(n,1);
replanner_ms=nan(n,1);
errors = {};
% 旧Flightにはシステム全体/リプランナー単独時間のログがない。
% 旧Flightのセンサ検証は継続可能にし、新形式のFlightでは時間も必須にする。
initial_status=replanner_instance.get_status();
expect_new_timing=isfield(initial_status,'LOG_REPLANNER') || ...
    isfield(D,'system_cycle_time_ms');
missing = 0;
valid = 0;
prev_ids = [];
original_ids = [replanner_instance.obstacles.id];

for k = 1:n
    if k > numel(R) || ~isstruct(R{k}) || ...
            ~isfield(R{k},'simulated_obstacle_sensor')
        missing = missing+1;
        add_error(sprintf('記録%d: simulated_obstacle_sensor が欠けています。',k));
        continue
    end
    frame = R{k}.simulated_obstacle_sensor;
    if ~isstruct(frame) || ~isscalar(frame)
        add_error(sprintf('記録%d: センサ結果がスカラー構造体ではありません。',k));
        continue
    end
    required = {'time_s','uav_position_ekf_m','sensor_range_m', ...
        'detected_count','detected_ids','newly_detected_ids','left_range_ids', ...
        'detected_obstacles','detected_distances_m','candidates_checked', ...
        'obstacles_screened','elapsed_ms','time_measured','occlusion_enabled'};
    if ~all(isfield(frame,required))
        add_error(sprintf('記録%d: センサ結果に必須フィールドがありません。',k));
        continue
    end
    valid = valid+1;
    if isfield(R{k},'replanning_time_ms') && ...
            isnumeric(R{k}.replanning_time_ms) && ...
            isscalar(R{k}.replanning_time_ms)
        replanner_ms(k)=double(R{k}.replanning_time_ms);
    elseif expect_new_timing
        add_error(sprintf('記録%d: リプランナー単独の時間がありません。',k));
    end
    tolerance = 1e-9+64*eps(max([1,abs(time_s(k)),abs(double(frame.time_s))]));
    if ~isscalar(frame.time_s) || abs(double(frame.time_s)-time_s(k)) > tolerance
        add_error(sprintf('記録%d: 時刻不一致。',k));
    end
    ids = double(frame.detected_ids(:)).';
    detected_ids{k} = ids;
    new_ids{k} = double(frame.newly_detected_ids(:)).';
    left_ids{k} = double(frame.left_range_ids(:)).';
    count(k) = double(frame.detected_count);
    elapsed_ms(k) = double(frame.elapsed_ms);
    range_m(k) = double(frame.sensor_range_m);
    candidate_count(k) = double(frame.candidates_checked);
    if ~(isscalar(count(k)) && isfinite(count(k)) && ...
            count(k)==numel(ids) && numel(unique(ids))==numel(ids))
        add_error(sprintf('記録%d: 検知数/IDの不一致。',k));
    end
    if any(~ismember(ids,original_ids))
        add_error(sprintf('記録%d: 未定義IDが検知されました。',k));
    end
    % 空集合 [] / 1x0 / 0x1 の形状差はID集合の違いではない。
    expected_new=setdiff(ids,prev_ids,'stable');
    expected_left=setdiff(prev_ids,ids,'stable');
    if ~isempty(setxor(new_ids{k},expected_new)) || ...
            ~isempty(setxor(left_ids{k},expected_left)) || ...
            numel(unique(new_ids{k}))~=numel(new_ids{k}) || ...
            numel(unique(left_ids{k}))~=numel(left_ids{k})
        add_error(sprintf('記録%d: 新規/離脱IDの不一致。',k));
    end
    prev_ids=ids;
    if numel(frame.detected_obstacles)~=numel(ids)
        add_error(sprintf('記録%d: 検知した障害物の元情報が不足。',k));
    elseif ~isempty(ids) && ~isequal([frame.detected_obstacles.id],ids)
        add_error(sprintf('記録%d: 検知障害物のIDと定義情報が不一致。',k));
    end
    ds=double(frame.detected_distances_m(:));
    if numel(ds)~=numel(ids) || any(~isfinite(ds)) || any(ds<0) || ...
            any(ds>range_m(k)+1e-9)
        add_error(sprintf('記録%d: 検知距離と半径の不一致。',k));
    end
    if frame.obstacles_screened~=numel(original_ids) || ...
            candidate_count(k)<numel(ids)
        add_error(sprintf('記録%d: 候補数・総障害物数が不一致。',k));
    end
    if frame.occlusion_enabled
        add_error(sprintf('記録%d: 遮蔽計算が有効になっています。',k));
    end
    if ~frame.time_measured || ~(isscalar(elapsed_ms(k)) && ...
            isfinite(elapsed_ms(k)) && elapsed_ms(k)>=0)
        add_error(sprintf('記録%d: 実行時間が記録されていません。',k));
    end

    % 独立して保存されたEKF位置とセンサの入力を照合する。
    if k>numel(E) || ~isstruct(E{k}) || ~isfield(E{k},'state')
        add_error(sprintf('記録%d: EKF状態がLOGGERにありません。',k));
    else
        st = E{k}.state;
        if isobject(st) && isprop(st,'p')
            p=st.p;
        elseif isstruct(st) && isscalar(st) && isfield(st,'p')
            p=st.p;
        else
            p=[];
        end
        ps=double(frame.uav_position_ekf_m(:));
        if ~(isnumeric(p) && isreal(p) && numel(p)==3 && ...
                all(isfinite(p(:))) && numel(ps)==3 && all(isfinite(ps)) && ...
                norm(double(p(:))-ps,inf)<=1e-9)
            add_error(sprintf('記録%d: EKF位置とセンサ入力が一致しません。',k));
        end
    end
end

% アーミング(記録1)は通常周期ではないので計測対象外。
% 全周期の実測値はrunが通常LOGGER.Dataへ保存する。
whole_ms=nan(n,1);
if ~isfield(D,'system_cycle_time_ms') || numel(D.system_cycle_time_ms)<n
    if expect_new_timing
        add_error('標準LOGGERにsystem_cycle_time_msが記録されていません。');
    end
else
    whole_ms=double(D.system_cycle_time_ms(1:n));
    whole_ms=whole_ms(:);
    invalid=find(~isfinite(whole_ms(2:end)) | whole_ms(2:end)<0,1,'first');
    if ~isempty(invalid)
        add_error(sprintf('記録%d: システム全体の時間が無効です。',invalid+1));
    end
end
if expect_new_timing
    if any(~isfinite(replanner_ms)) || any(replanner_ms<0)
        add_error('リプランナー単独の実測時間に欠落があります。');
    end
    if any(whole_ms(2:end)+1e-9<replanner_ms(2:end))
        add_error('システム全体の時間がリプランナー単独の時間より短い周期があります。');
    end
end

status = replanner_instance.get_status();
if status.definition_load_count~=1
    add_error('障害物定義がFlight中に再読み込みされています。');
end
if status.sensor_status.scan_count~=n
    add_error(sprintf('センサ実行回数=%d、LOGGER件数=%d。', ...
        status.sensor_status.scan_count,n));
end
if status.sensor_status.timing_count~=n
    add_error(sprintf('センサ時間記録回数=%d、LOGGER件数=%d。', ...
        status.sensor_status.timing_count,n));
end
if missing>0
    add_error(sprintf('センサ結果の欠測レコード=%d。',missing));
end
if any(~isfinite(range_m))
    add_error('検知距離ログに欠落があります。');
end

measured = sort(elapsed_ms(isfinite(elapsed_ms)));
report=struct();
report.status='PASS_SENSOR_FLIGHT_AND_SIM_LOGGER';
if ~isempty(errors),report.status='FAIL_SENSOR_FLIGHT_AND_SIM_LOGGER';end
report.logger_records=n;
report.valid_sensor_records=valid;
report.missing_records=missing;
report.original_obstacle_ids=original_ids;
report.time_s=time_s;
report.detected_count=count;
report.detected_ids=detected_ids;
report.newly_detected_ids=new_ids;
report.left_range_ids=left_ids;
report.elapsed_ms=elapsed_ms;
if expect_new_timing
    report.timing_status='RECORDED_AND_CHECKED';
else
    report.timing_status='NOT_RECORDED_IN_PREVIOUS_FLIGHT';
end
report.replanner_time_ms=replanner_ms;
report.system_cycle_time_ms=whole_ms;
report.sensor_range_m=range_m;
report.geometry_candidates_checked=candidate_count;
report.sensor_status=status.sensor_status;
report.definition_load_count=status.definition_load_count;
report.errors=errors;
report.total_detected_samples=sum(count>0);
report.all_seen_ids=[];
for k=1:n
    report.all_seen_ids=union(report.all_seen_ids,detected_ids{k});
end
report.time_median_ms=NaN;
report.time_p95_ms=NaN;
report.time_max_ms=NaN;
report.replanner_time_median_ms=NaN;
report.replanner_time_p95_ms=NaN;
report.replanner_time_max_ms=NaN;
report.system_cycle_median_ms=NaN;
report.system_cycle_p95_ms=NaN;
report.system_cycle_max_ms=NaN;
if ~isempty(measured)
    report.time_median_ms=median(measured);
    report.time_p95_ms=measured(max(1,ceil(.95*numel(measured))));
    report.time_max_ms=measured(end);
end

good=sort(replanner_ms(isfinite(replanner_ms)&replanner_ms>=0));
if ~isempty(good)
    report.replanner_time_median_ms=median(good);
    report.replanner_time_p95_ms=good(max(1,ceil(.95*numel(good))));
    report.replanner_time_max_ms=good(end);
end
good=sort(whole_ms(isfinite(whole_ms)&whole_ms>=0));
if ~isempty(good)
    report.system_cycle_median_ms=median(good);
    report.system_cycle_p95_ms=good(max(1,ceil(.95*numel(good))));
    report.system_cycle_max_ms=good(end);
end
folder=fileparts(mfilename('fullpath'));
mat_path=fullfile(folder,'SENSOR_FLIGHT_AUDIT.mat');
txt_path=fullfile(folder,'SENSOR_FLIGHT_AUDIT.txt');
save(mat_path,'report');
fid=fopen(txt_path,'wt');
if fid<0,error('CHECK_SENSOR_FLIGHT_LOG:Save','TXTを保存できません。');end
cleaner=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',report.status);
fprintf(fid,'logger_records=%d | sensor_records=%d | missing=%d\n',n,valid,missing);
fprintf(fid,'obstacle_ids=%s | observed_ids=%s\n', ...
    mat2str(original_ids),mat2str(report.all_seen_ids));
fprintf(fid,'sensor_scans=%d | timing_count=%d | definition_load_count=%d\n', ...
    status.sensor_status.scan_count,status.sensor_status.timing_count, ...
    status.definition_load_count);
fprintf(fid,'with_detection=%d | time_median_ms=%.6f | time_p95_ms=%.6f | time_max_ms=%.6f\n', ...
    report.total_detected_samples,report.time_median_ms, ...
    report.time_p95_ms,report.time_max_ms);
fprintf(fid,'timing_status=%s\n',report.timing_status);
fprintf(fid,'replanner_median_ms=%.6f | replanner_p95_ms=%.6f | replanner_max_ms=%.6f\n', ...
    report.replanner_time_median_ms,report.replanner_time_p95_ms,report.replanner_time_max_ms);
fprintf(fid,'cycle_median_ms=%.6f | cycle_p95_ms=%.6f | cycle_max_ms=%.6f\n', ...
    report.system_cycle_median_ms,report.system_cycle_p95_ms,report.system_cycle_max_ms);
fprintf(fid,'failures=%d\n',numel(errors));
for k=1:numel(errors)
    fprintf(fid,'ERROR %d: %s\n',k,errors{k});
end
fprintf('[SENSOR AUDIT] %s | records=%d/%d | missing=%d | errors=%d\n', ...
    report.status,valid,n,missing,numel(errors));
fprintf('[SENSOR AUDIT] observed IDs=%s | median=%.4f ms | P95=%.4f ms | max=%.4f ms\n', ...
    mat2str(report.all_seen_ids),report.time_median_ms, ...
    report.time_p95_ms,report.time_max_ms);
fprintf('[TIMING AUDIT] replanner median/P95/max = %.4f / %.4f / %.4f ms\n', ...
    report.replanner_time_median_ms,report.replanner_time_p95_ms,report.replanner_time_max_ms);
fprintf('[TIMING AUDIT] whole cycle median/P95/max = %.3f / %.3f / %.3f ms\n', ...
    report.system_cycle_median_ms,report.system_cycle_p95_ms,report.system_cycle_max_ms);
fprintf('SENSOR_FLIGHT_AUDIT.mat / .txt saved next to this script.\n');
if ~isempty(errors)
    fprintf(2,'[SENSOR AUDIT] CHECK FAILED: %s\n',errors{1});
end

    function add_error(message)
        errors{end+1}=message; %#ok<AGROW>
    end
end

function out=as_cells(value)
if iscell(value)
    out=value(:);
else
    out=num2cell(value(:));
end
end
