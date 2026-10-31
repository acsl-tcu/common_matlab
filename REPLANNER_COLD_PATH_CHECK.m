function report = REPLANNER_COLD_PATH_CHECK(logger,replanner_instance)
% REPLANNER_COLD_PATH_CHECK
% 実Flight後、事前update準備の状態保全と最初の周期の計算時間を監査する。
% 例: clear classes; run_flight_headless
%     report=REPLANNER_COLD_PATH_CHECK(logger,replanner_instance);
%     CHECK_SENSOR_FLIGHT_LOG(logger,replanner_instance);
%
% このファイルは一時検証用。REPLANNING_SYSTEM本体への診断コードは残さない。
% MATLABのJIT・OS割込みまで含むWCET保証を与える試験ではない。

if nargin~=2 || ~isobject(logger) || ~isprop(logger,'Data') || ...
        ~isprop(logger,'k') || ~isa(replanner_instance,'REPLANNING_SYSTEM')
    error('REPLANNER_COLD_PATH_CHECK:Args', ...
        'Flight後のLOGGERとREPLANNING_SYSTEMインスタンスを渡してください。');
end
S=replanner_instance.get_status();
D=logger.Data;
n=double(logger.k);
if ~(isscalar(n)&&n>=2&&isfield(D,'agent')&& ...
        isfield(D.agent(1),'reference')&& ...
        isfield(D.agent(1).reference,'result'))
    error('REPLANNER_COLD_PATH_CHECK:Logger','Flight LOGGERのデータが不完全です。');
end
if ~isfield(S,'preflight_update_ready')
    error('REPLANNER_COLD_PATH_CHECK:Source', ...
        '新しいREPLANNING_SYSTEM.mが実際に読み込まれていません。');
end
refs=D.agent(1).reference.result;
if iscell(refs)
    nr=numel(refs);
else
    nr=numel(refs);
end
errors={};
replan_ms=nan(n,1);
sensor_ms=nan(n,1);
sensor_time_s=nan(n,1);
detected_ids=cell(n,1);
new_ids=cell(n,1);
for k=1:n
    if k>nr
        errors{end+1}=sprintf('record %d: reference.result missing',k); %#ok<AGROW>
        continue
    end
    if iscell(refs),r=refs{k};else,r=refs(k);end
    if ~isstruct(r) || ~isscalar(r) || ...
            ~all(isfield(r,{'replanning_time_ms','simulated_obstacle_sensor', ...
                            'obstacle_information'}))
        errors{end+1}=sprintf('record %d: fields missing',k); %#ok<AGROW>
        continue
    end
    q=r.simulated_obstacle_sensor;
    replan_ms(k)=double(r.replanning_time_ms);
    sensor_ms(k)=double(q.elapsed_ms);
    if isfield(q,'time_s') && isnumeric(q.time_s) && isscalar(q.time_s)
        sensor_time_s(k)=double(q.time_s);
    end
    detected_ids{k}=double(q.detected_ids(:)).';
    new_ids{k}=double(q.newly_detected_ids(:)).';
end
if ~S.preflight_warmup_ready || ~S.preflight_update_ready
    errors{end+1}='Both geometry and full-update preflight must be ready';
end
if S.definition_load_count~=1
    errors{end+1}='Definition load was not exactly once';
end
if S.frame_count~=n || S.replanner_timing_count~=n || ...
        S.sensor_status.scan_count~=n || S.sensor_status.timing_count~=n
    errors{end+1}='Arming/Flight scan and frame counts differ from LOGGER records';
end
if any(~isfinite(replan_ms)) || any(replan_ms<0) || ...
        any(~isfinite(sensor_ms)) || any(sensor_ms<0)
    errors{end+1}='Timing record missing or nonfinite';
end
if isempty(detected_ids{1}) && ~isempty(new_ids{1})
    errors{end+1}='First scan contains ungrounded new ID';
end
if ~isequal(sort(unique(detected_ids{1})),sort(unique(new_ids{1})))
    errors{end+1}='First scan newly detected IDs do not match current detected IDs';
end
preflight_samples=double(S.preflight_update_samples_ms(:));
if numel(preflight_samples)~=8 || any(~isfinite(preflight_samples)) || ...
        any(preflight_samples<0) || ~(isfinite(S.preflight_update_ms)&&S.preflight_update_ms>=0)
    errors{end+1}='Preflight full-update samples are invalid';
end
first_count=min(6,n);
initial_values=replan_ms(1:first_count);
flight_values=replan_ms(2:end);
finite_flight=sort(flight_values(isfinite(flight_values)));
if isempty(finite_flight)
    flight_median=NaN;
    flight_p95=NaN;
else
    flight_median=median(finite_flight);
    flight_p95=finite_flight(max(1,ceil(.95*numel(finite_flight))));
end

report=struct();
report.status='PASS_PREFLIGHT_STATE_AND_FLIGHT_LOG';
if ~isempty(errors)
    report.status='FAIL_PREFLIGHT_STATE_OR_LOG';
end
report.preflight_complete=logical(S.preflight_update_ready);
report.preflight_full_path_ms=double(S.preflight_update_ms);
report.preflight_full_path_samples_ms=preflight_samples;
report.geometry_preflight_ms=double(S.preflight_warmup_ms);
report.records=n;
report.first_six_update_ms=initial_values;
report.first_six_max_ms=max(initial_values);
report.flight_update_median_ms=flight_median;
report.flight_update_p95_ms=flight_p95;
report.flight_update_max_ms=max(flight_values);
report.first_sensor_ms=sensor_ms(1);
report.first_six_sensor_ms=sensor_ms(1:first_count);
report.first_six_non_sensor_ms=replan_ms(1:first_count)-sensor_ms(1:first_count);
% Update内のセンサ時間は内数。上位周期で両者が重なるかを明示する。
valid_idx=find(isfinite(replan_ms) & isfinite(sensor_ms));
[~, ranked_order]=sort(replan_ms(valid_idx),'descend');
peak_idx=valid_idx(ranked_order(1:min(12,numel(ranked_order))));
report.top_update_record_indices=peak_idx;
report.top_update_time_s=sensor_time_s(peak_idx);
report.top_update_ms=replan_ms(peak_idx);
report.top_update_sensor_ms=sensor_ms(peak_idx);
report.top_update_other_ms=replan_ms(peak_idx)-sensor_ms(peak_idx);
valid_sensor_idx=find(isfinite(sensor_ms));
[~, sensor_order]=sort(sensor_ms(valid_sensor_idx),'descend');
report.top_sensor_record_indices=valid_sensor_idx(sensor_order(1:min(12,numel(sensor_order))));
report.sensor_peak_and_replanner_same_record = ...
    ~isempty(report.top_sensor_record_indices) && ~isempty(peak_idx) && ...
    report.top_sensor_record_indices(1)==peak_idx(1);
report.update_over_1ms_count=sum(replan_ms>1);
report.update_over_2ms_count=sum(replan_ms>2);
report.update_over_4ms_count=sum(replan_ms>4);
report.first_newly_detected_ids=new_ids{1};
report.errors=errors;
% 1msは性能の目安。厳密なリアルタイム保証と混同しない。
report.performance_target_ms=1;
if report.first_six_max_ms>report.performance_target_ms
    report.performance_status='EARLY_UPDATE_SPIKE_REMAINS';
else
    report.performance_status='EARLY_UPDATE_WITHIN_1MS_IN_THIS_FLIGHT';
end
report.note=['性能判定は観測した1 Flightの結果であり最悪実行時間の証明ではない。', ...
    'ログのreplanning_time_msは最終reference書戻し直前で計測を終了する。'];

root=fileparts(mfilename('fullpath'));
save(fullfile(root,'REPLANNER_COLD_PATH_AUDIT.mat'),'report');
file=fullfile(root,'REPLANNER_COLD_PATH_AUDIT.txt');
fid=fopen(file,'wt');
if fid<0,error('REPLANNER_COLD_PATH_CHECK:Write','TXT書込み失敗');end
clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',report.status);
fprintf(fid,'performance_status=%s\n',report.performance_status);
fprintf(fid,'preflight_full_update_ready=%d | elapsed=%.6f ms\n', ...
    report.preflight_complete,report.preflight_full_path_ms);
fprintf(fid,'preflight_update_samples_ms=');fprintf(fid,' %.6f',preflight_samples);fprintf(fid,'\n');
fprintf(fid,'logger_records=%d | first_new_ids=',n);
fprintf(fid,' %d',report.first_newly_detected_ids);fprintf(fid,'\n');
fprintf(fid,'first_six_update_ms=');fprintf(fid,' %.6f',initial_values);fprintf(fid,'\n');
fprintf(fid,'first_six_max_ms=%.6f | first_sensor_ms=%.6f\n', ...
    report.first_six_max_ms,report.first_sensor_ms);
fprintf(fid,'flight_update_median_p95_max_ms=%.6f %.6f %.6f\n', ...
    report.flight_update_median_ms,report.flight_update_p95_ms,report.flight_update_max_ms);
fprintf(fid,'update_over_1_2_4ms_counts=%d %d %d\n', ...
    report.update_over_1ms_count,report.update_over_2ms_count,report.update_over_4ms_count);
fprintf(fid,'sensor_peak_and_replanner_peak_same_record=%d\n', ...
    report.sensor_peak_and_replanner_same_record);
fprintf(fid,'TOP_UPDATE_SPIKES: record time_s update_ms sensor_ms other_ms\n');
for j=1:numel(peak_idx)
    fprintf(fid,'%d %.8f %.6f %.6f %.6f\n', ...
        peak_idx(j),report.top_update_time_s(j),report.top_update_ms(j), ...
        report.top_update_sensor_ms(j),report.top_update_other_ms(j));
end
fprintf(fid,'TOP_SENSOR_SPIKES: record time_s sensor_ms update_ms\n');
for j=1:numel(report.top_sensor_record_indices)
    k=report.top_sensor_record_indices(j);
    fprintf(fid,'%d %.8f %.6f %.6f\n',k,sensor_time_s(k),sensor_ms(k),replan_ms(k));
end
fprintf(fid,'errors=%d\n',numel(errors));
for j=1:numel(errors),fprintf(fid,'ERROR %d: %s\n',j,errors{j});end
fprintf(fid,'NOTE %s\n',report.note);
fprintf('[PREFLIGHT AUDIT] %s | %s\n',report.status,report.performance_status);
fprintf('[PREFLIGHT AUDIT] first six max=%.4f ms | Flight max=%.4f ms\n', ...
    report.first_six_max_ms,report.flight_update_max_ms);
if ~isempty(peak_idx)
    fprintf('[PREFLIGHT AUDIT] update peak: record=%d t=%.3f s | update=%.4f ms, sensor=%.4f ms, other=%.4f ms\n', ...
        peak_idx(1),report.top_update_time_s(1),report.top_update_ms(1), ...
        report.top_update_sensor_ms(1),report.top_update_other_ms(1));
end
fprintf('[PREFLIGHT AUDIT] sensor peak and update peak same record=%d\n', ...
    report.sensor_peak_and_replanner_same_record);
end
