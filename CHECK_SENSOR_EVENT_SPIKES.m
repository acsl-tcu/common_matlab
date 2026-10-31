function R = CHECK_SENSOR_EVENT_SPIKES(logger, replanner_instance)
% CHECK_SENSOR_EVENT_SPIKES
%   Flight後、Sim LOGGERに記録されたセンサ・リプランナー実行時間を
%   「新規検知/離脱イベントあり」「イベントなし」に分離する一時検証。
%
%   R = CHECK_SENSOR_EVENT_SPIKES(logger,replanner_instance)
%
%   検知前の障害物情報は使用しない。LOGGERに記録済みの観測結果のみを分析。
%   時間の閾値や数値を仮定してPASS判定しない。計算時間はMATLAB実測値。
%   本番ファイル/Sim LOGGERの状態は変更しない。

if nargin ~= 2
    error('CHECK_SENSOR_EVENT_SPIKES:Args', ...
        'logger と replanner_instance の2引数が必要です。');
end
if ~(isobject(logger) && isprop(logger,'Data') && isprop(logger,'k'))
    error('CHECK_SENSOR_EVENT_SPIKES:Logger','Sim LOGGER のloggerを渡してください。');
end
n=double(logger.k);
if ~(isscalar(n) && isfinite(n) && n>=2 && floor(n)==n)
    error('CHECK_SENSOR_EVENT_SPIKES:Logger','LOGGER.kの記録件数が不正です。');
end
if ~(isfield(logger.Data,'agent') && ~isempty(logger.Data.agent) && ...
        isfield(logger.Data.agent(1),'reference') && ...
        isfield(logger.Data.agent(1).reference,'result'))
    error('CHECK_SENSOR_EVENT_SPIKES:Logger','reference.resultが見つかりません。');
end
records=logger.Data.agent(1).reference.result;
if numel(records)<n
    error('CHECK_SENSOR_EVENT_SPIKES:Records','記録数が不足しています。');
end

time_s=nan(n,1);
sensor_ms=nan(n,1);
replanner_ms=nan(n,1);
num_detected=zeros(n,1);
new_ids=cell(n,1);
left_ids=cell(n,1);
is_event=false(n,1);
for i=1:n
    if iscell(records),rr=records{i};else,rr=records(i);end
    if ~isstruct(rr) || ~isfield(rr,'simulated_obstacle_sensor') || ...
            ~isfield(rr,'replanning_time_ms')
        error('CHECK_SENSOR_EVENT_SPIKES:Records', ...
            '記録%dにセンサ/リプランナー計測値がありません。',i);
    end
    s=rr.simulated_obstacle_sensor;
    time_s(i)=double(s.time_s);
    sensor_ms(i)=double(s.elapsed_ms);
    replanner_ms(i)=double(rr.replanning_time_ms);
    num_detected(i)=double(s.detected_count);
    new_ids{i}=reshape(double(s.newly_detected_ids),1,[]);
    left_ids{i}=reshape(double(s.left_range_ids),1,[]);
    is_event(i)=~isempty(new_ids{i}) || ~isempty(left_ids{i});
end
if any(~isfinite(sensor_ms)) || any(sensor_ms<0) || ...
        any(~isfinite(replanner_ms)) || any(replanner_ms<0) || ...
        any(~isfinite(time_s))
    error('CHECK_SENSOR_EVENT_SPIKES:Timing', ...
        '時間ログが無効です。LOG_SENSORとLOG_REPLANNERをONにしてください。');
end

% Armingのt=0記録と、Flight周期を分ける。既存runでは記録1が初期記録。
flight_idx=(2:n)';
flight_event_idx=flight_idx(is_event(flight_idx));
flight_plain_idx=flight_idx(~is_event(flight_idx));
if isempty(flight_plain_idx)
    error('CHECK_SENSOR_EVENT_SPIKES:Samples', ...
        'イベントなし周期が存在せず比較できません。');
end

% 現在の保存済み結果を検証するため、センサ状態も照合。
rs=replanner_instance.get_status();
sensor_status=rs.sensor_status;
if sensor_status.scan_count~=n || sensor_status.timing_count~=n
    error('CHECK_SENSOR_EVENT_SPIKES:Counts', ...
        '本番センサのscan_count/timing_countがLOGGER件数と不一致です。');
end
R=struct();
R.status='PASS_SENSOR_EVENT_LOG_INTEGRITY__PERFORMANCE_UNDECIDED';
R.records=n;
R.flight_records=numel(flight_idx);
R.event_records=numel(flight_event_idx);
R.event_indices=flight_event_idx;
R.event_times_s=time_s(flight_event_idx);
R.event_sensor_ms=sensor_ms(flight_event_idx);
R.event_replanner_ms=replanner_ms(flight_event_idx);
R.event_new_ids={new_ids{flight_event_idx}};
R.event_left_ids={left_ids{flight_event_idx}};
R.non_event_sensor_median_ms=median(sensor_ms(flight_plain_idx));
R.non_event_sensor_p95_ms=p95_exact(sensor_ms(flight_plain_idx));
R.non_event_sensor_max_ms=max(sensor_ms(flight_plain_idx));
R.non_event_replanner_median_ms=median(replanner_ms(flight_plain_idx));
R.non_event_replanner_max_ms=max(replanner_ms(flight_plain_idx));
R.event_sensor_median_ms=median_or_nan(sensor_ms(flight_event_idx));
R.event_sensor_max_ms=max_or_nan(sensor_ms(flight_event_idx));
R.event_replanner_max_ms=max_or_nan(replanner_ms(flight_event_idx));
R.arming_sensor_ms=sensor_ms(1);
R.arming_replanner_ms=replanner_ms(1);
R.flight_sensor_max_ms=max(sensor_ms(flight_idx));
R.flight_replanner_max_ms=max(replanner_ms(flight_idx));
R.event_to_steady_median_ratio=R.event_sensor_median_ms / ...
    R.non_event_sensor_median_ms;
R.top_sensor_idx=top_k(sensor_ms(flight_idx),flight_idx,12);
R.top_sensor_time_s=time_s(R.top_sensor_idx);
R.top_sensor_ms=sensor_ms(R.top_sensor_idx);
R.top_sensor_event_flag=is_event(R.top_sensor_idx);
R.sensor_p95_flight_ms=p95_exact(sensor_ms(flight_idx));
R.replanner_p95_flight_ms=p95_exact(replanner_ms(flight_idx));
R.note=['これは時系列の測定結果であり、最悪時間の保証ではない。', ...
    'イベントで長くなった場合、その因果原因を本ログだけから断定しない。'];
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'SENSOR_EVENT_SPIKE_AUDIT.mat'),'R');
fid=fopen(fullfile(root,'SENSOR_EVENT_SPIKE_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_EVENT_SPIKES:Output','TXTを作成できません。');end
cleaner=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',R.status);
fprintf(fid,'records=%d flight=%d events=%d\n',n,numel(flight_idx),numel(flight_event_idx));
fprintf(fid,'preflight/arming sensor=%.6f replanner=%.6f ms\n', ...
    R.arming_sensor_ms,R.arming_replanner_ms);
fprintf(fid,'Flight non-event sensor median/P95/max=%.6f %.6f %.6f ms\n', ...
    R.non_event_sensor_median_ms,R.non_event_sensor_p95_ms,R.non_event_sensor_max_ms);
fprintf(fid,'Flight event sensor median/max=%.6f %.6f ms | ratio=%.3f\n', ...
    R.event_sensor_median_ms,R.event_sensor_max_ms,R.event_to_steady_median_ratio);
fprintf(fid,'Flight replanner P95/max=%.6f %.6f ms\n', ...
    R.replanner_p95_flight_ms,R.flight_replanner_max_ms);
fprintf(fid,'EVENTS: record time_s new_ids left_ids sensor_ms replanner_ms\n');
for j=1:numel(flight_event_idx)
    k=flight_event_idx(j);
    fprintf(fid,'%d %.8f %s %s %.6f %.6f\n', ...
        k,time_s(k),mat2str(new_ids{k}),mat2str(left_ids{k}), ...
        sensor_ms(k),replanner_ms(k));
end
fprintf(fid,'TOP_SENSOR: record time_s event sensor_ms replanner_ms\n');
for k=R.top_sensor_idx(:)'
    fprintf(fid,'%d %.8f %d %.6f %.6f\n', ...
        k,time_s(k),is_event(k),sensor_ms(k),replanner_ms(k));
end
fprintf(fid,'NOTE %s\n',R.note);
fprintf('[SENSOR EVENT SPIKES] %s | Flight event median %.4f ms | steady median %.4f ms | max replanner %.4f ms\n', ...
    R.status,R.event_sensor_median_ms,R.non_event_sensor_median_ms,R.flight_replanner_max_ms);
end

function v=median_or_nan(x)
if isempty(x),v=NaN;else,v=median(x);end
end
function v=max_or_nan(x)
if isempty(x),v=NaN;else,v=max(x);end
end
function v=p95_exact(x)
x=sort(x(:));v=x(max(1,ceil(.95*numel(x))));
end
function idx=top_k(values,original_idx,k)
[~,order]=sort(values,'descend');
idx=original_idx(order(1:min(k,numel(order))));
end
