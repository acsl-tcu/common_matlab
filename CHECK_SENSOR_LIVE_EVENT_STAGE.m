function R = CHECK_SENSOR_LIVE_EVENT_STAGE(logger,replanner_instance)
% CHECK_SENSOR_LIVE_EVENT_STAGE
% 本番出力を変えない一時診断。前回Flightでも最大時刻/検知イベントを抽出できる。
% 診断版SIMULATED_OBSTACLE_SENSOR.mでFlightした場合のみ内部5区間を記録する。
%   R=CHECK_SENSOR_LIVE_EVENT_STAGE(logger,replanner_instance);
% 検知判定、障害物記憶、後段への入力、Sim LOGGERは変更しない。

if nargin~=2 || ~isobject(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Args', ...
        'Flight後の logger と replanner_instance を指定してください。');
end
n=double(logger.k);
if ~isscalar(n)||~isfinite(n)||n<2||fix(n)~=n
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Count','LOGGERの記録数が不正です。');
end
D=logger.Data;
if ~isfield(D,'agent')||isempty(D.agent)||...
        ~isfield(D.agent(1),'reference')||...
        ~isfield(D.agent(1).reference,'result')
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Logger','Sim LOGGERの参照結果がありません。');
end
records=D.agent(1).reference.result;
if numel(records)<n
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Logger','LOGGER記録数が不足しています。');
end
sensor_ms=nan(n,1); replanner_ms=nan(n,1); t=nan(n,1);
events=false(n,1); news=cell(n,1); lefts=cell(n,1);
candidates=nan(n,1); count=nan(n,1);
for i=1:n
    if iscell(records), rr=records{i}; else, rr=records(i); end
    if ~isstruct(rr)||~isfield(rr,'simulated_obstacle_sensor')||...
            ~isfield(rr,'replanning_time_ms')
        error('CHECK_SENSOR_LIVE_EVENT_STAGE:LogField', ...
            '記録%dのセンサ/リプランナー結果がありません。',i);
    end
    s=rr.simulated_obstacle_sensor;
    t(i)=double(s.time_s);
    sensor_ms(i)=double(s.elapsed_ms);
    replanner_ms(i)=double(rr.replanning_time_ms);
    news{i}=reshape(double(s.newly_detected_ids),1,[]);
    lefts{i}=reshape(double(s.left_range_ids),1,[]);
    events(i)=~isempty(news{i}) || ~isempty(lefts{i});
    candidates(i)=double(s.candidates_checked);
    count(i)=double(s.detected_count);
end
if any(~isfinite(sensor_ms))||any(sensor_ms<0)||...
        any(~isfinite(replanner_ms))||any(replanner_ms<0)||any(~isfinite(t))
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Timing', ...
        'タイミングフィールドが不正です。LOG_SENSOR/LOG_REPLANNERを確認してください。');
end
flight=(2:n)';
[flight_sensor_peak_ms,ks]=max(sensor_ms(flight));
peak_idx=flight(ks);
[flight_replanner_peak_ms,kr]=max(replanner_ms(flight));
replanner_peak_idx=flight(kr);
event_idx=flight(events(flight));
plain_idx=flight(~events(flight));
[~,order]=sort(sensor_ms(flight),'descend');
top_idx=flight(order(1:min(15,numel(order))));

R=struct();
R.status='FLIGHT_ONLY__STAGE_TRACE_NOT_AVAILABLE';
R.records=n;
R.sensor_ms=sensor_ms;
R.replanner_ms=replanner_ms;
R.times_s=t;
R.event_flags=events;
R.candidates_checked=candidates;
R.detected_count=count;
R.event_indices=event_idx;
R.event_times_s=t(event_idx);
R.event_sensor_ms=sensor_ms(event_idx);
R.event_replanner_ms=replanner_ms(event_idx);
R.flight_sensor_peak_ms=flight_sensor_peak_ms;
R.flight_sensor_peak_idx=peak_idx;
R.flight_sensor_peak_time_s=t(peak_idx);
R.flight_sensor_peak_is_event=events(peak_idx);
R.flight_replanner_peak_ms=flight_replanner_peak_ms;
R.flight_replanner_peak_idx=replanner_peak_idx;
R.flight_replanner_peak_time_s=t(replanner_peak_idx);
R.flight_peaks_same_record=peak_idx==replanner_peak_idx;
R.flight_non_event_median_ms=median_or_nan(sensor_ms(plain_idx));
R.flight_event_median_ms=median_or_nan(sensor_ms(event_idx));
R.top_indices=top_idx;
R.trace_columns={};
R.trace=zeros(0,13);
R.trace_count=0;
R.trace_consistent_with_logger=false;
R.peak_trace_row=zeros(0,13);
R.trace_stage_names={'geometry','selection_and_ids','result_pack','state','timing_overhead'};

if ~isobject(replanner_instance)||~ismethod(replanner_instance,'get_status')
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Replanner', ...
        '実際のreplanner_instanceを渡してください。');
end
rs=replanner_instance.get_status();
if ~isfield(rs,'sensor_status')
    error('CHECK_SENSOR_LIVE_EVENT_STAGE:Status','センサ状態を取得できません。');
end
ss=rs.sensor_status;
if isfield(ss,'diagnostic_trace') && ...
        isfield(ss,'diagnostic_capture_enabled') && ...
        logical(ss.diagnostic_capture_enabled)
    tr=double(ss.diagnostic_trace);
    if isempty(tr),tr=zeros(0,13);end
    if size(tr,2)~=13
        error('CHECK_SENSOR_LIVE_EVENT_STAGE:TraceShape','診断トレースの列数が不正です。');
    end
    idx=tr(:,1);
    if any(~isfinite(tr(:))) || any(idx<1) || any(idx>n) || ...
            any(idx~=fix(idx)) || numel(unique(idx))~=numel(idx)
        error('CHECK_SENSOR_LIVE_EVENT_STAGE:TraceIndices','診断トレースが不正です。');
    end
    if ~isempty(idx)
        ix=double(idx(:));
        match_time=all(abs(tr(:,2)-t(ix))<1e-8);
        match_event=all(logical(tr(:,3))==events(ix));
        match_ms=all(abs(tr(:,13)-sensor_ms(ix))<1e-8);
        R.trace_consistent_with_logger=match_time && match_event && match_ms;
        if ~R.trace_consistent_with_logger
            error('CHECK_SENSOR_LIVE_EVENT_STAGE:TraceMismatch', ...
                '時刻・イベント・センサ時間の診断トレースとLOGGERが不一致です。');
        end
    else
        R.trace_consistent_with_logger=true;
    end
    R.trace=tr;
    R.trace_count=size(tr,1);
    R.trace_columns=ss.diagnostic_trace_columns;
    k=find(tr(:,1)==peak_idx,1);
    if ~isempty(k),R.peak_trace_row=tr(k,:);end
    if isempty(R.peak_trace_row)
        R.status='TRACE_PRESENT__FLIGHT_PEAK_NOT_CAPTURED';
    else
        R.status='PASS_LIVE_STAGE_TRACE_AND_LOG_MATCH';
    end
end

outdir=fileparts(mfilename('fullpath'));
save(fullfile(outdir,'SENSOR_LIVE_EVENT_STAGE_AUDIT.mat'),'R');
fid=fopen(fullfile(outdir,'SENSOR_LIVE_EVENT_STAGE_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_LIVE_EVENT_STAGE:Output','TXTを作成できません。');end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',R.status);
fprintf(fid,'LOGGER records=%d | event_count_flight=%d | trace_records=%d\n',...
    n,numel(event_idx),R.trace_count);
fprintf(fid,'Flight SENSOR PEAK idx=%d t=%.8f ms=%.6f event=%d\n',...
    peak_idx,t(peak_idx),sensor_ms(peak_idx),events(peak_idx));
fprintf(fid,'Flight REPLANNER PEAK idx=%d t=%.8f ms=%.6f same_record=%d\n',...
    replanner_peak_idx,t(replanner_peak_idx),replanner_ms(replanner_peak_idx),...
    R.flight_peaks_same_record);
fprintf(fid,'Flight event/non-event sensor median ms=%.6f / %.6f\n',...
    R.flight_event_median_ms,R.flight_non_event_median_ms);
fprintf(fid,'EVENTS idx time_s new left candidates sensor_ms replanner_ms\n');
for ix=event_idx(:)'
    fprintf(fid,'%d %.8f %s %s %.0f %.6f %.6f\n',ix,t(ix),...
        mat2str(news{ix}),mat2str(lefts{ix}),candidates(ix),sensor_ms(ix),replanner_ms(ix));
end
fprintf(fid,'TOP_SENSOR idx time_s event candidates sensor_ms replanner_ms');
if R.trace_count>0
    fprintf(fid,' geometry selection_ids result_pack state timing_overhead [ms]');
end
fprintf(fid,'\n');
for ix=top_idx(:)'
    fprintf(fid,'%d %.8f %d %.0f %.6f %.6f',ix,t(ix),events(ix),...
        candidates(ix),sensor_ms(ix),replanner_ms(ix));
    if R.trace_count>0
        j=find(R.trace(:,1)==ix,1);
        if isempty(j),fprintf(fid,' NOT_CAPTURED');
        else,fprintf(fid,' %.6f %.6f %.6f %.6f %.6f',R.trace(j,8:12));end
    end
    fprintf(fid,'\n');
end
fprintf(fid,'NOTE: 段階別の追加tocは観測負荷を含む。時間は最悪時間の保証ではない。\n');
fprintf(fid,'NOTE: センサelapsed_msはlast_result代入前、replanning_time_msはagent.reference.result書戻し前に計測終了。\n');
fprintf(fid,'NOTE: 診断は検知判定を省略・事前確定しない。未検知障害物は公開しない。\n');
fprintf('[LIVE SENSOR STAGES] %s | peak sensor=%.4f ms at %.3f s | peak replanner=%.4f ms at %.3f s\n',...
    R.status,R.flight_sensor_peak_ms,R.flight_sensor_peak_time_s,...
    R.flight_replanner_peak_ms,R.flight_replanner_peak_time_s);
end

function m=median_or_nan(x)
if isempty(x),m=NaN;else,m=median(x);end
end
