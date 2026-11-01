function A = CHECK_SENSOR_EVENT_CAUSAL(logger)
% CHECK_SENSOR_EVENT_CAUSAL  Flight後だけに行うイベント遅延の区間別診断。
%  A=CHECK_SENSOR_EVENT_CAUSAL(logger)
%  REPLANNING_SYSTEM・HLC・EKF・plant・Sim LOGGERを一切変更しない。
%  LOGGERの記録済み現在EKF位置と検知半径を使い、別の一時センサに再生する。
%  検知前の障害物情報を回避/予測へ渡すことはない。
%
%  仮説を独立に分離する:
%   (a) 幾何距離判定 (b) 構造体/ID選択 (c) 新規/離脱ID
%   (d) 結果の構築   (e) 計測/状態更新
%  さらに境界の前後に同じ位置を交互に入力し、イベントを繰り返す。
%  ※区間別tic/tocを追加するためprobe自体の絶対時間は本番値と異なる。
%  ※この診断だけでOS中断やMATLAB JITの因果を証明することはできない。

if nargin~=1 || ~isobject(logger) || ~isprop(logger,'Data') || ~isprop(logger,'k')
    error('CHECK_SENSOR_EVENT_CAUSAL:Logger','Flight後のSim LOGGERが必要です。');
end
n=double(logger.k);
if ~isfinite(n)||n<3||n~=floor(n)
    error('CHECK_SENSOR_EVENT_CAUSAL:Count','LOGGERの記録件数が不正です。');
end
if ~isfield(logger.Data,'agent') || isempty(logger.Data.agent) || ...
        ~isfield(logger.Data.agent(1),'reference') || ...
        ~isfield(logger.Data.agent(1).reference,'result')
    error('CHECK_SENSOR_EVENT_CAUSAL:Data','LOGGERのreference.resultが見つかりません。');
end
records=logger.Data.agent(1).reference.result;
if numel(records)<n
    error('CHECK_SENSOR_EVENT_CAUSAL:Count','LOGGERの実記録数が不足しています。');
end
expected=fullfile(fileparts(mfilename('fullpath')),'reference', ...
    'REPLANNING_SYSTEM','SIMULATED_OBSTACLE_SENSOR','SIMULATED_OBSTACLE_SENSOR.m');
current=which('SIMULATED_OBSTACLE_SENSOR');
if ~strcmpi(current,expected)
    error('CHECK_SENSOR_EVENT_CAUSAL:Source', ...
        '実センサのパスが予想と異なります。対象=%s',current);
end
assert(exist('SENSOR_EVENT_STAGE_PROBE','class')==8, ...
    'SENSOR_EVENT_STAGE_PROBE.mを同じプロジェクト直下に配置してください。');
assert(exist('SENSOR_EVENT_TEMPLATE_CANDIDATE','class')==8, ...
    'SENSOR_EVENT_TEMPLATE_CANDIDATE.mを同じプロジェクト直下に配置してください。');

O=OBSTACLE_INFORMATION.definitions(); % Simの真値定義を診断リプレイにのみ使用
first_record=get_record(records,1);
first=first_record.simulated_obstacle_sensor;
base=SIMULATED_OBSTACLE_SENSOR(O,first.sensor_range_m,first.time_measured);
probe=SENSOR_EVENT_STAGE_PROBE(O,first.sensor_range_m,first.time_measured);
alt=SENSOR_EVENT_TEMPLATE_CANDIDATE(O,first.sensor_range_m,first.time_measured);
stage_names={'geometry','visible_selection','event_id_masks','result_pack','timing_and_state'};
stage_ms=nan(n,5);
replay_sensor_ms=nan(n,1);
alternative_sensor_ms=nan(n,1);
logged_sensor_ms=nan(n,1);
logged_replanner_ms=nan(n,1);
logged_candidates=nan(n,1);
times=nan(n,1);
flags=false(n,1);
failures={};
for i=1:n
    this_record=get_record(records,i);
    truth=this_record.simulated_obstacle_sensor;
    t=double(truth.time_s);
    p=double(truth.uav_position_ekf_m(:));
    if base.range_m~=truth.sensor_range_m
        base.set_range(truth.sensor_range_m);
        probe.set_range(truth.sensor_range_m);
        alt.set_range(truth.sensor_range_m);
    end
    if base.LOG_SENSOR~=logical(truth.time_measured)
        base.set_log(truth.time_measured);
        probe.set_log(truth.time_measured);
        alt.set_log(truth.time_measured);
    end
    % 元センサと候補センサの実行順を交互にしてCPUキャッシュ順序偏りを減らす。
    if mod(i,2)==0
        z=alt.observe(t,p);
        x=base.observe(t,p);
    else
        x=base.observe(t,p);
        z=alt.observe(t,p);
    end
    y=probe.observe(t,p);
    stage_ms(i,:)=probe.last_stage_ms;
    alternative_sensor_ms(i)=z.elapsed_ms;
    replay_sensor_ms(i)=x.elapsed_ms;
    logged_sensor_ms(i)=truth.elapsed_ms;
    logged_replanner_ms(i)=this_record.replanning_time_ms;
    logged_candidates(i)=truth.candidates_checked;
    times(i)=t;
    flags(i)=~isempty(truth.newly_detected_ids)||~isempty(truth.left_range_ids);
    x.elapsed_ms=[]; y.elapsed_ms=[]; z.elapsed_ms=[]; truth.elapsed_ms=[];
    if ~isequaln(x,y)
        failures{end+1}=sprintf('PROBE_OUTPUT record=%d',i); %#ok<AGROW>
    end
    if ~isequaln(x,z)
        failures{end+1}=sprintf('TEMPLATE_OUTPUT record=%d',i); %#ok<AGROW>
    end
    if ~isequaln(x,truth)
        failures{end+1}=sprintf('LOGGER_REPLAY record=%d',i); %#ok<AGROW>
    end
end
flight_idx=(2:n)';
event_idx=flight_idx(flags(flight_idx));
plain_idx=flight_idx(~flags(flight_idx));
if isempty(event_idx)||isempty(plain_idx)
    error('CHECK_SENSOR_EVENT_CAUSAL:Cases','Flight中の検知切替/非切替サンプルが必要です。');
end

% 4つの実イベントを各回「直前状態 -> イベント状態 -> 同じ状態を維持」で反復する。
% 本番センサ・LOGGER・現在EKFには触れない。時間は飛行中の測定ではない。
repeat_count=80;
skip_count=20;
event_repeated_ms=nan(numel(event_idx),repeat_count-skip_count);
steady_repeated_ms=nan(numel(event_idx),repeat_count-skip_count);
alt_event_repeated_ms=nan(numel(event_idx),repeat_count-skip_count);
alt_steady_repeated_ms=nan(numel(event_idx),repeat_count-skip_count);
repeat_event_stage=nan(numel(event_idx),repeat_count-skip_count,5);
repeat_steady_stage=nan(numel(event_idx),repeat_count-skip_count,5);
for j=1:numel(event_idx)
    k=event_idx(j);
    before_record=get_record(records,k-1);
    after_record=get_record(records,k);
    before=before_record.simulated_obstacle_sensor;
    after=after_record.simulated_obstacle_sensor;
    a=SIMULATED_OBSTACLE_SENSOR(O,before.sensor_range_m,true);
    b=SENSOR_EVENT_STAGE_PROBE(O,before.sensor_range_m,true);
    c=SENSOR_EVENT_TEMPLATE_CANDIDATE(O,before.sensor_range_m,true);
    for rep=1:repeat_count
        a.set_range(before.sensor_range_m);
        b.set_range(before.sensor_range_m);
        c.set_range(before.sensor_range_m);
        a.observe(before.time_s,before.uav_position_ekf_m);
        b.observe(before.time_s,before.uav_position_ekf_m);
        c.observe(before.time_s,before.uav_position_ekf_m);
        a.set_range(after.sensor_range_m);
        b.set_range(after.sensor_range_m);
        c.set_range(after.sensor_range_m);
        if mod(rep,2)==0
            cev=c.observe(after.time_s,after.uav_position_ekf_m);
            ev=a.observe(after.time_s,after.uav_position_ekf_m);
        else
            ev=a.observe(after.time_s,after.uav_position_ekf_m);
            cev=c.observe(after.time_s,after.uav_position_ekf_m);
        end
        bev=b.observe(after.time_s,after.uav_position_ekf_m);
        es=b.last_stage_ms;
        if ~isequaln(ev.detected_ids,after.detected_ids) || ...
                ~isequaln(ev.newly_detected_ids,after.newly_detected_ids) || ...
                ~isequaln(ev.left_range_ids,after.left_range_ids)
            failures{end+1}=sprintf('EVENT_RESET record=%d rep=%d',k,rep); %#ok<AGROW>
        end
        if rep>skip_count
            pos=rep-skip_count;
            event_repeated_ms(j,pos)=ev.elapsed_ms;
            alt_event_repeated_ms(j,pos)=cev.elapsed_ms;
            repeat_event_stage(j,pos,:)=reshape(es,1,1,[]);
        end
        if mod(rep,2)==0
            cst=c.observe(after.time_s,after.uav_position_ekf_m);
            st=a.observe(after.time_s,after.uav_position_ekf_m);
        else
            st=a.observe(after.time_s,after.uav_position_ekf_m);
            cst=c.observe(after.time_s,after.uav_position_ekf_m);
        end
        b.observe(after.time_s,after.uav_position_ekf_m);
        if rep>skip_count
            steady_repeated_ms(j,pos)=st.elapsed_ms;
            alt_steady_repeated_ms(j,pos)=cst.elapsed_ms;
            repeat_steady_stage(j,pos,:)=reshape(b.last_stage_ms,1,1,[]);
        end
        if any(~isfinite(bev.uav_position_ekf_m))
            error('CHECK_SENSOR_EVENT_CAUSAL:Probe','検証センサの出力が不正です。');
        end
    end
end

A=struct();
A.status='PASS_LOG_EQUIVALENCE__CAUSE_UNDECIDED';
if ~isempty(failures)
    A.status='FAIL_CAUSAL_REPLAY_EQUIVALENCE';
end
A.record_count=n;
A.event_count=numel(event_idx);
A.failures=failures;
A.event_idx=event_idx;
A.event_time_s=times(event_idx);
A.event_candidates_checked=logged_candidates(event_idx);
A.event_sensor_logged_ms=logged_sensor_ms(event_idx);
A.event_replanner_logged_ms=logged_replanner_ms(event_idx);
A.event_stage_probe_ms=stage_ms(event_idx,:);
A.neighbor_stage_probe_median_ms=nan(numel(event_idx),5);
A.neighbor_candidates_checked=cell(numel(event_idx),1);
for j=1:numel(event_idx)
    ix=event_idx(j);
    % Index vectors must share orientation.  A row neighborhood combined
    % with column flags expands to a 2-D logical mask in recent MATLAB.
    neighborhood=(max(2,ix-20):min(n,ix+20)).';
    neighbor_mask=(neighborhood~=ix) & ~flags(neighborhood) & ...
        (logged_candidates(neighborhood)==logged_candidates(ix));
    if ~isequal(size(neighbor_mask),size(neighborhood))
        error('CHECK_SENSOR_EVENT_CAUSAL:NeighborMaskShape', ...
            '近傍インデックスと論理マスクのサイズが一致しません。');
    end
    neighborhood=neighborhood(neighbor_mask);
    A.neighbor_candidates_checked{j}=logged_candidates(neighborhood);
    if ~isempty(neighborhood)
        A.neighbor_stage_probe_median_ms(j,:)=median(stage_ms(neighborhood,:),1);
    end
end
A.flight_event_median_ms=median(logged_sensor_ms(event_idx));
A.flight_plain_median_ms=median(logged_sensor_ms(plain_idx));
A.flight_event_max_ms=max(logged_sensor_ms(event_idx));
A.flight_replanner_max_ms=max(logged_replanner_ms(flight_idx));
A.event_repeated_sensor_median_ms=median(event_repeated_ms,2);
A.alternative_flight_replay_median_ms=median(alternative_sensor_ms(flight_idx));
A.original_flight_replay_median_ms=median(replay_sensor_ms(flight_idx));
A.alternative_event_repeated_median_ms=median(alt_event_repeated_ms,2);
A.alternative_steady_repeated_median_ms=median(alt_steady_repeated_ms,2);
A.same_pose_steady_sensor_median_ms=median(steady_repeated_ms,2);
A.event_repeated_sensor_p95_ms=arrayfun(@(j)p95(event_repeated_ms(j,:)), ...
    (1:numel(event_idx))');
A.same_pose_steady_sensor_p95_ms=arrayfun(@(j)p95(steady_repeated_ms(j,:)), ...
    (1:numel(event_idx))');
A.event_repeated_stage_median_ms=reshape(median(repeat_event_stage,2),numel(event_idx),5);
A.same_pose_steady_stage_median_ms=reshape(median(repeat_steady_stage,2),numel(event_idx),5);
A.stage_names=stage_names;
A.replay_flight_sensor_median_ms=median(replay_sensor_ms(flight_idx));
A.instrumented_flight_stage_ms=stage_ms; % 原因調査用一時MATのみ
A.event_repeated_sensor_ms=event_repeated_ms;
A.alternative_event_repeated_ms=alt_event_repeated_ms;
A.alternative_steady_repeated_ms=alt_steady_repeated_ms;
A.same_pose_steady_sensor_ms=steady_repeated_ms;
A.note=['区間別追加計測自体に負荷があり、本番時間の等式ではない。', ...
    '同位置反復で改善するなら、初回サイズ変更/JIT/環境中断等が候補。', ...
    '原因の断定には区間別比較と必要に応じた反復実験が必要。'];

root=fileparts(mfilename('fullpath'));
save(fullfile(root,'SENSOR_EVENT_CAUSAL_AUDIT.mat'),'A');
fid=fopen(fullfile(root,'SENSOR_EVENT_CAUSAL_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_EVENT_CAUSAL:Output','診断TXTを作成できません。');end
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',A.status);
fprintf(fid,'records=%d event_count=%d mismatches=%d\n',n,numel(event_idx),numel(failures));
fprintf(fid,'Flight sensor event/non-event median=%.6f/%.6f ms; Flight replanner max=%.6f ms\n', ...
    A.flight_event_median_ms,A.flight_plain_median_ms,A.flight_replanner_max_ms);
fprintf(fid,'Replay steady original/template median=%.6f/%.6f ms\n', ...
    A.original_flight_replay_median_ms,A.alternative_flight_replay_median_ms);
fprintf(fid,'STAGES geometry visible_selection event_id_masks result_pack timing_and_state [ms]\n');
for j=1:numel(event_idx)
    fprintf(fid,'EVENT idx=%d t=%.6f candidates=%d logged_sensor=%.6f logged_replanner=%.6f\n', ...
        event_idx(j),A.event_time_s(j),A.event_candidates_checked(j), ...
        A.event_sensor_logged_ms(j),A.event_replanner_logged_ms(j));
    fprintf(fid,' event-probe stage: %s\n',mat2str(A.event_stage_probe_ms(j,:),7));
    fprintf(fid,' neighbor probe median: %s\n',mat2str(A.neighbor_stage_probe_median_ms(j,:),7));
    fprintf(fid,' repeat same source event median/p95=%.6f/%.6f vs same-pose non-event median/p95=%.6f/%.6f\n', ...
        A.event_repeated_sensor_median_ms(j),A.event_repeated_sensor_p95_ms(j), ...
        A.same_pose_steady_sensor_median_ms(j),A.same_pose_steady_sensor_p95_ms(j));
    fprintf(fid,' repeat event original/template median=%.6f/%.6f, same-pose steady original/template=%.6f/%.6f ms\n', ...
        A.event_repeated_sensor_median_ms(j),A.alternative_event_repeated_median_ms(j), ...
        A.same_pose_steady_sensor_median_ms(j),A.alternative_steady_repeated_median_ms(j));
    fprintf(fid,' repeat event stage medians: %s\n',mat2str(A.event_repeated_stage_median_ms(j,:),7));
    fprintf(fid,' repeat non-event stage medians: %s\n',mat2str(A.same_pose_steady_stage_median_ms(j,:),7));
end
for j=1:min(8,numel(failures))
    fprintf(fid,'ERROR %s\n',failures{j});
end
fprintf(fid,'NOTE %s\n',A.note);
fprintf('[CAUSAL SENSOR EVENT] %s | event median %.4f ms vs plain median %.4f ms\n', ...
    A.status,A.flight_event_median_ms,A.flight_plain_median_ms);
end

function v=get_record(records,i)
if iscell(records),v=records{i};else,v=records(i);end
if ~isstruct(v)||~isfield(v,'simulated_obstacle_sensor')|| ...
        ~isfield(v,'replanning_time_ms')
    error('CHECK_SENSOR_EVENT_CAUSAL:Schema','LOGGER記録%dのフィールドが不正です。',i);
end
end

function value=p95(v)
v=sort(v(:));value=v(max(1,ceil(.95*numel(v))));
end
