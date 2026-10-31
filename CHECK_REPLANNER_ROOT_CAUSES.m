function report = CHECK_REPLANNER_ROOT_CAUSES(replanner_instance,agent,time)
% CHECK_REPLANNER_ROOT_CAUSES  update()初期スパイクの複数仮説を切り分ける一時診断。
% 使用: clear classes; run_flight_headless
%       D = CHECK_REPLANNER_UPDATE_SPIKES(replanner_instance);
%       C = CHECK_REPLANNER_ROOT_CAUSES(replanner_instance,agent,time);
%
% 注意:
%  - 実Flight終了後に実行すること（飛行ループ中には使用しない）。
%  - 実機・制御・plantには一切入力を送らない。試験用REPLANNING_SYSTEMだけを生成。
%  - agent.reference.resultは終了時（例外時も）元の値に復元する。
%  - 現在使用中の本番replanner_instanceのセンサ履歴・ログ件数は変更しない。
%  - 追加ベンチマークは「ウォーム状態」の処理費用を比較する。初期遅延の
%    原因は、この実験だけでは断定できない（別Flight比較が必要）。
%
if nargin~=3 || ~isa(replanner_instance,'REPLANNING_SYSTEM')
    error('CHECK_REPLANNER_ROOT_CAUSES:Inputs', ...
        'Flight後の (replanner_instance,agent,time) を渡してください。');
end
if ~ismethod(replanner_instance,'get_diagnostic_update_timing')
    error('CHECK_REPLANNER_ROOT_CAUSES:DiagnosticSource', ...
        '最初の20回を測る診断版REPLANNING_SYSTEM.mを使ってください。');
end
if ~(isobject(agent) && isscalar(agent) && isprop(agent,'reference'))
    error('CHECK_REPLANNER_ROOT_CAUSES:Agent', ...
        'Flight終了後のDRONE agentを渡してください。');
end
if ~isstruct(agent.reference.result) || ~isscalar(agent.reference.result)
    error('CHECK_REPLANNER_ROOT_CAUSES:Reference', ...
        'agent.reference.resultがスカラー構造体ではありません。');
end
if ~isobject(time) || ~isscalar(time) || ~isprop(time,'t')
    error('CHECK_REPLANNER_ROOT_CAUSES:Time', ...
        'Flight終了後のTIMEを渡してください。');
end
original_reference = agent.reference.result;
cleanup_ref = onCleanup(@() restore_reference(agent,original_reference)); %#ok<NASGU>
source_path = which('REPLANNING_SYSTEM');
sensor_path = which('SIMULATED_OBSTACLE_SENSOR');
geometry_path = which('OBSTACLE_GEOMETRY_QUERY');

original_status = replanner_instance.get_status();
D = replanner_instance.get_diagnostic_update_timing();
if size(D.stage_ms,1)<10
    error('CHECK_REPLANNER_ROOT_CAUSES:NotEnough', ...
        '最初の20回の診断記録が不足しています。');
end

report=struct();
report.status='DEEP_DIAGNOSTIC_FOR_ROOT_CAUSE_NOT_YET_PROOF';
report.source_path=source_path;
report.sensor_path=sensor_path;
report.geometry_path=geometry_path;
report.matlab_version=version;
report.preflight_ready=original_status.preflight_warmup_ready;
report.preflight_ms=original_status.preflight_warmup_ms;
report.preflight_geometry_checks=original_status.preflight_warmup_distance_checks;
report.original_scan_count=original_status.sensor_status.scan_count;
report.original_frame_count=original_status.frame_count;
report.original_timing_count=original_status.replanner_timing_count;
report.diagnostic=D;

A=double(D.stage_ms);
ref=median(A(7:end,:),1);
report.steady_stage_median_ms=ref;
report.early_stage_ms=A(1:5,:);
report.row5_to_steady_ratio=A(5,:)./max(ref,eps);
report.initial_logged_ms=D.recorded_update_ms(1:5);
report.after_early_logged_median_ms=median(D.recorded_update_ms(7:end));
report.measurement_gap_ms=D.inclusive_update_ms-D.recorded_update_ms;
report.stage_labels=D.stage_names;
report.concurrent_slow_stage_count=sum(bsxfun(@gt,A,5*max(ref,eps)),2);

% A/B:同じFlight終了後の実際のDRONE/TIMEオブジェクトで update() を測る。
% 元のreplannerを使うと履歴が進んでしまうので、必ず独立したインスタンスで比較。
options=struct('ENABLE_ALL_LOG',logical(original_status.ENABLE_ALL_LOG), ...
    'LOG_OBSTACLES',logical(original_status.LOG_OBSTACLES), ...
    'ENABLE_SENSOR',logical(original_status.ENABLE_SENSOR), ...
    'LOG_SENSOR',logical(original_status.LOG_SENSOR), ...
    'LOG_REPLANNER',logical(original_status.LOG_REPLANNER), ...
    'SENSOR_RANGE_M',double(original_status.SENSOR_RANGE_M));

names={'full','no_sensor','no_obstacle_log','no_sensor_no_obstacle_log', ...
       'sensor_without_timing_log','no_replanner_timing_log','all_logs_off'};
N=numel(names);
B=struct('name',cell(1,N),'first_update_ms',cell(1,N), ...
    'median_after24_ms',cell(1,N),'p95_after24_ms',cell(1,N), ...
    'max_after24_ms',cell(1,N),'logged_vs_wall_median_gap_ms',cell(1,N), ...
    'preflight_ms',cell(1,N),'all_60_update_ms',cell(1,N));
nsamples=60;

for b=1:N
    op=options;
    switch names{b}
        case 'no_sensor'
            op.ENABLE_SENSOR=false;
        case 'no_obstacle_log'
            op.LOG_OBSTACLES=false;
        case 'no_sensor_no_obstacle_log'
            op.ENABLE_SENSOR=false; op.LOG_OBSTACLES=false;
        case 'sensor_without_timing_log'
            op.LOG_SENSOR=false;
        case 'no_replanner_timing_log'
            op.LOG_REPLANNER=false;
        case 'all_logs_off'
            op.ENABLE_ALL_LOG=false;
    end
    % コンストラクタとウォームアップはベンチマーク時間に算入しない。
    probe=REPLANNING_SYSTEM(op);
    stat=probe.get_status();
    e=zeros(nsamples,1); gaps=nan(nsamples,1);
    for j=1:nsamples
        agent.reference.result=original_reference;
        t0=tic;
        probe.update(time,agent);
        e(j)=1000*toc(t0);
        if op.ENABLE_ALL_LOG && op.LOG_REPLANNER && ...
                isfield(agent.reference.result,'replanning_time_ms')
            gaps(j)=e(j)-double(agent.reference.result.replanning_time_ms);
        end
    end
    B(b).name=names{b};
    B(b).first_update_ms=e(1);
    B(b).median_after24_ms=median(e(25:end));
    B(b).p95_after24_ms=percentile95(e(25:end));
    B(b).max_after24_ms=max(e(25:end));
    B(b).logged_vs_wall_median_gap_ms=median_finite(gaps(25:end));
    B(b).preflight_ms=stat.preflight_warmup_ms;
    B(b).all_60_update_ms=e;
    agent.reference.result=original_reference;
end
report.ab_comparison=B;
report.ab_note=['同一MATLABセッションで、Flight後の実際のagent/timeを使った' ...
    'ウォーム状態の比較。A/B実行順やJITキャッシュの影響は排除できない。' ...
    '初回5 msを直接再現する実験ではない。'];
report.audited_reference_restored=isequaln(agent.reference.result,original_reference);
check=replanner_instance.get_status();
report.original_replanner_unchanged = ...
    (check.sensor_status.scan_count==original_status.sensor_status.scan_count) && ...
    (check.frame_count==original_status.frame_count) && ...
    (check.replanner_timing_count==original_status.replanner_timing_count);

% txt+mat: 一時的なファイルのみを検証関数の隣に保存。
dirpath=fileparts(mfilename('fullpath'));
matpath=fullfile(dirpath,'REPLANNER_ROOT_CAUSE_AUDIT.mat');
txtpath=fullfile(dirpath,'REPLANNER_ROOT_CAUSE_AUDIT.txt');
save(matpath,'report');
fid=fopen(txtpath,'wt');
if fid<0
    error('CHECK_REPLANNER_ROOT_CAUSES:CannotWrite','TXT出力に失敗しました。');
end
close_file=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',report.status);
fprintf(fid,'MATLAB=%s\n',report.matlab_version);
fprintf(fid,'REPLANNING_SYSTEM=%s\n',source_path);
fprintf(fid,'SENSOR=%s\n',sensor_path);
fprintf(fid,'GEOMETRY=%s\n',geometry_path);
fprintf(fid,'preflight_ready=%d preflight_ms=%.6f distance_checks=%d\n', ...
    report.preflight_ready,report.preflight_ms,report.preflight_geometry_checks);
fprintf(fid,'original_state_unchanged=%d reference_restored=%d\n', ...
    report.original_replanner_unchanged,report.audited_reference_restored);
fprintf(fid,'initial_logged_update_ms=');fprintf(fid,' %.6f',report.initial_logged_ms);fprintf(fid,'\n');
fprintf(fid,'post_early_logged_median_ms=%.6f\n',report.after_early_logged_median_ms);
fprintf(fid,'stage_name steady_median_ms time_0.075_ms ratio\n');
for k=1:numel(report.stage_labels)
    fprintf(fid,'%s %.6f %.6f %.2f\n',report.stage_labels{k}, ...
        ref(k),A(5,k),report.row5_to_steady_ratio(k));
end
fprintf(fid,'concurrent_slow_stage_count_first20=');
fprintf(fid,' %d',report.concurrent_slow_stage_count);fprintf(fid,'\n');
fprintf(fid,'first20_gap_inclusive_minus_recorded_ms=');
fprintf(fid,' %.6f',report.measurement_gap_ms);fprintf(fid,'\n');
fprintf(fid,'A_B_WARM_AFTER_FLIGHT\n');
fprintf(fid,'name first_ms median_after24_ms p95_after24_ms max_after24_ms wall_minus_logged_ms\n');
for b=1:N
    fprintf(fid,'%s %.6f %.6f %.6f %.6f %.6f\n', ...
        B(b).name,B(b).first_update_ms,B(b).median_after24_ms, ...
        B(b).p95_after24_ms,B(b).max_after24_ms, ...
        B(b).logged_vs_wall_median_gap_ms);
end
fprintf(fid,'NOTE %s\n',report.ab_note);

fprintf('[ROOT CAUSE] stage first-five, 0.075s, and warm A/B collected.\n');
fprintf('[ROOT CAUSE] original replanner untouched=%d | reference restored=%d\n', ...
    report.original_replanner_unchanged,report.audited_reference_restored);
fprintf('[ROOT CAUSE] report: REPLANNER_ROOT_CAUSE_AUDIT.txt / .mat\n');
end

function restore_reference(agent,original_reference)
agent.reference.result=original_reference;
end

function p=percentile95(v)
v=sort(v(isfinite(v)));
if isempty(v),p=NaN;return;end
p=v(max(1,ceil(.95*numel(v))));
end

function m=median_finite(v)
v=v(isfinite(v));
if isempty(v),m=NaN;else,m=median(v);end
end
