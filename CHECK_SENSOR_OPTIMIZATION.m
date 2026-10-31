function report = CHECK_SENSOR_OPTIMIZATION(logger)
% CHECK_SENSOR_OPTIMIZATION  最適化前後のセンサ結果の同値性と実測時間。
%
%     report = CHECK_SENSOR_OPTIMIZATION(logger);  % 直前Flightの2,602周期等を再現
%     report = CHECK_SENSOR_OPTIMIZATION();        % Flightなしの合成データ
%
% この検証は一時ファイル。本番用のセンサ・形状定義は書き換えない。
% 同梱のBASELINEクラスは、最適化前の2ファイルを名前だけ変えたもの。
% 元形状距離式・閾値ガード・検知イベントは最適化前後で照合する。
% 計測は同じ環境で交互に実行する。実行速度の改善は結果で判断する。
% 「全形状/全数値での形式的な同値性証明」ではない。

source = 'SYNTHETIC';
if nargin >= 1 && ~isempty(logger)
    source = 'FLIGHT_LOG';
end
saved_rng=rng;
cleanup=onCleanup(@()rng(saved_rng)); %#ok<NASGU>
rng(612,'twister');
errors={};
checks=0;
comparison_count=0;
obstacles=OBSTACLE_INFORMATION.definitions();

% 1) 既存Flightの全センサ入力を再生。未指定なら円柱3個の周囲を走査する。
if strcmp(source,'FLIGHT_LOG')
    if ~(isobject(logger) && isprop(logger,'Data') && isprop(logger,'k'))
        error('CHECK_SENSOR_OPTIMIZATION:Logger','Sim LOGGERを渡してください。');
    end
    n=double(logger.k);
    R=logger.Data.agent(1).reference.result;
    if ~iscell(R),R=num2cell(R);end
    times=zeros(n,1);
    P=zeros(3,n);
    ranges=zeros(n,1);
    for k=1:n
        if k>numel(R) || ~isstruct(R{k}) || ~isfield(R{k},'simulated_obstacle_sensor')
            error('CHECK_SENSOR_OPTIMIZATION:FlightData', ...
                '記録%dにセンサ記録がありません。',k);
        end
        f=R{k}.simulated_obstacle_sensor;
        times(k)=double(f.time_s);
        P(:,k)=double(f.uav_position_ekf_m(:));
        ranges(k)=double(f.sensor_range_m);
    end
else
    n=2602;
    times=(0:n-1)'*0.025;
    P=zeros(3,n);
    ranges=20*ones(n,1);
    for k=1:n
        t=times(k);
        P(:,k)=[3*sin(t/6)+0.1*cos(t); 1.5*t-5; 3+sin(t/5)];
    end
end

% 最初に単体試験で全形状と境界を検証。比較は静止・移動を含む。
shape_names={'sphere','ellipsoid','box','cylinder','capsule', ...
    'cone','frustum','pyramid','regular_prism','torus', ...
    'polygon_prism','polyhedron'};
V=[0 0 0;2 0 0;0 2 0;0 0 2];
F=[1 3 2;1 2 4;1 4 3;2 3 4];
outline=[0 0;4 0;4 1;1 1;1 3;0 3];
dims={2,[2 1 3],[2 4 6],[2 4],[0.5 3],[2 3], ...
    [2 1 3],[2 4],[6 2 3],[2 0.5], ...
    struct('outline',outline,'height',2), ...
    struct('vertices',V,'faces',F)};
for j=1:numel(shape_names)
    try
        if mod(j,2)==0
            obs=OBSTACLE_INFORMATION.define(100+j,shape_names{j}, ...
                [1 -2 3],dims{j},'euler_deg',[10 -20 35]);
        else
            obs=OBSTACLE_INFORMATION.define(100+j,shape_names{j}, ...
                [1 -2 3],dims{j},'velocity',[0.1 -0.08 0.03]);
        end
        old=SENSOR_BASELINE_REFERENCE(obs,0.1,false);
        now=SIMULATED_OBSTACLE_SENSOR(obs,0.1,false);
        % 内部・近傍・遠方・検知境界周辺で比較。
        centers=[1;-2;3];
        for k=1:55
            t=(k-1)*0.1;
            q=centers + 4*(2*rand(3,1)-1);
            if k==1,q=centers;end
            if k==2,q=centers+[0.1;0;0];end
            if k==3,q=centers+[20;0;0];end
            if k==4,q=centers+[0;0;0.1];end
            old_frame=old.observe(t,q);
            now_frame=now.observe(t,q);
            compare_frame(old_frame,now_frame,sprintf('%s[%d]',shape_names{j},k));
            comparison_count=comparison_count+1;
        end
        checks=checks+1;
    catch ex
        errors{end+1}=sprintf('%s: %s',shape_names{j},ex.message); %#ok<AGROW>
    end
end

% 2) 実飛行入力または合成軌道の全件で、ID・イベント・距離・形状を照合。
old=SENSOR_BASELINE_REFERENCE(obstacles,ranges(1),true);
now=SIMULATED_OBSTACLE_SENSOR(obstacles,ranges(1),true);
old_times=nan(n,1);
now_times=nan(n,1);
for k=1:n
    try
        if ranges(k)~=old.range_m
            old.set_range(ranges(k));
            now.set_range(ranges(k));
        end
        % 実際の実行順序による偏りを抑えるため交互に実行する。
        if mod(k,2)==0
            new_frame=now.observe(times(k),P(:,k));
            old_frame=old.observe(times(k),P(:,k));
        else
            old_frame=old.observe(times(k),P(:,k));
            new_frame=now.observe(times(k),P(:,k));
        end
        compare_frame(old_frame,new_frame,sprintf('flight[%d]',k));
        old_times(k)=old_frame.elapsed_ms;
        now_times(k)=new_frame.elapsed_ms;
        comparison_count=comparison_count+1;
    catch ex
        if numel(errors)<20
            errors{end+1}=sprintf('trace[%d]: %s',k,ex.message); %#ok<AGROW>
        end
    end
end
checks=checks+1;

% 初回のJIT/初期化時間を除いた中央値・P95を併記する。
% ただし最大時間は初回を含む全計測から求める。
warmup=min(100,floor(n/10));
old_stats=stats(old_times(warmup+1:end),old_times);
now_stats=stats(now_times(warmup+1:end),now_times);
report=struct();
report.status='PASS_SENSOR_OPTIMIZATION_EQUIVALENCE';
if ~isempty(errors),report.status='FAIL_SENSOR_OPTIMIZATION_EQUIVALENCE';end
report.source=source;
report.comparisons=comparison_count;
report.shape_checks=checks;
report.trace_samples=n;
report.errors=errors;
report.baseline=old_stats;
report.optimized=now_stats;
report.median_speedup=old_stats.median_ms/now_stats.median_ms;
report.elapsed_baseline_ms=old_times;
report.elapsed_optimized_ms=now_times;
if isfinite(report.median_speedup) && report.median_speedup>1
    report.speed_result='IMPROVED_IN_THIS_RUN';
else
    report.speed_result='NOT_CONFIRMED_IN_THIS_RUN';
end

root=fileparts(mfilename('fullpath'));
try
    save(fullfile(root,'SENSOR_OPTIMIZATION_AUDIT.mat'),'report');
    f=fopen(fullfile(root,'SENSOR_OPTIMIZATION_AUDIT.txt'),'w');
    if f>=0
        fprintf(f,'%s\n',report.status);
        fprintf(f,'source=%s | comparisons=%d | errors=%d\n',source,comparison_count,numel(errors));
        fprintf(f,'baseline median=%.6f ms | p95=%.6f ms | max=%.6f ms\n', ...
            old_stats.median_ms,old_stats.p95_ms,old_stats.max_all_ms);
        fprintf(f,'optimized median=%.6f ms | p95=%.6f ms | max=%.6f ms\n', ...
            now_stats.median_ms,now_stats.p95_ms,now_stats.max_all_ms);
        fprintf(f,'median speedup=%.4f | %s\n',report.median_speedup,report.speed_result);
        for j=1:numel(errors),fprintf(f,'ERROR %d: %s\n',j,errors{j});end
        fclose(f);
    end
catch ex
    warning('CHECK_SENSOR_OPTIMIZATION:Save','保存に失敗: %s',ex.message);
end
fprintf('[SENSOR OPT] %s | comparisons=%d | errors=%d\n', ...
    report.status,comparison_count,numel(errors));
fprintf('[SENSOR OPT] median old=%.4f ms | new=%.4f ms | x%.3f | %s\n', ...
    old_stats.median_ms,now_stats.median_ms,report.median_speedup,report.speed_result);
if ~isempty(errors)
    fprintf(2,'[SENSOR OPT] %s\n',errors{1});
    error('CHECK_SENSOR_OPTIMIZATION:Mismatch', ...
        '旧センサと最適化後で検知結果が一致しません。');
end

    function compare_frame(a,b,label)
        % elapsed_msは実測値なので比較対象外。それ以外は同一でなければ失敗。
        if ~isequal(sort(fieldnames(a)),sort(fieldnames(b)))
            error('CHECK_SENSOR_OPTIMIZATION:Fields','%s フィールドが不一致',label);
        end
        fields=fieldnames(a);
        for z=1:numel(fields)
            key=fields{z};
            if strcmp(key,'elapsed_ms'),continue;end
            if any(strcmp(key,{'detected_ids','newly_detected_ids', ...
                    'left_range_ids','detected_distances_m'}))
                av=reshape(a.(key),1,[]);bv=reshape(b.(key),1,[]);
                ok=isequaln(av,bv);
            else
                ok=isequaln(a.(key),b.(key));
            end
            if ~ok
                error('CHECK_SENSOR_OPTIMIZATION:Mismatch', ...
                    '%s: %s が一致しません。',label,key);
            end
        end
    end
end

function s=stats(flight_values,all_values)
    x=sort(flight_values(isfinite(flight_values)));
    y=all_values(isfinite(all_values));
    if isempty(x)
        s=struct('median_ms',NaN,'p95_ms',NaN,'max_all_ms',NaN);
        return
    end
    s=struct('median_ms',median(x), ...
        'p95_ms',x(max(1,ceil(0.95*numel(x)))), ...
        'max_all_ms',max(y));
end
