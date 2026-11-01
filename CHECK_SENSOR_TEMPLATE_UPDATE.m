function report = CHECK_SENSOR_TEMPLATE_UPDATE(varargin)
% CHECK_SENSOR_TEMPLATE_UPDATE  現行センサとテンプレート版の厳密な出力比較と実行時間比較。
%   report = CHECK_SENSOR_TEMPLATE_UPDATE();         % 12形状 + 境界 + 移動
%   report = CHECK_SENSOR_TEMPLATE_UPDATE(logger);   % 上記 + 現Flightの全周期を再生
%
% 一時検証ファイル。実装/障害物定義を変更しない。
% elapsed_ms だけは計測により異なるため、出力同値判定から除く。
% 累積時間も測定差があるため、検知数・ID・距離・イベント・フラグを比較。
if nargin>1
    error('CHECK_SENSOR_TEMPLATE_UPDATE:Args','入力は省略またはLOGGER 1個です。');
end
logobj=[]; refs=[];
if nargin==1
    logobj=varargin{1};
end
expected_sensor_path=fullfile(fileparts(mfilename('fullpath')),'reference', ...
    'REPLANNING_SYSTEM','SIMULATED_OBSTACLE_SENSOR','SIMULATED_OBSTACLE_SENSOR.m');
actual_sensor_path=which('SIMULATED_OBSTACLE_SENSOR');
if ~strcmpi(actual_sensor_path,expected_sensor_path)
    error('CHECK_SENSOR_TEMPLATE_UPDATE:Source', ...
        '想定したセンサが読み込まれていません: %s',actual_sensor_path);
end
assert(exist('SENSOR_TEMPLATE_BASELINE','class')==8, ...
    'SENSOR_TEMPLATE_BASELINE.mがMATLABパスにありません。');
failures={}; comparisons=0;
new_ms=[]; old_ms=[];
case_count=0;

% 実際の障害物定義は読み取りだけ。静的・移動・凹形状の双方を試す。
O=OBSTACLE_INFORMATION.definitions();
run_case('FlightDefinitions',O,20,make_paths(O,150));
shape_names={'sphere','ellipsoid','box','cylinder','capsule', ...
    'cone','frustum','pyramid','regular_prism','torus', ...
    'polygon_prism','polyhedron'};
V=[0 0 0;2 0 0;0 2 0;0 0 2];
F=[1 3 2;1 2 4;1 4 3;2 3 4];
outline=[0 0;4 0;4 1;1 1;1 3;0 3];
dims={2,[2 1 3],[2 4 6],[2 4],[0.5 3], ...
    [2 3],[2 1 3],[2 4],[6 2 3],[2 0.5], ...
    struct('outline',outline,'height',2), ...
    struct('vertices',V,'faces',F)};
for j=1:numel(shape_names)
    o=OBSTACLE_INFORMATION.define(200+j,shape_names{j},[1 2 3],dims{j}, ...
        'euler_deg',[10 20 30]);
    run_case(shape_names{j},o,0.5,make_paths(o,60));
end
stationary_shifted=OBSTACLE_INFORMATION.define(302,'box',[4 -2 5],[1 2 3], ...
    'euler_deg',[17 -34 21],'reference_time_s',12);
run_case('StaticNonzeroReferenceTime',stationary_shifted,2, ...
    make_paths(stationary_shifted,80));
move=OBSTACLE_INFORMATION.define(301,'sphere',[0 0 0],1, ...
    'velocity',[1 0.2 0],'reference_time_s',0);
run_case('Moving',move,1,make_paths(move,100));

flight_replay_records=0;
if ~isempty(logobj)
    if ~isobject(logobj)||~isprop(logobj,'Data')|| ...
            ~isfield(logobj.Data,'agent')|| ...
            ~isfield(logobj.Data.agent(1),'reference')|| ...
            ~isfield(logobj.Data.agent(1).reference,'result')
        error('CHECK_SENSOR_TEMPLATE_UPDATE:Logger','LOGGERが期待した構造ではありません。');
    end
    refs=logobj.Data.agent(1).reference.result;
    n=double(logobj.k);
    samples=struct('t',cell(1,n),'p',cell(1,n));
    for i=1:n
        if iscell(refs),r=refs{i};else,r=refs(i);end
        if ~isstruct(r)||~isfield(r,'simulated_obstacle_sensor')
            error('CHECK_SENSOR_TEMPLATE_UPDATE:FlightLog', ...
                'ログ記録%dにsimulated_obstacle_sensorがありません。',i);
        end
        q=r.simulated_obstacle_sensor;
        samples(i).t=double(q.time_s);
        samples(i).p=double(q.uav_position_ekf_m(:));
    end
    flight_replay_records=n;
    run_case('ActualFlightReplay',O,20,samples);
end

report=struct();
report.status='PASS_SENSOR_TEMPLATE_OUTPUT_EQUIVALENCE';
if ~isempty(failures),report.status='FAIL_SENSOR_TEMPLATE_OUTPUT_EQUIVALENCE';end
report.comparisons=comparisons;
report.cases=case_count;
report.flight_replay_records=flight_replay_records;
report.failures=failures;
report.errors=numel(failures);
report.baseline_median_ms=median(old_ms);
report.modified_median_ms=median(new_ms);
report.baseline_p95_ms=calc_p95(old_ms);
report.modified_p95_ms=calc_p95(new_ms);
report.baseline_max_ms=max(old_ms);
report.modified_max_ms=max(new_ms);
report.median_speedup=report.baseline_median_ms/report.modified_median_ms;
report.speed_status='NOT_DEMONSTRATED';
if report.modified_median_ms<report.baseline_median_ms
    report.speed_status='IMPROVED_IN_THIS_CHECK';
end
report.note=['PASSは検知出力同値を意味する。突出時間の消滅保証ではない。', ...
    '一回の実験で最大実行時間の保証はできない。'];
root=fileparts(mfilename('fullpath'));
save(fullfile(root,'SENSOR_TEMPLATE_UPDATE_AUDIT.mat'),'report');
fid=fopen(fullfile(root,'SENSOR_TEMPLATE_UPDATE_AUDIT.txt'),'wt');
if fid<0,error('CHECK_SENSOR_TEMPLATE_UPDATE:File','ログを出力できません。');end
closeguard=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',report.status);
fprintf(fid,'cases=%d comparisons=%d flight_replay=%d errors=%d\n', ...
    case_count,comparisons,flight_replay_records,numel(failures));
fprintf(fid,'baseline median/p95/max ms=%.6f %.6f %.6f\n', ...
    report.baseline_median_ms,report.baseline_p95_ms,report.baseline_max_ms);
fprintf(fid,'modified median/p95/max ms=%.6f %.6f %.6f\n', ...
    report.modified_median_ms,report.modified_p95_ms,report.modified_max_ms);
fprintf(fid,'median_speedup=%.4f %s\n',report.median_speedup,report.speed_status);
for i=1:numel(failures),fprintf(fid,'ERROR %d: %s\n',i,failures{i});end
fprintf(fid,'NOTE %s\n',report.note);
fprintf('[SENSOR TEMPLATE UPDATE] %s | comparisons=%d | mismatches=%d | median old/new = %.4f/%.4f ms\n', ...
    report.status,comparisons,numel(failures), ...
    report.baseline_median_ms,report.modified_median_ms);

    function run_case(name,obstacles,range_m,path)
        case_count=case_count+1;
        a=SENSOR_TEMPLATE_BASELINE(obstacles,range_m,true);
        b=SIMULATED_OBSTACLE_SENSOR(obstacles,range_m,true);
        is_replay=strcmp(name,'ActualFlightReplay');
        for k=1:numel(path)
            t=path(k).t; p=path(k).p;
            if is_replay
                if iscell(refs),rr=refs{k};else,rr=refs(k);end
                truth=rr.simulated_obstacle_sensor;
                if a.range_m~=truth.sensor_range_m
                    a.set_range(truth.sensor_range_m);
                    b.set_range(truth.sensor_range_m);
                end
                if a.LOG_SENSOR~=logical(truth.time_measured)
                    a.set_log(truth.time_measured);
                    b.set_log(truth.time_measured);
                end
            else
                if k==round(numel(path)*0.6)
                    a.set_range(range_m*1.3);
                    b.set_range(range_m*1.3);
                end
                if k==round(numel(path)*0.8)
                    a.set_log(false);
                    b.set_log(false);
                end
            end
            if mod(k,2)==0
                bb=b.observe(t,p);
                aa=a.observe(t,p);
            else
                aa=a.observe(t,p);
                bb=b.observe(t,p);
            end
            aa.elapsed_ms=[];bb.elapsed_ms=[];
            if ~isequaln(aa,bb)
                failures{end+1}=sprintf('%s sample=%d t=%.5f (old/new mismatch)',name,k,t); %#ok<AGROW>
            end
            if is_replay
                truth.elapsed_ms=[];
                if ~isequaln(bb,truth)
                    failures{end+1}=sprintf('%s sample=%d t=%.5f (LOGGER mismatch)',name,k,t); %#ok<AGROW>
                end
            end
            if aa.time_measured && bb.time_measured
                old_ms(end+1)=a.last_scan_time_ms; %#ok<AGROW>
                new_ms(end+1)=b.last_scan_time_ms; %#ok<AGROW>
            end
            comparisons=comparisons+1;
        end
        sa=a.get_status();sb=b.get_status();
        if sa.scan_count~=sb.scan_count || sa.timing_count~=sb.timing_count || ...
                ~isequaln(sa.last_detected_ids,sb.last_detected_ids)
            failures{end+1}=sprintf('%s: sensor status mismatch',name); %#ok<AGROW>
        end
    end

    function path=make_paths(obstacles,n)
        path=struct('t',cell(1,n),'p',cell(1,n));
        c=obstacles(1).initial_position(:);
        for k=1:n
            tt=(k-1)*0.025;
            % 内外・境界付近を交互に含む決定的な軌跡。
            j=mod(k-1,8);
            if j==0,p=c;
            elseif j==1,p=c+[1;0;0];
            elseif j==2,p=c+[18;0;0];
            elseif j==3,p=c+[30;0;0];
            elseif j==4,p=c+[0;30;0];
            elseif j==5,p=c+[0;0;35];
            elseif j==6,p=c+[0;0;2];
            else,p=c+[100;100;100];end
            path(k).t=tt;
            path(k).p=p;
        end
    end

    function result=calc_p95(data)
        sorted=sort(data(:));
        result=sorted(max(1,ceil(.95*numel(sorted))));
    end
end
