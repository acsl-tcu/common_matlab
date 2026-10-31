function FLIGHT_LOG_VISUALIZATION(logger, obstacles, show_plots, show_animation, autoplay, fps, dt, replanner)
% FLIGHT_LOG_VISUALIZATION  Sim LOGGERの記録済み状態だけを表示。
% 実飛行ループの外で実行。元形状を楕円体へ近似しない。
% 計測時間 Figure 9 は LOGGER に実際に保存された値だけを表示する。
% 第8引数 replanner は任意。渡された場合だけ障害物読込時間も表示する。
% ※旧runの controllertime_total は実際には motive〜plant の処理時間。
%   LOGGER.logging と障害物log_frame は計測区間の外にあり、含まれない。
if nargin < 8
    replanner = [];
end
[t, p, pL, ref, pTrue, pLTrue] = read_flight_log(logger);
valid = all(isfinite(p),2) & all(isfinite(pL),2);
if nnz(valid) < 2
    error('run_flight_headless:NoTrajectory', ...
        'LOGGERの推定位置p/pLが不足しており、軌道図を描けません。');
end

if show_plots
    % Figure 5: 3Dに元障害物を描き、他の3枠で経路を投影する。
    fig = figure(5); clf(fig);
    set(fig,'Name','Suspended-load flight: 3D trajectory','NumberTitle','off');
    layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
    ax = nexttile(layout,1);
    plot_paths_3d(ax,p,pL,ref,pTrue,pLTrue,valid,obstacles);
    title(ax,'3D: reference / UAV / payload / obstacles');
    ax = nexttile(layout,2);
    plot_projection(ax,p,pL,ref,valid,1,2,'Top view (x-y)');
    ax = nexttile(layout,3);
    plot_projection(ax,p,pL,ref,valid,2,3,'Side view (y-z)');
    ax = nexttile(layout,4);
    plot_projection(ax,p,pL,ref,valid,1,3,'Front view (x-z)');

    % Figure 7: x,y,z の目標・機体・吊荷を比較。
    fig = figure(7); clf(fig);
    set(fig,'Name','Suspended-load flight: position vs time','NumberTitle','off');
    names = {'x','y','z'};
    for j = 1:3
        ax = subplot(3,1,j,'Parent',fig);
        hold(ax,'on');grid(ax,'on');box(ax,'on');
        plot_series(ax,t,ref(:,j),'--','Reference');
        plot_series(ax,t,p(:,j),'-','UAV estimated');
        plot_series(ax,t,pL(:,j),'-','Payload estimated');
        ylabel(ax,[names{j} ' [m]']);
        if j == 1
            title(ax,'Position comparison (whole flight)');
            legend(ax,'Location','best');
        end
        if j == 3, xlabel(ax,'Time [s]'); end
        hold(ax,'off');
    end

    % Figure 9: 既存Flightで計測された処理時間（全ログの時系列と分布）。
    % ログを取っていない項目に、推定値やゼロを入れて表示することはしない。
    plot_computation_timing(logger, t, dt, replanner);
end

if show_animation
    % Figure 14: 画面を先に作っておく。再生はボタンまたはauto設定で開始。
    fig = figure(14); clf(fig);
    set(fig,'Name','Suspended load: animation','NumberTitle','off');
    ax = axes('Parent',fig,'Position',[0.09 0.12 0.75 0.80]);
    plot_paths_3d(ax,p,pL,ref,pTrue,pLTrue,valid,obstacles);
    title(ax,'Replay: press button to animate UAV / cable / payload');
    replay = @() replay_flight(ax,t,p,pL,valid,fps,dt);
    uicontrol('Parent',fig,'Style','pushbutton','String','Replay animation', ...
        'Units','normalized','Position',[0.84 0.88 0.15 0.075], ...
        'Callback',@(~,~)replay());
    drawnow;
    if autoplay, replay(); end
else
    drawnow;
end
fprintf('[DISPLAY] 3D/position plots=%d | animation window=%d | autoplay=%d\n', ...
    show_plots,show_animation,autoplay);
end

function [t,p,pL,ref,pTrue,pLTrue] = read_flight_log(logger)
% 以前の動作実績がある LOGGER.Data.agent(1).*.result のセルを読み取る。
D = logger.Data;
if ~isstruct(D) || ~isfield(D,'t') || ~isfield(D,'agent') || isempty(D.agent)
    error('run_flight_headless:MissingLoggerData','LOGGER.Dataにt/agentがありません。');
end
n = double(logger.k);
if n < 1 || numel(D.t) < n
    error('run_flight_headless:IncompleteLogger','LOGGERの時刻記録が不足しています。');
end
t = double(D.t(1:n)); t = t(:);
p = nan(n,3); pL = nan(n,3); ref = nan(n,3);
pTrue = nan(n,3); pLTrue = nan(n,3);
A = D.agent(1);
E = result_cells(A,'estimator');
R = result_cells(A,'reference');
P = result_cells(A,'plant');
for k = 1:min(n,numel(E))
    state = member(E{k},'state');
    p(k,:) = xyz(member(state,'p'));
    pL(k,:) = xyz(member(state,'pL'));
end
for k = 1:min(n,numel(R))
    state = member(R{k},'state');
    ref(k,:) = xyz(member(state,'xd'));
end
for k = 1:min(n,numel(P))
    state = member(P{k},'state');
    pTrue(k,:) = xyz(member(state,'p'));
    pLTrue(k,:) = xyz(member(state,'pL'));
end
end

function C = result_cells(A, group)
C = {};
if ~isstruct(A) || ~isfield(A,group) || ...
        ~isstruct(A.(group)) || ~isfield(A.(group),'result')
    return;
end
value = A.(group).result;
if iscell(value), C = value(:); else, C = num2cell(value(:)); end
end

function out = member(obj, key)
out = [];
if isstruct(obj) && isscalar(obj) && isfield(obj,key)
    out = obj.(key);
elseif isobject(obj) && isscalar(obj) && isprop(obj,key)
    out = obj.(key);
end
end

function out = xyz(value)
out = [NaN NaN NaN];
if isnumeric(value) && numel(value) >= 3
    v = double(value(:));
    if all(isfinite(v(1:3))), out = v(1:3).'; end
end
end

function plot_series(ax,t,y,style,label)
valid = isfinite(t) & isfinite(y);
if nnz(valid) > 1
    plot(ax,t(valid),y(valid),style,'LineWidth',1.25,'DisplayName',label);
end
end

function plot_paths_3d(ax,p,pL,ref,pTrue,pLTrue,valid,obstacles)
hold(ax,'on');grid(ax,'on');box(ax,'on');
for k = 1:numel(obstacles)
    draw_obstacle_shape(ax,obstacles(k));
end
vr = all(isfinite(ref),2);
if nnz(vr)>1
    plot3(ax,ref(vr,1),ref(vr,2),ref(vr,3),'--', ...
        'LineWidth',1.5,'DisplayName','Reference');
end
plot3(ax,p(valid,1),p(valid,2),p(valid,3),'-', ...
    'LineWidth',1.5,'DisplayName','UAV estimated');
plot3(ax,pL(valid,1),pL(valid,2),pL(valid,3),'-', ...
    'LineWidth',1.5,'DisplayName','Payload estimated');
vt = all(isfinite(pTrue),2);
if nnz(vt)>1
    plot3(ax,pTrue(vt,1),pTrue(vt,2),pTrue(vt,3),':', ...
        'LineWidth',1,'DisplayName','UAV plant truth');
end
vl = all(isfinite(pLTrue),2);
if nnz(vl)>1
    plot3(ax,pLTrue(vl,1),pLTrue(vl,2),pLTrue(vl,3),':', ...
        'LineWidth',1,'DisplayName','Payload plant truth');
end
% ケーブルを1秒おきに表示。参照軌道には物理的なケーブルを付けない。
indices = find(valid);
stride = max(1,round(numel(indices)/40));
for j = 1:stride:numel(indices)
    k = indices(j);
    plot3(ax,[p(k,1) pL(k,1)],[p(k,2) pL(k,2)], ...
        [p(k,3) pL(k,3)],':','Color',[.55 .55 .55], ...
        'HandleVisibility','off');
end
xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
axis(ax,'equal');view(ax,35,25);
legend(ax,'Location','best');hold(ax,'off');
end

function plot_projection(ax,p,pL,ref,valid,x,y,titleText)
hold(ax,'on');grid(ax,'on');box(ax,'on');
vr = all(isfinite(ref(:,[x y])),2);
if nnz(vr)>1
    plot(ax,ref(vr,x),ref(vr,y),'--','LineWidth',1.3, ...
        'DisplayName','Reference');
end
plot(ax,p(valid,x),p(valid,y),'LineWidth',1.3,'DisplayName','UAV');
plot(ax,pL(valid,x),pL(valid,y),'LineWidth',1.3,'DisplayName','Payload');
axisNames = {'x','y','z'};
xlabel(ax,[axisNames{x} ' [m]']);ylabel(ax,[axisNames{y} ' [m]']);
title(ax,titleText);legend(ax,'Location','best');hold(ax,'off');
end

function draw_obstacle_shape(ax,o)
% 元障害物のみ可視化。近接判定・安全マージン・楕円包絡は使用しない。
type = o.geometry.type;
dims = [];
if isnumeric(o.geometry.dimensions)
    dims = double(o.geometry.dimensions(:));
end
R = o.rotation;
c = o.position(:);
segments = 48;
theta = linspace(0,2*pi,segments+1);

switch type
    case 'cylinder'
        radius = dims(1); halfHeight = dims(2)/2;
        U = [radius*cos(theta);radius*sin(theta)];
        local = [U U; ...
                 -halfHeight*ones(1,numel(theta)), halfHeight*ones(1,numel(theta))];
        local = reshape(local,[3,numel(theta),2]);
        transformed = R*reshape(local,3,[]) + c;
        X = reshape(transformed(1,:),numel(theta),2).';
        Y = reshape(transformed(2,:),numel(theta),2).';
        Z = reshape(transformed(3,:),numel(theta),2).';
        surf(ax,X,Y,Z,'FaceColor',[.95 .47 .20],'FaceAlpha',.20, ...
            'EdgeAlpha',.18,'HandleVisibility','off');
        for cap = [-halfHeight,halfHeight]
            localCap = [U;cap*ones(1,numel(theta))];
            world = R*localCap + c;
            patch(ax,world(1,:),world(2,:),world(3,:),[.95 .47 .20], ...
                'FaceAlpha',.10,'EdgeColor','none','HandleVisibility','off');
        end
    case {'sphere','ellipsoid'}
        [xs,ys,zs] = sphere(28);
        if strcmp(type,'sphere'), radii = dims(1)*ones(3,1);
        else, radii = dims(1:3); end
        U = diag(radii)*[xs(:)';ys(:)';zs(:)'];
        W = R*U+c;
        surf(ax,reshape(W(1,:),size(xs)),reshape(W(2,:),size(ys)), ...
            reshape(W(3,:),size(zs)),'FaceColor',[.95 .47 .20], ...
            'FaceAlpha',.2,'EdgeAlpha',.12,'HandleVisibility','off');
    case 'box'
        b = dims(1:3)/2;
        V = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
             -1 -1 1;1 -1 1;1 1 1;-1 1 1].*b.';
        W = (R*V.'+c).';
        faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
        patch(ax,'Vertices',W,'Faces',faces,'FaceColor',[.95 .47 .20], ...
            'FaceAlpha',.2,'EdgeColor',[.75 .32 .12], ...
            'HandleVisibility','off');
    otherwise
        % 別形状を楕円体・球で代用しない。中心と名前だけ表示する。
        plot3(ax,c(1),c(2),c(3),'x','Color',[.75 .30 .10], ...
            'HandleVisibility','off');
        warning('run_flight_headless:UnrenderedObstacle', ...
            '障害物ID %d (%s)の表面描画は未対応のため中心だけ表示しました。',o.id,type);
end
text(ax,c(1),c(2),c(3),sprintf('  ID %d (%s)',o.id,type), ...
    'FontSize',9,'Color',[.5 .2 .1]);
end

function replay_flight(ax,t,p,pL,valid,fps,dt)
% 描画専用。元のLOGGERは一切変更しない。
if ~isgraphics(ax), return; end
fps = max(1,double(fps));
idx = find(valid);
stride = max(1,round(1/(fps*dt)));
idx = idx(1:stride:end);
if numel(idx)<2, return; end

% Replayボタンを連続して押しても前回の表示を積み重ねない。
delete(findobj(ax,'Tag','FlightReplayDynamic'));
hold(ax,'on');
hU = plot3(ax,NaN,NaN,NaN,'o','MarkerSize',9,'LineWidth',2, ...
    'DisplayName','UAV (current)','Tag','FlightReplayDynamic');
hL = plot3(ax,NaN,NaN,NaN,'o','MarkerSize',9,'LineWidth',2, ...
    'DisplayName','Payload (current)','Tag','FlightReplayDynamic');
hC = plot3(ax,NaN,NaN,NaN,'-','LineWidth',2, ...
    'DisplayName','Cable (current)','Tag','FlightReplayDynamic');
hold(ax,'off');
for j = 1:numel(idx)
    if ~isgraphics(ax),return;end
    ticFrame = tic;
    k = idx(j);
    set(hU,'XData',p(k,1),'YData',p(k,2),'ZData',p(k,3));
    set(hL,'XData',pL(k,1),'YData',pL(k,2),'ZData',pL(k,3));
    set(hC,'XData',[p(k,1),pL(k,1)], ...
        'YData',[p(k,2),pL(k,2)],'ZData',[p(k,3),pL(k,3)]);
    title(ax,sprintf('UAV / cable / payload: t = %.2f s',t(k)));
    drawnow limitrate;
    remaining = 1/fps - toc(ticFrame);
    if remaining > 0, pause(remaining); end
end
end


function plot_computation_timing(logger, t, dt, replanner)
% 現行runは毎tickの処理終了時に controller.result.controllertime_total
% を更新する。ただし名称に反して、計測範囲は motive〜plant のみ。
% LOGGER保存/障害物log_frame/描画の時間は含まれない。

C = result_cells(logger.Data.agent(1), 'controller');
n = numel(t);
tick_ms = nan(n,1);
for k = 1:min(n,numel(C))
    seconds = member(C{k},'controllertime_total');
    if isnumeric(seconds) && isreal(seconds) && isscalar(seconds) && ...
            isfinite(seconds) && seconds >= 0
        tick_ms(k) = double(seconds)*1000;
    end
end
% 最初のarmingフレームに残った過去の値は計測値とみなさない。
if n > 0 && abs(t(1)) < eps
    tick_ms(1) = NaN;
end

fig = figure(9); clf(fig);
set(fig,'Name','Flight: recorded computation time','NumberTitle','off');
valid = isfinite(t) & isfinite(tick_ms);

ax1 = subplot(2,1,1,'Parent',fig);
hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
if any(valid)
    plot(ax1,t(valid),tick_ms(valid),'LineWidth',1.0, ...
        'DisplayName','motive to plant (before LOGGER)');
    if isnumeric(dt) && isscalar(dt) && isfinite(dt) && dt > 0
        yline(ax1,dt*1000,'--',sprintf('dt = %.3g ms',dt*1000), ...
            'DisplayName','Sampling period');
    end
    legend(ax1,'Location','best');
else
    text(ax1,0.5,0.5,'No per-tick timing recorded in LOGGER', ...
        'Units','normalized','HorizontalAlignment','center');
end
xlabel(ax1,'Simulation time [s]'); ylabel(ax1,'Elapsed time [ms]');
title(ax1,'Recorded processing time: motive / EKF / reference / HLC / plant');
hold(ax1,'off');

ax2 = subplot(2,1,2,'Parent',fig);
grid(ax2,'on'); box(ax2,'on');
if any(valid)
    values = tick_ms(valid);
    histogram(ax2,values,'BinMethod','auto');
    xlabel(ax2,'Elapsed time [ms]'); ylabel(ax2,'Number of cycles');
    sorted = sort(values);
    nValid = numel(sorted);
    median_ms = median(sorted);
    p95_ms = sorted(max(1,ceil(0.95*nValid)));
    max_ms = sorted(end);
    title(ax2,sprintf('N=%d, median=%.3f ms, P95=%.3f ms, max=%.3f ms', ...
        nValid,median_ms,p95_ms,max_ms));
    fprintf('[TIMING] motive-to-plant (excludes logging): N=%d | median=%.3f ms | P95=%.3f ms | max=%.3f ms\n', ...
        nValid,median_ms,p95_ms,max_ms);
else
    text(ax2,0.5,0.5,'No recorded elapsed times', ...
        'Units','normalized','HorizontalAlignment','center');
    xlabel(ax2,'Elapsed time [ms]'); ylabel(ax2,'Number of cycles');
end

% 障害物定義ロード/フレーム生成の累積時間はランタイム統計からのみ読める。
% 毎tickの各モジュール時間とは区別し、可視化には実測値のみを書く。
if ~isempty(replanner)
    if isobject(replanner) && ismethod(replanner,'get_status')
        status = replanner.get_status();
    elseif isstruct(replanner) && isscalar(replanner)
        status = replanner;
    else
        error('FLIGHT_LOG_VISUALIZATION:InvalidStatus', ...
            '第8引数は get_status() 対応オブジェクトか状態構造体です。');
    end
    if isfield(status,'definition_load_ms') && ...
            isfield(status,'total_frame_time_ms') && isfield(status,'frame_count')
        load_ms = status.definition_load_ms;
        sum_ms = status.total_frame_time_ms;
        count = status.frame_count;
        if isfinite(load_ms) && isfinite(sum_ms) && isfinite(count)
            frame_mean_ms = NaN;
            if count > 0, frame_mean_ms = sum_ms/count; end
            sgtitle(fig,sprintf(['Obstacle definitions loaded: %.3f ms | ' ...
                'log_frame total: %.3f ms (%d calls, avg %.5f ms)'], ...
                load_ms,sum_ms,count,frame_mean_ms));
            fprintf('[TIMING] obstacles: definition load=%.3f ms | frame total=%.3f ms | frames=%d | mean/frame=%.5f ms\n', ...
                load_ms,sum_ms,count,frame_mean_ms);
        end
    end
end
end
