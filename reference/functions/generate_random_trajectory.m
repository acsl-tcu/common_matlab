function traj = generate_random_trajectory(varargin)
%GENERATE_RANDOM_TRAJECTORY 制約付き3次元ランダムreference軌道を生成する。
%   traj = generate_random_trajectory
%   traj = generate_random_trajectory('Seed', 42, 'Plot', false)
%
%   名前と値のオプション:
%     WaypointInterval : waypoint間の時間 [s]（既定値: 2）
%     SampleTime       : referenceの出力周期 [s]（既定値: 0.01）
%     Seed             : 再現用の乱数seed（既定値: []、現在の乱数列を使用）
%     MaxAttempts      : 再生成を含む試行回数上限（既定値: 10000）
%     Plot             : 3次元図・時系列図を表示（既定値: true）
%     Verbose          : 検証結果を表示（既定値: true）
%     NumWaypoints     : waypoint数（既定値: 17、4以上）
%     StartPoint       : 固定する始点 [x y z] [m]（既定値: [0 0 0.6]）
%     LowerBound       : [x y z] 下限 [m]（既定値: [-1 -1 0.6]）
%     UpperBound       : [x y z] 上限 [m]（既定値: [1 1 1.3]）
%     MaxStep          : waypoint間の最大変化量 [m]（既定値: [0.5 0.5 0.2]）
%     AngleLimit       : roll・pitchの絶対値上限 [rad]（既定値: 0.5）
%     Coverage         : 各軸の軌道の利用幅/指定幅の下限（既定値: 0.9）
%                        0以上1未満。0ならランダムウォークから候補を生成。
%                        完全に従来の生成方法にする場合はWiggleも0にする。
%     Wiggle           : 途中のwaypointに加えるうねりの強さ（0～1、既定0.8）
%
%   主な出力（各行が1時刻、各列が座標成分）:
%     waypoint         : NumWaypoints-by-3 の [x y z] [m]
%     waypointTime     : NumWaypoints-by-1 の通過時刻 [s]
%     t                : N-by-1 のreference時刻 [s]
%     position         : N-by-3 の位置 [m]
%     velocity         : N-by-3 の速度 [m/s]
%     acceleration     : N-by-3 の加速度 [m/s^2]
%     roll,pitch,yaw   : N-by-1 の目標姿勢 [rad]
%     reference        : N-by-7 の [t x y z roll pitch yaw]
%     ppPosition       : x,y,z の連続時間スプライン（1-by-3 cell）
%
%   姿勢の前提: z上向き、yaw=0、ZYX Euler角、推力は機体+z方向、
%   空気抵抗なし。|roll|, |pitch| <= AngleLimit（既定0.5 rad）を検証。
%   位置の極値と加速度の区間上界から、サンプル間も含めて検証する。
%   姿勢角速度や任意2時刻間の角度差に対する制約ではない。
%   基本MATLABのみ使用（追加Toolbox不要）。

    parser = inputParser;
    addParameter(parser, 'WaypointInterval', 2);
    addParameter(parser, 'SampleTime', 0.01);
    addParameter(parser, 'Seed', []);
    addParameter(parser, 'MaxAttempts', 10000);
    addParameter(parser, 'Plot', true);
    addParameter(parser, 'Verbose', true);
    addParameter(parser, 'NumWaypoints', 17);
    addParameter(parser, 'StartPoint', [0, 0, 0.6]);
    addParameter(parser, 'LowerBound', [-1, -1, 0.6]);
    addParameter(parser, 'UpperBound', [1, 1, 1.3]);
    addParameter(parser, 'MaxStep', [0.5, 0.5, 0.2]);
    addParameter(parser, 'AngleLimit', 0.5);
    addParameter(parser, 'Coverage', 0.9);
    addParameter(parser, 'Wiggle', 0.8);
    parse(parser, varargin{:});
    opt = parser.Results;
    validateattributes(opt.WaypointInterval, {'numeric'}, ...
        {'real','scalar','finite','positive'}, mfilename, 'WaypointInterval');
    validateattributes(opt.SampleTime, {'numeric'}, ...
        {'real','scalar','finite','positive'}, mfilename, 'SampleTime');
    validateattributes(opt.MaxAttempts, {'numeric'}, ...
        {'real','scalar','finite','integer','positive'}, mfilename, 'MaxAttempts');
    validateattributes(opt.Plot, {'logical'}, {'scalar'}, mfilename, 'Plot');
    validateattributes(opt.Verbose, {'logical'}, {'scalar'}, mfilename, 'Verbose');
    validateattributes(opt.NumWaypoints, {'numeric'}, ...
        {'real','scalar','finite','integer','>=',4}, mfilename, 'NumWaypoints');
    validateattributes(opt.LowerBound, {'numeric'}, ...
        {'real','size',[1,3],'finite'}, mfilename, 'LowerBound');
    validateattributes(opt.UpperBound, {'numeric'}, ...
        {'real','size',[1,3],'finite'}, mfilename, 'UpperBound');
    if any(opt.LowerBound >= opt.UpperBound)
        error('generate_random_trajectory:InvalidBounds', ...
            '各座標の下限は上限より小さくしてください。');
    end
    validateattributes(opt.StartPoint, {'numeric'}, ...
        {'real','size',[1,3],'finite'}, mfilename, 'StartPoint');
    if any(opt.StartPoint < opt.LowerBound | opt.StartPoint > opt.UpperBound)
        error('generate_random_trajectory:StartPointOutOfBounds', ...
            '始点が座標範囲外です。StartPointと座標の上下限を確認してください。');
    end
    validateattributes(opt.MaxStep, {'numeric'}, ...
        {'real','size',[1,3],'finite','nonnegative'}, mfilename, 'MaxStep');
    validateattributes(opt.AngleLimit, {'numeric'}, ...
        {'real','scalar','finite','positive','<',pi/2}, mfilename, 'AngleLimit');
    validateattributes(opt.Coverage, {'numeric'}, ...
        {'real','scalar','finite','nonnegative','<',1}, mfilename, 'Coverage');
    validateattributes(opt.Wiggle, {'numeric'}, ...
        {'real','scalar','finite','nonnegative','<=',1}, mfilename, 'Wiggle');
    if ~isempty(opt.Seed)
        validateattributes(opt.Seed, {'numeric'}, ...
            {'real','scalar','finite','integer','nonnegative','<=',2^32-1}, ...
            mfilename, 'Seed');
        previousRng = rng;
        restoreRng = onCleanup(@() rng(previousRng));
        rng(double(opt.Seed), 'twister');
    end

    numWaypoints = double(opt.NumWaypoints);
    lowerBound = double(opt.LowerBound);
    upperBound = double(opt.UpperBound);
    startPoint = double(opt.StartPoint);
    maxStep = double(opt.MaxStep);
    angleLimit = double(opt.AngleLimit);
    gravity = 9.81;
    waypointTime = (0:numWaypoints-1)' * double(opt.WaypointInterval);
    if any(~isfinite(waypointTime)) || any(diff(waypointTime) <= 0)
        error('generate_random_trajectory:InvalidTime', ...
            'WaypointIntervalが大きすぎる、または小さすぎます。');
    end
    ppPosition = cell(1, 3);
    ppVelocity = cell(1, 3);
    ppAcceleration = cell(1, 3);
    positionMin = zeros(1, 3);
    positionMax = zeros(1, 3);
    accelerationMin = zeros(1, 3);
    accelerationMax = zeros(1, 3);
    rejected = struct('waypoint', 0, 'position', 0, 'attitude', 0, 'coverage', 0);
    accepted = false;

    for attempt = 1:opt.MaxAttempts
        waypoint = makeWaypoints(numWaypoints, startPoint, lowerBound, ...
            upperBound, maxStep, opt.Coverage);
        if opt.Wiggle > 0
            waypoint = addWiggles(waypoint, lowerBound, upperBound, maxStep, opt.Wiggle);
        end
        if any(any(waypoint < lowerBound | waypoint > upperBound)) || ...
                any(any(abs(diff(waypoint,1,1)) > maxStep))
            rejected.waypoint = rejected.waypoint + 1;
            continue
        end

        for axisIndex = 1:3
            ppPosition{axisIndex} = spline(waypointTime, waypoint(:,axisIndex));
            [positionMin(axisIndex), positionMax(axisIndex)] = ...
                polynomialRange(ppPosition{axisIndex});
        end
        % 通常のsplineはオーバーシュートするので、始点以外を再生成。
        if any(~isfinite([positionMin, positionMax])) || ...
                any(positionMin < lowerBound | positionMax > upperBound)
            rejected.position = rejected.position + 1;
            continue
        end
        % 補間後の連続時間の極値を使い、狭い範囲に偏る候補を棄却する。
        coverage = (positionMax-positionMin)./(upperBound-lowerBound);
        if any(coverage < opt.Coverage)
            rejected.coverage = rejected.coverage + 1;
            continue
        end

        for axisIndex = 1:3
            ppVelocity{axisIndex} = differentiatePP(ppPosition{axisIndex});
            ppAcceleration{axisIndex} = differentiatePP(ppVelocity{axisIndex});
            [accelerationMin(axisIndex), accelerationMax(axisIndex)] = ...
                polynomialRange(ppAcceleration{axisIndex});
        end
        % 三次スプラインの加速度は区間内で一次。極値は区間両端にある。
        % Gmin > 0なら全時刻で
        % |pitch| <= atan2(max|ax|,Gmin), |roll| <= atan2(max|ay|,Gmin)。
        % 実際のrollの分母はhypot(ax,g+az)なので、この判定は十分条件。
        accelerationAbsMax = max(abs(accelerationMin), abs(accelerationMax));
        minimumVerticalThrust = gravity + accelerationMin(3);
        angleUpperBound = atan2(accelerationAbsMax([2,1]), minimumVerticalThrust);
        if any(~isfinite([accelerationMin, accelerationMax, angleUpperBound])) || ...
                minimumVerticalThrust <= 0 || any(angleUpperBound > angleLimit)
            rejected.attitude = rejected.attitude + 1;
            continue
        end
        accepted = true;
        break
    end

    if ~accepted
        error('generate_random_trajectory:NoFeasibleTrajectory', ...
            ['%d回試行しましたが制約を満たす軌道が見つかりませんでした。' ...
             '姿勢の棄却が多ければWaypointIntervalを増やし、' ...
             '範囲利用率の棄却が多ければNumWaypointsを増やすかCoverageを下げてください。' ...
             '\n棄却数: waypoint=%d, position=%d, attitude=%d, coverage=%d'], ...
            opt.MaxAttempts, rejected.waypoint, rejected.position, rejected.attitude, rejected.coverage);
    end

    % 出力周期が総時間を割り切らない場合も、最後の時刻を必ず含める。
    finalTime = waypointTime(end);
    t = (0:double(opt.SampleTime):finalTime)';
    if t(end) < finalTime
        t(end+1,1) = finalTime;
    end
    position = zeros(numel(t), 3);
    velocity = zeros(numel(t), 3);
    acceleration = zeros(numel(t), 3);
    for axisIndex = 1:3
        position(:,axisIndex) = ppval(ppPosition{axisIndex}, t);
        velocity(:,axisIndex) = ppval(ppVelocity{axisIndex}, t);
        acceleration(:,axisIndex) = ppval(ppAcceleration{axisIndex}, t);
    end
    verticalThrust = gravity + acceleration(:,3);
    pitch = atan2(acceleration(:,1), verticalThrust);
    roll = atan2(-acceleration(:,2), hypot(acceleration(:,1), verticalThrust));
    yaw = zeros(size(t));

    traj.waypoint = waypoint;
    traj.waypointTime = waypointTime;
    traj.t = t;
    traj.position = position;
    traj.velocity = velocity;
    traj.acceleration = acceleration;
    traj.roll = roll;
    traj.pitch = pitch;
    traj.yaw = yaw;
    traj.reference = [t, position, roll, pitch, yaw];
    traj.ppPosition = ppPosition;
    traj.ppVelocity = ppVelocity;
    traj.ppAcceleration = ppAcceleration;
    traj.options = opt;
    traj.limits = struct('lower', lowerBound, 'upper', upperBound, ...
        'waypointStep', maxStep, 'angle', angleLimit, 'gravity', gravity);
    traj.validation.positionMin = positionMin;
    traj.validation.positionMax = positionMax;
    traj.validation.coverage = coverage; % [x y z] の利用幅/指定幅
    traj.validation.maxWaypointStep = max(abs(diff(waypoint,1,1)),[],1);
    traj.validation.angleUpperBound = angleUpperBound; % [roll pitch] [rad]
    traj.validation.minimumVerticalThrust = minimumVerticalThrust; % [m/s^2]
    traj.validation.attempts = attempt;
    traj.validation.rejected = rejected;

    if opt.Verbose
        fprintf('Accepted: attempt %d, %d waypoints, duration %.3f s\n', ...
            attempt, numWaypoints, finalTime);
        fprintf('Continuous min [x y z]: [% .6f % .6f % .6f] m\n', positionMin);
        fprintf('Continuous max [x y z]: [% .6f % .6f % .6f] m\n', positionMax);
        fprintf('Max waypoint step:     [%.6f %.6f %.6f] m\n', ...
            traj.validation.maxWaypointStep);
        fprintf('Guaranteed |roll|, |pitch| bounds: [%.6f %.6f] rad\n', angleUpperBound);
        fprintf('Range coverage [x y z]: [%.2f %.2f %.2f] %%\n',100*coverage);
        fprintf('Rejected: waypoint=%d, position=%d, attitude=%d, coverage=%d\n', ...
            rejected.waypoint, rejected.position, rejected.attitude, rejected.coverage);
    end
    if opt.Plot
        plotTrajectory(traj);
    end
end

function waypoint = makeWaypoints(count, startPoint, lower, upper, maxStep, coverage)
    waypoint = zeros(count,3);
    waypoint(1,:) = startPoint;
    if coverage == 0
        % 従来方式: 直前の点から動ける範囲で一様乱数を引く。
        for k = 2:count
            nextMin = max(lower,waypoint(k-1,:)-maxStep);
            nextMax = min(upper,waypoint(k-1,:)+maxStep);
            waypoint(k,:) = nextMin + rand(1,3).*(nextMax-nextMin);
        end
        return
    end

    % 各軸で上端・下端付近を交互に目指す。向き・目標位置・歩幅は乱数。
    % 境界そのものは狙わず、スプラインのオーバーシュート用の余裕を残す。
    % coverage=0.9なら目標点は各端から指定幅の2.5～5%内側。
    width = upper-lower;
    band = (1-coverage)/2;
    towardUpper = rand(1,3) >= 0.5;
    normalizedStart = (startPoint-lower)./width;
    towardUpper(normalizedStart <= 0.25) = true;
    towardUpper(normalizedStart >= 0.75) = false;
    inset = band*(0.5+0.5*rand(1,3)).*width;
    target = lower+inset;
    target(towardUpper) = upper(towardUpper)-inset(towardUpper);
    % xとyが同時に折り返して細い対角線状になることを避けるため、
    % 水平方向の片方を少しゆっくり進める。どちらを遅くするかも乱数。
    speedScale = [1, 0.6+0.15*rand, 0.65+0.35*rand];
    speedScale(1:2) = speedScale(randperm(2));
    for k = 2:count
        distance = target-waypoint(k-1,:);
        step = maxStep.*speedScale.*(0.5+0.5*rand(1,3));
        reachesTarget = abs(distance) <= step;
        waypoint(k,:) = waypoint(k-1,:) + sign(distance).*min(abs(distance),step);
        waypoint(k,reachesTarget) = target(reachesTarget);
        % 到着した座標だけ反対側へ。各軸の折り返し時刻も異なる。
        towardUpper(reachesTarget) = ~towardUpper(reachesTarget);
        inset = band*(0.5+0.5*rand(1,3)).*width;
        newTarget = lower+inset;
        newTarget(towardUpper) = upper(towardUpper)-inset(towardUpper);
        target(reachesTarget) = newTarget(reachesTarget);
    end
end

function waypoint = addWiggles(waypoint, lower, upper, maxStep, strength)
% 始点・終点と、各軸の最小/最大waypointを保持し、途中の点をランダムに動かす。
% 両隣との変化量制約を同時に満たす範囲で選ぶので、追加で点数を増やさない。
    count = size(waypoint,1);
    fixed = false(size(waypoint));
    fixed([1,end],:) = true;
    for axisIndex = 1:3
        [~,minimumIndex] = min(waypoint(:,axisIndex));
        [~,maximumIndex] = max(waypoint(:,axisIndex));
        fixed([minimumIndex,maximumIndex],axisIndex) = true;
    end
    original = waypoint;
    % ランダムな順序で2回動かし、単純な一方向の曲線に偏りにくくする。
    for sweep = 1:2
        for k = 1+randperm(count-2)
            allowedMin = max(lower, max(waypoint(k-1,:),waypoint(k+1,:))-maxStep);
            allowedMax = min(upper, min(waypoint(k-1,:),waypoint(k+1,:))+maxStep);
            allowedMin = max(allowedMin,original(k,:)-strength*maxStep);
            allowedMax = min(allowedMax,original(k,:)+strength*maxStep);
            candidate = allowedMin + rand(1,3).*(allowedMax-allowedMin);
            movable = ~fixed(k,:);
            waypoint(k,movable) = candidate(movable);
        end
    end
end

function derivative = differentiatePP(pp)
% 係数を直接微分するので、数値差分も追加Toolboxも不要。
    [breaks, coefficients, pieces, order] = unmkpp(pp);
    if order == 1
        derivative = mkpp(breaks, zeros(pieces,1));
    else
        derivative = mkpp(breaks, coefficients(:,1:end-1).*(order-1:-1:1));
    end
end

function [minimumValue, maximumValue] = polynomialRange(pp)
% 全区間の端点と導関数の実根から連続時間での最小値・最大値を求める。
    [breaks, coefficients, pieces] = unmkpp(pp);
    minimumValue = inf;
    maximumValue = -inf;
    for piece = 1:pieces
        intervalLength = breaks(piece+1) - breaks(piece);
        c = coefficients(piece,:);
        stationaryPoints = roots(polyder(c));
        isNearlyReal = abs(imag(stationaryPoints)) <= ...
            1e-10 * max(1, abs(real(stationaryPoints)));
        stationaryPoints = real(stationaryPoints(isNearlyReal));
        stationaryPoints = stationaryPoints( ...
            stationaryPoints > 0 & stationaryPoints < intervalLength);
        % ppの係数は区間始点からの相対時刻uの多項式。
        values = polyval(c, [0; intervalLength; stationaryPoints(:)]);
        if any(~isfinite(values))
            minimumValue = NaN;
            maximumValue = NaN;
            return
        end
        minimumValue = min(minimumValue, min(values));
        maximumValue = max(maximumValue, max(values));
    end
end

function plotTrajectory(traj)
    wp = traj.waypoint;
    % 粗いSampleTimeでも3次元図には滑らかな軌道を表示する。
    plotTime = linspace(traj.t(1), traj.t(end), 4001)';
    plotPosition = zeros(numel(plotTime),3);
    for axisIndex = 1:3
        plotPosition(:,axisIndex) = ppval(traj.ppPosition{axisIndex}, plotTime);
    end
    figure('Name','3D random reference trajectory','Color','w');
    plot3(plotPosition(:,1),plotPosition(:,2),plotPosition(:,3), ...
        'b-','LineWidth',1.8,'DisplayName','Cubic spline');
    hold on
    plot3(wp(:,1),wp(:,2),wp(:,3),'ro--','LineWidth',0.8, ...
        'MarkerFaceColor','r','DisplayName',sprintf('%d waypoints',size(wp,1)));
    plot3(wp(1,1),wp(1,2),wp(1,3),'gs','MarkerSize',11, ...
        'MarkerFaceColor','g','DisplayName','Start');
    plot3(wp(end,1),wp(end,2),wp(end,3),'kd','MarkerSize',11, ...
        'MarkerFaceColor','k','DisplayName','End');
    for k = 1:size(wp,1)
        text(wp(k,1),wp(k,2),wp(k,3),sprintf('  %d',k),'FontSize',8);
    end
    grid on
    box on
    axis equal
    xlim([traj.limits.lower(1),traj.limits.upper(1)]);
    ylim([traj.limits.lower(2),traj.limits.upper(2)]);
    zlim([traj.limits.lower(3),traj.limits.upper(3)]);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Constrained 3D trajectory (attempt %d)',traj.validation.attempts));
    legend('Location','best');
    view(40,25);

    figure('Name','Position and attitude constraints','Color','w');
    labels = {'x [m]','y [m]','z [m]'};
    for axisIndex = 1:3
        subplot(4,1,axisIndex);
        plot(plotTime,plotPosition(:,axisIndex),'b-','LineWidth',1.2);
        hold on
        plot(traj.waypointTime,wp(:,axisIndex),'ro','MarkerFaceColor','r');
        yline(traj.limits.lower(axisIndex),'r--');
        yline(traj.limits.upper(axisIndex),'r--');
        ylabel(labels{axisIndex}); grid on
        xlim([traj.t(1),traj.t(end)]);
    end
    subplot(4,1,4);
    plotAcceleration = zeros(numel(plotTime),3);
    for axisIndex = 1:3
        plotAcceleration(:,axisIndex) = ppval(traj.ppAcceleration{axisIndex},plotTime);
    end
    gz = traj.limits.gravity + plotAcceleration(:,3);
    plotRoll = atan2(-plotAcceleration(:,2),hypot(plotAcceleration(:,1),gz));
    plotPitch = atan2(plotAcceleration(:,1),gz);
    plot(plotTime,[plotRoll,plotPitch],'LineWidth',1.2);
    hold on
    yline(traj.limits.angle,'r--','HandleVisibility','off');
    yline(-traj.limits.angle,'r--','HandleVisibility','off');
    ylabel('Angle [rad]'); xlabel('Time [s]'); grid on
    xlim([traj.t(1),traj.t(end)]); ylim(1.1*[-traj.limits.angle,traj.limits.angle]);
    legend('roll','pitch','Location','best');
end
