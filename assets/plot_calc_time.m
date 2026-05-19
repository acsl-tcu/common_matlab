function plot_calc_time(logger, LW, FS)
%check_calc_time 計算時間を確認・プロットする関数
%   logger: LOGGERクラス
%   LW: LineWidth
%   FS: FontSize
arguments
    logger
    LW = 1.0;
    FS = 18;
end

t0id = find(logger.Data.phase==97,1,'last')+1;
teid = find(logger.Data.phase==0,1,'first')-1;
dt = diff(logger.Data.t(t0id:teid));
t = logger.Data.t(t0id:teid-1);
figure(112)
ax = gca;
[t,dt];
plot(t,dt, Linewidth=LW);
hold on
yline(0.025,"LineWidth",LW)
hold off
grid on
legend("dt","25 ms")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
end

