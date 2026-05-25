function plot_calc_time(logger, LW, FS, set_dt)
%check_calc_time 計算時間を確認・プロットする関数
%   logger: LOGGERクラス
%   LW: LineWidth
%   FS: FontSize
%   dt: 制御周期
arguments
    logger
    LW = 1.0;
    FS = 16;
    set_dt = 0.025;
end

set_dt_ms = set_dt*10^3;
t0id = find(logger.Data.phase==97,1,'last')+1;
teid = find(logger.Data.phase==0,1,'first')-1;
dt = diff(logger.Data.t(t0id:teid));
t = logger.Data.t(t0id:teid-1);
figure(112)
ax = gca;
plot(t,dt*10^3, Linewidth=LW);
hold on
yline(set_dt_ms,"LineWidth",LW) % 制御周期プロット
hold off
grid on; grid minor;

% leg_txt = "Control period setting" + " ("+string(set_dt_ms)+" ms)";
leg_txt = "Control period setting";
legend("Actual computation time", leg_txt, "Location","southeast")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
xlabel("Time [s]", "FontSize",FS);
ylabel("Computation time [ms]", "FontSize",FS)
end
