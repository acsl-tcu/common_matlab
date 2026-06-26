function plot_calc_time(logger, set_dt, LW, FS)
%check_calc_time 計算時間を確認・プロットする関数
%   logger: LOGGERクラス
%   LW: LineWidth
%   FS: FontSize
%   dt: 制御周期
arguments
    logger
    set_dt = 0.025;
    LW = 1.0;
    FS = 16;
end

set_dt_ms = set_dt*10^3;
t=logger.data('t',"","");
dt = diff(t);
dt = dt.*10^3;

t_flight = logger.Data.t(find(logger.Data.phase=='t',1,'last'));
t_land   = logger.Data.t(find(logger.Data.phase=='f',1,'last'));
figure(112)
ax = gca;
plot(t(1:end-1),dt, Linewidth=LW);
hold on
yline(set_dt_ms,"LineWidth",LW) % 制御周期プロット
xline(t_flight, '--'); xline(t_land, '--');
hold off
grid on; grid minor;

% leg_txt = "Control period setting" + " ("+string(set_dt_ms)+" ms)";
leg_txt = "Control period setting";
legend("Actual computation time", leg_txt, "Location","best")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
xlabel("Time [s]", "FontSize",FS);
ylabel("Computation time [ms]", "FontSize",FS)
end
