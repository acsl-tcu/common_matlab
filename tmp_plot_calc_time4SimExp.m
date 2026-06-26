time = app.tmp;
idx = find(all(time == 0, 2), 1, 'first');
time = time(1:idx-1,:);

total_dt = time(:,1).*10^3;
dt= time(:,2:4).*10^3;
do_dt = time(:,5:11).*10^3;
t = time(:,end);
FS = 16;
t_flight = app.logger.Data.t(find(app.logger.Data.phase=='t',1,'last'));
t_land   = app.logger.Data.t(find(app.logger.Data.phase=='f',1,'last'));

figure(1);
ax=gca;
plot(t,total_dt, t,dt, "LineWidth",1)
hold on;
xline(t_flight, "--"); xline(t_land, "--")
xlim([0 max(t)])
ylim([0 25])
grid on; grid minor;

legend("Total", "drawnow", "motive.getData", "do calculation", "Location","best");
xlabel("Time [s]")
ylabel("Calculation time [ms]")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
hold off;

% ==============================
figure(2)
ax=gca;
plot(t,total_dt, t,do_dt, "LineWidth",1)
hold on;
xline(t_flight, "--"); xline(t_land, "--")
xlim([0 max(t)])
ylim([0 25])
grid on; grid minor;

legend("Total", "sensor", "estimator", "reference", "controller", "input tranform", "plant", "logging", "Location","best");
xlabel("Time [s]")
ylabel("Calculation time [ms]")
set(ax.XAxis, fontsize=FS-2)
set(ax.YAxis, fontsize=FS-2)
set(ax.Legend, 'FontSize',FS-4);
hold off;