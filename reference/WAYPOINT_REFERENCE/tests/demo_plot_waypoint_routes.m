function demo_plot_waypoint_routes()
%DEMO_PLOT_WAYPOINT_ROUTES  Visual overview of the geometric paths.
% Also check generated point samples and plotted reference positions.
refs = {gen_wp_straight(), gen_wp_gentle_turn(), ...
    gen_wp_circle(), gen_wp_saddle(), gen_wp_custom_formula()};
figure('Name','Waypoint routes (HLC reference)');
for j = 1:numel(refs)
    ref = refs{j};
    subplot(2,3,j); hold on;
    if ref.isLoop
        tmax = 6 + ref.pathLength / ref.speedPeak * 1.03;
    else
        tmax = ref.duration;
    end
    tt = linspace(0,tmax,801);
    trace = zeros(3,numel(tt));
    for i=1:numel(tt)
        xd=ref.evaluate(tt(i));
        trace(:,i)=xd(1:3);
    end
    plot3(trace(1,:), trace(2,:), trace(3,:), 'LineWidth',1.5);
    w = ref.pointsXYZ;
    plot3(w(:,1),w(:,2),w(:,3),'o','MarkerSize',3);
    grid on; axis equal; xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
    title(sprintf('%s (loop=%d)',ref.routeName,ref.isLoop),'Interpreter','none');
end
end
