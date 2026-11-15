function fig=wp_plot_preview_3d(world,preview)
% Visual diagnostic only, no PNG saved automatically.
% Requires existing plot/DRAW_DRONE_OBSTACLES on MATLAB path.
fig=figure('Name',['WP 4s preview (provisional) - ' preview.method]);
ax=axes('Parent',fig);hold(ax,'on');
drawer=DRAW_DRONE_OBSTACLES(world,'ax',ax,'fitToObstacles',false, ...
    'showOriginal',true,'showApprox',true,'originalAlpha',0.45,'approxAlpha',0.10);
drawer.update(preview.times(1));
valid=preview.validSamples;
pD=preview.pDrone(:,valid);pL=preview.pLoad(:,valid);
plot3(ax,pD(1,:),pD(2,:),pD(3,:),'b-','LineWidth',2,'DisplayName','nominal drone');
plot3(ax,pL(1,:),pL(2,:),pL(3,:),'Color',[.1 .5 .25],'LineWidth',2,'DisplayName','nominal load');
if ~isempty(pD)
    for k=1:max(1,floor(size(pD,2)/8)):size(pD,2)
        plot3(ax,[pD(1,k) pL(1,k)],[pD(2,k) pL(2,k)], ...
            [pD(3,k) pL(3,k)],'Color',[.35 .45 .55]);
    end
end
j=find(preview.collisionSamples & valid,1,'first');
if ~isempty(j)
    plot3(ax,preview.pLoad(1,j),preview.pLoad(2,j),preview.pLoad(3,j), ...
        'rx','MarkerSize',14,'LineWidth',3,'DisplayName','sampled collision');
end
if ~isempty(pD)
    allp=[pD pL];lo=min(allp,[],2);hi=max(allp,[],2);
    sp=max(1,hi-lo);ax.XLim=[lo(1)-.2*sp(1) hi(1)+.2*sp(1)];
    ax.YLim=[lo(2)-.2*sp(2) hi(2)+.2*sp(2)];
    ax.ZLim=[min(0,lo(3)-1) hi(3)+1];
end
axis(ax,'equal');grid(ax,'on');view(ax,3);
xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
title(ax,sprintf('%s / %s / 4 s sampled preview',preview.method,preview.status), ...
    'Interpreter','none');
setappdata(fig,'ObstacleDrawer',drawer);
end
