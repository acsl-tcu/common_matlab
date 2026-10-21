function OUT=AF7R27_3D_VIEW()
% Single dedicated 3-D view. Does not create four simultaneous 3-D windows.
% Uses the already drawn flight trajectory when available.
h6=findobj(groot,'Type','figure','Number',6);
assert(~isempty(h6),'[R27 VIEW] Figure 6 is unavailable.');
a6=findobj(h6,'Type','axes');
assert(~isempty(a6),'[R27 VIEW] Figure 6 has no axes.');
figure(12);clf;
a12=axes('Parent',gcf);
copyobj(allchild(a6(1)),a12);
xlabel(a12,'x [m]');ylabel(a12,'y [m]');zlabel(a12,'z [m]');
title(a12,'Dedicated 3-D avoidance view');
view(a12,3);grid(a12,'on');box(a12,'on');

% Numerical axes keep at least 3 m span independently, while plot-box aspect
% remains usable even for nearly planar trajectories.
mins=3.0;
xl=expand_lim(xlim(a12),mins);
yl=expand_lim(ylim(a12),mins);
zl=expand_lim(zlim(a12),mins);
xlim(a12,xl);ylim(a12,yl);zlim(a12,zl);
pbaspect(a12,[1 1 1]);
drawnow;

OUT=struct('schema','AF7R27_DEDICATED_3D_VIEW', ...
    'xlim',xl,'ylim',yl,'zlim',zl,'minimum_axis_span_m',mins);
end

function lim=expand_lim(lim,mins)
lim=double(lim);
if numel(lim)~=2||any(~isfinite(lim)),return;end
sp=lim(2)-lim(1);
if sp<mins
    c=0.5*(lim(1)+lim(2));
    lim=[c-0.5*mins,c+0.5*mins];
end
end
