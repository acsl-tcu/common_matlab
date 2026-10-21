function OUT=AF7R28_OPEN_3D_WINDOW()
% AF7R28_OPEN_3D_WINDOW
% Create ONE additional, large, dedicated 3-D window.
% Existing runtime and trajectory windows are left untouched.
%
% Data coordinates are not altered.  x/y/z each keep at least 3 m numeric
% span, while PlotBoxAspectRatio is decoupled from data aspect so a 50 m
% vertical trajectory does not collapse x/y to a thin line on screen.

src=[];
axs=findall(groot,'Type','axes');
for k=1:numel(axs)
    try
        tt=string(get(get(axs(k),'Title'),'String'));
        if contains(lower(tt),'flight 3-d')
            src=axs(k);break;
        end
    catch
    end
end

if isempty(src)
    h6=findobj(groot,'Type','figure','Number',6);
    if ~isempty(h6)
        aa=findall(h6(1),'Type','axes');
        if ~isempty(aa),src=aa(1);end
    end
end
assert(~isempty(src),'[R28 VIEW] Source Flight 3-D axes not found.');

fig=findobj(groot,'Type','figure','Tag','AF7R28_DEDICATED_3D');
if isempty(fig)
    fig=figure('Name','Flight 3-D | Dedicated', ...
        'NumberTitle','off','Tag','AF7R28_DEDICATED_3D');
else
    figure(fig(1));clf(fig(1));fig=fig(1);
end
set(fig,'Units','normalized','Position',[0.08 0.08 0.78 0.82]);

ax=axes('Parent',fig);
copyobj(allchild(src),ax);

xlabel(ax,'x [m]');
ylabel(ax,'y [m]');
zlabel(ax,'z [m]');
title(ax,'Flight 3-D | nominal / repaired / vehicle / payload');
view(ax,3);
grid(ax,'on');box(ax,'on');

mins=3.0;
xl=expand_lim(double(xlim(src)),mins);
yl=expand_lim(double(ylim(src)),mins);
zl=expand_lim(double(zlim(src)),mins);
xlim(ax,xl);ylim(ax,yl);zlim(ax,zl);

% Critical display fix: DO NOT use daspect([1 1 1]) here.
% A 50 m z-span versus 3-8 m x/y would visually crush the lateral geometry.
% Keep truthful tick values/ranges but allocate equal screen box dimensions.
pbaspect(ax,[1 1 1]);

try
    legend(ax,'show','Location','best');
catch
end
rotate3d(fig,'on');
drawnow;

OUT=struct('schema','AF7R28_DEDICATED_3D', ...
    'figure',double(fig.Number),'xlim',xl,'ylim',yl,'zlim',zl, ...
    'minimum_numeric_span_m',mins, ...
    'plot_box_aspect',[1 1 1]);
end

function lim=expand_lim(lim,mins)
if numel(lim)~=2||any(~isfinite(lim)),return;end
sp=lim(2)-lim(1);
if sp<mins
    c=0.5*(lim(1)+lim(2));
    lim=[c-0.5*mins,c+0.5*mins];
end
end
