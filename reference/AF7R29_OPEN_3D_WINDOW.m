function OUT=AF7R29_OPEN_3D_WINDOW()
% AF7R29_OPEN_3D_WINDOW
% Add one large third window dedicated to 3-D geometry.
%
% Source selection is CONTENT based, not figure-number/title based:
% choose the axes containing the largest amount of finite ZData.
% This avoids R28's brittle title lookup.
%
% If no 3-D source exists yet, do NOT abort the audit.

OUT=struct('schema','AF7R29_DEDICATED_3D', ...
    'created',false,'figure',NaN,'source_title','', ...
    'xlim',[NaN NaN],'ylim',[NaN NaN],'zlim',[NaN NaN], ...
    'minimum_numeric_span_m',3.0,'plot_box_aspect',[1 1 1], ...
    'message','');

axs=findall(groot,'Type','axes');
best=[];
bestScore=-1;
bestTitle='';
for ai=1:numel(axs)
    ax0=axs(ai);
    % Ignore legends/colorbars masquerading as axes.
    try
        tg=char(get(ax0,'Tag'));
        if strcmpi(tg,'legend')||strcmpi(tg,'Colorbar'),continue;end
    catch
    end

    objs=findall(ax0,'-property','ZData');
    score=0;
    for k=1:numel(objs)
        try
            z=get(objs(k),'ZData');
            if isnumeric(z)&&~isempty(z)
                zz=double(z(:));
                score=score+nnz(isfinite(zz));
            end
        catch
        end
    end
    if score>bestScore && score>0
        best=ax0;bestScore=score;
        try
            bestTitle=char(get(get(ax0,'Title'),'String'));
        catch
            bestTitle='';
        end
    end
end

if isempty(best)
    OUT.message='No axes with finite ZData exist yet. Run this helper after the simulation plots are drawn.';
    warning('[R29 VIEW] %s',OUT.message);
    return;
end

fig=findobj(groot,'Type','figure','Tag','AF7R29_DEDICATED_3D');
if isempty(fig)
    fig=figure('Name','Flight 3-D | Dedicated large view', ...
        'NumberTitle','off','Tag','AF7R29_DEDICATED_3D');
else
    fig=fig(1);figure(fig);clf(fig);
end
set(fig,'Units','normalized','Position',[0.06 0.06 0.84 0.86]);

ax=axes('Parent',fig);
copyobj(allchild(best),ax);

% Copy labels/title when useful, but use a stable dedicated title.
try, xlabel(ax,char(get(get(best,'XLabel'),'String'))); catch, xlabel(ax,'x [m]'); end
try, ylabel(ax,char(get(get(best,'YLabel'),'String'))); catch, ylabel(ax,'y [m]'); end
try, zlabel(ax,char(get(get(best,'ZLabel'),'String'))); catch, zlabel(ax,'z [m]'); end
title(ax,'Flight 3-D | dedicated large view');

view(ax,3);
grid(ax,'on');box(ax,'on');

mins=3.0;
xl=expand_lim(double(xlim(best)),mins);
yl=expand_lim(double(ylim(best)),mins);
zl=expand_lim(double(zlim(best)),mins);
xlim(ax,xl);ylim(ax,yl);zlim(ax,zl);

% Keep truthful numerical coordinates/ticks, but use a cubic screen box so
% large z travel cannot visually crush x/y motion.
pbaspect(ax,[1 1 1]);

try
    lg=findobj(ancestor(best,'figure'),'Type','legend');
    if ~isempty(lg)
        legend(ax,'show','Location','best');
    end
catch
end
rotate3d(fig,'on');
drawnow;

OUT.created=true;
OUT.figure=double(fig.Number);
OUT.source_title=bestTitle;
OUT.xlim=xl;OUT.ylim=yl;OUT.zlim=zl;
OUT.message='Dedicated large 3-D window created.';
end

function lim=expand_lim(lim,mins)
if numel(lim)~=2||any(~isfinite(lim)),return;end
sp=lim(2)-lim(1);
if sp<mins
    c=0.5*(lim(1)+lim(2));
    lim=[c-0.5*mins,c+0.5*mins];
end
end
