function OUT=AF7R30_OPEN_3D_WINDOW()
% AF7R30_OPEN_3D_WINDOW
% Create one additional large 3-D window without depending on figure number or
% title. The source axes are selected primarily from camera View state, then
% from ZData / z-label evidence.
%
% Existing runtime and trajectory figures are untouched.

drawnow;

OUT=struct('schema','AF7R30_DEDICATED_3D', ...
    'created',false,'figure',NaN,'source_title','', ...
    'xlim',[NaN NaN],'ylim',[NaN NaN],'zlim',[NaN NaN], ...
    'minimum_numeric_span_m',3.0,'plot_box_aspect',[1 1 1], ...
    'message','');

axs=findall(groot,'Type','axes');
best=[];
bestScore=-inf;
bestTitle='';

for ai=1:numel(axs)
    ax0=axs(ai);
    try
        tg=char(get(ax0,'Tag'));
        if strcmpi(tg,'legend')||strcmpi(tg,'Colorbar'),continue;end
    catch
    end

    score=0;

    % Strongest cue: actual non-top-down 3-D camera.
    try
        vw=double(get(ax0,'View'));
        if numel(vw)==2 && all(isfinite(vw)) && abs(vw(2)-90)>1e-9
            score=score+1e9;
        end
    catch
    end

    % Secondary cue: objects carrying finite ZData.
    try
        kids=allchild(ax0);
        for k=1:numel(kids)
            if isprop(kids(k),'ZData')
                z=get(kids(k),'ZData');
                if isnumeric(z)&&~isempty(z)
                    score=score+nnz(isfinite(double(z(:))));
                end
            end
        end
    catch
    end

    % Spatial-axis cues.
    try
        ztxt=char(get(get(ax0,'ZLabel'),'String'));
        if ~isempty(strtrim(ztxt)),score=score+1e6;end
        zl=double(zlim(ax0));
        if numel(zl)==2&&all(isfinite(zl))&&(zl(2)-zl(1))>0
            score=score+1e3;
        end
    catch
    end

    if score>bestScore
        best=ax0;bestScore=score;
        try
            bestTitle=char(get(get(ax0,'Title'),'String'));
        catch
            bestTitle='';
        end
    end
end

if isempty(best) || bestScore<1e6
    OUT.message='No usable 3-D source axes exist yet. Run this helper after the simulation has drawn the 3-D flight plot.';
    warning('[R30 VIEW] %s',OUT.message);
    return;
end

fig=findobj(groot,'Type','figure','Tag','AF7R30_DEDICATED_3D');
if isempty(fig)
    fig=figure('Name','Flight 3-D | Dedicated large view', ...
        'NumberTitle','off','Tag','AF7R30_DEDICATED_3D');
else
    fig=fig(1);figure(fig);clf(fig);
end
set(fig,'Units','normalized','Position',[0.04 0.05 0.88 0.88]);

ax=axes('Parent',fig);
copyobj(allchild(best),ax);

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

% Preserve truthful axis values but allocate a non-crushed screen box.
pbaspect(ax,[1 1 1]);
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
