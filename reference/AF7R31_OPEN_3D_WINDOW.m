function OUT=AF7R31_OPEN_3D_WINDOW()
% AF7R31_OPEN_3D_WINDOW
% Open ONE additional large 3-D window.
% Works with both ordinary Axes and UIAxes. Existing runtime/trajectory windows
% are left untouched.

drawnow;

OUT=struct('schema','AF7R31_DEDICATED_3D', ...
    'created',false,'figure',NaN,'source_class','', ...
    'source_title','','copied_children',0, ...
    'xlim',[NaN NaN],'ylim',[NaN NaN],'zlim',[NaN NaN], ...
    'message','');

H=findall(groot);
src=[];
best=-inf;
bestTitle='';
bestClass='';

for k=1:numel(H)
    h=H(k);
    isax=false;
    try
        isax=isa(h,'matlab.graphics.axis.Axes') || ...
             isa(h,'matlab.ui.control.UIAxes');
    catch
        isax=false;
    end
    if ~isax, continue; end

    score=0;
    ttl='';
    try
        ttl=char(h.Title.String);
    catch
    end
    if contains(lower(string(ttl)),'flight 3-d')
        score=score+1e12;
    end

    try
        ztxt=char(h.ZLabel.String);
        if ~isempty(strtrim(ztxt)), score=score+1e10; end
    catch
    end

    try
        vw=double(h.View);
        if numel(vw)==2 && all(isfinite(vw)) && abs(vw(2)-90)>1e-9
            score=score+1e9;
        end
    catch
    end

    try
        kids=allchild(h);
        for j=1:numel(kids)
            if isprop(kids(j),'ZData')
                z=kids(j).ZData;
                if isnumeric(z)&&~isempty(z)
                    score=score+nnz(isfinite(double(z(:))));
                end
            end
        end
    catch
    end

    if score>best
        best=score;
        src=h;
        bestTitle=ttl;
        bestClass=class(h);
    end
end

if isempty(src) || best<1e9
    OUT.message='No 3-D Axes/UIAxes source was found after plotting.';
    warning('[R31 VIEW] %s',OUT.message);
    return;
end

% Reuse/replace the dedicated window only.
old=findall(groot,'Tag','AF7R31_DEDICATED_3D');
for k=1:numel(old)
    try, delete(old(k)); catch, end
end

isUI=false;
try
    isUI=isa(src,'matlab.ui.control.UIAxes');
catch
end

if isUI
    fig=uifigure('Name','Flight 3-D | Dedicated large view', ...
        'Tag','AF7R31_DEDICATED_3D', ...
        'Position',[80 60 1350 850]);
    ax=uiaxes(fig,'Position',[70 70 1210 720]);
else
    fig=figure('Name','Flight 3-D | Dedicated large view', ...
        'NumberTitle','off','Tag','AF7R31_DEDICATED_3D', ...
        'Units','normalized','Position',[0.04 0.05 0.88 0.88]);
    ax=axes('Parent',fig,'Position',[0.09 0.10 0.84 0.83]);
end

copied=0;
kids=allchild(src);
for j=numel(kids):-1:1
    try
        copyobj(kids(j),ax);
        copied=copied+1;
    catch
        % Fallback for line-like objects if direct copying is unsupported
        % between UIAxes and ordinary axes.
        try
            if isprop(kids(j),'XData') && isprop(kids(j),'YData') && ...
                    isprop(kids(j),'ZData')
                x=double(kids(j).XData);
                y=double(kids(j).YData);
                z=double(kids(j).ZData);
                if ~isempty(x)&&~isempty(y)&&~isempty(z)
                    plot3(ax,x,y,z,'DisplayName',char(kids(j).DisplayName));
                    hold(ax,'on');
                    copied=copied+1;
                end
            end
        catch
        end
    end
end

if copied==0
    try, delete(fig); catch, end
    OUT.message='3-D source axes was found, but no drawable child could be copied.';
    warning('[R31 VIEW] %s',OUT.message);
    return;
end

try, xlabel(ax,char(src.XLabel.String)); catch, xlabel(ax,'x [m]'); end
try, ylabel(ax,char(src.YLabel.String)); catch, ylabel(ax,'y [m]'); end
try, zlabel(ax,char(src.ZLabel.String)); catch, zlabel(ax,'z [m]'); end
title(ax,'Flight 3-D | nominal / repaired / vehicle / payload');
grid(ax,'on'); box(ax,'on'); view(ax,3);

mins=3.0;
xl=expand_lim(double(src.XLim),mins);
yl=expand_lim(double(src.YLim),mins);
zl=expand_lim(double(src.ZLim),mins);
xlim(ax,xl); ylim(ax,yl); zlim(ax,zl);

% Keep the numerical coordinates truthful but give each screen direction room.
pbaspect(ax,[1 1 1]);

try, legend(ax,'show','Location','best'); catch, end
try, rotate3d(fig,'on'); catch, end
drawnow;

OUT.created=true;
try, OUT.figure=double(fig.Number); catch, OUT.figure=NaN; end
OUT.source_class=bestClass;
OUT.source_title=bestTitle;
OUT.copied_children=copied;
OUT.xlim=xl;OUT.ylim=yl;OUT.zlim=zl;
OUT.message='Dedicated large 3-D window created.';
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
