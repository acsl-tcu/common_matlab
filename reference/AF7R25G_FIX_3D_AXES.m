function OUT=AF7R25G_FIX_3D_AXES(min_span_m)
% AF7R25G_FIX_3D_AXES
% Prevent flat/planar 3-D trajectory plots from collapsing an axis to an
% unusably thin range. Applies only to axes that actually contain ZData.
%
% Default minimum full span is 3 m independently for x, y and z.
if nargin<1 || isempty(min_span_m), min_span_m=3.0; end
validateattributes(min_span_m,{'numeric'},{'real','finite','scalar','positive'});

figs=findall(groot,'Type','figure');
changed=0;
for fi=1:numel(figs)
    axs=findall(figs(fi),'Type','axes');
    for ai=1:numel(axs)
        ax=axs(ai);
        if ~af7r25g_is_spatial_axes(ax), continue; end
        xl=double(xlim(ax)); yl=double(ylim(ax)); zl=double(zlim(ax));
        [xl,cx]=af7r25g_expand_lim(xl,min_span_m);
        [yl,cy]=af7r25g_expand_lim(yl,min_span_m);
        [zl,cz]=af7r25g_expand_lim(zl,min_span_m);
        if cx, xlim(ax,xl); end
        if cy, ylim(ax,yl); end
        if cz, zlim(ax,zl); end
        if cx||cy||cz
            changed=changed+1;
        end
        view(ax,3);
        grid(ax,'on');
        box(ax,'on');
    end
end
drawnow;
OUT=struct('schema','AF7R25G_MINIMUM_3D_AXIS_SPAN', ...
    'minimum_span_m',double(min_span_m),'axes_changed',double(changed));
end

function tf=af7r25g_is_spatial_axes(ax)
tf=false;
try
    objs=findall(ax,'-property','ZData');
    for k=1:numel(objs)
        z=get(objs(k),'ZData');
        if isnumeric(z) && ~isempty(z) && any(isfinite(double(z(:))))
            tf=true; return;
        end
    end
catch
    tf=false;
end
end

function [lim,changed]=af7r25g_expand_lim(lim,minspan)
changed=false;
if numel(lim)~=2 || any(~isfinite(lim))
    return;
end
span=lim(2)-lim(1);
if span>=minspan
    return;
end
c=0.5*(lim(1)+lim(2));
lim=[c-0.5*minspan,c+0.5*minspan];
changed=true;
end
