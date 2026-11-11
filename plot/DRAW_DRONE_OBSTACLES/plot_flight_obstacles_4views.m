function fig=plot_flight_obstacles_4views(logger,world,varargin)
% PLOT_FLIGHT_OBSTACLES_4VIEWS 飛行ログ・吊り荷軌跡と障害物を4面表示。
% fig=plot_flight_obstacles_4views(logger,world,'time',12, ...
%     'filename','Data/FlightViews.png');
% DRAW_SUSPENDED_LOAD.m は一切呼び出さず、変更もしません。
opts=struct(varargin{:});
if ~isfield(opts,'target'),opts.target=1;end
if ~isfield(opts,'time'),opts.time=0;end
if ~isfield(opts,'filename'),opts.filename='';end
if ~isfield(opts,'showApprox'),opts.showApprox=true;end
if ~isfield(opts,'showOriginal'),opts.showOriginal=true;end
if ~isfield(opts,'showGround'),opts.showGround=true;end
p=read_xyz(logger,opts.target,'p','p');
pL=[];
for name={'estimator.result.state.pL','pL'}
    try
        if strcmp(name{1},'pL'),attribute='p';else,attribute='e';end
        pL=read_xyz(logger,opts.target,name{1},attribute);
        break;
    catch
    end
end
t=logger.data(0,'t',''); t=t(:);
if isempty(t),k=1;else,[~,k]=min(abs(t-opts.time));end
k=max(1,min(k,size(p,1)));
fig=figure('Name',sprintf('Flight and obstacles at t=%.2fs',opts.time), ...
    'Position',[100 100 1200 870]);
views={[45 25],[0 90],[0 0],[90 0]};
names={'3D','XY (top)','XZ (front)','YZ (side)'};
drawers=cell(1,4);
for ii=1:4
    ax=subplot(2,2,ii,'Parent',fig);hold(ax,'on');
    drawers{ii}=DRAW_DRONE_OBSTACLES(world,'ax',ax, ...
        'showApprox',opts.showApprox,'showOriginal',opts.showOriginal, ...
        'showGround',opts.showGround);
    drawers{ii}.update(opts.time);
    plot3(ax,p(:,1),p(:,2),p(:,3),'Color',[.83 .25 .18], ...
        'LineWidth',1.3,'DisplayName','Drone');
    plot3(ax,p(k,1),p(k,2),p(k,3),'o','MarkerFaceColor',[.83 .25 .18], ...
        'MarkerEdgeColor','k','MarkerSize',5);
    if ~isempty(pL)
        plot3(ax,pL(:,1),pL(:,2),pL(:,3),'Color',[.20 .48 .30], ...
            'LineWidth',1.3,'DisplayName','Suspended load');
        kp=min(k,size(pL,1));
        plot3(ax,pL(kp,1),pL(kp,2),pL(kp,3),'o', ...
            'MarkerFaceColor',[.20 .48 .30],'MarkerEdgeColor','k','MarkerSize',5);
    end
    % Keep the entire recorded path visible alongside the obstacles.
    allPath=p;
    if ~isempty(pL),allPath=[allPath;pL];end
    boundsMin=min(allPath,[],1);boundsMax=max(allPath,[],1);
    xl=ax.XLim;yl=ax.YLim;zl=ax.ZLim;
    margin=.4;
    set(ax,'XLim',[min(xl(1),boundsMin(1)-margin),max(xl(2),boundsMax(1)+margin)], ...
        'YLim',[min(yl(1),boundsMin(2)-margin),max(yl(2),boundsMax(2)+margin)], ...
        'ZLim',[min(zl(1),boundsMin(3)-margin),max(zl(2),boundsMax(3)+margin)]);
    view(ax,views{ii}(1),views{ii}(2));
    title(ax,names{ii});
    grid(ax,'on');axis(ax,'equal');
end
setappdata(fig,'ObstacleDrawers',drawers);
if ~isempty(opts.filename)
    pathName=char(opts.filename);
    [folder,~,~]=fileparts(pathName);
    if ~isempty(folder) && ~exist(folder,'dir'),mkdir(folder);end
    try
        exportgraphics(fig,pathName,'Resolution',200);
    catch
        saveas(fig,pathName);
    end
    fprintf('[plot_flight_obstacles_4views] saved: %s\n',pathName);
end
end

function X=read_xyz(logger,target,name,attr)
q=logger.data(target,name,attr);
X=reshape(q,size(q,1),size(q,2)/numel(target),numel(target));
X=X(:,:,1);
assert(size(X,2)==3,'Expected Nx3 log for %s',name);
end
