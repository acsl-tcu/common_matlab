function fig=plot_obstacles_4views(world,varargin)
% 静止画を一枚のFigureに4面表示：3D / 上面XY / 正面XZ / 側面YZ。
% 例: plot_obstacles_4views(world,'time',0,'filename','obstacles.png');
opts=struct(varargin{:});
if ~isfield(opts,'time'),opts.time=0;end
if ~isfield(opts,'filename'),opts.filename='';end
fig=figure('Name',sprintf('Obstacles: %s  t=%.2fs',world.scene,opts.time));
set(fig,'Position',[100 100 1180 860]);
drawers=cell(1,4);
views={[45 25],[0 90],[0 0],[90 0]};
names={'3D','XY（上面）','XZ（正面）','YZ（側面）'};
for i=1:4
 ax=subplot(2,2,i,'Parent',fig);
 drawers{i}=DRAW_DRONE_OBSTACLES(world,'ax',ax,'fitToObstacles',true);
 drawers{i}.update(opts.time);
 view(ax,views{i}(1),views{i}(2));
 title(ax,names{i});
end
setappdata(fig,'ObstacleDrawers',drawers);
if ~isempty(opts.filename)
 [folder,~,~]=fileparts(char(opts.filename));
 if ~isempty(folder) && ~exist(folder,'dir'),mkdir(folder);end
 try
  exportgraphics(fig,char(opts.filename),'Resolution',200);
 catch
  saveas(fig,char(opts.filename));
 end
 fprintf('[plot_obstacles_4views] saved: %s\n',char(opts.filename));
end
end
