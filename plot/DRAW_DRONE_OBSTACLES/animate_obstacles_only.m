function animate_obstacles_only(world,timeValues,filename)
% 動的障害物の単体アニメーション確認。保存はfilename指定時のみ。
% 例: animate_obstacles_only(world,0:0.1:12,'moving_obstacles.mp4');
if nargin<2 || isempty(timeValues),timeValues=0:0.1:10;end
if nargin<3,filename='';end
fig=figure('Name','DRONE_OBSTACLES animation','Position',[100 100 1100 700]);
ax=axes('Parent',fig);
drawer=DRAW_DRONE_OBSTACLES(world,'ax',ax);
writer=[];
if ~isempty(filename)
 [folder,~,~]=fileparts(char(filename));
 if ~isempty(folder) && ~exist(folder,'dir'),mkdir(folder);end
 writer=VideoWriter(char(filename),'MPEG-4');
 if numel(timeValues)>1
  writer.FrameRate=1/median(diff(timeValues));
 end
 open(writer);
end
try
 for t=timeValues
  drawer.update(t);
  title(ax,sprintf('Obstacle scene: %s  t=%.2f [s]',world.scene,t));
  drawnow;
  if ~isempty(writer),writeVideo(writer,getframe(fig));end
 end
catch ME
 if ~isempty(writer),close(writer);end
 rethrow(ME);
end
if ~isempty(writer),close(writer);end
end
