classdef DRAW_DRONE_OBSTACLES < handle
% DRAW_DRONE_OBSTACLES 障害物元形状と外側近似を同じ座標軸に描画。
% original : 濃い不透明メッシュ / approximation : 薄い半透明メッシュ。
% HGTRANSFORM を用い動的障害物を再メッシュ化せず動かす。
%
% ex:
%  d = DRAW_DRONE_OBSTACLES(world,'ax',gca);
%  d.update(2.0);
%  plot_obstacles_4views(world,'time',2.0);
properties
 world
 ax
 originalGroups
 approxGroups
 groundSurface=[]
 labels
 labelOffsets
 showLabels=true
 showOriginal=true
 showApprox=true
 showGround=true
 originalAlpha=0.68
 approxAlpha=0.12
 originalColor=[0.46 0.51 0.57]
 approxColor=[0.12 0.62 0.88]
 meshResolution=28
 motionRange=[0 12]
 baseObjects
end
methods
 function obj=DRAW_DRONE_OBSTACLES(world,varargin)
  opts=struct(varargin{:});
  obj.world=world;
  if isfield(opts,'ax') && ~isempty(opts.ax)
   obj.ax=opts.ax;
  else
   fig=figure('Name','DRONE_OBSTACLES');
   obj.ax=axes('Parent',fig);
  end
  if isfield(opts,'showOriginal'),obj.showOriginal=logical(opts.showOriginal);end
  if isfield(opts,'showLabels'),obj.showLabels=logical(opts.showLabels);end
  if isfield(opts,'showApprox'),obj.showApprox=logical(opts.showApprox);end
  if isfield(opts,'showGround'),obj.showGround=logical(opts.showGround);end
  if isfield(opts,'originalAlpha'),obj.originalAlpha=opts.originalAlpha;end
  if isfield(opts,'originalColor'),obj.originalColor=opts.originalColor;end
  if isfield(opts,'approxColor'),obj.approxColor=opts.approxColor;end
  if isfield(opts,'approxAlpha'),obj.approxAlpha=opts.approxAlpha;end
  if isfield(opts,'meshResolution'),obj.meshResolution=opts.meshResolution;end
  if isfield(opts,'motionRange'),obj.motionRange=opts.motionRange;end
  if ~isfield(world,'baseObjects')
   % Compatible with an older world without dynamic metadata.
   obj.baseObjects=world.objects;
  else
   obj.baseObjects=world.baseObjects;
  end
  ax=obj.ax;hold(ax,'on');grid(ax,'on');axis(ax,'equal');
  xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
  n=numel(obj.baseObjects);
  obj.originalGroups=gobjects(n,1);
  obj.approxGroups=gobjects(n,1);
  obj.labels=gobjects(n,1);
  obj.labelOffsets=zeros(n,1);
  for i=1:n
   obs=obj.baseObjects(i);
   allParts=world.parts;
   belonging=strcmp({allParts.obstacleName},obs.name);
   if obj.showOriginal
    g=hgtransform('Parent',ax);
    obj.originalGroups(i)=g;
    obs_draw_original_shape(g,obs,world.options,obj.originalColor, ...
        obj.originalAlpha,obj.meshResolution);
   end
   if obj.showApprox
    g=hgtransform('Parent',ax);
    obj.approxGroups(i)=g;
    obs_draw_approx_parts(g,allParts(belonging),obj.approxColor, ...
        obj.approxAlpha,obj.meshResolution);
   end
   if obj.showLabels
    c=obs.position;
    C=obs_components(obs,world.options);
    [~,hi]=obs_aabb_world(obs,C);
    obj.labelOffsets(i)=hi(3)-c(3)+.18;
    obj.labels(i)=text(ax,c(1),c(2),c(3)+obj.labelOffsets(i),obs.name, ...
        'Interpreter','none','FontSize',8,'Color',[.12 .22 .30]);
   end
  end
  fit=true;
  if isfield(opts,'fitToObstacles'),fit=logical(opts.fitToObstacles);end
  if fit, obj.expand_axis_limits();end
  if obj.showGround && isfield(world,'ground') && ~isempty(world.ground)
   obj.add_ground();
  end
  view(ax,3);
  obj.update(0);
 end
 function update(obj,t)
  % 障害物ごとの独立軌道を現在時刻で評価し、描画姿勢を更新。
  if ~isfield(obj.world,'sourceObjects'),return;end
  for i=1:numel(obj.baseObjects)
   original=obj.world.sourceObjects(i);
   if ~isfield(original,'isDynamic') || ~original.isDynamic,continue;end
   [p1,R1]=obs_pose_at_time(original,t);
   p0=obj.baseObjects(i).position;
   R0=obj.baseObjects(i).R;
   D=R1*R0';
   d=p1(:)-D*p0(:);
   H=eye(4);H(1:3,1:3)=D;H(1:3,4)=d;
   if i<=numel(obj.originalGroups) && isgraphics(obj.originalGroups(i))
    set(obj.originalGroups(i),'Matrix',H);
   end
   if i<=numel(obj.approxGroups) && isgraphics(obj.approxGroups(i))
    set(obj.approxGroups(i),'Matrix',H);
   end
   if i<=numel(obj.labels) && isgraphics(obj.labels(i))
    set(obj.labels(i),'Position',p1+[0 0 obj.labelOffsets(i)]);
   end
  end
 end
 function expand_axis_limits(obj)
  ax=obj.ax;
  limits=[ax.XLim;ax.YLim;ax.ZLim];
  if numel(obj.baseObjects)>0
   lo=[Inf Inf Inf];hi=[-Inf -Inf -Inf];
   for i=1:numel(obj.baseObjects)
    base=obj.baseObjects(i);
    C=obs_components(base,obj.world.options);
    [a,b]=obs_aabb_world(base,C);
    lo=min(lo,a);hi=max(hi,b);
    % Display-only bounds for independently moving obstacles; not a swept
    % collision enclosure. Users can override motionRange for long videos.
    if isfield(base,'isDynamic') && base.isDynamic
     tlist=linspace(obj.motionRange(1),obj.motionRange(2),31);
     for tt=tlist
      [pc,Rc,rpyc]=obs_pose_at_time(obj.world.sourceObjects(i),tt);
      instance=base;instance.position=pc;instance.R=Rc;
      instance.rpyDeg=rpyc;
      [a,b]=obs_aabb_world(instance,C);
      lo=min(lo,a);hi=max(hi,b);
     end
    end
   end
   if all(isfinite([lo hi]))
    pad=max(.4,.07*max(hi-lo));
    limits(:,1)=min(limits(:,1),lo(:)-pad);
    limits(:,2)=max(limits(:,2),hi(:)+pad);
   end
  end
  if isfield(obj.world,'ground') && ~isempty(obj.world.ground)
   limits(3,1)=min(limits(3,1),obj.world.ground.point(3)-.3);
  end
  set(ax,'XLim',limits(1,:),'YLim',limits(2,:),'ZLim',limits(3,:));
 end
 function add_ground(obj)
  ground=obj.world.ground; ax=obj.ax;
  % 現時点では ground_z0 を想定するが、軸平行でない場合は描かない。
  if norm(ground.normal-[0 0 1])>1e-10,return;end
  x=ax.XLim;y=ax.YLim;
  obj.groundSurface=surface('Parent',ax, ...
    'XData',[x(1) x(2);x(1) x(2)], ...
    'YData',[y(1) y(1);y(2) y(2)], ...
    'ZData',ground.point(3)*ones(2), ...
    'FaceColor',[.62 .62 .62],'FaceAlpha',.17,'EdgeColor','none');
 end
end
end
