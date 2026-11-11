function [fig, obstacleDrawer] = animate_suspended_with_obstacles(logger, self, world, varargin)
% ANIMATE_SUSPENDED_WITH_OBSTACLES 既存 DRAW_SUSPENDED_LOAD を改変せず障害物を重ねる。
%
% [fig, layer] = animate_suspended_with_obstacles(logger,agent(1),world, ...
%     'target',1,'fps',30,'skip',2,'pause',0, ...
%     'mp4','Data/FlightWithObstacles.mp4');
%
% Original drone DRAW is used only through its public constructor, .ax and
% .draw method. Its .animation method is deliberately NOT used because that
% would not update independently moving obstacles on every video frame.
%
% Current version targets one drone (target=1). Static/dynamic obstacles
% are rendering only; the controller and simulator are never modified.
opts=struct(varargin{:});
if ~isfield(opts,'target'), opts.target=1; end
if ~isequal(opts.target,1)
    error('animate_suspended_with_obstacles:Target', ...
        'This standalone adapter supports target=1 only.');
end
if ~isfield(opts,'fps'),opts.fps=30;end
if ~isfield(opts,'skip'),opts.skip=1;end
if ~isfield(opts,'pause'),opts.pause=0;end
if ~isfield(opts,'mp4'),opts.mp4='';end
if ~isfield(opts,'showApprox'),opts.showApprox=true;end
if ~isfield(opts,'showOriginal'),opts.showOriginal=true;end
if ~isfield(opts,'showGround'),opts.showGround=true;end
if ~isfield(opts,'motionRange'),opts.motionRange=[];end
if ~isfield(opts,'lims'),opts.lims=[];end
validateattributes(opts.fps,{'numeric'},{'scalar','positive','finite'});
validateattributes(opts.skip,{'numeric'},{'scalar','integer','positive'});
validateattributes(opts.pause,{'numeric'},{'scalar','nonnegative','finite'});

% Logger formats follow the unmodified DRAW_SUSPENDED_LOAD.m conventions.
t=logger.data(0,'t','');t=t(:);
p=read_logged(logger,opts.target,'p','p');
q=read_logged(logger,opts.target,'q','p');
u=read_logged(logger,opts.target,'input','');
if size(p,2)~=3 || size(u,2)~=4
    error('animate_suspended_with_obstacles:LoggerShape', ...
        'Expected position Nx3 and controller input Nx4.');
end
pL=load_position(logger,opts.target,self,p);
Q=convert_to_axis_angle(q);
% The original DRAW uses t(1:end-1), because the logger may contain one
% allocated trailing slot. Do not index past actual logged data.
numFrames=min([max(0,numel(t)-1),size(p,1),size(q,1),size(u,1),size(pL,1)]);
if numFrames<1,error('animate_suspended_with_obstacles:Empty','No logged frames.');end

initArgs={'target',opts.target,'self',self};
if ~isempty(opts.lims),initArgs=[initArgs,{'lims',opts.lims}];end
mov=DRAW_SUSPENDED_LOAD(logger,initArgs{:});
fig=ancestor(mov.ax,'figure');
if ~isempty(opts.motionRange)
    motionRange=opts.motionRange;
else
    motionRange=[min(t(1:numFrames)) max(t(1:numFrames))];
end
obstacleDrawer=DRAW_DRONE_OBSTACLES(world,'ax',mov.ax, ...
    'motionRange',motionRange,'showOriginal',opts.showOriginal, ...
    'showApprox',opts.showApprox,'showGround',opts.showGround);
if ~isempty(opts.lims)
    set(mov.ax,'XLim',opts.lims(1,:), ...
        'YLim',opts.lims(2,:),'ZLim',opts.lims(3,:));
end

% Match original animation's reference trajectory when available.
try
    r=read_logged(logger,opts.target,'p','r');
    if size(r,2)==3
        plot3(mov.ax,r(:,1),r(:,2),r(:,3),'r-','LineWidth',1);
    end
catch
    % A reference track is optional.
end

writer=[];
if ~(isempty(opts.mp4) || isequal(opts.mp4,false))
    if isequal(opts.mp4,true)
        outputName=fullfile('Data','FlightWithObstacles.mp4');
    else
        outputName=char(opts.mp4);
    end
    [outdir,~,~]=fileparts(outputName);
    if ~isempty(outdir) && ~exist(outdir,'dir'),mkdir(outdir);end
    writer=VideoWriter(outputName,'MPEG-4');
    writer.FrameRate=opts.fps;
    open(writer);
end

try
    for k=1:opts.skip:numFrames
        mov.draw(opts.target,p(k,:),Q(k,:),u(k,:),pL(k,:));
        obstacleDrawer.update(t(k));
        title(mov.ax,sprintf('time : %.3f [s]',t(k)));
        drawnow;
        if ~isempty(writer),writeVideo(writer,getframe(fig));end
        if opts.pause>0,pause(opts.pause);end
    end
catch ME
    if ~isempty(writer),close(writer);end
    rethrow(ME);
end
if ~isempty(writer)
    close(writer);
    fprintf('[animate_suspended_with_obstacles] saved: %s\n',outputName);
end
end

function x=read_logged(logger,source,var,attribute)
raw=logger.data(source,var,attribute);
if isempty(raw),error('Logger variable %s is empty',var);end
if mod(size(raw,2),numel(source))~=0
    error('Logger variable %s shape does not match target',var);
end
x=reshape(raw,size(raw,1),size(raw,2)/numel(source),numel(source));
x=x(:,:,1);
end

function pL=load_position(logger,source,self,p)
% Same priority as original DRAW_SUSPENDED_LOAD.get_load_position.
for name={'estimator.result.state.pL','pL'}
    try
        if strcmp(name{1},'pL'),attribute='p';else,attribute='e';end
        pL=read_logged(logger,source,name{1},attribute);
        if size(pL,2)==3,return;end
    catch
    end
end
for name={'estimator.result.state.pT','pT'}
    try
        if strcmp(name{1},'pT'),attribute='p';else,attribute='e';end
        pT=read_logged(logger,source,name{1},attribute);
        if size(pT,2)==3
            pL=p+pT*self.parameter.cableL;
            return;
        end
    catch
    end
end
error('animate_suspended_with_obstacles:NoLoadPosition', ...
    'pL/pT not available from logger.');
end

function Q=convert_to_axis_angle(q)
% This uses the same Eul2Quat/quaternion conventions as the original DRAW.
if size(q,2)==3
    rotQ=quaternion(Eul2Quat(q')');
elseif size(q,2)==4
    rotQ=quaternion(q);
elseif size(q,2)==9
    rotQ=quaternion(q,'rotmat','frame');
else
    error('animate_suspended_with_obstacles:Attitude', ...
        'Expected q with 3,4 or 9 columns.');
end
rotVec=rotvec(rotQ);
angles=vecnorm(rotVec,2,2);
Q=zeros(size(q,1),4);
Q(:,1)=1;
sel=angles>1e-12;
Q(sel,1:3)=rotVec(sel,:)./angles(sel);
Q(sel,4)=angles(sel);
end
