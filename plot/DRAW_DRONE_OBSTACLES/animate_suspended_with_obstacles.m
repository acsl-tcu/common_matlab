function [fig, obstacleDrawer] = animate_suspended_with_obstacles(logger, self, world, varargin)
% ANIMATE_SUSPENDED_WITH_OBSTACLES
% Replay the unchanged DRAW_SUSPENDED_LOAD together with independently
% moving obstacles. Draw / rendering ONLY; HLC, references and CBF untouched.
%
% By default, the camera fits the whole recorded flight path, as the
% original DRAW_SUSPENDED_LOAD does. Distant obstacles do NOT enlarge it.
%
% Example (no MP4 output):
%   animate_suspended_with_obstacles(logger,agent(1),worldObstacles, ...
%       'target',1,'skip',4,'fps',10,'mp4','', ...
%       'cameraMode','wholeFlight');
%
% cameraMode:
%   'wholeFlight' (default) fixed limits covering all flown positions
%   'follow'     follow drone/load, display recent flown paths
%   'scene'      old behavior: fit the whole obstacle scene into the view
% Passing 'lims'=[xmin xmax;ymin ymax;zmin zmax] overrides cameraMode.
%
% Options: cameraHalfSpan [hx hy hz] m, cameraSmoothing in [0,1),
%          trailSeconds (Inf for all history), showDroneTrail,
%          showLoadTrail, showReference, showLabels, originalAlpha,
%          approxAlpha, originalColor, approxColor.
%
% NOTE: DRAW_SUSPENDED_LOAD.m is not modified.

opts=struct(varargin{:});
if ~isfield(opts,'target'),opts.target=1;end
if ~isequal(opts.target,1)
    error('animate_suspended_with_obstacles:Target', ...
        'This adapter currently supports target=1 only.');
end
if ~isfield(opts,'fps'),opts.fps=30;end
if ~isfield(opts,'skip'),opts.skip=1;end
if ~isfield(opts,'pause'),opts.pause=0;end
if ~isfield(opts,'mp4'),opts.mp4='';end
if ~isfield(opts,'showApprox'),opts.showApprox=true;end
if ~isfield(opts,'showOriginal'),opts.showOriginal=true;end
if ~isfield(opts,'showGround'),opts.showGround=true;end
if ~isfield(opts,'showLabels'),opts.showLabels=false;end
if ~isfield(opts,'showDroneTrail'),opts.showDroneTrail=true;end
if ~isfield(opts,'showLoadTrail'),opts.showLoadTrail=true;end
if ~isfield(opts,'showReference'),opts.showReference=true;end
if ~isfield(opts,'trailSeconds'),opts.trailSeconds=Inf;end
if ~isfield(opts,'cameraMode'),opts.cameraMode='wholeFlight';end
if ~isfield(opts,'cameraHalfSpan'),opts.cameraHalfSpan=[4.0 4.0 3.5];end
if ~isfield(opts,'cameraSmoothing'),opts.cameraSmoothing=0.60;end
if ~isfield(opts,'motionRange'),opts.motionRange=[];end
if ~isfield(opts,'lims'),opts.lims=[];end
if ~isfield(opts,'originalAlpha'),opts.originalAlpha=0.68;end
if ~isfield(opts,'approxAlpha'),opts.approxAlpha=0.12;end

validateattributes(opts.fps,{'numeric'},{'scalar','positive','finite'});
validateattributes(opts.skip,{'numeric'},{'scalar','integer','positive'});
validateattributes(opts.pause,{'numeric'},{'scalar','nonnegative','finite'});
validateattributes(opts.cameraHalfSpan,{'numeric'},{'vector','numel',3,'positive','finite'});
validateattributes(opts.cameraSmoothing,{'numeric'},{'scalar','>=',0,'<',1});
validateattributes(opts.trailSeconds,{'numeric'},{'scalar','nonnegative'});
if ~isempty(opts.lims)
    validateattributes(opts.lims,{'numeric'},{'size',[3 2],'finite'});
    if any(opts.lims(:,2)<=opts.lims(:,1))
        error('animate_suspended_with_obstacles:Limits', ...
            'lims must be [xmin xmax; ymin ymax; zmin zmax].');
    end
end
cameraMode=lower(char(opts.cameraMode));
if ~ismember(cameraMode,{'follow','wholeflight','scene'})
    error('animate_suspended_with_obstacles:CameraMode', ...
        'cameraMode must be follow, wholeFlight or scene.');
end

% Logger formats follow the unmodified DRAW_SUSPENDED_LOAD conventions.
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
numFrames=min([max(0,numel(t)-1),size(p,1),size(q,1),size(u,1),size(pL,1)]);
if numFrames<1,error('animate_suspended_with_obstacles:Empty','No logged frames.');end

initArgs={'target',opts.target,'self',self};
if ~isempty(opts.lims),initArgs=[initArgs,{'lims',opts.lims}];end
mov=DRAW_SUSPENDED_LOAD(logger,initArgs{:});
ax=mov.ax;
fig=ancestor(ax,'figure');
if strcmpi(get(fig,'WindowStyle'),'normal')
    fig.Units='pixels';
    figurePos=fig.Position;
    figurePos(3:4)=[1280 720];
    fig.Position=figurePos;
end
if isempty(opts.motionRange)
    motionRange=[min(t(1:numFrames)) max(t(1:numFrames))];
else
    motionRange=opts.motionRange;
end

drawerArgs={'ax',ax,'motionRange',motionRange, ...
    'fitToObstacles',strcmp(cameraMode,'scene') && isempty(opts.lims), ...
    'showOriginal',opts.showOriginal,'showApprox',opts.showApprox, ...
    'showGround',opts.showGround,'showLabels',opts.showLabels, ...
    'originalAlpha',opts.originalAlpha,'approxAlpha',opts.approxAlpha};
if isfield(opts,'originalColor')
    drawerArgs=[drawerArgs,{'originalColor',opts.originalColor}];
end
if isfield(opts,'approxColor')
    drawerArgs=[drawerArgs,{'approxColor',opts.approxColor}];
end
obstacleDrawer=DRAW_DRONE_OBSTACLES(world,drawerArgs{:});

% Reference path (if available) and ACTUAL executed paths.
% Both flight and load traces are updated as the animation progresses.
if opts.showReference
    try
        ref=read_logged(logger,opts.target,'p','r');
        if size(ref,2)==3
            ref=ref(1:min(numFrames,size(ref,1)),:);
            plot3(ax,ref(:,1),ref(:,2),ref(:,3),'-', ...
                'Color',[0.92 0.43 0.27],'LineWidth',1.1, ...
                'DisplayName','Reference (nominal)');
        end
    catch
        % The reference track is optional.
    end
end
if opts.showDroneTrail
    droneTrail=plot3(ax,nan,nan,nan,'-', ...
        'Color',[0.12 0.34 0.83],'LineWidth',1.9, ...
        'DisplayName','Drone (actual)');
else
    droneTrail=[];
end
if opts.showLoadTrail
    loadTrail=plot3(ax,nan,nan,nan,'-', ...
        'Color',[0.10 0.58 0.36],'LineWidth',1.7, ...
        'DisplayName','Load (actual)');
else
    loadTrail=[];
end

% Lock limits after all plot3 traces are created.
% This prevents distant obstacles or logged reference data re-centering
% the drone animation during playback.
axis(ax,'equal');
view(ax,3);
axis(ax,'manual');
camCenter=[];
if ~isempty(opts.lims)
    set(ax,'XLim',opts.lims(1,:),'YLim',opts.lims(2,:), ...
        'ZLim',opts.lims(3,:));
elseif strcmp(cameraMode,'wholeflight')
    X=[p(1:numFrames,:);pL(1:numFrames,:)];
    lo=min(X,[],1);hi=max(X,[],1);
    % The original draw fits the actual flight, not distant obstacles.
    half=max((hi-lo)/2+[1 1 1],[1.5 1.5 1.5]);
    mid=(lo+hi)/2;
    set_camera_limits(ax,mid,half);
elseif strcmp(cameraMode,'follow')
    mid=(p(1,:)+pL(1,:))/2;
    half=max(opts.cameraHalfSpan(:)',abs(p(1,:)-pL(1,:))/2+0.8);
    camCenter=mid;
    set_camera_limits(ax,mid,half);
end
update_ground_extent(obstacleDrawer,ax);

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
        if ~isempty(droneTrail) && isgraphics(droneTrail)
            startIndex=trail_start_index(t,k,opts.trailSeconds);
            set(droneTrail,'XData',p(startIndex:k,1), ...
                'YData',p(startIndex:k,2),'ZData',p(startIndex:k,3));
        end
        if ~isempty(loadTrail) && isgraphics(loadTrail)
            startIndex=trail_start_index(t,k,opts.trailSeconds);
            set(loadTrail,'XData',pL(startIndex:k,1), ...
                'YData',pL(startIndex:k,2),'ZData',pL(startIndex:k,3));
        end
        if isempty(opts.lims) && strcmp(cameraMode,'follow')
            requested=(p(k,:)+pL(k,:))/2;
            camCenter=opts.cameraSmoothing*camCenter ...
                +(1-opts.cameraSmoothing)*requested;
            half=max(opts.cameraHalfSpan(:)',abs(p(k,:)-pL(k,:))/2+0.8);
            % Never clip the drone/load if smoothing slightly lags behind.
            half=max(half,max(abs(p(k,:)-camCenter), ...
                               abs(pL(k,:)-camCenter))+0.7);
            set_camera_limits(ax,camCenter,half);
            update_ground_extent(obstacleDrawer,ax);
        end
        title(ax,sprintf('time : %.3f [s]',t(k)));
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

function set_camera_limits(ax,center,half)
% Camera follows by translating world-coordinate window, preserving view.
% Explicitly translate the camera position and its target together, since
% view()/axis equal may leave camera properties in manual mode.
oldTarget=get(ax,'CameraTarget');
oldPosition=get(ax,'CameraPosition');
cameraOffset=oldPosition-oldTarget;
set(ax,'XLim',center(1)+[-half(1),half(1)], ...
       'YLim',center(2)+[-half(2),half(2)], ...
       'ZLim',center(3)+[-half(3),half(3)]);
set(ax,'CameraTarget',center,'CameraPosition',center+cameraOffset);
end

function update_ground_extent(drawer,ax)
% Existing z=0 ground patch follows the camera's X/Y viewing window.
if isempty(drawer.groundSurface) || ~isgraphics(drawer.groundSurface)
    return;
end
x=ax.XLim;y=ax.YLim;
set(drawer.groundSurface, ...
    'XData',[x(1) x(2);x(1) x(2)], ...
    'YData',[y(1) y(1);y(2) y(2)]);
end

function first=trail_start_index(t,k,seconds)
if isinf(seconds)
    first=1;
else
    first=find(t(1:k)>=t(k)-seconds,1,'first');
    if isempty(first),first=1;end
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
