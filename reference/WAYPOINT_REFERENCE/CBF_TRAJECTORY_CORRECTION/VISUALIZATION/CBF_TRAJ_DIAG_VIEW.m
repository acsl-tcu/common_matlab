function out=CBF_TRAJ_DIAG_VIEW(action,varargin)
% CBF_TRAJ_DIAG_VIEW  Optional, Sim-only trajectory-correction diagnostics.
%   line=CBF_TRAJ_DIAG_VIEW('display',agent,time,baseLine,enabled)
%   CBF_TRAJ_DIAG_VIEW('plot',logger,figureNo)
%   CBF_TRAJ_DIAG_VIEW('baseplot',logger,UIAxes)
%
% This is a READ-ONLY view of reference.result.cbfTraj. It does not call
% HLC, CBF, the plant, or the waypoint generator. No persistent history:
% figures use the already-saved LOGGER reference/estimator snapshots.
% The baseLine may already contain CBF_REAL_DIAG_VIEW output.
% Flight figures also show zero correction when no intervention occurred.
% BASEPLOT works with or without trajectory correction, for all CBF cases.

out=[];
switch upper(char(action))
    case 'DISPLAY'
        if numel(varargin)~=4
            error('CBFTrajView:DisplayArgs', ...
                'Expected (agent,time,baseLine,enabled).');
        end
        out=display_line(varargin{:});
    case 'BASEPLOT'
        if numel(varargin)~=2
            error('CBFTrajView:BasePlotArgs','Expected (logger,UIAxes).');
        end
        base_plot(varargin{:});
    case 'PLOT'
        if isempty(varargin)
            error('CBFTrajView:PlotArgs','LOGGER is required.');
        end
        fig=30;
        if numel(varargin)>=2 && ~isempty(varargin{2}),fig=varargin{2};end
        plot_history(varargin{1},fig);
    otherwise
        error('CBFTrajView:Action','Unsupported action: %s',char(action));
end
end

function line=display_line(agent,~,baseLine,enabled)
line=baseLine;
if ~enabled || isempty(baseLine) || isempty(agent)
    return
end
if ~strcmp(char(agent(1).cha),'f')
    return
end
rr=agent(1).reference.result;
if ~isstruct(rr) || ~isfield(rr,'cbfTraj') || ...
        ~isstruct(rr.cbfTraj)
    return
end
r=rr.cbfTraj;
required={'mode','before28','after28','signedCarryPosition', ...
    'signedCarryVelocity','lastIntervalDeltaPV','sourceStatus'};
if ~all(isfield(r,required)) || strcmpi(char(r.mode),'NOMINAL')
    % First CBF correction is applied to the target from the NEXT cycle.
    % Before then no extra console output is generated.
    return
end
if numel(r.before28)~=28 || numel(r.after28)~=28 || ...
        numel(r.signedCarryPosition)~=3 || ...
        numel(r.signedCarryVelocity)~=3 || ...
        numel(r.lastIntervalDeltaPV)~=6
    return
end
raw=r.before28(:);new=r.after28(:);
d=r.signedCarryPosition(:);w=r.signedCarryVelocity(:);
delta=r.lastIntervalDeltaPV(:);
line=[char(line),sprintf([ ...
    '\n  TRAJ %s src=%s | SLOADpre=[%+.3f,%+.3f,%+.3f]' ...
    ' SLOADpost=[%+.3f,%+.3f,%+.3f]' ...
    ' | carryP=[%+.3g,%+.3g,%+.3g]' ...
    ' carryV=[%+.3g,%+.3g,%+.3g]' ...
    ' | prevDeltaP=[%+.3g,%+.3g,%+.3g]' ...
    ' prevDeltaV=[%+.3g,%+.3g,%+.3g]'], ...
    char(r.mode),char(r.sourceStatus),raw(1:3),new(1:3),d,w, ...
    delta(1:3),delta(4:6))];
if isfield(r,'cbfId') && isnumeric(r.cbfId) && r.cbfId==3 && ...
        isfield(r,'physicalPrevious') && isstruct(r.physicalPrevious)
    h=r.physicalPrevious;
    required={'valid','T0','Tdot0','TddotApplied', ...
        'TddotBaseline','TendApplied','TendBaseline'};
    if all(isfield(h,required)) && isscalar(h.valid) && logical(h.valid)
        vals=[h.T0,h.Tdot0,h.TddotApplied,h.TddotBaseline, ...
              h.TendApplied,h.TendBaseline];
        if isnumeric(vals) && numel(vals)==6 && all(isfinite(vals))
            line=[line,sprintf(['\n  TRAJ prev ID3: T0=%+.4f N' ...
                ' Tdot0=%+.4f N/s Tddot(CBF/base)=[%+.4f,%+.4f] N/s^2' ...
                ' Tend(CBF/base)=[%+.4f,%+.4f] N'],vals)];
        end
    end
end
end

function base_plot(logger,uiAx)
% All plotting is performed after the run. Extract only leaf values.
[t,phase,A]=logged_samples(logger);
if isempty(t),return;end
n=numel(t);
refP=nan(n,3);refV=refP;droneP=refP;droneV=refP;
loadP=refP;loadV=refP;cycleMs=nan(n,1);
for k=1:n
    r=sample(A,'reference',k); e=sample(A,'estimator',k);
    c=sample(A,'controller',k);
    xd=member(member(r,'state'),'xd');
    if isnumeric(xd) && isreal(xd) && numel(xd)>=7
        refP(k,:)=finite_row(xd(1:3),3);
        refV(k,:)=finite_row(xd(5:7),3);
    end
    st=member(e,'state');
    droneP(k,:)=finite_row(member(st,'p'),3);
    droneV(k,:)=finite_row(member(st,'v'),3);
    loadP(k,:)=finite_row(member(st,'pL'),3);
    loadV(k,:)=finite_row(member(st,'vL'),3);
    cycleMs(k)=1000*finite_row(member(c,'controllertime_total'),1);
end
allPhase=ismember(phase,['t','f','l']); flight=phase=='f';
if any(allPhase)
    if isgraphics(uiAx,'axes') || isa(uiAx,'matlab.ui.control.UIAxes')
        cla(uiAx); position_lines(uiAx,t,allPhase,refP,loadP,droneP);
        title(uiAx,'Position: takeoff / flight / landing');
    end
    % Figure 3 preserves both drone and load velocity, adds the Z axis.
    fig=figure(3);clf(fig);tl=tiledlayout(fig,3,1,'TileSpacing','compact');
    names={'X','Y','Z'};
    for j=1:3
        ax=nexttile(tl);hold(ax,'on');
        plot(ax,t(allPhase),refV(allPhase,j),'--','DisplayName','Target load');
        plot(ax,t(allPhase),loadV(allPhase,j),'-','DisplayName','Estimated load');
        plot(ax,t(allPhase),droneV(allPhase,j),':','DisplayName','Estimated drone');
        grid(ax,'on');ylabel(ax,[names{j},' [m/s]']);
        if j==1,legend(ax,'Location','best');end
        hold(ax,'off');
    end
    xlabel(ax,'Time [s]');title(tl,'Velocity: takeoff / flight / landing');
else
    warning('CBFTrajView:NoTFL','No logged takeoff/flight/landing samples.');
end
if any(flight)
    fig=figure(2);clf(fig);ax=axes('Parent',fig);
    position_lines(ax,t,flight,refP,loadP,droneP);title(ax,'Flight position');
    fig=figure(4);clf(fig);ax=axes('Parent',fig);
    plot(ax,t(flight),cycleMs(flight),'DisplayName','Sensor through plant');
    hold(ax,'on');
    % TIME.dt is not stored here. Infer the sample period from the flight
    % log only when positive differences are available; no fixed 25ms.
    steps=diff(t(flight));steps=steps(isfinite(steps)&steps>0);
    if ~isempty(steps)
        yline(ax,1000*median(steps),'--','DisplayName','Median flight step');
    end
    hold(ax,'off');grid(ax,'on');xlabel(ax,'Time [s]');ylabel(ax,'Elapsed [ms]');
    legend(ax,'Location','best');title(ax,'Calculation time (excluding LOGGER)');
end
end

function position_lines(ax,t,mask,refP,loadP,droneP)
% Color identifies coordinate, line style identifies physical quantity.
colors=lines(3);names={'X','Y','Z'};
hold(ax,'on');
for j=1:3
    plot(ax,t(mask),refP(mask,j),'--','Color',colors(j,:), ...
        'DisplayName',['Target load ',names{j}]);
    plot(ax,t(mask),loadP(mask,j),'-','Color',colors(j,:), ...
        'DisplayName',['Estimated load ',names{j}]);
    plot(ax,t(mask),droneP(mask,j),':','Color',colors(j,:), ...
        'DisplayName',['Estimated drone ',names{j}]);
end
hold(ax,'off');grid(ax,'on');xlabel(ax,'Time [s]');ylabel(ax,'Position [m]');
legend(ax,'Location','best');
end

function plot_history(logger,figNum)
[t,phase,A]=logged_samples(logger);
if isempty(t),return;end
n=numel(t);
pre=nan(n,3);post=pre;loadPos=pre;carryP=pre;carryV=pre;
modeId=nan(n,1);previousActive=modeId;previousValid=modeId;
T=nan(n,8);valid=false(n,1);
for k=find(phase=='f').'
    e=sample(A,'reference',k); r=member(e,'cbfTraj');
    before=member(r,'before28');after=member(r,'after28');
    if ~isnumeric(before) || ~isnumeric(after) || ...
            numel(before)~=28 || numel(after)~=28 || ...
            ~isreal(before) || ~isreal(after) || ...
            any(~isfinite(before(:))) || any(~isfinite(after(:)))
        continue
    end
    valid(k)=true;
    pre(k,:)=finite_row(before(1:3),3);
    post(k,:)=finite_row(after(1:3),3);
    carryP(k,:)=finite_row(member(r,'signedCarryPosition'),3);
    carryV(k,:)=finite_row(member(r,'signedCarryVelocity'),3);
    switch upper(char(member(r,'mode')))
        case 'NOMINAL',modeId(k)=0;
        case 'CARRY',modeId(k)=1;
        case 'RETURN',modeId(k)=2;
    end
    st=member(sample(A,'estimator',k),'state');
    loadPos(k,:)=finite_row(member(st,'pL'),3);
    h=member(r,'physicalPrevious');
    previousValid(k)=flag_value(member(h,'valid'));
    if previousValid(k)==1
        previousActive(k)=flag_value(member(h,'active'));
        id=member(r,'cbfId');
        if isnumeric(id) && isscalar(id) && id==3
            names={'T0','Tdot0','TddotApplied','TddotBaseline', ...
                'TendApplied','TendBaseline','TdotEndApplied','TdotEndBaseline'};
            for j=1:numel(names),T(k,j)=finite_row(member(h,names{j}),1);end
        end
    end
end
if ~any(valid)
    warning('CBFTrajView:NoFlightReference','No valid 28-element flight trajectory diagnostics.');
    return
end
% Keep NaNs in the flight sequence: missing samples must not be bridged.
f=phase=='f';t=t(f);pre=pre(f,:);post=post(f,:);loadPos=loadPos(f,:);
carryP=carryP(f,:);carryV=carryV(f,:);modeId=modeId(f);
previousActive=previousActive(f);previousValid=previousValid(f);T=T(f,:);
delta=post-pre;axisNames={'X','Y','Z'};
fig=figure(figNum);clf(fig);tl=tiledlayout(fig,3,1,'TileSpacing','compact');
for j=1:3
    ax=nexttile(tl);hold(ax,'on');
    plot(ax,t,pre(:,j),'--','LineWidth',1.3,'DisplayName','SLOAD before');
    plot(ax,t,post(:,j),'-','LineWidth',1.1,'DisplayName','SLOAD after / HLC target');
    plot(ax,t,loadPos(:,j),':','LineWidth',1.1,'DisplayName','Estimated load');
    if j==1,legend(ax,'Location','best');end
    ylabel(ax,[axisNames{j},' [m]']);grid(ax,'on');hold(ax,'off');
end
xlabel(ax,'Time [s]');title(tl,'CBF trajectory: load target and estimated load');

fig=figure(figNum+1);clf(fig);tl=tiledlayout(fig,4,2,'TileSpacing','compact');
for j=1:3
    ax=nexttile(tl);hold(ax,'on');
    plot(ax,t,1000*delta(:,j),'-','DisplayName','After - before');
    plot(ax,t,1000*carryP(:,j),'--','DisplayName','Stored carry P');
    grid(ax,'on');ylabel(ax,[axisNames{j},' [mm]']);
    if j==1,legend(ax,'Location','best');end
    hold(ax,'off');
    ax=nexttile(tl);
    plot(ax,t,1000*carryV(:,j),'LineWidth',1.05);grid(ax,'on');
    ylabel(ax,[axisNames{j},' [mm/s]']);
end
ax=nexttile(tl);stairs(ax,t,modeId,'LineWidth',1.1);grid(ax,'on');
ylim(ax,[-0.2,2.2]);yticks(ax,[0 1 2]);yticklabels(ax,{'NOMINAL','CARRY','RETURN'});
xlabel(ax,'Time [s]');ylabel(ax,'Target mode');
ax=nexttile(tl);hold(ax,'on');
stairs(ax,t,previousActive,'DisplayName','Previous physical correction');
stairs(ax,t,previousValid,'--','DisplayName','Previous comparison valid');
ylim(ax,[-0.2,1.2]);yticks(ax,[0 1]);grid(ax,'on');xlabel(ax,'Time [s]');
legend(ax,'Location','best');hold(ax,'off');
title(tl,'Signed correction by axis (independent scales) / velocity / mode');

% ID3 values at t(k) describe the preceding interval, never a new solve.
if any(isfinite(T(:,3)) & isfinite(T(:,4)))
    fig=figure(figNum+2);clf(fig);tl=tiledlayout(fig,3,1,'TileSpacing','compact');
    ax=nexttile(tl);hold(ax,'on');
    plot(ax,t,T(:,3),'-','DisplayName','CBF previous');
    plot(ax,t,T(:,4),'--','DisplayName','Bounded nominal previous');
    ylabel(ax,'Tddot [N/s^2]');grid(ax,'on');legend(ax,'Location','best');hold(ax,'off');
    ax=nexttile(tl);hold(ax,'on');
    plot(ax,t,T(:,5),'-','DisplayName','Tend CBF');
    plot(ax,t,T(:,6),'--','DisplayName','Tend bounded nominal');
    plot(ax,t,T(:,1),':','DisplayName','T0');
    ylabel(ax,'Thrust [N]');grid(ax,'on');legend(ax,'Location','best');hold(ax,'off');
    ax=nexttile(tl);hold(ax,'on');
    plot(ax,t,T(:,7),'-','DisplayName','TdotEnd CBF');
    plot(ax,t,T(:,8),'--','DisplayName','TdotEnd bounded nominal');
    plot(ax,t,T(:,2),':','DisplayName','Tdot0');
    ylabel(ax,'Tdot [N/s]');xlabel(ax,'Time [s]');grid(ax,'on');
    legend(ax,'Location','best');hold(ax,'off');
    title(tl,'ID3 thrust extension: previous actuation interval');
end

% Target-only projections use auto scales to reveal small corrections.
% The 3-D comparison keeps equal physical axis units.
fig=figure(figNum+3);clf(fig);tl=tiledlayout(fig,2,2,'TileSpacing','compact');
ax=nexttile(tl);target_projection(ax,pre,post,1,2,axisNames);
title(ax,'Target XY (auto scales)');
ax=nexttile(tl);target_projection(ax,pre,post,2,3,axisNames);
title(ax,'Target YZ (auto scales)');
ax=nexttile(tl);hold(ax,'on');
plot3(ax,pre(:,1),pre(:,2),pre(:,3),'--','DisplayName','SLOAD before');
plot3(ax,post(:,1),post(:,2),post(:,3),'-','DisplayName','SLOAD after');
plot3(ax,loadPos(:,1),loadPos(:,2),loadPos(:,3),':','DisplayName','Estimated load');
axis(ax,'equal');view(ax,3);grid(ax,'on');xlabel(ax,'X [m]');ylabel(ax,'Y [m]');zlabel(ax,'Z [m]');
legend(ax,'Location','best');title(ax,'3D trajectory (equal scale)');hold(ax,'off');
ax=nexttile(tl);hold(ax,'on');
plot(ax,t,vecnorm(loadPos-pre,2,2),'--','DisplayName','Before-target error');
plot(ax,t,vecnorm(loadPos-post,2,2),'-','DisplayName','After-target error');
xlabel(ax,'Time [s]');ylabel(ax,'Load position error [m]');grid(ax,'on');
legend(ax,'Location','best');hold(ax,'off');
title(tl,'Target trajectory geometry and load tracking error');
end

function target_projection(ax,pre,post,i,j,names)
hold(ax,'on');
plot(ax,pre(:,i),pre(:,j),'--','DisplayName','SLOAD before');
plot(ax,post(:,i),post(:,j),'-','DisplayName','SLOAD after');
xlabel(ax,[names{i},' [m]']);ylabel(ax,[names{j},' [m]']);
grid(ax,'on');legend(ax,'Location','best');hold(ax,'off');
end

function [t,phase,A]=logged_samples(logger)
t=[];phase=[];A=[];
D=member(logger,'Data');agents=member(D,'agent');rawT=member(D,'t');
rawPhase=member(D,'phase');k=member(logger,'k');
if isempty(agents) || ~isnumeric(rawT) || ~isnumeric(k) || ...
        ~isscalar(k) || ~isfinite(k) || isempty(rawPhase)
    warning('CBFTrajView:MissingLogger','Missing logger agent/time/phase/k.');return
end
n=min([floor(double(k)),numel(rawT),numel(rawPhase)]);
if n<1,return;end
A=agents(1);t=double(rawT(1:n));t=t(:);
phase=char(rawPhase(1:n));phase=phase(:);
% Original logger phase filter skips the first four warm-up records.
phase(1:min(4,n))='a';
phase(~isfinite(t))='a';
end

function r=sample(A,kind,k)
h=member(member(A,kind),'result');r=[];
if iscell(h) && k<=numel(h),r=h{k};end
end

function v=member(s,name)
v=[];
if isscalar(s)
    if isstruct(s) && isfield(s,name),v=s.(name);
    elseif isobject(s) && isprop(s,name),v=s.(name);end
end
end

function row=finite_row(v,n)
row=nan(1,n);
if isnumeric(v) && isreal(v) && numel(v)==n
    row=double(v(:)).';row(~isfinite(row))=NaN;
end
end

function v=flag_value(x)
v=NaN;
if (islogical(x)||isnumeric(x)) && isscalar(x) && isreal(x) && isfinite(x)
    v=double(logical(x));
end
end
