function CBF_TRAJ_SAFE_BASE_PLOT(logger, uiAxes)
% CBF_TRAJ_SAFE_BASE_PLOT  Plot ordinary flight diagnostics without
% concatenating heterogeneous result structs across takeoff/flight/landing.
%
% REV15, SIM VISUALIZATION ONLY. Does not change control or logger contents.
% The standard LOGGER.plot implementation concatenates whole result structs
% before accessing .state and fails when reference.result gains cbfTraj only
% during flight. Read each logged CELL snapshot separately instead.
%
if ~isprop(logger,'Data') || ~isstruct(logger.Data) || ...
        ~isfield(logger.Data,'agent') || isempty(logger.Data.agent) || ...
        ~isfield(logger.Data,'t') || logger.k<1
    return
end
D=logger.Data;
A=D.agent(1);
n=min(double(logger.k),numel(D.t));
t=double(D.t(1:n));t=t(:);
if isfield(D,'phase')
    p=char(D.phase(1:n));p=p(:);
    use=p=='f';
else
    use=true(n,1);
end
ix=find(use);
if isempty(ix),return;end

pRef=nan(n,3);pEst=nan(n,3);pLoad=nan(n,3);
vRef=nan(n,3);vEst=nan(n,3);vLoad=nan(n,3);
timeCtrl=nan(n,1);

for ii=1:numel(ix)
    k=ix(ii);
    ref=snapshot(A,'reference',k);
    est=snapshot(A,'estimator',k);
    ctr=snapshot(A,'controller',k);
    refState=member(ref,'state');
    estState=member(est,'state');
    xd=member(refState,'xd');
    if isnumeric(xd) && numel(xd)>=7 && all(isfinite(xd([1:3 5:7])))
        xd=double(xd(:));
        pRef(k,:)=xd(1:3).';vRef(k,:)=xd(5:7).';
    else
        pRef(k,:)=vec3(member(refState,'p'));
        vRef(k,:)=vec3(member(refState,'v'));
    end
    pEst(k,:)=vec3(member(estState,'p'));
    pLoad(k,:)=vec3(member(estState,'pL'));
    vEst(k,:)=vec3(member(estState,'v'));
    vLoad(k,:)=vec3(member(estState,'vL'));
    a=member(ctr,'controllertime_total');
    if isnumeric(a)&&isscalar(a)&&isfinite(a),timeCtrl(k)=double(a);end
end

% Keep the UIAxes trend: estimated drone XYZ, target XYZ and load XYZ.
if nargin>=2 && ~isempty(uiAxes) && isgraphics(uiAxes)
    cla(uiAxes);
    plot(uiAxes,t(ix),pEst(ix,:),'-','LineWidth',0.8);
    hold(uiAxes,'on');
    plot(uiAxes,t(ix),pRef(ix,:),'--','LineWidth',0.8);
    plot(uiAxes,t(ix),pLoad(ix,:),':','LineWidth',0.8);
    hold(uiAxes,'off');grid(uiAxes,'on');
    xlabel(uiAxes,'time [s]');ylabel(uiAxes,'position [m]');
    title(uiAxes,'Flight f: drone / SLOAD target / load (XYZ)');
end

axisXYZ = 'XYZ'; axisXY = 'XY';
fig=figure(2);clf(fig);
tiledlayout(fig,3,1,'TileSpacing','compact');
for j=1:3
    nexttile;
    plot(t(ix),pEst(ix,j),'-','LineWidth',1);hold on;
    plot(t(ix),pRef(ix,j),'--','LineWidth',1);
    plot(t(ix),pLoad(ix,j),':','LineWidth',1);
    hold off;grid on;
    ylabel(sprintf('%c [m]', axisXYZ(j))); %#ok<NBRAK>
    if j==1,legend('drone estimate','SLOAD target','load estimate','Location','best');end
end
xlabel('time [s]');sgtitle('Flight f: positions');

fig=figure(3);clf(fig);
tiledlayout(fig,2,1,'TileSpacing','compact');
for j=1:2
    nexttile;
    plot(t(ix),vEst(ix,j),'-','LineWidth',1);hold on;
    plot(t(ix),vRef(ix,j),'--','LineWidth',1);
    plot(t(ix),vLoad(ix,j),':','LineWidth',1);
    hold off;grid on;
    ylabel(sprintf('%c velocity [m/s]',axisXY(j))); %#ok<NBRAK>
    if j==1,legend('drone estimate','SLOAD target','load estimate','Location','best');end
end
xlabel('time [s]');sgtitle('Flight f: XY velocities');

fig=figure(4);clf(fig);
plot(t(ix),timeCtrl(ix),'-','LineWidth',1);grid on;
xlabel('time [s]');ylabel('controller time [s]');
title('Flight f: controller.result.controllertime_total');
end

function e=snapshot(A,section,k)
e=[];
block=member(A,section);
res=member(block,'result');
if iscell(res)
    if k<=numel(res),e=res{k};end
elseif isstruct(res)
    if k<=numel(res),e=res(k);end
end
end

function x=member(o,name)
x=[];
if isempty(o),return;end
if isstruct(o) && isscalar(o) && isfield(o,name)
    x=o.(name);
elseif isobject(o) && isprop(o,name)
    x=o.(name);
end
end

function x=vec3(o)
x=nan(1,3);
if isnumeric(o) && numel(o)==3 && all(isfinite(o(:)))
    x=double(o(:)).';
end
end
