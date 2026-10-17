function OUT = AF7R51_POSTRUN_DASHBOARD(logger,runtime_profile,dt,replanner)
% AF7R51_POSTRUN_DASHBOARD
% Post-run only.  No online timing path, controller, plant, or safety state is
% changed.  Replaces the old fragmented/blank Figures 1--11 with:
%   Figure 5  : one integrated trajectory/avoidance dashboard
%   Figure 10 : one integrated timing dashboard

% Close old fragmented figures first.
for f=[1 2 3 4 6 7 8 9 11]
    try
        if isgraphics(figure(f)), close(f); end
    catch
    end
end

[t,p,pL,ref,pTrue,pLTrue,reqCount,status] = extract_log(logger);
tf0=double(replanner.flight_reference_time_origin);
if ~isfinite(tf0), tf0=5.0; end
flightEnd=min(max(t),47.0);
flight=(t>=tf0 & t<flightEnd);

% Nominal reference from the SAME base_ref.func used by production.
nom=nan(size(ref));
fh=replanner.base_ref.func;
for k=find(flight).'
    tau=max(0,double(t(k))-tf0);
    xd=double(fh(tau)); xd=xd(:);
    if numel(xd)>=3 && all(isfinite(xd(1:3)))
        nom(k,:)=xd(1:3).';
    end
end

validCorr=flight & all(isfinite(ref),2) & all(isfinite(nom),2);
corrNorm=nan(size(t));
corrNorm(validCorr)=sqrt(sum((ref(validCorr,:)-nom(validCorr,:)).^2,2));

commitTime=double(replanner.last_replan_commit_time);
rejoinTime=NaN;
if isstruct(replanner.active_repaired_reference) && ...
        isfield(replanner.active_repaired_reference,'rejoin_time')
    rejoinTime=double(replanner.active_repaired_reference.rejoin_time);
end

%% Figure 5: one trajectory dashboard
figure(5); clf;
set(gcf,'Name','AF7R51 Trajectory / Avoidance Dashboard','NumberTitle','off');

vn=flight & all(isfinite(nom),2);
va=flight & all(isfinite(ref),2);
vu=flight & all(isfinite(p),2);
vl=flight & all(isfinite(pL),2);

subplot(2,2,1);
hold on; grid on; box on;
draw_obstacles(replanner);
if nnz(vn)>=2, plot3(nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.3,'DisplayName','Nominal reference'); end
if nnz(va)>=2, plot3(ref(va,1),ref(va,2),ref(va,3),'LineWidth',1.8,'DisplayName','Active/HLC reference'); end
if nnz(vu)>=2, plot3(p(vu,1),p(vu,2),p(vu,3),'LineWidth',1.2,'DisplayName','UAV estimated'); end
if nnz(vl)>=2, plot3(pL(vl,1),pL(vl,2),pL(vl,3),'LineWidth',1.2,'DisplayName','Payload estimated'); end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Flight 3-D: nominal / repaired / vehicle / payload');
set_readable_3d_limits([nom(vn,:);ref(va,:);p(vu,:);pL(vl,:)]);
view(3); legend('Location','best'); hold off;

subplot(2,2,2);
hold on; grid on; box on;
if nnz(vn)>=2, plot(nom(vn,1),nom(vn,3),'--','LineWidth',1.3,'DisplayName','Nominal'); end
if nnz(va)>=2, plot(ref(va,1),ref(va,3),'LineWidth',1.8,'DisplayName','Active/HLC ref'); end
if nnz(vu)>=2, plot(p(vu,1),p(vu,3),'LineWidth',1.1,'DisplayName','UAV'); end
if nnz(vl)>=2, plot(pL(vl,1),pL(vl,3),'LineWidth',1.1,'DisplayName','Payload'); end
plot_obstacle_xz(replanner);
xlabel('x [m]'); ylabel('z [m]'); title('XZ plane — actual detour geometry');
axis equal; axis tight; legend('Location','best'); hold off;

subplot(2,2,3);
hold on; grid on; box on;
if any(validCorr)
    plot(t(validCorr),corrNorm(validCorr),'LineWidth',1.5,'DisplayName','||active - nominal||');
end
if isfinite(commitTime), xline(commitTime,'--','commit','HandleVisibility','off'); end
if isfinite(rejoinTime), xline(rejoinTime,'--','rejoin','HandleVisibility','off'); end
xlabel('time [s]'); ylabel('reference correction [m]');
title('Avoidance correction magnitude');
legend('Location','best'); hold off;

subplot(2,2,4);
hold on; grid on; box on;
if any(vn), plot(t(vn),nom(vn,1),'--','LineWidth',1.2,'DisplayName','Nominal x'); end
if any(va), plot(t(va),ref(va,1),'LineWidth',1.5,'DisplayName','Active x'); end
if any(vu), plot(t(vu),p(vu,1),'LineWidth',1.0,'DisplayName','UAV x'); end
if any(vl), plot(t(vl),pL(vl,1),'LineWidth',1.0,'DisplayName','Payload x'); end
if isfinite(commitTime), xline(commitTime,'--','commit','HandleVisibility','off'); end
xlabel('time [s]'); ylabel('x [m]');
title('Primary travel axis');
legend('Location','best'); hold off;

sgtitle('AF7R51 — trajectory / avoidance (single authoritative dashboard)');

%% Figure 10: one timing dashboard from runtime_profile
P=runtime_profile;
K=min(double(P.k),numel(P.t));
tp=double(P.t(1:K));
phase=P.phase(1:K);
online=1e3*double(P.online_before_plant_s(1:K));
plant=1e3*double(P.simulation_plant_s(1:K));
logms=1e3*double(P.logger_s(1:K));
full=1e3*double(P.full_iteration_s(1:K));
mask=(phase=='f');

figure(10); clf;
set(gcf,'Name','AF7R51 Timing Dashboard','NumberTitle','off');

subplot(2,2,1);
hold on; grid on; box on;
plot(tp(mask),online(mask),'LineWidth',1.3,'DisplayName','Online before plant');
plot(tp(mask),full(mask),'LineWidth',1.2,'DisplayName','Full iteration incl. logger');
yline(25,'--','25 ms control period','HandleVisibility','off');
xlabel('time [s]'); ylabel('runtime [ms]');
title('Flight runtime — complete');
legend('Location','best'); hold off;

subplot(2,2,2);
hold on; grid on; box on;
zoomMask=mask & tp>=9.70 & tp<=10.40;
plot(tp(zoomMask),online(zoomMask),'LineWidth',1.4,'DisplayName','Online');
plot(tp(zoomMask),full(zoomMask),'LineWidth',1.2,'DisplayName','Full');
plot(tp(zoomMask),plant(zoomMask),'LineWidth',1.1,'DisplayName','Plant');
yline(25,'--','25 ms','HandleVisibility','off');
xlabel('time [s]'); ylabel('runtime [ms]');
title('Replan window 9.70--10.40 s');
legend('Location','best'); hold off;

subplot(2,2,3);
hold on; grid on; box on;
plot(tp(mask),plant(mask),'LineWidth',1.1,'DisplayName','Numerical plant');
plot(tp(mask),logms(mask),'LineWidth',1.1,'DisplayName','Logger');
xlabel('time [s]'); ylabel('runtime [ms]');
title('Simulation-only costs');
legend('Location','best'); hold off;

subplot(2,2,4);
hold on; grid on; box on;
res=full-online-plant-logms;
plot(tp(mask),res(mask),'LineWidth',1.1,'DisplayName','Full - online - plant - logger');
xlabel('time [s]'); ylabel('runtime [ms]');
title('Outer-loop residual');
legend('Location','best'); hold off;

sgtitle('AF7R51 — timing (single authoritative dashboard)');
drawnow;

% Summary
peakCorr=NaN;
pathRatio=NaN;
if any(validCorr)
    peakCorr=max(corrNorm(validCorr));
    LA=path_length(ref(validCorr,:));
    LN=path_length(nom(validCorr,:));
    if LN>0, pathRatio=LA/LN; end
end

OUT=struct('schema','AF7R51_POSTRUN_DASHBOARD_V1', ...
    'peak_correction_m',peakCorr, ...
    'path_length_ratio',pathRatio, ...
    'commit_time_s',commitTime, ...
    'rejoin_time_s',rejoinTime);
assignin('base','AF7R51_DASHBOARD_OUT',OUT);
end

function [t,p,pL,ref,pTrue,pLTrue,reqCount,status]=extract_log(logger)
D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
t=double(D.t(:)); n=numel(t);
p=nan(n,3);pL=nan(n,3);ref=nan(n,3);pTrue=nan(n,3);pLTrue=nan(n,3);
reqCount=nan(n,1);status=nan(n,1);
Ce=get_result_cells(A,'estimator');Cr=get_result_cells(A,'reference');Cp=get_result_cells(A,'plant');
for k=1:min(n,numel(Ce))
    st=get_member(Ce{k},'state');p(k,:)=vec3(get_member(st,'p'));pL(k,:)=vec3(get_member(st,'pL'));
end
for k=1:min(n,numel(Cr))
    st=get_member(Cr{k},'state');ref(k,:)=vec3(get_member(st,'xd'));
    reqCount(k)=scalar(get_member(st,'generator_request_count'));
    status(k)=scalar(get_member(st,'generator_last_status_code'));
end
for k=1:min(n,numel(Cp))
    st=get_member(Cp{k},'state');pTrue(k,:)=vec3(get_member(st,'p'));pLTrue(k,:)=vec3(get_member(st,'pL'));
end
end

function set_readable_3d_limits(X)
X=X(all(isfinite(X),2),:); if isempty(X),return;end
lo=min(X,[],1);hi=max(X,[],1);ctr=(lo+hi)/2;sp=hi-lo;major=max([sp(1),sp(3),1]);
if sp(2)<0.15*major
    lo(2)=ctr(2)-0.075*major;hi(2)=ctr(2)+0.075*major;
end
pad=0.05*max(hi-lo,1e-9);lo=lo-pad;hi=hi+pad;
xlim([lo(1) hi(1)]);ylim([lo(2) hi(2)]);zlim([lo(3) hi(3)]);
daspect([1 1 1]);
end

function draw_obstacles(replanner)
try
    P=replanner.obstacle_packet;
    if ~isstruct(P)||~isfield(P,'count')||double(P.count)<1,return;end
    [sx,sy,sz]=sphere(10);U=[sx(:)';sy(:)';sz(:)'];
    for i=1:double(P.count)
        c=double(P.C(:,i));R=reshape(double(P.R(:,i)),3,3);a=double(P.Araw(:,i));
        X=R*diag(a)*U+c;
        mesh(reshape(X(1,:),size(sx)),reshape(X(2,:),size(sy)),reshape(X(3,:),size(sz)), ...
            'FaceAlpha',0.04,'EdgeAlpha',0.18,'HandleVisibility','off');
    end
catch
end
end

function plot_obstacle_xz(replanner)
try
    P=replanner.obstacle_packet;
    for i=1:double(P.count)
        c=double(P.C(:,i));
        plot(c(1),c(3),'x','MarkerSize',8,'LineWidth',1.2,'HandleVisibility','off');
        text(c(1),c(3),sprintf(' O%.0f',double(P.ids(i))));
    end
catch
end
end

function L=path_length(X)
if size(X,1)<2,L=0;else,L=sum(sqrt(sum(diff(X,1,1).^2,2)));end
end

function v=vec3(x)
v=[NaN NaN NaN];
if isnumeric(x)&&numel(x)>=3
    x=double(x(:));if all(isfinite(x(1:3))),v=x(1:3).';end
end
end

function y=scalar(x)
y=NaN;if (isnumeric(x)||islogical(x))&&isscalar(x)&&isfinite(double(x)),y=double(x);end
end

function C=get_result_cells(A,name)
C={};if ~isstruct(A)||~isfield(A,name),return;end
S=A.(name);if ~isstruct(S)||~isfield(S,'result'),return;end
R=S.result;if iscell(R),C=R(:);else,C=num2cell(R(:));end
end

function x=get_member(S,name)
x=[];
try
    if isstruct(S)&&isfield(S,name),x=S.(name);
    elseif isobject(S)&&isprop(S,name),x=S.(name);end
catch
end
end
