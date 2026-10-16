function OUT = AF7R47_TRAJECTORY_VISUAL_CLOSEOUT(logger,replanner)
% AF7R47_TRAJECTORY_VISUAL_CLOSEOUT
% Read-only visualization + trajectory audit for the CURRENT headless run.
%
% It does not modify the replanner, controller, safety certificate, logger,
% or any runtime timing path.
%
% Figure 5:
%   Left  = physical-scale 3-D all-phases view.  Reference samples outside
%           the physical envelope are clipped ONLY for display and counted.
%   Right = the full x/y/z active-reference history, so an outlier cannot be
%           hidden by the display clipping.
%
% Figure 6:
%   Flight-only detailed avoidance view:
%     3-D path + raw obstacle ellipsoids
%     XY top view
%     XZ side view
%     ||active reference - nominal reference|| versus time
%
% The function also prints exact extrema, nominal-vs-active detour, path
% length ratio, commit/rejoin times, and the source of Figure-5 autoscaling.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R47 | TRAJECTORY VISUAL / DETOUR / FIGURE-5 AUTOSCALE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | plottingOnly=1 | safetyLogicUnchanged=1 | runtimePathUnchanged=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R47] replanner must be REPLANNING_SYSTEM.');

[t,p,pL,ref,pTrue,pLTrue] = extract_log(logger);

tf0 = double(replanner.flight_reference_time_origin);
if ~(isscalar(tf0) && isfinite(tf0))
    tf0 = 5.0; % visualization fallback for the current headless scenario only
end

% Current run_flight_headless flight phase is [5,47).  Keep this a
% visualization-only convention and never feed it back into control/safety.
flightEnd = min(max(t),47.0);
flight = (t >= tf0) & (t < flightEnd);

% -------------------------------------------------------------------------
% Nominal reference, evaluated directly from the same base_ref.func.
% -------------------------------------------------------------------------
nom = nan(size(ref));
assert(isa(replanner.base_ref,'TIME_VARYING_REFERENCE') && ...
    isprop(replanner.base_ref,'func') && ...
    isa(replanner.base_ref.func,'function_handle'), ...
    '[AF7R47] base_ref.func unavailable.');

fh = replanner.base_ref.func;
idxF = find(flight);
for ii = idxF(:).'
    tau = max(0,double(t(ii))-tf0);
    xd0 = double(fh(tau));
    xd0 = xd0(:);
    if numel(xd0) >= 3 && all(isfinite(xd0(1:3)))
        nom(ii,:) = xd0(1:3).';
    end
end

% -------------------------------------------------------------------------
% Exact numeric audit.
% -------------------------------------------------------------------------
fprintf('\n[A. ACTIVE REFERENCE / ESTIMATE EXTREMA]\n');
print_extrema('UAV estimated',t,p);
print_extrema('Payload estimated',t,pL);
print_extrema('Active reference xd(1:3)',t,ref);
print_extrema('UAV plant truth',t,pTrue);
print_extrema('Payload plant truth',t,pLTrue);

validCorr = flight & all(isfinite(ref),2) & all(isfinite(nom),2);
corr = nan(size(ref));
corr(validCorr,:) = ref(validCorr,:) - nom(validCorr,:);
corrNorm = sqrt(sum(corr.^2,2));

if any(validCorr)
    [peakCorr,krel] = max(corrNorm(validCorr));
    ids = find(validCorr);
    kpeak = ids(krel);
    maxAxis = max(abs(corr(validCorr,:)),[],1);

    pathActive = path_length(ref(validCorr,:));
    pathNominal = path_length(nom(validCorr,:));
    if pathNominal > 0
        pathRatio = pathActive/pathNominal;
    else
        pathRatio = NaN;
    end

    fprintf('\n[B. NOMINAL -> ACTIVE AVOIDANCE CHANGE]\n');
    fprintf(' peak ||active-nominal|| [m]             = %.15g\n',peakCorr);
    fprintf(' peak correction time [s]                = %.15g\n',t(kpeak));
    fprintf(' max |dx| / |dy| / |dz| [m]             = %.15g / %.15g / %.15g\n', ...
        maxAxis(1),maxAxis(2),maxAxis(3));
    fprintf(' nominal Flight path length [m]          = %.15g\n',pathNominal);
    fprintf(' active  Flight path length [m]          = %.15g\n',pathActive);
    fprintf(' active/nominal path-length ratio        = %.15g\n',pathRatio);
else
    peakCorr=NaN; kpeak=NaN; maxAxis=[NaN NaN NaN];
    pathActive=NaN; pathNominal=NaN; pathRatio=NaN;
    fprintf('\n[B. NOMINAL -> ACTIVE AVOIDANCE CHANGE]\n');
    fprintf(' no overlapping valid nominal/active Flight samples.\n');
end

commitTime = double(replanner.last_replan_commit_time);
rejoinTime = NaN;
if isstruct(replanner.active_repaired_reference) && ...
        isfield(replanner.active_repaired_reference,'rejoin_time')
    rejoinTime = double(replanner.active_repaired_reference.rejoin_time);
end

fprintf('\n[C. CURRENT REFERENCE AUTHORITY]\n');
fprintf(' active reference mode/version           = %s / %.0f\n', ...
    char(replanner.active_reference_mode),double(replanner.active_reference_version));
fprintf(' last successful commit time [s]         = %.15g\n',commitTime);
fprintf(' current repaired rejoin time [s]        = %.15g\n',rejoinTime);
fprintf(' reference-only commit count             = %.0f\n', ...
    double(replanner.reference_only_commit_count));
fprintf(' last proposal displacement norm [m]     = %.15g\n', ...
    double(replanner.bezier_shadow_last_displacement_m));
fprintf(' last proposal dx/dy/dz [m]              = %.15g / %.15g / %.15g\n', ...
    double(replanner.bezier_shadow_last_dx), ...
    double(replanner.bezier_shadow_last_dy), ...
    double(replanner.bezier_shadow_last_dz));

% -------------------------------------------------------------------------
% Figure-5 autoscale diagnosis.
% Existing run_flight_headless plots every finite active-reference sample,
% then calls axis equal.  One finite reference outlier can therefore make
% the physical trajectory visually collapse.
% -------------------------------------------------------------------------
phys = [p; pL; pTrue; pLTrue];
phys = phys(all(isfinite(phys),2),:);

if isempty(phys)
    error('[AF7R47] no finite physical trajectory samples.');
end

lo = min(phys,[],1);
hi = max(phys,[],1);
span = hi-lo;
pad = 0.08*max(span,eps);
for a=1:3
    if ~(pad(a)>0) || ~isfinite(pad(a))
        pad(a)=1;
    end
end
viewLo = lo-pad;
viewHi = hi+pad;

refFinite = all(isfinite(ref),2);
outside = false(size(t));
outside(refFinite) = any(ref(refFinite,:) < viewLo | ref(refFinite,:) > viewHi,2);
outsideCount = nnz(outside);

fprintf('\n[D. FIGURE-5 AUTOSCALE DIAGNOSIS]\n');
fprintf(' physical display x range [m]            = [%.15g, %.15g]\n',viewLo(1),viewHi(1));
fprintf(' physical display y range [m]            = [%.15g, %.15g]\n',viewLo(2),viewHi(2));
fprintf(' physical display z range [m]            = [%.15g, %.15g]\n',viewLo(3),viewHi(3));
fprintf(' finite active-reference samples outside = %d\n',outsideCount);
if outsideCount>0
    ko=find(outside,1,'first');
    fprintf(' first outside sample time [s]           = %.15g\n',t(ko));
    fprintf(' first outside active reference [m]      = [%.15g %.15g %.15g]\n', ...
        ref(ko,1),ref(ko,2),ref(ko,3));
end

% -------------------------------------------------------------------------
% Figure 5 - readable all-phases view + full reference history.
% -------------------------------------------------------------------------
figure(5); clf;
tl=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
hold on; grid on; box on;
vp=all(isfinite(p),2);
vl=all(isfinite(pL),2);
vt=all(isfinite(pTrue),2);
vlt=all(isfinite(pLTrue),2);

if nnz(vp)>=2
    plot3(p(vp,1),p(vp,2),p(vp,3),'LineWidth',1.5, ...
        'DisplayName','UAV estimated');
end
if nnz(vl)>=2
    plot3(pL(vl,1),pL(vl,2),pL(vl,3),'LineWidth',1.5, ...
        'DisplayName','Payload estimated');
end
if nnz(vt)>=2
    plot3(pTrue(vt,1),pTrue(vt,2),pTrue(vt,3),':','LineWidth',1.1, ...
        'DisplayName','UAV plant truth');
end
if nnz(vlt)>=2
    plot3(pLTrue(vlt,1),pLTrue(vlt,2),pLTrue(vlt,3),':','LineWidth',1.1, ...
        'DisplayName','Payload plant truth');
end

% Reference is clipped only in this physical-scale panel.
vrDisplay = refFinite & ~outside;
if nnz(vrDisplay)>=2
    plot3(ref(vrDisplay,1),ref(vrDisplay,2),ref(vrDisplay,3),'--', ...
        'LineWidth',1.4,'DisplayName','Active reference (in physical view)');
end

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('All phases — physical-scale view | ref outside=%d',outsideCount));
xlim(viewLo([1 1])+[0 viewHi(1)-viewLo(1)]);
ylim(viewLo([2 2])+[0 viewHi(2)-viewLo(2)]);
zlim(viewLo([3 3])+[0 viewHi(3)-viewLo(3)]);
axis equal; view(3);
legend('Location','best');
hold off;

nexttile;
hold on; grid on; box on;
for a=1:3
    vv=isfinite(ref(:,a));
    if any(vv)
        plot(t(vv),ref(vv,a),'LineWidth',1.1, ...
            'DisplayName',sprintf('active ref %s',char('x'+a-1)));
    end
end
xline(tf0,'--','Flight start','HandleVisibility','off');
if isfinite(commitTime)
    xline(commitTime,'--','commit','HandleVisibility','off');
end
if isfinite(rejoinTime)
    xline(rejoinTime,'--','rejoin','HandleVisibility','off');
end
xlabel('time [s]'); ylabel('position [m]');
title('Full active-reference history — no clipping');
legend('Location','best');
hold off;

title(tl,'Figure 5 repaired: physical path + reference-extrema visibility');

% -------------------------------------------------------------------------
% Figure 6 - Flight avoidance geometry / nominal comparison.
% -------------------------------------------------------------------------
figure(6); clf;
tl6=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

validFlight = flight & all(isfinite(p),2) & all(isfinite(pL),2);
vr = flight & all(isfinite(ref),2);
vn = flight & all(isfinite(nom),2);

nexttile;
hold on; grid on; box on;
draw_raw_obstacles(replanner);
if nnz(vn)>=2
    plot3(nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.2, ...
        'DisplayName','Nominal reference');
end
if nnz(vr)>=2
    plot3(ref(vr,1),ref(vr,2),ref(vr,3),'LineWidth',1.7, ...
        'DisplayName','Active/HLC reference');
end
if nnz(validFlight)>=2
    plot3(p(validFlight,1),p(validFlight,2),p(validFlight,3),'LineWidth',1.3, ...
        'DisplayName','UAV estimated');
    plot3(pL(validFlight,1),pL(validFlight,2),pL(validFlight,3),'LineWidth',1.3, ...
        'DisplayName','Payload estimated');
end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Flight 3-D: nominal vs avoidance');
axis equal; view(3); legend('Location','best'); hold off;

nexttile;
hold on; grid on; box on;
if nnz(vn)>=2, plot(nom(vn,1),nom(vn,2),'--','LineWidth',1.2,'DisplayName','Nominal'); end
if nnz(vr)>=2, plot(ref(vr,1),ref(vr,2),'LineWidth',1.7,'DisplayName','Active ref'); end
if nnz(validFlight)>=2
    plot(p(validFlight,1),p(validFlight,2),'LineWidth',1.2,'DisplayName','UAV');
    plot(pL(validFlight,1),pL(validFlight,2),'LineWidth',1.2,'DisplayName','Payload');
end
plot_obstacle_centres_xy(replanner);
xlabel('x [m]'); ylabel('y [m]');
title('XY top view'); axis equal; legend('Location','best'); hold off;

nexttile;
hold on; grid on; box on;
if nnz(vn)>=2, plot(nom(vn,1),nom(vn,3),'--','LineWidth',1.2,'DisplayName','Nominal'); end
if nnz(vr)>=2, plot(ref(vr,1),ref(vr,3),'LineWidth',1.7,'DisplayName','Active ref'); end
if nnz(validFlight)>=2
    plot(p(validFlight,1),p(validFlight,3),'LineWidth',1.2,'DisplayName','UAV');
    plot(pL(validFlight,1),pL(validFlight,3),'LineWidth',1.2,'DisplayName','Payload');
end
xlabel('x [m]'); ylabel('z [m]');
title('XZ side view'); axis equal; legend('Location','best'); hold off;

nexttile;
hold on; grid on; box on;
if any(validCorr)
    plot(t(validCorr),corrNorm(validCorr),'LineWidth',1.5, ...
        'DisplayName','||active - nominal||');
end
if isfinite(commitTime), xline(commitTime,'--','commit','HandleVisibility','off'); end
if isfinite(rejoinTime), xline(rejoinTime,'--','rejoin','HandleVisibility','off'); end
xlabel('time [s]'); ylabel('reference correction [m]');
title('Avoidance correction magnitude');
legend('Location','best'); hold off;

title(tl6,'Figure 6 repaired: actual avoidance geometry and detour');

fprintf('\n[E. INTERPRETATION]\n');
fprintf(' Figure 5 old green/reference curve        = ACTIVE reference, not a separate nominal baseline.\n');
fprintf(' Figure 6 now separates nominal/active     = 1\n');
fprintf(' plotting changes affect safety/runtime    = 0\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R47_TRAJECTORY_VISUAL_CLOSEOUT_V1';
OUT.peak_correction_m=peakCorr;
OUT.peak_correction_time_s=ternary_index(t,kpeak);
OUT.max_axis_correction_m=maxAxis;
OUT.nominal_path_length_m=pathNominal;
OUT.active_path_length_m=pathActive;
OUT.path_length_ratio=pathRatio;
OUT.reference_samples_outside_physical_view=outsideCount;
OUT.commit_time_s=commitTime;
OUT.rejoin_time_s=rejoinTime;
OUT.active_reference_mode=char(replanner.active_reference_mode);
OUT.reference_only_commit_count=double(replanner.reference_only_commit_count);

assignin('base','AF7R47_TRAJECTORY_VISUAL_CLOSEOUT_OUT',OUT);
end


function [t,p,pL,ref,pTrue,pLTrue]=extract_log(logger)
D=logger.Data;
A=D.agent;
if numel(A)>1, A=A(1); end

if isfield(D,'t') && ~isempty(D.t)
    t=double(D.t(:));
else
    Cref=get_result_cells(A,'reference');
    t=(0:numel(Cref)-1)'*0.025;
end

n=numel(t);
p=nan(n,3); pL=nan(n,3); ref=nan(n,3);
pTrue=nan(n,3); pLTrue=nan(n,3);

Cest=get_result_cells(A,'estimator');
Cref=get_result_cells(A,'reference');
Cplant=get_result_cells(A,'plant');

for k=1:min(n,numel(Cest))
    st=get_member(Cest{k},'state');
    p(k,:)=vec3(get_member(st,'p'));
    pL(k,:)=vec3(get_member(st,'pL'));
end
for k=1:min(n,numel(Cref))
    st=get_member(Cref{k},'state');
    ref(k,:)=vec3(get_member(st,'xd'));
end
for k=1:min(n,numel(Cplant))
    st=get_member(Cplant{k},'state');
    pTrue(k,:)=vec3(get_member(st,'p'));
    pLTrue(k,:)=vec3(get_member(st,'pL'));
end
end


function print_extrema(name,t,X)
fprintf(' %-37s',name);
v=all(isfinite(X),2);
if ~any(v)
    fprintf(' = no finite samples\n');
    return;
end
Y=X(v,:);
tt=t(v);
mn=min(Y,[],1); mx=max(Y,[],1);
[ma,ix]=max(abs(Y(:)));
[r,c]=ind2sub(size(Y),ix);
fprintf(' min=[%.6g %.6g %.6g] max=[%.6g %.6g %.6g] |max|=%.6g at t=%.6g axis=%d\n', ...
    mn(1),mn(2),mn(3),mx(1),mx(2),mx(3),ma,tt(r),c);
end


function L=path_length(X)
if size(X,1)<2
    L=0;
else
    L=sum(sqrt(sum(diff(X,1,1).^2,2)));
end
end


function draw_raw_obstacles(replanner)
try
    P=replanner.obstacle_packet;
    if ~isstruct(P) || ~isfield(P,'count') || double(P.count)<1
        return;
    end
    [sx,sy,sz]=sphere(12);
    U=[sx(:)';sy(:)';sz(:)'];
    for i=1:double(P.count)
        c=double(P.C(:,i));
        R=reshape(double(P.R(:,i)),3,3);
        a=double(P.Araw(:,i));
        X=R*diag(a)*U+c;
        X1=reshape(X(1,:),size(sx));
        X2=reshape(X(2,:),size(sy));
        X3=reshape(X(3,:),size(sz));
        mesh(X1,X2,X3,'FaceAlpha',0.03,'EdgeAlpha',0.18, ...
            'HandleVisibility','off');
    end
catch
end
end


function plot_obstacle_centres_xy(replanner)
try
    P=replanner.obstacle_packet;
    if ~isstruct(P) || ~isfield(P,'count') || double(P.count)<1
        return;
    end
    for i=1:double(P.count)
        c=double(P.C(:,i));
        plot(c(1),c(2),'x','MarkerSize',8,'LineWidth',1.2, ...
            'HandleVisibility','off');
        text(c(1),c(2),sprintf(' O%.0f',double(P.ids(i))));
    end
catch
end
end


function v=vec3(x)
v=[NaN NaN NaN];
if isnumeric(x) && numel(x)>=3
    x=double(x(:));
    if all(isfinite(x(1:3)))
        v=x(1:3).';
    end
end
end


function C=get_result_cells(A,name)
C={};
if ~isstruct(A) || ~isfield(A,name), return; end
S=A.(name);
if ~isstruct(S) || ~isfield(S,'result'), return; end
R=S.result;
if iscell(R), C=R(:); else, C=num2cell(R(:)); end
end


function x=get_member(S,name)
x=[];
try
    if isstruct(S) && isfield(S,name)
        x=S.(name);
    elseif isobject(S) && isprop(S,name)
        x=S.(name);
    end
catch
    x=[];
end
end


function y=ternary_index(t,k)
if isnumeric(k) && isscalar(k) && isfinite(k) && k>=1 && k<=numel(t)
    y=double(t(k));
else
    y=NaN;
end
end
