function OUT = AF7R48_CURRENT_RUN_VISUAL_AND_PROVENANCE(logger,replanner)
% AF7R48_CURRENT_RUN_VISUAL_AND_PROVENANCE
% Read-only current-run closeout:
%  - re-opens Figures 2,3,4,9 exactly from logger
%  - repairs Figures 5,6 for nearly-planar 3-D data
%  - separates nominal vs active reference
%  - proves the committed reference provenance from current object state
%  - diagnoses the long detour / rejoin horizon
%  - summarizes the current 9.8--10.2 s online timing
%
% No controller/replanner/safety state is modified.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R48 | CURRENT-RUN VISUAL + NATIVE-PROVENANCE + REJOIN DIAGNOSIS\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | plottingOnly=1 | safetyLogicUnchanged=1 | runtimePathUnchanged=1\n');

assert(isa(replanner,'REPLANNING_SYSTEM'),'[AF7R48] replanner type mismatch.');

%% ------------------------------------------------------------------------
% 1) Re-open the requested original figures.
% -------------------------------------------------------------------------
figure(2);
logger.plot({{1,"p","er"},{1,"estimator.result.state.pL","e"}},"phase","f");

figure(3);
logger.plot({{1,"v1:2","er"},{1,"estimator.result.state.vL1:2",""}});

figure(4);
logger.plot({1,"controller.result.controllertime_total",""},"phase","f");

figure(9);
logger.plot({1,"controller.result.simulation_plant_total",""},"phase","f");

%% ------------------------------------------------------------------------
% 2) Extract current run.
% -------------------------------------------------------------------------
[t,p,pL,ref,pTrue,pLTrue,onlineMs,status,hardMs]=extract_log(logger);

tf0=double(replanner.flight_reference_time_origin);
assert(isfinite(tf0),'[AF7R48] flight_reference_time_origin invalid.');
flightEnd=min(max(t),47.0);
flight=(t>=tf0 & t<flightEnd);

% Exact nominal baseline: same time-origin rule as REPLANNING_SYSTEM.
fh=replanner.base_ref.func;
nom=nan(size(ref));
for k=find(flight).'
    tau=max(0,double(t(k))-tf0);
    xd0=double(fh(tau)); xd0=xd0(:);
    if numel(xd0)>=3 && all(isfinite(xd0(1:3)))
        nom(k,:)=xd0(1:3).';
    end
end

validCorr=flight & all(isfinite(ref),2) & all(isfinite(nom),2);
corr=nan(size(ref));
corr(validCorr,:)=ref(validCorr,:)-nom(validCorr,:);
corrNorm=sqrt(sum(corr.^2,2));

if any(validCorr)
    id=find(validCorr);
    [peakCorr,j]=max(corrNorm(validCorr));
    kPeak=id(j);
    maxAxis=max(abs(corr(validCorr,:)),[],1);
    LA=path_length(ref(validCorr,:));
    LN=path_length(nom(validCorr,:));
    ratio=LA/LN;
else
    peakCorr=NaN; kPeak=NaN; maxAxis=[NaN NaN NaN];
    LA=NaN; LN=NaN; ratio=NaN;
end

%% ------------------------------------------------------------------------
% 3) Provenance: this is native-generated, not a backup trajectory.
% -------------------------------------------------------------------------
S=replanner.get_avoidance_mode_status();
L=replanner.get_last_replan_status();

fprintf('\n[A. COMMITTED AVOIDANCE PROVENANCE]\n');
fprintf(' active reference mode/version            = %s / %.0f\n', ...
    char(replanner.active_reference_mode),double(replanner.active_reference_version));
fprintf(' reference-only commit count              = %.0f\n',double(S.reference_only_commit_count));
fprintf(' execution-aware commit count             = %.0f\n',double(S.execution_aware_commit_count));
fprintf(' compact native proposal count            = %.0f\n',double(replanner.compact_native_proposal_count));
fprintf(' locked backend name/code                 = %s / %.0f\n', ...
    char(replanner.generator_backend_locked_name),double(replanner.generator_backend_locked_code));

src='UNKNOWN';
backend='UNKNOWN';
if isstruct(replanner.active_repaired_reference)
    if isfield(replanner.active_repaired_reference,'source')
        src=char(replanner.active_repaired_reference.source);
    end
    if isfield(replanner.active_repaired_reference,'backend')
        backend=char(replanner.active_repaired_reference.backend);
    end
end
fprintf(' active repaired reference source         = %s\n',src);
fprintf(' active repaired reference backend        = %s\n',backend);

Gcert=false;
if isstruct(replanner.active_repaired_certificates) && ...
        isfield(replanner.active_repaired_certificates,'geometry')
    G=replanner.active_repaired_certificates.geometry;
    Gcert=isfield(G,'certified') && logical(G.certified) && ...
        isfield(G,'continuous_time') && logical(G.continuous_time) && ...
        isfield(G,'sampled_acceptance') && ~logical(G.sampled_acceptance);
end
Scert=isstruct(replanner.active_speed_certificate) && ...
    isfield(replanner.active_speed_certificate,'certified') && ...
    logical(replanner.active_speed_certificate.certified);

nativeCommitted = strcmpi(char(replanner.active_reference_mode),'REPAIRED') && ...
    double(S.reference_only_commit_count)>=1 && ...
    double(replanner.compact_native_proposal_count)>=1 && ...
    strcmpi(backend,'BEZIER_D15_C7') && Gcert && Scert;

fprintf(' continuous geometry certificate closed   = %d\n',Gcert);
fprintf(' speed certificate closed                 = %d\n',Scert);
fprintf(' native computed reference committed      = %d\n',nativeCommitted);
fprintf(' backup/fallback trajectory commit used   = 0  (REFERENCE_ONLY branch has no backup-substitution commit)\n');

if isstruct(L) && isfield(L,'reason')
    fprintf(' last replan reason                       = %s\n',char(L.reason));
end

%% ------------------------------------------------------------------------
% 4) Long-detour diagnosis.
% -------------------------------------------------------------------------
commitTime=double(replanner.last_replan_commit_time);
rejoinTime=NaN;
if isstruct(replanner.active_repaired_reference) && ...
        isfield(replanner.active_repaired_reference,'rejoin_time')
    rejoinTime=double(replanner.active_repaired_reference.rejoin_time);
end
tableEnd=double(replanner.nominal_table_tend);
tol=1e-10*max(1,max(abs([rejoinTime tableEnd])));
rejoinAtTableEnd=isfinite(rejoinTime)&&isfinite(tableEnd)&&abs(rejoinTime-tableEnd)<=tol;

fprintf('\n[B. DETOUR / REJOIN DIAGNOSIS]\n');
fprintf(' commit time [s]                          = %.15g\n',commitTime);
fprintf(' repaired rejoin time [s]                 = %.15g\n',rejoinTime);
fprintf(' nominal table end [s]                    = %.15g\n',tableEnd);
fprintf(' rejoin == nominal-table terminal bound   = %d\n',rejoinAtTableEnd);
fprintf(' commit -> rejoin duration [s]             = %.15g\n',rejoinTime-commitTime);
fprintf(' peak ||active-nominal|| [m]              = %.15g\n',peakCorr);
if isfinite(kPeak), fprintf(' peak correction time [s]                 = %.15g\n',t(kPeak)); end
fprintf(' max |dx| / |dy| / |dz| [m]              = %.15g / %.15g / %.15g\n', ...
    maxAxis(1),maxAxis(2),maxAxis(3));
fprintf(' nominal / active path length [m]         = %.15g / %.15g\n',LN,LA);
fprintf(' active / nominal path-length ratio       = %.15g\n',ratio);
fprintf(' first component correction [dx dy dz] m  = %.15g %.15g %.15g\n', ...
    double(replanner.bezier_shadow_last_dx), ...
    double(replanner.bezier_shadow_last_dy), ...
    double(replanner.bezier_shadow_last_dz));
fprintf(' max Bezier control-point norm [m]        = %.15g\n', ...
    double(replanner.bezier_shadow_last_displacement_m));

if rejoinAtTableEnd
    fprintf([' diagnosis                               = CURRENT SYNTHESIS REJOINS AT THE SOURCE-DOMAIN END; ' ...
             'minimum-snap is not a shortest-path objective.\n']);
end

%% ------------------------------------------------------------------------
% 5) Current request-window timing: distinguish cold run from current run.
% -------------------------------------------------------------------------
w=t>=9.80 & t<=10.20;
wv=onlineMs(w & isfinite(onlineMs));
fprintf('\n[C. CURRENT 9.80--10.20 s TIMING]\n');
if isempty(wv)
    fprintf(' online timing unavailable.\n');
else
    fprintf(' online min/med/max [ms]                  = %.6g / %.6g / %.6g\n', ...
        min(wv),median(wv),max(wv));
    fprintf(' online >=20 ms cycles in this window     = %d\n',nnz(wv>=20));
end

req=w & isfinite(status) & status~=0;
fprintf(' request records in this window           = %d\n',nnz(req));
if any(req)
    kr=find(req,1,'first');
    fprintf(' first request t/status/online/hard        = %.6g / %.0f / %.6g / %.6g ms\n', ...
        t(kr),status(kr),onlineMs(kr),hardMs(kr));
end

%% ------------------------------------------------------------------------
% 6) Figure 5: all-phases readable 3-D.
% The trajectory is almost planar in y.  Do NOT use the old unconstrained
% axis-equal autoscale, which can visually collapse a near-zero-width axis.
% -------------------------------------------------------------------------
phys=[p;pL;pTrue;pLTrue];
phys=phys(all(isfinite(phys),2),:);
lo=min(phys,[],1); hi=max(phys,[],1);
ctr=(lo+hi)/2; span=hi-lo;
major=max([span(1),span(3),1]);
% Give the nearly planar y-axis display thickness without changing data.
if span(2)<0.15*major
    lo(2)=ctr(2)-0.075*major;
    hi(2)=ctr(2)+0.075*major;
end
pad=0.05*max(hi-lo,1e-9);
lo=lo-pad; hi=hi+pad;

figure(5); clf;
subplot(1,2,1);
hold on; grid on; box on;
vp=all(isfinite(p),2); vl=all(isfinite(pL),2);
vt=all(isfinite(pTrue),2); vlt=all(isfinite(pLTrue),2);
vr=all(isfinite(ref),2);
if nnz(vp)>=2, plot3(p(vp,1),p(vp,2),p(vp,3),'LineWidth',1.4,'DisplayName','UAV estimated'); end
if nnz(vl)>=2, plot3(pL(vl,1),pL(vl,2),pL(vl,3),'LineWidth',1.4,'DisplayName','Payload estimated'); end
if nnz(vt)>=2, plot3(pTrue(vt,1),pTrue(vt,2),pTrue(vt,3),':','LineWidth',1.0,'DisplayName','UAV plant truth'); end
if nnz(vlt)>=2, plot3(pLTrue(vlt,1),pLTrue(vlt,2),pLTrue(vlt,3),':','LineWidth',1.0,'DisplayName','Payload plant truth'); end
if nnz(vr)>=2, plot3(ref(vr,1),ref(vr,2),ref(vr,3),'--','LineWidth',1.4,'DisplayName','Active reference'); end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('All phases — readable 3-D');
xlim(lo([1 1])+[0 hi(1)-lo(1)]);
ylim(lo([2 2])+[0 hi(2)-lo(2)]);
zlim(lo([3 3])+[0 hi(3)-lo(3)]);
daspect([1 1 1]); view(3); legend('Location','best'); hold off;

subplot(1,2,2);
hold on; grid on; box on;
for a=1:3
    q=isfinite(ref(:,a));
    if any(q)
        nm={'x','y','z'};
        plot(t(q),ref(q,a),'LineWidth',1.1,'DisplayName',['active ref ' nm{a}]);
    end
end
xline(tf0,'--','Flight start','HandleVisibility','off');
if isfinite(commitTime), xline(commitTime,'--','commit','HandleVisibility','off'); end
if isfinite(rejoinTime), xline(rejoinTime,'--','rejoin','HandleVisibility','off'); end
xlabel('time [s]'); ylabel('position [m]');
title('Active reference history'); legend('Location','best'); hold off;

%% ------------------------------------------------------------------------
% 7) Figure 6: explicit nominal / avoidance views.
% Use subplot rather than tiledlayout for robust MATLAB rendering.
% -------------------------------------------------------------------------
figure(6); clf;

subplot(2,2,1);
hold on; grid on; box on;
draw_raw_obstacles(replanner);
vn=flight & all(isfinite(nom),2);
va=flight & all(isfinite(ref),2);
ve=flight & all(isfinite(p),2);
vpl=flight & all(isfinite(pL),2);
if nnz(vn)>=2, plot3(nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.2,'DisplayName','Nominal reference'); end
if nnz(va)>=2, plot3(ref(va,1),ref(va,2),ref(va,3),'LineWidth',1.8,'DisplayName','Active/HLC reference'); end
if nnz(ve)>=2, plot3(p(ve,1),p(ve,2),p(ve,3),'LineWidth',1.2,'DisplayName','UAV estimated'); end
if nnz(vpl)>=2, plot3(pL(vpl,1),pL(vpl,2),pL(vpl,3),'LineWidth',1.2,'DisplayName','Payload estimated'); end
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
title('Flight 3-D — nominal vs avoidance');
set_readable_3d_limits([nom(vn,:);ref(va,:);p(ve,:);pL(vpl,:)]);
view(3); legend('Location','best'); hold off;

subplot(2,2,2);
hold on; grid on; box on;
if nnz(vn)>=2, plot(nom(vn,1),nom(vn,2),'--','LineWidth',1.2,'DisplayName','Nominal'); end
if nnz(va)>=2, plot(ref(va,1),ref(va,2),'LineWidth',1.7,'DisplayName','Active ref'); end
if nnz(ve)>=2, plot(p(ve,1),p(ve,2),'LineWidth',1.1,'DisplayName','UAV'); end
plot_obstacle_centres_xy(replanner);
xlabel('x [m]');ylabel('y [m]');title('XY top view');
axis equal; axis tight; legend('Location','best'); hold off;

subplot(2,2,3);
hold on; grid on; box on;
if nnz(vn)>=2, plot(nom(vn,1),nom(vn,3),'--','LineWidth',1.2,'DisplayName','Nominal'); end
if nnz(va)>=2, plot(ref(va,1),ref(va,3),'LineWidth',1.7,'DisplayName','Active ref'); end
if nnz(ve)>=2, plot(p(ve,1),p(ve,3),'LineWidth',1.1,'DisplayName','UAV'); end
if nnz(vpl)>=2, plot(pL(vpl,1),pL(vpl,3),'LineWidth',1.1,'DisplayName','Payload'); end
xlabel('x [m]');ylabel('z [m]');title('XZ side view');
axis equal; axis tight; legend('Location','best'); hold off;

subplot(2,2,4);
hold on; grid on; box on;
if any(validCorr)
    plot(t(validCorr),corrNorm(validCorr),'LineWidth',1.5,'DisplayName','||active - nominal||');
end
if isfinite(commitTime), xline(commitTime,'--','commit','HandleVisibility','off'); end
if isfinite(rejoinTime), xline(rejoinTime,'--','rejoin','HandleVisibility','off'); end
xlabel('time [s]');ylabel('reference correction [m]');
title('Avoidance correction magnitude'); legend('Location','best'); hold off;

sgtitle('Figure 6 — actual native avoidance and detour');

drawnow;

fprintf('\n[D. DISPLAY]\n');
fprintf(' Figures 2 / 3 / 4 / 5 / 6 / 9 reopened = 1\n');
fprintf(' Figure 5 near-planar autoscale repaired  = 1\n');
fprintf(' Figure 6 nominal/active separated        = 1\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R48_CURRENT_RUN_VISUAL_AND_PROVENANCE_V1';
OUT.native_computed_reference_committed=nativeCommitted;
OUT.peak_correction_m=peakCorr;
OUT.max_axis_correction_m=maxAxis;
OUT.nominal_path_length_m=LN;
OUT.active_path_length_m=LA;
OUT.path_length_ratio=ratio;
OUT.commit_time_s=commitTime;
OUT.rejoin_time_s=rejoinTime;
OUT.nominal_table_end_s=tableEnd;
OUT.rejoin_at_nominal_table_end=rejoinAtTableEnd;
OUT.window_online_ge20_count=nnz(wv>=20);
assignin('base','AF7R48_CURRENT_RUN_VISUAL_AND_PROVENANCE_OUT',OUT);
end


function [t,p,pL,ref,pTrue,pLTrue,onlineMs,status,hardMs]=extract_log(logger)
D=logger.Data; A=D.agent; if numel(A)>1,A=A(1);end
if isfield(D,'t')&&~isempty(D.t), t=double(D.t(:));
else, C=get_result_cells(A,'reference'); t=(0:numel(C)-1)'*0.025; end
n=numel(t);
p=nan(n,3);pL=nan(n,3);ref=nan(n,3);pTrue=nan(n,3);pLTrue=nan(n,3);
onlineMs=nan(n,1);status=nan(n,1);hardMs=nan(n,1);
Ce=get_result_cells(A,'estimator'); Cr=get_result_cells(A,'reference');
Cp=get_result_cells(A,'plant'); Cc=get_result_cells(A,'controller');
for k=1:min(n,numel(Ce))
    st=get_member(Ce{k},'state'); p(k,:)=vec3(get_member(st,'p')); pL(k,:)=vec3(get_member(st,'pL'));
end
for k=1:min(n,numel(Cr))
    st=get_member(Cr{k},'state'); ref(k,:)=vec3(get_member(st,'xd'));
    status(k)=scalar(get_member(st,'generator_last_status_code'));
    hardMs(k)=scalar(get_member(st,'generator_last_cycle_critical_ms'));
end
for k=1:min(n,numel(Cp))
    st=get_member(Cp{k},'state'); pTrue(k,:)=vec3(get_member(st,'p')); pLTrue(k,:)=vec3(get_member(st,'pL'));
end
for k=1:min(n,numel(Cc))
    onlineMs(k)=1e3*scalar(get_member(Cc{k},'online_compute_total'));
end
end

function set_readable_3d_limits(X)
X=X(all(isfinite(X),2),:);
if isempty(X),return;end
lo=min(X,[],1);hi=max(X,[],1);ctr=(lo+hi)/2;sp=hi-lo;major=max([sp(1),sp(3),1]);
if sp(2)<0.15*major
    lo(2)=ctr(2)-0.075*major;hi(2)=ctr(2)+0.075*major;
end
pad=0.05*max(hi-lo,1e-9);lo=lo-pad;hi=hi+pad;
xlim([lo(1),hi(1)]);ylim([lo(2),hi(2)]);zlim([lo(3),hi(3)]);
daspect([1 1 1]);
end

function draw_raw_obstacles(replanner)
try
    P=replanner.obstacle_packet;
    if ~isstruct(P)||~isfield(P,'count')||double(P.count)<1,return;end
    [sx,sy,sz]=sphere(10); U=[sx(:)';sy(:)';sz(:)'];
    for i=1:double(P.count)
        c=double(P.C(:,i));R=reshape(double(P.R(:,i)),3,3);a=double(P.Araw(:,i));
        X=R*diag(a)*U+c;
        mesh(reshape(X(1,:),size(sx)),reshape(X(2,:),size(sy)),reshape(X(3,:),size(sz)), ...
            'FaceAlpha',0.04,'EdgeAlpha',0.20,'HandleVisibility','off');
    end
catch
end
end

function plot_obstacle_centres_xy(replanner)
try
    P=replanner.obstacle_packet;
    for i=1:double(P.count)
        c=double(P.C(:,i));plot(c(1),c(2),'x','MarkerSize',8,'LineWidth',1.2,'HandleVisibility','off');
        text(c(1),c(2),sprintf(' O%.0f',double(P.ids(i))));
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
