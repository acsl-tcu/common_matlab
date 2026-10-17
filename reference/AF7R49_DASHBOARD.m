function OUT = AF7R49_DASHBOARD(logger,replanner,runtime_profile)
% Unified post-run display.  It deliberately does not use logger.plot(),
% because logger.plot may create its own figure and leave the requested
% figure number empty.  This helper closes the now-redundant 2/3/4/6/9
% windows and draws all useful current-run information directly.

for f=[2 3 4 6 9]
    try, close(f); catch, end
end

[t,p,pL,ref]=extract_ref(logger);
tf0=double(replanner.flight_reference_time_origin);
flightEnd=min(max(t),47.0); flight=t>=tf0 & t<flightEnd;
nom=nan(size(ref)); fh=replanner.base_ref.func;
for k=find(flight).'
    x=double(fh(max(0,t(k)-tf0))); x=x(:);
    if numel(x)>=3&&all(isfinite(x(1:3))),nom(k,:)=x(1:3).';end
end
vc=flight&all(isfinite(ref),2)&all(isfinite(nom),2);
d=nan(size(ref));d(vc,:)=ref(vc,:)-nom(vc,:);dn=sqrt(sum(d.^2,2));

% Figure 5: one reliable trajectory dashboard.
figure(5);clf;
subplot(2,2,1);hold on;grid on;box on;
draw_obs(replanner);
vn=flight&all(isfinite(nom),2);va=flight&all(isfinite(ref),2);
vu=flight&all(isfinite(p),2);vl=flight&all(isfinite(pL),2);
if nnz(vn)>1,plot3(nom(vn,1),nom(vn,2),nom(vn,3),'--','LineWidth',1.2,'DisplayName','Nominal reference');end
if nnz(va)>1,plot3(ref(va,1),ref(va,2),ref(va,3),'LineWidth',1.8,'DisplayName','Active reference');end
if nnz(vu)>1,plot3(p(vu,1),p(vu,2),p(vu,3),'LineWidth',1.2,'DisplayName','UAV estimated');end
if nnz(vl)>1,plot3(pL(vl,1),pL(vl,2),pL(vl,3),'LineWidth',1.2,'DisplayName','Payload estimated');end
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');title('Flight 3-D');
readable3d([nom(vn,:);ref(va,:);p(vu,:);pL(vl,:)]);view(3);legend('Location','best');hold off;

subplot(2,2,2);hold on;grid on;box on;
if nnz(vn)>1,plot(nom(vn,1),nom(vn,3),'--','LineWidth',1.2,'DisplayName','Nominal');end
if nnz(va)>1,plot(ref(va,1),ref(va,3),'LineWidth',1.8,'DisplayName','Active ref');end
if nnz(vu)>1,plot(p(vu,1),p(vu,3),'LineWidth',1.1,'DisplayName','UAV');end
if nnz(vl)>1,plot(pL(vl,1),pL(vl,3),'LineWidth',1.1,'DisplayName','Payload');end
xlabel('x [m]');ylabel('z [m]');title('XZ side view (main motion plane)');axis equal;axis tight;legend('Location','best');hold off;

subplot(2,2,3);hold on;grid on;box on;
if any(vc),plot(t(vc),dn(vc),'LineWidth',1.5,'DisplayName','||active-nominal||');end
ct=double(replanner.last_replan_commit_time);rt=NaN;
if isstruct(replanner.active_repaired_reference)&&isfield(replanner.active_repaired_reference,'rejoin_time'),rt=double(replanner.active_repaired_reference.rejoin_time);end
if isfinite(ct),xline(ct,'--','commit','HandleVisibility','off');end
if isfinite(rt),xline(rt,'--','rejoin','HandleVisibility','off');end
xlabel('time [s]');ylabel('correction [m]');title('Avoidance correction magnitude');legend('Location','best');hold off;

subplot(2,2,4);hold on;grid on;box on;
if nnz(vn)>1,plot(t(vn),nom(vn,1),'--','LineWidth',1.1,'DisplayName','Nominal x');end
if nnz(va)>1,plot(t(va),ref(va,1),'LineWidth',1.5,'DisplayName','Active x');end
if nnz(vu)>1,plot(t(vu),p(vu,1),'LineWidth',1.0,'DisplayName','UAV x');end
xlabel('time [s]');ylabel('x [m]');title('Reference and tracking');legend('Location','best');hold off;
sgtitle(sprintf('Trajectory | objective=%s | correction energy=%.4g m^2 s', ...
    objective_name(replanner.synthesis_objective_code),double(replanner.synthesis_correction_energy_m2s)));

% Figure 10: timing only, from external runtime_profile.  No self-profiling
% fields from REPLANNING_SYSTEM are required.
figure(10);clf;
K=double(runtime_profile.k);
tr=double(runtime_profile.t(1:K));
on=1e3*double(runtime_profile.online_before_plant_s(1:K));
fu=1e3*double(runtime_profile.full_iteration_s(1:K));
pl=1e3*double(runtime_profile.simulation_plant_s(1:K));
ph=runtime_profile.phase(1:K);mask=(ph=='f');
subplot(2,1,1);hold on;grid on;box on;
plot(tr(mask),on(mask),'LineWidth',1.1,'DisplayName','online before plant');
plot(tr(mask),fu(mask),'LineWidth',1.1,'DisplayName','full iteration incl. logger');
yline(25,'--','25 ms control period','HandleVisibility','off');
xlabel('time [s]');ylabel('runtime [ms]');title('Flight execution time');legend('Location','best');hold off;
subplot(2,1,2);hold on;grid on;box on;
plot(tr(mask),pl(mask),'LineWidth',1.0,'DisplayName','simulation plant only');
xlabel('time [s]');ylabel('runtime [ms]');title('Numerical plant integration (simulation only)');legend('Location','best');hold off;
sgtitle('Runtime — production measurement, diagnostic light-audit excluded');

drawnow;

peak=NaN;if any(vc),peak=max(dn(vc));end
OUT=struct('schema','AF7R49_DASHBOARD_V1','peak_correction_m',peak, ...
    'objective_code',double(replanner.synthesis_objective_code), ...
    'correction_energy_m2s',double(replanner.synthesis_correction_energy_m2s));
assignin('base','AF7R49_DASHBOARD_OUT',OUT);
end

function n=objective_name(c)
if double(c)==1,n='MIN_CORRECTION_ENERGY';elseif double(c)==2,n='MIN_SNAP_RECOVERABILITY';else,n='UNKNOWN';end
end
function [t,p,pL,ref]=extract_ref(logger)
D=logger.Data;A=D.agent;if numel(A)>1,A=A(1);end
if isfield(D,'t')&&~isempty(D.t),t=double(D.t(:));else,C=rc(A,'reference');t=(0:numel(C)-1)'*0.025;end
n=numel(t);p=nan(n,3);pL=nan(n,3);ref=nan(n,3);Ce=rc(A,'estimator');Cr=rc(A,'reference');
for k=1:min(n,numel(Ce)),st=mem(Ce{k},'state');p(k,:)=v3(mem(st,'p'));pL(k,:)=v3(mem(st,'pL'));end
for k=1:min(n,numel(Cr)),st=mem(Cr{k},'state');ref(k,:)=v3(mem(st,'xd'));end
end
function C=rc(A,n),C={};if isstruct(A)&&isfield(A,n),S=A.(n);if isstruct(S)&&isfield(S,'result'),R=S.result;if iscell(R),C=R(:);else,C=num2cell(R(:));end,end,end,end
function x=mem(S,n),x=[];try,if isstruct(S)&&isfield(S,n),x=S.(n);elseif isobject(S)&&isprop(S,n),x=S.(n);end,catch,end,end
function v=v3(x),v=[NaN NaN NaN];if isnumeric(x)&&numel(x)>=3,x=double(x(:));if all(isfinite(x(1:3))),v=x(1:3).';end,end,end
function readable3d(X)
X=X(all(isfinite(X),2),:);if isempty(X),return;end
lo=min(X,[],1);hi=max(X,[],1);c=(lo+hi)/2;sp=hi-lo;m=max([sp(1),sp(3),1]);
if sp(2)<0.15*m,lo(2)=c(2)-0.075*m;hi(2)=c(2)+0.075*m;end
pd=0.05*max(hi-lo,1e-9);lo=lo-pd;hi=hi+pd;xlim([lo(1) hi(1)]);ylim([lo(2) hi(2)]);zlim([lo(3) hi(3)]);daspect([1 1 1]);
end
function draw_obs(r)
try,P=r.obstacle_packet;if ~isstruct(P)||double(P.count)<1,return;end;[sx,sy,sz]=sphere(10);U=[sx(:)';sy(:)';sz(:)'];
for i=1:double(P.count),c=double(P.C(:,i));R=reshape(double(P.R(:,i)),3,3);a=double(P.Araw(:,i));X=R*diag(a)*U+c;mesh(reshape(X(1,:),size(sx)),reshape(X(2,:),size(sy)),reshape(X(3,:),size(sz)),'FaceAlpha',0.04,'EdgeAlpha',0.18,'HandleVisibility','off');end
catch,end
end
