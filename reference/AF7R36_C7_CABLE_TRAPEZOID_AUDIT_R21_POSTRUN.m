function S=AF7R36_C7_CABLE_TRAPEZOID_AUDIT_R21_POSTRUN()
% Read-only audit. No production/safety state is modified.
assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
assert(evalin('base','exist(''logger'',''var'')==1'));
R=evalin('base','replanner_instance');
LGR=evalin('base','logger');

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 C7 / CABLE / TRAPEZOID AUDIT R21\n');
fprintf('====================================================================================================\n');

assert(strcmpi(char(R.active_reference_mode),'REPAIRED'),'[R21] active reference is not REPAIRED.');
AR=R.active_repaired_reference;
AC=R.active_repaired_certificates;
M=AR.batch_flat_model;

%% A. Translation smoothness authority + exact stored Bezier joins
transCert=false; transOrder=NaN; transSource='';
if isfield(AC,'smoothness')
    C=AC.smoothness;
    transCert=isfield(C,'certified')&&logical(C.certified)&& ...
        isfield(C,'continuous_time')&&logical(C.continuous_time);
    if isfield(C,'continuity_order'),transOrder=double(C.continuity_order);end
    if isfield(C,'source'),transSource=char(C.source);end
end

yawPolicy='';
if isfield(AR,'yaw_policy'),yawPolicy=char(AR.yaw_policy);end
yawCert=false;yawOrder=NaN;
if isfield(AC,'yaw_c6')&&isstruct(AC.yaw_c6)
    C=AC.yaw_c6;
    yawCert=isfield(C,'certified')&&logical(C.certified)&& ...
        isfield(C,'continuous_time')&&logical(C.continuous_time);
    if isfield(C,'continuity_order'),yawOrder=double(C.continuity_order);end
end

assert(isfield(M,'corrections')&&isstruct(M.corrections));
G=numel(M.corrections);
jump=zeros(1,8); out0=zeros(1,8); out1=zeros(1,8);
contig=true; maxCtrl=0; maxAxis=[0 0 0];
constDur=0; transDur=0; constCount=0; segCount=0;

for g=1:G
    C=M.corrections(g);
    s0=double(C.segment_t0(:).'); s1=double(C.segment_t1(:).'); CP=C.control_points;
    assert(numel(s0)==numel(s1)&&numel(CP)==numel(s0));
    segCount=segCount+numel(CP);
    for k=1:numel(CP)
        P=double(CP{k}); assert(isequal(size(P),[3 16]));
        maxCtrl=max(maxCtrl,max(vecnorm(P,2,1)));
        maxAxis=max(maxAxis,max(abs(P),[],2).');
        sc=max(1,max(abs(P),[],'all'));
        if max(abs(P-P(:,1)),[],'all')<=4096*eps(sc)
            constDur=constDur+(s1(k)-s0(k)); constCount=constCount+1;
        else
            transDur=transDur+(s1(k)-s0(k));
        end
    end
    for r=0:7
        out0(r+1)=max(out0(r+1),norm(epjet(double(CP{1}),s1(1)-s0(1),r,true)));
        out1(r+1)=max(out1(r+1),norm(epjet(double(CP{end}),s1(end)-s0(end),r,false)));
    end
    for k=1:numel(CP)-1
        sc=max([1 abs(s1(k)) abs(s0(k+1))]);
        contig=contig && abs(s1(k)-s0(k+1))<=128*eps(sc);
        for r=0:7
            a=epjet(double(CP{k}),s1(k)-s0(k),r,false);
            b=epjet(double(CP{k+1}),s1(k+1)-s0(k+1),r,true);
            jump(r+1)=max(jump(r+1),norm(a-b));
        end
    end
end
guard=4096*eps(max(1,maxCtrl));
storedC7=max([jump out0 out1])<=guard;
translationC7=transCert&&transOrder>=7&&storedC7&&contig;

fprintf('\n[A. SMOOTHNESS AUTHORITY]\n');
fprintf('translation certificate/order              = %d / %.0f\n',transCert,transOrder);
fprintf('translation source                         = %s\n',transSource);
fprintf('stored correction join residual d0..d7     = %s\n',fmt(jump));
fprintf('outer start residual d0..d7                = %s\n',fmt(out0));
fprintf('outer end residual d0..d7                  = %s\n',fmt(out1));
fprintf('stored binary64 C7 closure                 = %d\n',storedC7);
fprintf('translation C7 authority                   = %d\n',translationC7);
fprintf('yaw policy                                 = %s\n',yawPolicy);
fprintf('yaw C6 exported authority/order            = %d / %.0f\n',yawCert,yawOrder);

%% B. Logger cable data
D=LGR.Data; A=D.agent; if numel(A)>1,A=A(1);end
if isfield(D,'t')&&~isempty(D.t)
    t=double(D.t(:));
else
    CE=getcells(A,'estimator'); t=(0:numel(CE)-1)'*double(R.prediction_dt);
end
n=numel(t); CE=getcells(A,'estimator'); CP=getcells(A,'plant'); CR=getcells(A,'reference');
qE=nan(n,3);wE=nan(n,3);qP=nan(n,3);wP=nan(n,3);yaw=nan(n,1);
for k=1:min(n,numel(CE))
    st=member(CE{k},'state'); qE(k,:)=vec3(member(st,'pT')); wE(k,:)=vec3(member(st,'wL'));
end
for k=1:min(n,numel(CP))
    st=member(CP{k},'state'); qP(k,:)=vec3(member(st,'pT')); wP(k,:)=vec3(member(st,'wL'));
end
for k=1:min(n,numel(CR))
    st=member(CR{k},'state'); xd=member(st,'xd');
    if isnumeric(xd)&&numel(xd)>=4,xd=double(xd(:));yaw(k)=xd(4);end
end
qE=normrows(qE);qP=normrows(qP);

if G>0
    ta=min(arrayfun(@(c)double(c.t0),M.corrections));
    tb=max(arrayfun(@(c)double(c.t1),M.corrections));
else
    ta=max(5,min(t));tb=min(47,max(t));
end
mask=t>=ta&t<=tb;

qD=nan(n,3);qdD=nan(n,3);
for k=find(mask).'
    y=double(AR.flat_eval_fn(t(k)));y=y(:);
    if numel(y)<15||any(~isfinite(y(9:15))),continue;end
    a=y(9:11);j=y(13:15); sf=a+[0;0;double(R.gravity)]; ns=norm(sf);
    if ~(isfinite(ns)&&ns>0),continue;end
    q=-sf/ns; qdot=-(j-q*(q.'*j))/ns;
    qD(k,:)=q.';qdD(k,:)=qdot.';
end
qD=normrows(qD);
eE=ang(qD,qE);eP=ang(qD,qP);eEP=ang(qE,qP);
tiltD=tilt(qD);tiltE=tilt(qE);tiltP=tilt(qP);
qdE=nan(n,3);qdP=nan(n,3);
for k=1:n
    if all(isfinite(qE(k,:)))&&all(isfinite(wE(k,:))),qdE(k,:)=cross(wE(k,:),qE(k,:));end
    if all(isfinite(qP(k,:)))&&all(isfinite(wP(k,:))),qdP(k,:)=cross(wP(k,:),qP(k,:));end
end
nD=rnorm(qdD);nE=rnorm(qdE);nP=rnorm(qdP);
deE=rnorm(qdE-qdD);deP=rnorm(qdP-qdD);

fprintf('\n[B. CABLE TRACKING DIAGNOSTIC]\n');
fprintf('interval                                   = %.6f -- %.6f s\n',ta,tb);
pst('angle(q_d,q_est) [deg]',eE(mask));
pst('angle(q_d,q_plant) [deg]',eP(mask));
pst('angle(q_est,q_plant) [deg]',eEP(mask));
pst('desired tilt from downward [deg]',tiltD(mask));
pst('estimated tilt from downward [deg]',tiltE(mask));
pst('plant tilt from downward [deg]',tiltP(mask));
pst('||qdot_d|| [1/s]',nD(mask));
pst('||qdot_est|| [1/s]',nE(mask));
pst('||qdot_plant|| [1/s]',nP(mask));
pst('||qdot_est-qdot_d|| [1/s]',deE(mask));
pst('||qdot_plant-qdot_d|| [1/s]',deP(mask));

%% C. Shape
ratio=constDur/max(realmin,constDur+transDur);
[~,ia]=max(maxAxis(1:2));nm={'x','y'};
rounded=constCount>0&&transDur>0;
fprintf('\n[C. TRAJECTORY SHAPE]\n');
fprintf('segments / constant segments               = %d / %d\n',segCount,constCount);
fprintf('constant duration / transition duration     = %.9g / %.9g s\n',constDur,transDur);
fprintf('plateau ratio                               = %.6f\n',ratio);
fprintf('max correction norm                         = %.9g m\n',maxCtrl);
fprintf('max abs correction xyz                      = [%.9g %.9g %.9g] m\n',maxAxis);
fprintf('dominant lateral axis                       = %s\n',nm{ia});
fprintf('rounded-trapezoid signature                 = %d\n',rounded);

yy=yaw(mask&isfinite(yaw));ys=NaN;if ~isempty(yy),ys=max(yy)-min(yy);end
if translationC7 && ~(yawCert&&yawOrder>=6)
    cls='TRANSLATION_C7_CLOSED__YAW_C6_NOT_EXPORTED_BY_REFERENCE_ONLY_AUTHORITY';
elseif translationC7 && yawCert&&yawOrder>=6
    cls='TRANSLATION_C7_AND_YAW_C6_CLOSED';
else
    cls='TRANSLATION_C7_NOT_CLOSED';
end
fprintf('\n[D. C6/C7 DECISION]\n');
fprintf('translation meets C6 minimum                = %d\n',translationC7);
fprintf('yaw logged span (diagnostic only)           = %.9g rad\n',ys);
fprintf('classification                              = %s\n',cls);

try
    figure(21);clf;plot(t(mask),eE(mask),'LineWidth',1.3);hold on;grid on;box on;
    if any(isfinite(eP(mask))),plot(t(mask),eP(mask),'LineWidth',1.3);legend('q_d vs estimator','q_d vs plant','Location','best');else,legend('q_d vs estimator');end
    xlabel('time [s]');ylabel('angle error [deg]');title('R21 cable direction tracking');hold off;
    figure(22);clf;plot(t(mask),tiltD(mask),'LineWidth',1.3);hold on;grid on;box on;
    plot(t(mask),tiltE(mask),'LineWidth',1.3);if any(isfinite(tiltP(mask))),plot(t(mask),tiltP(mask),'LineWidth',1.3);legend('desired','estimator','plant','Location','best');else,legend('desired','estimator');end
    xlabel('time [s]');ylabel('tilt from downward [deg]');title('R21 cable tilt');hold off;
    figure(23);clf;plot(t(mask),nD(mask),'LineWidth',1.3);hold on;grid on;box on;
    plot(t(mask),nE(mask),'LineWidth',1.3);if any(isfinite(nP(mask))),plot(t(mask),nP(mask),'LineWidth',1.3);legend('qdot desired','qdot estimator','qdot plant','Location','best');else,legend('qdot desired','qdot estimator');end
    xlabel('time [s]');ylabel('||qdot|| [1/s]');title('R21 cable direction rate');hold off;
catch ME
    fprintf('plot warning                                = %s\n',ME.message);
end

fprintf('\n[SUMMARY]\n');
fprintf('translation                                = %s\n',tfstr(translationC7,'C7 CLOSED','NOT CLOSED'));
fprintf('yaw authority                              = %s\n',tfstr(yawCert&&yawOrder>=6,'C6 CLOSED','NOT EXPORTED/CLOSED'));
fprintf('time-shape                                 = %s\n',tfstr(rounded,'ROUNDED TRAPEZOID BY CONSTRUCTION','NO PLATEAU SIGNATURE'));
fprintf('cable results                              = DIAGNOSTIC ONLY, NOT SAFETY AUTHORITY\n');
fprintf('====================================================================================================\n');

S=struct('schema','AF7_C7_CABLE_TRAPEZOID_AUDIT_R21');
S.translation=struct('C7_closed',translationC7,'join_d0_d7',jump,'outer_start',out0,'outer_end',out1);
S.yaw=struct('policy',yawPolicy,'C6_certified',yawCert&&yawOrder>=6,'logged_span_rad',ys,'classification',cls);
S.cable=struct('q_error_est_deg',st(eE(mask)),'q_error_plant_deg',st(eP(mask)), ...
    'est_vs_plant_deg',st(eEP(mask)),'tilt_des_deg',st(tiltD(mask)), ...
    'tilt_est_deg',st(tiltE(mask)),'tilt_plant_deg',st(tiltP(mask)), ...
    'qdot_des',st(nD(mask)),'qdot_est',st(nE(mask)),'qdot_plant',st(nP(mask)), ...
    'qdot_err_est',st(deE(mask)),'qdot_err_plant',st(deP(mask)));
S.shape=struct('rounded_trapezoid',rounded,'constant_duration_s',constDur, ...
    'transition_duration_s',transDur,'plateau_ratio',ratio,'max_correction_norm_m',maxCtrl);
end

function y=epjet(P,T,r,startSide)
D=P;deg=size(P,2)-1;
for k=1:r,D=(D(:,2:end)-D(:,1:end-1))*(deg/T);deg=deg-1;end
if startSide,y=D(:,1);else,y=D(:,end);end
end
function C=getcells(A,n)
C={};if ~isstruct(A)||~isfield(A,n),return;end;S=A.(n);if ~isstruct(S)||~isfield(S,'result'),return;end
Q=S.result;if iscell(Q),C=Q(:);else,C=num2cell(Q(:));end
end
function x=member(S,n)
x=[];try,if isstruct(S)&&isfield(S,n),x=S.(n);elseif isobject(S)&&isprop(S,n),x=S.(n);end,catch,end
end
function r=vec3(v)
r=[NaN NaN NaN];if isnumeric(v)&&numel(v)>=3,v=double(v(:));r=v(1:3).';end
end
function Q=normrows(Q)
for k=1:size(Q,1),if all(isfinite(Q(k,:))),z=norm(Q(k,:));if z>0,Q(k,:)=Q(k,:)/z;else,Q(k,:)=nan(1,3);end,end,end
end
function a=ang(A,B)
a=nan(size(A,1),1);for k=1:size(A,1),if all(isfinite(A(k,:)))&&all(isfinite(B(k,:))),a(k)=acosd(max(-1,min(1,dot(A(k,:),B(k,:)))));end,end
end
function a=tilt(Q)
a=nan(size(Q,1),1);for k=1:size(Q,1),if all(isfinite(Q(k,:))),a(k)=acosd(max(-1,min(1,-Q(k,3))));end,end
end
function z=rnorm(X)
z=sqrt(sum(X.^2,2));z(~all(isfinite(X),2))=NaN;
end
function Q=st(x)
x=double(x(:));x=x(isfinite(x));Q=struct('N',numel(x),'mean',NaN,'rms',NaN,'max',NaN);
if ~isempty(x),Q.mean=mean(x);Q.rms=sqrt(mean(x.^2));Q.max=max(x);end
end
function pst(n,x)
Q=st(x);fprintf('%-43s rms=%10.6g | max=%10.6g | mean=%10.6g | N=%d\n',n,Q.rms,Q.max,Q.mean,Q.N);
end
function s=fmt(v)
s=['[' strjoin(arrayfun(@(x)sprintf('%.3e',x),v,'UniformOutput',false),' ') ']'];
end
function s=tfstr(tf,a,b)
if tf,s=a;else,s=b;end
end
