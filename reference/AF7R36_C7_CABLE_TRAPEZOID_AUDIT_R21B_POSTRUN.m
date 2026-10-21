function S=AF7R36_C7_CABLE_TRAPEZOID_AUDIT_R21B_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
assert(evalin('base','exist(''logger'',''var'')==1'));
assert(exist('sym','file')==2,'[R21B] Symbolic Math Toolbox required.');
R=evalin('base','replanner_instance');
LGR=evalin('base','logger');
assert(strcmpi(char(R.active_reference_mode),'REPAIRED'));
AR=R.active_repaired_reference;
AC=R.active_repaired_certificates;
M=AR.batch_flat_model;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 EXACT BINARY64 C7 + CABLE AUDIT R21B\n');
fprintf(' exact rational continuity test | zero tolerance | read-only\n');
fprintf('====================================================================================================\n');

cert=false; ord=NaN; src='';
if isfield(AC,'smoothness')&&isstruct(AC.smoothness)
    C=AC.smoothness;
    cert=isfield(C,'certified')&&logical(C.certified)&&isfield(C,'continuous_time')&&logical(C.continuous_time);
    if isfield(C,'continuity_order'),ord=double(C.continuity_order);end
    if isfield(C,'source'),src=char(C.source);end
end
yawPolicy=''; if isfield(AR,'yaw_policy'),yawPolicy=char(AR.yaw_policy);end
yawCert=false; yawOrd=NaN;
if isfield(AC,'yaw_c6')&&isstruct(AC.yaw_c6)
    Y=AC.yaw_c6;
    yawCert=isfield(Y,'certified')&&logical(Y.certified)&&isfield(Y,'continuous_time')&&logical(Y.continuous_time);
    if isfield(Y,'continuity_order'),yawOrd=double(Y.continuity_order);end
end

fprintf('\n[A. PRODUCTION CERTIFICATE METADATA]\n');
fprintf('translation certified/order                = %d / %.0f\n',cert,ord);
fprintf('translation source                         = %s\n',src);
fprintf('yaw policy                                 = %s\n',yawPolicy);
fprintf('yaw C6 exported/order                      = %d / %.0f\n',yawCert,yawOrd);

G=numel(M.corrections);
exactInternal=true(1,8); exactOuter0=true(1,8); exactOuter1=true(1,8);
maxApproxInternal=zeros(1,8); maxApproxOuter0=zeros(1,8); maxApproxOuter1=zeros(1,8);
firstFailGroup=NaN; firstFailJoin=NaN; firstFailOrder=NaN;

for g=1:G
    C=M.corrections(g);
    s0=double(C.segment_t0(:).'); s1=double(C.segment_t1(:).'); CP=C.control_points;
    assert(numel(CP)==numel(s0)&&numel(s0)==numel(s1)&&~isempty(CP));
    for r=0:7
        a=sym_epjet(CP{1},s1(1)-s0(1),r,true);
        b=sym_epjet(CP{end},s1(end)-s0(end),r,false);
        exactOuter0(r+1)=exactOuter0(r+1)&&allzero(a);
        exactOuter1(r+1)=exactOuter1(r+1)&&allzero(b);
        maxApproxOuter0(r+1)=max(maxApproxOuter0(r+1),symnorm(a));
        maxApproxOuter1(r+1)=max(maxApproxOuter1(r+1),symnorm(b));
    end
    for k=1:numel(CP)-1
        assert(s1(k)==s0(k+1),'[R21B] internal join times are not bitwise identical.');
        for r=0:7
            d=simplify(sym_epjet(CP{k},s1(k)-s0(k),r,false)-sym_epjet(CP{k+1},s1(k+1)-s0(k+1),r,true));
            zz=allzero(d);
            exactInternal(r+1)=exactInternal(r+1)&&zz;
            maxApproxInternal(r+1)=max(maxApproxInternal(r+1),symnorm(d));
            if ~zz && isnan(firstFailOrder)
                firstFailGroup=g; firstFailJoin=k; firstFailOrder=r;
            end
        end
    end
end

exactByOrder=exactInternal & exactOuter0 & exactOuter1;
exactOrder=-1;
for r=0:7
    if all(exactByOrder(1:r+1)),exactOrder=r;else,break;end
end

fprintf('\n[B. EXACT STORED-REPRESENTATION CONTINUITY]\n');
fprintf('internal exact zero d0..d7                 = %s\n',boolvec(exactInternal));
fprintf('outer-start exact zero d0..d7              = %s\n',boolvec(exactOuter0));
fprintf('outer-end exact zero d0..d7                = %s\n',boolvec(exactOuter1));
fprintf('all joins exact zero d0..d7                = %s\n',boolvec(exactByOrder));
fprintf('exact continuity order of stored correction = C%d\n',exactOrder);
fprintf('approx max internal residual d0..d7         = %s\n',numvec(maxApproxInternal));
if ~isnan(firstFailOrder)
    fprintf('first exact nonzero                         = group %d / join %d / derivative order %d\n',firstFailGroup,firstFailJoin,firstFailOrder);
else
    fprintf('first exact nonzero                         = NONE through derivative order 7\n');
end

if cert&&ord>=7&&exactOrder<7
    bindClass='STRUCTURAL_CERTIFICATE_C7_BUT_COMMITTED_BINARY64_REPRESENTATION_NOT_EXACT_C7';
elseif cert&&ord>=7&&exactOrder>=7
    bindClass='STRUCTURAL_CERTIFICATE_AND_COMMITTED_BINARY64_REPRESENTATION_BOTH_C7';
else
    bindClass='PRODUCTION_TRANSLATION_C7_AUTHORITY_NOT_CLOSED';
end
fprintf('binding classification                     = %s\n',bindClass);

D=LGR.Data; A=D.agent; if numel(A)>1,A=A(1);end
if isfield(D,'t')&&~isempty(D.t),t=double(D.t(:));else,CE=getcells(A,'estimator');t=(0:numel(CE)-1)'*double(R.prediction_dt);end
n=numel(t); CE=getcells(A,'estimator'); CP=getcells(A,'plant');
qE=nan(n,3);wE=nan(n,3);qP=nan(n,3);wP=nan(n,3);
for k=1:min(n,numel(CE))
    ss=member(CE{k},'state'); qE(k,:)=vec3(member(ss,'pT')); wE(k,:)=vec3(member(ss,'wL'));
end
for k=1:min(n,numel(CP))
    ss=member(CP{k},'state'); qP(k,:)=vec3(member(ss,'pT')); wP(k,:)=vec3(member(ss,'wL'));
end
qE=normrows(qE); qP=normrows(qP);
ta=min(arrayfun(@(c)double(c.t0),M.corrections)); tb=max(arrayfun(@(c)double(c.t1),M.corrections));
mask=t>=ta&t<=tb;
qD=nan(n,3); qdD=nan(n,3);
for k=find(mask).'
    y=double(AR.flat_eval_fn(t(k))); y=y(:);
    if numel(y)<15||any(~isfinite(y(9:15))),continue;end
    aa=y(9:11); jj=y(13:15); sf=aa+[0;0;double(R.gravity)]; ns=norm(sf);
    if ~(isfinite(ns)&&ns>0),continue;end
    qq=-sf/ns; qdq=-(jj-qq*(qq.'*jj))/ns;
    qD(k,:)=qq.'; qdD(k,:)=qdq.';
end
qD=normrows(qD);
eE=ang(qD,qE); eP=ang(qD,qP); eEP=ang(qE,qP);
tiltD=tilt(qD); tiltP=tilt(qP);
qdE=nan(n,3); qdP=nan(n,3);
for k=1:n
    if all(isfinite(qE(k,:)))&&all(isfinite(wE(k,:))),qdE(k,:)=cross(wE(k,:),qE(k,:));end
    if all(isfinite(qP(k,:)))&&all(isfinite(wP(k,:))),qdP(k,:)=cross(wP(k,:),qP(k,:));end
end
nD=rnorm(qdD); nP=rnorm(qdP); deP=rnorm(qdP-qdD);

fprintf('\n[C. CABLE TRACKING DIAGNOSTIC]\n');
printstat('angle(q_d,q_est) [deg]',eE(mask));
printstat('angle(q_d,q_plant) [deg]',eP(mask));
printstat('angle(q_est,q_plant) [deg]',eEP(mask));
printstat('desired tilt from downward [deg]',tiltD(mask));
printstat('plant tilt from downward [deg]',tiltP(mask));
printstat('||qdot_d|| [1/s]',nD(mask));
printstat('||qdot_plant|| [1/s]',nP(mask));
printstat('||qdot_plant-qdot_d|| [1/s]',deP(mask));

constDur=0; transDur=0; constCount=0; segCount=0; maxCtrl=0;
for g=1:G
    C=M.corrections(g); s0=double(C.segment_t0(:).'); s1=double(C.segment_t1(:).'); CP=C.control_points;
    for k=1:numel(CP)
        P=double(CP{k}); segCount=segCount+1; maxCtrl=max(maxCtrl,max(vecnorm(P,2,1)));
        flat=true;
        for j=2:16,flat=flat&&isequal(typecast(P(:,j),'uint64'),typecast(P(:,1),'uint64'));end
        if flat,constDur=constDur+s1(k)-s0(k);constCount=constCount+1;else,transDur=transDur+s1(k)-s0(k);end
    end
end
ratio=constDur/max(realmin,constDur+transDur); rounded=constCount>0&&transDur>0;
fprintf('\n[D. TRAJECTORY SHAPE]\n');
fprintf('segments / exact-constant segments          = %d / %d\n',segCount,constCount);
fprintf('constant / transition duration              = %.9g / %.9g s\n',constDur,transDur);
fprintf('plateau ratio                               = %.6f\n',ratio);
fprintf('max correction norm                         = %.9g m\n',maxCtrl);
fprintf('rounded-trapezoid by construction           = %d\n',rounded);

fprintf('\n[E. DECISION]\n');
fprintf('translation structural certificate          = %s\n',tfstr(cert&&ord>=7,'C7','NOT C7'));
fprintf('committed stored representation             = C%d EXACTLY\n',exactOrder);
fprintf('yaw production authority                    = %s\n',tfstr(yawCert&&yawOrd>=6,'C6 EXPORTED','C6 NOT EXPORTED'));
fprintf('trajectory shape                            = %s\n',tfstr(rounded,'ROUNDED TRAPEZOID','OTHER'));
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_EXACT_BINARY64_C7_CABLE_AUDIT_R21B';
S.translation=struct('certificate_C7',cert&&ord>=7,'exact_stored_continuity_order',exactOrder,'binding_classification',bindClass);
S.yaw=struct('policy',yawPolicy,'C6_exported',yawCert&&yawOrd>=6);
S.cable=struct('q_error_est_deg',calcstats(eE(mask)),'q_error_plant_deg',calcstats(eP(mask)), ...
    'est_vs_plant_deg',calcstats(eEP(mask)),'tilt_des_deg',calcstats(tiltD(mask)), ...
    'tilt_plant_deg',calcstats(tiltP(mask)),'qdot_des',calcstats(nD(mask)), ...
    'qdot_plant',calcstats(nP(mask)),'qdot_err_plant',calcstats(deP(mask)));
S.shape=struct('rounded_trapezoid',rounded,'constant_duration_s',constDur,'transition_duration_s',transDur,'plateau_ratio',ratio);
end

function y=sym_epjet(P,T,r,startSide)
P=sym(double(P),'f'); T=sym(double(T),'f'); n=15; coef=sym(1);
for j=0:r-1,coef=coef*sym(n-j);end
v=sym(zeros(3,1));
if startSide
    for k=0:r,v=v+sym((-1)^(r-k)*nchoosek(r,k))*P(:,k+1);end
else
    for k=0:r,v=v+sym((-1)^k*nchoosek(r,k))*P(:,16-k);end
end
y=simplify(coef*v/(T^r));
end
function tf=allzero(v)
tf=true;for i=1:numel(v),tf=tf&&isequal(simplify(v(i)),sym(0));end
end
function z=symnorm(v)
z=norm(double(vpa(v,32)));
end
function C=getcells(A,n)
C={};if ~isstruct(A)||~isfield(A,n),return;end;Q=A.(n);if ~isstruct(Q)||~isfield(Q,'result'),return;end
Q=Q.result;if iscell(Q),C=Q(:);else,C=num2cell(Q(:));end
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
function Q=calcstats(x)
x=double(x(:));x=x(isfinite(x));Q=struct('N',numel(x),'mean',NaN,'rms',NaN,'max',NaN);
if ~isempty(x),Q.mean=mean(x);Q.rms=sqrt(mean(x.^2));Q.max=max(x);end
end
function printstat(n,x)
Q=calcstats(x);fprintf('%-43s rms=%10.6g | max=%10.6g | mean=%10.6g | N=%d\n',n,Q.rms,Q.max,Q.mean,Q.N);
end
function s=boolvec(v)
s=['[' strjoin(arrayfun(@(x)sprintf('%d',logical(x)),v,'UniformOutput',false),' ') ']'];
end
function s=numvec(v)
s=['[' strjoin(arrayfun(@(x)sprintf('%.3e',x),v,'UniformOutput',false),' ') ']'];
end
function s=tfstr(tf,a,b)
if tf,s=a;else,s=b;end
end
