function OUT = audit_CBF_HOCBF_model_independent(virtualFile,physicalFile)
% AUDIT_CBF_HOCBF_MODEL_INDEPENDENT
% 20261017 / INDEPENDENT_TAYLOR_MODEL_V1
%
% READ-ONLY as to the plant, HLC, obstacle definitions and CBF controller.
% Tests the saved S3 virtual and physical QP failure states with an
% INDEPENDENT truncated Taylor-series implementation of the equations in
% generateModel_SupendedLoad.m / with_load_model_euler_for_HL.m.
% No trajectory, altitude, vertical direction or particular obstacle layout
% is assumed. All static obstacles use their original full 3-D R_obs.
%
% Default files may be anywhere on the MATLAB path (which is used):
%   OUT = audit_CBF_HOCBF_model_independent();
% Or provide absolute paths:
%   OUT = audit_CBF_HOCBF_model_independent('C:/.../virtual.mat','C:/.../physical.mat');
%
% A single timestamped .mat and .csv audit report is written to pwd.
% NO simulation is performed and NO control command is applied.
%
% The tests are numerical at provided states. Even exact agreement cannot
% prove continuous-time safety, feasibility for all states, sampled-data
% invariance, moving-obstacle safety or robust uncertain-mass safety.
if nargin<1 || isempty(virtualFile)
    virtualFile='CBF_compare_virtual_QP_failure.mat';
end
if nargin<2 || isempty(physicalFile)
    physicalFile='CBF_compare_physical_QP_failure.mat';
end
assert(exist('Generate_CBF_ActuatorAware_6Methods','file')==2, ...
    'CBF generator must be visible on the MATLAB path.');
files={localResolve(virtualFile),localResolve(physicalFile)};
modes={'virtual','physical'};
components={'uav','cable','load'};
OUT=struct();
OUT.tag='INDEPENDENT_TAYLOR_MODEL_V1';
OUT.generator=which('Generate_CBF_ActuatorAware_6Methods');
OUT.plant=which('with_load_model_euler_for_HL');
OUT.massEKF=which('with_load_model_mL_euler_for_HL');
OUT.snapshots=files;
OUT.results=struct([]);
OUT.notes={};
summary=struct('mode',{},'time_s',{},'obs_id',{},'component',{}, ...
    'assumed_order',{},'max_derivative_relative_error',{}, ...
    'max_psi_chain_relative_error',{},'max_affine_relative_error',{}, ...
    'first_input_order_nu',{},'first_input_order_Mx',{}, ...
    'first_input_order_My',{},'first_input_order_Mz',{}, ...
    'max_lower_order_relative_effect',{}, ...
    'max_order_r_nonlinearity_rel',{},'snapshot_valid',{});
rr=0;
for im=1:2
    mode=modes{im}; raw=load(files{im},'D');
    assert(isfield(raw,'D'),'Missing D in %s',files{im});
    D=raw.D;
    assert(strcmpi(char(D.filterResult.mode),mode), ...
        'Snapshot claims a different input mode: %s',mode);
    assert(strcmpi(char(D.method),'S3'), ...
        'Saved comparison snapshots must use S3.');
    x=double(D.x(:));P=double(D.P(:));F1=double(D.F1(:));
    ref=double(D.xd([3,7,11,15,19,23,27]));
    kappa=double(D.kappa(:));
    aux=double(D.supervisor.aux(:));
    obs=D.obsList;
    rows=D.filterResult.rows;
    assert(numel(obs)>=1 && numel(rows)==3*numel(obs), ...
        'S3 snapshot must have 3 barrier rows per obstacle.');
    fprintf('\n=== %s snapshot at t=%.5f s, %d obstacles ===\n', ...
        upper(mode),double(D.time),numel(obs));
    fprintf('generator: %s\nmodel: %s\n',OUT.generator,OUT.plant);
    fprintf('CBF mass parameter P(6)=%.9g kg (possibly EKF estimate, not true mass)\n',P(6));
    [plantCheck,plantMessage]=localPlantCheck(x,P,aux,mode,ref,F1,D);
    fprintf('First-order independent model crosscheck: %s\n',plantMessage);
    localAuditQP(D,mode);
    maxDerErr=0;
    for io=1:numel(obs)
        o=obs(io);
        if isfield(o,'v_center') && norm(double(o.v_center(:)))>1e-12
            error('CBF:MovingObstacle', ...
                'This HOCBF has no obstacle time derivatives; nonzero v_center for obs %d.',io);
        end
        R=double(o.R_obs); a=double(o.ellipsoid_radii(:));
        assert(isequal(size(R),[3,3]) && abs(det(R)-1)<1e-5 ...
            && norm(R.'*R-eye(3),'fro')<1e-5 ...
            && numel(a)==3 && all(a>0),'Invalid static oriented ellipsoid.');
        for ip=1:3
            component=components{ip}; j=3*(io-1)+ip;
            % For older MATLAB versions, avoid nested field indexing above:
            g0=D.shapes.(component); geom=double(g0(:));
            degree=double(rows(j).relativeDegree);
            assert(degree>=1 && degree<=6 && degree==round(degree), ...
                'Unsupported relative degree in saved snapshot.');
            e=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,'S3',component, ...
                x,aux,ref,F1,P,o,geom,kappa);
            % A separately coded plant, geometry and series algebra are used
            % below. No derivative routines of the CBF generator are called.
            D0=localIndependentDerivatives(x,aux,ref,F1,P,o,component,geom,mode,degree,zeros(4,1));
            psi0=localPsi(D0,kappa,degree);
            errDer=max(abs(psi0.effectivePsiChain(:)-double(e.info.psiChain(:)))./ ...
                max(1,abs(psi0.effectivePsiChain(:))));
            errH=abs(D0(1)-double(e.h));
            errP=abs(psi0.finalB-double(e.braw))/max(1,abs(psi0.finalB));
            errA=zeros(4,1); appearing=nan(1,4);
            lowerErr=0; nonlinearErr=0;
            derivPlus=zeros(degree+1,4);derivMinus=zeros(degree+1,4);
            for jj=1:4
                w=zeros(4,1);w(jj)=1;
                dPlus=localIndependentDerivatives(x,aux,ref,F1,P,o,component,geom,mode,degree,w);
                dMinus=localIndependentDerivatives(x,aux,ref,F1,P,o,component,geom,mode,degree,-w);
                derivPlus(:,jj)=dPlus(:);derivMinus(:,jj)=dMinus(:);
                aff=.5*(localPsi(dPlus,kappa,degree).finalB- ...
                          localPsi(dMinus,kappa,degree).finalB);
                errA(jj)=abs(aff-double(e.Araw(jj)))/max([1,abs(aff),abs(e.Araw(jj))]);
                for dk=1:degree
                    sens=.5*(dPlus(dk)-dMinus(dk));
                    % This threshold is a NUMERICAL DETECTION threshold,
                    % not a proof that a mathematically exact coefficient vanishes.
                    sigTol=2e-7*max([1,abs(D0(dk)),abs(dPlus(dk)),abs(dMinus(dk))]);
                    if isnan(appearing(jj)) && abs(sens)>sigTol
                        appearing(jj)=dk-1;
                    end
                    if dk<=degree
                        localErr=abs(sens)/max(1,abs(D0(dk)));
                        lowerErr=max(lowerErr,localErr);
                    end
                end
                nonlinearErr=max(nonlinearErr,abs( ...
                    localPsi(dPlus,kappa,degree).finalB+ ...
                    localPsi(dMinus,kappa,degree).finalB-2*psi0.finalB) ...
                    /max(1,abs(psi0.finalB)));
            end
            w=[0.29;-0.17;0.23;-0.31];
            mix=localIndependentDerivatives(x,aux,ref,F1,P,o,component,geom,mode,degree,w);
            mixB=localPsi(mix,kappa,degree).finalB;
            predicted=psi0.finalB+double(e.Araw(:)).'*w;
            nonlinearErr=max(nonlinearErr,abs(mixB-predicted)/max(1,abs(mixB)));
            maxDer=max([errH,errDer,errP,errA(:).']);
            maxDerErr=max(maxDerErr,maxDer);
            lowIdx=appearing(~isnan(appearing));
            if any(lowIdx<degree) % lower-order inputs invalidate a single r.
                fprintf(2,'[LOW-ORDER INPUT] %s obs%d %s assumed r=%d first orders=%s\n', ...
                    mode,io,component,degree,mat2str(appearing));
            end
            fprintf('%-8s obs%02d %-5s r=%d | hErr=%.2e psiErr=%.2e bErr=%.2e AErr=%.2e | first[nu,Mx,My,Mz]=%s | nonaff=%.2e\n', ...
                mode,io,component,degree,errH,errDer,errP,max(errA), ...
                mat2str(appearing),nonlinearErr);
            rr=rr+1;
            if isfield(o,'id'),obid=double(o.id);else,obid=io;end
            summary(rr)=struct('mode',mode,'time_s',double(D.time), ...
                'obs_id',obid,'component',component,'assumed_order',degree, ...
                'max_derivative_relative_error',maxDer, ...
                'max_psi_chain_relative_error',errDer, ...
                'max_affine_relative_error',max(errA), ...
                'first_input_order_nu',appearing(1), ...
                'first_input_order_Mx',appearing(2), ...
                'first_input_order_My',appearing(3), ...
                'first_input_order_Mz',appearing(4), ...
                'max_lower_order_relative_effect',lowerErr, ...
                'max_order_r_nonlinearity_rel',nonlinearErr, ...
                'snapshot_valid',logical(e.info.valid));
            if any(~isfinite([maxDer,lowerErr,nonlinearErr]))
                error('CBF:NonFiniteAudit','Nonfinite audit results; inspect this row.');
            end
        end
    end
    % Probe a GENERIC rotated obstacle, rather than a specific vertical path.
    ot=obs(1);Rt=localRz(0.43)*localRy(-0.36)*localRx(0.25)*double(ot.R_obs);
    ot.R_obs=Rt;
    if isfield(ot,'Q_obs')
        aa=double(ot.ellipsoid_radii(:));
        ot.Q_obs=Rt*diag(1./aa.^2)*Rt.';
    end
    if strcmp(mode,'physical'),ab=4;else,ab=6;end
    gLoad=double(D.shapes.load(:));
    eTilt=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,'S3','load', ...
        x,aux,ref,F1,P,ot,gLoad,kappa);
    dtTilt=localIndependentDerivatives(x,aux,ref,F1,P,ot,'load',gLoad,mode,ab,zeros(4,1));
    rotatedErr=abs(dtTilt(1)-double(eTilt.h));
    fprintf('Rotated off-axis ellipsoid probe: h-error=%.3e, R orthErr=%.3e\n', ...
        rotatedErr,norm(Rt.'*Rt-eye(3),'fro'));
    % Additional deterministic, non-vertical, off-axis states. These are
    % independent of the original flight trajectory / obstacle placement.
    generic=struct('caseIndex',{},'component',{},'relativeError',{},'valid',{});
    gp=0;
    for icase=1:2
        [xx,aa,oo]=localGenericProbe(x,aux,P,obs(1),mode,icase);
        for jp=1:3
            cpart=components{jp};gg=D.shapes.(cpart);gg=double(gg(:));
            gr=4;
            if strcmp(mode,'virtual') && strcmp(cpart,'load'),gr=6;end
            gp=gp+1;
            try
                eg=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,'S3',cpart, ...
                    xx,aa,ref,F1,P,oo,gg,kappa);
                d0=localIndependentDerivatives(xx,aa,ref,F1,P,oo,cpart,gg,mode,gr,zeros(4,1));
                gpsi=localPsi(d0,kappa,gr);
                er=abs(gpsi.finalB-double(eg.braw))/max(1,abs(gpsi.finalB));
                for kk=1:4
                    ut=zeros(4,1);ut(kk)=1;
                    pp=localIndependentDerivatives(xx,aa,ref,F1,P,oo,cpart,gg,mode,gr,ut);
                    pm=localIndependentDerivatives(xx,aa,ref,F1,P,oo,cpart,gg,mode,gr,-ut);
                    av=.5*(localPsi(pp,kappa,gr).finalB-localPsi(pm,kappa,gr).finalB);
                    er=max(er,abs(av-double(eg.Araw(kk)))/max(1,abs(av)));
                end
                generic(gp)=struct('caseIndex',icase,'component',cpart, ...
                    'relativeError',er,'valid',logical(eg.info.valid));
                fprintf('GENERIC %s case%d %-5s: normalized coefficient error %.3e, valid=%d\n', ...
                    mode,icase,cpart,er,logical(eg.info.valid));
            catch probeErr
                generic(gp)=struct('caseIndex',icase,'component',cpart, ...
                    'relativeError',NaN,'valid',false);
                fprintf(2,'GENERIC %s case%d %-5s FAILED: %s\n', ...
                    mode,icase,cpart,probeErr.message);
            end
        end
    end
    OUT.results(im).mode=mode;
    OUT.results(im).snapshotFile=files{im};
    OUT.results(im).time=double(D.time);
    OUT.results(im).estimatedLoadMass=P(6);
    OUT.results(im).plantCheck=plantCheck;
    OUT.results(im).maxDerivativeRelativeError=maxDerErr;
    OUT.results(im).rotatedGeometryError=rotatedErr;
    OUT.results(im).genericProbes=generic;
end
OUT.summary=struct2table(summary);
stamp=datestr(now,'yyyymmdd_HHMMSS');
OUT.outputMAT=fullfile(pwd,['CBF_HOCBF_model_audit_' stamp '.mat']);
OUT.outputCSV=fullfile(pwd,['CBF_HOCBF_model_audit_' stamp '.csv']);
save(OUT.outputMAT,'OUT');
writetable(OUT.summary,OUT.outputCSV);
fprintf('\n[AUDIT] Saved %s\n[AUDIT] Saved %s\n',OUT.outputMAT,OUT.outputCSV);
fprintf(['[AUDIT] Numerical crosschecks only. No collision avoidance, sampled-time safety,\n' ...
    '        moving obstacles, uncertain-parameter safety or recursive QP feasibility is certified.\n']);
end

function p=localResolve(f)
f=char(f);
if exist(f,'file')==2
    p=which(f);if isempty(p),p=f;end
else
    p=which(f);
end
assert(~isempty(p) && exist(p,'file')==2, ...
    'File not found on MATLAB path: %s. Use addpath or full path.',f);
end

function localAuditQP(D,mode)
A=double(D.Aineq);b=double(D.bineq(:));w=double(D.wNom(:));
assert(size(A,2)==4 && size(A,1)==numel(b) && numel(w)==4);
fprintf('Saved QP: %d hard rows, max residual at HLC nominal %.6g, exitflag=%g\n', ...
    size(A,1),max(A*w-b),double(D.filterResult.qp.exitflag));
if exist('linprog','file')~=2,return;end
opts=optimoptions('linprog','Display','off');
[z,~,flag]=linprog([zeros(4,1);1],[A,-ones(size(A,1),1)], ...
    b,[],[],[],[],[-inf(4,1);0],[],opts);
if flag>0 && ~isempty(z)
    fprintf('Independent LP common slack=%.8g (not an invariance certificate).\n',z(5));
else
    fprintf('Independent LP inconclusive, exitflag=%g.\n',flag);
end
if strcmp(mode,'virtual')
    fprintf('Virtual decision variables: [nu,v2,v3,v4] (NOT physical thrust).\n');
else
    fprintf('Physical decision variables: [fDDot,Mx,My,Mz].\n');
end
end

function [res,msg]=localPlantCheck(x,P,aux,mode,ref,F1,D)
res=struct('tested',false,'maxRhsError',nan,'ekfMaxRhsError',nan,...
    'source','');
if exist('with_load_model_euler_for_HL','file')~=2
    msg='SKIPPED: generated 24-state plant not on MATLAB path';return
end
% Exact state conversion: plant stores [p; Euler; v; wB; pL; vL; pT; wL].
q=x(1:4);wb=x(5:7);pL=x(8:10);vL=x(11:13);
pt=x(14:16);wL=x(17:19);L=P(7);
ang=localQ2Euler(q);
plantState=[pL-L*pt;ang;vL-L*cross(wL,pt);wb;pL;vL;pt;wL];
par=zeros(1,21);par(1)=P(1);par(6:8)=P(2:4);par(9)=P(5);par(20)=P(6);par(21)=L;
if strcmp(mode,'physical')
    f=aux(1);
else
    ptd=cross(wL,pt);bz=localBodyZ(q);
    ddz=ref(3);ez=pL(3)-ref(1);ev=vL(3)-ref(2);
    a=ddz-F1(1)*ez-F1(2)*ev+aux(1)+P(5);
    f=((P(1)+P(6))*a/pt(3)+P(1)*L*dot(ptd,ptd))/dot(pt,bz);
end
u=[f;double(D.uBase(2:4))];
fp=double(with_load_model_euler_for_HL(plantState,u,par));fp=fp(:);
assert(numel(fp)==24 && all(isfinite(fp)));
qDot=.5*[-dot(q(2:4),wb); q(1)*wb(1)+q(3)*wb(3)-q(4)*wb(2);...
     q(1)*wb(2)+q(4)*wb(1)-q(2)*wb(3);...
     q(1)*wb(3)+q(2)*wb(2)-q(3)*wb(1)];
frhs=[qDot;fp(10:12);fp(13:15);fp(16:18);fp(19:21);fp(22:24)];
b3=localBodyZ(q);tdot=cross(wL,pt);
refRhs=[qDot;(u(2:4)-cross(wb,P(2:4).*wb))./P(2:4);vL;...
    -P(5)*[0;0;1]+pt*(dot(pt,u(1)*b3)-P(1)*L*dot(tdot,tdot))/(P(1)+P(6));...
    tdot;cross(-pt,u(1)*b3)/(P(1)*L)];
err=norm(frhs-refRhs,inf);
res=struct('tested',true,'maxRhsError',err,'ekfMaxRhsError',nan, ...
    'source',which('with_load_model_euler_for_HL'));
if exist('with_load_model_mL_euler_for_HL','file')==2
    p2=par;p2(20)=0; % mass parameter is included as state in EKF model
    ekf=double(with_load_model_mL_euler_for_HL([plantState;P(6)],u,p2));
    res.ekfMaxRhsError=max(norm(ekf(1:24)-fp,inf),abs(ekf(25)));
end
if norm(P(8:9))>0
    msg=sprintf('plant 1st-order delta %.3e; CBF has xy disturbance not in generated plant (P8/P9 nonzero)',err);
else
    msg=sprintf('plant vs independent ODE %.3e, mass-EKF residual %.3e',err,res.ekfMaxRhsError);
end
end

function [x2,aux2,o2]=localGenericProbe(x,aux,P,o,mode,k)
% Synthetic physically admissible, non-vertical cable/attitude state.
% This is a coefficient test, not an obstacle-avoidance simulation.
if k==1
    ang=[.16;-.27;.43]; pt=[.21;-.18;-.96];
    omega=[.15;-.13;.18]; body=[.21;-.08;.11];
    pl=[1.4;-.8;10.2]; v=[.18;-.32;.65];
else
    ang=[-.32;.18;-.41]; pt=[-.29;.20;-.94];
    omega=[-.12;.22;-.19];body=[-.16;.24;-.21];
    pl=[-1.2;1.1;7.4]; v=[-.25;.11;-.2];
end
pt=pt/norm(pt);omega=omega-pt*(pt.'*omega);
q=localEuler2Q(ang);
x2=[q;body;pl;v;pt;omega];
if strcmp(mode,'virtual')
    aux2=zeros(4,1);
else
    aux2=[(P(1)+P(6))*P(5);0];
end
o2=o;o2.p_center=pl+[4.7;-3.1;2.8];
o2.R_obs=localRz(.25+.16*k)*localRy(-.41)*localRx(.27)*double(o.R_obs);
if isfield(o2,'Q_obs')
    a=double(o2.ellipsoid_radii(:));
    o2.Q_obs=o2.R_obs*diag(1./(a.^2))*o2.R_obs.';
end
end
function q=localEuler2Q(ang)
r=ang(1)/2;p=ang(2)/2;y=ang(3)/2;
q=[cos(r)*cos(p)*cos(y)+sin(r)*sin(p)*sin(y);...
   sin(r)*cos(p)*cos(y)-cos(r)*sin(p)*sin(y);...
   cos(r)*sin(p)*cos(y)+sin(r)*cos(p)*sin(y);...
   cos(r)*cos(p)*sin(y)-sin(r)*sin(p)*cos(y)];
end

function D=localIndependentDerivatives(x,aux,ref,F1,P,obs,name,g,mode,r,u)
% Independent power-series solution of the original mechanical equations,
% not a call to localJetRHS/localHDerivatives/localGeometryJets.
if strcmp(mode,'virtual'),X=[x(:);aux(:)];else,X=[x(:);aux(:)];end
N=r+1;
Y=zeros(numel(X),N);Y(:,1)=X;
for kk=0:r-1
    rhs=localODETaylor(Y,ref,F1,P,u,mode);
    Y(:,kk+2)=rhs(:,kk+1)/(kk+1);
end
z=localSeparationTaylor(Y,P,obs,name,g);
D=z(:).';
for kk=0:r,D(kk+1)=D(kk+1)*factorial(kk);end
end

function rhs=localODETaylor(Y,ref,F1,P,u,mode)
N=size(Y,2);one=zeros(1,N);one(1)=1;
q=Y(1:4,:);wb=Y(5:7,:);pL=Y(8:10,:);vL=Y(11:13,:);
pt=Y(14:16,:);wL=Y(17:19,:);
m=P(1);J=P(2:4);grav=P(5);mL=P(6);L=P(7);
% Quaternion derivative independently as q \otimes (0,wb)/2.
qdot=[-localDot(q(2:4,:),wb);...
    localMul(q(1,:),wb(1,:))+localMul(q(3,:),wb(3,:))-localMul(q(4,:),wb(2,:));...
    localMul(q(1,:),wb(2,:))+localMul(q(4,:),wb(1,:))-localMul(q(2,:),wb(3,:));...
    localMul(q(1,:),wb(3,:))+localMul(q(2,:),wb(2,:))-localMul(q(3,:),wb(1,:))]/2;
JW=bsxfun(@times,wb,J);
wdot=-bsxfun(@rdivide,localCross(wb,JW),J);
wdot(:,1)=wdot(:,1)+u(2:4)./J;
% Uses 1-2(qx^2+qy^2) for b3z, unlike generator's quaternion expression.
b3=[2*(localMul(q(2,:),q(4,:))+localMul(q(1,:),q(3,:)));...
    2*(localMul(q(3,:),q(4,:))-localMul(q(1,:),q(2,:)));...
    one-2*(localMul(q(2,:),q(2,:))+localMul(q(3,:),q(3,:)))];
ptdot=localCross(wL,pt);speed2=localDot(ptdot,ptdot);
if strcmp(mode,'physical')
    f=Y(20,:);
else
    zr=zeros(1,N);zv=zr;za=zr;
    for j=1:N
        if j<=numel(ref),zr(j)=ref(j)/factorial(j-1);end
        if j+1<=numel(ref),zv(j)=ref(j+1)/factorial(j-1);end
        if j+2<=numel(ref),za(j)=ref(j+2)/factorial(j-1);end
    end
    vz=za-F1(1)*(pL(3,:)-zr)-F1(2)*(vL(3,:)-zv)+Y(20,:);
    % Solved directly from ddot(pL_z), not from generator's beta/base form.
    f=localDiv((m+mL)*localDiv(vz+grav*one,pt(3,:))+m*L*speed2, ...
        localDot(pt,b3));
end
F=localScale(b3,f);
tdot=localCross(-pt,F)/(m*L);
acc=localScale(pt,(localDot(pt,F)-m*L*speed2)/(m+mL));
acc(:,1)=acc(:,1)+[P(8);P(9);-grav];
rhs=zeros(size(Y));
rhs(1:4,:)=qdot;rhs(5:7,:)=wdot;
rhs(8:10,:)=vL;rhs(11:13,:)=acc;
rhs(14:16,:)=ptdot;rhs(17:19,:)=tdot;
if strcmp(mode,'virtual')
    rhs(20:22,:)=Y(21:23,:);rhs(23,1)=u(1);
else
    rhs(20,:)=Y(21,:);rhs(21,1)=u(1);
end
end

function h=localSeparationTaylor(Y,P,obs,part,g)
N=size(Y,2);pl=Y(8:10,:);pt=Y(14:16,:);L=P(7);
if strcmp(part,'uav')
    ca=pl-L*pt;
elseif strcmp(part,'cable')
    ca=pl-(L/2)*pt;
elseif strcmp(part,'load')
    ca=pl;
else
    error('Unknown component %s',part);
end
cv=zeros(3,N);cv(:,1)=double(obs.p_center(:));
d=cv-ca;normsq=localDot(d,d);dn=localSqrt(normsq);
if strcmp(part,'cable')
    aa=g(1);bb=g(2);
    sysproj=bb^2*normsq+(aa^2-bb^2)*localMul(localDot(d,pt),localDot(d,pt));
else
    sysproj=zeros(1,N);
    for jj=1:3
        sysproj=sysproj+g(jj)^2*localMul(d(jj,:),d(jj,:));
    end
end
R=double(obs.R_obs);o=double(obs.ellipsoid_radii(:));
SO=R*diag(o.^2)*R.';
obsproj=zeros(1,N);
for ii=1:3
    for jj=1:3
        obsproj=obsproj+SO(ii,jj)*localMul(d(ii,:),d(jj,:));
    end
end
if isfield(obs,'d_margin'),margin=double(obs.d_margin);else,margin=0;end
h=dn-localDiv(localSqrt(sysproj),dn)-localDiv(localSqrt(obsproj),dn);
h(1)=h(1)-margin;
end

function p=localPsi(d,kappa,r)
assert(numel(d)==r+1);
c=1;ch=zeros(r,1);ch(1)=d(1);
for j=1:r-1
    c=conv(c,[kappa(j),1]);
    ch(j+1)=c*d(1:j+1).';
end
c=conv(c,[kappa(r),1]);
p=struct('effectivePsiChain',ch,'finalB',c*d(:));
end

function out=localMul(a,b)
a=a(:).';b=b(:).';N=numel(a);
out=conv(a,b);out=out(1:N);
end
function out=localDot(a,b)
N=size(a,2);out=zeros(1,N);
for j=1:size(a,1),out=out+localMul(a(j,:),b(j,:));end
end
function out=localCross(a,b)
out=[localMul(a(2,:),b(3,:))-localMul(a(3,:),b(2,:));...
     localMul(a(3,:),b(1,:))-localMul(a(1,:),b(3,:));...
     localMul(a(1,:),b(2,:))-localMul(a(2,:),b(1,:))];
end
function out=localScale(a,p)
out=zeros(size(a));
for j=1:size(a,1),out(j,:)=localMul(a(j,:),p);end
end
function out=localInv(a)
a=a(:).';N=numel(a);
assert(isfinite(a(1)) && abs(a(1))>1e-10,'Taylor reciprocal singular.');
out=zeros(1,N);out(1)=1/a(1);
for j=2:N
    v=0;
    for k=2:j,v=v+a(k)*out(j-k+1);end
    out(j)=-v/a(1);
end
end
function out=localDiv(a,b)
out=localMul(a,localInv(b));
end
function out=localSqrt(a)
a=a(:).';N=numel(a);
assert(a(1)>1e-14 && isfinite(a(1)),'Taylor square-root singular.');
out=zeros(1,N);out(1)=sqrt(a(1));
for j=2:N
    v=0;
    for k=2:j-1,v=v+out(k)*out(j-k+1);end
    out(j)=(a(j)-v)/(2*out(1));
end
end
function z=localBodyZ(q)
z=[2*(q(2)*q(4)+q(1)*q(3));...
   2*(q(3)*q(4)-q(1)*q(2));1-2*(q(2)^2+q(3)^2)];
end
function ang=localQ2Euler(q)
q=q(:)/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
ang=[atan2(2*(w*x+y*z),1-2*(x*x+y*y));...
     asin(max(-1,min(1,2*(w*y-z*x))));...
     atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
end
function R=localRx(t)
R=[1 0 0;0 cos(t) -sin(t);0 sin(t) cos(t)];
end
function R=localRy(t)
R=[cos(t) 0 sin(t);0 1 0;-sin(t) 0 cos(t)];
end
function R=localRz(t)
R=[cos(t) -sin(t) 0;sin(t) cos(t) 0;0 0 1];
end
