function OUT = audit_CBF_dual_constraint_construction(virtualMAT,physicalMAT)
% AUDIT_CBF_DUAL_CONSTRAINT_CONSTRUCTION -- flight-only, read-only CBF audit.
% VERSION: 20261016_PATH_ROTATED_PLANT_V2
%
% Run with both failure .mat files ANYWHERE ON THE MATLAB PATH:
%   OUT = audit_CBF_dual_constraint_construction();
% Or pass explicit absolute/relative filenames:
%   OUT = audit_CBF_dual_constraint_construction('C:/data/virtual.mat','C:/data/physical.mat');
% No files are written, copied or moved. Existing HLC/CBF is NOT modified.
%
% Tests for each saved physical/virtual S3 failure:
%   (1) Saved QP A,b vs generator rows, independent geometry h,
%       arbitrary obstacle count, 3D attitude R_obs and semiaxes.
%   (2) Independent LP feasibility and isolated hard-row authority.
%   (3) Optional: generated 24-state plant / 25-state mass-EKF model
%       vs 19-state quaternion/cable analytic kinematics at the snapshot.
%   (4) First Lie derivative of h from CBF psi1 vs centered finite
%       difference along the independent generated PLANT vector field.
%   (5) Arbitrary 3D rotated-ellipsoid probes, not path-specific cases.
%
% Scope: STATIC ellipsoids (center and orientation frozen). Moving/rotating
% obstacles need explicit time derivatives. This cannot certify higher
% (4th/6th) Lie derivatives, intersample safety, or recursive feasibility.
if nargin<1 || isempty(virtualMAT),virtualMAT='CBF_compare_virtual_QP_failure.mat';end
if nargin<2 || isempty(physicalMAT),physicalMAT='CBF_compare_physical_QP_failure.mat';end
if exist('Generate_CBF_ActuatorAware_6Methods','file')~=2
    error('CBF:NoGenerator','Put Generate_CBF_ActuatorAware_6Methods.m on the MATLAB path.');
end
mats={virtualMAT,physicalMAT}; modes={'virtual','physical'};
OUT=struct();
for im=1:numel(modes)
    mode=modes{im}; file=localFindOnPath(mats{im});
    fprintf('\n===== %s/S3: %s =====\n',upper(mode),file);
    tmp=load(file,'D');
    assert(isfield(tmp,'D'),'MAT file %s does not contain D.',file);
    D=tmp.D; R=D.filterResult;
    assert(strcmpi(char(R.mode),mode),'Snapshot mode mismatch: expected %s.',mode);
    assert(strcmpi(char(D.method),'S3'),'This snapshot audit expects S3.');
    x=double(D.x(:));P=double(D.P(:));F1=double(D.F1(:));
    refZ=double(D.xd([3,7,11,15,19,23,27]));
    aux=double(D.supervisor.aux(:));kappa=double(D.kappa(:));
    A=double(D.Aineq);b=double(D.bineq(:));w0=double(D.wNom(:));
    obs=D.obsList; rows=R.rows; nCBF=numel(rows); nObs=numel(obs);
    assert(size(A,2)==4 && size(A,1)==numel(b) && numel(w0)==4,'Invalid QP dimensions');
    assert(nObs>=1 && nCBF==3*nObs,'S3 requires exactly three CBF rows per obstacle');
    assert(size(R.rawA,1)==nCBF && numel(R.rawb)==nCBF,'Stored normalized rows mismatch');
    mapAError=max(abs(A(:)-double(R.map.Aineq(:))));
    mapBError=max(abs(b-double(R.map.bineq(:))));
    fprintf('CBF generator resolved to: %s\n',which('Generate_CBF_ActuatorAware_6Methods'));
    fprintf('time=%.5f s, nObs=%d, nCBF=%d, nQP=%d, exitflag=%g\n',...
        D.time,nObs,nCBF,size(A,1),R.qp.exitflag);
    fprintf('QP map save consistency: A %.3e, b %.3e | max nominal violation %.6g\n',...
        mapAError,mapBError,max(A*w0-b));
    fprintf('QP A column norms: %s\n',mat2str(vecnorm(A,2,1),6));
    parts={'uav','cable','load'};
    geomError=zeros(nCBF,1);rowError=zeros(nCBF,1);
    radiusPositive=true; rotationError=zeros(nObs,1);qObsError=nan(nObs,1);
    hdotError=nan(nCBF,1);rotationTestError=nan(nCBF,1);
    for iv=1:nObs
        if isfield(obs(iv),'v_center') && any(abs(double(obs(iv).v_center(:)))>1e-10)
            error('CBF:MovingObstacle', ...
                'Snapshot obstacle %d is moving: HOCBF lacks obstacle time derivatives.',iv);
        end
    end
    % Verify the model-dispatcher mapping when it is available. A matching
    % formula is insufficient if MATLAB actually simulates ANOTHER file.
    if exist('get_model_name','file')==2
        try
            mapped=get_model_name("Load_HL");
            if isa(mapped,'function_handle'),mapped=func2str(mapped);end
            mapped=char(mapped);
            mappedFile=which(mapped);
            fprintf('Model dispatch: get_model_name("Load_HL") = %s -> %s\n', ...
                mapped,mappedFile);
            if ~strcmp(mapped,'with_load_model_euler_for_HL')
                warning('CBF:PlantMappingDiffers', ...
                    'Live Load_HL uses %s, whereas this audit uses the uploaded Euler24 source.',mapped);
            end
        catch dispatchErr
            warning('CBF:PlantMappingUnverified','Could not verify Load_HL dispatcher: %s',dispatchErr.message);
        end
    end
    hasPlant=exist('with_load_model_euler_for_HL','file')==2;
    plantInfo=struct('available',false,'rhsError',NaN,'massEKFError',NaN,...
        'quatEulerDerivativeError',NaN,'source','');
    if hasPlant
        fTest=localCurrentThrust(mode,x,aux,refZ,F1,P);
        uTest=[fTest;double(D.uBase(2:4))];
        [plantDot,plantInfo]=localComparePlant19(x,P,uTest);
        fprintf('Plant 24-state source: %s\n',plantInfo.source);
        fprintf('Plant vs independent 19-state RHS: %.3e; EKF25 vs plant24: %.3e\n',...
            plantInfo.rhsError,plantInfo.massEKFError);
        fprintf('Euler-rate -> quaternion-rate residual: %.3e\n',plantInfo.quatEulerDerivativeError);
    else
        plantDot=[];
        fprintf('PLANT CHECK SKIPPED: with_load_model_euler_for_HL.m not on MATLAB path.\n');
    end
    for io=1:nObs
        o=obs(io); rot=double(o.R_obs); rr=double(o.ellipsoid_radii(:));
        assert(isequal(size(rot),[3 3]) && numel(rr)==3 && all(rr>0),...
            'Malformed obstacle %d: need R_obs 3x3 and positive semiaxes.',io);
        rotationError(io)=norm(rot.'*rot-eye(3),'fro');
        radiusPositive=radiusPositive && det(rot)>0 && rotationError(io)<1e-6;
        Qcorrect=rot*diag(1./(rr.^2))*rot.';
        if isfield(o,'Q_obs') && ~isempty(o.Q_obs)
            qObsError(io)=norm(double(o.Q_obs)-Qcorrect,'fro');
        end
        fprintf('obs%d center=%s semiaxes=%s det(R)=%.6g orthErr=%.3g QobsErr=%g\n',...
            io,mat2str(double(o.p_center(:).'),5),mat2str(rr.',5),...
            det(rot),rotationError(io),qObsError(io));
        for ic=1:3
            idx=(io-1)*3+ic;part=parts{ic};g=double(D.shapes.(part));
            if isfield(o,'id'),expectedId=o.id;else,expectedId=io;end
            assert(strcmpi(char(rows(idx).component),part) && ...
                isequal(double(rows(idx).obstacleID),double(expectedId)),...
                'CBF row order does not match obstacle/component list at row %d',idx);
            h=localSupportS3(x,P,o,part,g);
            e=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,'S3',part,...
                x,aux,refZ,F1,P,o,g,kappa);
            sc=max([1,abs(e.braw),norm(e.Araw,inf)]);
            geomError(idx)=abs(h-double(rows(idx).h));
            rowError(idx)=max([norm(double(e.Araw(:))/sc-double(R.rawA(idx,:).'),inf),...
                 abs(double(e.braw)/sc-double(R.rawb(idx)))]);
            if ~isempty(plantDot)
                hdotGen=double(e.info.psiChain(2))-kappa(1)*double(e.h);
                hdotPlant=localFDh(x,plantDot,P,o,part,g);
                hdotError(idx)=abs(hdotGen-hdotPlant);
                % Non-axis-aligned full 3D orientation probe.
                oTilt=o;rotTilt=localRz(.37)*localRy(-.46)*localRx(.29);
                oTilt.R_obs=rotTilt*rot;
                oTilt.Q_obs=oTilt.R_obs*diag(1./rr.^2)*oTilt.R_obs.';
                hTilt=localSupportS3(x,P,oTilt,part,g);
                et=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,'S3',part,...
                    x,aux,refZ,F1,P,oTilt,g,kappa);
                dht=localFDh(x,plantDot,P,oTilt,part,g);
                rotationTestError(idx)=max(abs(hTilt-double(et.h)),...
                    abs(double(et.info.psiChain(2))-kappa(1)*double(et.h)-dht));
            end
            fprintf('  %-5s h=%+8.4f geometryErr=%.2e normalizedRowErr=%.2e dHdtErr=%g rotatedProbeErr=%g\n',...
                part,h,geomError(idx),rowError(idx),hdotError(idx),rotationTestError(idx));
        end
    end
    % Independent LP on SAVED already normalized inequalities.
    lpSlack=NaN; lpFlag=NaN; inequalityResidual=NaN; worstIsolated=NaN;worstRow=NaN;
    if exist('linprog','file')==2
        options=optimoptions('linprog','Display','none');
        [z,~,lpFlag]=linprog([zeros(4,1);1],[A -ones(size(A,1),1)],...
            b,[],[],[],[],[-Inf(4,1);0],[],options);
        if lpFlag>0 && ~isempty(z)
            lpSlack=z(5);inequalityResidual=max(A*z(1:4)-b);
            fprintf('Independent LP slack=%.9g, max original inequality residual=%.9g\n',...
                lpSlack,inequalityResidual);
        else
            fprintf('Independent LP no certificate, exitflag=%g\n',lpFlag);
        end
        if strcmp(mode,'physical')
            % Which CBF rows individually conflict with the remaining actuator rows?
            for j=1:nCBF
                [~,v,ef]=linprog(A(j,:).',A(nCBF+1:end,:),...
                     b(nCBF+1:end),[],[],[],[],[],options);
                if ef>0 && (v-b(j)>worstIsolated || isnan(worstIsolated))
                    worstIsolated=v-b(j);worstRow=j;
                end
            end
            fprintf('Worst individually unachievable CBF row = %g, residual lower bound = %.9g\n',...
                 worstRow,worstIsolated);
        end
    else
        fprintf('LP SKIPPED: linprog not on MATLAB path.\n');
    end
    % Tolerances are diagnostic, not proof. FD is first order ONLY.
    passGeom=max(geomError)<1e-7 && max(rowError)<1e-7;
    passRotation=radiusPositive && ...
        (isempty(rotationTestError(~isnan(rotationTestError))) || ...
         max(rotationTestError(~isnan(rotationTestError)))<3e-3);
    if hasPlant
        massOkay=isnan(plantInfo.massEKFError) || plantInfo.massEKFError<1e-6;
        passPlant=plantInfo.rhsError<1e-6 && massOkay && ...
            max(hdotError)<3e-3;
    else
        passPlant=false;
    end
    fprintf('[AUDIT] stored algebra: %d | arbitrary-rotation probes: %d | plant/first-Lie: %d%s\n',...
        passGeom,passRotation,passPlant,localConditional(~hasPlant,' (skipped)',''));
    OUT.(mode)=struct('snapshotFile',file,'time',D.time,'qpExitflag',R.qp.exitflag,...
        'numObstacles',nObs,'numCBF',nCBF,'maxGeometryError',max(geomError),...
        'maxNormalizedRowError',max(rowError),'maxFirstLieError',...
        localMaxFinite(hdotError),'maxRotationProbeError',...
        localMaxFinite(rotationTestError),'QobsConsistencyErrors',qObsError,...
        'maxRotationOrthogonalityError',max(rotationError),...
        'plant',plantInfo,'lpCommonSlack',lpSlack,'lpExitflag',lpFlag,...
        'worstIsolatedCBFRow',worstRow,'worstIsolatedViolation',worstIsolated,...
        'passSnapshotGeometry',passGeom,'passRotatedProbe',passRotation,...
        'passPlantFirstDerivative',passPlant);
end
fprintf('\nAudit is read-only. Sampled rows, 4th/6th Lie derivatives, moving obstacles,\n');
fprintf('actuator hold, invariance and recursive feasibility remain UNPROVEN.\n');
end

function path=localFindOnPath(filename)
% MATLAB which searches current folder AND MATLAB search path.
filename=char(filename);
if exist(filename,'file')==2
    path=which(filename);
    if isempty(path),path=filename;end
elseif exist(fullfile(fileparts(mfilename('fullpath')),filename),'file')==2
    path=fullfile(fileparts(mfilename('fullpath')),filename);
else
    path=which(filename);
    if isempty(path)
        error('CBF:MATNotFound',['Cannot find %s on MATLAB path. Use addpath(folder) ' ...
            'or pass its full path as an input argument.'],filename);
    end
end
assert(exist(path,'file')==2,'Resolved path does not exist: %s',path);
end

function h=localSupportS3(x,P,o,part,g)
x=double(x(:));P=double(P(:));L=P(7);pt=x(14:16);pl=x(8:10);
if strcmp(part,'uav')
    c=pl-L*pt;S=diag(g(:).^2);
elseif strcmp(part,'load')
    c=pl;S=diag(g(:).^2);
elseif strcmp(part,'cable')
    c=pl-.5*L*pt;S=g(2)^2*eye(3)+(g(1)^2-g(2)^2)*(pt*pt.');
else
    error('Unknown S3 component: %s',part);
end
r=double(o.ellipsoid_radii(:));R=double(o.R_obs);
SO=R*diag(r.^2)*R.';d=double(o.p_center(:))-c;
if norm(d)<=1e-7,error('CBF:OverlappingCenters','Support-plane direction undefined');end
n=d/norm(d);
if isfield(o,'d_margin'),m=double(o.d_margin);else,m=0;end
h=norm(d)-sqrt(n.'*S*n)-sqrt(n.'*SO*n)-m;
end

function dh=localFDh(x,xdot,P,o,part,g)
epsT=1e-5;
px=x+epsT*xdot;mx=x-epsT*xdot;
% Manifold-preserving retraction removes only O(eps^2) drift.
px(1:4)=px(1:4)/norm(px(1:4));mx(1:4)=mx(1:4)/norm(mx(1:4));
px(14:16)=px(14:16)/norm(px(14:16));
mx(14:16)=mx(14:16)/norm(mx(14:16));
dh=(localSupportS3(px,P,o,part,g)-localSupportS3(mx,P,o,part,g))/(2*epsT);
end

function f=localCurrentThrust(mode,x,aux,refZ,F1,P)
if strcmp(mode,'physical')
    f=aux(1);return
end
m=P(1);mL=P(6);L=P(7);g=P(5);q=x(1:4);pt=x(14:16);wL=x(17:19);
b3=localBodyZ(q);pdot=cross(wL,pt);
beta=pt(3)*dot(pt,b3)/(m+mL);
assert(abs(beta)>1e-8,'CBF:VirtualThrustSingular','Virtual thrust inverse beta is singular.');
base=-g-m*L*dot(pdot,pdot)*pt(3)/(m+mL);
v1=-F1(1)*(x(10)-refZ(1))-F1(2)*(x(13)-refZ(2))+aux(1);
f=(refZ(3)+v1-base)/beta;
end

function [xdot,inf0]=localComparePlant19(x,P,u)
% Input x: [quaternion4; bodyRates3; pLoad3; vLoad3; cableDir3; cableOmega3].
% Plant function: Euler24, with pDrone and vDrone reconstructed.
q=x(1:4);w=x(5:7);pl=x(8:10);vl=x(11:13);pT=x(14:16);wL=x(17:19);
L=P(7);ang=localQuatToEuler(q);dpT=cross(wL,pT);
eulerState=[pl-L*pT;ang;vl-L*dpT;w;pl;vl;pT;wL];
par=zeros(1,21);par(1)=P(1);par(6:8)=P(2:4);par(9)=P(5);
par(20)=P(6);par(21)=L;
y=double(with_load_model_euler_for_HL(eulerState,u(:),par));y=y(:);
assert(numel(y)==24 && all(isfinite(y)),'Generated 24-state plant returned invalid RHS.');
qdot=.5*[-dot(q(2:4),w); ...
  q(1)*w(1)-q(4)*w(2)+q(3)*w(3); ...
  q(4)*w(1)+q(1)*w(2)-q(2)*w(3); ...
 -q(3)*w(1)+q(2)*w(2)+q(1)*w(3)];
xdot=[qdot;y(10:12);y(13:15);y(16:18);y(19:21);y(22:24)];
J=P(2:4);m=P(1);mL=P(6);g=P(5);
force=u(1)*localBodyZ(q);aL=pT*(dot(pT,force)-m*L*dot(dpT,dpT))/(m+mL)-[0;0;g];
expected=[qdot; (u(2:4)-cross(w,J(:).*w))./J(:); ...
    vl; aL; dpT; cross(-pT,force)/(m*L)];
if norm(P(8:9))>0
    fprintf('WARNING: P(8:9) disturbances nonzero, omitted from generated plant.\n');
end
rhsError=norm(xdot-expected,inf);
% Independently reconstruct quaternion rate from plant Euler-angle rate.
if abs(cos(ang(2)))>1e-3
    hh=1e-7;
    qp=localEulerToQuat(ang+hh*y(4:6));
    qm=localEulerToQuat(ang-hh*y(4:6));
    if dot(qp,q)<0,qp=-qp;qm=-qm;end
    eulerQuatErr=norm((qp-qm)/(2*hh)-qdot,inf);
else
    eulerQuatErr=NaN;
end
if exist('with_load_model_mL_euler_for_HL','file')==2
    x25=[eulerState;P(6)];param25=par;param25(20)=0; % dummy parameter, mass is state.
    y25=double(with_load_model_mL_euler_for_HL(x25,u(:),param25));
    massErr=max(norm(y25(1:24)-y,inf),abs(y25(25)));
else
    massErr=NaN;
end
inf0=struct('available',true,'rhsError',rhsError,...
    'massEKFError',massErr,'quatEulerDerivativeError',eulerQuatErr,...
    'source',which('with_load_model_euler_for_HL'));
end

function q=localEulerToQuat(e)
r=e(1);p=e(2);y=e(3);
a=cos(r/2);b=sin(r/2);c=cos(p/2);d=sin(p/2);z=cos(y/2);w=sin(y/2);
q=[a*c*z+b*d*w;b*c*z-a*d*w;a*d*z+b*c*w;a*c*w-b*d*z];
end
function e=localQuatToEuler(q)
q=q(:)/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
e=[atan2(2*(w*x+y*z),1-2*(x*x+y*y));...
   asin(max(-1,min(1,2*(w*y-z*x))));...
   atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
end
function v=localBodyZ(q)
q=q(:)/norm(q);v=[2*(q(2)*q(4)+q(1)*q(3)); ...
    2*(q(3)*q(4)-q(1)*q(2));1-2*(q(2)^2+q(3)^2)];
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
function s=localConditional(pred,a,b)
if pred,s=a;else,s=b;end
end
function m=localMaxFinite(x)
x=x(isfinite(x));if isempty(x),m=NaN;else,m=max(x);end
end
