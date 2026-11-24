function out=CBF_REAL_D2T_ZOH_EVAL(x,u,P,parts,geom,dt,rkSteps)
% CBF_REAL_D2T_ZOH_EVAL - ZOH forward check for the FULL rotating cable ellipsoid.
% This is NOT the continuous-Tddot generated HOCBF. Physical T is held
% exactly constant during this step. Obstacle-only: NO drone, payload,
% ground or independent tilt constraints.
%
% Cable center C=pLoad-(L/2)*pT, body-axis half-length a=L/2+axialMargin,
% transverse semi-axis b. The FIXED current separating plane n gives
% h(s)=n'*(C(s)-Cobs)-sqrt(b^2+(a^2-b^2)*(n'*pT(s))^2)-supportObs(n).
% Positive h implies separation of BOTH full ellipsoids. Negative h does
% NOT imply collision. A conservative Lipschitz bound H is applied between
% every pair of numerical RK4 nodes for each constant-input 25ms hold:
%       min_{0<=s<=dt}h(s) >= min(hNodes)-H*dt/(2*rkSteps)
% for the EXACT ODE, PROVIDED the numerically computed nodes equal their
% exact states. MATLAB RK4 integration error is NOT rigorously enclosed.
% The routine therefore does NOT certify real-world intersample safety.
validateattributes(x,{'numeric'},{'vector','numel',19,'real','finite'});
validateattributes(u,{'numeric'},{'vector','numel',4,'real','finite'});
validateattributes(dt,{'numeric'},{'scalar','positive','real','finite'});
validateattributes(rkSteps,{'numeric'},{'scalar','integer','>=',0,'<=',20});
if ~isstruct(parts),error('CBFD2T:Parts','Observed obstacle parts must be a struct array');end
x=double(x(:));u=double(u(:));
if P.m<=0 || P.mL<0 || P.cableL<=0 || any(P.J<=0)
    error('CBFD2T:ModelParameters','Invalid physical parameters');
end
if ~isstruct(geom) || ~isfield(geom,'b') || ~isfield(geom,'a') || ...
        geom.b<=0 || geom.a<geom.b || ~isfinite(geom.a+geom.b)
    error('CBFD2T:Geometry','Invalid cable ellipsoid semi-axes');
end
qn=norm(x(1:4)); pn=norm(x(14:16));
if qn<0.99 || pn<0.99 || abs(qn-1)>0.005 || abs(pn-1)>0.005
    error('CBFD2T:Manifold','Quaternion or cable direction norm mismatch');
end
% Local evaluator projection only; never modify the plant or estimator.
x(1:4)=x(1:4)/qn;x(14:16)=x(14:16)/pn;
n=numel(parts); names=cell(n,1); normals=zeros(3,n); obsSupport=zeros(n,1);
center0=x(8:10)-0.5*P.cableL*x(14:16);
for i=1:n
    pt=parts(i);
    if ~isfield(pt,'center') || ~isfield(pt,'kind')
        error('CBFD2T:ObstacleFields','Obstacle lacks center or kind');
    end
    c=double(pt.center(:));
    validateattributes(c,{'numeric'},{'numel',3,'real','finite'});
    d=center0-c;
    if norm(d)<1e-9,error('CBFD2T:Normal','Cable and obstacle centres coincide');end
    normals(:,i)=d/norm(d);
    switch char(pt.kind)
        case 'sphere'
            if isfield(pt,'params') && ~isempty(pt.params), r=pt.params.radius; else,r=pt.radius;end
            validateattributes(r,{'numeric'},{'scalar','positive','real','finite'});
            obsSupport(i)=double(r);
        case 'ellipsoid'
            if isfield(pt,'params') && ~isempty(pt.params), ax=pt.params.axes; else,ax=pt.axes;end
            ax=double(ax(:));
            validateattributes(ax,{'numeric'},{'numel',3,'positive','real','finite'});
            if isfield(pt,'R'),R=double(pt.R);else,R=eye(3);end
            if ~isequal(size(R),[3 3]) || any(~isfinite(R(:))) || ...
                    norm(R.'*R-eye(3),'fro')>1e-5 || det(R)<0.9999
                error('CBFD2T:ObstacleRotation','Invalid rotation matrix');
            end
            obsSupport(i)=norm(diag(ax)*R.'*normals(:,i));
        otherwise
            error('CBFD2T:ObstacleKind','Unsupported obstacle kind %s',char(pt.kind));
    end
    if isfield(pt,'obstacleName'),names{i}=char(pt.obstacleName);
    elseif isfield(pt,'name'),names{i}=char(pt.name);
    else,names{i}=sprintf('obs_%d',i);end
    names{i}=[names{i} '/full_cable_ellipsoid'];
end
hNodes=zeros(n,rkSteps+1);state=x;step=dt/rkSteps;
hNodes(:,1)=clearance(state,parts,normals,obsSupport,geom,P.cableL);
% Bound cable angular velocity, payload acceleration and centre speed for
% every s in [0,dt]. The bound assumes unit q/pT under the continuous ODE.
Omega=norm(x(17:19))+abs(u(1))*dt/(P.m*P.cableL);
dist=0;
if isfield(P,'dstx'),dist=dist+P.dstx^2;end
if isfield(P,'dsty'),dist=dist+P.dsty^2;end
A=P.gravity+sqrt(dist)+(abs(u(1))+P.m*P.cableL*Omega^2)/(P.m+P.mL);
H=norm(x(11:13))+A*dt+(0.5*P.cableL+geom.a)*Omega;
if rkSteps==0
    % Fast coarse test: for the exact ODE and fixed separating plane,
    % h(s)>=h(0)-H*s, so h0-H*dt is a conservative full-hold bound.
    out=struct('h0',hNodes(:,1),'hEnd',hNodes(:,1), ...
        'hNodes',hNodes,'intervalLower',hNodes(:,1)-H*dt, ...
        'names',{names},'normals',normals,'xEnd',x,'slopeUpper',H, ...
        'physicalThrustZOH',u(1), ...
        'geometry','FULL_CABLE_ROTATING_ELLIPSOID_COARSE_MOTION_ENVELOPE');
    return
end
% Use identical RK4 integration scheme as the original model class's RHS.
for j=1:rkSteps
    k1=CBF_COMMON_MODEL(state,u,P);
    k2=CBF_COMMON_MODEL(state+0.5*step*k1,u,P);
    k3=CBF_COMMON_MODEL(state+0.5*step*k2,u,P);
    k4=CBF_COMMON_MODEL(state+step*k3,u,P);
    state=state+(step/6)*(k1+2*k2+2*k3+k4);
    if any(~isfinite(state)) || abs(norm(state(1:4))-1)>0.005 || ...
            abs(norm(state(14:16))-1)>0.005
        error('CBFD2T:Integration','Predicted state invalid');
    end
    hNodes(:,j+1)=clearance(state,parts,normals,obsSupport,geom,P.cableL);
end
out=struct('h0',hNodes(:,1),'hEnd',hNodes(:,end), ...
    'hNodes',hNodes,'intervalLower',min(hNodes,[],2)-H*dt/(2*rkSteps), ...
    'names',{names},'normals',normals,'xEnd',state,'slopeUpper',H, ...
    'physicalThrustZOH',u(1),'geometry','FULL_CABLE_ROTATING_ELLIPSOID_FIXED_PLANE');
end

function h=clearance(x,parts,n,sObs,geom,L)
C=x(8:10)-0.5*L*x(14:16); pT=x(14:16);
sAxis=n.'*pT;
sCable=sqrt(geom.b^2+(geom.a^2-geom.b^2)*sAxis.^2).';
centers=reshape([parts.center],3,[]);
h=sum(n.*(C-centers),1).'-sCable-sObs;
end
