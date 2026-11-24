function results=CBF_REAL_D2T_SELFTEST()
% Unit checks only; no flight safety certificate or 25 ms WCET claim.
root=fileparts(mfilename('fullpath'));
generate_CBF_REAL_D2T_OBSTACLE();addpath(fullfile(root,'GENERATED'));
common=fullfile(fileparts(fileparts(root)),'COMMON');addpath(common);
P=struct('m',0.52,'mL',0.14,'cableL',2,'gravity',9.81, ...
  'J',[0.004;0.0045;0.008],'dstx',0,'dsty',0);
ang=0.27;q=[cos(ang/2);0;sin(ang/2);0];
pT=[-0.12;0.02;-sqrt(1-0.12^2-0.02^2)];
x=[q;0.15;-0.12;0.08;0;0;1.1;0.1;0.2;0.0;pT;0.02;-0.01;0.0];
assert(numel(x)==19,'CBFD2T:SelftestState');
T=5.9;dT=0.25;ddT=7.3;tau=[0.05;-0.07;0.02];
j=CBF_REAL_D2T_JET(x,T,dT,ddT,tau,P);
[dx,~]=CBF_COMMON_MODEL(x,[T;tau],P);
assert(norm(j.stateTaylor(:,2)-dx,inf)<1e-10,'CBFD2T:PhysicalRHS');
assert(norm(j.extendedTaylor(20,1:3)-[T dT ddT/2],inf)<1e-12,'CBFD2T:ThrustJet');
assert(norm(j.extendedTaylor(21,1:2)-[dT ddT],inf)<1e-12,'CBFD2T:RateJet');
% Independently compare the Taylor model with a fine explicit RK4 rollout
% of the extended 21-state continuous dynamics over a short interval.
y=[x;T;dT];dtSmall=1e-3;
k1=extended_rhs(y,ddT,tau,P);
k2=extended_rhs(y+0.5*dtSmall*k1,ddT,tau,P);
k3=extended_rhs(y+0.5*dtSmall*k2,ddT,tau,P);
k4=extended_rhs(y+dtSmall*k3,ddT,tau,P);
yRK=y+dtSmall*(k1+2*k2+2*k3+k4)/6;
yTaylor=j.extendedTaylor*[1;dtSmall;dtSmall^2;dtSmall^3;dtSmall^4];
assert(norm(yRK-yTaylor,inf)<2e-8,'CBFD2T:RolloutMismatch');
parts=struct('kind','sphere','center',[0.65;0.35;1.4], ...
 'params',struct('radius',0.18),'name','testObstacle');
G=struct('collisionMode','load_sphere','radius',0.16);
lam=[1;2;8;16];
row=CBF_REAL_D2T_QP_ROWS(y,tau(3),P,G,parts,lam,0.025,struct());
assert(row.affineValid,'CBFD2T:ControlNonaffine');
assert(isequal(size(row.barrierInfluence),[1,3]),'CBFD2T:InfluenceDimension');
% Verify the fourth derivative and each lower stage are control-affine.
u=[10;-0.13;0.17];
e=CBF_REAL_D2T_OBSTACLE_EVAL(y,u,tau(3),P,G,parts,lam);
assert(abs(e.stages(5)-row.barrierBias-row.barrierInfluence*u)<1e-7, ...
    'CBFD2T:AffineRegression');
assert(all(isfinite(row.A(:))) && all(isfinite(row.b)), ...
    'CBFD2T:FiniteRows');
assert(row.thrustHoldValid && strcmp(row.thrustHoldStatus,'EXACT_THRUST_HOLD_BOUNDS'), ...
    'CBFD2T:ThrustHoldRows');
% Exact interval constraints, including cases where degree-2 Bernstein's
% intermediate control point would violate the box despite a valid hold.
dt=0.025;
rUpper=CBF_REAL_D2T_THRUST_ROWS(19.5,100,dt,[0 20]);
assert(rUpper.feasibleInitialHold && abs(rUpper.ddTInterval(2)+10000)<1e-8, ...
    'CBFD2T:UpperInteriorExtremum');
assert(thrust_range_ok(19.5,100,-10000,dt,[0 20]), ...
    'CBFD2T:UpperExactBound');
assert(~thrust_range_ok(19.5,100,-9999,dt,[0 20]), ...
    'CBFD2T:UpperExactTightness');
rLower=CBF_REAL_D2T_THRUST_ROWS(0.5,-100,dt,[0 20]);
assert(rLower.feasibleInitialHold && abs(rLower.ddTInterval(1)-10000)<1e-8, ...
    'CBFD2T:LowerInteriorExtremum');
assert(thrust_range_ok(0.5,-100,10000,dt,[0 20]), ...
    'CBFD2T:LowerExactBound');
assert(~thrust_range_ok(0.5,-100,9999,dt,[0 20]), ...
    'CBFD2T:LowerExactTightness');
rImpossible=CBF_REAL_D2T_THRUST_ROWS(0,-1,dt,[0 20]);
assert(~rImpossible.feasibleInitialHold,'CBFD2T:OutwardRate');
% Exercise a broad deterministic set of initial thrust and rate values.
for Ti=[0 0.01 0.5 5 10 19.5 19.99 20]
    for ratei=[-200 -100 -10 0 10 100 200]
        tr=CBF_REAL_D2T_THRUST_ROWS(Ti,ratei,dt,[0 20]);
        if ~tr.feasibleInitialHold,continue;end
        candidates=[tr.ddTInterval(1),tr.ddTInterval(2),mean(tr.ddTInterval)];
        for ci=candidates
            assert(thrust_range_ok(Ti,ratei,ci,dt,[0 20]), ...
                'CBFD2T:ThrustExtremeMismatch');
        end
    end
end
% Repeat with the original XY cable collision geometry (not extra tilt).
Gc=struct('collisionMode','cable_ellipsoid', ...
 'cableAxisShort',0.2,'cableAxisLong',1.04,'cableCenterFromLoad',0.5);
rowCable=CBF_REAL_D2T_QP_ROWS(y,tau(3),P,Gc,parts,lam,0.025,struct());
assert(rowCable.affineValid,'CBFD2T:CableGeometryNonaffine');
Gd=struct('collisionMode','drone_sphere','radius',0.25);
rowDrone=CBF_REAL_D2T_QP_ROWS(y,tau(3),P,Gd,parts,lam,0.025,struct());
assert(rowDrone.affineValid,'CBFD2T:DroneGeometryNonaffine');
% These illustrative h5 bounds test the converter's algebra only; they
% are NOT verified bounds and must not be treated as safe.
fake=struct('a5Lower',-1000,'b5Lower',-ones(1,3), ...
    'b5Upper',ones(1,3));
interval=CBF_REAL_D2T_INTERVAL_ROWS(row,0.025,fake);
assert(size(interval.A,1)==9,'CBFD2T:IntervalRowCount');
results=struct('success',true,'rhsResidual',norm(j.stateTaylor(:,2)-dx,inf), ...
 'rk4Residual',norm(yRK-yTaylor,inf), ...
 'loadAffinityResidual',row.affinityError, ...
 'exactThrustBoxTested',true,'computedDdTBounds',row.thrustExactDdTInterval, ...
 'cableAffinityResidual',rowCable.affinityError, ...
 'droneAffinityResidual',rowDrone.affinityError, ...
 'matlabFlightValidated',false,'sampledDataSafetyValidated',false);
disp(results);
end
function dy=extended_rhs(y,v,tau,P)
dx=CBF_COMMON_MODEL(y(1:19),[y(20);tau],P);
dy=[dx;y(21);v];
end

function ok=thrust_range_ok(T,dT,ddT,dt,limits)
% Direct analytic min/max of the quadratic thrust profile: independent
% of the halfspace derivation used by CBF_REAL_D2T_THRUST_ROWS.
vals=[T,T+dT*dt+0.5*ddT*dt^2];
if ddT~=0
    s=-dT/ddT;
    if s>0 && s<dt
        vals(end+1)=T+dT*s+0.5*ddT*s^2; %#ok<AGROW>
    end
end
ok=min(vals)>=limits(1)-1e-10 && max(vals)<=limits(2)+1e-10;
end
