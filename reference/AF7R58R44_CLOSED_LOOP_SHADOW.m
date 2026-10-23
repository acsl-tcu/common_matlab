function O=AF7R58R44_CLOSED_LOOP_SHADOW(P,t0)
% R44R1: estimated-state-origin numerical HLC + original plant RHS shadow.
% MODEL-BASED NUMERICAL SHADOW ONLY. No production trigger/commit change.
% No invented controller gains, model coefficients, mass, horizon or solver.
% NOTE: controller purity and rigorous continuous-time clearance NOT certified.
% IMPORTANT: estimator mL bound to both plant and controller packets.
assert(isa(P,'REPLANNING_SYSTEM'), 'Live REPLANNING_SYSTEM required.');
if nargin<2 || isempty(t0)
    t0=double(P.result.state.time);
end
assert(isscalar(t0) && isfinite(t0), 'Invalid live estimate timestamp.');
S=P.self.estimator.result.state;
xstate=double(S.get());xstate=xstate(:);
assert(numel(xstate)==25 && all(isfinite(xstate)), ...
    'The live Load_mL_HL EKF must provide exactly 25 state elements.');
mL=double(S.mL);
assert(isfinite(mL)&&isscalar(mL)&&mL>0 && xstate(25)==mL, ...
    'Estimator mass is inconsistent with the 25D state.');
% True 24D plant state starts at the estimated 24D pose/rates/cable state.
x=xstate(1:24);
Pplant=P.self.plant;
assert(isa(Pplant,'MODEL_CLASS') && isa(Pplant.method,'function_handle') && ...
       isa(Pplant.solver,'function_handle'), ...
    'Actual loaded MODEL_CLASS and its live RHS/solver are required.');
% Parameter layout comes from the PROVIDED symbolic plant source: mass=1,
% jx/jy/jz=6..8, gravity=9, loadmass=20, cableL=21.
params=double(P.self.parameter.get('all','row'));
params=params(:).';
assert(numel(params)>=21 && all(isfinite(params(1:21))), ...
    'Live plant parameter vector unavailable.');
params(20)=mL; % user contract: current mass estimate is planning truth.
assert(abs(params(21)-double(P.L_cable))<=64*eps(max(1,abs(params(21)))), ...
    'Cable length differs between plant and replanner.');
C=P.self.controller.hlc_suspended;
assert(isa(C,'HLC_SUSPENDED_LOAD'), ...
    'Actual HLC_SUSPENDED_LOAD handle not present in controller.hlc_suspended.');
G=C.param;
assert(all(isfield(G,{'F1','F2','F3','F4','dt'})), ...
    'Live HLC gains and dt are required.');
dt=double(P.prediction_dt);H=double(P.horizon_s);
assert(dt>0&&isfinite(dt)&&H>0&&isfinite(H), ...
    'Current external dt/H are invalid.');
assert(isequal(double(G.dt),dt),'Controller and trigger sample dt differ.');
N=H/dt;
assert(abs(N-round(N))<=64*eps(max(1,abs(N))), ...
    'Prediction horizon must be an integer number of real control steps.');
N=round(N);
times=t0+(0:N)*dt;
states=nan(24,N+1);uApplied=nan(4,N);states(:,1)=x;
% Original HLC evaluates at controller ticks and holds u over the plant step.
% Pure local controller map: NEVER write into live estimator/HLC/plant.
for k=1:N
    tk=times(k);
    xd0=P.evaluate_nominal_snapshot_global( ...
        P.base_ref.func,double(P.flight_reference_time_origin),tk);
    xd=P.evaluate_active_reference_xd(tk,xd0);
    u=local_real_hlc_control(C,P,x,xd);
    uApplied(:,k)=u;
    [~,traj]=Pplant.solver( ...
        @(t,z) Pplant.method(z,u,params),[times(k) times(k+1)],x);
    x=double(traj(end,:)).';
    assert(numel(x)==24 && all(isfinite(x)), ...
        'Plant integration produced invalid 24D state.');
    states(:,k+1)=x;
end
O=struct('schema','AF7R58R44_CLOSED_LOOP_SHADOW_V1', ...
  'status','MODEL_CONSISTENT_NUMERICAL_SHADOW_NOT_A_HARD_CERTIFICATE', ...
  't',times,'x',states,'u',uApplied, ...
  'pQ',states(1:3,:),'pL',states(13:15,:),'q',states(19:21,:), ...
  'mL_authoritative_kg',mL, 'dt',dt,'H',H, ...
  'real_controller_method',class(C), ...
  'real_plant_method',func2str(Pplant.method), ...
  'real_plant_solver',func2str(Pplant.solver), ...
  'initial_state_is_estimator',isequal(states(:,1),xstate(1:24)), ...
  'production_trigger_switched',false, ...
  'continuous_collision_certified',false, ...
  'continuous_prediction_error_certified',false);
L=double(P.L_cable);
O.max_cable_geometry_residual_m= ...
    max(vecnorm(O.pL-O.pQ-L*O.q,2,1));
O.max_cable_unit_norm_error=max(abs(vecnorm(O.q,2,1)-1));
end

function u=local_real_hlc_control(C,P,x,xd)
% Literal source-equivalent calls from user's HLC_SUSPENDED_LOAD.do()
% with a supplied local x: no modification of any baseline object.
xd=double(xd(:));
assert(numel(xd)>=11 && all(isfinite(xd)), 'Invalid active reference xd.');
if numel(xd)<28,xd=[xd; zeros(28-numel(xd),1)];end
modelx=[x(4:6);x(10:12);x(13:15);x(16:18);x(19:21);x(22:24)];
% Exact source HLC yaw error convention (same wrapToPi/sign/cross/acos).
yaw=wrapToPi(x(6));yawd=xd(4);
yawUnit=[cos(yaw);sin(yaw);0];
yawdUnit=[cos(yawd);sin(yawd);0];
deltaYaw=sign(cross(yawdUnit,yawUnit))*acos(yawdUnit.'*yawUnit);
xd(4)=-deltaYaw(3)+yaw;
Param=C.param;
Pphys=double(P.self.parameter.get( ...
    ["mass","jx","jy","jz","gravity","loadmass","cableL"]));
Pphys=[Pphys(:).',0,0];
assert(numel(Pphys)==9 && all(isfinite(Pphys)), ...
    'Live controller parameter packet invalid.');
% The original HLC reads an external loadmass, but this offline ideal
% rollout MUST use the latest estimator value for the controller too.
% Param Pphys order: [m,jx,jy,jz,g,mL,L,0,0].
mLEst=double(P.self.estimator.result.state.mL);
assert(isfinite(mLEst)&&isscalar(mLEst)&&mLEst>0, ...
    'Live estimator mL is required for the controller packet.');
% Original HLC_SUSPENDED_LOAD.do() reads parameter.get('loadmass').
% If it differs from the EKF, silently substituting mLEst here would
% NOT reproduce the actual HLC. Reject this shadow until the live
% controller parameter authority is correctly synchronized.
assert(abs(double(Pphys(6))-mLEst)<=64*eps(max(1,abs(mLEst))), ...
    ['HLC mass authority differs from latest estimator mL; ' ...
     'a source-equivalent closed-loop shadow cannot be claimed.']);
Pphys(6)=mLEst;
vf=C.Vfd_SuspendedLoadxyDst(Param.dt,modelx,xd.',Param.F1);
vs=C.Vs_SuspendedLoadxyDst(modelx,xd.',vf,Pphys, ...
                              Param.F2,Param.F3,Param.F4);
uf=C.Uf_SuspendedLoadxyDst(modelx,xd.',vf,Pphys);
beta2=C.Beta2_SuspendedLoadxyDst(modelx,xd.',vf,Pphys);
vs_alpha2=C.V2_alpha2_SuspendedLoadxyDst(modelx,xd.',vf,vs.',Pphys);
us=beta2\vs_alpha2;
tmp=[uf(1);us];
assert(numel(tmp)==4 && all(isfinite(tmp)),'Pure HLC produced invalid 4D input.');
% These are the live HLC_SUSPENDED_LOAD.do saturation constants, not new limits.
u=[max(0,min(20,tmp(1)));max(-1,min(1,tmp(2))); ...
   max(-1,min(1,tmp(3)));max(-1,min(1,tmp(4)))];
end
