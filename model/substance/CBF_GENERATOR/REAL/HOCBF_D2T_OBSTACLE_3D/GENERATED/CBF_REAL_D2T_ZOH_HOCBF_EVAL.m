function out=CBF_REAL_D2T_ZOH_HOCBF_EVAL(x,u,P,G,parts,lambda,dt)
% GENERATED ZOH HIGH-ORDER OBSTACLE EVALUATOR SOURCE (ID3 ONLY).
% Plant is continuous, controller is discrete, physical inputs are held
% through the next sample interval. The thrust-slew extension is NEVER
% applied to the physical ODE. The old generated 21-state Taylor jet is
% evaluated at dT=0, ddT=0; this gives the correct local derivatives for
% a constant T and constant torques during the *current* sample only.
% The fourth-order Taylor step is a numerical surrogate, NOT an exact Fdt.
% Its neglected remainder and estimator error are not bounded here.
validateattributes(x,{'numeric'},{'vector','numel',19,'finite','real'});
validateattributes(u,{'numeric'},{'vector','numel',4,'finite','real'});
validateattributes(dt,{'numeric'},{'scalar','positive','finite'});
validateattributes(lambda,{'numeric'},{'vector','numel',4,'positive','finite'});
x=double(x(:));u=double(u(:));lambda=double(lambda(:));
G.collisionMode='cable_ellipsoid';
% Do not interpret an intersample thrust ramp or ddT as physical input.
y=[x;u(1);0];v=[0;u(2);u(3)];
base=CBF_REAL_D2T_OBSTACLE_EVAL(y,v,u(4),P,G,parts,lambda);
% ZOH local 4th-order Taylor value of h at t+dt, not an exact flow map.
weights=dt.^(0:4);
hPred=weights*base.hTaylor;
% The one-step CBF uses exp(-lambda_1*dt) as an illustrative decay.
% Nominal acceptance also requires a separate one-hold geometry check.
res=hPred(:)-exp(-lambda(1)*dt)*base.stages(1,:).';
out=base;
out.zohStepDt=dt;
out.zohThrustHeld=true;
out.physicalInput=u;
out.discreteTaylorNextH=hPred(:);
out.discreteTaylorCBFResidual=res;
out.highOrderInput=[u(1);u(2);u(3)];
out.scope=['ZOH sample with local order-4 derivatives; exact discrete ', ...
    'dynamics, interval remainder, uncertainty and recursive feasibility ', ...
    'are NOT certified'];
end
