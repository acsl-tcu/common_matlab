function bounds = wp_six_trigger_taut_model_accel_bounds(model,wLoad0,timeOffsets,thrustMax)
%WP_SIX_TRIGGER_TAUT_MODEL_ACCEL_BOUNDS Analytic, model-conditional bounds.
%
% Model: ideal straight TAUT inextensible cable, fixed length L; no payload
% aerodynamic force; drone thrust F*b with ||b||=1, 0<=F<=thrustMax;
% gravity g and positive masses. wLoad0 is the CURRENT ESTIMATOR state.wL.
% Payload mass is *only* model.loadMass from the CURRENT estimator snapshot.
% This function NEVER reads plant truth or changes any parameter.
%
% Let q=(pLoad-pDrone)/L, ||q||=1, omega=q cross qdot (omega perpendicular q).
% On the taut branch the equations are
%   aD = -g*e3 + (F/mD)*b + (T/mD)*q,
%   aL = -g*e3 - (T/mL)*q,
%   T  = (mD*mL/(mD+mL))*L*||qdot||^2
%        - (mL/(mD+mL))*F*(q'*b),
%   omegaDot = -(F/(mD*L))*(q cross b).
% Therefore Omega(t) <= ||wLoad0|| + thrustMax*t/(mD*L), and
%   |T| <= (mD*mL/(mD+mL))*L*Omega(t)^2
%          + (mL/(mD+mL))*thrustMax.
% Use the right-end t(k+1) to bound each interval by monotonicity.
%
% These are LOOSE absolute acceleration bounds for the ESTIMATED-MASS
% MODEL, not guarantees about the real plant, not bounds on tracking error.
% If the taut branch, force model, state definition or thrust bound cannot
% be justified from the installed HLC/plant, DO NOT use them for CLEAR.
required={'loadMass','droneMass','cableLength','gravity'};
for k=1:numel(required)
    if ~isstruct(model) || ~isfield(model,required{k}) || ...
            ~isscalar(model.(required{k})) || ...
            ~isfinite(model.(required{k})) || model.(required{k})<=0
        error('LightTrigger:AnalyticBounds','Missing positive model field %s',required{k});
    end
end
if ~isnumeric(wLoad0) || numel(wLoad0)~=3 || any(~isfinite(wLoad0(:))) || ...
        ~isscalar(thrustMax) || ~isfinite(thrustMax) || thrustMax<0
    error('LightTrigger:AnalyticBounds','Need estimator wL and verified thrust upper bound');
end
t=reshape(timeOffsets,1,[]);
if numel(t)<2 || t(1)~=0 || any(diff(t)<=0) || any(~isfinite(t))
    error('LightTrigger:AnalyticBounds','Time grid must start at 0 and increase');
end
mD=model.droneMass;mL=model.loadMass;L=model.cableLength;g=model.gravity;
Omega0=norm(wLoad0(:));
OmegaUpper=Omega0+thrustMax*t/(mD*L);
OmegaRight=OmegaUpper(2:end);
Tmax=(mD*mL/(mD+mL))*L*OmegaRight.^2 + ...
     (mL/(mD+mL))*thrustMax;
bounds=struct();
bounds.timeOffsets=t;
bounds.angularSpeedUpper=OmegaUpper;
bounds.tensionAbsoluteUpper=Tmax;
bounds.accDrone=(g+thrustMax/mD)+Tmax/mD;
bounds.accLoad=g+Tmax/mL;
bounds.modelLoadMassUsed=mL;
bounds.modelAssumptions='taut_ideal_thrust_only_estimated_mass';
bounds.validForActualHlc=false;
bounds.actualPlantSafetyCertified=false;
bounds.isTrackingErrorTube=false;
end
