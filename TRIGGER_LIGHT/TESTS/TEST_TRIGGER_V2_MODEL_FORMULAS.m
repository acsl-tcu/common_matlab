function report=TEST_TRIGGER_V2_MODEL_FORMULAS(cases)
%TEST_TRIGGER_V2_MODEL_FORMULAS Mathematical check on ideal taut model.
if nargin<1,cases=500;end
rng(20261112);
agent.parameter=struct('mass',0.762,'cableL',2,'gravity',9.81, ...
    'jx',0.022,'jy',0.029,'jz',0.048, 'loadmass',999);
agent.estimator.result.state=struct('mL',0.14);
model=wp_six_trigger_model_snapshot(agent);
assert(model.loadMass==0.14 && agent.parameter.loadmass==999);
t=[0,.025,.25,.7,2,4];
Fmax=20; % TEST ONLY: explicitly supplied, never hardcoded in production.
w0=[.11;-.19;.02];
B=wp_six_trigger_taut_model_accel_bounds(model,w0,t,Fmax);
assert(B.modelLoadMassUsed==0.14 && ~B.actualPlantSafetyCertified);
assert(numel(B.accDrone)==numel(t)-1);
assert(all(diff(B.angularSpeedUpper)>=0));
for trial=1:cases
    k=randi(numel(t)-1);
    q=randn(3,1);q=q/norm(q);
    b=randn(3,1);b=b/norm(b);
    omega=randn(3,1);omega=omega-q*(q'*omega);
    omega=omega/max(1,norm(omega))*B.angularSpeedUpper(k+1)*rand();
    f=Fmax*rand();
    mD=model.droneMass;mL=model.loadMass;L=model.cableLength;g=model.gravity;
    qdot=cross(omega,q);
    tension=mD*mL*L/(mD+mL)*norm(qdot)^2 - mL/(mD+mL)*f*(q'*b);
    aD=-g*[0;0;1]+f/mD*b+tension/mD*q;
    aL=-g*[0;0;1]-tension/mL*q;
    omegaDot=-f/(mD*L)*cross(q,b);
    guard=1e-10;
    assert(norm(aD) <= B.accDrone(k)+guard);
    assert(norm(aL) <= B.accLoad(k)+guard);
    assert(norm(omegaDot) <= Fmax/(mD*L)+guard);
    assert(abs(tension)<=B.tensionAbsoluteUpper(k)+guard);
end
% Test the time-integral inequality, including nonuniform intervals.
delta=[1.5,2.0,0.4,3.2,0];
rho=wp_six_trigger_error_tube_from_accel(t,.03,.12,delta);
assert(rho.rhoPosition(1)==.03);
manual=.03+.12*t(end);
for i=1:numel(delta)
    manual=manual+delta(i)*((t(end)-t(i))^2-(t(end)-t(i+1))^2)/2;
end
assert(abs(rho.rhoPosition(end)-manual)<1e-11);
missingRejected=false;
try
    wp_six_trigger_taut_model_accel_bounds(model,w0,t,NaN);
catch
    missingRejected=true;
end
assert(missingRejected);
report=struct('status','PASS_ESTIMATED_MASS_MODEL_FORMULAS_ONLY', ...
    'cases',cases,'usesTruePlantMass',false,'derivedFromIdealTautEquations',true, ...
    'original24StateRhsEquivalenceCertified',false, ...
    'actualHlcTrajectoryErrorCertified',false, ...
    'physicalSafetyCertified',false);
fprintf('PASS %d taut model algebra checks, nonuniform integral, estimated mass ONLY\n',cases);
end
