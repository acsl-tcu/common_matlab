function r = RA_AUDIT_CAPTURE_EKF_AND_ORACLE(agent,time)
% Test-only capture. Oracle fields MUST NEVER be passed into forecast H.
r=struct();r.time=double(time.t);
st=agent.estimator.result.state;
r.estP=take(st,'p');r.estLoad=take(st,'pL');
r.estVelocity=take(st,'v');r.estLoadVelocity=take(st,'vL');
r.estCableDir=take(st,'pT');r.estMass=take(st,'mL');
r.estCableAngularVelocity=take(st,'wL');
% Configured physical model constants only, never a true plant mass.
r.cableLength=double(agent.parameter.get("cableL"));
r.gravity=double(agent.parameter.get("gravity"));
r.reference=agent.reference.result.state.xd(:);
g=agent.reference.waypoint;
r.tau=double(g.virtualTime);r.rateState=g.rateState(:);
r.heldCommand=double(g.heldRateCommand);
r.governorSettings=[double(g.slowdownStart),double(g.slowdownFull),...
    double(g.minScale),double(g.filterTime)];
% Oracle-only columns are used at scoring time, never in H/K/routeCache.
r.oracleDrone=take(agent.plant.state,'p');
r.oracleLoad=take(agent.plant.state,'pL');
r.oracleCableDirection=take(agent.plant.state,'pT');
r.hlcInput=take(agent.controller.result,'input');
end
function v=take(x,name)
v=nan(3,1);
if isstruct(x),has=isfield(x,name);else,has=isprop(x,name);end
if has
  val=x.(name);
  if isnumeric(val),v=double(val(:));end
end
end
