function test_waypoint_auto_adapt()
%TEST_WAYPOINT_AUTO_ADAPT  Route profile + exact 6-stage governor tests.
% Run with project root/subfolders on MATLAB path.
openRef = gen_wp_straight('speed',0.2);
assert(strcmp(openRef.rampMode,'auto'));
assert(openRef.rampTimeEffective > 0 && isfinite(openRef.rampTimeEffective));
assert(all(size(openRef.evaluate(0))==[28,1]));
% Safe cross-version check below:
startXd = openRef.evaluate(0);
assert(norm(startXd(5:7))<1e-10);
endXd = openRef.evaluate(openRef.duration);
assert(norm(endXd(5:28))<1e-9);
% Manually chosen ramp time remains possible for A/B comparison.
manualRef = gen_wp_straight('speed',0.2,'rampTime',6);
assert(strcmp(manualRef.rampMode,'manual'));
assert(abs(manualRef.rampTimeEffective-6)<1e-12);
% Periodic routes keep moving with no forced end stop.
loopRef = gen_wp_circle('speed',0.25);
assert(isinf(loopRef.duration));
t = loopRef.rampTimeEffective + 5;
first = loopRef.evaluate(t);
second = loopRef.evaluate(t + loopRef.periodS/loopRef.speedPeak);
assert(norm(first-second)<1e-5);
% Verify Faa di Bruno: unit virtual clock == original trajectory.
tauDer=[t;1;0;0;0;0;0];
assert(norm(loopRef.evaluateWarped(t,tauDer)-loopRef.evaluate(t))<1e-7);
% Exact filtered virtual clock: after a command drop, progress slows.
z=ones(6,1); tau=0;
for k=1:1500
    [z,tau,td] = wp_governor_step(z,tau,0.15,0.025,0.40);
    assert(all(isfinite(td)));
end
assert(td(2)>0 && td(2)<0.16);
assert(tau<1500*0.025);
% At command switches the rate and derivatives 0..6 do not jump.
[~,~,before] = wp_governor_step(z,tau,0.15,0,0.4);
[~,~,after] = wp_governor_step(z,tau,1.0,0,0.4);
assert(norm(before-after)<1e-12);
fprintf('PASS: auto ramp, manual override, C6 virtual time and periodic loop.\n');
end
