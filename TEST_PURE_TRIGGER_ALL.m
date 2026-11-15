function r=TEST_PURE_TRIGGER_ALL()
%TEST_PURE_TRIGGER_ALL Stand-alone geometry, contracts, and regression tests.
% Extract ZIP into common_matlab; run: R=TEST_PURE_TRIGGER_ALL();
% No GUI, original HLC edits, mex, or toolboxes needed.
root=fileparts(mfilename('fullpath'));
addpath(fullfile(root,'TRIGGER_LIGHT'));
fprintf('\n=== PURE MIGRATION LIGHT TRIGGER TEST ===\n');
cfg=struct('droneRadius',0.25,'loadRadius',0.16,'cableRadius',0.015, ...
 'safetyMargin',0,'checkGround',true);
far=scene([100;100;5],0.3,false);
known=wp_trigger_prepare_known(far);
samples=linspace(0,4,11);
F=linearForecast(samples,0);
R=wp_trigger_swept_gate(known,cfg,F);
assert(strcmp(R.status,'CLEAR_CONDITIONAL'));fprintf('PASS 1/10: distant obstacle => fast interval clearance\n');
assert(R.supportChecks==0,'Distant case must not call support test');

% A cable intersects a small obstacle ONLY between endpoints t=0 and t=1.
% The two end poses are safe; a sampled-only test would miss this event.
near=scene([0.5;0;2],0.15,false);
N=wp_trigger_prepare_known(near);
F=linearForecast([0,1,4],0);
R=wp_trigger_swept_gate(N,cfg,F);
assert(strcmp(R.status,'POSSIBLE_CONTACT') && R.firstInterval(1)==0); 
fprintf('PASS 2/10: mid-interval cable intrusion caught\n');

% Same drone x trajectory but starts displaced in y; nominal y=0 would miss it.
shift=scene([0.5;0.65;2],0.15,false);
N=wp_trigger_prepare_known(shift);
F=linearForecast([0,1,4],0.65);
R=wp_trigger_swept_gate(N,cfg,F);
assert(strcmp(R.status,'POSSIBLE_CONTACT'));
fprintf('PASS 3/10: off-route/rejoining initial geometry respected\n');

% Source must not invent unbounded estimator or acceleration error intervals.
F=linearForecast(samples,0);
F.rhoLoad(end)=NaN;
R=wp_trigger_swept_gate(known,cfg,F);
assert(strcmp(R.status,'UNKNOWN') && R.shouldRequestReplan);
fprintf('PASS 4/10: missing true-state bound cannot become safe\n');

% Rotated ellipsoid: exact ellipsoid support is used ONLY after sphere screen.
ell=scene([3;2;2],0.2,true);
K=wp_trigger_prepare_known(ell);
F=linearForecast(samples,0);
R=wp_trigger_swept_gate(K,cfg,F);
assert(strcmp(R.status,'CLEAR_CONDITIONAL'));
fprintf('PASS 5/10: rotated ellipsoid far clearance\n');

% Ground in normal z-up convention: load below z=0 is possible collision.
g=scene([100;100;5],0.3,false);
g.ground=struct('kind','halfspace','forbiddenSide','negative', ...
    'normal',[0 0 1],'point',[0 0 0],'safetyMargin',0);
K=wp_trigger_prepare_known(g);
F=linearForecast(samples,0);F.pDrone(3,:)=1.5;F.pLoad(3,:)=-0.5;
R=wp_trigger_swept_gate(K,cfg,F);
assert(strcmp(R.status,'POSSIBLE_CONTACT') && strcmp(R.firstObstacle,'ground'));
fprintf('PASS 6/10: floor collision covers payload and cable\n');

% Certified acceleration bounds inflate swept hull without arbitrary margin.
F=linearForecast(samples,0);
F.accDrone(:)=4;F.accLoad(:)=4;
F.rhoDrone(:)=0;F.rhoLoad(:)=0;
F.pDrone(1,:)=0;F.pLoad(1,:)=0; % static nominal, accelerating truth may move
R=wp_trigger_swept_gate(sceneCache([0.17;0;2.0],0.08,false),cfg,F);
assert(strcmp(R.status,'POSSIBLE_CONTACT'));
fprintf('PASS 7/10: acceleration-bounded sweep cannot false-clear\n');

% Deterministic fixed locations, background, per-part cap, never generated in gate.
K=wp_trigger_prepare_known(scene([1.4;0;2],0.45,false));
provider=@straightRef;
opts=struct('baseStep',0.4,'scanStep',0.05,'maxExtraPerPart',8);
plan=wp_trigger_plan_once(K,cfg,provider,opts);
plan2=wp_trigger_plan_once(K,cfg,provider,opts);
assert(isequal(plan.timeOffsets,plan2.timeOffsets));
assert(all(ismember(plan.baseSites,plan.timeOffsets)));
assert(all(plan.perPartSelectedCount<=8));
assert(plan.numSites>numel(plan.baseSites));
assert(plan.timeOffsets(1)==0 && plan.timeOffsets(end)==4);
assert(all(diff(plan.timeOffsets)>0));
fprintf('PASS 8/10: fixed nonuniform grid and per-part count cap\n');

% Explicitly reject dynamic obstacles without a true swept obstacle bound.
K=wp_trigger_prepare_known(scene([90;2;4],0.2,false));
K.dynamicUnsupported=true;
R=wp_trigger_swept_gate(K,cfg,linearForecast(samples,0));
assert(strcmp(R.status,'UNKNOWN'));
fprintf('PASS 9/10: unsupported moving geometry is never cleared\n');

% Valid initial state but unknown future HLC: bounding without asserted
% true acceleration cannot be 'invented' by the kinematic builder.
state=struct('pDrone',[0;0;3],'vDrone',[1;0;0], ...
    'pLoad',[0;0;1],'vLoad',[1;0;0]);
bounds=struct('aDrone',0,'aLoad',0,'posErrorDrone',0, ...
    'posErrorLoad',0,'velErrorDrone',0,'velErrorLoad',0);
F=wp_trigger_constant_velocity_enclosure(state,samples,bounds);
assert(max(abs(F.pDrone(1,:)-samples))<1e-10);
assert(all(F.rhoDrone==0));
try
    bad=rmfield(bounds,'aDrone');
    wp_trigger_constant_velocity_enclosure(state,samples,bad);
    error('Trigger:ExpectedError','Missing bound was not rejected');
catch ME
    assert(strcmp(ME.identifier,'Trigger:Bounds'));
end
fprintf('PASS 10/10: conditional reachable-tube builder and missing bound\n');
r=struct('status','PASS_10_CONDITIONAL_MATH_TESTS', ...
 'physicalSafetyCertified',false,'hlcErrorBoundDerived',false, ...
 'realtimeTargetMeasured',false,'numTests',10, ...
 'note','Results are conditional on true bounds; MATLAB not run by author.');
fprintf('=== ALL TESTS PASSED (conditional mathematical model only) ===\n');
end
function [d,l]=straightRef(t)
d=[t;0;3];l=[t;0;1];
end
function F=linearForecast(t,y)
t=t(:)';
F=struct('timeOffsets',t, ...
 'pDrone',[t;y*ones(1,numel(t));3*ones(1,numel(t))], ...
 'pLoad',[t;y*ones(1,numel(t));ones(1,numel(t))], ...
 'rhoDrone',zeros(1,numel(t)),'rhoLoad',zeros(1,numel(t)), ...
 'accDrone',zeros(1,numel(t)-1),'accLoad',zeros(1,numel(t)-1), ...
 'boundSource','analytically_linear_test_motion');
end
function K=sceneCache(c,rad,isEll)
K=wp_trigger_prepare_known(scene(c,rad,isEll));
end
function w=scene(c,rad,isEll)
if isEll
    p=struct('kind','ellipsoid','center',c(:)', ...
    'R',rotationZ(0.43)*rotationY(0.32), ...
    'params',struct('axes',[rad;rad*1.2;rad*.6]), ...
    'obstacleName','test_obstacle');
else
    p=struct('kind','sphere','center',c(:)', ...
    'R',eye(3),'params',struct('radius',rad), ...
    'obstacleName','test_obstacle');
end
w=struct('parts',p,'ground',[],'dynamicIndices',[]);
end
function R=rotationZ(a)
R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
end
function R=rotationY(a)
R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];
end
